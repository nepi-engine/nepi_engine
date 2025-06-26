#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import socket
import subprocess
import collections
import os
from datetime import datetime
import threading
import requests
import time
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
 

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import MgrSystemStatus
from nepi_interfaces.msg import Reset, NetworkWifiCredentials
from nepi_interfaces.srv import IPAddrQuery, IPAddrQueryRequest, IPAddrQueryResponse
from nepi_interfaces.srv import BandwidthUsageQueryRequest, BandwidthUsageQuery, BandwidthUsageQueryResponse
from nepi_interfaces.srv import WifiQuery, WifiQueryRequest, WifiQueryResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.connect_mgr_if_system import ConnectMgrSystemServicesIF
from nepi_api.connect_mgr_if_config import ConnectMgrConfigIF


class NetworkMgr:
    """The Network Manager Node of the NEPI core SDK.

    This node controls network IP settings. Users are not able to override the factory configuration
    but they can add and remove additional IPv4 addresses.
    """

    NET_IFACE = "eth0"
    WONDERSHAPER_CALL = "/opt/nepi/ros/share/wondershaper/wondershaper"
    BANDWIDTH_MONITOR_PERIOD_S = 2.0

    FACTORY_STATIC_IP_FILE = "/opt/nepi/config/etc/network/interfaces.d/nepi_static_ip"
    USER_IP_ALIASES_FILE = "/opt/nepi/config/etc/network/interfaces.d/nepi_user_ip_aliases"
    USER_IP_ALIASES_FILE_PREFACE = "# This file includes all user-added IP address aliases. It is sourced by the top-level static IP addr file.\n\n"

    # Following are to support changing rosmaster IP address
    SYS_ENV_FILE = "/opt/nepi/sys_env.bash"
    ROS_MASTER_PORT = 11311
    ROSLAUNCH_FILE = "/opt/nepi/ros/etc/roslaunch.sh"
    REMOTE_ROS_NODE_ENV_LOADER_FILES = ["numurus@num-sb1-zynq:/opt/nepi/ros/etc/env_loader.sh"]

    # Following support WiFi AP setup
    CREATE_AP_CALL = "/opt/nepi/ros/share/create_ap/create_ap"
    DEFAULT_WIFI_AP_SSID = "nepi_device_ap"
    DEFAULT_WIFI_AP_PASSPHRASE = "nepi_device_ap"

    # Following support WiFi Client setup
    ENABLE_DISABLE_WIFI_ADAPTER_PRE = ["ip", "link", "set"]
    ENABLE_WIFI_ADAPTER_POST = ["up"]
    DISABLE_WIFI_ADAPTER_POST = ["down"]
    WPA_SUPPLICANT_CONF_PATH = "/opt/nepi/ros/etc/nepi_env/nepi_wpa_supplicant.conf"
    WPA_START_SUPPLICANT_CMD_PRE = ["wpa_supplicant", "-B" ,"-i"]
    WPA_START_SUPPLICANT_CMD_POST = ["-c", WPA_SUPPLICANT_CONF_PATH]
    WPA_GENERATE_SUPPLICANT_CONF_CMD = "wpa_passphrase"
    STOP_WPA_SUPPLICANT_CMD = ['killall', 'wpa_supplicant']

    # Internet check
    INTERNET_CHECK_CMD = ['nc', '-zw1', 'google.com', '443']
    UPDATER_INTERVAL_S = 1.0
    UPDATER_WIFI_INTERVAL_S = 5.0

    node_if = None
    tx_bw_limit_mbps = -1.0

    current_ip_addrs = []
    report_ip_addrs = []

    connection_updated = True

    clock_skewed = False

    update_ip_table = True

    last_networks = []
 
    dhcp_enable_state = False

    dhcp_enabled = False # initialize to false -- will be updated in set_dhcp_enabl
    tx_bw_limit_mbps = -1
    wifi_ap_enabled = False
    wifi_ap_ssid = ''
    wifi_ap_passphrase = ''
    wifi_client_enabled = False
    wifi_client_ssid = ''
    wifi_client_passphrase = ''

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "network_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        
        ##############################
        ## Wait for NEPI core managers to start
        # Wait for System Manager
        mgr_sys_if = ConnectMgrSystemServicesIF()
        success = mgr_sys_if.wait_for_services()
        if success == False:
            nepi_sdk.signal_shutdown(self.node_name + ": Failed to get System Ready")
        status_dict = mgr_sys_if.get_system_status_dict()
        self.msg_if.pub_warn("Got System Status Dict: " + str(status_dict))
        self.in_container = status_dict['in_container']
        
        
        # Wait for Config Manager
        mgr_cfg_if = ConnectMgrConfigIF()
        success = mgr_cfg_if.wait_for_status()
        if success == False:
            nepi_sdk.signal_shutdown(self.node_name + ": Failed to get Config Ready")
        
        self.tx_byte_cnt_deque = collections.deque(maxlen=2)
        self.rx_byte_cnt_deque = collections.deque(maxlen=2)
        
        # Wifi stuff -- only enabled if WiFi is present
        self.wifi_ap_enabled = False
        self.wifi_ap_ssid = "n/a"
        self.wifi_ap_passphrase = "n/a"
        self.wifi_client_enabled = False
        self.wifi_client_connecting = False
        self.wifi_client_connected = False
        self.wifi_client_ssid = ""
        self.wifi_client_passphrase = ""
        self.available_wifi_networks = []
        self.wifi_scan_thread = None
        self.available_wifi_networks_lock = threading.Lock()
        self.internet_connected = False
        self.internet_connected_lock = threading.Lock()


        self.wifi_iface = None
        self.detectWifiDevice()
        if self.wifi_iface:
            self.msg_if.pub_warn("Detected WiFi (interface queried = " + self.wifi_iface + ")")
            self.wifi_ap_enabled = False
            self.wifi_ap_ssid = self.DEFAULT_WIFI_AP_SSID
            self.wifi_ap_passphrase = self.DEFAULT_WIFI_AP_PASSPHRASE





        ###########################

        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace': self.node_namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'enable_access_point': {
                'namespace': self.node_namespace,
                'factory_val': self.wifi_ap_enabled
            },
            'access_point_ssid': {
                'namespace': self.node_namespace,
                'factory_val': self.wifi_ap_ssid
            },
            'access_point_passphrase': {
                'namespace': self.node_namespace,
                'factory_val': self.wifi_ap_passphrase
            },
            'enable_client': {
                'namespace': self.node_namespace,
                'factory_val': self.wifi_client_enabled
            },
            'client_ssid': {
                'namespace': self.node_namespace,
                'factory_val': self.wifi_client_ssid
            },
            'client_passphrase': {
                'namespace': self.node_namespace,
                'factory_val': self.wifi_client_passphrase
            },
            'dhcp_enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.dhcp_enabled
            },
            'tx_bw_limit_mbps': {
                'namespace': self.node_namespace,
                'factory_val': -1.0
            }
        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'ip_addr_query': {
                'namespace': self.base_namespace,
                'topic': 'ip_addr_query',
                'srv': IPAddrQuery,
                'req': IPAddrQueryRequest(),
                'resp': IPAddrQueryResponse(),
                'callback': self.handle_ip_addr_query
            },
            'bandwidth_usage_query': {
                'namespace': self.base_namespace,
                'topic': 'bandwidth_usage_query',
                'srv': BandwidthUsageQuery,
                'req': BandwidthUsageQueryRequest(),
                'resp': BandwidthUsageQueryResponse(),
                'callback': self.handle_bandwidth_usage_query
            },
            'wifi_query': {
                'namespace': self.base_namespace,
                'topic': 'wifi_query',
                'srv': WifiQuery,
                'req': WifiQueryRequest(),
                'resp': WifiQueryResponse(),
                'callback': self.handle_wifi_query
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'store_params': {
                'namespace': self.base_namespace,
                'topic': 'store_params',
                'msg': String,
                'qsize': 1,
                'latch': False
            },
            'save_system_config': {
                'namespace': self.base_namespace,
                'topic': 'save_system_config',
                'msg': Empty,
                'qsize': 1,
                'latch': False
            }            
        }  

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'add_ip_addr': {
                'namespace': self.base_namespace,
                'topic': 'add_ip_addr',
                'msg': String,
                'qsize': 10,
                'callback': self.add_ip, 
                'callback_args': ()
            },
            'remove_ip_addr': {
                'namespace': self.base_namespace,
                'topic': 'remove_ip_addr',
                'msg': String,
                'qsize': 10,
                'callback': self.remove_ip, 
                'callback_args': ()
            },
            'enable_dhcp': {
                'namespace': self.base_namespace,
                'topic': 'enable_dhcp',
                'msg': Bool,
                'qsize': 10,
                'callback': self.enable_dhcp, 
                'callback_args': ()
            },
            'limit_mbps': {
                'namespace': self.base_namespace,
                'topic': 'set_tx_bw_limit_mbps',
                'msg': Int32,
                'qsize': 10,
                'callback': self.set_upload_bwlimit, 
                'callback_args': ()
            },
            'set_rosmaster': {
                'namespace': self.base_namespace,
                'topic': 'set_rosmaster',
                'msg': String,
                'qsize': 10,
                'callback': self.set_rosmaster, 
                'callback_args': ()
            },
            'enable_wifi': {
                'namespace': self.base_namespace,
                'topic': 'enable_wifi_access_point',
                'msg': Bool,
                'qsize': 10,
                'callback': self.enable_wifi_ap_handler, 
                'callback_args': ()
            },
            'set_wifi_access_point_credentials': {
                'namespace': self.base_namespace,
                'topic': 'set_wifi_access_point_credentials',
                'msg': NetworkWifiCredentials,
                'qsize': 10,
                'callback': self.set_wifi_ap_credentials_handler, 
                'callback_args': ()
            },
            'wifi_client': {
                'namespace': self.base_namespace,
                'topic': 'enable_wifi_client',
                'msg': Bool,
                'qsize': 10,
                'callback': self.enable_wifi_client_handler, 
                'callback_args': ()
            },
            'set_wifi_client_credentials': {
                'namespace': self.base_namespace,
                'topic': 'set_wifi_client_credentials',
                'msg': NetworkWifiCredentials,
                'qsize': 10,
                'callback': self.set_wifi_client_credentials_handler, 
                'callback_args': ()
            },
            'refresh_wifi_networks': {
                'namespace': self.base_namespace,
                'topic': 'refresh_available_wifi_networks',
                'msg': Empty,
                'qsize': 10,
                'callback': self.refresh_available_networks_handler, 
                'callback_args': ()
            }
        }



        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        msg_if = self.msg_if
        )

        #ready = self.node_if.wait_for_ready()
        nepi_sdk.wait()


        self.initCb(do_updates = True)
        ###########################
        # Complete Initialization

        nepi_sdk.start_timer_process(self.BANDWIDTH_MONITOR_PERIOD_S, self.monitor_bandwidth_usage)

        # Long duration internet check -- do oneshot and reschedule from within the callback
        nepi_sdk.start_timer_process(self.UPDATER_INTERVAL_S, self.updaterCb, oneshot = True)
        nepi_sdk.start_timer_process(self.UPDATER_INTERVAL_S, self.internetCheckCb, oneshot = True)

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        #########################################################

        self.run()


    #######################
    # Wait for System and Config Statuses Callbacks
    def systemStatusCb(self,msg):
        self.sys_status = msg

    def configStatusCb(self,msg):
        self.cfg_status = True
    

    #######################
    ### Mgr Config Functions

    def run(self):
        nepi_sdk.spin()

    def cleanup(self):
        self.process.stop()

    def get_primary_ip_addr(self):
        key = "inet static"
        with open(self.FACTORY_STATIC_IP_FILE, "r") as f:
            lines = f.readlines()
            for i,line in enumerate(lines):
                if key in line:
                    primary_ip = lines[i+1].split()[1]
                    return primary_ip
        return "Unknown Primary IP"

    def get_current_ip_addrs(self):
        primary_ip = self.get_primary_ip_addr()
        ip_addrs = [primary_ip]
        addr_list_output = subprocess.check_output(['ip','addr','list','dev',self.NET_IFACE], text=True)
        tokens = addr_list_output.split()
        for i, t in enumerate(tokens):
            if (t == 'inet'):
                # Ensure that aliases go at the back of the list and primary remains at the front -- we rely on that ordering throughout this module
                if (tokens[i+1] != primary_ip):
                    ip_addrs.append(tokens[i+1]) # Back of the line
        return ip_addrs

    def validate_cidr_ip(self, addr):
        # First, validate the input
        tokens = addr.split('/')
        new_ip = tokens[0]
        new_ip_bits = 0
        try:
            new_ip_bits = socket.inet_aton(new_ip)
        except Exception as e:
            self.msg_if.pub_warn("Rejecting invalid IP address " + str(new_ip) + " " + str(e))
            return False
        if (len(tokens) != 2):
            self.msg_if.pub_warn("Rejecting invalid address must be in CIDR notation (x.x.x.x/y). Got " + str(addr))
            return False
        cidr_netmask = (int)(tokens[1])
        if cidr_netmask < 1 or cidr_netmask > 32:
            self.msg_if.pub_warn("Rejecting invalid CIDR netmask (got " + str(addr))
            return False

        # Finally, verify that this isn't the "fixed" address on the device. Don't let anyone sneak past the same
        # address in a different numerical format - compare as a 32-bit val
        cur_ip_addrs = copy.deepcopy(self.current_ip_addrs)
        if len(cur_ip_addrs) > 0:
            fixed_ip_addr = cur_ip_addrs[0].split('/')[0]
            fixed_ip_bits = 0
            try:
                fixed_ip_bits = socket.inet_aton(fixed_ip_addr)
            except Exception as e:
                self.msg_if.pub_warn("Cannot validate IP address becaused fixed IP appears invalid "  + str(fixed_ip_addr) + " " + str(e))
                return False
            if (new_ip_bits == fixed_ip_bits):
                self.msg_if.pub_warn("IP address invalid because it matches fixed primary IP")
                return False

            return True
        else:
            return False

    def add_ip_impl(self, new_addr):
        try:
            subprocess.check_call(['ip','addr','add',new_addr,'dev',self.NET_IFACE])
            save_config(self)
        except Exception as e:
            self.msg_if.pub_warn("Failed to set IP address to " + str(new_addr) + " " + str(e))


    def add_ip(self, new_addr_msg):
        new_addr = new_addr_msg.data
        if new_addr not in self.current_ip_addrs:
            self.report_ip_addrs.append(new_addr)
            self.msg_if.pub_warn("Recieved Add IP address: " + str(new_addr_msg.data))
            if True == self.validate_cidr_ip(new_addr_msg.data):
                self.add_ip_impl(new_addr_msg.data)
                self.msg_if.pub_warn("Added IP address: " + str(new_addr_msg.data))
                save_config(self)
            else:
                self.msg_if.pub_warn("Unable to add invalid/ineligible IP address: " + str(new_addr_msg.data))
        else:
            self.msg_if.pub_warn("IP address allready in system: " + str(new_addr_msg.data))
        

    def remove_ip_impl(self, old_addr):
        try:
            subprocess.check_call(['ip','addr','del',old_addr,'dev',self.NET_IFACE])
            save_config(self)
        except Exception as e:
            self.msg_if.pub_warn("Failed to remove IP address " + str(old_addr) + " " + str(e))

    def remove_ip(self, old_addr_msg):
        self.msg_if.pub_warn("Recieved Remove IP address: " + str(old_addr_msg.data))
        old_addr = old_addr_msg.data
        if old_addr in self.current_ip_addrs:
            self.report_ip_addrs.remove(old_addr)
            if True == self.validate_cidr_ip(old_addr_msg.data):
                self.remove_ip_impl(old_addr_msg.data)
                self.msg_if.pub_warn("Removed IP address: " + str(old_addr_msg.data))
                save_config(self)
            else:
                self.msg_if.pub_warn("Unable to remove invalid/ineligible IP address: " + str(old_addr_msg.data))
        else:
            self.msg_if.pub_warn("IP address not found in system: " + str(old_addr_msg.data))

 
    def enable_dhcp_impl(self, enabled):

        if self.in_container == False:
            if self.clock_skewed == False:
                self.update_ip_table = False
                self.msg_if.pub_warn("Got DHCP enable request: " + str(enabled))
                if enabled is True:
                    if self.dhcp_enable_state is False:
                        self.dhcp_enabled = True
                        self.msg_if.pub_warn("Enabling DHCP Client")
                        with self.internet_connected_lock:
                            connected = self.internet_connected
                        self.msg_if.pub_warn("Enabling DHCP with connection: " + str(connected))
                        try:
                            if connected == False:
                                self.msg_if.pub_warn("Calling dhclient -nw subprocess")
                                subprocess.check_call(['dhclient', '-nw', self.NET_IFACE])
                            self.dhcp_enabled = True
                            self.msg_if.pub_warn("DHCP enabled")
                            save_config(self)
                        except Exception as e:
                            self.dhcp_enabled = False
                            self.msg_if.pub_warn("Unable to enable DHCP: " + str(e))
                    else:
                        self.msg_if.pub_warn("DHCP already enabled")
                else:
                    if self.dhcp_enable_state is True:
                        self.dhcp_enabled = False
                        self.msg_if.pub_warn("Disabling DHCP Client")
                        with self.internet_connected_lock:
                            connected = self.internet_connected
                        self.msg_if.pub_warn("Enabling DHCP with connection: " + str(connected))
                        restart_network = False
                        try:
                            # The dhclient -r call below causes all IP addresses on the interface to be dropped, so
                            # we reinitialize them here... this will not work for IP addresses that were
                            # added in this session but not saved to config (i.e., not known to param server)                        
                            if connected == True:
                                self.msg_if.pub_warn("Calling dhclient -r subprocess")
                                subprocess.check_call(['dhclient', '-r', self.NET_IFACE])
                                restart_network = True
                                nepi_sdk.wait()
                            self.dhcp_enable_state = False
                            self.msg_if.pub_warn("DHCP disabled")
                            save_config(self)
                        except Exception as e:
                            self.dhcp_enabled = True
                            self.msg_if.pub_warn("Unable to disable DHCP: " + str(e))
                        try:
                            if restart_network == True:
                                self.msg_if.pub_warn("Restarting network IFACE: " + self.NET_IFACE)
                                # Restart the interface -- this picks the original static IP back up and sources the user IP alias file
                                subprocess.call(['ifdown', self.NET_IFACE])
                                nepi_sdk.wait()
                                subprocess.call(['ifup', self.NET_IFACE])
                                self.msg_if.pub_warn("Network IFACE restarted: " + self.NET_IFACE)
                        except Exception as e:
                            self.msg_if.pub_warn("Unable to reset NET_IFACE: " + self.NET_IFACE + " " + str(e))
                    else:
                        self.msg_if.pub_warn("DHCP already disabled")
                    #nepi_sdk.sleep(5)
                self.update_ip_table = True

            else:
                self.msg_if.pub_warn("Ignoring DHCP change request due to clock skew. Sync system clock")
        else:
            self.msg_if.pub_warn("Ignoring DHCP change request from container. Update in host system")
        

    def enable_dhcp(self, enabled_msg):
        self.msg_if.pub_warn("Got DHCP enable request msg: " + str(enabled_msg))
        self.enable_dhcp_impl(enabled_msg.data)


    def initCb(self, do_updates = False):
        if self.node_if is not None:
            self.dhcp_enabled = self.node_if.get_param('dhcp_enabled')
            self.tx_bw_limit_mbps = self.node_if.get_param('tx_bw_limit_mbps')
            self.wifi_ap_enabled = self.node_if.get_param('enable_access_point')
            self.wifi_ap_ssid = self.node_if.get_param('access_point_ssid')
            self.wifi_ap_passphrase = self.node_if.get_param('access_point_passphrase')
            self.wifi_client_enabled = self.node_if.get_param('enable_client')
            self.wifi_client_ssid = self.node_if.get_param("client_ssid")
            self.wifi_client_passphrase = self.node_if.get_param("client_passphrase")

        if do_updates == True:
            self.enable_dhcp_impl(self.dhcp_enabled)
            self.set_upload_bw_limit()
            self.set_wifi_ap()
            self.set_wifi_client()
            
        success = self.save_config()

    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            self.node_if.reset_params()
        if do_updates == True:
            pass
        self.initCb()
        


    def factoryResetCb(self,do_updates = True):
        if self.node_if is not None:
            self.node_if.factory_reset_params()
        if do_updates == True:
            pass
        self.initCb()

        self.msg_if.pub_warn("Reseting Factory Config")
        with open(self.USER_IP_ALIASES_FILE, "w") as f:
            f.write(self.USER_IP_ALIASES_FILE_PREFACE)

        # Set the rosmaster back to localhost
        self.set_rosmaster_impl("localhost")
        self.msg_if.pub_warn("Factory reset complete -- must reboot device for IP and ROS_MASTER_URI changes to take effect")


    def save_network_config(self):
        success = False
        # First update user static IP file
        # Note that this is outside the scope of ROS param server because we need these
        # aliases to come up even before ROS (hence this node) comes up in case the remoted ROSMASTER
        # is on a subnet only reachable via one of these aliases
        current_ips = copy.deepcopy(self.report_ip_addrs)
        self.msg_if.pub_warn("Writing Updated Config to file: " + str(current_ips))
        with open(self.USER_IP_ALIASES_FILE, "w") as f:
            f.write(self.USER_IP_ALIASES_FILE_PREFACE)
            if (len(current_ips) > 1):
                for i,ip_cidr in enumerate(current_ips[1:]): # Skip the first one -- that is the factory default
                    alias_name = self.NET_IFACE + ":" + str(i+1)
                    f.write("auto " + alias_name + "\n")
                    f.write("iface " + alias_name + " inet static\n")
                    f.write("    address " + ip_cidr + "\n\n")
        success = True
        time.sleep(1) # Time for network changes to update
        self.msg_if.pub_warn("Saving system config with IP aliases")
        if self.node_if is not None:
            self.node_if.publish_pub('save_system_config',Empty())
        return success


    def save_config(self):
        success = True
        if self.node_if is not None:
            self.node_if.save_config()
        self.save_network_config()
        return success


    def set_upload_bwlimit(self, msg):
        if msg.data >= 0 and msg.data < 1:
            self.msg_if.pub_warn('Cannot set bandwidth limit below 1Mbps')
            return

        # First, update param server
        self.tx_bw_limit_mbps = msg.data
        self.set_upload_bw_limit()
        if self.node_if is not None:
            self.node_if.set_param('tx_bw_limit_mbps', msg.data)
            success = self.save_config()


    def set_rosmaster(self, msg):
        new_master_ip = msg.data
        self.set_rosmaster_impl(new_master_ip)
        success = self.save_config()

    def set_rosmaster_impl(self, master_ip):
        auto_comment = " # Modified by network_mgr " + str(datetime.now()) + "\n"

        # First, determine if the master is local, either as localhost or one of the configured IP addrs
        master_is_local = True if (master_ip == "localhost") else False
        if master_is_local is False:
            local_ips = copy.deepcopy(self.report_ip_addrs)
            for ip_cidr in local_ips:
                ip = ip_cidr.split('/')[0]
                if master_ip == ip:
                    master_is_local = True
                    break

        if master_is_local is True:
            master_ip = "localhost" # Force 'localhost' whenever the "new" master is a local IP in case that local IP (alias) is later removed
            master_ip_for_remote_hosts = self.get_primary_ip_addr().split('/')[0]
        else:
            master_ip_for_remote_hosts = master_ip
            # Now ensure we can contact the new rosmaster -- if not, bail out
            ret_code = subprocess.call(['nc', '-zvw5', master_ip,  str(self.ROS_MASTER_PORT)])
            if (ret_code != 0):
                self.msg_if.pub_warn("Failed to detect a remote rosmaster at " + master_ip + ":" + str(self.ROS_MASTER_PORT) + "... refusing to update ROS_MASTER_URI")
                return

        # Edit the sys_env file appropriately
        rosmaster_line_prefix = "export ROS_MASTER_URI="
        new_rosmaster_line = rosmaster_line_prefix + "http://" + master_ip + ":" + str(self.ROS_MASTER_PORT) + auto_comment
        sys_env_output_lines = []
        with open(self.SYS_ENV_FILE, "r") as f_in:
            for line in f_in:
                if rosmaster_line_prefix in line:
                    line = new_rosmaster_line
                sys_env_output_lines.append(line)
        with open(self.SYS_ENV_FILE, "w") as f_out:
            f_out.writelines(sys_env_output_lines)

        # And the roslaunch file (add --wait for remote ros master)
        roslaunch_line_prefix = "roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}"
        new_roslaunch_line = roslaunch_line_prefix
        if master_is_local is False:
            new_roslaunch_line += " --wait"
        new_roslaunch_line += auto_comment
        roslaunch_output_lines = []
        with open(self.ROSLAUNCH_FILE, "r") as f_in:
            for line in f_in:
                if roslaunch_line_prefix in line:
                    line = new_roslaunch_line
                roslaunch_output_lines.append(line)
        with open(self.ROSLAUNCH_FILE, "w") as f_out:
            f_out.writelines(roslaunch_output_lines)

        # And the env_loader files for remote machines
        new_rosmaster_line = rosmaster_line_prefix + "http://" + master_ip_for_remote_hosts + ":" + str(self.ROS_MASTER_PORT) + auto_comment
        tmp_env_loader_file = "./env_loader_file.tmp"
        for remote_env_loader_file in self.REMOTE_ROS_NODE_ENV_LOADER_FILES:
            env_loader_lines = []
            ret_code = subprocess.call(['scp', remote_env_loader_file, tmp_env_loader_file])
            if (ret_code != 0):
                self.msg_if.pub_warn("Failed to get copy of remote file " + remote_env_loader_file + "... not updating ROS_MASTER_URI for that remote host")
                continue
            with open(tmp_env_loader_file, "r") as f_in:
                for line in f_in:
                    if rosmaster_line_prefix in line:
                        line = new_rosmaster_line
                    env_loader_lines.append(line)
            with open(tmp_env_loader_file, "w") as f_out:
                f_out.writelines(env_loader_lines)
            ret_code = subprocess.call(['scp', tmp_env_loader_file, remote_env_loader_file])
            if (ret_code != 0):
                self.msg_if.pub_warn("Failed to update remote file " + remote_env_loader_file)
            os.remove(tmp_env_loader_file)

        self.msg_if.pub_warn("Updated ROS_MASTER_URI to " + master_ip + "... requires reboot to complete the switch")
        success = self.save_config()


    def set_upload_bw_limit(self):
        # Always clear the current settings
        try:
            subprocess.call([self.WONDERSHAPER_CALL, '-a', self.NET_IFACE, '-c'])
        except Exception as e:
            self.msg_if.pub_warn("Unable to clear current bandwidth limits: " + str(e))
            return

        if self.tx_bw_limit_mbps < 0: #Sentinel values to clear limits
            self.msg_if.pub_info("Cleared bandwidth limits")
            return

        # Now acquire the param from param server and update
        bw_limit_kbps = self.tx_bw_limit_mbps* 1000
        try:
            subprocess.call([self.WONDERSHAPER_CALL, '-a', self.NET_IFACE, '-u', str(bw_limit_kbps)])
            self.msg_if.pub_info("Updated TX bandwidth limit to " + str(self.tx_bw_limit_mbps) + " Mbps")
            #self.tx_byte_cnt_deque.clear()
        except Exception as e:
            self.msg_if.pub_warn("Unable to set upload bandwidth limit: " + str(e))
        success = self.save_config()

    def enable_wifi_ap_handler(self, enabled_msg):
        self.msg_if.pub_warn("Recieved enable wifi access point msg: " + str(enabled_msg))
        if self.wifi_iface is None:
            self.msg_if.pub_warn("Cannot enable WiFi access point - system has no WiFi adapter")
            return
        self.wifi_ap_enabled = enabled_msg.data
        self.set_wifi_ap()
        if self.node_if is not None:
            self.node_if.set_param("enable_access_point", enabled_msg.data)
            success = self.save_config()

    def set_wifi_ap_credentials_handler(self, msg):
        self.msg_if.pub_warn("Recieved set wifi access point msg: " + str(msg))
        self.wifi_ap_ssid = msg.ssid
        self.wifi_ap_passphrase = msg.passphrase
        self.set_wifi_ap()
        if self.node_if is not None:
            self.node_if.set_param("access_point_ssid", msg.ssid)
            self.node_if.set_param("access_point_passphrase", msg.passphrase)
            success = self.save_config()



    def set_wifi_ap(self):
        if self.wifi_ap_enabled is True:
            if self.wifi_iface is None:
                self.msg_if.pub_warn("Cannot enable WiFi access point - system has no WiFi adapter")
                return
            try:
                # Kill any current access point -- no problem if one isn't already running; just returns immediately
                subprocess.call([self.CREATE_AP_CALL, '--stop', self.wifi_iface])
                nepi_sdk.wait()
                self.msg_if.pub_warn("Starting WiFi access point on ssid: " + str(self.wifi_ap_ssid))
                # Use the create_ap command line
                subprocess.check_call([self.CREATE_AP_CALL, '-n', '--redirect-to-localhost', '--isolate-clients', '--daemon',
                                       self.wifi_iface, self.wifi_ap_ssid, self.wifi_ap_passphrase])
                self.msg_if.pub_info("Started WiFi access point: " + str(self.wifi_ap_ssid))
            except Exception as e:
                self.msg_if.pub_warn("Unable to start wifi access point with " + str(e))
        else:
            try:
                subprocess.check_call([self.CREATE_AP_CALL, '--stop', self.wifi_iface])
            except Exception as e:
                self.msg_if.pub_warn("Unable to terminate wifi access point: " + str(e))

    def enable_wifi_client_handler(self, enabled_msg):
        if self.wifi_iface is None:
            self.msg_if.pub_warn("Cannot enable WiFi client - system has no WiFi adapter")
            return
        if (enabled_msg.data):
            self.msg_if.pub_info("Enabling WiFi client")
        else:
            self.msg_if.pub_info("Disabling WiFi client")

        self.wifi_client_enabled = enabled_msg.data
        if self.node_if is not None:
            self.node_if.set_param("enable_client", enabled_msg.data)
            success = self.save_config()
        self.set_wifi_client()
        

    def set_wifi_client_credentials_handler(self, msg):
        self.msg_if.pub_info("Updating WiFi client credentials (SSID: " + msg.ssid + ", Passphrase: " + msg.passphrase + ")")
        self.wifi_client_ssid = msg.ssid
        self.wifi_client_passphrase = msg.passphrase
        if msg.ssid != "":
            self.wifi_client_connecting = True
        self.set_wifi_client()
        if self.node_if is not None:
            self.node_if.set_param("client_ssid", msg.ssid)
            self.node_if.set_param("client_passphrase", msg.passphrase)
            success = self.save_config()
       

    def auto_retry_wifi_client_connect(self, event):
        self.msg_if.pub_info("Automatically retrying wifi client setup")
        if self.wifi_client_connected != True:
            self.set_wifi_client()


    def set_wifi_client(self):
        if self.wifi_client_enabled is True:
            if self.wifi_iface is None:
                self.msg_if.pub_warn("Cannot enable WiFi client - system has no WiFi adapter")
                return
            if (self.wifi_client_ssid != "None" and self.wifi_client_ssid != ""):
                if (self.wifi_client_ssid in self.available_wifi_networks):
                    try:
                        # First, enable the hardware (might be unnecessary, but no harm)
                        link_up_cmd = self.ENABLE_DISABLE_WIFI_ADAPTER_PRE + [self.wifi_iface] + self.ENABLE_WIFI_ADAPTER_POST
                        subprocess.check_call(link_up_cmd)
                        
                        try:
                            with open(self.WPA_SUPPLICANT_CONF_PATH, 'w') as f:

                                if (self.wifi_client_passphrase != "None"):
                                    wpa_generate_supplicant_conf_cmd = [self.WPA_GENERATE_SUPPLICANT_CONF_CMD, self.wifi_client_ssid,
                                                                        self.wifi_client_passphrase]
                                    subprocess.check_call(wpa_generate_supplicant_conf_cmd, stdout=f)
                                else: # Open network
                                    # wpa_passphrase can't help us here, so generate the conf. manually
                                    f.write("network={\n\tssid=\"" + self.wifi_client_ssid + "\"\n\tkey_mgmt=NONE\n}")

                            start_supplicant_cmd = self.WPA_START_SUPPLICANT_CMD_PRE + [self.wifi_iface] + self.WPA_START_SUPPLICANT_CMD_POST
                            #self.msg_if.pub_warn("DEBUG: Using command " + str(start_supplicant_cmd))
                        
                            subprocess.call(self.STOP_WPA_SUPPLICANT_CMD)
                            self.wifi_client_connected = False
                            nepi_sdk.wait()
                            subprocess.check_call(start_supplicant_cmd)
                            # Wait a few seconds for it to connect
                            connected_ssid = None
                            for i in range(10):
                                connected_ssid = self.get_wifi_client_connected_ssid()
                                if connected_ssid is not None:
                                    break
                                nepi_sdk.sleep(1)
                            if connected_ssid is None:
                                raise Exception("Wifi client failed to connect")
                            self.wifi_client_connecting = False    
                            subprocess.check_call(['dhclient', '-nw', self.wifi_iface])
                            self.msg_if.pub_info("Connected to WiFi network " + connected_ssid)
                            self.wifi_client_connected = True
                        except Exception as e:
                            self.msg_if.pub_warn("Failed to start WiFi client (SSID=" + self.wifi_client_ssid + " Passphrase=" + \
                                            self.wifi_client_passphrase + "): " + str(e))
                            # Auto retry in 3 seconds
                            self.msg_if.pub_info("Automatically retrying Wifi connect in 3 seconds")
                            if not nepi_sdk.is_shutdown():
                                self.retry_wifi_timer = nepi_sdk.start_timer_process(3, self.auto_retry_wifi_client_connect, oneshot=True)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to check WiFi client as configured: " + str(e))
                        return
                            
            elif self.wifi_client_connected == True:
                # Stop the supplicant
                subprocess.call(self.STOP_WPA_SUPPLICANT_CMD)
                nepi_sdk.wait()
                # Bring down the interface
                link_down_cmd = self.ENABLE_DISABLE_WIFI_ADAPTER_PRE + [self.wifi_iface] + self.DISABLE_WIFI_ADAPTER_POST
                subprocess.call(link_down_cmd)

                if self.wifi_scan_thread is not None:
                    self.wifi_scan_thread.join(1)
                    self.wifi_scan_thread = None
                try:
                    wifi_client = self.get_wifi_client_connected_ssid()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get info for WiFi network: " + str(e))
                    wifi_client = None
                    self.wifi_client_connected = False
                if ( wifi_client is not None):
                    self.msg_if.pub_warn("Failed to disconnect from WiFi network")
                else:
                    with self.available_wifi_networks_lock:
                        self.available_wifi_networks = []
            else:
                self.msg_if.pub_info("Wifi client ready -- need SSID and passphrase to connect")
                self.retry_wifi_timer = None
                
                # Run a refresh
                self.refresh_available_networks()

    def get_wifi_client_connected_ssid(self):
        if self.wifi_iface is None:
            self.wifi_client_connected = False
            return None

        check_connection_cmd = ['iw', self.wifi_iface, 'link']
        connection_status = subprocess.check_output(check_connection_cmd, text=True)
        if connection_status.startswith('Connected'):
           self.wifi_client_connected = True
           self.wifi_client_connecting = False 
           for line in connection_status.splitlines():
               if line.strip().startswith('SSID'):
                   return line.strip().split()[1]
        
        self.wifi_client_connected = False
        return None

    def monitor_bandwidth_usage(self, event):
        with open('/sys/class/net/' + self.NET_IFACE + '/statistics/tx_bytes', 'r') as f:
            tx_bytes = int(f.read())
            self.tx_byte_cnt_deque.append(tx_bytes)
        with open('/sys/class/net/' + self.NET_IFACE + '/statistics/rx_bytes', 'r') as f:
            rx_bytes = int(f.read())
            self.rx_byte_cnt_deque.append(rx_bytes)

    def updaterCb(self, event):
        #self.msg_if.pub_warn("Entering updater process")
        prev_ip_addrs = copy.deepcopy(self.current_ip_addrs)
        current_ip_addrs = self.get_current_ip_addrs()
        self.report_ip_addrs = copy.deepcopy(current_ip_addrs)

        #self.msg_if.pub_warn("***")
        #self.msg_if.pub_warn("Prev IP Addrs: " + str(prev_ip_addrs))
        #self.msg_if.pub_warn("New IP Addrs: " + str(current_ip_addrs))


        if self.update_ip_table == True:
            if current_ip_addrs != prev_ip_addrs:
                self.current_ip_addrs = current_ip_addrs
                self.msg_if.pub_warn("Calling save network config")
                self.save_network_config()
        else:
            self.msg_if.pub_warn("Skipping IP Update until dhcp change complete")

        #self.msg_if.pub_warn("Exiting updater process")
        nepi_sdk.start_timer_process(self.UPDATER_INTERVAL_S, self.updaterCb, oneshot = True)


    def internetCheckCb(self, event):
        #self.msg_if.pub_warn("Entering internet check process")
        with self.internet_connected_lock:
            prev_connected = self.internet_connected
        connected = False
        try:
            #subprocess.check_call(self.INTERNET_CHECK_CMD, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            response = requests.get("https://www.github.com", timeout=1)
            connected = True
            if prev_connected == False:
                self.msg_if.pub_warn("Detected new internet connection")
                self.msg_if.pub_warn("Internet check response: " + str(response) )
                self.connection_updated = True
            
        except Exception as e:
            connected = False 
            if prev_connected == True:
                self.msg_if.pub_warn("Lost internet connection")
                self.connection_updated = True

        with self.internet_connected_lock:
            self.internet_connected = connected

 
        cur_year = datetime.now().year
        self.clock_skewed = cur_year < 2024
            
        #self.msg_if.pub_warn("Exiting internet check process")
        nepi_sdk.start_timer_process(self.UPDATER_INTERVAL_S, self.internetCheckCb, oneshot = True)

    def handle_ip_addr_query(self, req):
        ips = self.report_ip_addrs
        if self.connection_updated == True:
            self.connection_updated = False
            if self.internet_connected == True:
                self.dhcp_enabled = True
        return {'ip_addrs':ips, 'dhcp_enabled': self.dhcp_enabled}

    def handle_bandwidth_usage_query(self, req):
        tx_rate_mbps = 0
        if len(self.tx_byte_cnt_deque) > 1:
            tx_rate_mbps =  8 * (self.tx_byte_cnt_deque[1] - self.tx_byte_cnt_deque[0]) / (self.BANDWIDTH_MONITOR_PERIOD_S * 1000000)

        rx_rate_mbps = 0
        if len(self.rx_byte_cnt_deque) > 1:
            rx_rate_mbps = 8 * (self.rx_byte_cnt_deque[1] - self.rx_byte_cnt_deque[0]) / (self.BANDWIDTH_MONITOR_PERIOD_S * 1000000)



        return {'tx_rate_mbps':tx_rate_mbps, 'rx_rate_mbps':rx_rate_mbps, 'tx_limit_mbps': self.tx_bw_limit_mbps}

    def handle_wifi_query(self, req):
        with self.available_wifi_networks_lock:
            available_networks = list(self.available_wifi_networks)

        with self.internet_connected_lock:
            internet_connected = self.internet_connected
        
        return {'has_wifi': (self.wifi_iface is not None), 
                'wifi_ap_enabled': self.wifi_ap_enabled,
                'wifi_ap_ssid': self.wifi_ap_ssid, 
                'wifi_ap_passphrase': self.wifi_ap_passphrase,
                'wifi_client_enabled': self.wifi_client_enabled,
                'wifi_client_connecting': self.wifi_client_connecting,
                'wifi_client_connected': self.wifi_client_connected,
                'wifi_client_ssid': self.wifi_client_ssid,
                'wifi_client_passphrase': self.wifi_client_passphrase,
                'available_networks': available_networks,
                'clock_skewed': self.clock_skewed,
                'internet_connected': internet_connected}

    def refresh_available_networks_handler(self, msg):
        self.refresh_available_networks()


    def refresh_available_networks(self):
        #if self.wifi_scan_thread is not None:
        #    self.msg_if.pub_info("Not refreshing available wifi networks because a refresh is already in progress")
        #    return

        # Clear the list, let the scan thread update it later
        with self.available_wifi_networks_lock:
            self.available_wifi_networks = []

        self.wifi_scan_thread = threading.Thread(target=self.update_available_wifi_networks)
        self.wifi_scan_thread.daemon = True
        self.wifi_scan_thread.start()

    
    def update_available_wifi_networks(self):
        #self.msg_if.pub_warn("Debugging: Scanning for available WiFi networks")
        if self.wifi_client_enabled == True and self.wifi_iface is not None:
            available_networks = []
            network_scan_cmd = ['iw', self.wifi_iface, 'scan']
            scan_result = ""
            try:
                scan_result = subprocess.check_output(network_scan_cmd, text=True)
            except Exception as e:
                #self.msg_if.pub_info("Failed to scan for available WiFi networks: " + str(e))
                pass
            for scan_line in scan_result.splitlines():
                if "SSID:" in scan_line:
                    network = scan_line.split(':')[1].strip()
                    # TODO: Need more checks to ensure this is a connectable network?
                    if network and (network not in available_networks):
                        available_networks.append(network)
            with self.available_wifi_networks_lock:
                self.available_wifi_networks = list(available_networks)

    def detectWifiDevice(self):
        self.wifi_iface = None # Flag non-existence and then correct below as necessary

        wifi_check_output = subprocess.check_output(['iw','dev'], text=True)
        for line in wifi_check_output.splitlines():
            # For now, just check for the existence of a single interface
            if line.strip().startswith('Interface'):
               if 'p2p' in line: # Peer-to-peer/WiFi Direct: NEPI does not support
                   self.msg_if.pub_warn("Ignoring P2P WiFi Direct interface " + line.strip().split()[1])
                   continue
               self.wifi_iface = line.strip().split()[1]
               return 


if __name__ == "__main__":
    NetworkMgr()
