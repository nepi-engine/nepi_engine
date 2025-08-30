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
import ipaddress

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
 

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import MgrNetworkStatus
from nepi_interfaces.msg import MgrSystemStatus
from nepi_interfaces.msg import Reset, NetworkWifiCredentials
from nepi_interfaces.srv import IPAddrQuery, IPAddrQueryRequest, IPAddrQueryResponse
from nepi_interfaces.srv import BandwidthUsageQueryRequest, BandwidthUsageQuery, BandwidthUsageQueryResponse
from nepi_interfaces.srv import WifiQuery, WifiQueryRequest, WifiQueryResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF


class NetworkMgr:
    """The Network Manager Node of the NEPI core SDK.

    This node controls network IP settings. Users are not able to override the factory configuration
    but they can add and remove additional IPv4 addresses.
    """

    NET_IFACE = "eth0"
    WONDERSHAPER_CALL = "/opt/nepi/nepi_engine/share/wondershaper/wondershaper"
    BANDWIDTH_MONITOR_PERIOD_S = 2.0

    FACTORY_STATIC_IP_FILE = "/opt/nepi/etc/network/interfaces.d/nepi_static_ip"
    USER_IP_ALIASES_FILE = "/opt/nepi/etc/network/interfaces.d/nepi_user_ip_aliases"
    USER_IP_ALIASES_FILE_PREFACE = "# This file includes all user-added IP address aliases. It is sourced by the top-level static IP addr file.\n\n"

    # Following are to support changing rosmaster IP address
    SYS_ENV_FILE = "/opt/nepi/sys_env.bash"
    ROS_MASTER_PORT = 11311
    ROSLAUNCH_FILE = "/opt/nepi/nepi_engine/etc/roslaunch.sh"
    REMOTE_ROS_NODE_ENV_LOADER_FILES = ["numurus@num-sb1-zynq:/opt/nepi/nepi_engine/etc/env_loader.sh"]

    # Following support WiFi AP setup
    CREATE_AP_CALL = "/opt/nepi/nepi_engine/share/create_ap/create_ap"
    DEFAULT_WIFI_AP_SSID = "nepi_device_ap"
    DEFAULT_WIFI_AP_PASSPHRASE = "nepi_device_ap"

    # Following support WiFi Client setup
    ENABLE_DISABLE_WIFI_ADAPTER_PRE = ["ip", "link", "set"]
    ENABLE_WIFI_ADAPTER_POST = ["up"]
    DISABLE_WIFI_ADAPTER_POST = ["down"]
    WPA_SUPPLICANT_CONF_PATH = "/opt/nepi/etc/wpa_supplicant/wpa_supplicant.conf"
    WPA_START_SUPPLICANT_CMD_PRE = ["wpa_supplicant", "-B" ,"-i"]
    WPA_START_SUPPLICANT_CMD_POST = ["-c", WPA_SUPPLICANT_CONF_PATH]
    WPA_GENERATE_SUPPLICANT_CONF_CMD = "wpa_passphrase"
    STOP_WPA_SUPPLICANT_CMD = ['killall', 'wpa_supplicant']
    

    # Internet check
    INTERNET_CHECK_CMD = ['nc', '-zw1', 'google.com', '443']
    UPDATER_INTERVAL_S = 1.0
    UPDATER_WIFI_INTERVAL_S = 5.0

    node_if = None

    status_msg = MgrNetworkStatus()
    tx_bw_limit_mbps = -1.0


    
    found_ip_addrs = []
    last_ip_addrs = None

    clock_skewed = False

    last_networks = []
 

    primary_ip_addr = '192.168.179.103/24'
    
    managed_ip_addrs = []
    last_ip_addrs = None
    dhcp_enabled = False # initialize to false -- will be updated in set_dhcp_enabl
    dhcp_connecting = False
    dhcp_enable_state = False
    dhcp_ip_addr = ''

    tx_bw_limit_mbps = -1

    wifi_iface = None


    wifi_adapter_ready = False
    wifi_adapter_low_power_mode = True
    wifi_client_enabled = False
    wifi_client_ssid = 'None'
    wifi_client_passphrase = ''
    
    wifi_client_connecting = False
    wifi_client_connected = False
    wifi_client_connected_ssid = None
    wifi_client_connected_pp = None

    wifi_ap_ready = False

    wifi_ap_enabled = False
    wifi_ap_ssid = ''
    wifi_ap_passphrase = ''

    wifi_ap_connecting = False
    wifi_ap_running = False
    wifi_ap_running_ssid = None


    wifi_scan_thread = None
    wifi_available_networks = []
    wifi_available_networks_lock = threading.Lock()
    internet_connected = False
    internet_connected_lock = threading.Lock()

    network_checked = False
    internet_checked = False
    
    in_container = False

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "network_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        nepi_sdk.sleep(1)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        
        ##############################
        # Wait for System Info
        self.msg_if.pub_info("Waiting for nepi config info")
        self.nepi_config = nepi_system.get_nepi_config(log_name_list = [self.node_name])

        self.in_container = self.nepi_config['NEPI_IN_CONTAINER']
        self.msg_if.pub_warn("Got NEPI In Container: " + str(self.in_container))

        self.manages_network = self.nepi_config['NEPI_MANAGES_NETWORK'] == 1
        self.msg_if.pub_warn("Got running in container: " + str(self.manages_network))

        #self.manages_network = self.manages_network and (self.in_container == False) 

        ##############################
        # Initialize Variables

        self.tx_byte_cnt_deque = collections.deque(maxlen=2)
        self.rx_byte_cnt_deque = collections.deque(maxlen=2)
        

        self.detectWifiDevice()

        self.initCb(do_updates = False) 

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
            'managed_ip_addrs': {
                'namespace': self.node_namespace,
                'factory_val': []
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
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': MgrNetworkStatus,
                'qsize': 1,
                'latch': True
            },
            'saveParamsCb': {
                'namespace': self.base_namespace,
                'topic': 'saveParamsCb',
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
                'callback': self.addIpCb, 
                'callback_args': ()
            },
            'remove_ip_addr': {
                'namespace': self.base_namespace,
                'topic': 'remove_ip_addr',
                'msg': String,
                'qsize': 10,
                'callback': self.removeIpCb, 
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

        nepi_sdk.start_timer_process(self.BANDWIDTH_MONITOR_PERIOD_S, self.monitorBandwidthUsageCb)
        nepi_sdk.start_timer_process(1, self.networkIpCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1, self.internetCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1, self.dhcpCheckCb, oneshot = True)



        nepi_sdk.start_timer_process(1, self.publishStatusCb)
        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        nepi_sdk.spin()
        #########################################################

        


    #######################
    # Wait for System and Config Statuses Callbacks
    def systemStatusCb(self,msg):
        self.sys_status = msg

    def configStatusCb(self,msg):
        self.cfg_status = True
    


    def initCb(self, do_updates = False):
        self.primary_ip_addr = self.get_primary_ip_addr()
        if self.node_if is not None and self.manages_network == True:
            managed_ip_addrs = self.node_if.get_param('managed_ip_addrs')
            dhcp_enabled = self.node_if.get_param('dhcp_enabled')
            if self.in_container == True:
                self.dmanaged_ip_addrs = self.nepi_config['NEPI_IP_ALIASES']
                managed_ip_addrs = managed_ip_addrs + self.dmanaged_ip_addrs
                self.ddhcp_enabled = self.nepi_config['NEPI_DHCP_ON_START']
                dhcp_enabled = dhcp_enabled + self.ddhcp_enabled


            self.managed_ip_addrs = managed_ip_addrs
            self.dhcp_enabled = dhcp_enabled

            self.tx_bw_limit_mbps = self.node_if.get_param('tx_bw_limit_mbps')

            self.msg_if.pub_warn("Starting Init with Managed addrs: " + str(self.managed_ip_addrs))
            self.msg_if.pub_warn("Starting Init with DHCP Ennabled: " + str(self.dhcp_enabled))


            wifi_enabled = (self.dhcp_enabled == False) and self.node_if.get_param('enable_client')
            self.wifi_client_enabled = wifi_enabled
            self.wifi_client_ssid = self.node_if.get_param("client_ssid")
            self.wifi_client_passphrase = self.node_if.get_param("client_passphrase")

            self.wifi_ap_enabled = self.node_if.get_param('enable_access_point')
            self.wifi_ap_ssid = self.node_if.get_param('access_point_ssid')
            self.wifi_ap_passphrase = self.node_if.get_param('access_point_passphrase')


            self.msg_if.pub_warn("Starting Init with Wifi Client ssid: " + str(self.wifi_client_ssid))
            self.msg_if.pub_warn("Starting Init with Wifi Client password: " + str(self.wifi_client_passphrase))
           
        
       

        if do_updates == True:
            if self.node_if is not None:
                pass

            self.msg_if.pub_warn("Init Updating IP List")
            ip_addrs = self.update_ip_addr_lists()


            self.msg_if.pub_warn("Init Checking Internet Connection")
            connected = self.internet_check(do_checks=False)
            self.msg_if.pub_warn("Init Internet Check: " + str(connected))

            # Update Upload BW Limite
            # self.set_upload_bw_limit()

            if self.wifi_iface is not None:
                success = self.enable_wifi_connection(self.wifi_client_enabled)
                nepi_sdk.sleep(2)

                # Don't start these until system ready
                nepi_sdk.start_timer_process(3, self.wifiUpdateProcessesCb, oneshot=True)

            

            #self.msg_if.pub_warn("Found IPs check: " + str(ip_addrs))

            # Save Config
            if len(ip_addrs) > 0:
                self.save_network_config()
                
            
        #self.msg_if.pub_warn("Ending Init with Managed addrs: " + str(self.managed_ip_addrs))
        #self.msg_if.pub_warn("Ending Init with Wifi Client ssid: " + str(self.wifi_client_ssid))
        #self.msg_if.pub_warn("Ending Init with Wifi Client password: " + str(self.wifi_client_passphrase))

        success = self.save_config()

    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    def factoryResetCb(self,do_updates = True):
        self.aifs_classes_dict = dict()
        self.aif_classes_dict = dict()
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)

        nepi_sdk.sleep(1)
        self.msg_if.pub_warn("Reseting Factory Config")
        with open(self.USER_IP_ALIASES_FILE, "w") as f:
            f.write(self.USER_IP_ALIASES_FILE_PREFACE)

        # Set the rosmaster back to localhost
        self.set_rosmaster_impl("localhost")
        self.msg_if.pub_warn("Factory reset complete -- must reboot device for IP and ROS_MASTER_URI changes to take effect")

    def publishStatusCb(self,timer):
        self.publish_status()

    #######################
    ### Mgr Functions

    def get_primary_ip_addr(self):
        if self.in_container == False:
            key = "inet static"
            with open(self.FACTORY_STATIC_IP_FILE, "r") as f:
                lines = f.readlines()
                for i,line in enumerate(lines):
                    if key in line:
                        primary_ip_addr = lines[i+1].split()[1]
                        return primary_ip_addr
            return "Unknown Primary IP"
        else:

            """
            Attempts to find the primary IP address of the local machine.
            """
            try:
                # Create a UDP socket
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                # Connect to a public IP address (doesn't actually send data)
                # This forces the system to choose a local interface for the connection
                s.connect(("8.8.8.8", 80))  # Using Google's public DNS server
                ip_address = s.getsockname()[0]
                s.close()
                return ip_address
            except Exception as e:
                self.msg_if.pub_warn("Error getting IP address " + str(e))
                return "Unknown Primary IP"


   
    def networkIpCheckCb(self, event):
        save_config = False
        #self.msg_if.pub_warn("Entering updater process")
        found_ip_addrs = self.update_ip_addr_lists()         
        nepi_sdk.start_timer_process(self.UPDATER_INTERVAL_S, self.networkIpCheckCb, oneshot = True)



    def update_ip_addr_lists(self):
        self.primary_ip_addr = self.get_primary_ip_addr()
        last_ip_addrs = copy.deepcopy(self.found_ip_addrs)
        managed_ips = self.managed_ip_addrs
        dhcp_ip_addr = ''
        rep_ips = [self.primary_ip_addr]
        addr_list_output = subprocess.check_output(['ip','addr','list','dev',self.NET_IFACE], text=True)
        tokens = addr_list_output.split()
        for i, t in enumerate(tokens):
            #self.msg_if.pub_warn("Start Return IPs: " + str(rep_ips))
            if (t == 'inet'):
                ip_addr = tokens[i+1]
                # Ensure that aliases go at the back of the list and primary remains at the front and dhcp at end
                #self.msg_if.pub_warn("Checking IP: " + str(ip_addr))
                if ip_addr != self.primary_ip_addr and ip_addr != '':
                    if ip_addr in managed_ips:
                        #self.msg_if.pub_warn("Adding managed IP: " + str(ip_addr))
                        rep_ips.append(ip_addr)
                    elif ip_addr not in rep_ips:
                        #self.msg_if.pub_warn("Updating DHCP IP: " + str(ip_addr))
                        dhcp_ip_addr = copy.deepcopy(ip_addr)
        rep_ips.append(dhcp_ip_addr)
        #self.msg_if.pub_warn("End Return IPs: " + str(rep_ips))
        self.dhcp_ip_addr = dhcp_ip_addr
        found_ip_addrs = rep_ips
        if rep_ips[-1] == '':
            found_ip_addrs = rep_ips[:-1]
        

        for ip in self.managed_ip_addrs:
            if ip not in found_ip_addrs:
                self.add_ip(ip)
        self.network_checked = True


        self.found_ip_addrs = found_ip_addrs
        if last_ip_addrs != self.found_ip_addrs:
            self.msg_if.pub_warn("Return IPs: " + str(rep_ips))
            self.msg_if.pub_warn("Found IPs: " + str(self.found_ip_addrs))
            self.msg_if.pub_warn("Managed IPs: " + str(managed_ips))
            self.msg_if.pub_warn("DHCP IP: " + str(dhcp_ip_addr))

        if self.dhcp_ip_addr != '' and self.dhcp_enabled == False and self.internet_connected == True:
            [self.dhcp_enabled,self.dhdhcp_enable_state] = [True,True]




        return found_ip_addrs




    def internetCheckCb(self, event):
        cur_year = datetime.now().year
        self.clock_skewed = cur_year < 2024
        connected = self.internet_check()
        nepi_sdk.start_timer_process(self.UPDATER_INTERVAL_S, self.internetCheckCb, oneshot = True)

    def internet_check(self,do_checks = True):
        connected = False
        if ((self.dhcp_enabled == True or self.wifi_client_enabled == True) and (self.wifi_client_connecting == False and self.dhcp_connecting == False)) or do_checks == False:
            #self.msg_if.pub_warn("Checking Internet")
            

            #self.msg_if.pub_warn("Entering internet check process")
            with self.internet_connected_lock:
                prev_connected = self.internet_connected
                connected = False
                try:
                    #subprocess.check_call(self.INTERNET_CHECK_CMD, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    response = requests.get("https://www.google.com", timeout=1)
                    connected = True
                    if connected == True and prev_connected == False:
                        self.msg_if.pub_warn("Detected new internet connection")
                        #self.msg_if.pub_warn("Internet check response: " + str(response) )
                        
                    
                except Exception as e:
                    connected = False 
                    if connected == False and prev_connected == True:
                        self.msg_if.pub_warn("Lost internet connection")

                self.internet_connected = connected

            self.internet_checked = True
            #self.msg_if.pub_warn("Internet checked: " + str(connected) )
        return connected


    def dhcpCheckCb(self,timer):
        # Update DHCP state
        dhcp_enable = (self.wifi_client_enabled == False) and self.dhcp_enabled
        self.enable_dhcp_impl(dhcp_enable)
        nepi_sdk.start_timer_process(self.UPDATER_INTERVAL_S, self.dhcpCheckCb, oneshot = True)



    def enable_dhcp_impl(self, enabled):
        last_dhcp = copy.deepcopy(self.dhcp_enabled)
        #self.msg_if.pub_warn("Entering dhcp enable check 1 with: " + str([self.manages_network==False,self.network_checked,self.internet_checked,self.clock_skewed==False]))
        if self.manages_network == False and self.network_checked == True and self.internet_checked == True and self.clock_skewed == False:
                connected = copy.deepcopy(self.internet_connected)
                #self.msg_if.pub_warn("Entering dhcp enable check 2 with: " + str([enabled,self.dhcp_enable_state == False,self.dhcp_connecting == False]))
                if enabled is True and self.dhcp_enable_state == False and self.dhcp_connecting == False:
                        self.dhcp_connecting = True
                        self.dhcp_enabled = True
                        #self.msg_if.pub_warn("Entering dhcp enable check 3 with: " + str([self.dhcp_enable_state == False,self.wifi_client_enabled == False]))
                        if self.dhcp_enable_state == False and self.wifi_client_enabled == False:
                            self.dhcp_enable_state = True
                            self.msg_if.pub_warn("Enabling DHCP Client")
                            self.msg_if.pub_warn("Enabling DHCP with current connection state: " + str(connected))
                            try:
                                self.msg_if.pub_warn("Calling dhclient -nw subprocess")
                                subprocess.check_call(['dhclient', '-nw', self.NET_IFACE])
                                self.dhcp_enabled = True
                                self.dhcp_enable_state = True
                                self.msg_if.pub_warn("DHCP enabled")
                                #nepi_sdk.sleep(10)
                                #internet = self.internet_check(do_checks = False)
                                #self.msg_if.pub_warn("DHCP Internet check returned: " + str(internet))
                            except Exception as e:
                                self.dhcp_enabled = False
                                self.dhcp_enable_state = False
                                self.msg_if.pub_warn("Unable to enable DHCP: " + str(e))
                            success = self.publish_status()
                elif enabled is False and self.dhcp_enable_state == True:
                        #self.dhcp_enabled = False
                        #self.dhcp_ip_addr = 'Disabled->Requires Power Cycle'
                        # Requires Reboot
                        self.msg_if.pub_warn("DHCP Disabled. Requires reboot to take affect")
                        '''
                        try:
                            # The dhclient -r call below causes all IP addresses on the interface to be dropped, so
                            # we reinitialize them here... this will not work for IP addresses that were
                            # added in this session but not saved to config (i.e., not known to param server)                        
                            self.msg_if.pub_warn("Calling dhclient -r subprocess")
                            subprocess.check_call(['dhclient', '-r', self.NET_IFACE])
                            restart_network = True
                            nepi_sdk.wait()
                            self.dhcp_enabled = False
                            self.dhcp_enable_state = False
                            self.dhcp_ip_addr = ''
                         except Exception as e:
                            #self.dhcp_enabled = True
                            #self.dhcp_enable_state == True
                            self.msg_if.pub_warn("Unable to disable DHCP: " + str(e))
                        success = self.publish_status()
                        try:
                            self.msg_if.pub_warn("Restarting network IFACE: " + self.NET_IFACE)
                            # Restart the interface -- this picks the original static IP back up and sources the user IP alias file
                            subprocess.call(['ifdown', self.NET_IFACE])
                            nepi_sdk.wait()
                            subprocess.call(['ifup', self.NET_IFACE])
                            self.msg_if.pub_warn("Network IFACE restarted: " + self.NET_IFACE)
                        except Exception as e:
                            self.msg_if.pub_warn("Unable to reset NET_IFACE: " + self.NET_IFACE + " " + str(e))
                        success = self.publish_status()
                        '''
                if self.node_if is not None:
                    self.node_if.set_param('dhcp_enabled', self.dhcp_enabled)
                if last_dhcp != self.dhcp_enabled:
                    self.save_config()
        self.dhcp_connecting = False
       

    def cleanup(self):
        self.process.stop()


 

    def is_valid_ip(self,addr):
        """
        Checks if a given string is a valid IPv4 or IPv6 address.

        Args:
            ip_string (str): The string to validate as an IP address.

        Returns:
            bool: True if the string is a valid IP address, False otherwise.
        """
        valid = False
        addr_split = addr.split('/')
        if len(addr_split) > 1:
            ip = addr_split[0]
            try:
                ipaddress.ip_address(ip)
                valid = True
            except ValueError:
                pass
        return valid


    def validate_cidr_ip(self, addr):
        # First, validate the input
        tokens = addr.split('/')
        new_ip = tokens[0]
        new_ip_bits = 0
        try:
            new_ip_bits = socket.inet_aton(new_ip)
        except Exception as e:
            self.msg_if.pub_warn("Rejecting invalid IP address: " + str(addr) + " " + str(e))
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
        cur_ip_addrs = copy.deepcopy(self.found_ip_addrs)
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

    def add_ip_impl(self, addr):
        try:
            subprocess.check_call(['ip','addr','add',addr,'dev',self.NET_IFACE])
            self.save_config()
        except Exception as e:
            self.msg_if.pub_warn("Failed to set IP address to " + str(addr) + " " + str(e))


    def addIpCb(self, addr_msg):
        self.msg_if.pub_warn("Recieved Add IP address: " + str(addr_msg.data))
        addr = addr_msg.data
        self.add_ip(addr)


    def add_ip(self, addr):
        success = False
        if True == self.is_valid_ip(addr):
            if addr not in self.found_ip_addrs:
                if addr != self.primary_ip_addr: # and addr != self.dhcp_ip_addr:
                    self.found_ip_addrs.append(addr)
                    success = self.publish_status()
                    self.add_ip_impl(addr)
                    self.msg_if.pub_warn("Added IP address: " + str(addr))
                    if addr not in self.managed_ip_addrs:
                        self.managed_ip_addrs.append(addr)
                        if self.node_if is not None:
                            self.node_if.set_param('managed_ip_addrs',self.managed_ip_addrs)
                        self.save_config()
                    self.save_network_config()
                    success = True
            else:
                self.msg_if.pub_warn("Unable to add invalid/ineligible IP address: " + str(addr))
        else:
            self.msg_if.pub_warn("IP address already in system: " + str(addr))
        return success
        

    def remove_ip_impl(self, addr):
        try:
            subprocess.check_call(['ip','addr','del',addr,'dev',self.NET_IFACE])
            self.save_config()
        except Exception as e:
            self.msg_if.pub_warn("Failed to remove IP address " + str(addr) + " " + str(e))

    def removeIpCb(self, addr_msg):
        self.msg_if.pub_warn("Recieved Remove IP address: " + str(addr_msg.data))
        addr = addr_msg.data
        self.remove_ip(addr)


    def remove_ip(self, addr):
        success = False
        if True == self.is_valid_ip(addr):
            if addr in self.found_ip_addrs:
                if addr in self.managed_ip_addrs:
                        self.managed_ip_addrs.remove(addr)
                if addr != self.primary_ip_addr : # and addr != self.dhcp_ip_addr:
                    self.found_ip_addrs.remove(addr)
                    success = self.publish_status()
                    self.remove_ip_impl(addr)
                    self.msg_if.pub_warn("Removed IP address: " + str(addr))
                    if self.node_if is not None:
                        self.node_if.set_param('managed_ip_addrs',self.managed_ip_addrs)
                    self.save_config()
                    self.save_network_config()
                    success = True
            else:
                self.msg_if.pub_warn("Unable to remove invalid/ineligible IP address: " + str(addr))
        else:
            self.msg_if.pub_warn("IP address not found in system: " + str(addr))
        return success



    def enable_dhcp(self, msg):
        self.msg_if.pub_warn("Got DHCP enable request msg: " + str(msg))
        dhcp_enable = (self.wifi_client_enabled == False) and msg.data
        self.msg_if.pub_warn("Updating DHCP enable to: " + str(dhcp_enable))
        self.enable_dhcp_impl(dhcp_enable)




    def save_network_config(self):
        success = False
        # First update user static IP file
        # Note that this is outside the scope of ROS param server because we need these
        # aliases to come up even before ROS (hence this node) comes up in case the remoted ROSMASTER
        # is on a subnet only reachable via one of these aliases
        current_ips = copy.deepcopy(self.found_ip_addrs)
        managed_ips = copy.deepcopy(self.managed_ip_addrs)
        self.msg_if.pub_warn("Writing managed network ips to file: " + str(managed_ips) + " " + str(self.USER_IP_ALIASES_FILE))
        tmp_file = self.USER_IP_ALIASES_FILE + '.tmp'
        with open(tmp_file, "w") as f:
            f.write(self.USER_IP_ALIASES_FILE_PREFACE)
            if (len(current_ips) > 1):
                for i,ip_cidr in enumerate(current_ips[1:]): # Skip the first one -- that is the factory default
                    add_ip = False
                    if ip_cidr in managed_ips:
                        add_ip = True
                    elif self.dhcp_enabled == True:
                        add_ip = True
                    if add_ip:
                        alias_name = self.NET_IFACE + ":" + str(i+1)
                        f.write("auto " + alias_name + "\n")
                        f.write("iface " + alias_name + " inet static\n")
                        f.write("    address " + ip_cidr + "\n\n")
        
        os.system('cp -rf ' + tmp_file + ' ' + self.USER_IP_ALIASES_FILE)
        os.system('rm ' + tmp_file)
        success = True
        time.sleep(1) # Time for network changes to update
        self.msg_if.pub_warn("Saving system config with IP aliases")
        if self.node_if is not None:
            self.node_if.publish_pub('save_system_config',Empty())
        return success

    def get_network_config(self):
        success = False
        # First update user static IP file
        # Note that this is outside the scope of ROS param server because we need these
        # aliases to come up even before ROS (hence this node) comes up in case the remoted ROSMASTER
        # is on a subnet only reachable via one of these aliases
        config_ips = None
        if os.path.exists(self.USER_IP_ALIASES_FILE):
            self.msg_if.pub_warn("Reading network config file: " + str(self.USER_IP_ALIASES_FILE))
            tmp_file = self.USER_IP_ALIASES_FILE + '.tmp'
            os.system('cp -rf ' + self.USER_IP_ALIASES_FILE + ' ' + tmp_file)
            key = 'address '
            try:
                with open(tmp_file, "r") as f:
                    config_ips = []
                    lines = f.readlines()
                    for line in lines:
                        if line.find(key) != -1:
                            line_split = line.split(key)
                            if len(line_split)>1:
                                addr = line_split[1].replace('\n','')
                                if self.is_valid_ip(addr) == True:
                                    config_ips.append(addr)
                self.msg_if.pub_warn("Read network config file: " + str(config_ips))
            except Exception as e:
                self.msg_if.pub_warn('Failed to read network config file: ' + str(self.USER_IP_ALIASES_FILE) + ' ' + str(e))
            os.system('rm ' + tmp_file)
        return config_ips


    def save_config(self):
        success = True
        if self.node_if is not None:
            self.node_if.save_config()
            time.sleep(1) # Time for network changes to update
            #self.msg_if.pub_warn("Saving system config")
            self.node_if.publish_pub('save_system_config',Empty())
        #self.save_network_config()
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
            local_ips = copy.deepcopy(self.found_ip_addrs)
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

    def monitorBandwidthUsageCb(self, event):
        with open('/sys/class/net/' + self.NET_IFACE + '/statistics/tx_bytes', 'r') as f:
            tx_bytes = int(f.read())
            self.tx_byte_cnt_deque.append(tx_bytes)
        with open('/sys/class/net/' + self.NET_IFACE + '/statistics/rx_bytes', 'r') as f:
            rx_bytes = int(f.read())
            self.rx_byte_cnt_deque.append(rx_bytes)



    def detectWifiDevice(self):
        self.msg_if.pub_warn("Entering check Wifi iface")
        wifi_check_output = subprocess.check_output(['iw','dev'], text=True)
        for line in wifi_check_output.splitlines():
            # For now, just check for the existence of a single interface
            if line.strip().startswith('Interface'):
                if 'p2p' in line: # Peer-to-peer/WiFi Direct: NEPI does not support
                    self.msg_if.pub_warn("Ignoring P2P WiFi Direct interface " + line.strip().split()[1])
                    continue
                self.wifi_iface = line.strip().split()[1]
                self.msg_if.pub_warn("Detected Wifi on: " + str(self.wifi_iface))

                nepi_sdk.sleep(1)
                self.wifi_ap_enabled = False
                self.wifi_ap_ssid = self.DEFAULT_WIFI_AP_SSID
                self.wifi_ap_passphrase = self.DEFAULT_WIFI_AP_PASSPHRASE
        if self.wifi_iface is None:        
            self.msg_if.pub_warn("Wifi iface is None")


    def wifiUpdateProcessesCb(self,timer):
        # Configure Wifi Power Mode
        if self.wifi_adapter_ready == True:
            if self.wifi_client_enabled == True or self.wifi_ap_enabled is True:
                if self.wifi_adapter_low_power_mode == True:
                    # Turn off wifi power saving
                    self.msg_if.pub_warn("Turning off WiFi low power mode")
                    try:
                        subprocess.run(['iw',self.wifi_iface,'set','power_save','off'],
                                            capture_output=True, text=True, check=True)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed turn off wifi low power mode: " + str(e))
                    self.wifi_adapter_low_power_mode = False
            else:
                if self.wifi_adapter_low_power_mode == False:
                    # Turn off wifi power saving
                    self.msg_if.pub_warn("Turning on WiFi low power mode")
                    try:
                        subprocess.run(['iw',self.wifi_iface,'set','power_save','on'],
                                            capture_output=True, text=True, check=True)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed turn off wifi low power mode: " + str(e))
                    self.wifi_adapter_low_power_mode = True

        # Refresh Wifi Networks
        if self.wifi_adapter_ready == True:
            self.refresh_available_networks()

        # Refresh Wifi Client Connections
        if self.wifi_client_connecting == False:
            [ssid, passphrase] = self.update_wifi_client_credentials(self.wifi_client_ssid, self.wifi_client_passphrase)
            success = self.set_wifi_client(ssid,passphrase)

        # Refresh Wifi AP Connections
        success = self.set_wifi_ap()

     
        nepi_sdk.start_timer_process(1, self.wifiUpdateProcessesCb, oneshot=True)



    def refresh_available_networks_handler(self, msg):
        self.refresh_available_networks()


    def refresh_available_networks(self):
        if self.wifi_scan_thread is not None:
            #self.msg_if.pub_warn("Not refreshing available wifi networks because a refresh is already in progress")
            return
        #self.msg_if.pub_info("Clear to refresh available wifi networks with iface: " + str(self.wifi_iface) )
        # Clear the list, let the scan thread update it later
        if (self.wifi_adapter_ready == True):
            with self.wifi_available_networks_lock:
                self.wifi_scan_thread = threading.Thread(target=self.update_wifi_available_networks)
                self.wifi_scan_thread.daemon = True
                self.wifi_scan_thread.start()
    
    def update_wifi_available_networks(self):
        if self.wifi_client_enabled == True and self.wifi_adapter_ready == True and self.wifi_client_connecting == False:
            #self.msg_if.pub_warn("Debugging: Scanning for available WiFi networks") 
            #self.msg_if.pub_warn("Debugging:" + str(self.wifi_client_enabled) + " " + " " + str(self.wifi_adapter_ready))
            #self.msg_if.pub_info("Updating available WiFi connections on iface: " + str(self.wifi_iface))
            available_networks = []
            network_scan_cmd = ['iw', self.wifi_iface, 'scan']
            scan_result = ""
            try:
                scan_result = subprocess.check_output(network_scan_cmd, text=True)
                #self.msg_if.pub_warn("Got WiFi scan results: " + str(scan_result))
            except Exception as e:
                self.msg_if.pub_warn("Failed to scan for available WiFi networks: " + str(scan_result) + str(e))
                pass
            if scan_result != "":
                for scan_line in scan_result.splitlines():
                    if "SSID:" in scan_line:
                        network = scan_line.split(':')[1].strip()
                        # TODO: Need more checks to ensure this is a connectable network?
                        if network and (network not in available_networks):
                            available_networks.append(network)
                with self.wifi_available_networks_lock:
                    self.wifi_available_networks = list(available_networks)
                #self.msg_if.pub_warn("Got WiFi networks list: " + str(self.wifi_available_networks))
            self.wifi_scan_thread = None


    def enable_wifi_client_handler(self, msg):
        enabled = (self.dhcp_enabled == False) and msg.data
        self.wifi_client_enabled = enabled
        self.enable_wifi_connection(enabled)
        if self.node_if is not None:
            self.node_if.set_param("enable_client", self.wifi_client_enabled)
            success = self.save_config()

        
    def enable_wifi_connection(self,wifi_enabled):
        success = False
        if wifi_enabled == True and self.wifi_iface is not None:
            self.msg_if.pub_warn("Enabling Wifi Hardware: " + str(self.wifi_iface))
            nepi_sdk.sleep(1) # Give time for updates to publish
            try:
                link_up_cmd = self.ENABLE_DISABLE_WIFI_ADAPTER_PRE + [self.wifi_iface] + self.ENABLE_WIFI_ADAPTER_POST
                subprocess.check_call(link_up_cmd)
                self.wifi_adapter_ready = True
                success = self.publish_status()
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to get info for WiFi network: " + str(e))    
        elif wifi_enabled == False and self.wifi_iface is not None:
            self.msg_if.pub_warn("Disabling Wifi Hardware: " + str(self.wifi_iface))
            nepi_sdk.sleep(1) # Give time for updates to publish
            try:
                # Stop the supplicant
                subprocess.call(self.STOP_WPA_SUPPLICANT_CMD)
                nepi_sdk.wait()
                # Bring down the interface
                link_down_cmd = self.ENABLE_DISABLE_WIFI_ADAPTER_PRE + [self.wifi_iface] + self.DISABLE_WIFI_ADAPTER_POST
                subprocess.call(link_down_cmd)
                self.wifi_adapter_ready = False
                success = self.publish_status()
                success = True
            except:
                self.msg_if.pub_warn("Failed to stop WiFi network: " + str(e))

        return success


    def update_wifi_client_credentials(self,ssid,passphrase,save=False):
        if ssid is None or ssid == 'None' or ssid == '':
            ssid = 'None'
            passphrase = ''        

        if self.wifi_client_ssid != ssid or self.wifi_client_passphrase != passphrase:
            self.wifi_client_ssid = ssid
            self.wifi_client_passphrase = passphrase
            if save == True:
                if self.node_if is not None:
                    self.node_if.set_param("client_ssid", ssid)
                    self.node_if.set_param("client_passphrase", passphrase)
                    success = self.save_config()
                self.publish_status()
        return ssid,passphrase


    def set_wifi_client_credentials_handler(self, msg):
        self.msg_if.pub_info("Updating WiFi client credentials (SSID: " + msg.ssid + ", Passphrase: " + msg.passphrase + ")")
        if msg.ssid == 'None' or msg.ssid == '':
            ssid = 'None'
            update_wifi_client_credentials(ssid,'',save = True)
        self.set_wifi_client(msg.ssid,msg.passphrase)

    def check_wifi_connection(self):
        interface = self.wifi_iface
        try:
            # You might need to adjust the path to wpa_supplicant and its configuration
            # This example assumes a basic setup and focuses on output parsing.
            process = subprocess.Popen(
                ['wpa_supplicant', '-i', interface, '-c', '/etc/wpa_supplicant/wpa_supplicant.conf'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            start_time = time.time()
            timeout = 30  # seconds

            while True:
                line = process.stdout.readline()
                if not line and process.poll() is not None:
                    break  # Process exited
                if "CTRL-EVENT-CONNECTED" in line or "COMPLETED" in line:
                    process.terminate() # Terminate wpa_supplicant if connected
                    return True
                if time.time() - start_time > timeout:
                    process.terminate()
                    return False # Timeout
                time.sleep(0.1) # Prevent busy-waiting

        except FileNotFoundError:
            print("Error: wpa_supplicant not found. Ensure it's installed and in your PATH.")
            return False
        except Exception as e:
            print(f"An error occurred")
            return False
       
    def set_wifi_client(self, ssid, passphrase):
        success = False
        if self.wifi_client_connecting == False and self.wifi_client_enabled == True and self.dhcp_enabled == False and self.clock_skewed == False:
            self.wifi_client_connecting = True
            if self.wifi_adapter_ready == False or ssid == 'None':
                self.wifi_client_connecting = False
                self.wifi_client_connected = False
                self.wifi_client_connected_ssid = None
                self.wifi_client_connected_pp = None
                return success
            else:
                # Get current state
                ssid_set = ssid
                pp_set = passphrase
                ssid_cur = self.wifi_client_connected_ssid
                pp_cur = self.wifi_client_connected_pp
                connecting = self.wifi_client_connecting
                connected = self.wifi_client_connected

                # Check for updates
                #self.msg_if.pub_warn("Check Connecting state: " + str(updated) + " " + str(connecting) + " " + str(connected) + " ")
                
                # First shut down any connected networks
                if (ssid_set != ssid_cur or pp_set != pp_cur):
                    self.msg_if.pub_warn("Recieved Wifi credential update") 
                    self.msg_if.pub_warn("Current  Wifi Credentials (SSID: " + str(ssid_cur) + ", Passphrase: " + str(pp_cur) + ")")
                    self.msg_if.pub_warn("New Wifi Credentials (SSID: " + str(ssid_set) + ", Passphrase: " + str(pp_set) + ")")
                    self.msg_if.pub_warn("Calling Stop Wifi with command " + str(self.STOP_WPA_SUPPLICANT_CMD))
                    subprocess.call(self.STOP_WPA_SUPPLICANT_CMD)
                    self.wifi_client_connecting = False
                    self.wifi_client_connected = False
                    self.wifi_client_connected_ssid = None
                    self.wifi_client_connected_pp = None
                    nepi_sdk.sleep(1)


                
                    # Update wpa suplicant file if needed
                    
                    self.msg_if.pub_warn("Updating WiFi client credentials (SSID: " + ssid_set + ", Passphrase: " + pp_set + ")")
                    self.msg_if.pub_warn("Calling Open Wifi with command " + str(self.WPA_GENERATE_SUPPLICANT_CONF_CMD))
                    with open(self.WPA_SUPPLICANT_CONF_PATH, 'w') as f:
                        if (pp_set != ''):
                            wpa_generate_supplicant_conf_cmd = [self.WPA_GENERATE_SUPPLICANT_CONF_CMD, ssid_set,
                                                                pp_set]
                            subprocess.check_call(wpa_generate_supplicant_conf_cmd, stdout=f)
                        else: # Open network
                            # wpa_passphrase can't help us here, so generate the conf. manually
                            f.write("network={\n\tssid=\"" + ssid_set + "\"\n\tkey_mgmt=NONE\n}")
                    
                    self.wifi_client_connected_pp = pp_set
                    nepi_sdk.sleep(1)

                    # Try and connect if needed
                    if ssid_set == 'None':
                        self.wifi_client_connected_ssid = 'None'
                        self.wifi_client_connected_passphrase = ''
                    else:
                        #with self.wifi_available_networks_lock:
                        #    wifis = self.wifi_available_networks
                        #self.msg_if.pub_warn("Checking Wifi in networks: " + ssid_set + str(wifis))
                        if True: #ssid_set in wifis:
                            self.msg_if.pub_warn("Connecting WiFi client with credentials (SSID: " + ssid_set + ", Passphrase: " + pp_set + ")")
                            #### Set the connecting flag
                            self.wifi_client_connecting = True

                            #### Try to connect
                            start_supplicant_cmd = self.WPA_START_SUPPLICANT_CMD_PRE + [self.wifi_iface] + self.WPA_START_SUPPLICANT_CMD_POST
                            self.msg_if.pub_warn("Calling Start Wifi with command " + str(start_supplicant_cmd))
                            subprocess.check_call(start_supplicant_cmd)
                            
                            nepi_sdk.sleep(1)
                            success = False
                            self.msg_if.pub_warn("Updating dhclient for: " + str(self.wifi_iface))
                            try:
                                    subprocess.check_call(['dhclient', '-nw', self.wifi_iface])
                                    nepi_sdk.sleep(1)
                                    success = True
                            except Exception as e:
                                self.msg_if.pub_warn("Failed to start WiFi client (SSID=" + ssid_set + " Passphrase=" + \
                                                pp_set + "): " + str(e))
                            
                            connected = False
                            if success == True:
                                # Check for connection
                                self.msg_if.pub_warn("Checking for WiFi connection")
                                connected = self.check_wifi_connection()
                                self.msg_if.pub_warn("Got connected ssid check: " + str(connected))

                            # Check current wifi connection
                            connected_ssid = None
                            if self.wifi_client_connecting == False:
                                connected_ssid = self.get_wifi_client_connected_ssid()
                                self.msg_if.pub_warn("Got connected ssid check: " + str(connected_ssid))
                                if connected_ssid is not None:
                                    connected = True
                                    self.wifi_client_connected_ssid = connected_ssid



                            if connected == True:
                                self.msg_if.pub_warn("Connected to WiFi client (SSID=" + ssid_set + " Passphrase=" + pp_set)
                                self.wifi_client_connected = True
                                self.update_wifi_client_credentials(ssid_set,pp_set,save=True)
                                self.wifi_client_connected_ssid = ssid_set
                                self.wifi_client_connected_passphrase = pp_set
                                self.msg_if.pub_warn("Checking WiFi Internet in 5 Seconds")
                                nepi_sdk.sleep(5)
                                internet = self.internet_check(do_checks = False)
                                self.msg_if.pub_warn("Wifi Internet check returned: " + str(internet))

                            else:
                                subprocess.call(self.STOP_WPA_SUPPLICANT_CMD)
                                nepi_sdk.sleep(1)
                                subprocess.check_call(['dhclient', '-nw', self.wifi_iface])
                                self.update_wifi_client_credentials('None','',save=False)
                                self.wifi_client_connected_ssid = 'None'
                                self.wifi_client_connected_passphrase = ''
                                self.msg_if.pub_warn("Failed to start WiFi client (SSID=" + ssid_set + " Passphrase=" + pp_set )
                                self.wifi_client_connected = False

                            self.msg_if.pub_warn("WiFi Update Complete")
                            #self.msg_if.pub_warn("Current  Wifi Credentials (SSID: " + str(ssid_cur) + ", Passphrase: " + str(pp_cur) + ")")
                            #self.msg_if.pub_warn("New Wifi Credentials (SSID: " + str(ssid_set) + ", Passphrase: " + str(pp_set) + ")")
                            #self.msg_if.pub_warn("Calling Stop Wifi with command " + str(self.STOP_WPA_SUPPLICANT_CMD))

                            

        #### Clear the connecting flag
        self.wifi_client_connecting = False
        return success


               
    def get_wifi_client_connected_ssid(self):
        
        if self.wifi_iface is None:
            return None

        #self.msg_if.pub_warn("Starting Check wifi connection process with wifi iface: " + str(self.wifi_iface))
        try:
            check_connection_cmd = ['iw', self.wifi_iface, 'link']
            connection_status = subprocess.check_output(check_connection_cmd, text=True)
            #self.msg_if.pub_warn("Got wifi connection status: " + str(connection_status))
        except Excetion as e: 
            self.msg_if.pub_warn("Failed to check on wifi connection: " + str(e))
            return connected
        if connection_status.startswith('Connected'):
           connected = True
            
           for line in connection_status.splitlines():
               if line.strip().startswith('SSID'):
                   return line.strip().split()[1]
        
        return None





    def enable_wifi_ap_handler(self, msg):
        self.msg_if.pub_warn("Recieved enable wifi access point msg: " + str(msg))
        if self.wifi_iface is None:
            self.msg_if.pub_warn("Cannot enable WiFi access point - system has no WiFi adapter")
            return
        enabled = msg.data
        self.wifi_ap_enabled = enabled
        success = self.publish_status()
        self.set_wifi_ap()
        if self.node_if is not None:
            self.node_if.set_param("enable_access_point", msg.data)
            success = self.save_config()

    def set_wifi_ap_credentials_handler(self, msg):
        self.msg_if.pub_warn("Recieved set wifi access point msg: " + str(msg))
        self.wifi_ap_ssid = msg.ssid
        self.wifi_ap_passphrase = msg.passphrase
        if self.wifi_ap_ssid != msg.ssid or self.wifi_ap_passphrase != msg.passphrase:
            self.wifi_ap_ssid = msg.ssid
            self.wifi_ap_passphrase = msg.passphrase
            success = self.publish_status()
            self.set_wifi_ap()
            if self.node_if is not None:
                self.node_if.set_param("access_point_ssid", msg.ssid)
                self.node_if.set_param("access_point_passphrase", msg.passphrase)
                success = self.save_config()



    def set_wifi_ap(self):
        success = False
        if self.wifi_ap_enabled is True and self.wifi_iface is not None:
            if self.wifi_ap_running_ssid != self.wifi_ap_ssid:
                self.msg_if.pub_warn("Updating Wifi Access Point with : " + str(self.wifi_ap_ssid) + " " + str(self.wifi_ap_ssid))
                if self.wifi_ap_ssid != self.wifi_ap_running_ssid:
                    if self.wifi_ap_running  == True:
                        try:
                            # Kill any current access point -- no problem if one isn't already running; just returns immediately
                            subprocess.call([self.CREATE_AP_CALL, '--stop', self.wifi_iface])
                           
                            self.wifi_ap_running_ssid = None
                            self.wifi_ap_running = False
                            self.wifi_ap_connecting = False
                            nepi_sdk.wait()
                        except Exception as e:
                            self.msg_if.pub_warn("Unable to terminate wifi access point: " + str(e))
                    if self.wifi_ap_ssid != '':
                        try:
                            self.msg_if.pub_warn("Starting WiFi access point on ssid: " + str(self.wifi_ap_ssid))
                            # Use the create_ap command line
                            subprocess.check_call([self.CREATE_AP_CALL, '-n', '--redirect-to-localhost', '--isolate-clients', '--daemon',
                                                self.wifi_iface, self.wifi_ap_ssid, self.wifi_ap_passphrase])
           
                            self.msg_if.pub_warn("Started WiFi access point: " + str(self.wifi_ap_ssid))
                            self.wifi_ap_running_ssid = self.wifi_ap_ssid
                            self.wifi_ap_running = True
                            self.wifi_ap_connecting = False
                            success = True
                        except Exception as e:
                            self.msg_if.pub_warn("Unable to start wifi access point with " + str(e))
        elif self.wifi_ap_running == True and self.wifi_iface is not None:
            self.msg_if.pub_warn("Disabling Wifi Access Point")
            try:
                subprocess.check_call([self.CREATE_AP_CALL, '--stop', self.wifi_iface])
                self.wifi_ap_running_ssid = None
                self.wifi_ap_running = False
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Unable to terminate wifi access point: " + str(e))
        else: 
            success = True
        return success


    def handle_ip_addr_query(self, req):
        return {'ip_addrs':self.found_ip_addrs, 
            'primary_ip_addr':self.primary_ip_addr,
            'managed_ip_addrs':self.managed_ip_addrs,
            'dhcp_enabled': self.dhcp_enabled, 
            'dhcp_ip_addr': self.dhcp_ip_addr,
            'internet_connected':self.internet_connected}

    def handle_bandwidth_usage_query(self, req):
        tx_rate_mbps = 0
        if len(self.tx_byte_cnt_deque) > 1:
            tx_rate_mbps =  8 * (self.tx_byte_cnt_deque[1] - self.tx_byte_cnt_deque[0]) / (self.BANDWIDTH_MONITOR_PERIOD_S * 1000000)

        rx_rate_mbps = 0
        if len(self.rx_byte_cnt_deque) > 1:
            rx_rate_mbps = 8 * (self.rx_byte_cnt_deque[1] - self.rx_byte_cnt_deque[0]) / (self.BANDWIDTH_MONITOR_PERIOD_S * 1000000)



        return {'tx_rate_mbps':tx_rate_mbps, 'rx_rate_mbps':rx_rate_mbps, 'tx_limit_mbps': self.tx_bw_limit_mbps}

    def handle_wifi_query(self, req):
        with self.wifi_available_networks_lock:
            wifis = self.wifi_available_networks

        with self.internet_connected_lock:
            internet_connected = self.internet_connected
        

        return {'has_wifi': (self.wifi_iface is not None), 
                'wifi_ap_enabled': self.wifi_ap_enabled,
                'wifi_ap_ssid': self.wifi_ap_ssid, 
                'wifi_ap_passphrase': self.wifi_ap_passphrase,
                'wifi_ap_running': self.wifi_ap_running,
                'wifi_client_enabled': self.wifi_client_enabled,
                'wifi_client_connecting': self.wifi_client_connecting,
                'wifi_client_connected': self.wifi_client_connected,
                'wifi_client_ssid': self.wifi_client_ssid,
                'wifi_client_passphrase': self.wifi_client_passphrase,
                'available_networks': wifis,
                'clock_skewed': self.clock_skewed,
                'internet_connected': internet_connected}


    def publish_status(self):
        success = False
        with self.wifi_available_networks_lock:
            wifis = self.wifi_available_networks

        with self.internet_connected_lock:
            internet_connected = self.internet_connected

        self.status_msg.manages_network = self.manages_network
        # Wired Info
        self.status_msg.ip_addrs = self.found_ip_addrs 
        self.status_msg.primary_ip_addr = self.primary_ip_addr
        self.status_msg.managed_ip_addrs = self.managed_ip_addrs
        self.status_msg.dhcp_enabled =  self.dhcp_enabled 
        
        
        # WiFi Info
        self.status_msg.has_wifi =  (self.wifi_iface is not None) 
        self.status_msg.wifi_ap_enabled =  self.wifi_ap_enabled
        self.status_msg.wifi_ap_ssid =  self.wifi_ap_ssid 
        self.status_msg.wifi_ap_passphrase =  self.wifi_ap_passphrase
        self.status_msg.wifi_ap_running =  self.wifi_ap_running
        self.status_msg.wifi_client_enabled =  self.wifi_client_enabled
        self.status_msg.wifi_client_connecting =  self.wifi_client_connecting
        self.status_msg.wifi_client_connected =  self.wifi_client_connected
        self.status_msg.wifi_client_ssid =  self.wifi_client_ssid
        self.status_msg.wifi_client_passphrase =  self.wifi_client_passphrase
        self.status_msg.available_networks =  wifis
        self.status_msg.clock_skewed =  self.clock_skewed

        # Internet Info
        self.status_msg.dhcp_ip_addr =  self.dhcp_ip_addr
        self.status_msg.internet_connected = internet_connected

        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', self.status_msg)
            success = True

        return success


if __name__ == "__main__":
    NetworkMgr()
