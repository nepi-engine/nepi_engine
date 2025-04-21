#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import os.path
from shutil import copyfile
import re
import errno
import subprocess
import sys


from nepi_sdk import nepi_ros
 

from std_msgs.msg import String, Empty, Time
from nepi_ros_interfaces.msg import SystemStatus
from std_srvs.srv import Empty as EmptySrv

from nepi_ros_interfaces.msg import Reset
from nepi_ros_interfaces.msg import TimeStatus

from nepi_ros_interfaces.srv import TimeStatusQuery, TimeStatusQueryResponse


from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.connect_mgr_if_system import ConnectMgrSystemIF
from nepi_api.connect_mgr_if_config import ConnectMgrConfigIF


FACTORY_CFG_SUFFIX = '.factory'
USER_CFG_SUFFIX = '.user'

CHRONY_CFG_LINKNAME = '/etc/chrony/chrony.conf'
CHRONY_CFG_BASENAME = '/opt/nepi/config/etc/chrony/chrony.conf'
CHRONY_SYSTEMD_SERVICE_NAME = 'chrony.service'

g_last_set_time = nepi_ros.Time(0.0)
g_sys_time_updated_pub = None
g_ntp_first_sync_time = None
g_ntp_status_check_timer = None

class time_sync_mgr(object):
 
    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = 'time_sync_mgr' # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################
        ## Wait for NEPI core managers to start
        # Wait for System Manager
        mgr_sys_if = ConnectMgrSystemIF()
        success = mgr_sys_if.wait_for_status()
        if success == False:
            nepi_ros.signal_shutdown(self.node_name + ": Failed to get System Status Msg")
        sys_stats_dict = mgr_sys_if.get_system_stats_dict()
        in_container = False
        if sys_stats_dict is not None: 
            #self.msg_if.pub_info("Got System Stats Dict: " + str(sys_stats_dict))
            in_container = sys_stats_dict['in_container']
        self.in_container = in_container
        
        # Wait for Config Manager
        mgr_cfg_if = ConnectMgrConfigIF()
        success = mgr_cfg_if.wait_for_status()
        if success == False:
            nepi_ros.signal_shutdown(self.node_name + ": Failed to get Config Status Msg")
        
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

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'sys_time_updated': {
                'namespace': self.node_namespace,
                'topic': 'sys_time_updated'
                'msg': Empty,
                'qsize': 3,
                'latch': True
            }
        }  

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'add_ntp_server': {
                'namespace': self.node_namespace,
                'topic': 'add_ntp_server',
                'msg': String,
                'qsize': None,
                'callback': self.add_server, 
                'callback_args': ()
            },
            'remove_ntp_server': {
                'namespace': self.node_namespace,
                'topic': 'remove_ntp_server',
                'msg': String,
                'qsize': None,
                'callback': self.remove_server, 
                'callback_args': ()
            },
            'reset': {
                'namespace': self.node_namespace,
                'topic': 'reset',
                'msg': Reset,
                'qsize': None,
                'callback': self.reset, 
                'callback_args': ()
            },
            'set_time': {
                'namespace': self.node_namespace,
                'topic': 'set_time',
                'msg': Time,
                'qsize': None,
                'callback': self.set_time, 
                'callback_args': ()
            }
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'aifs_dict': {
                'namespace': self.node_namespace,
                'factory_val': dict()
            },
            'init_time_from_rtc': {
                'namespace': self.node_namespace,
                'factory_val': True
            }
        }



        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        ready = self.node_if.wait_for_ready()
            

        if self.in_container == False:

            nepi_ros.create_service('time_status_query', TimeStatusQuery, self.handle_time_status_query)

            global g_sys_time_updated_pub

            # Initialize the system clock from the RTC if so configured
            # RTC will be updated whenever a "good" clock source is detected; that will control drift
            init_from_rtc = self.node_if.get_param("~init_time_from_rtc", True)
            if init_from_rtc is True:
                self.msg_if.pub_info("Initializing system clock from hardware clock")
                subprocess.call(['hwclock', '-s'])
                self.informClockUpdate() 

            # Set up a periodic timer to check for NTP sync so we can inform the rest of the system when first sync detected
            global g_ntp_status_check_timer
            g_ntp_status_check_timer = nepi_ros.start_timer_process(5.0, self.gather_ntp_status_timer_cb)
            g_ntp_status_check_timer.run()
        else:
            self.msg_if.pub_info("NEPI running in Container Mode. Time and NTP managed by host system")
        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################


    def configStatusCb(self,msg):
        self.cfg_status = True
    

    #######################
    ### Mgr Config Functions



    def symlink_force(self,target, link_name):
        try:
            os.symlink(target, link_name)
        except OSError as e:
            if e.errno == errno.EEXIST:
                os.remove(link_name)
                os.symlink(target, link_name)
            else:
                self.msg_if.pub_warn("Unable to create symlink")
                return False
        
        return True

    def ensure_user_conf(self):
        userconf_path = CHRONY_CFG_BASENAME + USER_CFG_SUFFIX
        if (False == os.path.isfile(userconf_path)):
            # Need to create it from a copy of the factory config
            factoryconf_path = CHRONY_CFG_BASENAME + FACTORY_CFG_SUFFIX
            try:
                copyfile(factoryconf_path, userconf_path)
            except:
                self.msg_if.pub_warn("Unable to copy conf")
                return False

        return self.symlink_force(userconf_path, CHRONY_CFG_LINKNAME)

    def restart_systemd_service(self,service_name):
        subprocess.call(["systemctl", "restart", service_name])

    def reset_to_factory_conf(self):
        userconf_path = CHRONY_CFG_BASENAME + USER_CFG_SUFFIX
        factoryconf_path = CHRONY_CFG_BASENAME + FACTORY_CFG_SUFFIX

        self.symlink_force(factoryconf_path, CHRONY_CFG_LINKNAME)
        if (True == os.path.isfile(userconf_path)):
            os.remove(userconf_path)
            self.msg_if.pub_info("Removed user config: " + userconf_path)

        # Restart chrony to allow changes to take effect
        self.restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)

    def add_server(self,server_host):
        if (False == self.ensure_user_conf()):
            return

        userconf_path = CHRONY_CFG_BASENAME + USER_CFG_SUFFIX

        #ensure just a simple hostname is being added
        host = server_host.data.split()[0]

        new_server_cfg_line = 'server ' + host + ' iburst minpoll 2'
        # TODO: May one day want to user chrony option initstepslew for even earlier synchronization
        #init_slew_cfg_line = 'initstepslew 1 ' + host
        match_line = '^' + new_server_cfg_line
        file = open(userconf_path, 'r+')
        found_match = False
        for line in file.readlines():
            if re.search(match_line, line):
                self.msg_if.pub_info("Ignoring redundant NTP server additions for: " + str(host))
                found_match = True
                break

        #At EOF, so just write here
        if (False == found_match):
            self.msg_if.pub_info("Adding new NTP server: " + host)
            file.write(new_server_cfg_line + '\n')
            # Restart chrony to allow changes to take effect
            self.restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)

    def remove_server(self,server_host):
        userconf_path = CHRONY_CFG_BASENAME + USER_CFG_SUFFIX
        if (False == os.path.isfile(userconf_path)):
            self.msg_if.pub_info("Ignoring request to remove NTP server since factory config is in use")
            return

        # Make sure the symlink points to the user config  we've already established that user cfg exists
        if (False == self.ensure_user_conf()):
            return

        #ensure just a simple hostname is being added
        host = server_host.data.split()[0]

        match_line = '^server ' + host + ' iburst minpoll 2'
        # Must copy the file linebyline to a tmp, then overwrite the original
        orig_file = open(userconf_path, 'r')
        tmpfile_path = userconf_path + ".tmp"
        tmp_file = open(tmpfile_path, 'w')
        found_it = False
        for line in orig_file.readlines():
            if re.search(match_line, line):
                # Don't write this line as we want to eliminate it
                self.msg_if.pub_info("Removing NTP server: " + str(host))
                found_it = True
                continue
            else:
                tmp_file.write(line)

        orig_file.close()
        tmp_file.close()
        os.rename(tmpfile_path, userconf_path)

        if True == found_it:
            # Restart chrony to allow changes to take effect
            self.restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)

    def reset(self,msg):
        if Reset.USER_RESET == msg.reset_type:
            # Nothing to do for a User Reset as config file is always up-to-date
            self.msg_if.pub_info("Ignoring NTP user-reset NO-OP")
        elif Reset.FACTORY_RESET == msg.reset_type:
            self.msg_if.pub_info("Restoring NTP to factory config")
            self.reset_to_factory_conf()
        elif Reset.SOFTWARE_RESET == msg.reset_type:
            self.msg_if.pub_info("Executing soft reset for NTP")
            self.restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)
            nepi_ros.signal_shutdown('Shutdown by request')
        elif Reset.HARDWARE_RESET == msg.reset_type:
            self.msg_if.pub_info("Executing hard reset for NTP")
            # TODO: Any hardware restart required?
            self.restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)
            nepi_ros.signal_shutdown('Shutdown by request')

    def gather_ntp_status_timer_cb(self,event):
        # Just call the implementation method. We don't care about the event payload
        self.gather_ntp_status()

    def gather_ntp_status(self):
        global g_ntp_first_sync_time
        global g_ntp_status_check_timer


        chronyc_sources = subprocess.check_output(["chronyc", "sources"], text=True).splitlines()
        ntp_status = [] # List of lists
        for line in chronyc_sources[1:]:
            if re.search('^\^|#', line): # Find sources lines by their leading "Mode" indicator
                tokens = line.split()
                source = tokens[1]
                currently_syncd = ('*' in tokens[0]) or ('+' in tokens[0])
                last_sync = tokens[5]
                current_offset = tokens[6].split('[')[0] # The string has two parts
                ntp_status.append((source, currently_syncd, last_sync, current_offset))
                if (g_ntp_first_sync_time is None) and (currently_syncd is True):
                    self.msg_if.pub_info("NTP sync first detected... publishing on sys_time_update")
                    g_ntp_first_sync_time = nepi_ros.get_rostime_to_sec()
                    self.informClockUpdate()

                    # Update the RTC with this "better" clock source
                    self.msg_if.pub_info("Updating hardware clock with NTP time")
                    subprocess.call(['hwclock', '-w'])

                    # No longer need to run the timer
                    # TODO: Should we continue to run timer to monitor for big changes, additional syncs, etc. to
                    # inform the rest of the system about the time update?
                    g_ntp_status_check_timer.shutdown()

        return ntp_status

    def handle_time_status_query(self,req):
        time_status = TimeStatus()
        time_status.current_time = nepi_ros.get_rostime_to_sec()

        # Get Last PPS time from the sysfs node
        #pps_exists = os.path.isfile('/sys/class/pps/pps0/assert')
        pps_exists = False # Hard code if for now, since Jetson isn't defining /sys/class/pps -- we may never actually use PPS
        if pps_exists:
            pps_string = subprocess.check_output(["cat", "/sys/class/pps/pps0/assert"], text=True)
            pps_tokens = pps_string.split('#')
            if (len(pps_tokens) >= 2):
                time_status.last_pps = nepi_ros.Time(float(pps_string.split('#')[0]))
            else:
                time_status.last_pps = nepi_ros.Time(0)
                self.msg_if.pub_warn("Unable to parse /sys/class/pps/pps0/assert"):
        else: # Failed to find the assert file - just return no PPS
            time_status.last_pps = nepi_ros.Time(0)

        ntp_status = self.gather_ntp_status()
        for status_entry in ntp_status:
            time_status.ntp_sources.append(status_entry[0])
            time_status.currently_syncd.append(status_entry[1])
            time_status.last_ntp_sync.append(status_entry[2])
            time_status.current_offset.append(status_entry[3])


        # Last set time (cheater clock sync method)
        time_status.last_set_time = g_last_set_time

        return  { 'time_status': time_status }

    def set_time(self,msg):
        # TODO: Bounds checking?
        # Use the Linux 'date -s' command-line utility.
        self.msg_if.pub_info("Setting time from set_time topic: " + str(msg.data.secs) + '.' + str(msg.data.nsecs))
        timestring = '@' + str(float(msg.data.secs) + (float(msg.data.nsecs) / float(1e9)))
        subprocess.call(["date", "-s", timestring])
        global g_last_set_time
        g_last_set_time = msg.data
        new_date = subprocess.check_output(["date"], text=True)
        self.msg_if.pub_info("Updated date: " + str(new_date))

        # Update the hardware clock from this "better" clock source; helps with RTC drift
        self.msg_if.pub_info("Updating hardware clock from set_time value")
        subprocess.call(['hwclock', '-w'])

        # And tell the rest of the system
        self.informClockUpdate()

        # TODO: Should we use this CTypes call into librt instead?
    #    import ctypes
    #    import ctypes.util
    #    import time

        # /usr/include/linux/time.h:
        #
        # define CLOCK_REALTIME             0
    #    CLOCK_REALTIME = 0

        # /usr/include/time.h
        #
        # struct timespec
        #  {
        #    __time_t tv_sec;        /* Seconds.  */
        #    long int tv_nsec;       /* Nanoseconds.  */
        #  };
        # Structure for the timespec arg. to librt::clock_settime()
    #    class timespec(ctypes.Structure):
    #        _fields_ = [("tv_sec", ctypes.c_long),
    #                    ("tv_nsec", ctypes.c_long)]

        # Bring in the C-language librt
    #    librt = ctypes.CDLL(ctypes.util.find_library("rt"))

        # Create a new timespec object
    #    ts = timespec()
    #    ts.tv_sec = int(msg.data.secs)
    #    ts.tv_nsec = int(msg.data.nsecs)



    def initCB(self):
        pass

    def resetCb(self):
        pass

    def factoryResetCb(self):
        pass

    def informClockUpdate(self):
        global g_sys_time_updated_pub
       self.node_if.publish_pub('g_sys_time_updated_pub') # Make sure to inform the rest of the nodes that the system clock was updated

        # For onvif_mgr, must use a service rather than the system_time_updated topic due to limitation with onvif_mgr message subscriptions
        try:
            nepi_ros.wait_for_service('onvif_mgr/resync_onvif_device_clocks', timeout=0.1)
            resync_srv = nepi_ros.create_service('onvif_mgr/resync_onvif_device_clocks', EmptySrv)
            resync_srv()
        except Exception as e:
            pass

if __name__ == '__main__':
    time_sync_mgr()
