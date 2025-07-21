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
import time
from shutil import copyfile
import re
import errno
import subprocess
import sys
import pytz
import datetime


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
 

from std_msgs.msg import String, Empty, Time
from nepi_interfaces.msg import MgrSystemStatus
from std_srvs.srv import Empty as EmptySrv

from nepi_interfaces.msg import Reset, TimeUpdate
from nepi_interfaces.msg import MgrTimeStatus

from nepi_interfaces.srv import TimeStatusQuery, TimeStatusQueryRequest, TimeStatusQueryResponse


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF


FACTORY_CFG_SUFFIX = '.factory'
USER_CFG_SUFFIX = '.user'

CHRONY_CFG_LINKNAME = '/etc/chrony/chrony.conf'
CHRONY_CFG_BASENAME = '/opt/nepi/config/etc/chrony/chrony.conf'
CHRONY_SYSTEMD_SERVICE_NAME = 'chrony.service'

FACTORY_TIMEZONE = 'UTC'

class time_sync_mgr(object):
 

    last_set_time = 0.0
    ntp_first_sync_time = None
    ntp_status_check_timer = None

    node_if = None

    timezone = FACTORY_TIMEZONE

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = 'time_sync_mgr' # Can be overwitten by luanch command
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
        self.msg_if.pub_info("Waiting for system info")
        self.in_container = nepi_system.get_in_container(log_name_list = [self.node_name])
        self.msg_if.pub_warn("Got running in container: " + str(self.in_container))
        
              
        ##############################
        # Initialize Class Variables
        self.time_status = MgrTimeStatus()
        self.set_timezone(FACTORY_TIMEZONE)

        self.initCb(do_updates = False)
        
        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self.initCb,
            'system_reset_callback': self.sysResetCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace': self.node_namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'init_time_from_rtc': {
                'namespace': self.node_namespace,
                'factory_val': True
            },
            'timezone': {
                'namespace': self.node_namespace,
                'factory_val': FACTORY_TIMEZONE
            }
        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'time_status_query': {
                'namespace': self.base_namespace,
                'topic': 'time_status_query',
                'srv': TimeStatusQuery,
                'req': TimeStatusQueryRequest(),
                'resp': TimeStatusQueryResponse(),
                'callback': self.handle_time_status_query
            }
        }



        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': Empty,
                'qsize': 1,
                'latch': True
            },
            'sys_time_updated': {
                'namespace': self.base_namespace,
                'topic': 'sys_time_updated',
                'msg': Empty,
                'qsize': 3,
                'latch': True
            }
        }  

        if self.in_container == False:
            # Subscribers Config Dict ####################
            self.SUBS_DICT = {
                'add_ntp_server': {
                    'namespace': self.base_namespace,
                    'topic': 'add_ntp_server',
                    'msg': String,
                    'qsize': None,
                    'callback': self.add_server, 
                    'callback_args': ()
                },
                'remove_ntp_server': {
                    'namespace': self.base_namespace,
                    'topic': 'remove_ntp_server',
                    'msg': String,
                    'qsize': None,
                    'callback': self.remove_server, 
                    'callback_args': ()
                },
                'set_time': {
                    'namespace': self.base_namespace,
                    'topic': 'set_time',
                    'msg': TimeUpdate,
                    'qsize': 10,
                    'callback': self.set_time, 
                    'callback_args': ()
                }
            }
        else:
            self.SUBS_DICT = None





        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT
        )



        #ready = self.node_if.wait_for_ready()
        nepi_sdk.wait()
            

        # Upddate Vars
        timezone = self.node_if.get_param('timezone')
        self.set_timezone(timezone)

        if self.in_container == False:

            # Initialize the system clock from the RTC if so configured
            # RTC will be updated whenever a "good" clock source is detected; that will control drift
            init_from_rtc = self.node_if.get_param("init_time_from_rtc")
            if init_from_rtc is True:
                self.msg_if.pub_info("Initializing system clock from hardware clock")
                subprocess.call(['hwclock', '-s'])
                self.informClockUpdate() 

            # Set up a periodic timer to check for NTP sync so we can inform the rest of the system when first sync detected
            self.ntp_status_check_timer = nepi_sdk.start_timer_process(5.0, self.gather_ntp_status_timer_cb)
            self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)
            

        else:
            self.msg_if.pub_info("NEPI running in Container Mode. Time and NTP managed by host system")


        nepi_sdk.start_timer_process(1, self.statusPubCb)

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_sdk.spin()
        #########################################################

    def statusPubCb(self,timer):
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', Empty())
    

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

    def gather_ntp_status_timer_cb(self,event):
        # Just call the implementation method. We don't care about the event payload
        self.gather_ntp_status()


    def updaterCb(self,timer):

        #os.environ['TZ'] = time.strftime('%Z')
        #time.tzset()

        # Get Last PPS time from the sysfs node
        #pps_exists = os.path.isfile('/sys/class/pps/pps0/assert')
        pps_exists = False # Hard code if for now, since Jetson isn't defining /sys/class/pps -- we may never actually use PPS
        if pps_exists:
            pps_string = subprocess.check_output(["cat", "/sys/class/pps/pps0/assert"], text=True)
            pps_tokens = pps_string.split('#')
            if (len(pps_tokens) >= 2):
                self.time_status.last_pps = float(pps_string.split('#')[0])
            else:
                self.time_status.last_pps = 0.0
                self.msg_if.pub_warn("Unable to parse /sys/class/pps/pps0/assert")
        else: # Failed to find the assert file - just return no PPS
            self.time_status.last_pps = 0.0


        ntp_status = self.gather_ntp_status()
        for status_entry in ntp_status:
            if status_entry[0] not in self.time_status.ntp_sources:
                self.time_status.ntp_sources.append(status_entry[0])
                self.time_status.currently_syncd.append(status_entry[1])
                self.time_status.last_ntp_sync.append(status_entry[2])
                self.time_status.current_offset.append(status_entry[3])
            else:
                ind = self.time_status.ntp_sources.index(status_entry[0])
                if ind != -1:
                    self.time_status.currently_syncd[ind] = status_entry[1]
                    self.time_status.last_ntp_sync[ind] = status_entry[2]
                    self.time_status.current_offset[ind] = status_entry[3]

        self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)



    def gather_ntp_status(self):
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
                if (self.ntp_first_sync_time is None) and (currently_syncd is True):
                    self.msg_if.pub_info("NTP sync first detected... publishing on sys_time_update")
                    self.ntp_first_sync_time = nepi_utils.get_time()
                    self.informClockUpdate()

                    # Update the RTC with this "better" clock source
                    self.msg_if.pub_info("Updating hardware clock with NTP time")
                    subprocess.call(['hwclock', '-w'])

        return ntp_status


    def set_time(self,msg):
        # TODO: Bounds checking?
        self.msg_if.pub_debug("Got time update msg: " + str(msg), throttle_s = 5.0)
        update_time = msg.update_time
        if update_time == True and self.in_container == False:
            self.msg_if.pub_info("Setting time from set_time topic: " + str(msg.secs) + '.' + str(msg.nsecs))
            timestring = '@' + str(float(msg.secs) + (float(msg.nsecs) / float(1e9)))
            try:
                subprocess.call(["date", "-s", timestring])
                self.last_set_time = float(msg.secs) + float(msg.nsecs)/1000000000
                new_date = subprocess.check_output(["date"], text=True)
                self.msg_if.pub_info("Updated date: " + str(new_date))
            except Exception as e:
                self.msg_if.pub_warn("Failed to update time: " + str(e))


        update_timezone = msg.update_timezone
        if update_timezone == True:
            self.msg_if.pub_info("Setting timezone to: " + msg.timezone)
            self.set_timezone(msg.timezone)
            self.node_if.save_config()

        if self.in_container == False:
            # Update the hardware clock from this "better" clock source; helps with RTC drift
            self.msg_if.pub_info("Updating hardware clock from set_time value")
            subprocess.call(['hwclock', '-w'])

        # And tell the rest of the system
        #self.informClockUpdate()

    def set_timezone(self,timezone):
        if self.timezone != timezone:
            try:
                self.msg_if.pub_warn("Updating timezone: " + str(timezone))
                os.environ["TZ"] = timezone
                time.tzset()
                self.timezone = timezone
                if self.node_if is not None:
                    self.node_if.set_param('timezone',timezone)
                    self.node_if.save_config()
            except Exception as e:
                self.msg_if.pub_warn("Failed to update timezone: " + str(e))
                tz = datetime.datetime.now(datetime.timezone.utc).astimezone().tzname()
                self.timezone = nepi_utils.get_timezone_description(tz)
            if self.in_container == False:
                try:
                    subprocess.call(["timedatectl", "set-timezone", self.timezone])
                except:
                    pass
            nepi_sdk.set_param('timezone',self.timezone)


    def handle_time_status_query(self,req):
        current_time = nepi_utils.get_time()
        self.time_status.current_time = current_time

        self.time_status.timezone_id = time.strftime('%Z')
        self.time_status.timezone_description = self.timezone

        tzo = pytz.timezone(self.timezone)
        now = datetime.datetime.now(tzo) # current date and time
        dt_str = now.strftime("%m/%d/%Y,%H:%M:%S")
        self.time_status.date_str = dt_str.split(',')[0]
        self.time_status.time_str = dt_str.split(',')[1]
        
        timezones = nepi_utils.standard_timezones_dict.keys()
             
        #self.msg_if.pub_warn("Returning Time Status response: " + str(self.time_status))

        #self.msg_if.pub_warn("Handle time: " + str(nepi_utils.get_time() - start_time))

        # Last set time (cheater clock sync method)
        self.time_status.last_set_time = self.last_set_time


        return  { 'time_status': self.time_status, 'available_timezones': timezones }



    def initCb(self, do_updates = False):
        if do_updates == True:
             nepi_sdk.set_param('timezone',self.timezone)


    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    def factoryResetCb(self,do_updates = True):
        if self.node_if is not None:
            pass
        if do_updates == True:
            self.msg_if.pub_info("Restoring NTP to factory config")
            self.reset_to_factory_conf()

        self.initCb()       


    def sysResetCb(self,reset_type = 0):
        if Reset.USER_RESET == reset_type:
            # Nothing to do for a User Reset as config file is always up-to-date
            self.msg_if.pub_info("Ignoring NTP user-reset NO-OP")
        elif Reset.FACTORY_RESET == reset_type:
            pass # factoryResetCb called by node_if
        elif Reset.SOFTWARE_RESET == reset_type:
            self.msg_if.pub_info("Executing soft reset for NTP")
            self.restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)
            nepi_sdk.signal_shutdown('Shutdown by request')
        elif Reset.HARDWARE_RESET == reset_type:
            self.msg_if.pub_info("Executing hard reset for NTP")
            # TODO: Any hardware restart required?
            self.restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)
            nepi_sdk.signal_shutdown('Shutdown by request')


    def informClockUpdate(self):
        self.node_if.publish_pub('sys_time_updated', Empty()) # Make sure to inform the rest of the nodes that the system clock was updated

        # For onvif_mgr, must use a service rather than the system_time_updated topic due to limitation with onvif_mgr message subscriptions
        topic = nepi_sdk.find_service('onvif_mgr/resync_onvif_device_clocks')
        if topic != "":
            try:
                nepi_sdk.wait_for_service('onvif_mgr/resync_onvif_device_clocks', timeout=0.1)
                resync_srv = nepi_sdk.create_service('onvif_mgr/resync_onvif_device_clocks', EmptySrv)
                resync_srv()
            except Exception as e:
                pass

if __name__ == '__main__':
    time_sync_mgr()
