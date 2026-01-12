#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
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
 

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Time
from std_srvs.srv import Empty as EmptySrv

from nepi_interfaces.msg import Reset, TimeUpdate
from nepi_interfaces.msg import MgrTimeStatus

from nepi_interfaces.srv import TimeStatusQuery, TimeStatusQueryRequest, TimeStatusQueryResponse


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF


USER_CFG_SUFFIX = '.user'

CHRONY_CFG_BASENAME = '/opt/nepi/etc/chrony/chrony.conf'
CHRONY_CFG_FACTORY = '/mnt/nepi_config/factory_cfg/etc/chrony/chrony.conf'

FACTORY_TIMEZONE = 'UTC'

class time_sync_mgr(object):
 

    last_set_time = 0.0
    clock_synced = False
    ntp_first_sync_time = None
    ntp_status_check_timer = None

    node_if = None

    timezone = FACTORY_TIMEZONE

    status_msg = MgrTimeStatus()

    auto_sync_clocks = True
    auto_sync_timezones = True
    
    chrony_running = False
    in_container = False

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



        nepi_system.set_timezone(self.timezone)

        ##############################
        # Wait for System Info
        self.msg_if.pub_info("Waiting for nepi config info")
        self.nepi_config = nepi_system.get_nepi_config(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got NEPI config: " + str(self.nepi_config))

        self.in_container = self.nepi_config['NEPI_IN_CONTAINER'] == 1
        self.msg_if.pub_warn("Got NEPI In Container: " + str(self.in_container))

        self.manages_time = self.nepi_config['NEPI_MANAGES_TIME'] == 1
        self.msg_if.pub_warn("Got NEPI Manages Time: " + str(self.manages_time))
  
        self.nepi_config_path = self.nepi_config['NEPI_CONFIG']
    
        
        self.msg_if.pub_warn("Waiting for Config Mgr")
        config_folders = nepi_system.get_config_folders()
        ##############################
        # Initialize Class Variables
        self.msg_if.pub_warn("Setting Timezone to: " + str(FACTORY_TIMEZONE))
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
            },
            'auto_sync_clocks': {
                'namespace': self.node_namespace,
                'factory_val': True
            },
            'auto_sync_timezones': {
                'namespace': self.node_namespace,
                'factory_val': True
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
                'msg': MgrTimeStatus,
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

        if self.manages_time == True:
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
                },
                'auto_sync_clocks': {
                    'namespace': self.base_namespace,
                    'topic': 'auto_sync_clocks',
                    'msg': Bool,
                    'qsize': 10,
                    'callback': self.autoSyncClocksCb, 
                    'callback_args': ()
                },
                'auto_sync_timezones': {
                    'namespace': self.base_namespace,
                    'topic': 'auto_sync_timezones',
                    'msg': Bool,
                    'qsize': 10,
                    'callback': self.autoSyncTimezonesCb, 
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

        if self.manages_time == True:

            # Initialize the system clock from the RTC if so configured
            # RTC will be updated whenever a "good" clock source is detected; that will control drift
            init_from_rtc = self.node_if.get_param("init_time_from_rtc")
            if init_from_rtc is True:
                self.msg_if.pub_info("Initializing system clock from hardware clock")
                subprocess.call(['sudo','hwclock', '-s'])
                self.informClockUpdate() 

            # Set up a periodic timer to check for NTP sync so we can inform the rest of the system when first sync detected
            #self.ntp_status_check_timer = nepi_sdk.start_timer_process(5.0, self.gather_ntp_status_timer_cb)
            self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)
            

        else:
            self.msg_if.pub_info("NEPI Time Management Disabled")


        nepi_sdk.start_timer_process(1, self.statusPubCb)

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_sdk.spin()
        #########################################################



   
    def initCb(self, do_updates = False):
        if self.node_if is not None:
            self.timezone = nepi_sdk.get_param('timezone')
            nepi_system.set_timezone(self.timezone)
            self.auto_sync_clocks = nepi_sdk.get_param('auto_sync_clocks')
            self.auto_sync_timezones = nepi_sdk.get_param('auto_sync_timezones')
        self.publish_status()
        


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
     

    def check_chrony_process(self):
        if self.in_container == True:
            return False
        else:    
            try:
                # Check for the chronyd process using 'pgrep'
                subprocess.check_output(['sudo',"pgrep", "chronyd"])
                return True
            except subprocess.CalledProcessError:
                # pgrep returns a non-zero exit code if the process is not found
                return False


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



    def restart_chrony(self):
            ### Update ETC Files
            etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_time_ntps.sh"
            subprocess.call([etc_update_script])
            nepi_utils.sleep(1)
            self.nepi_config = self.get_nepi_system_config()
            ####################

    def reset_to_factory_conf(self):
        if os.path.exists(CHRONY_CFG_FACTORY) == True:
            nepi_utils.copy_file(CHRONY_CFG_FACTORY,CHRONY_CFG_BASENAME)
            self.restart_chrony()

    def add_server(self,server_host):
            ### Update ETC Files
            nepi_system.update_nepi_system_config("NEPI_NTP_IP",server_host)
            etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_time_ntps.sh"
            subprocess.call([etc_update_script])
            nepi_utils.sleep(1)
            self.nepi_config = self.get_nepi_system_config()
            ####################

    def remove_server(self,server_host):
            ### Update ETC Files
            nepi_system.update_nepi_system_config("NEPI_NTP_IP","NONE")
            etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_time_ntps.sh"
            subprocess.call([etc_update_script])
            nepi_utils.sleep(1)
            self.nepi_config = self.get_nepi_system_config()
            ####################

    # def gather_ntp_status_timer_cb(self,event):
    #     # Just call the implementation method. We don't care about the event payload
    #     self.gather_ntp_status()


    def updaterCb(self,timer):

        #os.environ['TZ'] = time.strftime('%Z')
        #time.tzset()

        # Get Last PPS time from the sysfs node
        #pps_exists = os.path.isfile('/sys/class/pps/pps0/assert')
        pps_exists = False # Hard code if for now, since Jetson isn't defining /sys/class/pps -- we may never actually use PPS
        if pps_exists:
            pps_string = subprocess.check_output(['sudo',"cat", "/sys/class/pps/pps0/assert"], text=True)
            pps_tokens = pps_string.split('#')
            if (len(pps_tokens) >= 2):
                self.status_msg.last_pps = float(pps_string.split('#')[0])
            else:
                self.status_msg.last_pps = 0.0
                self.msg_if.pub_warn("Unable to parse /sys/class/pps/pps0/assert")
        else: # Failed to find the assert file - just return no PPS
            self.status_msg.last_pps = 0.0


        ntp_status = self.gather_ntp_status()
        for status_entry in ntp_status:
            if status_entry[0] not in self.status_msg.ntp_sources:
                self.status_msg.ntp_sources.append(status_entry[0])
                self.status_msg.currently_syncd.append(status_entry[1])
                self.status_msg.last_ntp_sync.append(status_entry[2])
                self.status_msg.current_offset.append(status_entry[3])
            else:
                ind = self.status_msg.ntp_sources.index(status_entry[0])
                if ind != -1:
                    self.status_msg.currently_syncd[ind] = status_entry[1]
                    self.status_msg.last_ntp_sync[ind] = status_entry[2]
                    self.status_msg.current_offset[ind] = status_entry[3]

        self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)



    def gather_ntp_status(self):
        ntp_status = [] # List of lists
        try:
            chronyc_sources = subprocess.check_output(['sudo',"chronyc", "sources"], text=True).splitlines()
            #self.msg_if.pub_warn("Chrony returned status: " + str(chronyc_sources))
        except Exception as e:
            #self.msg_if.pub_warn("Failed to get Chrony status: " + str(e))
            return ntp_status
        
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
                    self.clock_synced = True
                    self.informClockUpdate()

                    # Update the RTC with this "better" clock source
                    self.msg_if.pub_info("Updating hardware clock with NTP time")
                    subprocess.call(['sudo','hwclock', '-w'])

        return ntp_status


    def set_time(self,msg):
        # TODO: Bounds checking?
        #self.msg_if.pub_warn("Got time update msg: " + str(msg), throttle_s = 5.0)
        update_time = msg.update_time
        if update_time == True and self.manages_time == True:
            self.msg_if.pub_info("Setting time from set_time topic: " + str(msg.secs) + '.' + str(msg.nsecs))
            timestring = '@' + str(float(msg.secs) + (float(msg.nsecs) / float(1e9)))
            try:
                subprocess.call(["sudo","date", "-s", timestring])
                self.last_set_time = float(msg.secs) + float(msg.nsecs)/1000000000
                self.clock_synced = True
                new_date = subprocess.check_output(["date"], text=True)
                self.msg_if.pub_info("Updated date: " + str(new_date))
            except Exception as e:
                self.msg_if.pub_warn("Failed to update time: " + str(e))


        update_timezone = msg.update_timezone
        if update_timezone == True:
            self.msg_if.pub_info("Setting timezone to: " + msg.timezone)
            self.set_timezone(msg.timezone)
            self.node_if.save_config()

        if self.manages_time == True:
            # Update the hardware clock from this "better" clock source; helps with RTC drift
            self.msg_if.pub_info("Updating hardware clock from set_time value")
            subprocess.call(['sudo','hwclock', '-w'])

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
            # if self.manages_time == True:
            #     try:
            #         subprocess.call(['sudo',"timedatectl", "set-timezone", self.timezone])
            #     except:
            #         pass
            nepi_system.set_timezone(self.timezone)
            self.publish_status()
            if self.node_if is not None:
                nepi_sdk.set_param('timezone',self.timezone)




    def autoSyncClocksCb(self,msg):
        self.auto_sync_clocks = msg.data
        self.publish_status()
        if self.node_if is not None:
            nepi_sdk.set_param('auto_sync_clocks',msg.data)

    def autoSyncTimezonesCb(self,msg):
        self.auto_sync_timezones = msg.data
        self.publish_status()
        if self.node_if is not None:
            nepi_sdk.set_param('auto_sync_timezones',msg.data)



    def sysResetCb(self,reset_type = 0):
        if self.chrony_running == False:
            self.msg_if.pub_info("Ignoring NTP, Chrony not running")
        elif Reset.USER_RESET == reset_type:
            # Nothing to do for a User Reset as config file is always up-to-date
            self.msg_if.pub_info("Ignoring NTP user-reset NO-OP")
        elif Reset.FACTORY_RESET == reset_type:
            pass # factoryResetCb called by node_if
        elif Reset.SOFTWARE_RESET == reset_type:
            self.msg_if.pub_info("Executing soft reset for NTP")
            self.restart_chrony()
            nepi_sdk.signal_shutdown('Shutdown by request')
        elif Reset.HARDWARE_RESET == reset_type:
            self.msg_if.pub_info("Executing hard reset for NTP")
            # TODO: Any hardware restart required?
            self.restart_chrony()
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


    def get_status_msg(self):
        current_time = nepi_utils.get_time()
        self.status_msg.current_time = current_time

        self.status_msg.timezone_id = time.strftime('%Z')
        self.status_msg.timezone_description = self.timezone

        tzo = pytz.timezone(self.timezone)
        now = datetime.datetime.now(tzo) # current date and time
        dt_str = now.strftime("%m/%d/%Y,%H:%M:%S")
        self.status_msg.date_str = dt_str.split(',')[0]
        self.status_msg.time_str = dt_str.split(',')[1]
        
             
        #self.msg_if.pub_warn("Returning Time Status response: " + str(self.status_msg))

        #self.msg_if.pub_warn("Handle time: " + str(nepi_utils.get_time() - start_time))

        # Last set time (cheater clock sync method)
        self.status_msg.last_set_time = self.last_set_time
        self.status_msg.clock_synced = self.clock_synced

        self.status_msg.auto_sync_clocks = self.auto_sync_clocks
        self.status_msg.auto_sync_timezones = self.auto_sync_timezones
        return self.status_msg
    

        
    def handle_time_status_query(self,req):
        status_msg = self.get_status_msg()
        timezones = nepi_utils.standard_timezones_dict.keys()
        return  { 'time_status': status_msg, 'available_timezones': timezones }


    def statusPubCb(self,timer):
        self.publish_status()

    def publish_status(self):
        status_msg = self.get_status_msg()
        if self.node_if is not None:
            #self.msg_if.pub_warn("Publishing Status Msg: " + str(status_msg))
            self.node_if.publish_pub('status_pub', status_msg)
    


if __name__ == '__main__':
    time_sync_mgr()
