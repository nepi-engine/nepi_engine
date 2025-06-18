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
import time
import shutil
from collections import deque
import re
import datetime
import subprocess
import importlib

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_states
from nepi_sdk import nepi_triggers

import nepi_sdk.nepi_software_update_utils as sw_update_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import SystemStatus, SystemDefs, WarningFlags, StampedString, SaveDataStatus
from nepi_interfaces.srv import SystemDefsQuery, SystemDefsQueryRequest, SystemDefsQueryResponse, \
                             OpEnvironmentQuery, OpEnvironmentQueryRequest, OpEnvironmentQueryResponse, \
                             SystemSoftwareStatusQuery, SystemSoftwareStatusQueryRequest, SystemSoftwareStatusQueryResponse, \
                             SystemStorageFolderQuery, SystemStorageFolderQueryRequest, SystemStorageFolderQueryResponse, \
                             DebugQuery, DebugQueryRequest, DebugQueryResponse, \
                            SystemStatusQuery, SystemStatusQueryRequest, SystemStatusQueryResponse


from nepi_interfaces.msg import SystemTrigger, SystemTriggersStatus
from nepi_interfaces.srv import SystemTriggersQuery, SystemTriggersQueryRequest, SystemTriggersQueryResponse

from nepi_interfaces.msg import SystemState, SystemStatesStatus
from nepi_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import StatesIF, TriggersIF


BYTES_PER_MEGABYTE = 2**20


class SystemMgrNode():
    STATUS_PERIOD = 1.0  # TODO: Configurable update period?

    DISK_FULL_MARGIN_MB = 250  # MB TODO: Configurable?

    SYS_ENV_PATH = "/opt/nepi/sys_env.bash"
    FW_VERSION_PATH = "/opt/nepi/ros/etc/fw_version.txt"

    STATES_DICT = dict()

    node_if = None
    status_msg = SystemStatus()
    system_defs_msg = SystemDefs()

    storage_mountpoint = "/mnt/nepi_storage"
    data_folder = storage_mountpoint + "/data"

    storage_uid = 1000 # default to nepi
    storage_gid = 130 # default to "sambashare" # TODO This is very fragile


    check_ignore_folders = ["data","logs","logs/automation_script_logs","nepi_src","tmp"]

    REQD_STORAGE_SUBDIRS = ["ai_models", 
                            "ai_training",
                            "automation_scripts", 
                            "automation_scripts/sys_trigger_scripts",
                            "automation_scripts/sys_state_scripts",
                            "data", 
                            "databases", 
                            "install",
                            "install/apps",
                            "install/ai_frameworks",
                            "install/drivers",
                            "license", 
                            "logs", 
                            "logs/ros_log",
                            "logs/automation_script_logs", 
                            "nepi_full_img", 
                            "nepi_full_img_archive", 
                            "nepi_src",
                            "user_cfg",
                            "user_cfg/ros",
                            "sample_data",
                            "tmp"]
                            
    REQD_STORAGE_SUBDIRS_CN = ["ai_models",
                            "ai_training", 
                            "automation_scripts", 
                            "data", 
                            "databases", 
                            "install",
                            "install/apps",
                            "install/ai_frameworks",
                            "install/drivers",
                            "license", 
                            "logs", 
                            "logs/ros_log",
                            "logs/automation_script_logs", 
                            "nepi_src",
                            "user_cfg",
                            "user_cfg/ros",
                            "sample_data",
                            "tmp"]
    
    CATKIN_TOOLS_PATH = '/opt/nepi/ros/.catkin_tools'
    SDK_SHARE_PATH = '/opt/nepi/ros/lib/python3/dist-packages/nepi_sdk'
    API_SHARE_PATH = '/opt/nepi/ros/lib/python3/dist-packages/nepi_api'
    DRIVERS_SHARE_PATH = '/opt/nepi/ros/lib/nepi_drivers'
    APPS_SHARE_PATH = '/opt/nepi/ros/share/nepi_apps'
    AIFS_SHARE_PATH = '/opt/nepi/ros/share/nepi_aifs'

    # disk_usage_deque = deque(maxlen=10)
    # Shorter period for more responsive updates
    disk_usage_deque = deque(maxlen=3)

    first_stage_rootfs_device = "/dev/mmcblk0p1"
    nepi_storage_device = "/dev/nvme0n1p3"
    new_img_staging_device = "/dev/nvme0n1p3"
    new_img_staging_device_removable = False
    usb_device = "/dev/sda" 
    sd_card_device = "/dev/mmcblk1p"
    emmc_device = "/dev/mmcblk0p"
    ssd_device = "/dev/nvme0n1p"
    auto_switch_rootfs_on_new_img_install = True
    sw_update_progress = ""

    installing_new_image = False
    new_img_file = ""
    new_img_version = ""
    new_img_filesize = ""
    install_status = True
    archiving_inactive_image = False

    in_container = False

    triggers_list = []
    triggers_status_interval = 1.0

    states_status_interval = 1.0

    current_throttle_ratio = 1.0

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "system_mgr" # Can be overwitten by luanch command
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


        ###############################
        # Initialize Class Variables
        self.msg_if.pub_warn("Getting System Info")
        self.first_stage_rootfs_device = nepi_sdk.get_param(
            "~first_stage_rootfs_device", self.first_stage_rootfs_device)

        if self.first_stage_rootfs_device == "container":
            self.in_container = True
        self.system_defs_msg.in_container = self.in_container
        self.status_msg.in_container = self.in_container
        

        if self.in_container == False:
          self.req_storage_subdirs = self.REQD_STORAGE_SUBDIRS
        else:
          self.req_storage_subdirs = self.REQD_STORAGE_SUBDIRS_CN
        
        self.system_defs_msg.inactive_rootfs_fw_version = "uknown"
        '''
        self.msg_if.pub_warn("Deleting old log files")
        logs_path_subdir = os.path.join(self.storage_mountpoint, 'logs/ros_log')
        os.system('rm -r ' + logs_path_subdir + '/*')
        '''

        self.msg_if.pub_warn("Mounting Storage Drive")
        # Need to get the storage_mountpoint and first-stage rootfs early because they are used in init_msgs()
        self.storage_mountpoint = nepi_sdk.get_param(
            "~storage_mountpoint", self.storage_mountpoint)
        
        self.msg_if.pub_warn("Updating Rootfs Scheme")
        # Need to identify the rootfs scheme because it is used in init_msgs()
        self.rootfs_ab_scheme = sw_update_utils.identifyRootfsABScheme()
        self.init_msgs()

        self.msg_if.pub_warn("Checking User Storage Partition")
        # First check that the storage partition is actually mounted
        if not os.path.ismount(self.storage_mountpoint):
           self.msg_if.pub_warn("NEPI Storage partition is not mounted... attempting to mount")
           ret, msg = sw_update_utils.mountPartition(self.nepi_storage_device, self.storage_mountpoint)
           if ret is False:
               self.msg_if.pub_warn("Unable to mount NEPI Storage partition... system may be dysfunctional")
               #return False # Allow it continue on local storage...

        # ... as long as there is enough space
        self.update_storage()
        if self.status_msg.warnings.flags[WarningFlags.DISK_FULL] is True:
            self.msg_if.pub_warn("Insufficient space on storage partition")
            self.storage_mountpoint = ""
            return False

        # Gather owner and group details for storage mountpoint
        stat_info = os.stat(self.storage_mountpoint)
        self.storage_uid = stat_info.st_uid
        self.storage_gid = stat_info.st_gid


        self.msg_if.pub_warn("Checking System Folders")
        # Ensure that the user partition is properly laid out
        self.storage_subdirs = {} # Populated in function below
        if self.ensure_reqd_storage_subdirs() is True:
            # Now can advertise the system folder query
            nepi_sdk.create_service('system_storage_folder_query', SystemStorageFolderQuery,
                self.provide_system_data_folder)



        self.msg_if.pub_warn("Checking valid device id")
        self.valid_device_id_re = re.compile(r"^[a-zA-Z][\w]*$")



        if self.in_container == False:
            self.msg_if.pub_warn("Updating Rootfs Load Fail Counter")
            # Reset the A/B rootfs boot fail counter -- if this node is running, pretty safe bet that we've booted successfully
            # This should be redundant, as we need a non-ROS reset mechanism, too, in case e.g., ROS nodes are delayed waiting
            # for a remote ROS master to start. That could be done in roslaunch.sh or a separate start-up script.
            if self.rootfs_ab_scheme == 'nepi': # The 'jetson' scheme handles this itself
                status, err_msg = sw_update_utils.resetBootFailCounter(
                    self.first_stage_rootfs_device)
                if status is False:
                    self.msg_if.pub_warn("Failed to reset boot fail counter: " + err_msg)


        self.msg_if.pub_warn("Starting Node IF Setup")    
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
            'op_environment': {
                'namespace': self.base_namespace,
                'factory_val': OpEnvironmentQueryResponse.OP_ENV_AIR
            },
            'storage_mountpoint': {
                'namespace': self.base_namespace,
                'factory_val': self.storage_mountpoint
            },
            'auto_switch_rootfs_on_new_img_install': {
                'namespace': self.base_namespace,
                'factory_val': self.auto_switch_rootfs_on_new_img_install
            },
            'first_stage_rootfs_device': {
                'namespace': self.base_namespace,
                'factory_val': self.first_stage_rootfs_device
            },
            'new_img_staging_device': {
                'namespace': self.base_namespace,
                'factory_val': self.new_img_staging_device
            },
            'new_img_staging_device_removable': {
                'namespace': self.base_namespace,
                'factory_val': self.new_img_staging_device_removable
            },
            'emmc_device': {
                'namespace': self.base_namespace,
                'factory_val': self.emmc_device
            },
            'usb_device': {
                'namespace': self.base_namespace,
                'factory_val': self.usb_device
            },
            'sd_card_device': {
                'namespace': self.base_namespace,
                'factory_val': self.sd_card_device
            },
            'ssd_device': {
                'namespace': self.base_namespace,
                'factory_val': self.ssd_device
            },
            'debug_enabled': {
                'namespace': self.base_namespace,
                'factory_val': False
            }
        }

    
        # Services Config Dict ####################
        self.SRVS_DICT = {
            'system_defs_query': {
                'namespace': self.base_namespace,
                'topic': 'system_defs_query',
                'srv': SystemDefsQuery,
                'req': SystemDefsQueryRequest(),
                'resp': SystemDefsQueryResponse(),
                'callback': self.provide_system_defs
            },
            'op_environment_query': {
                'namespace': self.base_namespace,
                'topic': 'op_environment_query',
                'srv': OpEnvironmentQuery,
                'req': OpEnvironmentQueryRequest(),
                'resp': OpEnvironmentQueryResponse(),
                'callback': self.provide_op_environment
            },
            'sw_update_status_query': {
                'namespace': self.base_namespace,
                'topic': 'sw_update_status_query',
                'srv': SystemSoftwareStatusQuery,
                'req': SystemSoftwareStatusQueryRequest(),
                'resp': SystemSoftwareStatusQueryResponse(),
                'callback': self.provide_sw_update_status
            },
            'debug_query': {
                'namespace': self.base_namespace,
                'topic': 'debug_mode_query',
                'srv': DebugQuery,
                'req': DebugQueryRequest(),
                'resp': DebugQueryResponse(),
                'callback': self.provide_debug_status
            },
            'status_query': {
                'namespace': self.base_namespace,
                'topic': 'system_status_query',
                'srv': SystemStatusQuery,
                'req': SystemStatusQueryRequest(),
                'resp': SystemStatusQueryResponse(),
                'callback': self.provide_system_status
            }
        }



        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.base_namespace,
                'topic': 'system_status',
                'msg': SystemStatus,
                'qsize': 1,
                'latch': True
            },
            'store_params': {
                'namespace': self.base_namespace,
                'topic': 'store_params',
                'msg': String,
                'qsize': 10,
                'latch': True
            },
            'apply_throttle': {
                'namespace': self.base_namespace,
                'topic': 'apply_throttle',
                'msg': Float32,
                'qsize': 3,
                'latch': True
            },
            'set_op_environment': {
                'namespace': self.base_namespace,
                'topic': 'settings',
                'msg': String,
                'qsize': 3,
                'latch': True
            },
            'save_data_pub': {
                'namespace': self.base_namespace,
                'topic': 'save_data',
                'msg': Bool,
                'qsize': 1,
                'latch': True
            },
            'system_triggers': {
                'namespace': self.base_namespace,
                'topic': 'system_triggers',
                'msg': SystemTrigger,
                'qsize': 1,
                'latch': True
            },
            'triggers_status_pub': {
                'namespace': self.base_namespace,
                'topic': 'system_triggers_status',
                'msg': SystemTriggersStatus,
                'qsize': 1,
                'latch': True
            },
            'states_status_pub': {
                'namespace': self.base_namespace,
                'topic': 'system_states_status',
                'msg': SystemStatesStatus,
                'qsize': 1,
                'latch': True
            },
            'debug_pub': {
                'namespace': self.base_namespace,
                'topic': 'debug_mode',
                'msg': Bool,
                'qsize': 1,
                'latch': True
            }
        }  

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'save_data': {
                'namespace': self.base_namespace,
                'topic': 'save_data',
                'msg': Bool,
                'qsize': None,
                'callback': self.set_save_status, 
                'callback_args': ()
            },
            'clear_data_folder': {
                'namespace': self.base_namespace,
                'topic': 'clear_data_folder',
                'msg': Empty,
                'qsize': None,
                'callback': self.clear_data_folder, 
                'callback_args': ()
            },
            'set_op_environment': {
                'namespace': self.base_namespace,
                'topic': 'set_op_environment',
                'msg': String,
                'qsize': None,
                'callback': self.set_op_environment, 
                'callback_args': ()
            },
            'set_device_id': {
                'namespace': self.base_namespace,
                'topic': 'set_device_id',
                'msg': String,
                'qsize': None,
                'callback': self.set_device_id, 
                'callback_args': ()
            },        
            'submit_system_error_msg': {
                'namespace': self.base_namespace,
                'topic': 'submit_system_error_msg',
                'msg': String,
                'qsize': None,
                'callback': self.handle_system_error_msg, 
                'callback_args': ()
            },
            'install_new_image': {
                'namespace': self.base_namespace,
                'topic': 'install_new_image',
                'msg': String,
                'qsize': 1,
                'callback': self.handle_install_new_img, 
                'callback_args': ()
            },
            'switch_active_inactive_rootfs': {
                'namespace': self.base_namespace,
                'topic': 'switch_active_inactive_rootfs',
                'msg': Empty,
                'qsize': None,
                'callback': self.handle_switch_active_inactive_rootfs, 
                'callback_args': ()
            },
            'archive_inactive_rootfs': {
                'namespace': self.base_namespace,
                'topic': 'archive_inactive_rootfs',
                'msg': Empty,
                'qsize': 1,
                'callback': self.handle_archive_inactive_rootfs, 
                'callback_args': ()
            },
            'save_data_prefix': {
                'namespace': self.base_namespace,
                'topic': 'save_data_prefix',
                'msg': String,
                'qsize': None,
                'callback': self.save_data_prefix_callback, 
                'callback_args': ()
            },
            'system_triggers': {
                'namespace': self.base_namespace,
                'topic': 'system_triggers',
                'msg': SystemTrigger,
                'qsize': None,
                'callback': self.system_triggers_callback, 
                'callback_args': ()
            },
            'enable_debug': {
                'namespace': self.base_namespace,
                'topic': 'debug_mode_enable',
                'msg': Bool,
                'qsize': None,
                'callback': self.enable_debug_callback, 
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


        self.msg_if.pub_warn("Starting System IF Setup")   
        #######################
        # Setup System IF Classes
        
        # Setup States IF
        self.STATES_DICT = {
                        "in_container": {
                            "name":"in_container",
                            "node_name": self.node_name,
                            "description": "NEPI running in container",
                            "type":"Bool",
                            "options": [],
                            "value": str(self.in_container)
                            }
        }
        self.sys_if_states = StatesIF(self.getStatesDictCb,
                        msg_if = self.msg_if)
        time.sleep(1)


        self.msg_if.pub_warn("Completing Initialization Processes") 
        ########################
        # Complete Initialization

        # Call the method to update s/w status once internally to prime the status fields now that we have all the parameters
        # established
        self.provide_sw_update_status(0) # Any argument is fine here as the req. field is unused
               

        # Create Triggers Status Pub Processes
        self.triggers_status_interval = 1.0

        self.msg_if.pub_info(":" + self.class_name + ": Starting triggers status pub service: ")
        #nepi_sdk.start_timer_process(self.triggers_status_interval, self.triggersStatusPubCb, oneshot = True)

        # Create States Status Pub Processes
        self.states_status_interval = 1.0

        self.msg_if.pub_info(":" + self.class_name + ": Starting states status pub service: ")
        nepi_sdk.start_timer_process(self.states_status_interval, self.statesStatusPubCb, oneshot = True)

        self.status_msg.sys_debug_enabled = self.node_if.get_param('debug_enabled')

    
        # Want to update the op_environment (from param server) through the whole system once at
        # start-up, but the only reasonable way to do that is to delay long enough to let all nodes start
        self.msg_if.pub_warn("Updating From Param Server")
        self.initConfig()
    


        # Crate system status pub
        self.msg_if.pub_warn("Starting System Status Messages")
        nepi_sdk.start_timer_process(self.STATUS_PERIOD, self.publish_status)
        self.msg_if.pub_warn("System status ready")
        #########################################################
        ## Initiation Complete
        self.msg_if.pub_warn("Initialization Complete")
        nepi_sdk.spin()



    

    def initConfig(self):
        if self.node_if is not None:
            op_env = self.node_if.get_param("op_environment")
            # Publish it to all subscribers (which includes this node) to ensure the parameter is applied
            self.node_if.publish_pub('set_op_env_pub', String(op_env))

            # Now gather all the params and set members appropriately
            self.storage_mountpoint = self.node_if.get_param("storage_mountpoint")
            
            self.auto_switch_rootfs_on_new_img_install = nepi_sdk.get_param(
                "~auto_switch_rootfs_on_new_img_install", self.auto_switch_rootfs_on_new_img_install)

            self.first_stage_rootfs_device = nepi_sdk.get_param(
                "~first_stage_rootfs_device", self.first_stage_rootfs_device)

            # nepi_storage_device has some additional logic
            self.getNEPIStorageDevice()
            
            self.new_img_staging_device = nepi_sdk.get_param(
                "~new_img_staging_device", self.new_img_staging_device)

            self.new_img_staging_device_removable = nepi_sdk.get_param(
                "~new_img_staging_device_removable", self.new_img_staging_device_removable)

            self.emmc_device = self.node_if.get_param("emmc_device")

            self.usb_device = self.node_if.get_param("usb_device")

            self.sd_card_device = self.node_if.get_param("sd_card_device")

            self.ssd_device = self.node_if.get_param("ssd_device")
    
    def initCb(self, do_updates = False):
        pass

    def resetCb(self):
        pass

    def factoryResetCb(self):
        pass



    def getStatesDictCb(self):
        return self.STATES_DICT

    def system_triggers_callback(self,msg):
        trigger_name = msg.name
        if trigger_name not in self.triggers_list:
            self.triggers_list.append(trigger_name)

    def triggersStatusPubCb(self,timer):
        triggers_name_list = []
        has_triggered_list = []
        msg = SystemTriggersStatus()
        namespaces = nepi_triggers.get_triggers_publisher_namespaces()
        if namespaces is not None:
            for namespace in namespaces:
                topic = os.path.join(namespace,'system_triggers_query')
                if topic not in self.service_dict.keys():
                    service = nepi_sdk.create_service(topic,SystemTrigger)
                    if service is not None:
                        self.service_dict[topic] = service
                        time.sleep(1)
                if topic in self.service_dict.keys():
                    service = self.service_dict[topic]
                    req = SystemTriggersQueryRequest()
                    try:
                        resp = nepi_sdk.call_service(service, req)
                        triggers_list = resp.triggers_list
                        for trigger in triggers_list:
                            trigger_name = trigger.name
                            if trigger_name not in triggers_name_list:
                                triggers_name_list.append(trigger_name) 
                    except:
                        self.msg_if.pub_info(":" + self.class_name + ": Failed to call service: " + str(e))

            for trigger_name in triggers_name_list:
                has_triggered = trigger_name in self.triggers_list
                has_triggered_list.append(has_triggered)
            self.triggers_list = [] # Clear List
            msg = nepi_triggers.create_triggers_status_msg(triggers_name_list,has_triggered_list)
            if self.node_if is not None:
                self.node_if.publish_pub('triggers_status_pub', msg)
        nepi_sdk.start_timer_process(self.triggers_status_interval, self.triggersStatusPubCb, oneshot = True)



    def statesStatusPubCb(self,timer):
        states_list = []
        msg = SystemStatesStatus()
        namespaces = nepi_states.get_states_publisher_namespaces()
        if namespaces is not None:
            for namespace in namespaces:
                topic = os.path.join(namespace,'system_states_query')
                if topic not in self.service_dict.keys():
                    service = nepi_sdk.create_service(topic,SystemState)
                    if service is not None:
                        self.service_dict[topic] = service
                        time.sleep(1)
                if topic in self.service_dict.keys():
                    service = self.service_dict[topic]
                    req = SystemStatesQueryRequest()
                    try:
                        resp = nepi_sdk.call_service(service, req)
                        for state in resp.states_list:
                            states_list.append(state)
                    except:
                        self.msg_if.pub_info(":" + self.class_name + ": Failed to call service: " + str(e))
            try:
                msg = nepi_states.create_states_status_msg(states_list)
            except:
                self.msg_if.pub_info(":" + self.class_name + ": Failed to create status msg: " + str(e))
            if self.node_if is not None:
                self.node_if.publish_pub('states_status_pub', msg)
        nepi_sdk.start_timer_process(self.states_status_interval, self.statesStatusPubCb, oneshot = True)


    def add_info_string(self, string, level):
        self.status_msg.info_strings.append(StampedString(
            timestamp=nepi_sdk.get_msg_stamp(), payload=string, priority=level))

    def get_device_sn(self):
        with open(self.SYS_ENV_PATH, "r") as f:
            for line in f:
                if line.startswith("export DEVICE_SN="):
                    return line.split('=')[1].rstrip()
        return "undefined"

    def get_fw_rev(self):
        if (os.path.exists(self.FW_VERSION_PATH) and (os.path.getsize(self.FW_VERSION_PATH) > 0)):
            with open(self.FW_VERSION_PATH, "r") as f:
                return f.readline().strip()
        return "UNSPECIFIED"
    
    def get_device_type(self):
        with open(self.SYS_ENV_PATH, "r") as f:
            for line in f:
                if line.startswith("export DEVICE_TYPE="):
                    return line.split('=')[1].rstrip()
        return "undefined"            

    def update_temperatures(self):
        # Get the current temperatures

        # TODO: Should the temperature sensor or the entire subproc. cmd line be configurable?
        temp_string_mdegC = subprocess.check_output(
            ["cat", "/sys/class/thermal/thermal_zone0/temp"])
        self.status_msg.temperatures[0] = float(temp_string_mdegC) / 1000.0

        # Check for temperature warnings and do thermal throttling
        throttle_ratio_min = 1.0
        for i, t in enumerate(self.status_msg.temperatures):
            if (t > self.system_defs_msg.critical_temps[i]):
                # Critical implies high
                self.status_msg.warnings.flags[WarningFlags.CRITICAL_TEMPERATURE] = True
                self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = True
                # Make sure to send a user string
                self.add_info_string(
                    WarningFlags.CRITICAL_TEMPERATURE_STRING, StampedString.PRI_HIGH)
                # Set the throttle ratio to 0% globally
                self.msg_if.log_msg_warn("temperature: " + str(WarningFlags.CRITICAL_TEMPERATURE_STRING) + " " + str(t))
                throttle_ratio_min = 0.0
            else:
                self.status_msg.warnings.flags[WarningFlags.CRITICAL_TEMPERATURE] = False
                if (t > self.system_defs_msg.warning_temps[i]):
                    self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = True
                    throttle_ratio_i = 1.0 - ((t - self.system_defs_msg.warning_temps[i]) / (
                        self.system_defs_msg.critical_temps[i] - self.system_defs_msg.warning_temps[i]))
                    throttle_ratio_min = min(
                        throttle_ratio_i, throttle_ratio_min)
                else:
                    self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = False
            # If a new thermal throttle ratio was computed, publish it globally
            if (throttle_ratio_min != self.current_throttle_ratio):
                if self.node_if is not None:
                    self.node_if.publish_pub('throttle_ratio_pub', Float32(throttle_ratio_min))
                self.current_throttle_ratio = throttle_ratio_min
                #self.msg_if.pub_warn("New thermal rate throttle value: %f%%", self.current_throttle_ratio)

    def update_storage(self):
        # Data partition
        try:
            statvfs = os.statvfs(self.storage_mountpoint)
        except Exception as e:
            warn_str = "Error checking data storage status of " + self.storage_mountpoint + ": " + e.what()
            self.msg_if.pub_warn(warn_str)
            self.add_info_string("warn_str")
            self.status_msg.disk_usage = 0
            self.storage_rate = 0
            return

        disk_free = float(statvfs.f_frsize) * \
            statvfs.f_bavail / BYTES_PER_MEGABYTE  # In MB
        self.status_msg.disk_usage = self.system_defs_msg.disk_capacity - disk_free

        self.disk_usage_deque.append(self.status_msg.disk_usage)
        self.status_msg.storage_rate = (
            self.disk_usage_deque[-1] - self.disk_usage_deque[0]) / (len(self.disk_usage_deque)*self.STATUS_PERIOD)

        # Check for disk warnings
        if (disk_free < self.DISK_FULL_MARGIN_MB):
            self.status_msg.warnings.flags[WarningFlags.DISK_FULL] = True
            self.add_info_string("Max disk usage exceeded",
                                 StampedString.PRI_HIGH)
            # Force all nodes to stop data saving
            if self.node_if is not None:
                self.node_if.publish_pub('save_data_pub', False)
        else:
            self.status_msg.warnings.flags[WarningFlags.DISK_FULL] = False

    def provide_sw_update_status(self, req):
        resp = SystemSoftwareStatusQueryResponse()
        resp.new_sys_img_staging_device = self.get_device_friendly_name(self.new_img_staging_device)
        resp.new_sys_img_staging_device_free_mb = sw_update_utils.getPartitionFreeByteCount(self.new_img_staging_device) / BYTES_PER_MEGABYTE

        # Don't query anything if we are in the middle of installing a new image
        if self.installing_new_image:
            resp.new_sys_img = self.new_img_file
            resp.new_sys_img_version = self.new_img_version
            resp.new_sys_img_size_mb = self.new_img_filesize / BYTES_PER_MEGABYTE
            return resp
        
        # At this point, not currently installing, so clear any previous query failed message so the status update logic below will work
        self.status_msg.sys_img_update_status = ""

        (status, err_string, self.new_img_file, self.new_img_version, self.new_img_filesize) = sw_update_utils.checkForNewImageAvailable(
            self.new_img_staging_device, self.new_img_staging_device_removable)
        if status is False:
            self.msg_if.pub_warn("Unable to update software status: " + err_string)
            resp.new_sys_img = 'query failed'
            resp.new_sys_img_version = 'query failed'
            resp.new_sys_img_size_mb = 0
            self.status_msg.sys_img_update_status = 'query failed'
            return resp
        
        # Update the response
        if self.new_img_file:
            resp.new_sys_img = self.new_img_file
            resp.new_sys_img_version = self.new_img_version
            resp.new_sys_img_size_mb = self.new_img_filesize / BYTES_PER_MEGABYTE
            self.status_msg.sys_img_update_status = "ready to install"
        else:
            resp.new_sys_img = 'none detected'
            resp.new_sys_img_version = 'none detected'
            resp.new_sys_img_size_mb = 0
            self.status_msg.sys_img_update_status = "no new image available"
                
        return resp
    
    def provide_system_data_folder(self, req):
        response = SystemStorageFolderQueryResponse()
        if req.type not in self.storage_subdirs:
            response.folder_path = ''
        else:
            response.folder_path = self.storage_subdirs[req.type]
        return response

    def provide_driver_folder(self, req):
        return self.DRIVERS_SHARE_PATH

    def provide_debug_status(self, req):
        response = DebugQueryResponse()
        response.degug_enabled = self.status_msg.sys_debug_enabled
        return response

    def provide_system_status(self, req):
        response = SystemStatusQueryResponse()
        response.system_status = self.status_msg
        #self.msg_if.pub_warn("Returning status query response: " + str(response))
        return response

    def publish_status(self, event):
        # Populate the rest of the message contents
        # Temperature(s)
        self.update_temperatures()

        # Disk usage
        self.update_storage()

        # Now publish it
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', self.status_msg)
            self.node_if.publish_pub('debug_pub', self.status_msg.sys_debug_enabled)

        # Always clear info strings after publishing
        del self.status_msg.info_strings[:]

    def set_save_status(self, save_msg):
        self.status_msg.save_all_enabled = save_msg.data

    def enable_debug_callback(self, msg):
        self.status_msg.sys_debug_enabled = msg.data
        if self.node_if is not None:
            self.node_if.set_param('debug_enabled',msg.data)
            self.node_if.save_config()

    def ensure_reqd_storage_subdirs(self):
        # Check for and create subdirectories as necessary
        self.msg_if.pub_warn("Checking user storage partition folders")
        for subdir in self.req_storage_subdirs:
            full_path_subdir = os.path.join(self.storage_mountpoint, subdir)
            if not os.path.isdir(full_path_subdir):
                self.msg_if.pub_warn("Required storage subdir " + subdir + " not present... will create")
                os.makedirs(full_path_subdir)
                # And set the owner:group and permissions. Do this every time to fix bad settings e.g., during SSD setup
                # TODO: Different owner:group for different folders?
            if subdir not in self.check_ignore_folders:
                self.msg_if.pub_warn("Checking user storage partition folder permissions: " + subdir)
                os.system('chown -R ' + str(self.storage_uid) + ':' + str(self.storage_gid) + ' ' + full_path_subdir) # Use os.system instead of os.chown to have a recursive option
                #os.chown(full_path_subdir, self.storage_uid, self.storage_gid)
                os.system('chmod -R 0775 ' + full_path_subdir)
            self.storage_subdirs[subdir] = full_path_subdir



        # Check system folders
        self.msg_if.pub_warn("Checking nepi_sdk share folder")
        if not os.path.isdir(self.SDK_SHARE_PATH):
                self.msg_if.pub_warn("Driver folder " + self.SDK_SHARE_PATH + " not present... will create")
                os.makedirs(self.SDK_SHARE_PATH)
        os.system('chown -R ' + str(self.storage_uid) + ':' + str(self.storage_gid) + ' ' + self.SDK_SHARE_PATH) # Use os.system instead of os.chown to have a recursive option
        os.system('chmod -R 0775 ' + self.SDK_SHARE_PATH)
        self.storage_subdirs['sdk'] = self.SDK_SHARE_PATH

        self.msg_if.pub_warn("Checking nepi_api share folder")
        if not os.path.isdir(self.API_SHARE_PATH):
                self.msg_if.pub_warn("Driver folder " + self.API_SHARE_PATH + " not present... will create")
                os.makedirs(self.API_SHARE_PATH)
        os.system('chown -R ' + str(self.storage_uid) + ':' + str(self.storage_gid) + ' ' + self.API_SHARE_PATH) # Use os.system instead of os.chown to have a recursive option
        os.system('chmod -R 0775 ' + self.API_SHARE_PATH)
        self.storage_subdirs['api'] = self.API_SHARE_PATH



        self.msg_if.pub_warn("Checking nepi_drivers lib folder")
        if not os.path.isdir(self.DRIVERS_SHARE_PATH):
                self.msg_if.pub_warn("Driver folder " + self.DRIVERS_SHARE_PATH + " not present... will create")
                os.makedirs(self.DRIVERS_SHARE_PATH)
        os.system('chown -R ' + str(self.storage_uid) + ':' + str(self.storage_gid) + ' ' + self.DRIVERS_SHARE_PATH) # Use os.system instead of os.chown to have a recursive option
        os.system('chmod -R 0775 ' + self.DRIVERS_SHARE_PATH)
        self.storage_subdirs['drivers'] = self.DRIVERS_SHARE_PATH

        self.msg_if.pub_warn("Checking nepi_apps param folder")
        if not os.path.isdir(self.APPS_SHARE_PATH):
                self.msg_if.pub_warn("Apps folder " + self.APPS_SHARE_PATH + " not present... will create")
                os.makedirs(self.APPS_SHARE_PATH)
        os.system('chown -R ' + str(self.storage_uid) + ':' + str(self.storage_gid) + ' ' + self.APPS_SHARE_PATH) # Use os.system instead of os.chown to have a recursive option
        os.system('chmod -R 0775 ' + self.APPS_SHARE_PATH)
        self.storage_subdirs['apps'] = self.APPS_SHARE_PATH

        self.msg_if.pub_warn("Checking nepi_aifs param folder")
        if not os.path.isdir(self.AIFS_SHARE_PATH):
                self.msg_if.pub_warn("AIF folder " + self.AIFS_SHARE_PATH + " not present... will create")
                os.makedirs(self.AIFS_SHARE_PATH)
        os.system('chown -R ' + str(self.storage_uid) + ':' + str(self.storage_gid) + ' ' + self.AIFS_SHARE_PATH) # Use os.system instead of os.chown to have a recursive option
        os.system('chmod -R 0775 ' + self.AIFS_SHARE_PATH)
        self.storage_subdirs['aifs'] = self.AIFS_SHARE_PATH
        return True

    def clear_data_folder(self, msg):
        if (self.status_msg.save_all_enabled is True):
            self.msg_if.pub_warn(
                "Refusing to clear data folder because data saving is currently enabled")
            return

        self.msg_if.pub_info("Clearing data folder by request")
        data_folder = self.storage_subdirs['data']
        if not os.path.isdir(data_folder):
            self.msg_if.pub_warn(
                "No such folder " + data_folder + "... nothing to clear"
            )
            return

        for filename in os.listdir(data_folder):
            file_path = os.path.join(data_folder, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                self.msg_if.pub_warn('Failed to delete: ' + file_path + " " + str(e))

    def set_op_environment(self, msg):
        if (msg.data != OpEnvironmentQueryResponse.OP_ENV_AIR) and (msg.data != OpEnvironmentQueryResponse.OP_ENV_WATER):
            self.msg_if.pub_warn(
                "Setting environment parameter to a non-standard value: " + str(msg.data))
        nepi_sdk.set_param("~op_environment", msg.data)

    def set_device_id(self, msg):
        # First, validate the characters in the msg as namespace chars -- blank string is okay here to clear the value
        if (msg.data) and (not self.valid_device_id_re.match(msg.data)):
            self.msg_if.pub_warn("Invalid device ID: " +  str(msg.data))
            return

        # Otherwise, overwrite the DEVICE_ID in sys_env.bash
        file_lines = []
        with open(self.SYS_ENV_PATH, "r") as f:
            for line in f:
                if line.startswith("export DEVICE_ID"):
                    file_lines.append("export DEVICE_ID=" + msg.data + '\n')
                else:
                    file_lines.append(line)
        tmp_filename = self.SYS_ENV_PATH + ".tmp"
        with open(tmp_filename, "w") as f:
            for line in file_lines:
                f.write(line)

        # Now overwrite the original file as an autonomous operation
        os.rename(tmp_filename, self.SYS_ENV_PATH)
        self.msg_if.pub_warn("Device ID Updated - Requires device reboot")
        self.add_info_string(
            "Device ID updated - Requires device reboot", StampedString.PRI_ELEVATED)

    def handle_system_error_msg(self, msg):
        self.add_info_string(msg.data, StampedString.PRI_HIGH)

    def receive_sw_update_progress(self, progress_val):
        self.status_msg.sys_img_update_progress = progress_val
        if progress_val > 0 and progress_val < 1:
            self.status_msg.sys_img_update_status = 'flashing'

    def receive_archive_progress(self, progress_val):
        self.status_msg.sys_img_archive_progress = progress_val
    
    def handle_install_new_img(self, msg):
        if self.installing_new_image:
            self.msg_if.pub_warn("New image is already being installed")
            return

        decompressed_img_filename = msg.data
        self.status_msg.sys_img_update_status = 'flashing'
        self.installing_new_image = True
        self.install_status = True
        self.install_status, err_msg = sw_update_utils.writeImage(self.new_img_staging_device, decompressed_img_filename, self.inactive_rootfs_device, 
                                                     do_slow_transfer=False, progress_cb=self.receive_sw_update_progress)

        # Finished installing
        self.installing_new_image = False
        if self.install_status is False:
            self.msg_if.pub_warn("Failed to flash image: " + err_msg)
            self.status_msg.sys_img_update_status = 'failed'
            return
        else:
            self.msg_if.pub_info("Finished flashing new image to inactive partition")
            self.status_msg.sys_img_update_status = 'complete - needs rootfs switch and reboot'

        # Check and repair the newly written filesystem as necessary
        self.install_status, err_msg = sw_update_utils.checkAndRepairPartition(self.inactive_rootfs_device)
        if self.install_status is False:
            self.msg_if.pub_warn("Newly flashed image has irrepairable filesystem issues: ", err_msg)
            self.status_msg.sys_img_update_status = 'failed - fs errors'
            return
        else:
            self.msg_if.pub_info("New image filesystem checked and repaired (as necessary)")

        # Do automatic rootfs switch if so configured
        if self.auto_switch_rootfs_on_new_img_install:
            if self.rootfs_ab_scheme == 'nepi':
                status, err_msg = sw_update_utils.switchActiveAndInactivePartitions(self.first_stage_rootfs_device)
            elif self.rootfs_ab_scheme == 'jetson':
                status, err_msg = sw_update_utils.switchActiveAndInactivePartitionsJetson()
            else:
                err_msg = "Unknown ROOTFS A/B Scheme"
                status = False
                
            if status is False:
                self.msg_if.pub_warn("Automatic rootfs active/inactive switch failed: " + err_msg)
            else:
                self.msg_if.pub_info("Executed automatic rootfs A/B switch... on next reboot new image will load")
                self.status_msg.warnings.flags[WarningFlags.ACTIVE_INACTIVE_ROOTFS_STALE] = True
                self.status_msg.sys_img_update_status = 'complete - needs reboot'
    
    def handle_switch_active_inactive_rootfs(self, msg):
        if self.rootfs_ab_scheme == 'nepi':
            status, err_msg = sw_update_utils.switchActiveAndInactivePartitions(self.first_stage_rootfs_device)
        elif self.rootfs_ab_scheme == 'jetson':
            status, err_msg = sw_update_utils.switchActiveAndInactivePartitionsJetson()
        else:
            err_msg = "Unknown ROOTFS A/B Scheme"
            status = False
            
        if status is False:
            self.msg_if.pub_warn("Failed to switch active/inactive rootfs: " + err_msg)
            return

        self.status_msg.warnings.flags[WarningFlags.ACTIVE_INACTIVE_ROOTFS_STALE] = True
        self.msg_if.pub_warn(
            "Switched active and inactive rootfs. Must reboot system for changes to take effect")

    def handle_archive_inactive_rootfs(self, msg):
        if self.archiving_inactive_image is True:
            self.msg_if.pub_warn("Already in the process of archiving image")
            return
        fw_str = self.system_defs_msg.inactive_rootfs_fw_version
        fw_str = fw_str.replace('.','_')
        fw_str = fw_str.replace(' ','_')
        fw_str = fw_str.replace('/','_')
        now = datetime.datetime.now()
        backup_file_basename = 'nepi_' + fw_str + now.strftime("_%Y_%m_%d_%H%M%S") + '.img.raw'
        self.msg_if.pub_warn("Archiving inactive rootfs to filename: " + backup_file_basename)
        self.status_msg.sys_img_archive_status = 'archiving'
        self.status_msg.sys_img_archive_filename = backup_file_basename

        # Transfers to USB seem to have trouble with the standard block size, so allow those to proceed at a lower
        # block size
        slow_transfer = True if self.usb_device in self.new_img_staging_device else False
                
        self.archiving_inactive_image = True
        status, err_msg = sw_update_utils.archiveInactiveToStaging(
            self.inactive_rootfs_device, self.new_img_staging_device, backup_file_basename, slow_transfer, progress_cb = self.receive_archive_progress)
        self.archiving_inactive_image = False

        if status is False:
            self.msg_if.pub_warn("Failed to backup inactive rootfs: " + err_msg)
            self.status_msg.sys_img_archive_status = 'failed'
        else:
            self.msg_if.pub_info("Finished archiving inactive rootfs")
            self.status_msg.sys_img_archive_status = 'archive complete'
    
    def provide_system_defs(self, req):
        return SystemDefsQueryResponse(self.system_defs_msg)

    def provide_op_environment(self, req):
        # Just proxy the param server
        if self.node_if is not None:
            return OpEnvironmentQueryResponse(self.node_if.get_param("op_environment"))
        else:
            return OpEnvironmentQueryResponse(OpEnvironmentQueryResponse.OP_ENV_AIR)
    
    def save_data_prefix_callback(self, msg):
        save_data_prefix = msg.data

        data_folder = self.storage_subdirs['data']
        if data_folder is None:
            return # No data directory

        # Now ensure the directory exists if this prefix defines a subdirectory
        full_path = os.path.join(data_folder, save_data_prefix)
        parent_path = os.path.dirname(full_path)
        if not os.path.exists(parent_path):
            self.msg_if.pub_info("Creating new data subdirectory " + parent_path)
            os.makedirs(parent_path)
            
            # Gather owner and group details for data folder to propagate them
            # TODO: Do we need to do this recursively in case this we are creating multiple levels of subdirectory here
            stat_info = os.stat(data_folder)
            new_dir_uid = stat_info.st_uid
            new_dir_guid = stat_info.st_gid

            os.chown(parent_path, new_dir_uid, new_dir_guid)
    
    def get_device_friendly_name(self, devfs_name):
        # Leave space for the partition number
        friendly_name = devfs_name.replace(self.emmc_device, "EMMC Partition ")
        friendly_name = friendly_name.replace(self.usb_device, "USB Partition ")
        friendly_name = friendly_name.replace(self.sd_card_device, "SD Partition ")
        friendly_name = friendly_name.replace(self.ssd_device, "SSD Partition ")
        return friendly_name

    def init_msgs(self):
        self.system_defs_msg.firmware_version = self.get_fw_rev()

        self.system_defs_msg.device_sn = self.get_device_sn()

        self.system_defs_msg.device_type = self.get_device_type()

        # TODO: Determine how many temperature readings we have. On Jetson, for example
        #      there are 8 "thermal zones" in /sys/class/thermal/
        self.system_defs_msg.temperature_sensor_names.append('CPU Zone 0')
        # TODO: Configurable warning/error temperatures
        self.system_defs_msg.warning_temps.append(60.0)
        self.system_defs_msg.critical_temps.append(70.0)

        statvfs = os.statvfs(self.storage_mountpoint)
        self.system_defs_msg.disk_capacity = statvfs.f_frsize * statvfs.f_blocks / \
            BYTES_PER_MEGABYTE     # Size of data filesystem in Megabytes

        # Gather some info about ROOTFS A/B configuration
        status = False
        if self.rootfs_ab_scheme == 'nepi':
            self.system_defs_msg.first_stage_rootfs_device = self.get_device_friendly_name(self.first_stage_rootfs_device)
            (status, err_msg, rootfs_ab_settings_dict) = sw_update_utils.getRootfsABStatus(
                self.first_stage_rootfs_device)
        elif self.rootfs_ab_scheme == 'jetson':
            self.system_defs_msg.first_stage_rootfs_device = 'N/A'
            (status, err_msg, rootfs_ab_settings_dict) = sw_update_utils.getRootfsABStatusJetson()
        else:
            self.msg_if.pub_warn("Failed to identify the ROOTFS A/B Scheme... cannot update A/B info and status")

        if status is True:
            self.system_defs_msg.active_rootfs_device = self.get_device_friendly_name(rootfs_ab_settings_dict[
                'active_part_device'])

            self.system_defs_msg.active_rootfs_size_mb = sw_update_utils.getPartitionByteCount(rootfs_ab_settings_dict[
                'active_part_device']) / BYTES_PER_MEGABYTE
            
            self.inactive_rootfs_device = rootfs_ab_settings_dict[
                'inactive_part_device']
            self.system_defs_msg.inactive_rootfs_device = self.get_device_friendly_name(self.inactive_rootfs_device)

            self.system_defs_msg.inactive_rootfs_size_mb = sw_update_utils.getPartitionByteCount(self.inactive_rootfs_device) / BYTES_PER_MEGABYTE
            
            self.system_defs_msg.inactive_rootfs_fw_version = rootfs_ab_settings_dict[
                'inactive_part_fw_version']
            self.system_defs_msg.max_boot_fail_count = rootfs_ab_settings_dict[
                'max_boot_fail_count']
        else:
            self.msg_if.pub_warn(
                "Unable to gather ROOTFS A/B system definitions: " + err_msg)
            self.system_defs_msg.active_rootfs_device = "Unknown"
            self.system_defs_msg.inactive_rootfs_device = "Unknown"
            self.inactive_rootfs_device = "Unknown"
            self.system_defs_msg.inactive_rootfs_fw_version = "Unknown"
            self.system_defs_msg.max_boot_fail_count = 0

        for i in self.system_defs_msg.temperature_sensor_names:
            self.status_msg.temperatures.append(0.0)

	    # TODO: Should this be queried somehow e.g., from the param server
        self.status_msg.save_all_enabled = False

    def getNEPIStorageDevice(self):
        if self.in_container == False:
          # Try to read the NEPI storage device out of /etc/fstab
          if os.path.exists('/etc/fstab'):
              with open('/etc/fstab', 'r') as fstab:
                  lines = fstab.readlines()
                  for line in lines:
                      if self.storage_mountpoint in line and not line.startswith('#'):
                          candidate = line.split()[0] # First token is the device
                          if candidate.startswith('/dev/'):
                              self.nepi_storage_device = candidate
                              self.msg_if.pub_info('Identified NEPI storage device ' + self.nepi_storage_device + ' from /etc/fstab')
                              return
                          else:
                              self.msg_if.pub_warn('Candidate NEPI storage device from /etc/fstab is of unexpected form: ' + candidate)
            
          # If we get here, failed to get the storage device from /etc/fstab
          self.msg_if.pub_warn('Failed to get NEPI storage device from /etc/fstab -- falling back to system_mgr config file')
          if not nepi_sdk.has_param("~nepi_storage_device"):
              self.msg_if.pub_warn("Parameter nepi_storage_device not available -- falling back to hard-coded " + self.nepi_storage_device)
          else:
              self.nepi_storage_device = nepi_sdk.get_param(
                "~nepi_storage_device", self.nepi_storage_device)
              self.msg_if.pub_info("Identified NEPI storage device " + self.nepi_storage_device + ' from config file')
        else:
            self.nepi_storage_device = self.storage_mountpoint
    



    



if __name__ == '__main__':
    SysMgr = SystemMgrNode()
