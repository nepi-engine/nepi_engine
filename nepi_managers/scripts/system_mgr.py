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
from socket import INADDR_UNSPEC_GROUP
import time
import shutil
from collections import deque
import re
import datetime
import subprocess
import importlib
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system

                 
#####################
# System         

from nepi_sdk import nepi_states
from nepi_sdk import nepi_triggers
from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_interfaces.msg import MgrSystemStatus, WarningFlags, StampedString, StringArray, \
                                DictString, DictStringEntry, UpdateBool, UpdateString
                      
from nepi_interfaces.srv import SystemStatusQuery, SystemStatusQueryRequest, SystemStatusQueryResponse


from nepi_interfaces.msg import SystemTrigger, SystemTriggersStatus
from nepi_interfaces.srv import SystemTriggersQuery, SystemTriggersQueryRequest, SystemTriggersQueryResponse

from nepi_interfaces.msg import SystemState, SystemStatesStatus
from nepi_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import StatesIF, TriggersIF, SaveDataIF


BYTES_PER_MEGABYTE = 2**20



NEPI_FOLDER='/opt/nepi'

class SystemMgrNode():


    
    STATUS_PERIOD = 1.0  # TODO: Configurable update period?

    DISK_FULL_MARGIN_MB = 250  # MB TODO: Configurable?

    SYS_ETC_PATH = NEPI_FOLDER + "/etc"
    SYS_ENV_PATH = SYS_ETC_PATH + "/sys_env.bash"
    FW_VERSION_PATH = NEPI_FOLDER + "/nepi_engine/etc/fw_version.txt"

    STATES_DICT = dict()

    RUN_MODES = ['prototype','deploy']

    REQD_STORAGE_SUBDIRS = ["ai_models", 
                        "ai_training",
                        "data", 
                        "databases", 
                        "databases/targets", 
                        "install",
                        "install/updates",
                        "install/apps",
                        "install/drivers",
                        "license", 
                        "logs", 
                        "logs/ros_log",
                        "logs/nepi_scripts_logs", 
                        "nepi_src",
                        "nepi_scripts",
                        "user_cfg",
                        "user_cfg/cals",]

    REQD_CONFIG_SUBDIRS = ["docker_cfg", 
                            "factory_cfg",
                            "system_cfg"]
                            
    
    STORAGE_CHECK_SKIP_LIST = ["ai_training",
                            "data",
                            "logs", 
                            "logs/ros_log",
                            "logs/nepi_scripts_logs", 
                            "nepi_src",
                            "tmp"]
    

    MANAGERS_OPTIONS = ['NETWORK MANAGER',
                        'TIME MANAGER',
                        'SOFTWERE MANAGER',
                        'DEVICE MANAGER',
                        'APPS MANAGER',
                        'AI MODEL MANAGER']

    USER_RESTRICTION_OPTIONS = {
        'Sys-Admin': "Desc",
        'Sys-Debug': "Desc",
        'Cfg-Factory': "Desc",
        'Cfg-System': "Desc",
        'Cfg-User': "Desc",
        'Mgr-Device': "Desc",        
        'Mgr-Device-License': "Desc",
        'Mgr-Time': "Desc",
        'Mgr-Time_NTP': "Desc",
        'Mgr-Time_Sync_Clocks': "Desc",
        'Mgr-Network': "Desc",
        'Mgr-Network-Wired': "Desc",
        'Mgr-Network-DHCP': "Desc",
        'Mgr-Network-WiFi': "Desc",
        'Mgr-Network-Access Point': "Desc", 
        'Mgr-Software Manager': "Desc",
        'Mgr-NavPose Manager': "Desc",
        'Mgr-Driver Manager': "Desc",
        'Mgr-AI Manager': "Desc",
        'Mgr-Apps Manager': "Desc",
        'Dvc': "Desc",
        'Dvc-Controls': "Desc",
        'Dvc-Settings': "Desc",
        'Dvc-Config': "Desc",
        'Set-View': "Desc",
        'Set-Controls': "Desc",
        'Tfm-View': "Desc",
        'Tfm-Controls': "Desc",
        'Trig-View': "Desc",
        'Tri-Controls': "Desc",
        'Sta-View': "Desc",
        'Sta-Controls': "Desc",
        'Img-Stats': "Desc",
        'Img-Controls': "Desc",
        'Msg-View': "Desc",
        'Msg-Controls': "Desc",
        'Dat': "Desc",
        'Dat-Controls': "Desc",
        'Dat-View': "Desc",
        'Sav-View': "Desc",
        'Sav-Controls': "Desc",
        'Sav-All': "Desc",
    }


    node_if = None
    status_msg = MgrSystemStatus()
    status_published = False

    storage_subdirs = dict()
    user_folders = dict()
    system_folders = dict()

    folders_uid = 'nepi' #1000 # default to nepi
    folders_gid = 'nepi'

    # Shorter period for more responsive updates
    disk_usage_deque = deque(maxlen=3)


    in_container = False

    triggers_list = []
    triggers_status_interval = 1.0

    states_status_interval = 1.0

    current_throttle_ratio = 1.0


    admin_enabled = False
    admin_password = None
    admin_password_valid = False
    admin_mode_set = False
    admin_mode_updated = True
    develop_enabled = False
    debug_enabled = False
    managers_enabled_dict = dict()
    deploy_nodes_dict = dict()
    user_restrictions = []
    rui_restricted = []
    run_mode = 'deploy'

    node_names_dict = dict()

    init_complete = False
    ready = False

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


        self.nepi_config = nepi_system.load_nepi_system_config()
        self.msg_if.pub_warn("Got System Config: " + str(self.nepi_config))
        if self.nepi_config is None:
            self.nepi_config = dict()
        if len(self.nepi_config.keys()) == 0:
            self.msg_if.pub_warn("Failed to Read NEPI config file")
            nepi_sdk.signal_shutdown("Shutting Down: Failed to Read NEPI config file")
            return
        for key in self.nepi_config.keys(): # Fix empty arrays
            if self.nepi_config[key] is None:
                self.nepi_config[key]=[]

        # Gather owner and group details for storage mountpoint
        # stat_info = os.stat(self.storage_folder)
        # self.folders_uid = stat_info.st_uid
        # self.folders_gid = stat_info.st_gid
        self.admin_password = self.nepi_config['NEPI_ADMIN_PW']
        self.folders_uid = self.nepi_config['NEPI_USER']
        self.folders_gid = self.nepi_config['NEPI_USER']

        check_path=self.nepi_config['NEPI_STORAGE']
        if os.path.exists(check_path)==False:
            os.mkdir(check_path)
        check_path=self.nepi_config['NEPI_CONFIG']
        if os.path.exists(check_path)==False:
            os.mkdir(check_path)
            os.mkdir(self.nepi_config['FACTORY_CONFIG'])
            os.mkdir(self.nepi_config['SYSTEM_CONFIG'])
            os.mkdir(self.nepi_config['DOCKER_CONFIG'])

        check_path=self.nepi_config['NEPI_IMPORT_PATH']
        if check_path not in self.REQD_STORAGE_SUBDIRS:
            self.REQD_STORAGE_SUBDIRS.append(check_path)
        check_path=self.nepi_config['NEPI_EXPORT_PATH']
        if check_path not in self.REQD_STORAGE_SUBDIRS:
            self.REQD_STORAGE_SUBDIRS.append(check_path)

        nepi_system.set_nepi_config(self.nepi_config)

        self.status_msg.serial_number = str(self.nepi_config['NEPI_DEVICE_SN'])
        hw_type = self.nepi_config['NEPI_HW_TYPE']
        if hw_type != 'unknown':
            self.status_msg.hw_type = hw_type
        hw_model = self.nepi_config['NEPI_HW_MODEL']
        if hw_model != 'unknown':
            self.status_msg.hw_model = hw_model
        self.status_msg.sw_desc = self.nepi_config['NEPI_SW_DESC']
        self.status_msg.has_cuda = self.nepi_config['NEPI_HAS_CUDA'] == 1
        self.status_msg.manages_time = self.nepi_config['NEPI_MANAGES_TIME'] == 1
        self.status_msg.manages_network = self.nepi_config['NEPI_MANAGES_NETWORK'] == 1

        self.in_container = self.nepi_config['NEPI_IN_CONTAINER'] == 1
        self.status_msg.in_container = self.in_container

        self.config_folder = self.nepi_config['NEPI_CONFIG']
        self.storage_folder = self.nepi_config['NEPI_STORAGE']
        self.data_folder = self.storage_folder + "/data"

        
        self.SDK_PATH_DICT = {
            'sdk_pkg': NEPI_FOLDER + '/nepi_engine/lib/python3/dist-packages/nepi_sdk',
            'sdk_lib': NEPI_FOLDER + '/nepi_engine/lib/nepi_sdk',
            }
        self.API_PATH_DICT = {
            'api_pkg': NEPI_FOLDER + '/nepi_engine/lib/python3/dist-packages/nepi_api',
            'api_lib': NEPI_FOLDER + '/nepi_engine/lib/nepi_api',
            }
        self.ETC_PATH_DICT = {
            'etc': NEPI_FOLDER + '/nepi_engine/etc'
            }
        self.CONFIG_FOLDER_DICT = {
            'docker_cfg': self.config_folder + "/docker_cfg",
            'factory_cfg':  self.config_folder + "/factory_cfg",
            'system_cfg':  self.config_folder + "/system_cfg",
            'user_cfg':  self.storage_folder + "/user_cfg",
            }
        self.RUI_PATH_DICT = {
            'rui_env': NEPI_FOLDER + '/rui/.nvm',
            'rui_bld': NEPI_FOLDER + '/rui/src/rui_webserver/rui-app',
            'rui_src': NEPI_FOLDER + '/rui/src/rui_webserver/rui-app/src'
            }
            
        self.DRIVERS_PATH_DICT = {
            'drivers_pkg': NEPI_FOLDER + '/nepi_engine/lib/python3/dist-packages/nepi_drivers',
            'drivers_lib': NEPI_FOLDER + '/nepi_engine/lib/nepi_drivers',
            'drivers_param': NEPI_FOLDER + '/nepi_engine/lib/nepi_drivers',
            'drivers_install': '/mnt/nepi_storage/install/drivers'
            }
        self.AIFS_PATH_DICT = {
            'aifs_pkg': NEPI_FOLDER + '/nepi_engine/lib/python3/dist-packages/nepi_aifs',
            'aifs_lib': NEPI_FOLDER + '/nepi_engine/lib/nepi_aifs',
            'aifs_param': NEPI_FOLDER + '/nepi_engine/share/nepi_aifs',
            'aifs_models': '/mnt/nepi_storage/ai_models/',
            'aifs_install': '/mnt/nepi_storage/install/ai_frameworks'
            }
        self.APPS_PATH_DICT = {
            'apps_pkg': NEPI_FOLDER + '/nepi_engine/lib/python3/dist-packages/nepi_apps',
            'apps_lib': NEPI_FOLDER + '/nepi_engine/lib/nepi_apps',
            'apps_param': NEPI_FOLDER + '/nepi_engine/share/nepi_apps/params',
            'apps_install': '/mnt/nepi_storage/install/apps'
            }

        self.SYSTEM_CHECK_SKIP_LIST = ['rui']
        self.SYSTEM_PATH_DICT = {
            'sdk': self.SDK_PATH_DICT,
            'api': self.API_PATH_DICT,
            'etc': self.ETC_PATH_DICT,
            'cfg': self.CONFIG_FOLDER_DICT,
            'rui': self.RUI_PATH_DICT,
            'drivers': self.DRIVERS_PATH_DICT,
            'aifs': self.AIFS_PATH_DICT,
            'apps': self.APPS_PATH_DICT
        }


        self.req_storage_subdirs = self.REQD_STORAGE_SUBDIRS
        self.req_config_subdirs = self.REQD_CONFIG_SUBDIRS
        


        self.msg_if.pub_warn("Checking User Storage Partition")
        # First check that the storage partition is actually mounted
        if not os.path.ismount(self.storage_folder):
            self.msg_if.pub_warn("NEPI Storage partition is not mounted... attempting to mount")
            ret, msg = self.nepi_image.mountPartition(self.nepi_storage_device, self.storage_folder)
            if ret is False:
                self.msg_if.pub_warn("Unable to mount NEPI Storage partition... system may be dysfunctional")
                #return False # Allow it continue on local storage...
                
        # ... as long as there is enough space
        self.update_storage()
        if self.status_msg.warnings.flags[WarningFlags.DISK_FULL] is True:
            self.msg_if.pub_warn("Insufficient space on storage partition")
            self.storage_folder = ""
            #return False




        self.msg_if.pub_warn("Checking System Folders")
        # Ensure that the user partition is properly laid out
        self.storage_subdirs = {} # Populated in function below
        self.ensure_reqd_subdirs()


        self.msg_if.pub_warn("Storing User Folders")
        nepi_system.set_user_folders(self.user_folders)
        #self.msg_if.pub_warn("Stored user folders: " + str(self.user_folders))

        self.msg_if.pub_info("Waiting for user folders")
        user_folders = nepi_system.get_user_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got user folders: " + str(user_folders))
        
        self.msg_if.pub_warn("Storing System Folders")
        nepi_system.set_system_folders(self.system_folders)
        #self.msg_if.pub_warn("Stored System Folders: " + str(self.system_folders))

        self.msg_if.pub_info("Waiting for system folders")
        system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got system folders: " + str(system_folders))



        self.msg_if.pub_warn("Checking valid device id")
        self.valid_device_id_re = re.compile(r"^[a-zA-Z][\w]*$")


        self.msg_if.pub_warn("Updating Boot Fail Counter")
        nepi_system.update_nepi_docker_config("NEPI_FAIL_COUNT",0)


        self.status_msg.user_restrictions_options = list(self.USER_RESTRICTION_OPTIONS.keys())
        descriptions = []
        for key in self.USER_RESTRICTION_OPTIONS.keys():
            descriptions.append(self.USER_RESTRICTION_OPTIONS[key])
        self.status_msg.user_restrictions_descriptions = descriptions

        self.status_msg.sys_run_mode_options = self.RUN_MODES
        
        for mode in self.RUN_MODES:
            self.managers_enabled_dict[mode] = self.MANAGERS_OPTIONS



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
            'admin_enabled': {
                'namespace': self.base_namespace,
                'factory_val': self.admin_enabled
            },
            'develop_enabled': {
                'namespace': self.base_namespace,
                'factory_val': self.develop_enabled
            },
            'debug_enabled': {
                'namespace': self.base_namespace,
                'factory_val': self.debug_enabled
            },
            'managers_enabled_dict': {
                'namespace': self.base_namespace,
                'factory_val': self.managers_enabled_dict
            },
            'deploy_nodes_dict': {
                'namespace': self.base_namespace,
                'factory_val': self.deploy_nodes_dict
            },
            'run_mode': {
                'namespace': self.base_namespace,
                'factory_val': self.run_mode
            },
            'user_restrictions': {
                'namespace': self.base_namespace,
                'factory_val': []
            },
            'rui_restrictions': {
                'namespace': self.base_namespace,
                'factory_val': []
            },
            'node_names_dict': {
                'namespace': self.base_namespace,
                'factory_val': self.node_names_dict
            },
            'storage_folder': {
                'namespace': self.base_namespace,
                'factory_val': self.storage_folder
            },
        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'system_status_query': {
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
                'topic': 'status',
                'msg': MgrSystemStatus,
                'qsize': 1,
                'latch': True
            },
            'saveParamsCb': {
                'namespace': self.base_namespace,
                'topic': 'saveParamsCb',
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
            }
        }  

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'enable_admin': {
                'namespace': self.base_namespace,
                'topic': 'admin_mode_enable',
                'msg': Bool,
                'qsize': None,
                'callback': self.enableAdminCb, 
                'callback_args': ()
            },
            'set_admin_password': {
                'namespace': self.base_namespace,
                'topic': 'set_admin_password',
                'msg': String,
                'qsize': None,
                'callback': self.setAdminPasswordCb, 
                'callback_args': ()
            },            
            'enable_develop': {
                'namespace': self.base_namespace,
                'topic': 'develop_mode_enable',
                'msg': Bool,
                'qsize': None,
                'callback': self.enableDevelopCb, 
                'callback_args': ()
            },
            'enable_debug': {
                'namespace': self.base_namespace,
                'topic': 'debug_mode_enable',
                'msg': Bool,
                'qsize': None,
                'callback': self.enableDebugCb, 
                'callback_args': ()
            },
            'set_run_mode': {
                'namespace': self.base_namespace,
                'topic': 'set_run_mode',
                'msg': String,
                'qsize': None,
                'callback': self.setRunModeCb, 
                'callback_args': ()
            },
            'add_user_restriction': {
                'namespace': self.base_namespace,
                'topic': 'add_user_restriction',
                'msg': String,
                'qsize': None,
                'callback': self.addUserRestrictionCb, 
                'callback_args': ()
            },
            'remove_user_restriction': {
                'namespace': self.base_namespace,
                'topic': 'remove_user_restriction',
                'msg': String,
                'qsize': None,
                'callback': self.removeUserRestrictionCb, 
                'callback_args': ()
            },
            'add_rui_restriction': {
                'namespace': self.base_namespace,
                'topic': 'add_rui_restriction',
                'msg': String,
                'qsize': None,
                'callback': self.addRuiRestrictionCb, 
                'callback_args': ()
            },
            'remove_rui_restriction': {
                'namespace': self.base_namespace,
                'topic': 'remove_rui_restriction',
                'msg': String,
                'qsize': None,
                'callback': self.removeRuiRestrictionCb, 
                'callback_args': ()
            },
            'update_node_name': {
                'namespace': self.base_namespace,
                'topic': 'update_node_name',
                'msg': UpdateString,
                'qsize': None,
                'callback': self.updateNodeNameCb, 
                'callback_args': ()
            },
            'reset_node_name': {
                'namespace': self.base_namespace,
                'topic': 'reset_node_name',
                'msg': String,
                'qsize': None,
                'callback': self.resetNodeNameCb, 
                'callback_args': ()
            },
            'save_data': {
                'namespace': self.base_namespace,
                'topic': 'save_data',
                'msg': Bool,
                'qsize': None,
                'callback': self.setSaveStatusCb, 
                'callback_args': ()
            },
            'clear_data_folder': {
                'namespace': self.base_namespace,
                'topic': 'clear_data_folder',
                'msg': Empty,
                'qsize': None,
                'callback': self.clearDataFolderCb, 
                'callback_args': ()
            },

            'set_device_id': {
                'namespace': self.base_namespace,
                'topic': 'set_device_id',
                'msg': String,
                'qsize': None,
                'callback': self.setDeviceIdCb, 
                'callback_args': ()
            },        
            'submit_system_error_msg': {
                'namespace': self.base_namespace,
                'topic': 'submit_system_error_msg',
                'msg': String,
                'qsize': None,
                'callback': self.systemErrorCb, 
                'callback_args': ()
            },
            'system_triggers': {
                'namespace': self.base_namespace,
                'topic': 'system_triggers',
                'msg': SystemTrigger,
                'qsize': None,
                'callback': self.systemTriggersCb, 
                'callback_args': ()
            },
            'restart_nepi': {
                'namespace': self.base_namespace,
                'topic': 'restart_nepi',
                'msg': Empty,
                'qsize': None,
                'callback': self.restartNepiCb, 
                'callback_args': ()
            },

        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        wait_cfg_mgr = False,
                        msg_if = self.msg_if
        )

        #ready = self.node_if.wait_for_ready()
        nepi_sdk.wait()

        # Config mgr not running yet, so have to load saved configs ourselfs
        user_cfg_file = self.node_name + '.yaml'
        user_cfg_path = nepi_sdk.create_namespace(self.CONFIG_FOLDER_DICT['user_cfg'],user_cfg_file)
        self.msg_if.pub_warn("Updating From Param Server")
        params_dict = nepi_sdk.load_params_from_file(user_cfg_path,self.node_namespace)
        nepi_sdk.sleep(1)
        self.initCb(do_updates = True)




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
        self.states_if = StatesIF(self.getStatesDictCb,
                        msg_if = self.msg_if)
        time.sleep(1)


        self.msg_if.pub_warn("Completing Initialization Processes") 
        ########################
        # Complete Initialization


               

        # Create Triggers Status Pub Processes
        self.triggers_status_interval = 1.0

        self.msg_if.pub_info(":" + self.class_name + ": Starting triggers status pub service: ")
        #nepi_sdk.start_timer_process(self.triggers_status_interval, self.triggersStatusPubCb, oneshot = True)

        # Create States Status Pub Processes
        self.states_status_interval = 1.0

        self.msg_if.pub_info(":" + self.class_name + ": Starting states status pub service: ")
        nepi_sdk.start_timer_process(self.states_status_interval, self.statesStatusPubCb, oneshot = True)

    
        # Want to update the op_environment (from param server) through the whole system once at
        # start-up, but the only reasonable way to do that is to delay long enough to let all nodes start
        self.msg_if.pub_warn("Updating From Param Server")
        self.initConfig()
    


        # Crate system status pub
        self.msg_if.pub_warn("Starting System Status Messages")
        nepi_sdk.start_timer_process(self.STATUS_PERIOD, self.publishStatusCb)
        nepi_sdk.start_timer_process(1, self.updateTopicsServicesCb, oneshot = True)
        nepi_sdk.start_timer_process(5, self.updateDockerCb)
        self.msg_if.pub_warn("System status ready")

        ##################################
        # Setup Save Data IF Class ####################
        self.msg_if.pub_debug("Starting Save Data IF Initialization")
        self.data_products_list = ['All']
        factory_data_rates= {}
        factory_data_rates['All'] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
       

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_us': False,
            'suffix': "idx",
            'add_node_name': True
            }

        self.msg_if.pub_debug("Starting save_rate_dict: " + str(factory_data_rates))
        sd_namespace = self.base_namespace
        self.save_data_if = SaveDataIF(namespace = sd_namespace,
                                data_products = self.data_products_list,
                                factory_rate_dict = factory_data_rates,
                                factory_filename_dict = factory_filename_dict,
                                log_name_list = [self.node_name],
                                msg_if = self.msg_if
                        )


        #########################################################
        ## Initiation Complete
        self.ready = True
        self.msg_if.pub_warn("Initialization Complete")
        nepi_sdk.spin()


    def initConfig(self):
        if self.node_if is not None:

            # Now gather all the params and set members appropriately
            self.storage_folder = self.node_if.get_param("storage_folder")


            # nepi_storage has some additional logic
            self.getNEPIStorageDevice()
            

            fw_str = self.get_fw_rev()
            self.nepi_config = nepi_system.update_nepi_system_config('NEPI_VERSION',fw_str)
            self.status_msg.firmware_version = fw_str


            # TODO: Determine how many temperature readings we have. On Jetson, for example
            #      there are 8 "thermal zones" in /sys/class/thermal/
            self.status_msg.temperature_sensor_names.append('CPU Zone 0')
            # TODO: Configurable warning/error temperatures
            self.status_msg.warning_temps.append(60.0)
            self.status_msg.critical_temps.append(70.0)

            statvfs = os.statvfs(self.storage_folder)
            self.status_msg.disk_capacity = statvfs.f_frsize * statvfs.f_blocks / \
                BYTES_PER_MEGABYTE     # Size of data filesystem in Megabytes

            for i in self.status_msg.temperature_sensor_names:
                self.status_msg.temperatures.append(0.0)

            # TODO: Should this be queried somehow e.g., from the param server
            self.status_msg.save_all_enabled = False

    
    def initCb(self, do_updates = False):
        if self.node_if is not None:
            self.admin_enabled = self.node_if.get_param('admin_enabled')
            self.develop_enabled = self.node_if.get_param('develop_enabled')
            self.debug_enabled = self.node_if.get_param('debug_enabled')
            self.managers_enabled_dict = self.node_if.get_param('managers_enabled_dict')
            self.deploy_nodes_dict = self.node_if.get_param('deploy_nodes_dict')
            self.run_mode = self.node_if.get_param('run_mode')
            self.user_restrictions = self.node_if.get_param('user_restrictions')
            self.rui_restrictions = self.node_if.get_param('rui_restrictions')
            self.node_names_dict = self.node_if.get_param('node_names_dict')
        if do_updates == True:
            self.updateSystemAdminSettings()

        # self.publish_settings() # Make sure to always publish settings updates

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



    def getStatesDictCb(self):
        return self.STATES_DICT

    def systemTriggersCb(self,msg):
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
            except Exception as e:
                self.msg_if.pub_info(":" + self.class_name + ": Failed to create status msg: " + str(e))
            if self.node_if is not None:
                self.node_if.publish_pub('states_status_pub', msg)
        nepi_sdk.start_timer_process(self.states_status_interval, self.statesStatusPubCb, oneshot = True)


    def add_info_string(self, string, level):
        self.status_msg.info_strings.append(StampedString(
            timestamp=nepi_sdk.get_msg_stamp(), payload=string, priority=level))


    def get_fw_rev(self):
        if (os.path.exists(self.FW_VERSION_PATH) and (os.path.getsize(self.FW_VERSION_PATH) > 0)):
            with open(self.FW_VERSION_PATH, "r") as f:
                fw_version = f.readline().strip()
                return fw_version

        return "UNSPECIFIED"
    
          

    def update_temperatures(self):
        # Get the current temperatures
        if len(self.status_msg.temperatures) == 0:
            return
        
        # TODO: Should the temperature sensor or the entire subproc. cmd line be configurable?
        temp_string_mdegC = subprocess.check_output(
            ["cat", "/sys/class/thermal/thermal_zone0/temp"])
       
        self.status_msg.temperatures[0] = float(temp_string_mdegC) / 1000.0

        # Check for temperature warnings and do thermal throttling
        throttle_ratio_min = 1.0
        for i, t in enumerate(self.status_msg.temperatures):
            if (t > self.status_msg.critical_temps[i]):
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
                if (t > self.status_msg.warning_temps[i]):
                    self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = True
                    throttle_ratio_i = 1.0 - ((t - self.status_msg.warning_temps[i]) / (
                        self.status_msg.critical_temps[i] - self.status_msg.warning_temps[i]))
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
            statvfs = os.statvfs(self.storage_folder)
        except Exception as e:
            warn_str = "Error checking data storage status of " + self.storage_folder + ": " + e.what()
            self.msg_if.pub_warn(warn_str)
            self.add_info_string("warn_str")
            self.status_msg.disk_usage = 0
            self.storage_rate = 0
            return

        disk_free = float(statvfs.f_frsize) * \
            statvfs.f_bavail / BYTES_PER_MEGABYTE  # In MB
        self.status_msg.disk_usage = self.status_msg.disk_capacity - disk_free

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

 

    def updateDockerCb(self, event):
        nepi_system.update_nepi_docker_config("NEPI_FAIL_COUNT" , 0)

    def updateTopicsServicesCb(self, event):
        [topics_list,types_list] = nepi_sdk.get_topics_data_list()
        #self.msg_if.pub_warn("Got Topics List: " + str(topics_list))
        self.status_msg.active_topics = topics_list
        self.status_msg.active_topic_types = types_list

        servicess_list = nepi_sdk.get_service_list()
        #self.msg_if.pub_warn("Got Services List: " + str(servicess_list))
        self.status_msg.active_services = servicess_list
        nepi_sdk.start_timer_process(5, self.updateTopicsServicesCb, oneshot = True)
 


    def setSaveStatusCb(self, save_msg):
        self.status_msg.save_all_enabled = save_msg.data

    #######################
    def updateSystemAdminSettings(self):
        admin_password_valid = (self.admin_password_valid or self.run_mode == 'develop')
        managers_enabled = self.MANAGERS_OPTIONS
        if self.run_mode in self.managers_enabled_dict.keys():
            managers_enabled = self.managers_enabled_dict[self.run_mode]


        self.admin_mode_set = self.admin_enabled and admin_password_valid
        self.status_msg.sys_admin_enabled = self.admin_enabled
        self.status_msg.sys_admin_password_valid = admin_password_valid 
        self.status_msg.sys_admin_mode_set = self.admin_mode_set
        self.status_msg.sys_develop_enabled = self.develop_enabled   
        self.status_msg.sys_debug_enabled = self.debug_enabled    

        self.status_msg.sys_run_mode = self.run_mode

        self.status_msg.sys_managers_options = self.MANAGERS_OPTIONS
        self.status_msg.sys_managers_enabled = managers_enabled

        self.status_msg.user_restrictions = self.user_restrictions
        user_restricted = []
        if self.develop_enabled == False:
            user_restricted = self.user_restrictions
        self.status_msg.user_restricted = user_restricted

        self.status_msg.rui_restrictions = self.rui_restrictions
        rui_restricted = []
        if self.develop_enabled == False:
            rui_restricted = self.rui_restrictions
        self.status_msg.rui_restricted = rui_restricted

        
        node_name_aliases = []
        for node_name in self.node_names_dict.keys():
            node_name_aliases.append(self.node_names_dict[node_name])
        self.status_msg.sys_node_name_keys = list(self.node_names_dict.keys())
        self.status_msg.sys_node_name_aliases = node_name_aliases

        self.publish_status()
        nepi_system.set_admin_mode(self.admin_mode_set)
        nepi_system.set_develop_mode(self.develop_enabled)
        nepi_system.set_debug_mode(self.debug_enabled)
        nepi_system.set_managers_enabled(managers_enabled)
        nepi_system.set_user_restrictions(user_restricted)
        nepi_system.set_run_mode(self.run_mode)
        nepi_system.set_node_names_dict(self.node_names_dict)

    def enableAdminCb(self, msg):
        self.admin_enabled = msg.data
        self.updateSystemAdminSettings()
        if self.node_if is not None:
            self.node_if.set_param('admin_enabled',msg.data)
            self.node_if.save_config()

    def setAdminPasswordCb(self, msg):
        password = msg.data
        if password == self.admin_password:
            self.admin_password_valid = True
            self.updateSystemAdminSettings()

    def enableDevelopCb(self, msg):
        self.develop_enabled = msg.data
        self.updateSystemAdminSettings()
        if self.node_if is not None:
            self.node_if.set_param('develop_enabled',msg.data)
            self.node_if.save_config()
            
    def enableDebugCb(self, msg):
        self.debug_enabled = msg.data
        self.updateSystemAdminSettings()
        if self.node_if is not None:
            self.node_if.set_param('debug_enabled',msg.data)
            self.node_if.save_config()

    def setRunModeCb(self, msg):
        run_mode = msg.data
        if run_mode in self.RUN_MODES:
            self.run_mode = msg.data
            self.updateSystemAdminSettings()
            if self.node_if is not None:
               self.node_if.set_param('run_mode',msg.data)
               self.node_if.save_config()


    def addUserRestrictionCb(self, msg):
        name = msg.data
        if name in self.USER_RESTRICTION_OPTIONS.keys():
            if name not in self.user_restrictions:  
                self.user_restrictions.append(name)
        self.updateSystemAdminSettings()
        if self.node_if is not None:
            self.node_if.set_param('user_restrictions',self.user_restrictions)
            self.node_if.save_config()


    def removeUserRestrictionCb(self, msg):
        name = msg.data
        if name in self.user_restrictions:  
            self.user_restrictions.remove(name)
        self.updateSystemAdminSettings()
        if self.node_if is not None:
            self.node_if.set_param('user_restrictions',self.user_restrictions)
            self.node_if.save_config()


    def addRuiRestrictionCb(self, msg):
        name = msg.data
        if name not in self.rui_restrictions:  
            self.rui_restrictions.append(name)
        self.updateSystemAdminSettings()
        if self.node_if is not None:
            self.node_if.set_param('rui_restrictions',self.rui_restrictions)
            self.node_if.save_config()


    def removeRuiRestrictionCb(self, msg):
        name = msg.data
        if name in self.rui_restrictions:  
            self.rui_restrictions.remove(name)
        self.updateSystemAdminSettings()
        if self.node_if is not None:
            self.node_if.set_param('rui_restrictions',self.rui_restrictions)
            self.node_if.save_config()

    ###################
    def restartNepiCb(self, msg):
        if self.in_container == True:
            self.nepi_image.restart()

    def updateNodeNameCb(self, msg):
        cur_node_name = msg.name
        new_node_name = nepi_utils.get_clean_name(msg.value)
        if cur_node_name != '' and new_node_name != '':

            is_valid = False
            count = 0
            while is_valid == False:
                is_valid = True
                count = count + 1
                for node_name in self.node_names_dict.keys():
                    if new_node_name == self.node_names_dict[node_name]:
                        is_valid = False
                if is_valid == False:
                    new_node_name = new_node_name + '_' + str(count)

            
            needs_update = True
            for node_name in self.node_names_dict.keys():
                set_node_name = self.node_names_dict[node_name]
                if set_node_name == cur_node_name:
                    self.node_names_dict[node_name] = new_node_name
                    needs_update = False
            if needs_update == True:
                self.node_names_dict[cur_node_name] = new_node_name
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('node_names_dict',self.node_names_dict)
            self.node_if.save_config()


    def resetNodeNameCb(self, msg):
        clear_node_name = msg.data
        if clear_node_name != '':
            purge_name = None
            needs_update = True
            for node_name in self.node_names_dict.keys():
                set_node_name = self.node_names_dict[node_name]
                if set_node_name == clear_node_name:
                    purge_name = node_name
                    needs_update = False
            if needs_update == True and clear_node_name in self.node_names_dict.keys():
                purge_name = clear_node_name
        if purge_name is not None and purge_name in self.node_names_dict.keys():
            del self.node_names_dict[purge_name]
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('node_names_dict',self.node_names_dict)
            self.node_if.save_config()




    def ensure_reqd_subdirs(self):
        # Check for and create subdirectories as necessary
        self.msg_if.pub_warn("Checking nepi config folders")
        for subdir in self.req_config_subdirs:
            full_path_subdir = os.path.join(self.config_folder, subdir)
            if not os.path.isdir(full_path_subdir):
                self.msg_if.pub_warn("Required config subdir " + subdir + " not present... will create")
                os.makedirs(full_path_subdir)
                # And set the owner:group and permissions. Do this every time to fix bad settings e.g., during SSD setup
                # TODO: Different owner:group for different folders?
            if False: # subdir not in self.STORAGE_CHECK_SKIP_LIST:
                self.msg_if.pub_warn("Checking nepi config folder permissions: " + subdir)
                os.system('chown -R ' + str(self.folders_uid) + ':' + str(self.folders_gid) + ' ' + full_path_subdir) # Use os.system instead of os.chown to have a recursive option
                #os.chown(full_path_subdir, self.folders_uid, self.folders_gid)
                os.system('chmod -R 0775 ' + full_path_subdir)
            self.storage_subdirs[subdir] = full_path_subdir
            self.user_folders[subdir] = full_path_subdir


        self.msg_if.pub_warn("Checking nepi storage folders")
        for subdir in self.req_storage_subdirs:
            full_path_subdir = os.path.join(self.storage_folder, subdir)
            if not os.path.isdir(full_path_subdir):
                self.msg_if.pub_warn("Required storage subdir " + subdir + " not present... will create")
                os.makedirs(full_path_subdir)
                # And set the owner:group and permissions. Do this every time to fix bad settings e.g., during SSD setup
                # TODO: Different owner:group for different folders?
            if subdir not in self.STORAGE_CHECK_SKIP_LIST:
                self.msg_if.pub_warn("Checking nepi storage folder permissions: " + subdir)
                os.system('chown -R ' + str(self.folders_uid) + ':' + str(self.folders_gid) + ' ' + full_path_subdir) # Use os.system instead of os.chown to have a recursive option
                #os.chown(full_path_subdir, self.folders_uid, self.folders_gid)
                os.system('chmod -R 0775 ' + full_path_subdir)
            self.storage_subdirs[subdir] = full_path_subdir
            self.user_folders[subdir] = full_path_subdir



        # Check system folders
        self.msg_if.pub_warn("Checking nepi config folder")
        if not os.path.isdir(self.SYS_ETC_PATH):
                self.msg_if.pub_warn("Folder " + self.SYS_ETC_PATH + " not present... will create")
                os.makedirs(self.SYS_ETC_PATH)
        os.system('chown -R ' + str(self.folders_uid) + ':' + str(self.folders_gid) + ' ' + self.SYS_ETC_PATH) # Use os.system instead of os.chown to have a recursive option
        os.system('chmod -R 0775 ' + self.SYS_ETC_PATH)
        self.storage_subdirs['config'] = self.SYS_ETC_PATH


        for entry in self.SYSTEM_PATH_DICT.keys():
            path_dict = self.SYSTEM_PATH_DICT[entry]
            for key in path_dict:
                path_entry = path_dict[key]
                if entry not in self.SYSTEM_CHECK_SKIP_LIST and key not in self.SYSTEM_CHECK_SKIP_LIST:
                    self.msg_if.pub_warn("Checking system folder: " + key + " at: " + path_entry)
                    if not os.path.isdir(path_entry):
                            self.msg_if.pub_warn("Folder " + path_entry + " not present... will create")
                            os.makedirs(path_entry)
                    
                    os.system('chown -R ' + str(self.folders_uid) + ':' + str(self.folders_gid) + ' ' + path_entry) # Use os.system instead of os.chown to have a recursive option
                    os.system('chmod -R 0775 ' + path_entry)
                    #nepi_utils.remove_pycache_folders(path_entry)
                self.storage_subdirs[key] = path_entry
                self.system_folders[key] = path_entry
        return True




    def clearDataFolderCb(self, msg):
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


    def setDeviceIdCb(self, msg):
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
        # Update etc files
        nepi_system.update_nepi_system_config("NEPI_DEVICE_ID",msg.data)
        etc_update_script = self.NEPI_ETC_UPDATE_SCRIPTS_PATH + "/update_etc_hostname.sh"
        subprocess.call([etc_update_script])
        nepi_utils.sleep(1)
        self.nepi_config = self.get_nepi_system_config()
        
        if 'NEPI_HW_TPE' not in self.nepi_config.keys():
            self.nepi_config = nepi_system.update_nepi_system_config('NEPI_HW_TYPE','unknown')
        
        if 'NEPI_SW_DESC' not in self.nepi_config.keys():
             self.nepi_config = nepi_system.update_nepi_system_config('NEPI_SW_DESC','unknown')


        self.msg_if.pub_warn("Device ID Updated - Requires device reboot")
        self.add_info_string(
            "Device ID updated - Requires device reboot", StampedString.PRI_ELEVATED)
        self.node_if.save_config()

    def systemErrorCb(self, msg):
        self.add_info_string(msg.data, StampedString.PRI_HIGH)
    

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





    def getNEPIStorageDevice(self):
        if self.in_container == True:
            self.nepi_storage_device = self.storage_folder
        else:
          # Try to read the NEPI storage device out of /etc/fstab
          if os.path.exists('/etc/fstab'):
              with open('/etc/fstab', 'r') as fstab:
                  lines = fstab.readlines()
                  for line in lines:
                      if self.storage_folder in line and not line.startswith('#'):
                          candidate = line.split()[0] # First token is the device
                          if candidate.startswith('/dev/'):
                              self.nepi_storage_device = candidate
                              self.msg_if.pub_info('Identified NEPI storage device ' + self.nepi_storage_device + ' from /etc/fstab')
                              return
                          else:
                              self.msg_if.pub_warn('Candidate NEPI storage device from /etc/fstab is of unexpected form: ' + candidate)
            
          # If we get here, failed to get the storage device from /etc/fstab
          self.msg_if.pub_warn('Failed to get NEPI storage device from /etc/fstab -- falling back to system_mgr config file')
    


   
    def provide_system_status(self, req):
        response = SystemStatusQueryResponse()
        response.system_status = self.status_msg
        #self.msg_if.pub_warn("Returning status query response: " + str(response))
        return response


    def publishStatusCb(self, event):
        self.publish_status()



    def publish_status(self):
        
        self.update_temperatures()
        # Populate the rest of the message contents
        # Temperature(s)
        self.update_temperatures()

        # Disk usage
        self.update_storage()

        # Now publish it
        if self.node_if is not None:
            if self.status_published == False:
                self.status_published = True
                #self.msg_if.pub_warn("Publishing status message: " + str(self.status_msg))
            self.node_if.publish_pub('status_pub', self.status_msg)

        # Always clear info strings after publishing
        del self.status_msg.info_strings[:]

    



if __name__ == '__main__':
    SysMgr = SystemMgrNode()