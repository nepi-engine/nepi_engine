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
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_states
from nepi_sdk import nepi_triggers
from nepi_sdk import nepi_software
from nepi_sdk import nepi_docker

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import MgrSystemStatus, SystemDefs, WarningFlags, StampedString, SaveDataStatus, StringArray, \
                                DictString, DictStringEntry

                                
from nepi_interfaces.srv import SystemDefsQuery, SystemDefsQueryRequest, SystemDefsQueryResponse, \
                             OpEnvironmentQuery, OpEnvironmentQueryRequest, OpEnvironmentQueryResponse, \
                             SystemSoftwareStatusQuery, SystemSoftwareStatusQueryRequest, SystemSoftwareStatusQueryResponse, \
                             SystemStorageFolderQuery, SystemStorageFolderQueryRequest, SystemStorageFolderQueryResponse, \
                             DebugQuery, DebugQueryRequest, DebugQueryResponse, \
                             AdminQuery, AdminQueryRequest, AdminQueryResponse,  \
                            SystemStatusQuery, SystemStatusQueryRequest, SystemStatusQueryResponse


from nepi_interfaces.msg import SystemTrigger, SystemTriggersStatus
from nepi_interfaces.srv import SystemTriggersQuery, SystemTriggersQueryRequest, SystemTriggersQueryResponse

from nepi_interfaces.msg import SystemState, SystemStatesStatus
from nepi_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import StatesIF, TriggersIF


BYTES_PER_MEGABYTE = 2**20



NEPI_FOLDER='/opt/nepi'

class SystemMgrNode():
    

    STATUS_PERIOD = 1.0  # TODO: Configurable update period?

    DISK_FULL_MARGIN_MB = 250  # MB TODO: Configurable?

    SYS_ETC_PATH = NEPI_FOLDER + "/etc"
    SYS_ENV_PATH = NEPI_FOLDER + "/sys_env.bash"
    FW_VERSION_PATH = NEPI_FOLDER + "/nepi_engine/etc/fw_version.txt"
    NEPI_CONFIG_FILE = NEPI_FOLDER + '/etc/nepi_system_config.yaml'

    STATES_DICT = dict()


    REQD_STORAGE_SUBDIRS = ["ai_models", 
                        "automation_scripts", 
                        "data", 
                        "databases", 
                        "databases/targets", 
                        "license", 
                        "logs", 
                        "logs/ros_log",
                        "logs/automation_script_logs", 
                        "nepi_src",
                        "user_cfg",
                        "user_cfg/cal",]

    REQD_CONFIG_SUBDIRS = ["docker_cfg", 
                            "factory_cfg",
                            "system_cfg"]
                            
    
    STORAGE_CHECK_SKIP_LIST = ["ai_training",
                            "data",
                            "logs", 
                            "logs/ros_log",
                            "logs/automation_script_logs", 
                            "nepi_src",
                            "tmp"]

    ADMIN_RESTRICT_OPTIONS = {
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
        'Dat-Viwew': "Desc"
    }


    node_if = None
    status_msg = MgrSystemStatus()
    system_defs_msg = SystemDefs()

    storage_subdirs = dict()
    user_folders = dict()
    system_folders = dict()

    folders_uid = 1000 # default to nepi
    folders_gid = 130 # default to "sambashare" # TODO This is very fragile

    # Shorter period for more responsive updates
    disk_usage_deque = deque(maxlen=3)


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

    debug_enabled = False
    admin_enabled = False
    admin_restricted = []

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "system_mgr" # Can be overwitten by luanch command
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


        ###############################
        # Initialize Class Variables


        self.nepi_config = nepi_utils.read_dict_from_file(self.NEPI_CONFIG_FILE)
        self.msg_if.pub_warn("Got System Config: " + str(self.nepi_config))
        if self.nepi_config is None:
            self.nepi_config = dict()
        for key in self.nepi_config.keys(): # Fixe empty arrays
            if self.nepi_config[key] is None:
                self.nepi_config[key]=[]



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

        self.system_defs_msg.hw_type = self.nepi_config['NEPI_HW_TYPE']
        self.status_msg.hw_type = self.nepi_config['NEPI_HW_TYPE']

        self.system_defs_msg.hw_model = self.nepi_config['NEPI_HW_MODEL']
        self.status_msg.hw_model = self.nepi_config['NEPI_HW_MODEL']

        self.system_defs_msg.has_cuda = self.nepi_config['NEPI_HAS_CUDA']
        self.status_msg.has_cuda = self.nepi_config['NEPI_HAS_CUDA']

        self.system_defs_msg.manages_time = self.nepi_config['NEPI_MANAGES_TIME'] == 1
        self.status_msg.manages_time = self.nepi_config['NEPI_MANAGES_TIME'] == 1

        self.system_defs_msg.manages_network = self.nepi_config['NEPI_MANAGES_NETWORK'] == 1
        self.status_msg.manages_network = self.nepi_config['NEPI_MANAGES_NETWORK'] == 1

        self.in_container = self.nepi_config['NEPI_IN_CONTAINER'] == 1
        self.system_defs_msg.in_container = self.in_container
        self.status_msg.in_container = self.in_container


        self.first_rootfs = self.nepi_config['NEPI_FS_DEVICE']
        self.nepi_storage_device = self.nepi_config['NEPI_STORAGE_DEVICE']
        self.new_img_staging = self.nepi_config['NEPI_STORAGE_DEVICE']
        self.new_img_staging_removable = False

        self.usb_device = "/dev/sda" 
        self.sd_card_device = "/dev/mmcblk1p"
        self.emmc_device = "/dev/mmcblk0p"
        self.ssd_device = "/dev/nvme0n1p"

        self.config_folder = self.nepi_config['NEPI_CONFIG']
        self.storage_folder = self.nepi_config['NEPI_STORAGE']
        self.data_folder = self.storage_folder + "/data"

        if self.in_container == True:
            self.nepi_image = nepi_docker
            self.msg_if.pub_warn("NEPI Running in Container")
        else:
            self.nepi_image = nepi_software
            self.msg_if.pub_warn("Using first stage boot device: " + str(self.first_rootfs))

        self.system_defs_msg.inactive_rootfs_fw_version = "uknown"
        '''
        self.msg_if.pub_warn("Deleting old log files")
        logs_path_subdir = os.path.join(self.storage_folder, 'logs/ros_log')
        os.system('rm -r ' + logs_path_subdir + '/*')
        '''
        
        self.msg_if.pub_warn("Updating Rootfs Scheme")
        # Need to identify the rootfs scheme because it is used in init_msgs()
        self.rootfs_ab_scheme = self.nepi_image.identifyRootfsABScheme()
        self.msg_if.pub_warn("Got Rootfs Scheme: " + self.rootfs_ab_scheme)
        self.init_msgs()




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
            return False

        # Gather owner and group details for storage mountpoint
        stat_info = os.stat(self.storage_folder)
        self.folders_uid = stat_info.st_uid
        self.folders_gid = stat_info.st_gid


        self.msg_if.pub_warn("Checking System Folders")
        # Ensure that the user partition is properly laid out
        self.storage_subdirs = {} # Populated in function below
        if self.ensure_reqd_subdirs() is True:
            # Now can advertise the system folder query
            nepi_sdk.create_service('system_storage_folder_query', SystemStorageFolderQuery, self.provide_system_data_folder)

        self.msg_if.pub_warn("Storing User Folders")
        nepi_system.set_user_folders(self.user_folders)
        self.msg_if.pub_warn("Stored user folders: " + str(self.user_folders))
        
        self.msg_if.pub_warn("Storing System Folders")
        nepi_system.set_system_folders(self.system_folders)
        self.msg_if.pub_warn("Stored System Folders: " + str(self.system_folders))



        self.msg_if.pub_warn("Checking valid device id")
        self.valid_device_id_re = re.compile(r"^[a-zA-Z][\w]*$")


        self.msg_if.pub_warn("Updating Rootfs Load Fail Counter")
        # Reset the A/B rootfs boot fail counter -- if this node is running, pretty safe bet that we've booted successfully
        # This should be redundant, as we need a non-ROS reset mechanism, too, in case e.g., ROS nodes are delayed waiting
        # for a remote ROS master to start. That could be done in roslaunch.sh or a separate start-up script.
        if self.rootfs_ab_scheme == 'nepi': # The 'jetson' scheme handles this itself
            status, err_msg = self.nepi_image.resetBootFailCounter(
                self.first_rootfs)
            if status is False:
                self.msg_if.pub_warn("Failed to reset boot fail counter: " + err_msg)

        roptions=[]
        for key in self.ADMIN_RESTRICT_OPTIONS.keys():
            ropt=DictStringEntry()
            ropt.key = key
            ropt.value = self.ADMIN_RESTRICT_OPTIONS[key]
        self.status_msg.sys_admin_restrict_options = roptions
        

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
            'storage_folder': {
                'namespace': self.base_namespace,
                'factory_val': self.storage_folder
            },
            'auto_switch_rootfs_on_new_img_install': {
                'namespace': self.base_namespace,
                'factory_val': self.auto_switch_rootfs_on_new_img_install
            },
            'first_rootfs': {
                'namespace': self.base_namespace,
                'factory_val': self.first_rootfs
            },
            'new_img_staging': {
                'namespace': self.base_namespace,
                'factory_val': self.new_img_staging
            },
            'new_img_staging_removable': {
                'namespace': self.base_namespace,
                'factory_val': self.new_img_staging_removable
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
            },
            'admin_enabled': {
                'namespace': self.base_namespace,
                'factory_val': False
            },
            'admin_restricted': {
                'namespace': self.base_namespace,
                'factory_val': []
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
            'admin_query': {
                'namespace': self.base_namespace,
                'topic': 'admin_mode_query',
                'srv': AdminQuery,
                'req': AdminQueryRequest(),
                'resp': AdminQueryResponse(),
                'callback': self.provide_admin_status
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
            }
        }  

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
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
            'set_op_environment': {
                'namespace': self.base_namespace,
                'topic': 'set_op_environment',
                'msg': String,
                'qsize': None,
                'callback': self.setOpEnvCb, 
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
            'install_new_image': {
                'namespace': self.base_namespace,
                'topic': 'install_new_image',
                'msg': String,
                'qsize': 1,
                'callback': self.installNewImageCb, 
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
                'callback': self.archiveImageCb, 
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
                'callback': self.systemTriggersCb, 
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
            'enable_admin': {
                'namespace': self.base_namespace,
                'topic': 'admin_mode_enable',
                'msg': Bool,
                'qsize': None,
                'callback': self.enableAdminCb, 
                'callback_args': ()
            },
            'set_admin_restricted': {
                'namespace': self.base_namespace,
                'topic': 'set_admin_restricted',
                'msg': StringArray,
                'qsize': None,
                'callback': self.setAdminRestrictedCb, 
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
                        wait_cfg_mgr = False,
                        msg_if = self.msg_if
        )

        #ready = self.node_if.wait_for_ready()
        nepi_sdk.wait()

        # Config mgr not running yet, so have to load saved configs ourselfs
        user_cfg_file = self.node_name + '.yaml.user'
        user_cfg_path = nepi_sdk.create_namespace(self.CONFIG_FOLDER_DICT['user_cfg'],user_cfg_file)
        self.msg_if.pub_warn("Updating From Param Server")
        params_dict = nepi_sdk.load_params_from_file(user_cfg_path,self.node_namespace)
        nepi_sdk.sleep(1)
        self.initCb(do_updates = True)

        self.status_msg.sys_debug_enabled = self.debug_enabled
        nepi_system.set_debug_mode(self.debug_enabled)

        self.status_msg.sys_admin_enabled = self.admin_enabled
        nepi_system.set_admin_mode(self.admin_enabled)

        self.status_msg.sys_admin_restricted = self.admin_restricted
        nepi_system.set_admin_restricted(self.admin_restricted)


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

    
        # Want to update the op_environment (from param server) through the whole system once at
        # start-up, but the only reasonable way to do that is to delay long enough to let all nodes start
        self.msg_if.pub_warn("Updating From Param Server")
        self.initConfig()
    


        # Crate system status pub
        self.msg_if.pub_warn("Starting System Status Messages")
        nepi_sdk.start_timer_process(self.STATUS_PERIOD, self.publishStatusCb)
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
            self.storage_folder = self.node_if.get_param("storage_folder")
            
            self.auto_switch_rootfs_on_new_img_install = nepi_sdk.get_param(
                "~auto_switch_rootfs_on_new_img_install", self.auto_switch_rootfs_on_new_img_install)

            self.first_rootfs = nepi_sdk.get_param(
                "~first_rootfs", self.first_rootfs)

            # nepi_storage has some additional logic
            self.getNEPIStorageDevice()
            
            self.new_img_staging = nepi_sdk.get_param(
                "~new_img_staging", self.new_img_staging)

            self.new_img_staging_removable = nepi_sdk.get_param(
                "~new_img_staging_removable", self.new_img_staging_removable)

            self.emmc_device = self.node_if.get_param("emmc_device")

            self.usb_device = self.node_if.get_param("usb_device")

            self.sd_card_device = self.node_if.get_param("sd_card_device")

            self.ssd_device = self.node_if.get_param("ssd_device")



    
    def initCb(self, do_updates = False):
        if self.node_if is not None:
            self.debug_enabled = self.node_if.get_param('debug_enabled')
            self.admin_enabled = self.node_if.get_param('admin_enabled')
            self.admin_restricted = self.node_if.get_param('admin_restricted')
        if do_updates == True:
            pass
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
        resp.new_sys_img_staging = self.get_friendly_name(self.new_img_staging)
        resp.new_sys_img_staging_free_mb = self.nepi_image.getPartitionFreeByteCount(self.new_img_staging) / BYTES_PER_MEGABYTE

        # Don't query anything if we are in the middle of installing a new image
        if self.installing_new_image:
            resp.new_sys_img = self.new_img_file
            resp.new_sys_img_version = self.new_img_version
            resp.new_sys_img_size_mb = self.new_img_filesize / BYTES_PER_MEGABYTE
            return resp
        
        # At this point, not currently installing, so clear any previous query failed message so the status update logic below will work
        self.status_msg.sys_img_update_status = ""

        (status, err_string, self.new_img_files, self.new_img_versions, self.new_img_filesizes) = self.nepi_image.checkForNewImagesAvailable(
            self.new_img_staging, self.new_img_staging_removable)
        if status is False:
            self.msg_if.pub_warn("Unable to update software status: " + err_string)
            resp.new_sys_img = 'query failed'
            resp.new_sys_img_version = 'query failed'
            resp.new_sys_img_size_mb = 0
            self.status_msg.sys_img_update_status = 'query failed'
            return resp
        
        # Update the response
        success = False
        if self.new_img_files:
            if len(self.new_img_files) > 0:
                
                #####
                self.selected_new_img=self.new_img_files[0]
                #####
                self.status_msg.sys_img_update_options= ['None'] + self.new_img_files
                sel_new_img = copy.deepcopy(self.selected_new_img)
                sel_new_ind = self.new_img_files.index(sel_new_img)
                if sel_new_ind != -1:
                    resp.new_sys_img = self.new_img_files[sel_new_ind]
                    resp.new_sys_img_version = self.new_img_versions[sel_new_ind]
                    resp.new_sys_img_size_mb = self.new_img_filesizes[sel_new_ind] / BYTES_PER_MEGABYTE
                    self.status_msg.sys_img_update_status = "ready to install"
                    success = True
        if success == False:
            self.status_msg.sys_img_update_options = ["None"]
            self.status_msg.sys_img_update_selected = "None"
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
        response.degug_enabled = self.debug_enabled
        return response

    def provide_admin_status(self, req):
        response = AdminQueryResponse()
        response.admin_enabled = self.admin_enabled
        response.admin_restrict_options = self.ADMIN_RESTRICT_OPTIONS
        response.admin_restricted = self.admin_restricted
        return response

    def provide_system_status(self, req):
        response = SystemStatusQueryResponse()
        response.system_status = self.status_msg
        #self.msg_if.pub_warn("Returning status query response: " + str(response))
        return response

    def publishStatusCb(self, event):
        self.publish_status()
        
    def publish_status(self):
        # Populate the rest of the message contents
        # Temperature(s)
        self.update_temperatures()

        # Disk usage
        self.update_storage()

        # Update sys status and params if needed
        self.status_msg.sys_debug_enabled = self.debug_enabled
        self.status_msg.sys_admin_enabled = self.admin_enabled
        self.status_msg.sys_admin_restricted = self.admin_restricted
        # Now publish it
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', self.status_msg)

        # Always clear info strings after publishing
        del self.status_msg.info_strings[:]

    def setSaveStatusCb(self, save_msg):
        self.status_msg.save_all_enabled = save_msg.data

    def enableDebugCb(self, msg):
        self.debug_enabled = msg.data
        self.status_msg.sys_debug_enabled = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('debug_enabled',msg.data)
            self.node_if.save_config()

    def enableAdminCb(self, msg):
        self.admin_enabled = msg.data
        self.status_msg.sys_admin_enabled = msg.data
        self.publish_status()
        #if self.node_if is not None:
        #    self.node_if.set_param('admin_enabled',msg.data)
        #    self.node_if.save_config()


    def setAdminRestrictedCb(self, msg):
        restricted = []
        for key in msg.data:
            if key in ADMIN_RESTRICT_OPTIONS:
                restricted.appen(key)
        self.admin_restricted = restricted
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('admin_restricted',restricted)
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
            if subdir not in self.STORAGE_CHECK_SKIP_LIST:
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

    def setOpEnvCb(self, msg):
        if (msg.data != OpEnvironmentQueryResponse.OP_ENV_AIR) and (msg.data != OpEnvironmentQueryResponse.OP_ENV_WATER):
            self.msg_if.pub_warn(
                "Setting environment parameter to a non-standard value: " + str(msg.data))
        nepi_sdk.set_param("~op_environment", msg.data)

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
        self.msg_if.pub_warn("Device ID Updated - Requires device reboot")
        self.add_info_string(
            "Device ID updated - Requires device reboot", StampedString.PRI_ELEVATED)
        self.node_if.save_config()

    def systemErrorCb(self, msg):
        self.add_info_string(msg.data, StampedString.PRI_HIGH)

    def receive_sw_update_progress(self, progress_val):
        self.status_msg.sys_img_update_progress = progress_val
        if progress_val > 0 and progress_val < 1:
            self.status_msg.sys_img_update_status = 'flashing'

    def receive_archive_progress(self, progress_val):
        self.status_msg.sys_img_archive_progress = progress_val
    
    def installNewImageCb(self, msg):
        if self.installing_new_image:
            self.msg_if.pub_warn("New image is already being installed")
            return

        decompressed_img_filename = msg.data
        self.status_msg.sys_img_update_status = 'flashing'
        self.installing_new_image = True
        self.install_status = True
        self.install_status, err_msg = self.nepi_image.writeImage(self.new_img_staging, decompressed_img_filename, self.inactive_rootfs, 
                                                     do_slow_transfer=False, progress_cb=self.receive_sw_update_progress)

        # Finished installing
        self.installing_new_image = False
        if self.install_status is False:
            self.msg_if.pub_warn("Failed to flash image: " + err_msg)
            self.status_msg.sys_img_update_status = 'failed'
            return
        else:
            self.msg_if.pub_info("Finished flashing new image to inactive rootfs")
            self.status_msg.sys_img_update_status = 'complete - needs rootfs switch and reboot'

        # Check and repair the newly written filesystem as necessary
        self.install_status, err_msg = self.nepi_image.checkAndRepairPartition(self.inactive_rootfs)
        if self.install_status is False:
            self.msg_if.pub_warn("Newly flashed image has irrepairable filesystem issues: ", err_msg)
            self.status_msg.sys_img_update_status = 'failed - fs errors'
            return
        else:
            self.msg_if.pub_info("New image filesystem checked and repaired (as necessary)")

        # Do automatic rootfs switch if so configured
        if self.auto_switch_rootfs_on_new_img_install:
            if self.in_container == True:
                status, err_msg = self.nepi_image.switchActiveAndInactivePartitionsJetson()
            elif self.rootfs_ab_scheme == 'nepi':
                status, err_msg = self.nepi_image.switchActiveAndInactivePartitions(self.first_rootfs)
            elif self.rootfs_ab_scheme == 'jetson':
                status, err_msg = self.nepi_image.switchActiveAndInactivePartitionsJetson()
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
        if self.in_container == True:
            status, err_msg = self.nepi_image.switchActiveAndInactivePartitionsJetson()
        elif self.rootfs_ab_scheme == 'nepi':
            status, err_msg = self.nepi_image.switchActiveAndInactivePartitions(self.first_rootfs)
        elif self.rootfs_ab_scheme == 'jetson':
            status, err_msg = self.nepi_image.switchActiveAndInactivePartitionsJetson()
        else:
            err_msg = "Unknown ROOTFS A/B Scheme"
            status = False
            
        if status is False:
            self.msg_if.pub_warn("Failed to switch active/inactive rootfs: " + err_msg)
            return

        self.status_msg.warnings.flags[WarningFlags.ACTIVE_INACTIVE_ROOTFS_STALE] = True
        self.msg_if.pub_warn(
            "Switched active and inactive rootfs. Must reboot system for changes to take effect")

    def archiveImageCb(self, msg):
        if self.archiving_inactive_image is True:
            self.msg_if.pub_warn("Already in the process of archiving image")
            return
        fw_str = self.system_defs_msg.inactive_rootfs_fw_version
        fw_str = fw_str.replace('.','p')
        fw_str = fw_str.replace(' ','_')
        fw_str = fw_str.replace('/','_')
        now = datetime.datetime.now()
        backup_file_basename = 'nepi_' + fw_str + now.strftime("_%Y_%m-%d-%H%M%S") + '.img.raw'
        self.msg_if.pub_warn("Archiving inactive rootfs to filename: -" + backup_file_basename)
        self.status_msg.sys_img_archive_status = 'archiving'
        self.status_msg.sys_img_archive_filename = backup_file_basename

        # Transfers to USB seem to have trouble with the standard block size, so allow those to proceed at a lower
        # block size
        slow_transfer = True if self.usb_device in self.new_img_staging else False
                
        self.archiving_inactive_image = True
        status, err_msg = self.nepi_image.archiveInactiveToStaging(
            self.inactive_rootfs, self.new_img_staging, backup_file_basename, slow_transfer, progress_cb = self.receive_archive_progress)
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
    
    def get_friendly_name(self, devfs_name):
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

        statvfs = os.statvfs(self.storage_folder)
        self.system_defs_msg.disk_capacity = statvfs.f_frsize * statvfs.f_blocks / \
            BYTES_PER_MEGABYTE     # Size of data filesystem in Megabytes

        # Gather some info about ROOTFS A/B configuration
        status = False
        if self.in_container == True:
            self.system_defs_msg.first_rootfs = "container"
            (status, err_msg, ab_fs_dict) = self.nepi_image.getRootfsABStatusJetson()
        else:
            if self.rootfs_ab_scheme == 'nepi':
                self.system_defs_msg.first_rootfs = self.get_friendly_name(self.first_rootfs)
                (status, err_msg, ab_fs_dict) = self.nepi_image.getRootfsABStatus(self.first_rootfs)
            elif self.rootfs_ab_scheme == 'jetson':
                self.system_defs_msg.first_rootfs = 'N/A'
                (status, err_msg, ab_fs_dict) = self.nepi_image.getRootfsABStatusJetson()
            else:
                self.msg_if.pub_warn("Failed to identify the ROOTFS A/B Scheme... cannot update A/B info and status")

        if status is True:
            self.msg_if.pub_warn("Got Rootfs Dict: " + str(ab_fs_dict))
            self.system_defs_msg.active_rootfs = self.get_friendly_name(ab_fs_dict['active_part_device'])

            self.system_defs_msg.active_rootfs_size_mb = self.nepi_image.getPartitionByteCount(ab_fs_dict[
                'active_part_device']) / BYTES_PER_MEGABYTE
            
            self.inactive_rootfs = ab_fs_dict[
                'inactive_part_device']
            self.system_defs_msg.inactive_rootfs = self.get_friendly_name(self.inactive_rootfs)

            self.system_defs_msg.inactive_rootfs_size_mb = self.nepi_image.getPartitionByteCount(self.inactive_rootfs) / BYTES_PER_MEGABYTE
            
            self.system_defs_msg.inactive_rootfs_fw_version = ab_fs_dict[
                'inactive_part_fw_version']
            self.system_defs_msg.max_boot_fail_count = ab_fs_dict[
                'max_boot_fail_count']
        else:
            self.msg_if.pub_warn(
                "Unable to gather ROOTFS A/B system definitions: " + err_msg)
            self.system_defs_msg.active_rootfs = "Unknown"
            self.system_defs_msg.inactive_rootfs = "Unknown"
            self.inactive_rootfs = "Unknown"
            self.system_defs_msg.inactive_rootfs_fw_version = "Unknown"
            self.system_defs_msg.max_boot_fail_count = 0

        for i in self.system_defs_msg.temperature_sensor_names:
            self.status_msg.temperatures.append(0.0)

	    # TODO: Should this be queried somehow e.g., from the param server
        self.status_msg.save_all_enabled = False

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
    



    



if __name__ == '__main__':
    SysMgr = SystemMgrNode()
