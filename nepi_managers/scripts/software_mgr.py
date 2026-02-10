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
# Software
from nepi_sdk import nepi_software
from nepi_sdk import nepi_docker

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_interfaces.msg import MgrSoftwareStatus                           
from nepi_interfaces.srv import  SoftwareStatusQuery, SoftwareStatusQueryRequest, SoftwareStatusQueryResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import StatesIF, TriggersIF, SaveDataIF

BYTES_PER_MEGABYTE = 2**20



NEPI_FOLDER='/opt/nepi'

class SoftwareMgrNode():
    
    STATUS_PERIOD = 1.0  # TODO: Configurable update period?

    DISK_FULL_MARGIN_MB = 250  # MB TODO: Configurable?

    FW_VERSION_PATH = NEPI_FOLDER + "/nepi_engine/etc/fw_version.txt"

    STATES_DICT = dict()



    node_if = None
    status_msg = MgrSoftwareStatus()

    
    folders_uid = 'nepi' #1000 # default to nepi
    folders_gid = 'nepi'

    # Shorter period for more responsive updates
    # disk_usage_deque = deque(maxlen=3)


    auto_switch_rootfs_on_new_img_install = True
    sw_update_progress = ""
    selected_new_img="none_detected"
    installing_new_image = False
    new_img_file = ""
    new_img_version = ""
    new_img_filesize = 0
    install_status = True
    saving_image = False

    in_container = False

    install_img = ''
    install_img_version = ''
    install_img_size = ''

    new_img_staging = None

    init_complete = False
    ready = False

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "software_mgr" # Can be overwitten by luanch command
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


        ##############################
        # Get for System Folders
        self.msg_if.pub_info("Waiting for system folders")
        system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got system folders: " + str(system_folders))
        self.nepi_config = nepi_system.get_nepi_config()

        self.msg_if.pub_warn("Got System Config: " + str(self.nepi_config))
        if self.nepi_config is None:
            self.nepi_config = dict()
        if len(self.nepi_config.keys()) == 0:
            self.msg_if.pub_warn("Failed to Read NEPI config file")
            nepi_sdk.signal_shutdown("Shutting Down: Failed to Read NEPI config file")
            return



        # Gather owner and group details for storage mountpoint
        # stat_info = os.stat(self.storage_folder)
        # self.folders_uid = stat_info.st_uid
        # self.folders_gid = stat_info.st_gid
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

        # check_path=self.nepi_config['NEPI_IMPORT_PATH']
        # if check_path not in self.REQD_STORAGE_SUBDIRS:
        #     self.REQD_STORAGE_SUBDIRS.append(check_path)
        # check_path=self.nepi_config['NEPI_EXPORT_PATH']
        # if check_path not in self.REQD_STORAGE_SUBDIRS:
        #     self.REQD_STORAGE_SUBDIRS.append(check_path)

        #nepi_system.set_nepi_config(self.nepi_config)


        self.in_container = self.nepi_config['NEPI_IN_CONTAINER'] == 1



        self.first_rootfs = self.nepi_config['NEPI_FS_DEVICE']
        self.has_ab_fs = self.nepi_config['NEPI_AB_FS'] == 1
        self.nepi_storage_device = self.nepi_config['NEPI_STORAGE_DEVICE']
        self.new_img_staging = self.nepi_config['NEPI_STORAGE_DEVICE']
        self.new_img_staging_removable = False
        self.new_img_folder = self.nepi_config['NEPI_IMPORT_PATH']

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


    
        self.msg_if.pub_warn("Waiting for Config Mgr")
        config_folders = nepi_system.get_config_folders()    



        self.status_msg.inactive_rootfs_fw_version = "unknown"
       
        self.msg_if.pub_warn("Updating Rootfs Scheme")
        # Need to identify the rootfs scheme because it is used in init_msgs()
        self.rootfs_ab_scheme = self.nepi_image.identifyRootfsABScheme()
        self.msg_if.pub_warn("Got Rootfs Scheme: " + self.rootfs_ab_scheme)
        





        # self.msg_if.pub_warn("Checking User Storage Partition")
        # # First check that the storage partition is actually mounted
        # if not os.path.ismount(self.storage_folder):
        #     self.msg_if.pub_warn("NEPI Storage partition is not mounted... attempting to mount")
        #     ret, msg = self.nepi_image.mountPartition(self.nepi_storage_device, self.storage_folder)
        #     if ret is False:
        #         self.msg_if.pub_warn("Unable to mount NEPI Storage partition... system may be dysfunctional")
        #         #return False # Allow it continue on local storage...

        # self.msg_if.pub_warn("Starting Node IF Setup")    
        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        # self.CFGS_DICT = {
        #     'reset_callback': self.resetCb,
        #     'factory_reset_callback': self.factoryResetCb,
        #     'init_configs': True,
        #     'namespace': self.node_namespace
        # }


        # Params Config Dict ####################
        self.PARAMS_DICT = {
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
            }
        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'software_status_query': {
                'namespace': self.base_namespace + '/softaware_mgr',
                'topic': 'system_status_query',
                'srv': SoftwareStatusQuery,
                'req': SoftwareStatusQueryRequest(),
                'resp': SoftwareStatusQueryResponse(),
                'callback': self.provide_software_status
            }
        }



        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.base_namespace + '/softaware_mgr',
                'topic': 'status',
                'msg': MgrSoftwareStatus,
                'qsize': 1,
                'latch': True
            }
        }  

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {     
            'select_nepi_image': {
                'namespace': self.base_namespace,
                'topic': 'select_nepi_image',
                'msg': String,
                'qsize': 1,
                'callback': self.selectImageCb, 
                'callback_args': ()
            },
            'install_nepi_image': {
                'namespace': self.base_namespace,
                'topic': 'install_nepi_image',
                'msg': String,
                'qsize': 1,
                'callback': self.installImageCb, 
                'callback_args': ()
            },
            'switch_nepi_image': {
                'namespace': self.base_namespace,
                'topic': 'switch_nepi_image',
                'msg': Empty,
                'qsize': None,
                'callback': self.handle_switch_active_inactive_image, 
                'callback_args': ()
            },
            'save_nepi_image': {
                'namespace': self.base_namespace,
                'topic': 'save_nepi_image',
                'msg': Empty,
                'qsize': 1,
                'callback': self.saveImageCb, 
                'callback_args': ()
            },
            'save_data_prefix': {
                'namespace': self.base_namespace,
                'topic': 'save_data_prefix',
                'msg': String,
                'qsize': None,
                'callback': self.save_data_prefix_callback, 
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        # configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        wait_cfg_mgr = False,
                        msg_if = self.msg_if
        )

        #ready = self.node_if.wait_for_ready()
        nepi_sdk.wait()



        self.msg_if.pub_warn("Completing Initialization Processes") 
        ########################
        # Complete Initialization


               
    
        # Want to update the op_environment (from param server) through the whole system once at
        # start-up, but the only reasonable way to do that is to delay long enough to let all nodes start
        self.msg_if.pub_warn("Updating From Param Server")
        self.initConfig()
    


        # Crate system status pub
        self.msg_if.pub_warn("Starting System Status Messages")
        nepi_sdk.start_timer_process(1.0, self.publishStatusCb)
        # nepi_sdk.start_timer_process(1, self.updateTopicsServicesCb, oneshot = True)
        # nepi_sdk.start_timer_process(1, self.updateSoftwareStatusCb, oneshot = True)
        # nepi_sdk.start_timer_process(5, self.updateDockerCb)
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
            # self.getNEPIStorageDevice()
            
            if self.in_container==True:
                self.new_img_staging = self.new_img_folder

                self.new_img_staging_removable = False
            else:
                self.new_img_staging = self.new_img_staging

                self.new_img_staging_removable = self.new_img_staging_removable

            self.emmc_device = self.node_if.get_param("emmc_device")

            self.usb_device = self.node_if.get_param("usb_device")

            self.sd_card_device = self.node_if.get_param("sd_card_device")

            self.ssd_device = self.node_if.get_param("ssd_device")

            self.init_complete = True

            self.init_msgs()

            # Call the method to update s/w status once internally to prime the status fields now that we have all the parameters
            # established
            self.update_software_status()


    # def getStatesDictCb(self):
    #     return self.STATES_DICT

    # def systemTriggersCb(self,msg):
    #     trigger_name = msg.name
    #     if trigger_name not in self.triggers_list:
    #         self.triggers_list.append(trigger_name)

    # def triggersStatusPubCb(self,timer):
    #     triggers_name_list = []
    #     has_triggered_list = []
    #     msg = SystemTriggersStatus()
    #     namespaces = nepi_triggers.get_triggers_publisher_namespaces()
    #     if namespaces is not None:
    #         for namespace in namespaces:
    #             topic = os.path.join(namespace,'system_triggers_query')
    #             if topic not in self.service_dict.keys():
    #                 service = nepi_sdk.create_service(topic,SystemTrigger)
    #                 if service is not None:
    #                     self.service_dict[topic] = service
    #                     time.sleep(1)
    #             if topic in self.service_dict.keys():
    #                 service = self.service_dict[topic]
    #                 req = SystemTriggersQueryRequest()
    #                 try:
    #                     resp = nepi_sdk.call_service(service, req)
    #                     triggers_list = resp.triggers_list
    #                     for trigger in triggers_list:
    #                         trigger_name = trigger.name
    #                         if trigger_name not in triggers_name_list:
    #                             triggers_name_list.append(trigger_name) 
    #                 except:
    #                     self.msg_if.pub_info(":" + self.class_name + ": Failed to call service: " + str(e))

    #         for trigger_name in triggers_name_list:
    #             has_triggered = trigger_name in self.triggers_list
    #             has_triggered_list.append(has_triggered)
    #         self.triggers_list = [] # Clear List
    #         msg = nepi_triggers.create_triggers_status_msg(triggers_name_list,has_triggered_list)
    #         if self.node_if is not None:
    #             self.node_if.publish_pub('triggers_status_pub', msg)
    #     nepi_sdk.start_timer_process(self.triggers_status_interval, self.triggersStatusPubCb, oneshot = True)



    # def statesStatusPubCb(self,timer):
    #     states_list = []
    #     msg = SystemStatesStatus()
    #     namespaces = nepi_states.get_states_publisher_namespaces()
    #     if namespaces is not None:
    #         for namespace in namespaces:
    #             topic = os.path.join(namespace,'system_states_query')
    #             if topic not in self.service_dict.keys():
    #                 service = nepi_sdk.create_service(topic,SystemState)
    #                 if service is not None:
    #                     self.service_dict[topic] = service
    #                     time.sleep(1)
    #             if topic in self.service_dict.keys():
    #                 service = self.service_dict[topic]
    #                 req = SystemStatesQueryRequest()
    #                 try:
    #                     resp = nepi_sdk.call_service(service, req)
    #                     for state in resp.states_list:
    #                         states_list.append(state)
    #                 except:
    #                     self.msg_if.pub_info(":" + self.class_name + ": Failed to call service: " + str(e))

    #         try:
    #             msg = nepi_states.create_states_status_msg(states_list)
    #         except Exception as e:
    #             self.msg_if.pub_info(":" + self.class_name + ": Failed to create status msg: " + str(e))
    #         if self.node_if is not None:
    #             self.node_if.publish_pub('states_status_pub', msg)
    #     nepi_sdk.start_timer_process(self.states_status_interval, self.statesStatusPubCb, oneshot = True)



    def get_fw_rev(self):
        if (os.path.exists(self.FW_VERSION_PATH) and (os.path.getsize(self.FW_VERSION_PATH) > 0)):
            with open(self.FW_VERSION_PATH, "r") as f:
                fw_version = f.readline().strip()
                return fw_version

        return "UNSPECIFIED"

 

    # def updateDockerCb(self, event):
    #     nepi_system.update_nepi_docker_config("NEPI_FAIL_COUNT" , 0)
 

    def updateSoftwareStatusCb(self, event):
        self.update_software_status()
        nepi_sdk.start_timer_process(5, self.updateSoftwareStatusCb, oneshot = True)


    def receive_sw_update_progress(self, progress_val):
        self.status_msg.sys_img_update_progress = progress_val
        if progress_val > 0 and progress_val < 1:
            self.status_msg.sys_img_update_status = 'flashing'

    def receive_archive_progress(self, progress_val):
        self.status_msg.sys_img_archive_progress = progress_val

    def selectImageCb(self, msg):
        selected_new_img = msg.data
        if selected_new_img in self.new_img_files:
            self.selected_new_img = selected_new_img
            self.update_software_status() 
            self.publish_status()       
    
    def installImageCb(self, msg):
        if self.installing_new_image:
            self.msg_if.pub_warn("New image is already being installed")
            return

        # Install Image
        img_filename = msg.data
        self.selected_new_img=img_filename
        self.status_msg.sys_img_update_status = 'flashing'
        self.installing_new_image = True
        self.install_status = True
    
        
        self.install_status, err_msg = self.nepi_image.installImage(self.new_img_staging, img_filename, self.inactive_rootfs, 
                                                     do_slow_transfer=False, progress_cb=self.receive_sw_update_progress)

        # Finished installing
        self.installing_new_image = False
        if self.install_status is False:
            self.msg_if.pub_warn("Failed to flash image: " + err_msg)
            self.status_msg.sys_img_update_status = 'failed'
            return
        else:
            self.msg_if.pub_info("Finished flashing new image")
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
                status, err_msg = self.nepi_image.switchActiveAndInactiveContainers()
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
    
    def handle_switch_active_inactive_image(self, msg):
        self.msg_if.pub_warn("Received switch active/inactive nepi images msg")
        if self.in_container == True:
            self.msg_if.pub_warn("In Container")
            status, err_msg = self.nepi_image.switchActiveAndInactiveContainers()
        elif self.rootfs_ab_scheme == 'nepi':
            self.msg_if.pub_warn("In Nepi")
            status, err_msg = self.nepi_image.switchActiveAndInactivePartitions(self.first_rootfs)
        elif self.rootfs_ab_scheme == 'jetson':
            self.msg_if.pub_warn("In Jetson")
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

    def saveImageCb(self, msg):
        if self.saving_image is True:
            self.msg_if.pub_warn("Already in the process of saving image")
            return
        
        if self.in_container == True:
            info_dict=self.nepi_image.getContainerInfo('Active')
            fw_str = self.get_fw_rev()
            self.nepi_config = nepi_system.update_nepi_system_config('NEPI_VERSION',fw_str)
            info_dict['version']=fw_str
            now = datetime.datetime.now()
            backup_file_basename = 'nepi_' + fw_str + now.strftime("_%Y_%m-%d-%H%M%S")
            self.msg_if.pub_warn("Saving NEPI File System to filename: -" + backup_file_basename)
            self.status_msg.sys_img_archive_status = 'archiving'
            self.status_msg.sys_img_archive_filename = backup_file_basename


        else:
            fw_str = self.status_msg.inactive_rootfs_fw_version
            fw_str = fw_str.replace('.','p')
            fw_str = fw_str.replace(' ','_')
            fw_str = fw_str.replace('/','_')
            now = datetime.datetime.now()
            backup_file_basename = 'nepi_' + fw_str + now.strftime("_%Y%m%d")
            self.msg_if.pub_warn("Archiving inactive rootfs to filename: -" + backup_file_basename)
            self.status_msg.sys_img_archive_status = 'archiving'
            self.status_msg.sys_img_archive_filename = backup_file_basename

        # Transfers to USB seem to have trouble with the standard block size, so allow those to proceed at a lower
        # block size
        slow_transfer = True if self.usb_device in self.new_img_staging else False
                
        self.saving_image = True
        status, err_msg = self.nepi_image.saveImage(
            self.inactive_rootfs, self.new_img_staging, backup_file_basename, slow_transfer, progress_cb = self.receive_archive_progress)
        self.saving_image = False

        if status is False:
            self.msg_if.pub_warn("Failed to save image file: " + err_msg)
            self.status_msg.sys_img_archive_status = 'failed'
        else:
            self.msg_if.pub_info("Finished saving image file")
            self.status_msg.sys_img_archive_status = 'archive complete'
    

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
        if self.init_complete == True:
            fw_str = self.get_fw_rev()
            self.nepi_config = nepi_system.update_nepi_system_config('NEPI_VERSION',fw_str)
            self.status_msg.firmware_version = fw_str

            # Gather some info about ROOTFS A/B configuration
            status = False
            if self.in_container == True:
                self.status_msg.first_rootfs = "container"
                (status, err_msg, ab_fs_dict) = self.nepi_image.getRootfsABStatus()
            else:
                if self.rootfs_ab_scheme == 'nepi':
                    self.status_msg.first_rootfs = self.get_friendly_name(self.first_rootfs)
                    (status, err_msg, ab_fs_dict) = self.nepi_image.getRootfsABStatus(self.first_rootfs)
                elif self.rootfs_ab_scheme == 'jetson':
                    self.status_msg.first_rootfs = 'N/A'
                    (status, err_msg, ab_fs_dict) = self.nepi_image.getRootfsABStatusJetson()
                else:
                    self.msg_if.pub_warn("Failed to identify the ROOTFS A/B Scheme... cannot update A/B info and status")

            if status is True:
                self.msg_if.pub_warn("Got Rootfs Dict: " + str(ab_fs_dict))
                self.status_msg.active_rootfs = self.get_friendly_name(ab_fs_dict['active_part_device'])

                self.status_msg.active_rootfs_size_mb = self.nepi_image.getPartitionByteCount(ab_fs_dict[
                    'active_part_device']) / BYTES_PER_MEGABYTE
                
                self.inactive_rootfs = ab_fs_dict[
                    'inactive_part_device']
                self.status_msg.inactive_rootfs = self.get_friendly_name(self.inactive_rootfs)

                self.status_msg.inactive_rootfs_size_mb = self.nepi_image.getPartitionByteCount(self.inactive_rootfs) / BYTES_PER_MEGABYTE
                
                self.status_msg.inactive_rootfs_fw_version = ab_fs_dict[
                    'inactive_part_fw_version']
                self.status_msg.max_boot_fail_count = ab_fs_dict[
                    'max_boot_fail_count']
            else:
                self.msg_if.pub_warn(
                    "Unable to gather ROOTFS A/B system definitions: " + err_msg)
                self.status_msg.active_rootfs = "Unknown"
                self.status_msg.inactive_rootfs = "Unknown"
                self.inactive_rootfs = "Unknown"
                self.status_msg.inactive_rootfs_fw_version = "Unknown"
                self.status_msg.max_boot_fail_count = 0



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
    




       
    def update_software_status(self):
        if self.init_complete == True:
            # self.msg_if.pub_warn("Entering Provide Software Update Status")
            self.status_msg.new_sys_img_staging = self.get_friendly_name(self.new_img_staging)
            self.status_msg.new_sys_img_staging_free_mb = self.nepi_image.getPartitionFreeByteCount(self.new_img_staging) / BYTES_PER_MEGABYTE
            # self.msg_if.pub_warn("Got New Img Staging Name: " + str(self.status_msg.new_sys_img_staging))
            # self.msg_if.pub_warn("Got Partition Free MB: " + str(self.status_msg.new_sys_img_staging_free_mb))
            # Don't query anything if we are in the middle of installing a new image
            if self.installing_new_image:
                return self.status_msg
            
        
            # At this point, not currently installing, so clear any previous query failed message so the status update logic below will work
            self.install_img = ''
            self.install_img_version = ''
            self.install_img_size = ''
            self.status_msg.sys_img_update_status = ""

            if self.in_container==True:
                self.new_img_staging = self.new_img_folder

                self.new_img_staging_removable = False
            else:
                self.new_img_staging = self.new_img_staging

                self.new_img_staging_removable = self.new_img_staging_removable

            (status, err_string, new_img_files, new_img_versions, new_img_filesizes) = self.nepi_image.checkForNewImagesAvailable(
                self.new_img_staging, self.new_img_staging_removable)
            #self.msg_if.pub_warn("Availible files List: " + str(new_img_files) + " " + err_string)
            if status is False:
                self.msg_if.pub_warn("Unable to update software status: " + err_string)
                self.status_msg.new_sys_img = 'query failed'
                self.status_msg.new_sys_img_version = 'query failed'
                self.status_msg.new_sys_img_size_mb = 0
                self.status_msg.sys_img_update_status = 'query failed'
                return self.status_msg

            
            # Update the response
            success = False
            selected_new_img="none_detected"
            sel_new_img = copy.deepcopy(self.selected_new_img)
            if new_img_files:
                if len(new_img_files) > 0:
                    self.status_msg.sys_img_update_options= new_img_files
                    
                    if sel_new_img in new_img_files:
                        sel_new_ind = new_img_files.index(sel_new_img)
                    else:
                        sel_new_ind=0
                        self.selected_new_img=new_img_files[0]

                    img_size_gigabytes=int(new_img_filesizes[sel_new_ind]) / (1024**3)
                    self.status_msg.new_sys_img = new_img_files[sel_new_ind]
                    self.status_msg.new_sys_img_version = new_img_versions[sel_new_ind]
                    self.status_msg.new_sys_img_size_mb = img_size_gigabytes
                    self.status_msg.sys_img_update_status = "ready to install"
                    success = True
            if success == False:
                self.selected_new_img="none_detected"
                self.status_msg.sys_img_update_options = ["none_detected"]
                self.status_msg.sys_img_update_selected = "None"
                self.status_msg.new_sys_img = 'none detected'
                self.status_msg.new_sys_img_version = 'none detected'
                self.status_msg.new_sys_img_size_mb = 0
                self.status_msg.sys_img_update_status = "no new image available"
                
        return self.status_msg

    #########################
    #### Software
    def provide_software_status(self, req):
        response = SoftwareStatusQueryResponse()
        response.software_status = self.status_msg
        #self.msg_if.pub_warn("Returning status query response: " + str(response))
        return response


    def publishStatusCb(self, event):
        self.publish_status()



    def publish_status(self):
        
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', self.status_msg)

    



if __name__ == '__main__':
    SoftwareMgr = SoftwareMgrNode()