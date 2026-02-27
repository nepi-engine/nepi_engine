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

# The config_mgr node serves as a bridge between the ROS param server and the filesystem.
# It provides the rest of the Numurus/ROS node set the ability to save and restore config.
# files with rudimentary coordination to reduce overloading file system during times of
# heavy updates.

import os
import time
import shutil
from pathlib import Path


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
 

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF


NEPI_ENV_PACKAGE = 'nepi_env'

NEPI_HOME_PATH = '/home/nepi'
NEPI_ETC_PATH = '/opt/nepi/etc'
NEPI_ENGINE_ETC_PATH = '/opt/nepi/nepi_engine/etc'
FACTORY_CFG_PATH = '/mnt/nepi_config/factory_cfg'
SYSTEM_CFG_PATH = '/mnt/nepi_config/system_cfg'
USER_CFG_PATH = '/mnt/nepi_storage/user_cfg'
CFG_SUFFIX = '.yaml'

SYSTEM_MGR_NODENAME = 'system_mgr'



class config_mgr(object):

    node_if = None
    config_folders = dict()
    config_folders['factory_cfg']=FACTORY_CFG_PATH
    config_folders['user_cfg']=USER_CFG_PATH
    config_folders['system_cfg']=SYSTEM_CFG_PATH

    save_disabled = False
    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "config_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        time.sleep(1)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################
        # Wait for System Manager
        #self.msg_if.pub_info("Waiting for 10 secs")
        #time.sleep(10)
        self.msg_if.pub_info("Waiting for system folders")
        folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got system folders: " + str(folders))
        for folder in self.config_folders.keys():
            if folder in folders.keys():
                self.config_folders[folder] = folders[folder]
        self.FACTORY_CFG_PATH = self.config_folders['factory_cfg']
        self.SYSTEM_CFG_PATH = self.config_folders['system_cfg']
        self.USER_CFG_PATH = self.config_folders['user_cfg']
        self.msg_if.pub_warn("Using config folders: " + str(self.config_folders))


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
        self.PARAMS_DICT = None


        # Services Config Dict ####################
        self.SRVS_DICT = None

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': Empty,
                'qsize': 1,
                'latch': True
            },
            'init_config': {
                'namespace': self.base_namespace,
                'topic': 'init_config',
                'msg': Empty,
                'qsize': 10,
                'latch': True
            }
        }  

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'save_params': {
                'namespace': self.base_namespace,
                'topic': 'save_params',
                'msg': String,
                'qsize': 5,
                'callback': self.saveParamsCb, 
                'callback_args': ()
            },
            'reset_params': {
                'namespace': self.base_namespace,
                'topic': 'reset_params',
                'msg': String,
                'qsize': 5,
                'callback': self.resetParamsCb, 
                'callback_args': ()
            },
            'save_params_all': {
                'namespace': self.base_namespace,
                'topic': 'save_params_all',
                'msg': String,
                'qsize': 5,
                'callback': self.saveParamsAllCb, 
                'callback_args': ()
            },
            'factory_save': {
                'namespace': self.base_namespace,
                'topic': 'factory_save',
                'msg': Empty,
                'qsize': 5,
                'callback': self.saveFactoryCb, 
                'callback_args': ()
            },
            'factory_reset': {
                'namespace': self.base_namespace,
                'topic': 'factory_reset',
                'msg': Empty,
                'qsize': 5,
                'callback': self.resetFactoryCb, 
                'callback_args': ()
            },
            'factory_clear': {
                'namespace': self.base_namespace,
                'topic': 'factory_clear',
                'msg': Empty,
                'qsize': 5,
                'callback': self.clearFactoryCb, 
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



        #########################################################
        ## Complete Initialization


        # Restore configurations
        success = self.sync_nepi_config()
        if success == True:
            self.msg_if.pub_warn("NEPI config files restored")
        
        nepi_sdk.start_timer_process(1, self.statusPubCb)
        ################
        # Save the current system config
        sys_ns = nepi_sdk.create_namespace(self.base_namespace,SYSTEM_MGR_NODENAME)
        self.save_params(self.USER_CFG_PATH, sys_ns)

        nepi_sdk.sleep(1)
        self.initCb(do_updates = True)
        #########################################################
        ## Initiation Complete

        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_sdk.spin()
        #########################################################

    def sysResetCb(self,reset_type):
        pass

    def initCb(self, do_updates = False):
      if self.node_if is not None:
        pass
      if do_updates == True:
        self.msg_if.pub_warn("Setting config folders param to: " + str(self.config_folders))
        nepi_system.set_config_folders(self.config_folders)
        nepi_sdk.set_param('config_folders',self.config_folders)
      self.publish_status()



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


    ####################################################

    def get_filename_from_namespace(self,namespace, all_config = False):
        base_namespace = nepi_sdk.get_base_namespace()
        namespace = namespace.replace(base_namespace + '/','')
        if namespace[-1] == '/':
            namespace = namespace[:-1]
        if all_config == True:
            namespace_parts = namespace.split('/',1)
            clean_namespace_parts = [x for x in namespace_parts if x]

            node_name = clean_namespace_parts[0]
            node_name_all = node_name.rsplit('_',1)[0] + '_ALL'
            #self.msg_if.pub_warn("Got clean all node_name for namespace: " + namespace + " : " + str(node_name_all))
            clean_namespace_parts[0] = node_name_all

            all_namespace = '/'.join(clean_namespace_parts)
            #self.msg_if.pub_warn("Got clean all node_name namespace for namespace: " + namespace + " : " + str(all_namespace))
            namespace = all_namespace
            
        if len(namespace) > 0:
            if namespace[0] == '/':
                namespace = namespace [1:]
        filename = namespace.replace('/','-') + CFG_SUFFIX
        return filename



    def get_config_pathname(self, cfg_path, namespace, all_config = False):
        filename = self.get_filename_from_namespace(namespace, all_config = all_config)
        
        # Ensure the path we report actually exists
        if not os.path.isdir(cfg_path):
            os.makedirs(cfg_path)

        pathname = os.path.join(cfg_path, filename)
        return pathname


    def update_from_file(self,file_pathname, namespace):
        if os.path.exists(file_pathname) == False:
            self.msg_if.pub_warn("Could not find params file for namespace: " + namespace  + " at " + file_pathname )
            return False
        else:
            self.msg_if.pub_warn("Loading Params for namespace: " + namespace  + " from file " + file_pathname )
            params_dict = None
            try:
                params_dict = nepi_sdk.load_params_from_file(file_pathname, namespace, log_name_list = [self.class_name])
                self.msg_if.pub_warn("Got Params for namespace: " + namespace  + " from file " + file_pathname  + " : " + str(params_dict.keys()), log_name_list = [self.class_name])
            except Exception as e:
                self.msg_if.pub_warn("Unable to load parameters from file " + file_pathname + " " + str(e))
                return False
            self.msg_if.pub_warn("Updated Params for namespace: " + namespace )
        return True

    ##################################

    def saveParamsCb(self,msg):
        namespace = msg.data
        #self.msg_if.pub_info("Got Save Params for namespace: " + namespace )
        self.save_params(namespace)

    def saveParamsAllCb(self,msg):
        namespace = msg.data
        #self.msg_if.pub_info("Got Save Params All for namespace: " + namespace )
        self.save_params(namespace, save_all = True)
    
    def save_params(self, namespace, save_all = False):
        success = False
        cfg_path = self.USER_CFG_PATH
        if os.path.exists(cfg_path) and self.save_disabled == False:
            config_pathname = self.get_config_pathname(cfg_path, namespace, all_config = save_all)
            #backup_pathname = os.path.dirname(config_pathname) + '/.' + os.path.basename(config_pathname)

            #self.msg_if.pub_info("Storing Params for namespace: " + namespace  + " in file " + config_pathname )
            # First, write to the user file
            nepi_sdk.save_params_to_file(config_pathname, namespace)
            #nepi_sdk.save_params_to_file(backup_pathname, namespace)
            #self.msg_if.pub_info("Params saved for namespace: " + namespace  + " in file " + config_pathname )
            success = True
        return success
    

    def resetParamsCb(self,msg):
        namespace = msg.data
        #self.msg_if.pub_info("Got Reset Params All for namespace: " + namespace )
        self.reset_params(namespace)


    def reset_params(self,namespace):
        # Restore saved param config if exists from first find in order (user,system,factory)
        config_folders = ['user_cfg','system_cfg','factory_cfg']
        success = False
        for folder in config_folders:
            #self.msg_if.pub_warn("Checking for Saved config for namespace: " + namespace + " folder: " + str(folder))
            if folder in self.config_folders.keys():
                restore_path = self.config_folders[folder]
                # Restore config if exits
                restore_pathname = self.get_config_pathname(restore_path, namespace)
                #self.msg_if.pub_warn("Checking for Saved config for namespace: " + namespace + " params file: " + str(restore_pathname))
                if nepi_system.supports_all_config(namespace) == True:
                    restore_pathname_all = self.get_config_pathname(restore_path, namespace, all_config = True)
                    self.msg_if.pub_warn("Checking for ALL config for namespace: " + namespace + " params file: " + str(restore_pathname_all))
                    if restore_pathname_all is not None:
                        if os.path.exists(restore_pathname_all):
                            restore_pathname = restore_pathname_all
                #self.msg_if.pub_warn("Checking for saved config for namespace: " + namespace + " params file: " + str(restore_pathname))
                success = False
                if os.path.exists(restore_pathname):
                    #self.msg_if.pub_warn("Loading config for namespace: " + namespace + " from: " + str(restore_pathname))
                    success = self.update_from_file(restore_pathname, namespace)
                    if success == True:
                        self.msg_if.pub_warn("Loaded saved config for namespace: " + namespace + " from: " + str(restore_pathname))
                        return success
                    else:
                        self.msg_if.pub_warn("Failed to load. Removing config file for namespace: " + namespace + " from: " + str(restore_pathname))
                        try:
                            os.remove(restore_pathname)
                        except Exception as e:
                            print(f"An error occurred: {e}")
        return success


    ###################################


    def saveFactoryCb(self,msg):
        source_folder = self.USER_CFG_PATH
        dest_folder = self.SYSTEM_CFG_PATH
        # Copy User Configs to System Configs
        #self.msg_if.pub_warn("Looking for dest config target path: " + dest_folder )
        if os.path.exists(dest_folder) == False:
            os.makedirs(dest_folder)
        #self.msg_if.pub_warn("Looking for source config target path: " + dest_folder )
        if os.path.exists(dest_folder) == True and os.path.exists(dest_folder) == True:
            self.msg_if.pub_warn("Syncing source to target config paths: " + source_folder + " : " + dest_folder )
            success = nepi_utils.rsync_folders(source_folder,dest_folder, folders = False)
            if success == True:
                self.msg_if.pub_warn("Synced source to target config paths: " + source_folder + " : " + dest_folder )

    def resetFactoryCb(self,msg):
        self.save_disabled = True
        clear_folder = self.USER_CFG_PATH
        nepi_utils.delete_files_in_folder(clear_folder,ext = CFG_SUFFIX)


    def clearFactoryCb(self,msg):
        clear_folder = self.SYSTEM_CFG_PATH
        nepi_utils.delete_files_in_folder(clear_folder,ext = CFG_SUFFIX)
        



    #####################################    



    def sync_nepi_config(self,config_folders = ['factory_cfg','system_cfg','user_cfg']): 
        success = False
        target_dir = NEPI_ENGINE_ETC_PATH
        for name in config_folders:
            if name in self.config_folders.keys():
                folder = os.path.join(self.config_folders[name],'etc')
                source_dir = os.path.join(folder,'etc')
                if os.path.exists(source_dir) == True and os.path.exists(target_dir) == True:
                    ret_etc = nepi_utils.rsync_folders(source_dir,target_dir)
                    if ret_etc == True:
                        success = True
        return success
               



    def statusPubCb(self,timer):
        self.publish_status()

    def publish_status(self):
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', Empty())

if __name__ == '__main__':
    config_mgr()
