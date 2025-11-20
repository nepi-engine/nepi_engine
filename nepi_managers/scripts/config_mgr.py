#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# The config_mgr node serves as a bridge between the ROS param server and the filesystem.
# It provides the rest of the Numurus/ROS node set the ability to save and restore config.
# files with rudimentary coordination to reduce overloading file system during times of
# heavy updates.

import os
import time
import errno
import time
import shutil


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
 

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.srv import ParamsReset, ParamsResetRequest, ParamsResetResponse

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
            'system_reset_callback': self.sysResetCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace': self.node_namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'factory_reset': {
                'namespace': self.base_namespace,
                'topic': 'factory_reset',
                'srv': ParamsReset,
                'req': ParamsResetRequest(),
                'resp': ParamsResetResponse(),
                'callback': self.factoryResetHandler
            },
            'system_reset': {
                'namespace': self.base_namespace,
                'topic': 'system_reset',
                'srv': ParamsReset,
                'req': ParamsResetRequest(),
                'resp': ParamsResetResponse(),
                'callback': self.systemResetHandler
            },
            'user_reset': {
                'namespace': self.base_namespace,
                'topic': 'user_reset',
                'srv': ParamsReset,
                'req': ParamsResetRequest(),
                'resp': ParamsResetResponse(),
                'callback': self.userResetHandler
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
            'save_nepi_config': {
                'namespace': self.base_namespace,
                'topic': 'save_nepi_config',
                'msg': Empty,
                'qsize': None,
                'callback': self.saveNepiCfgCb, 
                'callback_args': ()
            },
            'save_params': {
                'namespace': self.base_namespace,
                'topic': 'save_params',
                'msg': String,
                'qsize': None,
                'callback': self.saveParamsCb, 
                'callback_args': ()
            },
            'restore_factory_cfgs': {
                'namespace': self.base_namespace,
                'topic': 'restore_factory_cfgs',
                'msg': Empty,
                'qsize': None,
                'callback': self.restoreFactoryCfgsCb, 
                'callback_args': ()
            },
            'save_system_cfgs': {
                'namespace': self.base_namespace,
                'topic': 'save_system_cfgs',
                'msg': Empty,
                'qsize': None,
                'callback': self.saveSystemCfgsCb, 
                'callback_args': ()
            },
            'restore_system_cfgs': {
                'namespace': self.base_namespace,
                'topic': 'restore_system_cfgs',
                'msg': Empty,
                'qsize': None,
                'callback': self.restoreSystemCfgsCb, 
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

        # Save Factory if Empty

        #if os.path.isdir(self.FACTORY_CFG_PATH):
        #    empty = not os.listdir(self.FACTORY_CFG_PATH) 
        #    if empty == True:
        #        self.msg_if.pub_warn("Initializing Factory Config Folder")
        #        self.save_cfgs(self.FACTORY_CFG_PATH)


        # Restore configurations
        success = self.restore_cfgs()
        if success == True:
            self.msg_if.pub_warn("NEPI config files restored")
        #succes = self.save_cfgs(self.SYSTEM_CFG_PATH)
        #if success == True:
        #    self.msg_if.pub_warn("NEPI config files saved")

        nepi_sdk.start_timer_process(1, self.statusPubCb)
        ################
        # Save the current system config
        sys_ns = nepi_sdk.create_namespace(self.base_namespace,SYSTEM_MGR_NODENAME)
        self.save_params(self.USER_CFG_PATH, sys_ns)

        nepi_sdk.sleep(1)
        self.initCb(do_updates = True)
        #########################################################
        ## Initiation Complete
        self.msg_if.pub_warn("Setting config folders param to: " + str(self.config_folders))
        nepi_system.set_config_folders(self.config_folders)
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

    def get_filename_from_namespace(self,namespace):
        if namespace[-1] == '/':
            namespace = namespace[:-1]
        base_namespace = nepi_sdk.get_base_namespace()
        namespace = namespace.replace(base_namespace,'')
        if len(namespace) > 0:
            if namespace[0] == '/':
                namespace = namespace [1:]
        filename = namespace.replace('/','-')
        return filename

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

    def get_config_pathname(self, cfg_path, namespace):
        filename = self.get_filename_from_namespace(namespace)
        
        # Ensure the path we report actually exists
        if not os.path.isdir(cfg_path):
            os.makedirs(cfg_path)

        pathname = os.path.join(cfg_path, filename + CFG_SUFFIX)
        return pathname




    def saveParamsCb(self,msg):
        namespace = msg.data
        self.msg_if.pub_info("Got Save Params for namespace: " + namespace  + " in Folder " + self.USER_CFG_PATH )
        self.save_params(self.USER_CFG_PATH, namespace)
    
    def save_params(self, cfg_path, namespace):
        if os.path.exists(cfg_path):
            config_pathname = self.get_config_pathname(cfg_path, namespace)
            #backup_pathname = os.path.dirname(config_pathname) + '/.' + os.path.basename(config_pathname)

            self.msg_if.pub_info("Storing Params for namespace: " + namespace  + " in file " + config_pathname )
            # First, write to the user file
            nepi_sdk.save_params_to_file(config_pathname, namespace)
            #nepi_sdk.save_params_to_file(backup_pathname, namespace)
            self.msg_if.pub_info("Params saved for namespace: " + namespace  + " in file " + config_pathname )
            success = True
        return success

 
    def reset_params(self,namespace):
        # Restore saved param config if exists from first find in order (user,system,factory)
        config_folders = ['user_cfg','system_cfg','factory_cfg']
        success = False
        for key in config_folders:
            if key in self.config_folders.keys():
                restore_path = self.config_folders[key]
                # Restore config if exits
                restore_pathname = self.get_config_pathname(restore_path, namespace)
                #self.msg_if.pub_warn("Checking for saved config for namespace: " + namespace + " params file: " + str(restore_pathname))
                success = False
                if os.path.exists(restore_pathname):
                    self.msg_if.pub_warn("Loading for saved config for namespace: " + namespace + " from: " + str(restore_pathname))
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

    def save_nepi_cfg(self,cfg_path = None):
            if cfg_path is None:
                cfg_path = self.SYSTEM_CFG_PATH
            success = False
            if cfg_path != self.USER_CFG_PATH:
                # Save Save Files
                source_dir = NEPI_ETC_PATH
                target_dir = os.path.join(cfg_path,'etc')
                self.msg_if.pub_warn("Looking for dest config target path: " + target_dir )
                if os.path.exists(target_dir) == False:
                    os.makedirs(target_dir)
                self.msg_if.pub_warn("Looking for source config target path: " + source_dir )
                if os.path.exists(source_dir) == True and os.path.exists(target_dir) == True:
                    self.msg_if.pub_warn("Syncing source to target path: " + source_dir + " : " + str(target_dir) )
                    success = nepi_utils.rsync_folders(source_dir,target_dir)
            return success

 
    def save_cfgs(self,cfg_path = None):
        if cfg_path is None:
            cfg_path = self.SYSTEM_CFG_PATH
        success = False
        if cfg_path != self.USER_CFG_PATH:
            ret_etc = self.save_nepi_cfg(cfg_path)
            if cfg_path == self.SYSTEM_CFG_PATH:
                # Copy User Configs to System Configs
                source_dir = self.USER_CFG_PATH
                target_dir = cfg_path
                self.msg_if.pub_warn("Looking for dest config target path: " + target_dir )
                if os.path.exists(target_dir) == False:
                    os.makedirs(target_dir)
                self.msg_if.pub_warn("Looking for source config target path: " + source_dir )
                if os.path.exists(source_dir) == True and os.path.exists(target_dir) == True:
                    self.msg_if.pub_warn("Syncing source to target path: " + source_dir + " : " + str(target_dir) )
                    ret_user = nepi_utils.rsync_folders(source_dir,target_dir)
            if ret_etc == True or ret_user == True:
                success = True
        return success




    def saveFactoryCfgsCb(self,msg):
        self.save_factory_cfgs()

    def save_factory_cfgs(self):
        self.save_cfgs(self.FACTORY_CFG_PATH)


    def saveNepiCfgCb(self,msg):
        self.msg_if.pub_warn("Recieved save nepi config message")
        self.msg_if.pub_warn("Saving nepi config to " + str(self.SYSTEM_CFG_PATH))
        self.save_cfg(self.SYSTEM_CFG_PATH)
            
       

    def saveSystemCfgsCb(self,msg):
        self.msg_if.pub_warn("Recieved save system configs message")
        self.save_system_cfgs()

    def save_system_cfgs(self):
        self.msg_if.pub_warn("Saving system configs to " + str(self.SYSTEM_CFG_PATH))
        self.save_cfgs(self.SYSTEM_CFG_PATH)

        



    def restore_cfgs(self,config_folders = ['factory_cfg','system_cfg','user_cfg']): 
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
               


    def restoreFactoryCfgsCb(self,msg):
        self.restore_factory_configs()

    def restore_factory_configs(self):
        nepi_utils.delete_files_in_folder(self.USER_CFG_PATH)
        nepi_utils.delete_files_in_folder(self.SYSTEM_CFG_PATH)
        success = self.restore_cfgs(config_folders=[self.FACTORY_CFG_PATH])
        time.sleep(1)
        os.system('reboot')


    def restoreSystemCfgsCb(self,msg):
        self.restore_system_configs()

    def restore_system_cfgs(self):
        nepi_utils.delete_files_in_folder(self.USER_CFG_PATH)
        success = self.restore_cfgs(config_folders = self.SYSTEM_CFG_PATH)            



    def reset_handler(self,namespace, cfg_path = None):
        if cfg_path is None:
            cfg_path = self.USER_CFG_PATH
        success = True
        if cfg_path != self.USER_CFG_PATH:
            # First delete user config files if it exists
            self.msg_if.pub_warn("Deleting User Config Files")
            ucfg_pathname = self.get_config_pathname(self.USER_CFG_PATH, namespace)
            if os.path.exists(ucfg_pathname):
                nepi_utils.delete_files_in_folder(ucfg_pathname)
        if cfg_path == self.FACTORY_CFG_PATH:
            # Delete system config files for factory reset
            self.msg_if.pub_warn("Restoring Factory Config Files")
            scfg_pathname = self.get_scfg_pathname(self.SYSTEM_CFG_PATH, namespace)
            if os.path.exists(scfg_pathname):
                nepi_utils.clear_folder(scfg_pathname)
            factory_pathname = self.get_scfg_pathname(self.FACTORY_CFG_PATH, namespace)
            if os.path.exists(factory_pathname):
                shutil.copy2(factory_pathname,scfg_pathname)  # Use copy2 to preserve metadata
        success = self.reset_params(namespace)
        return success

    



    def factoryResetHandler(self,req):
        self.msg_if.pub_warn("Got Factory Reset request: " + str(req))
        return self.reset_handler(req.namespace, cfg_path = self.FACTORY_CFG_PATH)

    def systemResetHandler(self,req):
        self.msg_if.pub_warn("Got System Reset request: " + str(req))
        return self.reset_handler(req.namespace, cfg_path = self.SYSTEM_CFG_PATH)

    def userResetHandler(self,req):
        self.msg_if.pub_warn("Got User Reset request: " + str(req))
        return self.reset_handler(req.namespace, cfg_path = self.USER_CFG_PATH)



    def statusPubCb(self,timer):
        self.publish_status()

    def publish_status(self):
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', Empty())

if __name__ == '__main__':
    config_mgr()
