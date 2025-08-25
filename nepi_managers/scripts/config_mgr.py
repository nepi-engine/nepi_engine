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


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
 

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.srv import ParamsReset, ParamsResetRequest, ParamsResetResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF


NEPI_ENV_PACKAGE = 'nepi_env'

NEPI_HOME_PATH = '/home/nepi'
NEPI_ETC_PATH = '/opt/nepi/nepi_engine/etc'
FACTORY_CFG_PATH = '/mnt/nepi_config/factory_cfg'
SYSTEM_CFG_PATH = '/mnt/nepi_config/system_cfg'
USER_CFG_PATH = '/mnt/nepi_storage/user_cfg'
CFG_SUFFIX = '.yaml'
FACTORY_SUFFIX = '.factory'
USER_SUFFIX = '.user'

SYSTEM_MGR_NODENAME = 'system_mgr'


# Files outside the normal NEPI-ROS cfg. scheme
SYS_CFGS_TO_PRESERVE = {
    'sys_env' : '/opt/nepi/sys_env.bash', # Serial number, ROS launch file, external ROS MASTER etc.
    'nepi_config' : '/opt/nepi/etc/nepi_config.yaml', # NEPI Config
    'hostname' : '/opt/nepi/etc/hostname', # NEPI Device hosts
    'hosts' : '/opt/nepi/etc/hosts', # NEPI Device hostnam
    'wpa_supplicant' : '/opt/nepi/etc/wpa_supplicant/wpa_supplicant.conf', # NEPI WiFi config
    'sshd_config' : '/opt/nepi/etc/ssh/sshd_config', # SSH Server Config
    'chrony.conf' : '/opt/nepi/etc/chrony/chrony.conf', # NTP/Chrony Config
    'nepi_iptables.rules' : '/opt/nepi/etc/network/nepi_iptables.rules', # Route and forwarding rules; e.g., for dual-interface devices
    'nepi_user_ip_aliases' : '/opt/nepi/etc/network/interfaces.d/nepi_user_ip_aliases', # IP alias addresses for primary ethernet interface
    'nepi_static_ip' : '/opt/nepi/etc/network/interfaces.d/nepi_static_ip', # Principal static IP address for primary ethernet interface
    'fstab' : '/opt/nepi/etc/fstabs/fstab', # Filesystem mounting rules; e.g., nepi_storage on SD vs SSD
    'smb.conf' : '/opt/nepi/etc/samba/smb.conf'
}



class config_mgr(object):

    node_if = None
    config_folders = dict()
    config_folders['factory_cfg']=FACTORY_CFG_PATH
    config_folders['system_cfg']=SYSTEM_CFG_PATH
    config_folders['user_cfg']=USER_CFG_PATH


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
        self.msg_if.pub_warn("Got system folders: " + str(folders))
        for folder in self.config_folders.keys():
            if folder in folders.keys():
                self.config_folders[folder] = folders[folder]
        
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
                'topic': 'user_reset',
                'srv': ParamsReset,
                'req': ParamsResetRequest(),
                'resp': ParamsResetResponse(),
                'callback': self.systemResetHandler
            }
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
            'save_config': {
                'namespace': self.base_namespace,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': None,
                'callback': self.saveSystemCfgsCb, 
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

        # Restore configurations
        self.restore_cfgs()
        self.msg_if.pub_warn("System config files restored")
        time.sleep(1)
        self.save_cfgs()
        self.msg_if.pub_warn("System config files saved")

        nepi_sdk.start_timer_process(1, self.statusPubCb)
        ################
        # Save the current system config
        sys_ns = nepi_sdk.create_namespace(self.base_namespace,SYSTEM_MGR_NODENAME)
        self.save_params(USER_CFG_PATH, sys_ns)

        nepi_sdk.sleep(2)
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
            return [False]
        else:
            self.msg_if.pub_warn("Loading Params for namespace: " + namespace  + " from file " + file_pathname )
            params_dict = None
            try:
                params_dict = nepi_sdk.load_params_from_file(file_pathname, namespace, log_name_list = [self.class_name])
                self.msg_if.pub_warn("Got Params for namespace: " + namespace  + " from file " + file_pathname  + " : " + str(params_dict.keys()), log_name_list = [self.class_name])
                if namespace.find('idx') != -1:
                    params = nepi_sdk.get_params(namespace)
                    self.msg_if.pub_warn("Got Params for idx namespace: " + namespace  + " from file " + file_pathname  + " : " + str(params_dict), log_name_list = [self.class_name])
            except Exception as e:
                self.msg_if.pub_warn("Unable to load parameters from file " + file_pathname + " " + str(e))
            self.msg_if.pub_warn("Updated Params for namespace: " + namespace )
        return [True]

    def get_config_pathname(self, path, namespace):
        filename = self.get_filename_from_namespace(namespace)
        
        # Ensure the path we report actually exists
        if not os.path.isdir(path):
            os.makedirs(path)

        pathname = os.path.join(path, filename + CFG_SUFFIX)
        return pathname




    def saveParamsCb(self,msg):
        namespace = msg.data
        self.save_params(USER_CFG_PATH, namespace)
    
    def save_params(self, path, namespace):
        success = False
        if os.path.exists(path):
            config_pathname = self.get_config_pathname(path, namespace)
            backup_pathname = os.path.dirname(config_pathname) + '/.' + os.path.basename(config_pathname)

            self.msg_if.pub_info("Storing Params for namespace: " + namespace  + " in file " + config_pathname )
            # First, write to the user file
            nepi_sdk.save_params_to_file(config_pathname, namespace)
            nepi_sdk.save_params_to_file(backup_pathname, namespace)
            self.msg_if.pub_info("Params saved for namespace: " + namespace  + " in file " + config_pathname )
            success = True
        return success

 
 
    def save_cfgs(self,path):
        if path != USER_CFG_PATH:
            # Save System Files
            target_dir = os.path.join(path, 'sys')
            if not os.path.exists(target_dir):
                os.makedirs(target_dir)
            for cfg in SYS_CFGS_TO_PRESERVE:
                source = SYS_CFGS_TO_PRESERVE[cfg]
                target = os.path.join(USER_CFG_PATH, 'sys', cfg)
                #self.msg_if.pub_info("Save Config copying file: " + source  + " to " + target )
                os.system('cp -rfp ' + source + ' ' + target)
                btarget = backup_pathname = os.path.dirname(target) + '/.' + os.path.basename(config_pathname)
                os.system('cp -rfp ' + source + ' ' + btarget)

            # Copy User Configs to System Configs
            source_folder = USER_CFG_PATH
            destination_folder = path
            
            # Create destination folder if it doesn't exist
            os.makedirs(destination_folder, exist_ok=True)

            for filename in os.listdir(source_folder):
                source_path = os.path.join(source_folder, filename)
                destination_path = os.path.join(destination_folder, filename)

                if os.path.isfile(source_path):  # Ensure it's a file, not a subdirectory
                    try:
                        shutil.copy2(source_path, destination_path)  # Use copy2 to preserve metadata
                        print(f"Copied '{filename}'")
                    except Exception as e:
                        print(f"Error copying '{filename}': {e}")


    def saveFactoryCfgsCb(self,msg):
        self.save_factory_cfgs()

    def save_factory_cfgs(self):
        self.save_cfgs(FACTORY_CFG_PATH)

                
    def saveSystemCfgsCb(self,msg):
        self.save_system_cfgs()

    def save_system_cfgs(self):
        self.save_cfgs(SYSTEM_CFG_PATH)

        



    def restore_cfgs(self,path): 
        for name in SYS_CFGS_TO_PRESERVE.keys():
            source = os.path.join(path, 'sys', name)
            if os.path.exists(source):
                if os.path.isdir(source):
                    source = os.path.join(source,'*') # Wildcard avoids copying source folder into target folder as a subdirectory
                target = SYS_CFGS_TO_PRESERVE[name]
                self.msg_if.pub_info("Restore copying system file: " + source  + " to " + target )
                os.system('cp -rfp ' + source + ' ' + target)
                

                # don't update sys_env NEPI_ENV_PACKAGE value
                if name == 'sys_env.bash':
                    self.msg_if.pub_warn("Updating sys_env.bash file with correct Package name")
                    file_lines = []
                    with open(source, "r") as f:
                        for line in f:
                            #self.msg_if.pub_info("Got sys_env line: " + line)
                            if line.startswith("export ROS1_PACKAGE="):
                                self.msg_if.pub_warn("Found sys_env Package line")
                                update_line = ("export ROS1_PACKAGE=" + NEPI_ENV_PACKAGE + '\n')
                                self.msg_if.pub_warn("Update sys_env Package line: " + update_line)
                            else:
                                update_line = line
                            #self.msg_if.pub_info("Update sys_env line: " + update_line)
                            file_lines.append(update_line)
                    tmp_file = target + ".tmp"
                    if os.path.exists(tmp_file):
                        os.system('rm ' + tmp_file)
                    with open(tmp_file,'w') as f:
                        f.writelines(file_lines)
                    
                    os.system('cp -rfp ' + tmp_file + ' ' + target)
                os.system('chown -R nepi:nepi ' + target)


    def restoreFactoryCfgsCb(self,msg):
        self.restore_factory_configs()

    def restore_factory_configs(self):
        nepi_utils.delete_files_in_folder(USER_CFG_PATH)
        nepi_utils.delete_files_in_folder(SYSTEM_CFG_PATH)
        success = self.restore_cfgs(FACTORY_CFG_PATH)
        time.sleep(1)
        os.system('reboot')


    def restoreSystemCfgsCb(self,msg):
        self.restore_system_configs()

    def restore_system_cfgs(self,path):
        nepi_utils.delete_files_in_folder(USER_CFG_PATH)
        success = self.restore_cfgs(SYSTEM_CFG_PATH)            



    def reset_handler(self,namespace):
        success = True
        # First delete user config file if it exists
        config_pathname = self.get_config_pathname(USER_CFG_PATH, namespace)
        if os.path.exists(config_pathname):
            try:
                os.remove(config_pathname)
                print(f"File '{config_pathname}' deleted successfully.")
            except FileNotFoundError:
                success = False
                print(f"File '{config_pathname}' not found.")
            except PermissionError:
                success = False
                print(f"Permission denied to delete '{config_pathname}'.")
            except Exception as e:
                success = False
                print(f"An error occurred: {e}")
        if success == True:
            # Restore config if exits
            restore_pathname = self.get_config_pathname(SYSTEM_CFG_PATH, namespace)
            if os.path.exists(restore_pathname):
                success = self.update_from_file(restore_pathname, namespace)
            if success == False:
                try:
                    os.remove(config_pathname)
                except Exception as e:
                    print(f"An error occurred: {e}")
                restore_pathname = self.get_config_pathname(FACTORY_CFG_PATH, namespace)
                if os.path.exists(restore_pathname):
                    success = self.update_from_file(restore_pathname, namespace)
                    return success
        return success


    def factoryResetHandler(self,req):
        return self.reset_handler(FACTORY_CFG_PATH. req.namespace)

    def systemResetHandler(self,req):
        return self.reset_handler(SYSTEM_CFG_PATH. req.namespace)

    def userResetHandler(self,req):
        return self.reset_handler(USER_CFG_PATH. req.namespace)



    def statusPubCb(self,timer):
        self.publish_status()

    def publish_status(self):
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', Empty())

if __name__ == '__main__':
    config_mgr()
