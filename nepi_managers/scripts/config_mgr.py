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


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
 

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.srv import ParamsReset, ParamsResetRequest, ParamsResetResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF


NEPI_ENV_PACKAGE = 'nepi_env'

NEPI_CFG_PATH = '/opt/nepi/ros/etc'
USER_CFG_PATH = '/mnt/nepi_storage/user_cfg'
CFG_SUFFIX = '.yaml'
FACTORY_SUFFIX = '.factory'
USER_SUFFIX = '.user'

SYSTEM_MGR_NODENAME = 'system_mgr'


# Files outside the normal NEPI-ROS cfg. scheme
SYS_CFGS_TO_PRESERVE = {
    'sys_env.bash' : '/opt/nepi/sys_env.bash', # Serial number, ROS launch file, external ROS MASTER etc.
    'authorized_keys' : '/opt/nepi/config/home/nepi/ssh/authorized_keys', # NEPI Device SSH public keys
    'hostname' : '/opt/nepi/config/etc/hostname', # NEPI Device hostname
    'sshd_config' : '/opt/nepi/config/etc/ssh/sshd_config', # SSH Server Config
    'chrony.conf.user' : '/opt/nepi/config/etc/chrony/chrony.conf.user', # NTP/Chrony Config
    's2x_iptables.rules' : '/opt/nepi/config/etc/iptables/s2x_iptables.rules', # Route and forwarding rules; e.g., for dual-interface devices
    's2x_sensor_if_static_ip' : '/opt/nepi/config/etc/network/interfaces.d/s2x_sensor_if_static_ip', # Static IP address for secondary/sensor ethernet interface
    'nepi_user_ip_aliases' : '/opt/nepi/config/etc/network/interfaces.d/nepi_user_ip_aliases', # IP alias addresses for primary ethernet interface
    'nepi_static_ip' : '/opt/nepi/config/etc/network/interfaces.d/nepi_static_ip', # Principal static IP address for primary ethernet interface
    'fstab' : '/opt/nepi/config/etc/fstab', # Filesystem mounting rules; e.g., nepi_storage on SD vs SSD
    'smb.conf' : '/opt/nepi/config/etc/samba/smb.conf'
}



class config_mgr(object):

    node_if = None
    config_folders = dict()

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "config_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
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
        self.msg_if.pub_info("Waiting for system folders")
        folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        self.msg_if.pub_warn("Got system folders: " + str(folders))
        for folder in folders.keys():
            if folder.find('user_cfg') != -1:
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
                'callback': self.factory_reset
            },
            'user_reset': {
                'namespace': self.base_namespace,
                'topic': 'user_reset',
                'srv': ParamsReset,
                'req': ParamsResetRequest(),
                'resp': ParamsResetResponse(),
                'callback': self.user_reset
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
                'callback': self.save_system_cfgs_Cb, 
                'callback_args': ()
            },
            'save_system_config': {
                'namespace': self.base_namespace,
                'topic': 'save_system_config',
                'msg': Empty,
                'qsize': None,
                'callback': self.save_system_cfgs_Cb, 
                'callback_args': ()
            },
            'store_params': {
                'namespace': self.base_namespace,
                'topic': 'store_params',
                'msg': String,
                'qsize': None,
                'callback': self.store_params, 
                'callback_args': ()
            },
            'full_factory_restore': {
                'namespace': self.base_namespace,
                'topic': 'full_factory_restore',
                'msg': Empty,
                'qsize': None,
                'callback': self.restore_factory_cfgs, 
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

        # Restore user configurations
        self.restore_system_cfgs()
        self.msg_if.pub_warn("System config files restored")
        time.sleep(1)
        self.save_system_cfgs()
        self.msg_if.pub_warn("System config files saved")

        nepi_sdk.start_timer_process(1, self.statusPubCb)
        ################
        # Save the current system config
        sys_ns = nepi_sdk.create_namespace(self.base_namespace,SYSTEM_MGR_NODENAME)
        self.save_params(sys_ns)

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
            self.node_if.reset_params()
        if do_updates == True:
            pass
        self.initCb()


    def factoryResetCb(self,do_updates = True):
        self.aifs_classes_dict = dict()
        self.aif_classes_dict = dict()
        if self.node_if is not None:
            self.node_if.factory_reset_params()
        if do_updates == True:
            pass
        self.initCb()



    def publish_status(self):
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', Empty())

    def statusPubCb(self,timer):
        self.publish_status()


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
            self.msg_if.pub_info("Updating Params for namespace: " + namespace  + " from file " + file_pathname )
            paramlist = None
            try:
                paramlist = nepi_sdk.load_params_from_file(file_pathname, namespace)
                self.msg_if.pub_warn("Got Params for namespace: " + namespace  + " from file " + file_pathname  + " : " + str(paramlist))
            except Exception as e:
                self.msg_if.pub_warn("Unable to load parameters from file " + file_pathname + " " + str(e))
            if paramlist is not None:
                for params, ns in paramlist:
                    try:
                        nepi_sdk.upload_params(ns, params, verbose=True)     
                    except Exception as e:
                        self.msg_if.pub_warn("Unable to upload parameters "  + str(e))
                self.msg_if.pub_info("Updated Params for namespace: " + namespace )
            #if namespace == '/nepi/s2x/nexigo_23':
            #    self.msg_if.pub_warn("Current Params for namespace: " + namespace + " " + str())
        return [True]

    def get_factory_pathname(self,namespace):
        filename = self.get_filename_from_namespace(namespace)
        self.msg_if.pub_warn("Got filename: " + namespace + " " + filename)
        pathname = os.path.join(NEPI_CFG_PATH,filename + CFG_SUFFIX)
        return pathname

    def get_user_pathname(self,namespace):
        filename = self.get_filename_from_namespace(namespace)
        user_cfg_dirname = os.path.join(USER_CFG_PATH)
        
        # Ensure the path we report actually exists
        if not os.path.isdir(user_cfg_dirname):
            os.makedirs(user_cfg_dirname)

        pathname = os.path.join(user_cfg_dirname, filename + CFG_SUFFIX + USER_SUFFIX)
        return pathname

    def user_reset(self,req):
        namespace = req.namespace
        success = False
        user_pathname = self.get_user_pathname(namespace)
        if os.path.exists(user_pathname):
            self.msg_if.pub_warn("Reseting params for namespace from user cfg file: " + namespace  + " from file " + user_pathname)
            success = self.update_from_file(user_pathname, namespace)
        # Now update the param server
        return success

    def factory_reset(self,req):
        namespace = req.namespace
        success = False
        # First delete user config file if it exists
        user_pathname = self.get_user_pathname(namespace)
        if os.path.exists(user_pathname):
            try:
                os.remove(user_pathname)
                print(f"File '{user_pathname}' deleted successfully.")
                success = True
            except FileNotFoundError:
                print(f"File '{user_pathname}' not found.")
            except PermissionError:
                print(f"Permission denied to delete '{user_pathname}'.")
            except Exception as e:
                print(f"An error occurred: {e}")
        # Restore factory config if exits
        factory_pathname = self.get_factory_pathname(namespace)
        if os.path.exists(factory_pathname):
            success = self.update_from_file(factory_pathname, namespace)
        return success

    def store_params(self,msg):
        namespace = msg.data
        self.save_params(namespace)
    
    def save_params(self,namespace):
        user_pathname = self.get_user_pathname(namespace)
        self.msg_if.pub_info("Storing Params for namespace: " + namespace  + " in file " + user_pathname )
        # First, write to the user file
        nepi_sdk.save_params_to_file(user_pathname, namespace)
        self.msg_if.pub_info("Params saved for namespace: " + namespace  + " in file " + user_pathname )

    def save_system_cfgs_Cb(self,msg):
        self.save_system_cfgs()


    def save_system_cfgs(self):
        target_dir = os.path.join(USER_CFG_PATH, 'sys')
        if not os.path.exists(target_dir):
            os.makedirs(target_dir)

        for cfg in SYS_CFGS_TO_PRESERVE:
            source = SYS_CFGS_TO_PRESERVE[cfg]
            target = os.path.join(USER_CFG_PATH, 'sys', cfg)
            os.system('cp -rf ' + source + ' ' + target)



    def restore_system_cfgs_all(self,msg):
        self.restore_system_cfgs()
        time.sleep(1)
        os.system('reboot')

    def restore_system_cfgs(self):
        # Handle non-ROS user system configs.        
        for name in SYS_CFGS_TO_PRESERVE.keys():
            full_name = os.path.join(USER_CFG_PATH, 'sys', name)
            if os.path.exists(full_name):
                if os.path.isdir(full_name):
                    full_name = os.path.join(full_name,'*') # Wildcard avoids copying source folder into target folder as a subdirectory
                target = SYS_CFGS_TO_PRESERVE[name]
                self.msg_if.pub_warn("Updating " + target + " from user config")
                os.system('cp -rf ' + full_name + ' ' + target)
                

                # don't update sys_env NEPI_ENV_PACKAGE value
                if name == 'sys_env.bash':
                    self.msg_if.pub_warn("Updating sys_env.bash file with correct Package name")
                    tmp_file = target + ".tmp"
                    os.system('cp -rf ' + target + ' ' + tmp_file)
                    file_lines = []
                    with open(tmp_file, "r") as f:
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
                    

                    bak_filename = target + ".bak"
                    os.system('cp -rf ' + tmp_file + ' ' + target)
                    os.system('rm ' + tmp_file)
                os.system('chown -R nepi:nepi ' + target)


    def restore_factory_cfgs_all(self,msg):
        self.restore_factory_cfgs()
        time.sleep(1)
        os.system('reboot')

    def restore_factory_cfgs(self):
        # First handle the ROS user configs.
        for name in SYS_CFGS_TO_PRESERVE.keys():
            full_name = os.path.join(USER_CFG_PATH, name)
            if os.path.exists(full_name):
                os.system('rm ' + full_name)

        '''
        # Now handle non-ROS user system configs.        
        for name in SYS_CFGS_TO_PRESERVE.keys():
            full_name = os.path.join(USER_CFG_PATH, 'sys', name)
            if os.path.exists(full_name):
                os.system('rm ' + full_name)
        '''
                

if __name__ == '__main__':
    config_mgr()
