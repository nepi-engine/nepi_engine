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

import rospy
import rosparam

import os
import time
import errno

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
 

from std_msgs.msg import String, Empty
from nepi_ros_interfaces.srv import FileReset

from nepi_api.node_if import NodeClassIF
from nepi_api.sys_if_msg import MsgIF
from nepi_api.connect_mgr_if_system import ConnectMgrSystemIF


NEPI_ENV_PACKAGE = 'nepi_env'

NEPI_CFG_PATH = '/opt/nepi/ros/etc'
USER_CFG_PATH = '/mnt/nepi_storage/user_cfg'
CFG_SUFFIX = '.yaml'
FACTORY_SUFFIX = '.factory'
USER_SUFFIX = '.user'

pending_nodes = {}

# Files outside the normal NEPI-ROS cfg. scheme
SYS_CFGS_TO_PRESERVE = {
    'sys_env.bash' : '/opt/nepi/sys_env.bash', # Serial number, ROS launch file, external ROS MASTER etc.
    'authorized_keys' : '/opt/nepi/config/home/nepi/ssh/authorized_keys', # NEPI Device SSH public keys
    'hostname' : '/opt/nepi/config/etc/hostname', # NEPI Device hostname
    'sshd_config' : '/opt/nepi/config/etc/ssh/sshd_config', # SSH Server Config
    'chrony.conf' : '/opt/nepi/config/etc/chrony/chrony.conf.user', # NTP/Chrony Config
    'iptables.rules' : '/opt/nepi/config/etc/iptables/s2x_iptables.rules', # Route and forwarding rules; e.g., for dual-interface devices
    'sensor_if_static_ip' : '/opt/nepi/config/etc/network/interfaces.d/s2x_sensor_if_static_ip', # Static IP address for secondary/sensor ethernet interface
    'ip_aliases' : '/opt/nepi/config/etc/network/interfaces.d/nepi_user_ip_aliases', # IP alias addresses for primary ethernet interface
    'static_ip' : '/opt/nepi/config/etc/network/interfaces.d/nepi_static_ip', # Principal static IP address for primary ethernet interface
    'fstab' : '/opt/nepi/config/etc/fstab', # Filesystem mounting rules; e.g., nepi_storage on SD vs SSD
    'smb.conf' : '/opt/nepi/config/etc/samba/smb.conf', # Samba configuration
    'nepi_connect_cfg' : '/opt/nepi/nepi_link/nepi-bot/devinfo/', # NEPI Connect Device NUID, SSH keys, etc.
    'nepi_connect_bot_cfg.json' : '/opt/nepi/nepi_link/nepi-bot/cfg/bot/config.json' # NEPI Connect device (nepi-bot) config file
}

class config_mgr(object):
    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "config_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################
        ## Wait for NEPI core managers to start
        # Wait for System Manager
        mgr_sys_if = ConnectMgrSystemIF()
        success = mgr_sys_if.wait_for_status()
        if success == False:
            nepi_ros.signal_shutdown(self.node_name + ": Failed to get System Status Msg")
        ###########################



        rospy.Subscriber('save_config', Empty, self.save_non_ros_cfgs) # Global one only
        rospy.Subscriber('store_params', String, self.store_params)
        rospy.Subscriber('full_user_restore', Empty, self.restore_user_cfgs_mgr)

        rospy.Service('factory_reset', FileReset, self.factory_reset)
        rospy.Service('user_reset', FileReset, self.user_reset)

        self.status_pub = rospy.Publisher("~status", Empty, queue_size=1, latch=True)
        time.sleep(1)
        self.status_pub.publish()
        # Restore user configurations
        self.restore_user_cfgs()

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################


    # Moving symlinks is typically faster and more robust than copying files, so to reduce the
    # chance of filesystem corruption in the event of e.g., power failure, we use a symlink-based config
    # file scheme.
    def symlink_force(self,target, link_name):
        self.msg_if.pub_info("Will try link update for link " +  link_name  + " with target: " + target )
        link_dirname = os.path.dirname(link_name)
        if not os.path.exists(link_dirname):
            self.msg_if.pub_info("Skipping symlink for " + link_name + " because path does not exist... missing factory config?")
            return False
        try:
            os.symlink(target, link_name)
            self.msg_if.pub_info("Updated link " +  link_name  + " with target: " + target )
        except OSError as e:
            self.msg_if.pub_info("Got error updating existing link " +  link_name  + " with target: " + target)
            if e.errno == errno.EEXIST:
                self.msg_if.pub_info("Recreating link" +  link_name  + " with target: " + target)
                os.remove(link_name)
                os.symlink(target, link_name)
            else:
                rospy.logerr("Unable to create symlink " + str(e))
                return False
        link = nepi_utils.get_symlink_target(link_name)
        self.msg_if.pub_info("File " + target + " updated with link: " + str(link))
        return True

    def separate_node_name_in_msg(self,qualified_node_name):
        return qualified_node_name.split("/")[-1]

    def update_from_file(self,file_pathname, namespace):
        self.msg_if.pub_info("Updating Params for namespace: " + namespace  + " from file " + file_pathname )
        try:
            paramlist = rosparam.load_file(file_pathname, namespace, verbose=False)
            #self.msg_if.pub_warn("Got Params for namespace: " + namespace  + " from file " + file_pathname  + " : " + str(paramlist))

            for params, ns in paramlist:
                rosparam.upload_params(ns, params, verbose=False)
        except Exception as e:
            self.msg_if.pub_warn("Unable to load factory parameters from file " + file_pathname + " " + str(e))
            return [False]

        return [True]

    def get_cfg_pathname(self,qualified_node_name):
        node_name = self.separate_node_name_in_msg(qualified_node_name)
        self.msg_if.pub_warn("Got node_name: " + qualified_node_name + " " + node_name)
        cfg_pathname = os.path.join(USER_CFG_PATH,'ros',node_name + CFG_SUFFIX + FACTORY_SUFFIX)
        mgr_file_path = os.path.join(NEPI_CFG_PATH,"nepi_managers",node_name + CFG_SUFFIX)
        if os.path.islink(mgr_file_path): # Check if a manager config
            #self.msg_if.pub_info("Found manager config: " + qualified_node_name)
            cfg_pathname = mgr_file_path  
        else:
            pathname = os.path.join(NEPI_CFG_PATH, node_name + CFG_SUFFIX)    
            if os.path.islink(pathname) == True:
                cfg_pathname = pathname
                    
        self.msg_if.pub_warn("Got config file path: " + qualified_node_name + " " + cfg_pathname)
        return cfg_pathname

    def get_user_cfg_pathname(self,qualified_node_name):
        node_name = self.separate_node_name_in_msg(qualified_node_name)
        user_cfg_dirname = os.path.join(USER_CFG_PATH, 'ros')
        
        # Ensure the path we report actually exists
        if not os.path.isdir(user_cfg_dirname):
            os.makedirs(user_cfg_dirname)

        user_cfg_pathname = os.path.join(user_cfg_dirname, node_name + CFG_SUFFIX + USER_SUFFIX)
        return user_cfg_pathname

    def user_reset(self,req):
        qualified_node_name = req.node_name
        cfg_pathname = self.get_cfg_pathname(qualified_node_name)
        self.msg_if.pub_info("Reseting params for node_name: " + qualified_node_name  + " from file " + cfg_pathname)
        # Now update the param server
        return self.update_from_file(cfg_pathname, qualified_node_name)

    def factory_reset(self,req):
        qualified_node_name = req.node_name
        cfg_pathname = self.get_cfg_pathname(qualified_node_name)
        factory_cfg_pathname = cfg_pathname + FACTORY_SUFFIX

        # First, move the symlink
        if os.path.islink(cfg_pathname):
            if False == self.symlink_force(factory_cfg_pathname, cfg_pathname):
                return [False] # Error logged upstream
        # Now update the param server
        return self.update_from_file(cfg_pathname, qualified_node_name)

    def store_params(self,msg):
        qualified_node_name = msg.data
        user_cfg_pathname = self.get_user_cfg_pathname(qualified_node_name)
        self.msg_if.pub_info("Storing Params for node_name: " + qualified_node_name  + " in file " + user_cfg_pathname )
        # First, write to the user file
        rosparam.dump_params(user_cfg_pathname, qualified_node_name)

        # Now, ensure the link points to the correct file
        cfg_pathname = self.get_cfg_pathname(qualified_node_name)
        self.symlink_force(user_cfg_pathname, cfg_pathname) # Error logged upstream

    def save_non_ros_cfgs(self,msg):
        target_dir = os.path.join(USER_CFG_PATH, 'sys')
        if not os.path.exists(target_dir):
            os.makedirs(target_dir)

        for cfg in SYS_CFGS_TO_PRESERVE:
            source = SYS_CFGS_TO_PRESERVE[cfg]
            target = os.path.join(USER_CFG_PATH, 'sys', cfg)
            os.system('cp -rf ' + source + ' ' + target)

    def restore_user_cfgs_mgr(self,msg):
        self.restore_user_cfgs()

    def restore_user_cfgs(self):
        # First handle the NEPI-ROS user configs.
        for root, dirs, files in os.walk(NEPI_CFG_PATH):
            for name in files:
                full_name = os.path.join(root, name)
                if full_name.endswith(CFG_SUFFIX) and os.path.islink(full_name):
                    user_cfg_name = os.path.join(USER_CFG_PATH, 'ros', name + USER_SUFFIX)
                    if os.path.exists(user_cfg_name): # Restrict to those with present user configs
                        if os.path.islink(name):
                            link_name = os.path.join(root, name.replace(FACTORY_SUFFIX, ''))
                            self.msg_if.pub_info("Updating " + link_name + " to user config: " + user_cfg_name)
                            self.symlink_force(user_cfg_name, link_name)
                    else:
                        pass
                    	#self.msg_if.pub_info("User config file does not exist at " + user_cfg_name)

        # Now handle non-ROS user system configs.        
        for name in SYS_CFGS_TO_PRESERVE:
            full_name = os.path.join(USER_CFG_PATH, 'sys', name)
            if os.path.exists(full_name):
                if os.path.isdir(full_name):
                    full_name = os.path.join(full_name,'*') # Wildcard avoids copying source folder into target folder as a subdirectory
                target = SYS_CFGS_TO_PRESERVE[name]
                self.msg_if.pub_info("Upettings_capabilities_query:1672] call_service InvalidServiceException: Service /nedating " + target + " from user config")
                # don't overwrite changes to the system's NEPI_ENV_PACKAGE
                if name == 'sys_env.bash':
                    file_lines = []
                    with open(full_name, "r") as f:
                        for line in f:
                            if line.startswith("export ROS1_PACKAGE="):
                                file_lines.append("export ROS1_PACKAGE=" + NEPI_ENV_PACKAGE + '\n')
                            else:
                                file_lines.append(line)
                    tmp_filename = full_name + ".tmp"
                    with open(tmp_filename, "w") as f:
                        for line in file_lines:
                            f.write(line)
                os.system('cp -rf ' + full_name + ' ' + target)
                os.system('chown -R nepi:nepi ' + target)



if __name__ == '__main__':
    config_mgr()
