#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import rospy

from std_msgs.msg import UInt8
from nepi_ros_interfaces.msg import RUISettings

from nepi_api.node_if import NodeClassIF
from nepi_api.sys_if_msg import MsgIF
from nepi_api.connect_mgr_if_system import ConnectMgrSystemIF
from nepi_api.connect_mgr_if_config import ConnectMgrConfigIF

from nepi_sdk import nepi_ros
 

class RUICfgMgrNode:

    DEFAULT_IMAGE_QUALITY = 95

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "rui_config_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

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



        self.settings_pub = rospy.Publisher('~settings', RUISettings, queue_size=1, latch=True)
        self.settings_msg = RUISettings()
        self.publish_settings() # Do it once so that latch works on next connection

        rospy.Subscriber('~set_streaming_image_quality', UInt8, self.set_streaming_image_quality_cb)

        self.save_cfg_if = SaveCfgIF(updateParamsCallback=None, paramsModifiedCallback=None)

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################

    def publish_settings(self):
        # Gather all settings for the message
        self.settings_msg.streaming_image_quality = nepi_ros.get_param(self,"~streaming_image_quality", self.DEFAULT_IMAGE_QUALITY)
        self.settings_msg.nepi_hb_auto_offload_visible = nepi_ros.get_param(self,"~nepi_hb_auto_offload_visible", False)

        # Publish it
        self.settings_pub.publish(self.settings_msg)

    def set_streaming_image_quality_cb(self, msg):
        if (msg.data < 1 or msg.data > 100):
            self.msg_if.pub_warn("Invalid image qualtiy: " + str(msg.data))
            return

        self.msg_if.pub_info("Setting streaming image quality to: " + str(msg.data))
        nepi_ros.set_param(self,"~streaming_image_quality", msg.data)
        self.publish_settings() # Make sure to always publish settings updates


if __name__ == '__main__':
  RUICfgMgr = RUICfgMgrNode()
