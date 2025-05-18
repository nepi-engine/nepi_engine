#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


from std_msgs.msg import UInt8
from nepi_ros_interfaces.msg import RUISettings

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
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
            nepi_ros.signal_shutdown(self.node_name + ": Failed to get System Ready")
        

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
            'streaming_image_quality': {
                'namespace': self.node_namespace,
                'factory_val': self.DEFAULT_IMAGE_QUALITY
            },
            'nepi_hb_auto_offload_visible': {
                'namespace': self.node_namespace,
                'factory_val': False
            }

        }


        # Services Config Dict ####################
        self.SRVS_DICT = None

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'settings_pub': {
                'namespace': self.node_namespace,
                'topic': 'settings',
                'msg': RUISettings,
                'qsize': 1,
                'latch': True
            }
        }  



        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'image_quality': {
                'namespace': self.node_namespace,
                'topic': 'set_streaming_image_quality',
                'msg': UInt8,
                'qsize': None,
                'callback': self.set_streaming_image_quality_cb, 
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
                        msg_if = self.msg_if
        )

        self.msg_if.pub_warn("Waiting for Node Class Ready")
        ready = self.node_if.wait_for_ready()
        self.msg_if.pub_warn("Got Node Class Ready: " + str(ready))


        #########################################################
        ## Complete Initiation 

        self.settings_msg = RUISettings()
        self.publish_settings() # Do it once so that latch works on next connection

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################

    def publish_settings(self):
        # Gather all settings for the message
        self.settings_msg.streaming_image_quality = self.node_if.get_param("streaming_image_quality")
        self.settings_msg.nepi_hb_auto_offload_visible = self.node_if.get_param("nepi_hb_auto_offload_visible")

        # Publish it
        self.node_if.publish_pub('settings_pub', self.settings_msg)

    def set_streaming_image_quality_cb(self, msg):
        if (msg.data < 1 or msg.data > 100):
            self.msg_if.pub_warn("Invalid image qualtiy: " + str(msg.data))
            return


    def initCb(self, do_updates = False):
        if do_updates == True:
            self.resetCb()

    def resetCb(self):
        self.publish_settings() # Make sure to always publish settings updates

    def factoryResetCb(self):
        self.publish_settings() # Make sure to always publish settings updates


if __name__ == '__main__':
  RUICfgMgr = RUICfgMgrNode()
