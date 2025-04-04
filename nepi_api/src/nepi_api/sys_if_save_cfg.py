#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import rospy


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from std_msgs.msg import Empty, String
from nepi_ros_interfaces.msg import Reset
from nepi_ros_interfaces.srv import *

from nepi_api.sys_if_msg import MsgIF

class SaveCfgIF(object):

    ready = False
    namespace = None

    ### IF Initialization
    def __init__(self, initCb = None, resetCb = None, factoryResetCb = None, namespace = None, init_params = True):
        #################################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################    
        self.initCb = initCb
        self.resetCb = resetCb
        self.factoryResetCb = factoryResetCb
        self.save_params_pub = rospy.Publisher('store_params', String, queue_size=1)
        
        if namespace is None:
            self.namespace = self.node_namespace
        else:
            self.namespace = namespace

        self.msg_if.pub_info("Loading saved config data")
        self._resetCb('user_reset')

        # Subscribe to provided save config namespace
        save_topic = os.path.join(self.namespace,'save_config')
        reset_topic = os.path.join(self.namespace,'reset_config')
        factory_reset_topic = os.path.join(self.namespace,'factory_reset_config')
        rospy.Subscriber(save_topic, Empty, self._saveConfigCb)
        rospy.Subscriber(reset_topic, Reset, self.sendResetMsgCb)
        rospy.Subscriber(reset_topic, Reset, self.sendResetMsgCb)
        rospy.Subscriber(factory_reset_topic, Reset, self.sendResetMsgCb)

        # Global Topic Subscribers
        rospy.Subscriber('save_config', Empty, self._saveConfigCb)
        rospy.Subscriber('reset_config', Reset, self.sendResetMsgCb)
        rospy.Subscriber('factory_reset_config', Reset, self.sendResetMsgCb)

        nepi_ros.sleep(1)

        if init_params == True:
            nepi_ros.sleep(1)
            self.init()

        ###############################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        self.msg_if.pub_info("Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Failed to Connect")
        else:
            self.msg_if.pub_info("Connected")
        return self.ready

    def init(self):
        if (self.initCb):
            self.initCb() # Callback provided by the container class to set values to param server, etc.

    def save(self):
        self.init()
        # Save the current nodes params
        self.save_params_pub.publish(self.node_namespace)

    def saveConfig(self): 
        self.save()

    def reset(self):
        self._resetCb('user_reset')
        nepi_ros.sleep(1)
        if (self.resetCb and ret_val == True):
            self.resetCb() # Callback provided by container class to update based on param server, etc.

        return ret_val


    def factory_reset(self):
        self._resetCb('factory_reset')
        nepi_ros.sleep(1)
        if (self.factoryResetCb):
            self.factoryResetCb() # Callback provided by container class to update based on param server, etc.
        return ret_val







    ###############################
    # Class Private Methods
    ###############################

    def _saveConfigCb(self, msg):
    	self.save()
    
    def _resetCb(self,reset_type):
        reset_proxy = rospy.ServiceProxy(reset_type, FileReset)
        try:
            resp = reset_proxy(self.node_namespace) # Need the current nodes fully-qualified namespace name here
            ret_val = resp.success
        except rospy.ServiceException as e:
            self.msg_if.pub_warn("Failed to connect to cfg mgr reset service: " + str(e))
        nepi_ros.sleep(1)

    def sendResetMsgCb(self, msg):
        reset_proxy = None
        ret_val = False
        if (msg.reset_type == Reset.USER_RESET):
            ret_val = self.reset()
        elif (msg.reset_type == Reset.FACTORY_RESET):
            ret_val = self.factory_reset()
        return ret_val








