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


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nepi_ros_interfaces.msg import NavPoseData
from nepi_ros_interfaces.srv import NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryRequest



from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_nav

class NavPoseIF:

    connected = False
    namespace = None

    nav_caps_dict = None

    nav_sub = None
    nav_msg = None

    #######################
    ### IF Initialization
    log_name = 'ConnectNavPoseIF'
    def __init__(self, 
                 navpose_namespace,
                 getNavPoseCallbackFunction = None,
                 timeout = float('inf')):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting IF Initialization Processes")
        ##############################  
        #   
        self.navpose_namespace = navpose_namespace

        time.sleep(1)
        request = NavPoseCapabilitiesQueryRequest()
        resp = None
        try:
            resp = nepi_ros.call_service(service, request)
        except:
            pass
        if resp is not None:
            self.nav_caps_dict = nepi_ros.convert_msg2dict(resp)

        topic = os.path.join(navpose_namespace,'navpose')
        self.nav_sub = rospy.Subcriber(topic,NavPoseData,self._navSub)
            
        #################################
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Initialization Complete")



    ###############################
    # Class Public Methods
    ###############################

    def check_connection(self):
        return self.connection

    def wait_for_connection(self, timout = float('inf') ):
        success = False
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.connected == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.connected == False:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Connect")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Connected")
        return self.connected

    def check_has_capabilities(self):
        has_caps = self.nav_caps_dict is not None
        return has_caps

    def get_capabilities_dict(self):
        return self.nav_caps_dict


    def get_namespace(self):
        return self.namespace

    def get_navpose_capabilities_dict(self):
        return self.nav_caps_dict

        
    def check_for_navpose_data(self):
        return self.nav_msg is not None

    def get_navpose_data_dict(self, clear_dict = True):
        nav_dict = None
        if self.nav_msg is not None:
            nav_dict = nepi_nav.convert_navposedata_msg2dict(self.nav_msg)
            if clear_data == True:
                self.nav_msg = None
        return nav_dict

    def register_navpose_handler(self, navpose_handler_function):
        # test function
        nav_msg = NavPoseData()
        nav_dict = nepi_nav.convert_navposedata_msg2dict(nav_msg)
        if navpose_handler_function is not None:
        try:
            navpose_handler_function(nav_dict)
            self.nav_handler = navpose_handler_function
        except Exception as e:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to register navpose handler: " + str(e))
    
    def unregister_listener(self): 
        success = False
        if self.nav_sub is None:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": NavPose Data listener not running")
            success = True
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Killing NavPose Data listener")
            try:
                self.nav_sub.unregister()
                success = True
                time.sleep(1)
            except:
                pass
            self.connected = False
            self.namespace = None
            self.nav_sub = None
            self.nav_msg = None
            self.nav_handler = None
        return success


    ###############################
    # Class Private Methods
    ###############################


    # Update System Status
    def _navSub(self,msg):
        self.nav_msg = msg
        self.connected = True
        if self.nav_handler is not None:
            try:
                nav_dict = self.get_navpose_dict(clear_msg = False)
                if nav_dict is not None:
                    self.nav_handler(nav_dict)
            except Exception as e:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to call navpose handler: " + str(e))
            
