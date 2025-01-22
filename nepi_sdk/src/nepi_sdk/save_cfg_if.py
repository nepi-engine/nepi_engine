#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import rospy

from std_msgs.msg import Empty, String
from nepi_ros_interfaces.msg import Reset
from nepi_ros_interfaces.srv import *

'''
Basic interface for the global and private save_config topics.
'''
class SaveCfgIF(object):

    def __init__(self, updateParamsCallback=None, paramsModifiedCallback=None, namespace = None):
        self.updateParams = updateParamsCallback
        self.paramsModified = paramsModifiedCallback
        self.store_params_publisher = rospy.Publisher('store_params', String, queue_size=1)
        
        
        if namespace != None:
            self.namespace = namespace
            save_topic = namespace + '/save_config'
            reset_topic = namespace + '/reset_config'
        else:
            self.namespace = rospy.get_name()
            save_topic = '~save_config'
            reset_topic = '~reset_config'   

        
        rospy.Subscriber(save_topic, Empty, self.saveConfigCb)
        rospy.Subscriber(reset_topic, Reset, self.resetConfigCb)
        
        rospy.Subscriber('save_config', Empty, self.saveConfigCb)
        rospy.Subscriber('reset_config', Reset, self.resetConfigCb)

    def saveConfigCb(self, msg):
    	self.saveConfig()
    
    def saveConfig(self,do_param_updates = True):
        if (self.updateParams) and do_param_updates:
            self.updateParams() # Callback provided by the container class to set values to param server, etc.
        self.store_params_publisher.publish(self.namespace)


    def resetConfigCb(self, msg):
        reset_proxy = None
        ret_val = False
        if (msg.reset_type == Reset.USER_RESET):
            ret_val = self.userReset()
        elif (msg.reset_type == Reset.FACTORY_RESET):
            ret_val = self.factoryReset()
        return ret_val


    def factoryReset(self):
        reset_proxy = rospy.ServiceProxy('factory_reset', FileReset)
        try:
            #if self.namespace != None:
            #    resp = reset_proxy(self.namespace)
            #else:
            resp = reset_proxy(rospy.get_name()) # Need the fully-qualified namespace name here
            ret_val = resp.success
        except rospy.ServiceException as e:
            rospy.logerr("%s: service call failed: %s", rospy.get_name(), e)

        if (self.paramsModified):
            self.paramsModified() # Callback provided by container class to update based on param server, etc.

        return ret_val


    def userReset(self):
        reset_proxy = rospy.ServiceProxy('user_reset', FileReset)
        ret_val = False
        try:
            #if self.namespace != None:
            #    resp = reset_proxy(self.namespace)
            #else:
            resp = reset_proxy(rospy.get_name()) # Need the fully-qualified namespace name here
            ret_val = resp.success
        except rospy.ServiceException as e:
            rospy.logerr("%s: service call failed: %s", rospy.get_name(), e)

        if (self.paramsModified and ret_val == True):
            self.paramsModified() # Callback provided by container class to update based on param server, etc.

        return ret_val




