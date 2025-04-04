#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg

from std_msgs.msg import String, Empty
from nepi_ros_interfaces.msg import Reset


class ConnectSettingsIF(object):

    connected = False

    namespace = None

    config_save_pub = None
    config_reset_pub = None

    #######################
    ### IF Initialization
    log_name = "ConnectSaveConfigIF"
    def __init__(self, namespace , timeout = float('inf')):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting IF Initialization Processes")
        ##############################   

        # Find topic for namespace
        find_topic = os.path.join(namespace,'save_config')
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for topic: " + str(find_topic))
        found_topic = nepi_ros.wait_for_topic(find_topic, timeout = timeout)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Found topic: " + found_topic)
        if found_topic == "":
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to find topic: " + find_topic)
                
        else:
            config_save_topic = os.path.join(namespace, 'save_config')
            self.config_save_pub = rospy.Publisher(config_save_topic, Empty, queue_size=1) 

            config_reset_topic = os.path.join(namespace, 'reset_config')
            self.config_reset_pub = rospy.Publisher(config_reset_topic, Reset, queue_size=1) 

            time.sleep(1)

            self.namespace = namespace
            self.connected = True

        ##############################   
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": IF Initialization Complete")




    ###############################
    # Class Public Methods
    ###############################

    def check_connection(self):
        return self.connected

    def get_namespace(self):
        return self.namespace

    def save_config(self, config_dict):
        success = False
        if self.config_save_pub is not None:
            try:
                msg = Empty()
                self.config_save_pub.publish(msg)
                success = True
            except Exception as e:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed send update config: " + str(e))
        return success


    def reset_user_config(self):
        success = False
        if self.config_reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.USER_RESET
                self.config_reset_pub.publish(msg)
                success = True
            except Exception as e:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed send config reset: "  + str(e))
        return success

    def reset_factory_config(self):
        success = False
        if self.config_reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.FACTORY_RESET
                self.config_reset_pub.publish(msg)
                success = True
            except Exception as e:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed send config reset: "  + str(e))
        return success

    def unregister_save_config_if(self): 
        success = False
        if self.connected is False or self.namespace is None:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Save Config IF not running")
            success = True
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Killing Save Confg IF for trigger: " + self.namespace)
            try:
                self.config_save_pub.unregister()
                self.config_reset_pub.unregister()

                success = True
            except:
                pass
            time.sleep(1)
            self.connected = False
            self.config_save_pub = None
            self.config_reset_pub = None
            self.namespace = None
        return success       

    ###############################
    # Class Private Methods
    ###############################


