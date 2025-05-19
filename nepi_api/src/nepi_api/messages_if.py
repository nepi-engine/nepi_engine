#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_ros_interfaces.msg import Message
from nepi_ros_interfaces.srv import DebugQuery, DebugQueryRequest, DebugQueryResponse




class MsgIF:

    ns_str = ""
    cn_str = ""
    ln_str = ""
    log_name_list = []

    print_debug = False
    #######################
    ### IF Initialization
    def __init__(self, log_name = None):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()

        ############################## 
        self.ns_str = self.node_name + ": "
        self.cn_str = self.class_name + ": "
        if log_name is not None:
            self.ln_str = str(log_name) + ": "
        self._logSelfMsg("Starting IF Initialization Processes", )
        ##############################   
        nepi_ros.create_subscriber('debug_mode', Bool, self._debugCb, queue_size = 10)

        self._createMsgPublishers()
        ##############################
        self._logSelfMsg("IF Initialization Complete")


    ###############################
    # Class Public Methods
    
    def pub_msg(self, msg, level = "None", log_name_list = [], throttle_s = None):
        if msg is None:
            msg = "MSGIF got None msg"
        msg_str = self._createMsgString(msg, log_name_list = log_name_list)
        nepi_ros.log_msg(msg_str, level = level, throttle_s = throttle_s)
        self.msg_pub.publish(msg_str)
        self.msg_pub_sys.publish(msg_str)
    
    def pub_info(self, msg, throttle_s = None, log_name_list = []):
        self.pub_msg(msg, level = 'info', log_name_list = log_name_list, throttle_s = throttle_s)
    
    def pub_warn(self, msg, throttle_s = None, log_name_list = []):
        self.pub_msg(msg, level = 'warn', log_name_list = log_name_list, throttle_s = throttle_s)
    
    def pub_debug(self, msg, throttle_s = None, log_name_list = []):
        if self.print_debug == True:
            self.pub_msg(msg, level = 'debug', log_name_list = log_name_list, throttle_s = throttle_s)
    
    def pub_error(self, msg, throttle_s = None, log_name_list = []):
        self.pub_msg(msg,level = 'error', log_name_list = log_name_list, throttle_s = throttle_s)
    
    def pub_fatal(self, msg, throttle_s = None, log_name_list = []):
        self.pub_msg(msg,level = 'fatal', log_name_list = log_name_list, throttle_s = throttle_s)



    ###############################
    # Class Private Methods
    ###############################
  
    def _createMsgPublishers(self):
        self.msg_pub = nepi_ros.create_publisher("~messages", Message, queue_size=1)
        self.msg_pub_sys = nepi_ros.create_publisher("messages", Message, queue_size=1)


    def _logSelfMsg(self,msg, log_name_list = []):
        ln_str = self._createLogNameStr(log_name_list)
        msg_str = self.ns_str + ln_str + self.ln_str + self.cn_str + str(msg)
        nepi_ros.log_msg_info(msg_str)

    def _createMsgString(self,msg, log_name_list = []):
        ln_str = self._createLogNameStr(log_name_list)
        msg_str = self.ns_str + ln_str + self.ln_str + str(msg)
        return msg_str

    def _createLogNameStr(self,log_name_list):
        ln_str = ""
        for log_name in log_name_list:
            ln_str = ln_str + log_name + ": "
        return ln_str

    def _debugCb(self,msg):
        self.print_debug = msg.data
