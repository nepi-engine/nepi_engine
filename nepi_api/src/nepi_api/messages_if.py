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


class MsgIF(object):

    ns_str = ""
    ln_str = ""

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
        if log_name is not None:
            self.ln_str = log_name + ": "
        self._logSelfMsg("Starting IF Initialization Processes")
        ##############################   
        '''
        mgr_sys_if = ConnectMgrSystemServicesIF()
        success = mgr_sys_if.wait_for_services()
        if success == False:
            nepi_ros.signal_shutdown(self.node_name + ": Failed to get System Ready")
        print_debug = mgr_sys_if.get_system_debug_mode()
        if print_debug is not None: 
            self.print_debug = print_debug
        '''
        self._createMsgPublishers()
        ##############################
        self._logSelfMsg("IF Initialization Complete")


    ###############################
    # Class Public Methods
    
    def pub_msg(self, msg, level = "None", throttle_s = None):
        if msg is None:
            msg = "MSGIF got None msg"
        msg_str = self._createMsgString(msg)
        nepi_ros.log_msg(msg_str, level = level, throttle_s = throttle_s)
        self.msg_pub.publish(msg_str)
        self.msg_pub_sys.publish(msg_str)
    
    def pub_info(self, msg, throttle_s = None):
        self.pub_msg(msg, level = 'info', throttle_s = throttle_s)
    
    def pub_warn(self, msg, throttle_s = None):
        self.pub_msg(msg, level = 'warn', throttle_s = throttle_s)
    
    def pub_debug(self, msg, throttle_s = None):
        if self.print_debug == True:
            self.pub_msg(msg, level = 'debug', throttle_s = throttle_s)
    
    def pub_error(self, msg, throttle_s = None):
        self.pub_msg(msg,level = 'error', throttle_s = throttle_s)
    
    def pub_fatal(self, msg, throttle_s = None):
        self.pub_msg(msg,level = 'fatal', throttle_s = throttle_s)



    ###############################
    # Class Private Methods
    ###############################
  
    def _createMsgPublishers(self):
        #self._logSelfMsg("Creating Msg Publishers")
        self.msg_pub = nepi_ros.create_publisher("~messages", Message, queue_size=1)
        self.msg_pub_sys = nepi_ros.create_publisher("messages", Message, queue_size=1)
        
        return 
    def _logSelfMsg(self,msg):
        msg_str = self.ns_str + self.ln_str + self.class_name + ": " + str(msg)
        nepi_ros.log_msg_info(msg_str)

    def _createMsgString(self,msg):
         return self.ns_str + self.ln_str + str(msg)


