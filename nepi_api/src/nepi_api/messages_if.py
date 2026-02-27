#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

import inspect

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_interfaces.msg import Message


class MsgIF:

    ns_str = ""
    cn_str = ""
    ln_str = ""
    log_name_list = []

    debug_mode = False

    throttle_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, log_name = None):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.node_name = nepi_sdk.get_node_name()
        self.base_namespace = nepi_sdk.get_base_namespace()

        ############################## 
        self.ns_str = self.node_name + ": "
        self.cn_str = self.class_name + ": "
        if log_name is not None:
            self.ln_str = str(log_name) + ": "
        self._logSelfMsg("Starting IF Initialization Processes", )
        ##############################   
        nepi_sdk.create_subscriber('debug_mode', Bool, self._debugCb, queue_size = 10)

        self._createMsgPublishers()
        ##############################
        self._logSelfMsg("IF Initialization Complete")


    ###############################
    # Class Public Methods
    
    def pub_msg(self, msg, level = "None", log_name_list = [], throttle_s = None, uid = None):

        if throttle_s is not None:
            ct = nepi_utils.get_time()
            if uid is not None:
                if uid not in self.throttle_dict.keys():
                    self.throttle_dict[uid] = ct
                else:
                    lt = self.throttle_dict[uid]
                    if (ct-lt) > throttle_s:
                        self.throttle_dict[uid] = ct
                    else:
                        return
            
        if msg is None:
            msg = "MSGIF got None msg"
        msg_str = self._createMsgString(msg, log_name_list = log_name_list)
        nepi_sdk.log_msg(msg_str, level = level)
        if level != 'debug':
            self.msg_pub.publish(msg_str)
            self.msg_pub_sys.publish(msg_str)
        elif self.debug_mode == True:
            self.msg_pub.publish(msg_str)
            self.msg_pub_sys.publish(msg_str)
    
    def pub_info(self, msg, throttle_s = None, log_name_list = []):
        uid = None
        if throttle_s is not None:
            stack = inspect.stack()
            frame = stack[1]
            try:
                uid = frame.filename+frame.function+str(frame.lineno)
            except:
                print("Failed to create msg uid with frame: " + str(frame))
        self.pub_msg(msg, level = 'info', log_name_list = log_name_list, throttle_s = throttle_s, uid = uid)
    
    def pub_warn(self, msg, throttle_s = None, log_name_list = []):
        uid = None
        if throttle_s is not None:
            stack = inspect.stack()
            frame = stack[1]
            try:
                uid = frame.filename+frame.function+str(frame.lineno)
            except:
                print("Failed to create msg uid with frame: " + str(frame))
        self.pub_msg(msg, level = 'warn', log_name_list = log_name_list, throttle_s = throttle_s, uid = uid)
    
    def pub_debug(self, msg, throttle_s = None, log_name_list = []):
        uid = None
        if throttle_s is not None:
            stack = inspect.stack()
            frame = stack[1]
            try:
                uid = frame.filename+frame.function+str(frame.lineno)
            except:
                print("Failed to create msg uid with frame: " + str(frame))
        self.pub_msg(msg, level = 'debug', log_name_list = log_name_list, throttle_s = throttle_s, uid = uid)
    
    def pub_error(self, msg, throttle_s = None, log_name_list = []):
        uid = None
        if throttle_s is not None:
            stack = inspect.stack()
            frame = stack[1]
            try:
                uid = frame.filename+frame.function+str(frame.lineno)
            except:
                print("Failed to create msg uid with frame: " + str(frame))
        self.pub_msg(msg,level = 'error', log_name_list = log_name_list, throttle_s = throttle_s, uid = uid)
    
    def pub_fatal(self, msg, throttle_s = None, log_name_list = []):
        uid = None
        if throttle_s is not None:
            stack = inspect.stack()
            frame = stack[1]
            try:
                uid = frame.filename+frame.function+str(frame.lineno)
            except:
                print("Failed to create msg uid with frame: " + str(frame))
        self.pub_msg(msg,level = 'fatal', log_name_list = log_name_list, throttle_s = throttle_s, uid = uid)



    ###############################
    # Class Private Methods
    ###############################
  
    def _createMsgPublishers(self):
        self.msg_pub = nepi_sdk.create_publisher("~messages", Message, queue_size=1)
        self.msg_pub_sys = nepi_sdk.create_publisher("messages", Message, queue_size=1)


    def _logSelfMsg(self,msg, log_name_list = []):
        ln_str = self._createLogNameStr(log_name_list)
        msg_str = self.ns_str + ln_str + self.ln_str + self.cn_str + str(msg)
        nepi_sdk.log_msg_info(msg_str)

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
        enabled = msg.data
        if self.debug_mode != enabled:
            nepi_sdk.set_debug_log(enabled)
        self.debug_mode = enabled

