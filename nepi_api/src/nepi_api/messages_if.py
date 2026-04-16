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
from nepi_sdk import nepi_system

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
        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)

        self._createMsgPublishers()
        ##############################
        self._logSelfMsg("IF Initialization Complete")


    ###############################
    # Class Public Methods
    
    def updaterCb(self,timer):
        self.debug_mode = nepi_system.get_debug_mode()
        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)




    def pub_msg(self, msg, level = "None", log_name_list = [], throttle_s = None, uid = None):
        """Publishes a message to ROS logging and the NEPI message topics.

        Formats the message string with node name and optional log-name prefix chain,
        logs it at the requested level, and publishes it to both the node-local
        ``~messages`` topic and the system-wide ``messages`` topic. Debug-level
        messages are only published when debug mode is active.

        If throttle_s and uid are both provided, the message is suppressed if less
        than throttle_s seconds have elapsed since the last publication for that uid.

        Args:
            msg (str): The message body to publish.
            level (str, optional): Log level string — one of ``'info'``, ``'warn'``,
                ``'debug'``, ``'error'``, or ``'fatal'``. Defaults to ``'None'``.
            log_name_list (list, optional): Ordered list of context label strings
                prepended to the message. Defaults to [].
            throttle_s (float, optional): Minimum interval in seconds between
                publications of the same uid. Defaults to None (no throttling).
            uid (str, optional): Unique key used to track throttle timing. Required
                for throttling to take effect. Defaults to None.
        """
        msg = str(msg)
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
        """Publishes an info-level message to ROS logging and the NEPI message topics.

        When throttle_s is specified, a uid is automatically derived from the caller's
        filename, function name, and line number so that repeated calls from the same
        source are suppressed until throttle_s seconds have elapsed.

        Args:
            msg (str): The message body to publish.
            throttle_s (float, optional): Minimum interval in seconds between
                publications from the same call site. Defaults to None (no throttling).
            log_name_list (list, optional): Ordered list of context label strings
                prepended to the message. Defaults to [].
        """
        msg = str(msg)
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
        """Publishes a warning-level message to ROS logging and the NEPI message topics.

        When throttle_s is specified, a uid is automatically derived from the caller's
        filename, function name, and line number so that repeated calls from the same
        source are suppressed until throttle_s seconds have elapsed.

        Args:
            msg (str): The message body to publish.
            throttle_s (float, optional): Minimum interval in seconds between
                publications from the same call site. Defaults to None (no throttling).
            log_name_list (list, optional): Ordered list of context label strings
                prepended to the message. Defaults to [].
        """
        msg = str(msg)
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
        """Publishes a debug-level message to ROS logging and the NEPI message topics.

        The message is only forwarded to the ROS topics when the node's debug mode
        is active; it is always passed to the ROS logger. When throttle_s is specified,
        a uid is derived from the caller's filename, function name, and line number to
        suppress repeated messages until throttle_s seconds have elapsed.

        Args:
            msg (str): The message body to publish.
            throttle_s (float, optional): Minimum interval in seconds between
                publications from the same call site. Defaults to None (no throttling).
            log_name_list (list, optional): Ordered list of context label strings
                prepended to the message. Defaults to [].
        """
        msg = str(msg)
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
        """Publishes an error-level message to ROS logging and the NEPI message topics.

        When throttle_s is specified, a uid is automatically derived from the caller's
        filename, function name, and line number so that repeated calls from the same
        source are suppressed until throttle_s seconds have elapsed.

        Args:
            msg (str): The message body to publish.
            throttle_s (float, optional): Minimum interval in seconds between
                publications from the same call site. Defaults to None (no throttling).
            log_name_list (list, optional): Ordered list of context label strings
                prepended to the message. Defaults to [].
        """
        msg = str(msg)
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
        """Publishes a fatal-level message to ROS logging and the NEPI message topics.

        When throttle_s is specified, a uid is automatically derived from the caller's
        filename, function name, and line number so that repeated calls from the same
        source are suppressed until throttle_s seconds have elapsed.

        Args:
            msg (str): The message body to publish.
            throttle_s (float, optional): Minimum interval in seconds between
                publications from the same call site. Defaults to None (no throttling).
            log_name_list (list, optional): Ordered list of context label strings
                prepended to the message. Defaults to [].
        """
        msg = str(msg)
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


    def _logSelfMsg(self, msg, log_name_list = []):
        msg = str(msg)
        ln_str = self._createLogNameStr(log_name_list)
        msg_str = self.ns_str + ln_str + self.ln_str + self.cn_str + str(msg)
        nepi_sdk.log_msg_info(msg_str)

    def _createMsgString(self,msg, log_name_list = []):
        msg = str(msg)
        ln_str = self._createLogNameStr(log_name_list)
        msg_str = self.ns_str + ln_str + self.ln_str + str(msg)
        return msg_str

    def _createLogNameStr(self,log_name_list):
        ln_str = ""
        for log_name in log_name_list:
            ln_str = ln_str + log_name + ": "
        return ln_str



