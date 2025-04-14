#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import rospy
import os
import time
import copy

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_triggers

from std_msgs.msg import String, Empty
from nepi_ros_interfaces.msg import SystemTrigger, SystemTriggersStatus
from nepi_ros_interfaces.srv import SystemTriggersQuery, SystemTriggersQueryRequest, SystemTriggersQueryResponse

from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.sys_if_msg import MsgIF

EXAMPLE_TRIGGER_DICT = {"name":"None",
                        "description": "None",
                        "data_str_list":["None"],
                        "time":nepi_utils.get_time(),
                        "node":"None"
}

EXAMPLE_TRIGGERS_DICT = {"example_trigger":EXAMPLE_TRIGGER_DICT}



class ConnectTriggersIF(object):

    msg_if = None
    ready = False

    trigger_handlers_dict = dict()

    status_msg = None

    #######################
    ### IF Initialization
    def __init__(self):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        

        #############################
        # Initialize Class Variables
        if namespace is None:
            namespace = '~'
        self.namespace = nepi_ros.get_full_namespace(namespace)


        ##############################  
        # Create NodeClassIF Class  

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'namespace': self.base_namespace,
                'msg': SystemTriggersStatus,
                'topic': 'system_triggers_status',
                'qsize': 1,
                'callback': self._statusSubCb,
                'callback_args': ()
            },
            'trigger_sub': {
                'namespace': self.base_namespace,
                'msg': SystemTrigger,
                'topic': 'system_trigger',
                'qsize': 1,
                'callback': self._triggerCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(subs_dict = self.SUBS_DICT)

        self.con_node_if.wait_for_ready()

        ##############################
        # Complete Initialization
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.ready == False and self.status_msg is None and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready  


    def get_triggers_names(self):
        trigger_names = []
        if self.status_sub is not None:
            if self.status_msg is not None:
                trigger_names = self.status_msg.triggers_names_list
        else:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Trigger Status listener has not received any data yet: ")
        return trigger_names


    def get_triggers_status_dict(self):
        triggers_dict = None
        if self.status_sub is not None:
            if self.status_msg is not None:
                triggers_dict = nepi_triggers.parse_triggers_status_msg(self.status_msg)
        else:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Trigger Status listener has not received any data yet: ")
        return triggers_dict



    def get_registered_triggers(self):
        return self.trigger_handlers_dict.keys()

    def register_trigger(self, trigger_handler_function, trigger_name = "All"):
        success = False
        th_dict = dict()
        th_dict['handler'] = trigger_handler_function
        th_dict['last_time'] = nepi_utils.get_time()
        self.trigger_handlers_dict[trigger_name] = th_dict
        success = True
        return success

    def unregister_trigger(self, trigger_name):
        success = False
        try:
            del self.trigger_handlers_dict[trigger_name]
            success = True
        except Exception as e:
            self.msg_if.pub_info("No trigger handler registed for trigger: " + trigger_name + " " + str(e))
        return success


    def get_last_trigger_time(self, trigger_name):
        last_time = None
        try:
            last_time self.trigger_handlers_dict[trigger_name]['last_time']
        except Exception as e:
            self.msg_if.pub_info("No trigger handler registed for trigger: " + trigger_name + " " + str(e))
        return self.last_time

 



    ###############################
    # Class Private Methods
    ###############################


    def _triggerCb(self,msg):
        trigger_dict = nepi_triggers.parse_trigger_msg(msg)
        trigger_name = trigger_dict['name']
        if "All" in self.trigger_handlers_dict.keys():
            self.trigger_handlers_dict["All"]['last_time'] = nepi_utils.get_time()
            try:
                self.trigger_handlers_dict["All"]['handler'](trigger_dict)
            except:
                self.msg_if.pub_info("Failed to call trigger handler function: " + str(e))
        if trigger_name in self.trigger_handlers_dict.keys():
            self.trigger_handlers_dict[trigger_name]['last_time'] = nepi_utils.get_time()
            try:
                self.trigger_handlers_dict[trigger_name]['handler'](trigger_dict)
            except:
                self.msg_if.pub_info("Failed to call trigger handler function: " + str(e))

    # Update System Status
    def _statusCb(self,msg):
        self.status_msg = msg
        self.ready = True