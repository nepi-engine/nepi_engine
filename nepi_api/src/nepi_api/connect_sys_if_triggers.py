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

EXAMPLE_TRIGGERS_DICT = {"None":EXAMPLE_TRIGGER_DICT}



class ConnectTriggerIF(object):

    trigger_handler = None
    trigger_check = None
    last_trigger_time = None

    #######################
    ### IF Initialization
    def __init__(self, trigger_handler_function, trigger_name = "All"):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        log_name = self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        

        #############################
        self.trigger_handler = trigger_handler_function



        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'trigger_query': {
                'topic': 'system_triggers_query',
                'msg': SystemTriggersQuery,
            }
        }

        self.SRVS_CONFIG_DICT = {
                'namespace': '~',
                'srvs_dict': self.SRVS_DICT
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'trigger_sub': {
                'msg': SystemTrigger,
                'topic': 'system_trigger',
                'qsize': 1,
                'callback': self._triggerCb,
                'callback_args': ()
            }
        }

        self.SUBS_CONFIG_DICT = {
            'namespace': None,
            'subs_dict': self.SUBS_DICT
        }


        self.class_if = ConnectNodeClassIF(srvs_config_dict = self.SRVS_CONFIG_DICT,
                        subs_config_dict = self.SUBS_CONFIG_DICT
                        )


        ##############################
        self.msg_if.pub_info("IF Initialization Complete")




    ###############################
    # Class Public Methods
    ###############################

    def get_trigger_info(self):
        req = SystemTriggersQueryRequest()
        self.class_if.call_srv('trigger_query',req)


    def get_last_trigger_time(self):
        return self.last_trigger_time

    def unregister_trigger_if(self): 
        success = False
        if self.connected is False or self.trigger_check is None:
            self.msg_if.pub_warn("Trigger IF not running")
            success = True
        else:
            self.msg_if.pub_info("Killing Trigger IF for trigger: " + self.trigger_check)
            try:
                self.trigger_sub.unregister()
                success = True
            except:
                pass
            time.sleep(1)
            self.trigger_sub = None
            self.trigger_handler = None
            self.trigger_check = None
            self.last_trigger_time = None
        return success

    ###############################
    # Class Private Methods
    ###############################


    def _triggerCb(self,msg):
        trigger_dict = nepi_triggers.parse_trigger_msg(msg)
        trigger_name = trigger_dict['name']
        if self.trigger_handler is not None and self.trigger_check is not None:
            if self.trigger_check == "All" or self.trigger_check == trigger_name:
                try:
                    self.last_trigger_time = nepi_utils.get_time()
                    self.trigger_handler(trigger_dict)
                except:
                    self.msg_if.pub_info("Failed to call trigger handler function: " + str(e))

