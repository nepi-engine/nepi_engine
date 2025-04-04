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

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_ros_interfaces.msg import SystemTrigger
from nepi_ros_interfaces.srv import SystemTriggersQuery, SystemTriggersQueryRequest, SystemTriggersQueryResponse

from nepi_api.node_if import NodeClassIF
from nepi_api.sys_if_msg import MsgIF


EXAMPLE_TRIGGER_DICT = {"name":"None",
                        "description": "None",
                        "data_str_list":["None"],
                        "time":nepi_utils.get_time(),
                        "node":"None"
}

EXAMPLE_TRIGGERS_DICT = {"None":EXAMPLE_TRIGGER_DICT}



class TriggersIF(object):


    #######################
    ### IF Initialization
    def __init__(self, triggers_dict = None):
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
        


        if triggers_dict is None:
            self.triggers_dict = dict()
        else:
            self.triggers_dict = triggers_dict
        ##############################  
        # Create NodeClassIF Class  



        # Services Config Dict ####################
        self.SRVS_DICT = {
            'trigger_query': {
                'topic': 'system_triggers_query',
                'msg': SystemTriggersQuery,
                'callback': self._triggersQueryHandler
            }
        }

        self.SRVS_CONFIG_DICT = {
                'namespace': '~',
                'srvs_dict': self.SRVS_DICT
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'trigger_pub': {
                'msg': SystemTrigger,
                'topic': 'system_trigger',
                'qsize': 1,
                'latch': False
            }
        }

        self.PUBS_CONFIG_DICT = {
            'namespace': None,
            'pubs_dict': self.PUBS_DICT
        }


        self.class_if = NodeClassIF(srvs_config_dict = self.SRVS_CONFIG_DICT,
                        pubs_config_dict = self.PUBS_CONFIG_DICT
                        )




        ##############################
        self.msg_if.pub_info("IF Initialization Complete")




    ###############################
    # Class Public Methods
    ###############################


    def publish_trigger(self, trigger_dict):
        trig_msg = SystemTrigger()
        trig_msg.header.stamp = nepi_ros.get_ros_stamp_from_sec(trigger_dict['time'])
        trig_msg.name = trigger_dict['name']
        trig_msg.description = trigger_dict['description']
        trig_msg.data_str_list = trigger_dict['data_str_list']
        trig_msg.node = trigger_dict['node_name']
        self.class_if.publish_pub('trigger_pub',trig_msg)
 

    ###############################
    # Class Private Methods
    ###############################

    def _triggersQueryHandler(self, req):
        resp = SystemTriggersQueryResponse()
        try:
            resp = nepi_triggers.create_query_resp_from_triggers_dict(self.node_name,self.triggers_dict)
        except:
            self.msg_if.pub_info("Failed to create resp msg: " + str(e))
        return resp


