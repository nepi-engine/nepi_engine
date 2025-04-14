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
                        "node_name":"~"
}

EXAMPLE_TRIGGERS_DICT = {"None":EXAMPLE_TRIGGER_DICT}



class TriggersIF(object):

    msg_if = None
    ready = False
    namespace = '~'

    triggers_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, triggers_dict = None, namespace = None):
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

        if triggers_dict is None:
            self.triggers_dict = dict()
        else:
            self.triggers_dict = triggers_dict
        ##############################  
        # Create NodeClassIF Class  


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'trigger_query': {
                'namespace': self.base_namespace,
                'topic': 'system_triggers_query',
                'srv': SystemTriggersQuery,
                'req': SystemTriggersQueryRequest(),
                'resp': SystemTriggersQueryResponse(),
                'callback': self._triggersQueryHandler
            }
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'trigger_pub': {
                'msg': SystemTrigger,
                'namespace': self.base_namespace,
                'topic': 'system_trigger',
                'qsize': 1,
                'latch': False
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(services_dict = self.SRVS_DICT,
                                    pubs_dict = self.PUBS_DICT
                                )

        self.node_if.wait_for_ready()

        ##############################
        # Complete Initialization
        self.ready = True
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
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready  


    def publish_trigger(self, trigger_dict):
        trig_msg = SystemTrigger()
        trig_msg.header.stamp = nepi_ros.get_ros_stamp_from_sec(trigger_dict['time'])
        trig_msg.name = trigger_dict['name']
        trig_msg.description = trigger_dict['description']
        trig_msg.data_str_list = trigger_dict['data_str_list']
        trig_msg.node = trigger_dict['node_name']
        self.node_if.publish_pub('trigger_pub',trig_msg)
 

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


