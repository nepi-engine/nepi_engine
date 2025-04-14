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
from nepi_sdk import nepi_states

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_ros_interfaces.msg import SystemState
from nepi_ros_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_api.node_if import NodeClassIF
from nepi_api.sys_if_msg import MsgIF

STATE_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]

EXAMPLE_STATE_DICT = {"name":"None",
                        "description": "None",
                        "type":"Bool",
                        "options": [],
                        "value":"False"
}


class StatesIF(object):


    ready = False
    msg_if = None
    namespace = '~'

    get_states_dict_function = None



    #######################
    ### IF Initialization
    log_name = "StatesIF"
    def __init__(self, get_states_dict_function, namespace = None):
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

        self.get_states_dict_function = get_states_dict_function


        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'states_query': {
                'namespace': self.namespace,
                'topic': 'system_states_query',
                'srv': SystemStatesQuery,
                'req': SystemStatesQueryRequest(),
                'resp': SystemStatesQueryResponse(),                
                'callback': self._statesQueryHandler
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(services_dict = self.SRVS_DICT)

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



    ###############################
    # Class Private Methods
    ###############################

    def _statesQueryHandler(self, req):
        resp = SystemStatesQueryResponse()
        states_dict = self.get_states_dict_function()
        try:
            resp = nepi_states.create_query_resp_from_states_dict(self.node_name,states_dict)
        except:
            self.msg_if.pub_info("Failed to create resp msg: " + str(e))
        return resp





