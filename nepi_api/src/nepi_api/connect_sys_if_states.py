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
from nepi_sdk import nepi_states

from std_msgs.msg import String, Empty
from nepi_ros_interfaces.msg import SystemState, SystemStatesStatus
from nepi_ros_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.sys_if_msg import MsgIF

STATE_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]

EXAMPLE_STATE_DICT = {"name":"None",
                        "description": "None",
                        "type":"Bool",
                        "options": [],
                        "value":"False"
}

EXAMPLE_STATES_DICT = {"None":EXAMPLE_STATE_DICT}



class ConnectStatesIF(object):

    connected = False

    service_namespace = None

    service_name = 'system_states_query'
    service_request = SystemStatesQueryRequest()
    service = None


    #######################
    ### IF Initialization
    log_name = "ConnectStatesIF"
    def __init__(self, namespace, timeout = float('inf')):
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
        
        ############################
        self.get_states_dict_function = get_states_dict_function


        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'states_query': {
                'topic': 'system_states_query',
                'msg': SystemStatesQuery,
                'callback': self._statesQueryHandler
            }
        }

        self.SRVS_CONFIG_DICT = {
                'namespace': '~',
                'srvs_dict': self.SRVS_DICT
        }


        self.class_if = ConnectNodeClassIF(srvs_config_dict = self.SRVS_CONFIG_DICT
                        )


        ##############################
        self.msg_if.pub_info("IF Initialization Complete")




    ###############################
    # Class Public Methods
    ###############################



    def get_states_dict(self):
        req = SystemStatesQueryRequest()
        self.class_if.call_srv('states_query',req)

      
    def unregister_states_if(self): 
        success = False
        self.class_if.unregister_class()
        success = True
        return success  

    ###############################
    # Class Private Methods
    ###############################





