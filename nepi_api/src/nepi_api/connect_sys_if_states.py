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

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_ros_interfaces.msg import SystemState, SystemStatesStatus
from nepi_ros_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_api.connect_con_node_if import ConnectNodeClassIF
from nepi_api.sys_if_msg import MsgIF

STATE_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]

EXAMPLE_STATE_DICT = {"name":"None",
                        "description": "None",
                        "type":"Bool",
                        "options": [],
                        "value":"False"
}

EXAMPLE_STATES_DICT = {"example_state":EXAMPLE_STATE_DICT}



class ConnectStatesIF(object):

    msg_if = None
    ready = False
    namespace = '~'

    status_msg = None


    #######################
    ### IF Initialization
    log_name = "ConnectStatesIF"
    def __init__(self):
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
                'msg': SystemStatesStatus,
                'topic': 'system_states_status',
                'qsize': 1,
                'callback': self._statusSubCb,
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


    def get_states_status_dict(self):
        states_dict = None
        if self.status_sub is not None:
            if self.status_msg is not None:
                states_dict = nepi_states.parse_states_status_msg(self.status_msg)
        else:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ": state Status listener has not received any data yet: ")
        return states_dict


    def get_states_names(self):
        state_names = []
        if self.status_sub is not None:
            if self.status_msg is not None:
                state_names = self.status_msg.states_names_list
        else:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ": state Status listener has not received any data yet: ")
        return state_names

    def get_state_value(self, state_name):
        state_value = None
        states_dict = self.get_states_status_dict()
        if states_dict is not None:
            if state_name in states_dict.keys():
                state_dict = states_dict[state_name]
                state_value = nepi_states.get_data_from_state_dict(state_dict)
        return state_names


    ###############################
    # Class Private Methods
    ###############################

    # Update System Status
    def _statusCb(self,msg):
        self.status_msg = msg
        self.ready = True

