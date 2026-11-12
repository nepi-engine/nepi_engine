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

import os
import time 
import copy
import threading
import inspect
import re

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_dict
# from nepi_sdk import nepi_settings
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data
from nepi_sdk import nepi_nav

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_interfaces.msg import UpdateString


from nepi_interfaces.msg import NodeIFStatus, MgrSystemStatus
from nepi_interfaces.srv import NodeIFStatusQuery, NodeIFStatusQueryRequest, NodeIFStatusQueryResponse

from nepi_interfaces.msg import Message, SettingsStatus, SaveDataStatus, TransformStatus, NavPoseStatus
from nepi_interfaces.msg import SystemStatesStatus, SystemTriggersStatus
from nepi_interfaces.msg import Transform, NavPose, SystemStates, SystemTriggers



from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyRequest as EmptySrvRequest
from std_srvs.srv import EmptyResponse as EmptySrvResponse


from nepi_api.messages_if import MsgIF

#########################################
# Node IF Class
#########################################

#################
# node_if Configs Updates
#################
## If if a node_if is not passed in the if_dict,
## a new node_if class is created with the following 
## class callback functions that can by overriden by Child Class

# self.init(self, do_updates = False)
# self.reset(self, do_updates = True)
# self.factory_reset(self, do_updates = True)

## These class functions also try to call the following functions
## at the end of the self.init, self.reset, and self.factory_reset functions

# self.initCb(self, do_updates = False)
# self.initCb(self)
# self.resetCb(self, do_updates = True)
# self.resetCb(self)
# self.factoryResetCb(self, do_updates = True)
# self.factoryResetCb(self)


#################
# node_if Params Updates
#################
## Creates or Adds the following params to node_if,
## depending on whether a node_if is passed in the if_dict

# 'display_name'

#################
# node_if Services
#################
## Creates or Adds the following services-callback_functions to node_if,
## depending on whether a node_if is passed in the if_dict
## The callback functions can be overrided by Child Class

# self.namespace/if_status_query - self._nodeStatusQueryCb

#################
# node_if Publishers Updates
#################
## Creates or Adds the following publishers to node_if,
## depending on whether a node_if is passed in the if_dict

# 'status_pub'

#################
# node_if Subscribers Updates
#################
## Creates or Adds the following subscribers-callback_functions to node_if,
## depending on whether a node_if is passed in the if_dict
## The callback functions can be overrided by Child Class

# self.namespace/set_display_name - self._setDisplayNameCb
# self.base_namespace/status - self._systemStatusCb

#############################################################

#################
# Node IF Function Overrides
#################

# NONE

#############################################################

#################
# Info Dictionary
#################

## The Node IF IF_INFO_DICT is updated in constructor
## any values passed in info_dict by a Child Class 
## are then added or updated

IF_INFO_DICT = dict(
    timestamp = 0,

    id = '',
    name = '',
    display_name = '',
    description = '',

    node_name = '',
    node_namespace = '',
    namespace = '',

    device_name = 'NEPI Device',
    serial_num = '999999',
    hw_version = '0',
    sw_version = '0',

    type = '', # Set by Child IF Class.
    sub_type = '', # Caller ID
    group = '', # First Topic after Node Namespace if exists
    sub_group = '', # second Topic after Node Namespace if exists

    source_description = '',
    source_ref_description = '',
    end_description = '',
    end_ref_description = '',
)


#################
# Capabilities Dictionary
#################
IF_CAPS_DICT = dict(
    has_name_control = False,
)

#################
# Stats Dictionary
#################

IF_STATS_DICT= dict(
    msg_str = '',
    state = False,
)
'''
### USE the self.set_value(name,value) to upate value
### USE the self.get_value(name) to get value
'''
#################
# RUI Dictionary
#################

IF_RUI_DICT = dict()

#################
# Config Control Group
#################
BLANK_CONFIG_CONTROL_DICT = dict(
    controls_if = None,
    controls_dict = dict(),

    data_if = None,
    data_dict = dict(),

    config_dict = dict()
)

'''
########################################
##### UPDATE CONFIG CONTROL DICT

config_control_dict = dict(
    controls_if = self.config_controls_if,
    controls_dict = self.config_control_dict,
    data_if = None,
    data_dict = self.none_data_dict
    config_dict = self.none_config_dict
)


### THEN EITHER
    #####
    # 1) Pass the node controls dict in IF contstructor
    node_config_dict = dict(config_control = config_control_dict)
    
    #####
    # 2) Use class's self.update_config_control_dict function
    # with the components to update in args

    self.update_config_control_dict(controls_if = self.config_controls_if, data_dict = self.none_data_dict)

### USE the self.set_value(name,value) to upate value
### USE the self.get_value(name) to get value
########################################
'''

#################
# Node Control Groups
#################

BLANK_CONTROL_DICT = dict(
    name = 'control_group',
    display_name = None,
    namespace = None,

    controls_if = None,
    controls_dict = dict(),

    data_if = None,
    data_dict = dict(),

    config_dict = dict()
)


'''
########################################
##### EXAMPLE NODE CONTROLS DICT

none_control_dict = dict(
    name = 'none_control',
    display_name = 'None Control'
    controls_if = self.none_controls_if,
    controls_dict = self.none_controls_dict,
    data_if = None,
    data_dict = self.none_data_dict
    config_dict = self.none_config_dict
)


### THEN EITHER
    #####
    # 1) Pass the node controls dict in IF contstructor
    node_controls_dict = dict(none_control = none_control_dict)
    
    #####
    # 2) Use class's self.set_node_control_dict function
    self.set_node_control_dict(none_control_dict, 'none_control')
    
########################################
'''

#################
# Topics Dictionaries
#################


BLANK_TOPIC_DICT = dict(
    name = '',
    display_name = '',
    type = '',
    sub_type = '',
    group = '',
    sub_group = '',
    status_msg_type = '',
    data_msg_type = '',
    lock_topic = False,
    available_topics = [],
    available_topic_names = [],
    topic = '',
    topic_name = '',
    connecting = False,
    connected = False,
    status_dict = None,
)

########################

SYSTEM_TOPICS_DICT = dict()

add_dict  = copy.deepcopy(BLANK_TOPIC_DICT)
add_dict['name'] = 'msg'
add_dict['status_msg_type'] = Message.__name__
add_dict['data_msg_type'] = Message.__name__
SYSTEM_TOPICS_DICT[add_dict['name']] = copy.deepcopy(add_dict)

add_dict = copy.deepcopy(BLANK_TOPIC_DICT)
add_dict['name'] = 'node'
add_dict['status_msg_type'] = ''
add_dict['data_msg_type'] = ''
SYSTEM_TOPICS_DICT[add_dict['name']] = copy.deepcopy(add_dict)

add_dict = copy.deepcopy(BLANK_TOPIC_DICT)
add_dict['name'] = 'settings'
add_dict['status_msg_type'] = SettingsStatus.__name__
add_dict['data_msg_type'] = SettingsStatus.__name__
SYSTEM_TOPICS_DICT[add_dict['name']] = copy.deepcopy(add_dict)


add_dict = copy.deepcopy(BLANK_TOPIC_DICT)
add_dict['name'] = 'save_data'
add_dict['status_msg_type'] = SaveDataStatus.__name__
add_dict['data_msg_type'] = SaveDataStatus.__name__
SYSTEM_TOPICS_DICT[add_dict['name']] = copy.deepcopy(add_dict)

add_dict = copy.deepcopy(BLANK_TOPIC_DICT)
add_dict['name'] = 'transform'
add_dict['status_msg_type'] = TransformStatus.__name__
add_dict['data_msg_type'] = Transform.__name__
SYSTEM_TOPICS_DICT[add_dict['name']] = copy.deepcopy(add_dict)


add_dict = copy.deepcopy(BLANK_TOPIC_DICT)
add_dict['name'] = 'navpose'
add_dict['status_msg_type'] = NavPoseStatus.__name__
add_dict['data_msg_type'] = NavPose.__name__
SYSTEM_TOPICS_DICT[add_dict['name']] = copy.deepcopy(add_dict)

add_dict = copy.deepcopy(BLANK_TOPIC_DICT)
add_dict['name'] = 'states'
add_dict['status_msg_type'] = SystemStatesStatus.__name__
add_dict['data_msg_type'] = SystemStates.__name__
SYSTEM_TOPICS_DICT[add_dict['name']] = copy.deepcopy(add_dict)

add_dict = copy.deepcopy(BLANK_TOPIC_DICT)
add_dict['name'] = 'triggers'
add_dict['status_msg_type'] = SystemTriggersStatus.__name__
add_dict['data_msg_type'] = SystemTriggers.__name__
SYSTEM_TOPICS_DICT[add_dict['name']] = copy.deepcopy(add_dict)

add_dict = copy.deepcopy(BLANK_TOPIC_DICT)
add_dict['name'] = 'save_config'
add_dict['status_msg_type'] = ''
add_dict['data_msg_type'] = ''
SYSTEM_TOPICS_DICT[add_dict['name']] = copy.deepcopy(add_dict)

########################
# Status msg *_topic field filled from each System Topics Dict entry's topic

SYSTEM_TOPICS_STATUS_FIELDS = dict(
    msg = 'msgs_topic',
    node = 'node_topic',
    settings = 'settings_topic',
    transform = 'transform_topic',
    save_data = 'save_data_topic',
    navpose = 'navpose_topic',
    states = 'states_topic',
    triggers = 'triggers_topic',
    save_config = 'config_topic'
)






################################


SYSTEM_IF_DICT = dict(
    msg_if = None,
    node_if = None,
    settings_if = None,
    save_data_if = None,
    transform_if = None,
    navpose_if = None,
    states_if = None,
    triggers_if = None
)


IF_CALLBACK_DICT = dict(
    status = None,
    data = None,
    navpose = None,
    states = None,
    triggers = None
)



class NodeIF:

    caller_class = 'Unknown'

    id = 'nepi_node'
    name = 'nepi_node'
    display_name = 'NEPI'
    description = 'NEPI Node'

    data_products = []

    namespace = None
    node_if_prefix = ''

    if_status_msg = NodeIFStatus()
    if_status_msg_class = NodeIFStatus
    if_query_class = NodeIFStatusQuery
    if_request_class = NodeIFStatusQueryRequest
    if_response_class = NodeIFStatusQueryResponse

    if_ready = False
    if_started = False

    msg_if = None
    node_if = None
    settings_if = None
    save_data_if = None
    transform_if = None
    navpose_if = None
    states_if = None
    triggers_if = None

    status_has_published = False

    #######

    status_msg = None
    status_class = None
    status_type = None

    data_msg = None
    data_class = None
    data_type = None

    #######
    info_dict = copy.deepcopy(IF_INFO_DICT)
    caps_dict = copy.deepcopy(IF_CAPS_DICT)
    stats_dict = copy.deepcopy(IF_STATS_DICT)
    rui_dict = copy.deepcopy(IF_RUI_DICT)

    #######
    config_control_dict = copy.deepcopy(BLANK_CONFIG_CONTROL_DICT)

    #######
    node_controls_dict = dict()

    #################################
    # Node Topics
    node_topic_dict = copy.deepcopy(BLANK_TOPIC_DICT)

    #################################
    # Node Used Topics
    if_dict = copy.deepcopy(SYSTEM_IF_DICT)
    node_topics_dict = copy.deepcopy(SYSTEM_TOPICS_DICT)
    node_topics_lock = threading.Lock()
    is_topic = 'None'

    #################################
    # Node Connected Topics
    connect_topics_dict = dict()
    connect_if_dict = dict()
   



    ######
    callback_dict = copy.deepcopy(IF_CALLBACK_DICT)

    ###########
    active_nodes = []
    active_topics = []
    active_topic_types =  []
    active_services =  []  

    system_config_dict = dict()


    #######################
    ### IF Initialization
    def __init__(self, 
                id = None,
                name = None,
                namespace = None,
                status_msg = None,
                data_msg = None,
                info_dict = dict(), # self.info_dict can be created/updated later
                caps_dict = dict(), # self.caps_dict can be created/updated later
                stats_dict = dict(), # self.config_controls_if can be created/updated later
                rui_dict = dict(), # self.rui_dict can be created/updated later
                config_control_dict = dict(), # Used to populate and initialize config components
                node_controls_dict = dict(), # Use self.add_control_group functio to add control groups later
                callback_dict = dict(), # Use class set_callback and clear_callback functions to update later
                if_dict = dict(), # self.if_dict can be created/updated later
                ):

        ########################################
        ####  IF INSTANCE STATE ####
        # The state dicts and lists are declared on the class. Copy each one onto
        # the instance, unless a Child Class set it before calling super().__init__,
        # so NodeIF-based objects in one process do not share them. if_dict,
        # connect_if_dict and callback_dict hold live objects and bound methods,
        # so they get a shallow copy, never a deepcopy.
        for state_name in ['info_dict', 'caps_dict', 'stats_dict', 'rui_dict',
                           'config_control_dict', 'node_controls_dict',
                           'node_topic_dict', 'node_topics_dict', 'connect_topics_dict',
                           'active_nodes', 'active_topics', 'active_topic_types', 'active_services',
                           'system_config_dict', 'if_status_msg']:
            if state_name not in self.__dict__:
                setattr(self, state_name, copy.deepcopy(getattr(type(self), state_name)))
        for state_name in ['if_dict', 'connect_if_dict', 'callback_dict']:
            if state_name not in self.__dict__:
                setattr(self, state_name, dict(getattr(type(self), state_name)))
        if 'node_topics_lock' not in self.__dict__:
            self.node_topics_lock = threading.Lock()
        
        ########################################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # MSG IF SETUP
        ##############################
        if if_dict.get('msg_if', None) is None:
            self.if_dict['msg_if'] = MsgIF()
        else:
            self.if_dict['msg_if'] = if_dict['msg_if']
        self.msg_if = self.if_dict['msg_if']
        self.node_topics_dict['msg']['topic'] = self.msg_if.get_namespace() 

        self.msg_if.pub_debug("Starting " + str(self.class_name) + " Initialization Processes")

        ##############################
        # Get System Config Dict
        ##############################
        self.msg_if.pub_debug("Loading system config")
        self.system_config_dict = nepi_system.load_nepi_system_config()


        ##############################  
        # Node IF Data Setup
        ##############################

        ################
        # Caller Class Info
        caller_frame = inspect.currentframe().f_back
        if 'self' in caller_frame.f_locals:
            self.caller_class_name = caller_frame.f_locals['self'].__class__.__name__
        else:
            self.caller_class = 'Unknown'


        ##############
        # ID and Name Data
        if id is None and name is not None:
            id = copy.deepcopy(name)
        elif id is not None and name is None:
            name = copy.deepcopy(id)
        elif id is None and name is None:
            raise ValueError(str(type(self).__name__) +
                                ": id or name is required)")

        if id.strip() == '' and name .strip() != '':
            id = copy.deepcopy(name)
        elif id.strip() != '' and name .strip() == '':
            name = copy.deepcopy(id)
        elif id.strip() == '' and name .strip() == '':
            raise ValueError(str(type(self).__name__) +
                                ": id or name is required)")

        self.id = nepi_sdk.get_clean_name(str(id))
        self.name = nepi_sdk.get_clean_name(str(name))
        display_name = info_dict.get('display_name','')
        if display_name.strip() == '':
            display_name = self.name.replace('_',' ').capitalize()
        self.display_name = display_name
        description = info_dict.get('description','')
        if description.strip() == '':
            description = self.display_name
        self.description = description


        ##################
        # Instance Namespace and Node IF Prefix
        if namespace is not None:
            # Strip the name only when it is the last segment, so a parent segment
            # that merely contains it (e.g. /idx_cam for name idx) is left intact
            for strip_name in [name, self.name]:
                if namespace.endswith('/' + strip_name):
                    namespace = namespace[:-len('/' + strip_name)]
                    break
            namespace = nepi_sdk.get_full_namespace(namespace)
        else:
            namespace = self.node_namespace

        namespace = namespace.replace('/' + name, '') 
        namespace = namespace.replace('/' + self.name, '') 

        self.msg_if.pub_info("Creating Node IF with name and namespace", data = [self.name, namespace], color = 'BLUE')

        self.namespace = nepi_sdk.create_namespace(namespace,self.name)
        self.node_if_prefix = self.namespace.replace(self.node_namespace + '/','') + '/'



        ##############################
        # Update Node Message Data
        ##############################

        ########################################
        # Setup Connect Status Settings
        if status_msg is None:
            raise ValueError(str(type(self).__name__) +
                                ": status_msg is required)")

        self.status_msg = status_msg
        try:
            status_class = status_msg.__class__
            status_type = status_class.__name__
        except Exception as e:
            self.msg_if.pub_debug("Failed to get status class)", data = str(e))
            raise ValueError(str(type(self).__name__) +
                                            ": Failed to get status class required)")
        
        self.status_msg_class = status_class
        self.status_msg_type = status_type
        self.node_topic_dict['status_msg_type'] = self.status_msg_type

        ########################################
        # Setup Connect Data Settings
        if data_msg is not None:
          
            self.data_msg = data_msg
            try:
                data_class = data_msg.__class__
                data_type = data_class.__name__
            except Exception as e:
                self.msg_if.pub_debug("Failed to get data class)", data = str(e))
                raise ValueError(str(type(self).__name__) +
                                                ": Failed to get data class required)")
            
            self.data_msg_class = data_class
            self.data_msg_type = data_type
            self.node_topic_dict['data_msg_type'] = self.data_msg_type



        #########################
        # Update Node IF Dictionaries
        #########################

        ############################################
        # Update Info Dict

        self.info_dict['id'] = id
        self.info_dict['name'] = name
        display_name = name.replace('_',' ').capitalize()
        self.info_dict['display_name'] = display_name
        self.info_dict['description'] = display_name

        self.info_dict['node_name'] = self.node_name
        self.info_dict['node_namespace'] = self.node_namespace
        self.info_dict['namespace'] = self.namespace



        self.info_dict['type'] = '' # Filleed By Caller
        self.info_dict['sub_type'] = id
        self.info_dict['group'] = self.node_name
        self.info_dict['sub_group'] = '' # Filleed By Caller


        self.info_dict['source_description'] = name
        self.info_dict['source_ref_description'] = name
        self.info_dict['end_description'] = name
        self.info_dict['end_ref_description'] = name

        self.info_dict['device_name'] = self.system_config_dict.get('NEPI_DEVICE_ID', 'NEPI Device')
        self.info_dict['serial_num'] = self.system_config_dict.get('NEPI_DEVICE_SN', '999999')
        self.info_dict['hw_version'] = self.system_config_dict.get('NEPI_DEVICE_MD', 'NEPI Device ')
        self.info_dict['sw_version'] = self.system_config_dict.get('NEPI_VERSION', '0x0x0')


        try:
            self.info_dict.update(info_dict)
        except:
            pass



        #######################################
        # Update Caps Dict
        try:
            self.caps_dict.update(caps_dict)
        except:
            pass

        # Update Class variables if they exist
        for name in self.caps_dict.keys():
            value = self.caps_dict[name]
            self.set_value(name, value, pub_status = False, set_param = False, update_dict = False)



        ################################
        # Update Config Dict
        self.config_control_dict['name'] = 'config'
        self.config_control_dict['display_name'] = 'Config'
        self.config_control_dict['namespace'] = self.namespace
        if config_control_dict is not None:
            controls_if = config_control_dict.get('controls_if', None)
            controls_dict = config_control_dict.get('controls_dict', None)
            data_if = config_control_dict.get('data_if', None)
            data_dict = config_control_dict.get('data_dict', None)
            config_dict = config_control_dict.get('config_dict', None)
            self.update_config_control_dict(controls_if,controls_dict,data_if,data_dict,config_dict)

        ###################
        # Update Node Controls Dict
        if node_controls_dict is not None:
            for name in node_controls_dict.keys():
                try:
                    self.set_node_control_dict(node_controls_dict[name])
                except:
                    pass

        ##################
        # Update Info Dict
        try:
            self.stats_dict.update(stats_dict)
        except:
            pass

        # Update Class variables if they exist
        for name in self.stats_dict.keys():
            value = self.stats_dict[name]
            self.set_value(name, value, pub_status = False, set_param = False, update_dict = False)

        ###################
        # Update RUI Dict
        try:
            self.rui_dict.update(rui_dict)
        except:
            pass

        # Update Class variables if they exist
        for name in self.rui_dict.keys():
            value = self.rui_dict[name]
            self.set_value(name, value, pub_status = False, set_param = False, update_dict = False)

        ###################
        # Update Callback Dict
        try:
            self.callback_dict.update(callback_dict)
        except:
            pass

        ##############################   
        ## Node Topic Setup
        ##############################

        self.node_topic_dict = nepi_dict.get_blank_topic_dict()
        self.node_topic_dict['name'] = self.info_dict['name']
        self.node_topic_dict['display_name'] = self.info_dict['display_name']
        self.node_topic_dict['type'] = self.info_dict['type']
        self.node_topic_dict['sub_type'] = self.info_dict['sub_type']
        self.node_topic_dict['group'] = self.info_dict['group']
        self.node_topic_dict['sub_group'] = self.info_dict['sub_group']
        self.node_topic_dict['status_msg_type'] = self.status_msg_type
        self.node_topic_dict['data_msg_type'] = self.data_msg_type if data_msg is not None else ''
        self.node_topic_dict['topic'] = self.namespace
        self.node_topic_dict['topic_name'] = self.info_dict['display_name']

        ##############################   
        ## Node Class Setup
        ##############################

        # Configs Config Dict ####################
        self.node_cfgs_dict =  {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': False, # start() runs the init callback
            'namespace': self.node_namespace
        }

        # Params Config Dict ####################
        # Persist the selected topic under the connect namespace so the
        # selection survives node restarts (via the config manager). Passing a
        # params_dict is what enables config management on NodeClassIF.
        self.node_params_dict = {
                self.node_if_prefix + 'display_name': {
                    'name': 'display_name',
                    'namespace': self.namespace,
                    'factory_val': self.display_name
                }
        }



        self.node_srvs_dict = {
            self.node_if_prefix + 'node_if_status_query': {
                'namespace': self.namespace,
                'topic': 'node_if_status_query',
                'srv': self.if_query_class,
                'req': self.if_request_class(),
                'resp': self.if_response_class(),
                'callback': self._nodeStatusQueryCb
            }
        }



        # Publishers Config Dict ####################
        self.node_pubs_dict = {
            self.node_if_prefix + 'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': self.status_msg_class,
                'qsize': 1,
                'latch': True
            }
        }





        # Subscribers Config Dict ####################
        self.node_subs_dict = {
            self.node_if_prefix + 'set_display_name': {
                'msg': String,
                'namespace': self.namespace,
                'topic': 'set_display_name',
                'qsize': 5,
                'callback': self._setDisplayNameCb
            },
            self.node_if_prefix + 'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self._systemStatusCb
            },
        }




        ###########
        # NODE IF SETUP

        node_if = if_dict.get('node_if', None)


        if node_if is None:
            node_if = NodeClassIF(
                            configs_dict = self.node_cfgs_dict,
                            params_dict = self.node_params_dict,
                            services_dict = self.node_srvs_dict,
                            pubs_dict = self.node_pubs_dict,
                            subs_dict = self.node_subs_dict,
                            msg_if = self.msg_if
            )
            node_if.wait_for_ready()
            self.node_if_shared = False
        else:
            self.node_if_shared = True
            try:
                node_if = node_if
                self.node_cfgs_dict['namespace'] = self.namespace
                node_if.add_configs(self.node_cfgs_dict)
                node_if.add_params(self.node_params_dict) 
                node_if.register_services(self.node_srvs_dict)
                node_if.register_pubs(self.node_pubs_dict)
                node_if.register_subs(self.node_subs_dict)
                # Register the persisted selection param on the shared node_if too.

                # A short settle, as after every interface's registrations (0.1 s, was 1 s).
                # Master registration completes inside the register calls; only peer
                # connections finish afterwards. No construction path publishes anything
                # but latched, 1 Hz status, so nothing relies on a longer wait, and at 1 s
                # per interface drivers_mgr's ~19 settings IFs took ~45 s to start.
                nepi_sdk.sleep(0.1)
            except Exception as e:
                # Warn, not debug: returning here leaves if_ready False, so the
                # caller's wait_for_ready() (infinite timeout) blocks forever and
                # that IF never publishes. At debug level that was invisible.
                self.msg_if.pub_warn(str(self.class_name) + " " + self.namespace +
                                     ": Failed to register on shared node_if, IF will not become ready: " +
                                     type(e).__name__ + ": " + str(e))
                return

        self.node_if = node_if
        self.if_dict['node_if'] = self.node_if
        self.node_topics_dict['node']['topic'] = self.node_if.get_namespace()
        self.node_topics_dict['save_config']['topic'] = self.node_if.get_namespace()



        #######################
        # Capture any other if entries missed
        for key in if_dict.keys():
            if if_dict[key] is not None:
                self.if_dict[key] = if_dict[key]
                setattr(self, key, if_dict[key])

        ##############################
        # Complete Initialization
        # The updater and status timers and the init callback run from start(),
        # which the most-derived Child Class calls at the end of its __init__.

        self.if_ready = True
        self.msg_if.pub_debug(str(self.class_name) + " Initialization Complete")
        ###############################
    
    #######################
    # Class Public Setup Methods
    #######################

    def start(self):
        """Run the init callback and start the NodeIF updater and status timers.

        Call once, at the end of the most-derived class's __init__, after it has
        set its own state and params. NodeIF.__init__ does not start these, so a
        Child Class that never calls start() never initializes from its params
        and never publishes status. Calls after the first do nothing.
        """
        if self.if_started == True:
            return
        self.if_started = True
        self._initCb(do_updates = True)
        nepi_sdk.start_timer_process(1.0, self._nodeIFUpdater, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._nodeIFPublishStatus)



    ############################
    ## NODE Class Config Methods

    def add_params(self,params_dict):
            try:
                clean_dict = self._cleanNodeClassDict(params_dict)
                # NodeParamsIF.add_params skips any entry without a 'name' (the param's ROS
                # name under its namespace). Default it to the key without the node_if
                # prefix, the name every class that sets one uses.
                for param_key, param_dict in clean_dict.items():
                    if 'name' not in param_dict:
                        clean_dict[param_key] = dict(param_dict, name = param_key.replace(self.node_if_prefix, ''))
                self.node_if.add_params(clean_dict)
                self.node_params_dict.update(clean_dict)
            except Exception as e:
                self.msg_if.pub_debug("Failed to register Params", data = [params_dict,str(e)])


    def add_services(self,srvs_dict):
            try:
                clean_dict = self._cleanNodeClassDict(srvs_dict)
                self.node_if.register_services(clean_dict) 
                self.node_srvs_dict.update(clean_dict)
            except Exception as e:
                self.msg_if.pub_debug("Failed to register Services", data = [srvs_dict,str(e)])

    def add_pubs(self,pubs_dict):
            try:
                clean_dict = self._cleanNodeClassDict(pubs_dict)
                self.node_if.register_pubs(clean_dict) 
                self.node_pubs_dict.update(clean_dict)
            except Exception as e:
                self.msg_if.pub_debug("Failed to register Pubs", data = [pubs_dict,str(e)])

    def add_subs(self,subs_dict):
            try:
                clean_dict = self._cleanNodeClassDict(subs_dict)
                self.node_if.register_subs(clean_dict) 
                self.node_subs_dict.update(clean_dict)
            except Exception as e:
                self.msg_if.pub_debug("Failed to register Subs", data = [subs_dict,str(e)])



    def init(self, do_updates = False):
      if self.node_if is not None:
        self.init_values()
        try:
            self.initCb(do_updates)
        except:
            try:
                self.initCb()
            except:
                pass
      if do_updates == True:
        pass
      self.publish_status()

    def reset(self, do_updates = True):
        """Reset parameters to their last-saved (user) values and reinitialize.

        Calls node_if.reset_params() to reload the user configuration tier, then
        reinitializes from the param server.
        """
        if self.node_if is not None:
            param_keys = list(self.node_params_dict.keys())
            self.node_if.reset_params(param_keys)
            nepi_sdk.sleep(1)
            try:
                self.resetCb(do_updates)
            except:
                try:
                    self.resetCb()
                except:
                    pass

        if do_updates == True:
            pass

        self.init(do_updates = True)

    def factory_reset(self, do_updates = True):
        """Reset parameters to factory defaults and reinitialize.

        Calls node_if.factory_reset_params() to restore factory values, then
        reinitializes from the param server.
        """
        if self.node_if is not None:
            param_keys = list(self.node_params_dict.keys())
            self.node_if.factory_reset_params(param_keys)
            nepi_sdk.sleep(1)
            try:
                self.factoryResetCb(do_updates)
            except:
                try:
                    self.factoryResetCb()
                except:
                    pass

        if do_updates == True:
            pass
        
        self.init(do_updates = True)


    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this depth map interface."""          
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.node_if is not None:
                    if self.node_pubs_dict is not None:
                        for pub_name in self.node_pubs_dict.keys():
                            self.node_if.unregister_pub(pub_name)

    def register_pubs(self):
        """Re-register all ROS publishers managed by this image interface."""
        if self.node_if is not None:
            # Its own PUBS_DICT, matching unregister_pubs above and DepthMapIF:
            # on a shared node_if a bare call would also re-advertise publishers
            # a sibling IF had deliberately taken down.
            self.node_if.register_pubs(self.node_pubs_dict)

    def unregister(self):
        """Shut down this settings interface and release its ROS resources."""
        self.ready = False
        success = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                # These were SUBS_DICT/PUBS_DICT, which this class never
                # defines -- it builds controls_node_subs_dict and
                # controls_node_pubs_dict. The shared-node_if branch therefore
                # raised AttributeError instead of unregistering anything.
                if self.node_subs_dict is not None:
                        for sub_name in self.node_subs_dict.keys():
                            self.node_if.unregister_sub(sub_name)
                self.node_subs_dict = None

                if self.node_pubs_dict is not None:
                        for pub_name in self.node_pubs_dict.keys():
                            self.node_if.unregister_pub(pub_name)
                self.node_pubs_dict = None

        time.sleep(1)
        try:
            self.node_if = None
            self.ready = False
            success = True
        except Exception as e:
            self.msg_if.pub_debug("Failed to unregister:  " + str(e) )
        return success


    def unsubscribe(self):
        """Shut down this interface, unregister all ROS resources, and clear state."""
        self.if_ready = False
        if self.node_if is not None:
            self.node_if.unregister_class()
        time.sleep(1)



    ##############################
    # NodeIF Config Support Methods

    def get_config_control_dict(self, name):
        return self.config_control_dict

    def update_config_control_dict(self, controls_if = None, controls_dict = dict(), data_if = None, data_dict = dict(), config_dict = dict()):
        if controls_if is not None:
            self.config_control_dict['controls_if'] = controls_if

        if controls_dict is not None:
            try:
                controls_dict = nepi_controls.create_controls_dict(controls_dict, name = 'config_controls')
                self.config_control_dict['controls_dict'].update(controls_dict)         
            except:
                pass

        if data_if is not None:
            self.config_control_dict['data_if'] = data_if

        if data_dict is not None:
            try:
                data_dict = nepi_data.create_data_dict(data_dict, name = 'config_data')
                self.config_control_dict['data_dict'].update(data_dict)         
            except:
                pass

        if config_dict is not None:
            try:
                self.config_control_dict['config_dict'].update(config_dict)       
                # Update Class variables if they exist
                for name in self.config_control_dict['config_dict'].keys():
                    value = self.config_control_dict['config_dict'][name]
                    self.set_value(name, value, pub_status = False, set_param = False, update_dict = False)
            except:
                pass


    def get_node_controls_names(self):
        return list(self.node_controls_dict.keys())

    def get_node_control_dict(self, name):
        node_control_dict = None
        if name in self.node_controls_dict.keys():
            node_control_dict = self.node_controls_dict[name]
        return node_control_dict

    def set_node_control_dict(self, control_group_dict):

        name = control_group_dict.get('name',None)
        if name is None:
            name = ''
        if name.strip() == '':
            name = 'control_group'  
        name = str(name)  
     
        display_name = control_group_dict.get('display_name',str(name).replace('_',' ').capitalize())
        namespace = control_group_dict.get('namespace',self.node_namespace + '/' + nepi_utils.get_clean_name(name))

        controls_if = control_group_dict.get('controls_if',None)
        init_controls_dict = control_group_dict.get('controls_dict',dict())
        try:
            controls_dict = nepi_controls.create_controls_dict(init_controls_dict, name = name, display_name = display_name)
        except:
            controls_dict = dict()
            

        data_if = control_group_dict.get('data_if',None)
        init_data_dict = control_group_dict.get('data_dict',dict())
        try:
            data_dict = nepi_data.create_data_dict(init_data_dict, name = name, display_name = display_name)
        except:
            data_dict = dict()

        config_dict = control_group_dict.get('config_dict',dict())
        if config_dict is None:
            config_dict = dict()
                
        self.node_controls_dict[name] = dict(
            name = name,
            display_name = display_name,
            namespace = namespace,
            controls_if = controls_if,
            controls_dict = controls_dict,
            data_if = data_if,
            data_dict = data_dict,
            config_dict = config_dict
        )



    ##############################
    # IF and Topic Dict Support Methods

    def set_if(self, name, if_instance = None):
        self.if_dict[name] = if_instance

    ############
    # Node Topic Methods Classes
    
    def get_topic_dict(self):
        return self.node_topic_dict

    def get_topics_dict(self):
        return self.node_topics_dict

    ############
    # Connect Topic Support Methods

    def get_connect_topics_dict(self):
        return self.connect_topics_dict
    


    def add_connect_topic(self, name, topic_dict):
        if name not in self.connect_topics_dict.keys():
            add_topic_dict = copy.deepcopy(BLANK_TOPIC_DICT)
            for key in topic_dict.keys():
                if key in add_topic_dict.keys():
                    add_topic_dict[key] = topic_dict[key]
            self.connect_topics_dict[name] = add_topic_dict

    def del_connect_topic(self, name):
        topic_names = list(self.connect_topics_dict.keys())
        if name in topic_names:
            del self.connect_topics_dict[name]

    def update_connect_topic(self, name, topic = None, topic_name = None, connecting = None, connected = None):
        if name in self.connect_topics_dict.keys():

            update_dict = dict()
            if topic is not None:
                update_dict['topic'] = topic

            if topic_name is not None:
                update_dict['topic_name'] = topic_name

            if connecting is not None:
                connecting = connecting == True
                update_dict['connecting'] = connecting
                if connecting == True:
                    update_dict['connected'] = False

            if connected is not None:
                connected = connected == True
                update_dict['connected'] = connected
                if connected == True:
                    update_dict['connecting'] = False

            self.connect_topics_dict[name].update(update_dict)




    def update_connect_topic(self, name, update_topic_dict):
         if name in self.connect_topics_dict.keys():
             for key in update_topic_dict.keys():
                 if key in self.connect_topics_dict[name].keys():
                     value = update_topic_dict[key]
                     self.connect_topics_dict[name][key] = value
                     if key == 'topic' and value == '':
                         self.connect_topics_dict[name]['connecting'] = False
                         self.connect_topics_dict[name]['connected'] = False
                     if key == 'connecting' and value == True:
                         self.connect_topics_dict[name]['connected'] = False
                     if key == 'connected' and value == True:
                         self.connect_topics_dict[name]['connecting'] = False

    # def reset_topics(self):
    #     self.node_topics_lock.acquire()
    #     system_dict = copy.deepcopy(self.connect_topics_dict)
    #     for key in system_dict.keys():
    #         topic_locked = system_dict[key].get(topic_locked, False)
    #         if topic_locked == False:
    #             system_dict[key]['topic'] = ''
    #             system_dict[key]['topic_name'] = ''
    #             system_dict[key]['connecting'] = False
    #             system_dict[key]['connected'] = False
    #     self.connect_topics_dict = system_dict
    #     self.node_topics_lock.release()



    ############################
    ## Node Callback Support Methods
    def set_callback(self,name, callback, namespace = None):
        if name in self.callback_dict.keys():
            self.callback_dict[name] = callback

    def clear_callback(self,name, namespace = None):
        if name in self.callback_dict.keys():
            self.callback_dict[name] = None


    ############################
    ## Class Variable and Dict support 

    def get_param(self,name, fallback = None):
        param_value = fallback
        if self.node_if is not None:
            param_value = self.node_if.get_param(self.node_if_prefix +  name, fallback)
        return param_value



    def set_param(self,name, value):
            success = False
            if self.node_if is not None:
                success = self.node_if.set_param(self.node_if_prefix +  name, value)
            return success



    def set_value(self, name, value, set_local = True, set_param = True, update_dict = True, pub_status = True):
            ##########################
            # Try to set class variable
            success = True
            if set_local == True:
                if hasattr(self.__class__, name) == True:
                    try:
                        setattr(self, name, value)
                    except Exception as e:
                        success = False
                        self.msg_if.pub_debug("Failed set class variable value", data = [name,value,str(e)], throttle_s = 5)

            ##########################
            # Try to set Dict Value
            if update_dict == True:
                # Update dict value if in dict
                if name in self.info_dict.keys():
                    self.info_dict[name] = value
                if name in self.caps_dict.keys():
                    self.caps_dict[name] = value             
                if name in self.stats_dict.keys():
                    self.stats_dict[name] = value

                # Update Config Dict Values
                if self.config_control_dict['controls_if'] is not None:
                    if self.config_control_dict['controls_if'].has_control(name) == True:
                        self.config_control_dict['controls_if'].set_control_value(name,value)

                if name in self.config_control_dict['controls_dict'].keys():
                    self.config_control_dict['controls_dict'] = \
                        nepi_controls.set_value(self.config_control_dict['controls_dict'],name, value)

                if self.config_control_dict['data_if'] is not None:
                    if self.config_control_dict['data_if'].has_datum(name) == True:
                        self.config_control_dict['data_if'].set_datum_value(name,value)

                if name in self.config_control_dict['data_dict'].keys():
                    self.config_control_dict['data_dict'] = \
                        nepi_data.set_value(self.config_control_dict['data_dict'],name, value)

                if name in self.config_control_dict['config_dict'].keys():
                    self.config_control_dict['config_dict'][name] = value
                

            ##########################
            # publish status if name in status and pub_status == True
            if pub_status == True:
                if hasattr(self.status_msg, name):
                    self.publish_status()

            ##########################
            # try and set param if set_param == True
            if set_param == True:
                self.set_param(name,value)
            return success


    def set_value_local(self, name, value):
        self.set_value(name, value, set_local = True, set_param = False, update_dict = False, pub_status = False,)

    def set_value_dict(self, name, value):
        self.set_value(name, value, set_local = False, set_param = False, update_dict = True, pub_status = False,)

    def get_value(self,name, fallback = None):
            # Try to set class variable
            success = True
            value = fallback 
            try:
                value = getattr(self, name)
            except:
                success = False
            if success == False:
                if name in self.info_dict.keys():
                    value = self.info_dict[name]
                elif name in self.caps_dict.keys():
                    value = self.caps_dict[name]
                elif name in self.config_control_dict['config_dict'].keys():
                    value = self.config_control_dict['config_dict'][name]              
                elif name in self.stats_dict.keys():
                    value = self.stats_dict[name]  
                elif name in self.rui_dict.keys():
                    value = self.rui_dict[name]  
            return value


    def init_value(self,name, fallback = None, update_dict = True):
            success = True
            init_value = self.node_if.get_param(self.node_if_prefix +  name, fallback)
            if init_value is not None:
                success = self.set_value(name, init_value, pub_status = False, set_param = False, update_dict = update_dict)
            return init_value


    def init_values(self, update_dict = True):
        for key in self.node_params_dict.keys():
            name = key.replace(self.node_if_prefix, '')
            self.init_value(name, update_dict = update_dict)



    ##############################
    # Status Support Methods

    def get_if_status_msg(self):
        # The fields below belong to BaseIFStatus, which NodeIFStatus carries as if_status
        base_status_msg = self.if_status_msg.if_status
        #####################################
        self.info_dict['timestamp'] = nepi_utils.get_time()
        base_status_msg.info_data = nepi_dict.convert_values_dict2msg(self.info_dict)

        base_status_msg.caps_data = nepi_dict.convert_values_dict2msg(self.caps_dict)

        base_status_msg.stats_data = nepi_dict.convert_values_dict2msg(self.stats_dict)

        rui_dict = copy.deepcopy(self.rui_dict)
        for key in rui_dict.keys():
            if str(key).replace('show_','has_') in self.caps_dict.keys():
                rui_dict[key] = rui_dict[key] and self.caps_dict[str(key).replace('show_','has_')]
        base_status_msg.rui_data = nepi_dict.convert_values_dict2msg(rui_dict)

        #####################################

        base_status_msg.config_control_group = nepi_dict.convert_control_group_dict2msg(self.config_control_dict)

        #####################################
        if self.node_controls_dict is not None:
            for key_name in self.node_controls_dict.keys():
                

                try:
                    controls_topic = self.node_controls_dict[key_name]['controls_if'].get_namespace()
                except:
                    controls_topic = ''
                if controls_topic is None:
                    controls_topic = ''
                self.node_controls_dict[key_name]['controls_topic'] = controls_topic

                try:
                    data_topic = self.node_controls_dict[key_name]['data_if'].get_namespace()
                except:
                    data_topic = ''
                if data_topic is None:
                    data_topic = ''
                self.node_controls_dict[key_name]['data_topic'] = data_topic

            try:
                base_status_msg.node_control_groups = nepi_dict.convert_control_groups_dict2msg(self.node_controls_dict)
            except Exception as e:
                self.msg_if.pub_debug("Failed convert conrol_group_dict to msg", data = str(e), throttle_s = 5)
        

        #####################################
        base_status_msg.node_topic = nepi_dict.convert_topic_dict2msg(self.node_topic_dict)

        if self.node_topics_lock.locked() == False:
            base_status_msg.node_topics = nepi_dict.convert_topics_dict2msg(self.node_topics_dict)

        base_status_msg.node_connect_topics = nepi_dict.convert_topics_dict2msg(self.connect_topics_dict)

        return self.if_status_msg

    def get_if_status_dict(self):
        msg = self.get_if_status_msg()
        msg_dict = nepi_sdk.convert_msg2dict(msg)
        return msg_dict
    
    #######################
    def update_status_msg(self, status_msg):
        return status_msg
    
    def get_status_msg(self):
        if_status_msg = self.get_if_status_msg()
        if hasattr(self.status_msg, "if_status") == True:
            self.status_msg.if_status = if_status_msg
        if self.status_msg_class != self.if_status_msg_class:

            # Auto Update Value from Class Variables and Dicts
            for name in list(nepi_sdk.convert_msg2dict(self.status_msg).keys()):
                if name != 'if_status':

                    # Try to get from Class variable or IF Dicts
                    value = self.get_value(name)

                    # Try to get from System Topics Dict, by status field name
                    if value is None:
                        for topic_key in SYSTEM_TOPICS_STATUS_FIELDS.keys():
                            if SYSTEM_TOPICS_STATUS_FIELDS[topic_key] == name and topic_key in self.node_topics_dict.keys():
                                value = self.node_topics_dict[topic_key]['topic']

                    # Try and Update status message value if found
                    if value is not None:
                        try:
                            setattr(self.status_msg, name, value)
                        except Exception as e:
                            self.msg_if.pub_debug("Failed set Status value from Info Dict", data = [name,value,str(e)], throttle_s = 5)

        # Get any updates if provided by Child Class
        self.status_msg = self.update_status_msg(self.status_msg)
        return self.status_msg

    def get_status_dict(self):
        """Return the current status message converted to a plain dictionary.

        Returns:
            dict: Status fields as a dictionary, or None if no status message exists.
        """
        status_dict = self.get_status_msg()
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    

    ############################

    def publish_msg(self,name, msg):
        if self.node_if_prefix not in name:
            name = self.node_if_prefix + name
        if self.node_if is not None:
            self.node_if.publish_pub(name, msg)
        

    def publish_status(self):
        pub_status_msg = self.get_status_msg()
        ###########
        if self.node_if is not None:
            if self.status_has_published == False:
                #self.msg_if.pub_debug("Publishing Status: " + str(pub_status_msg))
                self.status_has_published = True
            self.node_if.publish_pub(self.node_if_prefix + 'status_pub', pub_status_msg) 
            #self.node_if.save_config()





    #######################
    # Node Public Methods
    #######################

    

    def get_ready_state(self):
        """Return the if_ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.if_ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is if_ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became if_ready, False if the timeout was reached.
        """
        success = False

        self.msg_if.pub_debug("Waiting for connection")
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.get_ready_state() == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        ready = self.get_ready_state()
        if ready == False:
            self.msg_if.pub_debug("Failed to Connect")
        else:
            self.msg_if.pub_debug("Connected")
        return ready

    def get_namespace(self):
        """Return the fully-resolved ROS namespace for the connected PTX device.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.namespace





    def set_display_name(self, name):
        self.display_name = str(name)
        self.info_dict['display_name'] = self.display_name
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'display_name',self.display_name)



    ####################################################
    # Class Public System IF Methods
    ###################################################

    ############################
    ## Msg IF Support

    def pub_info(self, *args, **kwargs):
        self.msg_if.log_info(*args, **kwargs)
    
    def pub_warn(self,  *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
        self.msg_if.log_warn(*args, **kwargs)
    
    def pub_error(self,  *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
        self.msg_if.log_error(*args, **kwargs)
    
    def pub_fatal(self, *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
        self.msg_if.log_fatal(*args, **kwargs)
      
    def pub_debug(self,  *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
        self.msg_if.log_debug(*args, **kwargs)

    ##############################
    # Settings IF Support


    ##############################
    # Save Data IF Support


    ##############################
    # Transform IF Support


    ##############################
    # NavPose IF Support

    def get_navpose_msg(self):
        if self.navpose_if is not None:
            navpose_msg = self.navpose_if.get_navpose_msg()
        else:
            navpose_msg = NavPose()
        return navpose_msg

    def get_navpose_dict(self):
        if self.navpose_if is not None:
            navpose_dict = self.navpose_if.get_navpose_dict()
        else:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return navpose_dict

    def get_blank_navpose_dict(self):
        """Return a deep copy of the blank nav pose dictionary template.

        Returns:
            dict: A blank nav pose dictionary with all fields at default values.
        """
        blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return blank_navpose_dict
    ##############################
    # States IF Support


    ##############################
    # Triggers IF Support


    ##############################
    # Save Config IF Support

    def save_config(self):
        """Request a save of the current config through the node_if config manager.
        """
        if self.node_if is not None:
            self.node_if.save_config()

    def reset_config(self):
        """Restore the last saved config through the node_if config manager.
        """
        if self.node_if is not None:
            self.node_if.reset_config()

    def factory_reset_config(self):
        """Restore the factory default config through the node_if config manager.
        """
        if self.node_if is not None:
            self.node_if.factory_reset_config()



    #######################
    # Class Private Methods
    #######################

    ###############################
    def _initCb(self,do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.reset(do_updates = do_updates)

    def _factoryResetCb(self,do_updates = True):
        self.factory_reset(do_updates = do_updates)
    ###############################


    def _cleanNodeClassDict(self, input_dict):
        clean_dict = dict()
        for input_key in input_dict.keys():
            output_key = input_key
            if self.node_if_prefix not in output_key:
                output_key = self.node_if_prefix + output_key
            clean_dict[output_key] = input_dict[input_key]
        return clean_dict



    def _setDisplayNameCb(self,msg):
        self.set_display_name(msg.data)

    def _systemStatusCb(self,msg):
            self.active_nodes = msg.active_nodes
            self.active_topics = msg.active_topics
            self.active_topic_types = msg.active_topic_types
            self.active_services = msg.active_services


    def _nodeIFUpdater(self,timer):
        needs_publish = False
        ##############


        #######################
        # Sync System IF Chanages
        if_keys = list(SYSTEM_IF_DICT.keys())
        for key in if_keys:
            dict_if = self.if_dict[key]
            class_if = None
            try:
               class_if = getattr(self, key)
            except:
               class_if = None                
            if class_if is not None and dict_if is None: 
                self.if_dict[key] = class_if
            if dict_if is not None and class_if is None:
                setattr(self, key, dict_if)

        #########################
        # Update Config Data
        
        try:
            config_controls_topic = self.config_controls_if.get_namespace()
        except:
            config_controls_topic = ''
        if config_controls_topic is None:
            config_controls_topic = ''
        self.config_controls_topic = config_controls_topic

        try:
            config_data_topic = self.config_data_if.get_namespace()
        except:
            config_data_topic = ''
        if config_data_topic is None:
            config_data_topic = ''
        self.config_data_topic = config_data_topic

        #######################
        # Update Topics Dict
        purge_topics = []
        if self.node_topics_lock.locked() == False:
            for key_name in self.if_dict.keys():
                topic_key = key_name.replace('_if','')
                # node_if has no get_topic_dict(); its 'node' entry is set at startup
                # and would be purged below
                if key_name == 'node_if':
                    continue
                # No System Topics Dict entry for this IF (or purged): skip it, so the
                # updater always reaches its re-arm below
                if topic_key not in self.node_topics_dict.keys():
                    continue
                if self.if_dict[key_name] is None:
                    if topic_key in self.node_topics_dict.keys():
                        purge_topics.append(topic_key)
                else:
                    
                    status_msg_type = self.node_topics_dict[topic_key].get('status_msg_type', '')
                    if self.status_msg_type == status_msg_type:
                        self.node_topics_dict[topic_key] = copy.deepcopy(self.node_topic_dict)
                        self.node_topics_dict[topic_key]['topic'] = self.node_topic_dict['topic']
                        self.node_topics_dict[topic_key]['connecting'] = True
                        self.node_topics_dict[topic_key]['connected'] = True
                    else:
                        self.node_topics_dict[topic_key]['topic_locked'] = False
                        try:
                            self.node_topics_dict[topic_key] = copy.deepcopy(self.if_dict[key_name].get_topic_dict())
                            self.node_topics_dict[topic_key]['connecting'] = True
                            self.node_topics_dict[topic_key]['connected'] = True
                        except:
                            purge_topics.append(topic_key)

        for topic_key in purge_topics:
            del self.node_topics_dict[topic_key]



        #######################
        # Update Connect Topics Dict
        for key_name in self.connect_if_dict.keys():
            if self.connect_if_dict[key_name] is not None:
                topic_key = key_name.replace('_if','')
                topic = self.connect_topics_dict.get(topic_key,'')
                if topic == '':
                    try:
                        self.connect_topics_dict[topic_key]['topic'] = self.connect_if_dict[key_name].get_namespace() 
                        self.connect_topics_dict[topic_key]['connecting'] = True
                        self.connect_topics_dict[topic_key]['connected'] = True
                    except:
                        pass


        #######################
        # Update Data Products Dict
        if self.save_data_if is not None:
            data_products = self.save_data_if.get_data_products()
            self.set_value_local('data_products', data_products)

        #######################
        # Update IF Local Variables Dict
        for if_name in self.if_dict.keys():
            value = self.if_dict[if_name]
            self.set_value_local(if_name, value)

        nepi_sdk.start_timer_process(1.0, self._nodeIFUpdater, oneshot = True)


    def _nodeStatusQueryCb(self, _):
        if_status_msg = self.get_if_status_msg()
        return if_status_msg

    def _nodeIFPublishStatus(self,timer):
        self.publish_status()
    




#########################################
# Node Setup Classes
#########################################


##################################################
### Node Config Class


BLANK_CONFIG_DICT = {
        'namespace': None,
        'alt_namespace': None,
        'init_callback': None,
        'reset_callback': None,
        'factory_reset_callback': None,
        'init_configs': True,
        'manage_configs': True,
        'clear_params': True
}



class NodeConfigsIF:

    MAX_SAVE_RATE = 1

    msg_if = None
    ready = False
    configs_dict = None
    namespace = '~'
    alt_namespace = None


    initCb = []
    sysResetCb = []
    resetCb = []
    factoryResetCb = []

    save_triggered = False
    save_all_triggered = False
    last_save = nepi_sdk.get_time()
    ### IF Initialization
    def __init__(self, 
                configs_dict,
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        
        ##############################  
        # Create Msg Class
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        
        self.msg_if.pub_debug("Starting Node Configs IF Initialization Processes" )

        ##############################    




        configs_dict = self.add_configs(configs_dict)
            

        if 'namespace' not in configs_dict.keys():
            configs_dict['namespace'] = None
        namespace = configs_dict['namespace']
        if namespace is None:
            namespace = self.node_namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        if 'alt_namespace' in configs_dict.keys():
            self.alt_namespace = configs_dict['alt_namespace']
        #self.msg_if.pub_debug("Using Config namespace: " + str(self.namespace) )
        #self.msg_if.pub_debug("Using Base namespace: " + str(self.base_namespace) )

        clear_params = True
        if 'clear_params' in configs_dict.keys():
            clear_params = configs_dict['clear_params']
        if clear_params == True:
            self.msg_if.pub_debug("Clearing params for namespace: " + str(self.namespace) )
            nepi_sdk.delete_params(self.namespace)
        

        manage_configs = True
        if 'manage_configs' in configs_dict.keys():
            manage_configs = configs_dict['manage_configs']
        if manage_configs == True:
            ###############
            # Create Config Publishers
            ns = nepi_sdk.create_namespace(self.base_namespace,'save_params')
            self.save_params_pub = nepi_sdk.create_publisher(ns, String, queue_size=1)
            ns = nepi_sdk.create_namespace(self.base_namespace,'save_params_all')
            self.save_params_all_pub = nepi_sdk.create_publisher(ns, String, queue_size=1)
            ns = nepi_sdk.create_namespace(self.base_namespace,'reset_params')
            self.reset_params_pub = nepi_sdk.create_publisher(ns, UpdateString, queue_size=1)
            ns = nepi_sdk.create_namespace(self.base_namespace,'delete_configs')
            self.delete_configs_pub = nepi_sdk.create_publisher(ns, UpdateString, queue_size=1)


            nepi_sdk.sleep(1)

            ###############
            # Create Config Subscribers
            # Subscribe to save config for node namespace
            nepi_sdk.create_subscriber(self.namespace + '/save_config', Empty, self._saveCb, queue_size=5)

            nepi_sdk.create_subscriber(self.namespace + '/init_config', Empty, self._initCb, queue_size=5)
            nepi_sdk.create_subscriber(self.namespace + '/reset_config', Empty, self._resetCb, queue_size=5)
            nepi_sdk.create_subscriber(self.namespace + '/factory_reset_config', Empty, self._factoryResetCb, queue_size=5)


            # Global Topic Subscribers
            nepi_sdk.create_subscriber('save_config', Empty, self._saveCb, queue_size=5)
            nepi_sdk.create_subscriber('init_config', Empty, self._initCb, queue_size=5)
            nepi_sdk.create_subscriber('reset_config', Empty, self._resetCb, queue_size=5)
            nepi_sdk.create_subscriber('factory_reset_config', Empty, self._factoryResetCb, queue_size=5)

            if nepi_system.supports_all_config(self.namespace) == True:
                nepi_sdk.create_subscriber(self.namespace + '/save_config_all', Empty, self._saveAllCb)
            nepi_sdk.create_subscriber(self.namespace + '/delete_configs', Empty, self._deleteConfigsCb)


            ################
            self.msg_if.pub_debug("Resetting Params" )
            self.reset_params()
            nepi_sdk.sleep(1)

        # params_dict = nepi_sdk.get_param(self.namespace, dict())
        # self.msg_if.pub_debug("Got Reset Params: " + str(params_dict) )

        if 'init_configs' in configs_dict.keys():
            
            init_configs = configs_dict['init_configs']
            if init_configs == True:
                self.msg_if.pub_debug("Calling parent reset function" )
                self.reset_config()
        nepi_sdk.start_timer_process(self.MAX_SAVE_RATE, self._updaterCb, oneshot = True)
        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("Node Configs IF Initialization Complete" )
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready" )
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out" )
        else:
            self.msg_if.pub_debug("Ready" )
        return self.ready

    def reset_params(self):
        msg = UpdateString()
        msg.name = self.namespace
        if self.alt_namespace is not None:
            msg.name2 = self.alt_namespace
            self.msg_if.pub_debug("Reseting Params: " + str(self.namespace) + " including alt namespace" + str(self.alt_namespace))
        else:
            self.msg_if.pub_debug("Reseting Params: " + str(self.namespace))
        self.reset_params_pub.publish(msg)



    def add_configs(self, configs_dict):
        if configs_dict is None:
            configs_dict = copy.deepcopy(BLANK_CONFIG_DICT)
        else:
            for key in BLANK_CONFIG_DICT.keys():
                if key not in configs_dict.keys():
                    configs_dict[key] = BLANK_CONFIG_DICT[key]
        self.msg_if.pub_debug("Adding Config Dict: " + str(configs_dict) )

        if 'init_callback' in configs_dict.keys():  
            self.initCb.append(configs_dict['init_callback'])

        if 'reset_callback' in configs_dict.keys():
            self.resetCb.append(configs_dict['reset_callback'])

        if 'factory_reset_callback' in configs_dict.keys():
            self.factoryResetCb.append(configs_dict['factory_reset_callback'])

        return configs_dict


    def init_config(self, do_updates = False):
        for i, callback in enumerate(self.initCb):
            self.msg_if.pub_debug("Calling init callback: " + str(callback) )
            if (callback is not None):
                try:
                    if not nepi_sdk.is_shutdown():
                        callback() # Callback provided by container class to update based on param server, etc.
                        if i == 0 and len(self.initCb) > 0:
                            nepi_sdk.sleep(1)
                except: 
                    pass


    def reset_config(self):
        self.msg_if.pub_debug("Resetting Configs: " + str(self.namespace))
        for i, callback in enumerate(self.resetCb):
            self.msg_if.pub_debug("Calling reset callback: " + str(callback) )
            if (callback is not None):
                try:
                    if not nepi_sdk.is_shutdown():
                        callback() # Callback provided by container class to update based on param server, etc.
                        if i == 0 and len(self.resetCb) > 0:
                            nepi_sdk.sleep(1)
                except: 
                    pass
        



    def factory_reset_config(self):
        self.msg_if.pub_debug("Factory Resetting Configs: " + str(self.namespace))
        for i, callback in enumerate(self.factoryResetCb):
            self.msg_if.pub_debug("Calling factory reset callback: " + str(callback) )
            if (callback is not None):
                try:
                    if not nepi_sdk.is_shutdown():
                        callback() # Callback provided by container class to update based on param server, etc.
                        if i == 0 and len(self.factoryResetCb) > 0:
                            nepi_sdk.sleep(1)
                except: 
                    pass

    def save_config(self):
        self.save_triggered = True


    def save_config_all(self):
        self.msg_if.pub_debug("Saving Config All: " + str(self.namespace))
        self.save_all_triggered = True

    def factory_save_config(self):
        self.msg_if.pub_debug("Factory Saving Configs: " + str(self.namespace))

    def delete_config_all(self):
        self.msg_if.pub_debug("Deleting Config All: " + str(self.namespace))
        msg = UpdateString()
        msg.name = self.namespace
        msg.name2 = self.namespace
        if self.alt_namespace is not None:
            msg.name2 = self.alt_namespace
        self.delete_configs_pub.publish(msg)



    ###############################
    # Class Private Methods
    ###############################

    def _updaterCb(self, timer):
        if self.save_all_triggered == True:
            self.save_params_all_pub.publish(self.namespace)
        elif self.save_triggered == True:
            self.save_params_pub.publish(self.namespace)
        self.save_triggered = False
        self.save_all_triggered = False
        nepi_sdk.start_timer_process(self.MAX_SAVE_RATE, self._updaterCb, oneshot = True)

    def _saveCb(self,msg):
        self.save_config()

    def _saveAllCb(self,msg):
        self.msg_if.pub_debug("Got Save Config All Msg" )
        self.save_config_all()

    def _deleteConfigsCb(self,msg):
        self.msg_if.pub_debug("Got Delete Config All Msg" )
        self.delete_config_all()

    def _initCb(self,msg):
        self.init_config(do_updates = True)

    def _resetCb(self,msg):
        self.msg_if.pub_debug("Got Reset Config Request" )
        self.reset_config() 

    def _factorySaveCb(self,msg):
        self.factory_save_config()

    def _factoryResetCb(self,msg):
        self.msg_if.pub_debug("Got Factory Reset Config Request" )
        self.factory_reset_config()






##################################################
### Node Params Class
'''
EXAMPLE_node_params_dict = {
    'param11_key': {
        'name': 'param1',
        'namespace':  self.node_namespace,
        'factory_val': 100,
        'current_val: 20  # Optional
    },
    'param2_key': {
        'name': 'param2',
        'namespace':  self.node_namespace,
        'factory_val': "Something"
    }
}
'''


class NodeParamsIF:

    DEBUG_PARAM = 'NONE'

    msg_if = None
    ready = False
    params_dict = dict()
    

    initCb = None
    resetCb = None
    factoryResetCb = None

    params_ns_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                params_dict = None,
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.msg_if.pub_debug("Starting Node Params IF Initialization Processes" )
        ##############################   

        ##############################
        # Initialize Params System

        # Guards params_dict. A NodeClassIF is shared by every sub-IF in a node
        # (image IFs, ControlsIF, ...), and IDXDeviceIF builds those sub-IFs from
        # separate data-product threads at the same time. initialize_params()
        # deep-copies params_dict and then assigns the copy back, so an unguarded
        # add_param() from another thread is either lost on the reassign or makes
        # the deepcopy raise "dictionary changed size during iteration".
        self.params_lock = threading.RLock()
        self.params_dict = params_dict
        if self.params_dict is None:
            self.params_dict = dict()
        self.initialize_params()


        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready" )
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out" )
        else:
            self.msg_if.pub_debug("Ready" )
        return self.ready

    def load_params(self, file_path):
        self.nepi_sdk.load_params_from_file(file_path,self.namespace)        

    def getNestedInitVal(self, ns_param_dict, param_key):
        # A param key may contain '/' ('home_position/pan_deg'), and get_params returns
        # the namespace as a NESTED dict, so a slashed key never matches a top-level
        # key. Matching against the top level made every slashed param fall through to
        # its factory_val and then overwrite the correctly reloaded server value, which
        # is why svx and ptx home position could not survive a restart. Walk the
        # segments instead. A flat key still resolves on the first pass.
        val = ns_param_dict
        for key in param_key.split('/'):
            if isinstance(val, dict) and key in val.keys():
                val = val[key]
            else:
                return None
        return val

    def initialize_params(self):
        with self.params_lock:
            return self._initializeParams()

    def _initializeParams(self):

        #self.msg_if.pub_debug("Initializing params: " + str(self.params_dict.keys()) )
        #self.msg_if.pub_debug("Initializing params: " + str(self.params_dict) )
        #get_params = nepi_sdk.get_params(self.node_namespace)
        #self.msg_if.pub_debug("Got Init params for node namespace: " + str(self.node_namespace) + " : " + str(get_params) )
        params_dict = copy.deepcopy(self.params_dict)
        init_val = None
        got_params_dict = dict()
        for param_key in params_dict.keys():
            namespace = params_dict[param_key]['namespace']
            if namespace not in got_params_dict.keys():
                get_params_dict = nepi_sdk.get_params(namespace)
                #self.msg_if.pub_debug("Got Init params dict for namespace: " + str(namespace) + " : " + str(get_params_dict) )
                if get_params_dict is not None:
                    if isinstance(get_params_dict, dict):

                        got_params_dict[namespace] = get_params_dict
            if 'init_val' not in params_dict[param_key].keys():
                param_dict = params_dict[param_key]
                param_name = param_dict.get('name',param_key)
                param_name = nepi_utils.get_clean_name(param_name)
                if param_name is None:
                    param_name = param_key
                if param_name == '':
                    param_name = param_key
                param_dict['name'] = param_name
                # get_params_dict['name'] = param_name
                factory_val = param_dict['factory_val']

                ns_param_dict = got_params_dict[namespace]
                init_val = self.getNestedInitVal(ns_param_dict, param_name)
                #self.msg_if.pub_debug("Got Init value for param name: " + str(param_name) + " : " + str(init_val) )
                if init_val is None:
                    init_val = factory_val
                param_dict['init_val'] = init_val
                params_dict[param_key] = param_dict
                self.set_param(param_key, init_val)
                cur_val = self.get_param(param_key)

                if param_key == self.DEBUG_PARAM:
                    self.msg_if.pub_debug("Initialized param factory,init,value " + str([param_key,factory_val,init_val,cur_val]) )
                    self.msg_if.pub_debug("Initialized params dict " + str([param_key,params_dict[param_key]]) )
        self.params_dict = params_dict
        return True
            
    def reset_params(self, param_keys = None):
        self.msg_if.pub_debug("Resetting params" )
        success = self.initialize_params()
        if param_keys is None:
            param_keys = list(self.params_dict.keys())
        for param_key in param_keys:
            if param_key in self.params_dict.keys():
            
                cur_val = self.get_param(param_key)

                init_val = None
                if param_key == self.DEBUG_PARAM:
                    self.msg_if.pub_debug("Looking for init value " + str([param_key,self.params_dict[param_key]]))
                if 'init_val' in self.params_dict[param_key].keys():
                    init_val = self.params_dict[param_key]['init_val']
                if init_val is None:
                    init_val = self.params_dict[param_key]['factory_val']
                if param_key == self.DEBUG_PARAM:
                    self.msg_if.pub_debug("Resetting param from:to " + str([param_key,cur_val,init_val]) )
                self.set_param(param_key, init_val)

    def factory_reset_params(self, param_keys = None):
        self.msg_if.pub_debug("Factory Resetting params" )
        success = self.initialize_params()
        if param_keys is None:
            param_keys = list(self.params_dict.keys())
        for param_key in param_keys:
            if param_key in self.params_dict.keys():

                cur_val = self.get_param(param_key)
                
                factory_val = self.params_dict[param_key]['factory_val']
                if param_key == self.DEBUG_PARAM:
                    self.msg_if.pub_debug("Factory Resetting param from:to " + str([param_key,cur_val,factory_val]) )
                self.set_param(param_key, factory_val)

    def save_params(self, file_path):
        if not nepi_sdk.is_shutdown():
            self.nepi_sdk.save_params_to_file(file_path,self.namespace)       

    def has_param(self, param_key):
        namespace = self.get_param_namespace(param_key)
        if namespace is not None:
            return nepi_sdk.has_param(namespace)
        return False

    def get_param_keys(self):
        param_keys = list(self.params_dict.keys())
        return param_keys

    def get_param(self, param_key, fallback = None):
        value = fallback
        if param_key in self.params_dict.keys():
            param_dict = self.params_dict[param_key]
            namespace = self.get_param_namespace(param_key)
            if namespace is not None:
                if namespace in self.params_ns_dict.keys():
                    value = self.params_ns_dict[namespace]

                fallback = None
                if 'init_val' in param_dict.keys():
                    fallback = param_dict['init_val']
                
                if fallback is None:  
                    fallback = param_dict['factory_val']

                if value is None:
                    self.msg_if.pub_debug("Got param using fallback_value " + str([param_key,fallback]) )
                    value = fallback
        if param_key == self.DEBUG_PARAM:
            self.msg_if.pub_debug("Got param val " + str([param_key,value]) )

        return value

    def set_param(self, param_key, value):
        if not nepi_sdk.is_shutdown():
            namespace = self.get_param_namespace(param_key)
            if namespace is not None and value is not None:
                self.params_ns_dict[namespace] = value
                nepi_sdk.set_param(namespace,value)
                if param_key == self.DEBUG_PARAM:
                    self.msg_if.pub_debug("Set param val " + str([param_key,self.params_ns_dict[namespace]]) )
                

    def reset_param(self, param_key):
        if param_key in self.params_dict.keys():
            if 'init_val' in self.params_dict[param_key].keys():
                init_val = self.params_dict[param_key]['init_val']
            else:
                init_val = self.params_dict[param_key]['factory_val']
            self.set_param(param_key, init_val)

    def factory_reset_param(self, param_key):
        if param_key in self.params_dict.keys():
            factory_val = self.params_dict[param_key]['factory_val']
            self.set_param(param_key, factory_val)

    def get_params(self):
        return list(self.params_dict.keys())


    def get_param_namespace(self,param_key):
        namespace = None
        if param_key in self.params_dict.keys() and not nepi_sdk.is_shutdown():
            param_dict = self.params_dict[param_key]
            param_name = param_dict.get('name',param_key)
            namespace = nepi_sdk.create_namespace(param_dict['namespace'],param_name)
        return namespace


    def add_param(self, param_key, name, namespace, value, do_init = True):
        with self.params_lock:
            if not nepi_sdk.is_shutdown():
                if param_key is not None and namespace is not None and value is not None:
                    if param_key not in self.params_dict.keys():
                        self.params_dict[param_key] = {
                    'namespace': namespace,
                    'name': name,
                    'factory_val': value
                }
            if do_init == True:
                self.initialize_params()

    def add_params(self,params_dict):
        with self.params_lock:
            if params_dict is not None:
                for param_key in params_dict:
                    param_dict = params_dict[param_key]
                    try:
                        self.add_param(param_key, param_dict['name'], param_dict['namespace'], param_dict['factory_val'], do_init = False)
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to add param: " + str(params_dict[param_key]) + " " + str(e))
                # self.params_dict.update(params_dict)
                self.initialize_params()




    ###############################
    # Class Private Methods
    ###############################





##################################################
### Node Services Class
def EXAMPLE_CALLBACK_FUNCTION(request):
    response = EmptySrvResponse()
    return response
'''
EXAMPLE_node_srvs_dict = {
    'service_name': {
        'namespace':  self.node_namespace,
        'topic': 'empty_query',
        'srv': EmptySrv,
        'req': EmptySrvRequest(),
        'resp': EmptySrvResponse(),
        'callback': EXAMPLE_CALLBACK_FUNCTION
    }
}
'''
class NodeServicesIF:

    msg_if = None
    ready = False
    srvs_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                services_dict = None,
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.msg_if.pub_debug("Starting Node Services IF Initialization Processes" )
        ##############################   

        ##############################
        # Initialize Services System
        # Guards srvs_dict: sub-IFs sharing this node's NodeClassIF register from
        # their own threads (see NodeParamsIF.params_lock).
        self.srvs_lock = threading.RLock()
        self.srvs_dict = services_dict
        if self.srvs_dict is None:
            self.srvs_dict = dict()
        self._initializeServices()

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready" )
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out" )
        else:
            self.msg_if.pub_debug("Ready" )
        return self.ready

       
    def get_services(self):
        return list(self.srvs_dict.keys())

    def create_request_msg(self,service_name):
        req = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'req' in srv_dict.keys():
                req = srv_dict['req']
        return req

    def create_response_msg(self,service_name):
        resp = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'resp' in srv_dict.keys():
                resp = srv_dict['resp']
        return resp

    def register_service(self,service_name, service_dict):
        with self.srvs_lock:
            self.srvs_dict[service_name] = service_dict
            self._initializeServices()

    def register_services(self,services_dict):
        with self.srvs_lock:
            if services_dict is not None:
                for service_name in services_dict.keys():
                    service_dict = services_dict[service_name]
                    self.srvs_dict[service_name] = service_dict
                self._initializeServices()

    def unregister_service(self,service_name):
        with self.srvs_lock:
            self._unregisterService(service_name)

    def unregister_services(self):
        with self.srvs_lock:
            service_names = list(self.srvs_dict.keys())
            for service_name in service_names:
                self._unregisterService(service_name)

    def add_services(self,services_dict):
        with self.srvs_lock:
            self.srvs_dict.update(services_dict)
            self._initializeServices()
    ###############################
    # Class Private Methods
    ###############################

    def _initializeServices(self):
        with self.srvs_lock:
            self._initializeServicesLocked()

    def _initializeServicesLocked(self):
        for service_name in self.srvs_dict.keys():
            #self.msg_if.pub_debug("Will try to create service for: " + service_name )
            srv_dict = self.srvs_dict[service_name]
            if 'service' not in srv_dict.keys() and srv_dict['callback'] is not None:
                srv_callback = None
                try:
                    srv_namespace = nepi_sdk.create_namespace(srv_dict['namespace'],srv_dict['topic'])
                    srv_msg = srv_dict['srv']
                    srv_callback = srv_dict['callback']
                except Exception as e:
                    self.msg_if.pub_debug("Failed to get service info from dict: " + service_name + " " + str(e), throttle_s = 5.0 ) 
                if srv_callback is not None and not nepi_sdk.is_shutdown():
                    #self.msg_if.pub_debug("Created service for: " + service_name + " with namespace: " + str(srv_namespace) )
                    service = None
                    try:
                        service = nepi_sdk.create_service(srv_namespace, srv_msg, srv_callback )   
                        self.srvs_dict[service_name]['service'] = service
                        self.srvs_dict[service_name]['namespace'] = srv_namespace
                        #self.msg_if.pub_debug("Created service for: " + service_name + " with namespace: " + str(srv_namespace), throttle_s = 5.0 )                 
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to get service connection: " + service_name + " " + str(e) )  
                    

    def _unregisterService(self, service_name):
        with self.srvs_lock:
            purge = False
            if service_name in self.srvs_dict.keys():
                srv_dict = self.srvs_dict[service_name]
                purge = True
                if 'service' in srv_dict.keys() and not nepi_sdk.is_shutdown():
                    try:
                        self.srvs_dict[service_name]['service'].shutdown()
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to get unregister service: " + service_name + " " + str(e) )
            if purge == True:
                del self.srvs_dict[service_name]
                    







##################################################
### Node Publishers Class
'''

EXAMPLE_PUBS_DICT = {
    'pub_name': {
        'namespace':  self.node_namespace,
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'latch': False
    }
}
'''

class NodePublishersIF:

    msg_if = None
    ready = False
    pubs_dict = dict()
    #######################
    ### IF Initialization
    def __init__(self, 
                pubs_dict = None,
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.msg_if.pub_debug("Starting Node Pubs IF Initialization Processes" )
        ##############################   


        ##############################
        # Initialize Publishers System
        # Guards pubs_dict against concurrent registration from sub-IFs sharing
        # this NodeClassIF (see NodeParamsIF.params_lock). publish_pub() and
        # has_subscribers_check() stay lock-free: they run on every frame, do a
        # single lookup rather than iterate, and _unregisterPub() never deletes
        # a key here, so a lookup cannot race a removal.
        self.pubs_lock = threading.RLock()
        self.pubs_dict = pubs_dict
        if self.pubs_dict is None:
            self.pubs_dict = dict()
        self.msg_if.pub_debug("Initializing with pub dict: " + str(self.pubs_dict) )
        self._initializePubs()

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready" )
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out" )
        else:
            self.msg_if.pub_debug("Ready" )
        return self.ready

        

    def get_pubs(self):
        return list(self.pubs_dict.keys())

    def has_subscribers_check(self,pub_name):
        has_subs = False
        if pub_name in self.pubs_dict.keys() and not nepi_sdk.is_shutdown():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                if pub_dict['pub'] is not None:
                    has_subs = pub_dict['pub'].get_num_connections() > 0
                    #self.msg_if.pub_debug("Pub has subscribers: " + pub_dict['namespace'] + "/" + pub_dict['topic'] + " " + str(has_subs), throttle_s = 5.0 ) 
        return has_subs

    def publish_pub(self,pub_name,pub_msg):
        success = False
        #self.pubs_dict_lock.acquire()
        if pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                if pub_dict['pub'] is not None and not nepi_sdk.is_shutdown():
                    try:
                        nepi_sdk.publish_pub(pub_dict['pub'], pub_msg )
                        success = True
                    except Exception as e:
                        namespace =  pub_dict['namespace']
                        self.msg_if.pub_debug("Failed to publish msg: " + pub_name + \
                            " " + str(namespace)  + " " + str(pub_msg) + str(e), throttle_s = 5.0 )   
        #self.pubs_dict_lock.release()
        return success

    def register_pub(self,pub_name, pub_dict):
        with self.pubs_lock:
            self.pubs_dict[pub_name] = pub_dict
            self._initializePubs(print_msg = True)


    def register_pubs(self,pubs_dict = None):
        # A no-arg call means "re-advertise what is already registered", which is
        # what every caller that stops and restarts publishing does. Guarding the
        # whole body on pubs_dict made that call a silent no-op: _unregisterPub
        # leaves the entry in pubs_dict with 'pub' set to None, and only
        # _initializePubs builds a publisher for an entry in that state, so the
        # topic stayed off the wire for the life of the node.
        with self.pubs_lock:
            if pubs_dict is not None:
                self.pubs_dict.update(pubs_dict)
            self._initializePubs(print_msg = True)




    def unregister_pub(self,pub_name):
        with self.pubs_lock:
            self._unregisterPub(pub_name)

    def unregister_pubs(self):
        with self.pubs_lock:
            pub_names = list(self.pubs_dict.keys())
            for pub_name in pub_names:
                self._unregisterPub(pub_name)

    def add_pubs(self,pubs_dict):
        self.msg_if.pub_debug("Adding pubs dict: " + str(pubs_dict)  )
        with self.pubs_lock:
            self.pubs_dict.update(pubs_dict)
            #self.msg_if.pub_debug("Updated pubs dict: " + str(pubs_dict)  )
            self._initializePubs(print_msg = True)

    ###############################
    # Class Private Methods
    ###############################
    def _initializePubs(self, print_msg = False):
        with self.pubs_lock:
            self._initializePubsLocked(print_msg = print_msg)

    def _initializePubsLocked(self, print_msg = False):
        for pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            add_pub = False
            if 'pub' not in pub_dict.keys():
                add_pub = True
            elif pub_dict['pub'] is None:
                add_pub = True
            if add_pub == True:  
                if 'topic' in pub_dict.keys() and 'msg' in pub_dict.keys() and not nepi_sdk.is_shutdown():
                    pub_namespace = nepi_sdk.create_namespace(pub_dict['namespace'] ,pub_dict['topic'])
                    #self.msg_if.pub_debug("Creating pub for: " + pub_name + " with namespace: " + pub_namespace  ) 
                    pub = None
                    if 'qsize' not in pub_dict.keys():
                        self.pubs_dict[pub_name]['qsize'] = 1
                        pub_dict['qsize'] = 1
                    if 'latch' not in pub_dict.keys():
                        self.pubs_dict[pub_name]['latch'] = False
                        pub_dict['latch'] = False
                    try:
                        pub = nepi_sdk.create_publisher(pub_namespace, pub_dict['msg'], queue_size = pub_dict['qsize'], \
                                latch = pub_dict['latch'] )
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to create publisher: " + pub_name + " " + str(e) ) 
                    self.pubs_dict[pub_name]['pub_namespace'] = pub_namespace
                    self.pubs_dict[pub_name]['pub'] = pub
                    # if print_msg == True:
                    #     self.msg_if.pub_debug("Added Pub: " + pub_name )
            else:
                #self.msg_if.pub_debug("Pubublisher already exists for: " + pub_name )
                pass


    def _unregisterPub(self, pub_name):
        with self.pubs_lock:
            if pub_name in self.pubs_dict.keys():
                pub_dict = self.pubs_dict[pub_name]
                purge = True
                if 'pub' in pub_dict.keys() and not nepi_sdk.is_shutdown():
                    if pub_dict['pub'] is not None:
                        try:
                            self.pubs_dict[pub_name]['pub'].unregister()
                            self.msg_if.pub_debug("Unregister pub: " + pub_name )
                        except Exception as e:
                            self.msg_if.pub_debug("Failed to get unregister pub: " + pub_name + " " + str(e) )
                        self.pubs_dict[pub_name]['pub'] = None



##################################################
### Node Subscribers Class
'''
def EXAMPLE_SUB_CALLBACK(msg):
    return msg

EXAMPLE_SUBS_DICT = {
    'sub_name': {
        'namespace':  self.node_namespace,
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'callback': EXAMPLE_SUB_CALLBACK, 
        'callback_args': ()
    }
}
'''
class NodeSubscribersIF:

    msg_if = None
    ready = False
    subs_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                subs_dict = None,
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.msg_if.pub_debug("Starting Node Subs IF Initialization Processes" )

        ##############################

        # Guards subs_dict against concurrent registration from sub-IFs sharing
        # this NodeClassIF (see NodeParamsIF.params_lock). _initializeSubs()
        # iterates subs_dict while create_subscriber() blocks on the ROS master,
        # which is exactly the window a second thread's register_subs() hit:
        # ZED's color_image and depth_map threads both build image IFs on the
        # IDX node_if at once, one raised "dictionary changed size during
        # iteration", and that IF's wait_for_ready() then blocked forever.
        self.subs_lock = threading.RLock()
        self.subs_dict = subs_dict
        if self.subs_dict is None:
            self.subs_dict = dict()
        self._initializeSubs()


        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready" )
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out" )
        else:
            self.msg_if.pub_debug("Ready" )
        return self.ready

        
    def get_subs(self):
        return list(self.subs_dict.keys())


    def register_sub(self,sub_name, sub_dict):
        with self.subs_lock:
            self.subs_dict[sub_name] = sub_dict
            self._initializeSubs()

    def register_subs(self,subs_dict):
        with self.subs_lock:
            if subs_dict is not None:
                for sub_name in subs_dict.keys():
                    sub_dict = subs_dict[sub_name]
                    self.subs_dict[sub_name] = sub_dict
                self._initializeSubs()


    def unregister_sub(self,sub_name):
        with self.subs_lock:
            self._unregisterSub(sub_name)

    def unregister_subs(self):
        with self.subs_lock:
            sub_names = list(self.subs_dict.keys())
            for sub_name in sub_names:
                self._unregisterSub(sub_name)


    def add_subs(self,subs_dict):
        with self.subs_lock:
            self.subs_dict.update(subs_dict)
            self._initializeSubs()
    ###############################
    # Class Private Methods
    ###############################
    def _initializeSubs(self):
        with self.subs_lock:
            self._initializeSubsLocked()

    def _initializeSubsLocked(self):
        for sub_name in self.subs_dict.keys():
            sub_dict = self.subs_dict[sub_name]
            #self.msg_if.pub_debug("Will try to create sub for: " + sub_name )
            if 'sub' not in sub_dict.keys() and sub_dict['callback'] is not None and not nepi_sdk.is_shutdown():
                sub_namespace = nepi_sdk.create_namespace(sub_dict['namespace'],sub_dict['topic'])
                #self.msg_if.pub_debug("Creating sub for: " + sub_name + " with namespace: " + sub_namespace ) 
                if 'callback_args' not in sub_dict.keys():
                    sub_dict['callback_args'] = ()
                if sub_dict['callback_args'] is None:
                    sub_dict['callback_args'] = ()
                try:
                    if len(sub_dict['callback_args']) == 0:
                        sub = nepi_sdk.create_subscriber(sub_namespace, sub_dict['msg'], sub_dict['callback'], queue_size = sub_dict['qsize'])
                            
                    else:
                        sub = nepi_sdk.create_subscriber(sub_namespace, sub_dict['msg'],sub_dict['callback'], queue_size = sub_dict['qsize'], \
                             callback_args=sub_dict['callback_args'] )
                    self.subs_dict[sub_name]['sub'] = sub
                    self.subs_dict[sub_name]['sub_namespace'] = sub_namespace
                    success = True
                    #self.msg_if.pub_debug("Created sub for: " + sub_name + " with namespace: " + sub_namespace ) 
                except Exception as e:
                    self.msg_if.pub_debug("Failed to create subscriber: " + sub_name + " " + str(e) )
                    self.subs_dict[sub_name]['sub'] = None

    def _unregisterSub(self, sub_name):
        with self.subs_lock:
            purge = False
            if sub_name in self.subs_dict.keys():
                sub_dict = self.subs_dict[sub_name]
                purge = True
                if 'sub' in sub_dict.keys() and not nepi_sdk.is_shutdown():
                    try:
                        self.subs_dict[sub_name]['sub'].unregister()
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to get unregister sub: " + sub_name + " " + str(e), throttle_s = 5.0)
            if purge == True:
                del self.subs_dict[sub_name]


##################################################
### Node Class

# Configs Dict ####################
'''
EXAMPLE_CONFIGS_DICT = {
        'init_callback': None,
        'reset_callback': None,
        'factory_reset_callback': None,
        'init_configs': True,
        'namespace':  self.node_namespace
}


# Params Dict ####################
EXAMPLE_node_params_dict = {
    'param1_name': {
        'namespace':  self.node_namespace,
        'factory_val': 100,
        'current_val: 20  # Optional
    },
    'param2_name': {
        'namespace':  self.node_namespace,
        'factory_val': "Something"
    }
}


# Services Dict ####################
EXAMPLE_node_srvs_dict = {
    'service_name': {
        'namespace':  self.node_namespace,
        'topic': 'empty_query',
        'srv': EmptySrv,
        'req': EmptySrvRequest(),
        'resp': EmptySrvResponse(),
        'callback': EXAMPLE_CALLBACK_FUNCTION
    }
}


# Publishers Dict ####################
EXAMPLE_PUBS_DICT = {
    'pub_name': {
        'namespace':  self.node_namespace,
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'latch': False
    }
}


# Subscribers Dict ####################
EXAMPLE_SUBS_DICT = {
    'sub_name': {
        'namespace':  self.node_namespace,
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'callback': EXAMPLE_SUB_CALLBACK, 
        'callback_args': ()
    }
}


# Create Node Class ####################

EXAMPLE_NODE_IF = NodeClassIF(
                configs_dict = EXAMPLE_CONFIGS_DICT,
                params_dict = EXAMPLE_node_params_dict,
                services_dict = EXAMPLE_node_srvs_dict,
                pubs_dict = EXAMPLE_PUBS_DICT,
                subs_dict = EXAMPLE_SUBS_DICT,
                log_class_name = True
)
'''

class NodeClassIF:

    ready = False
    node_if = None
    msg_if = None
    configs_dict = None
    configs_if = None
    params_if = None
    services_if = None
    pubs_if = None
    subs_if = None


    #######################
    ### IF Initialization
    def __init__(self, 
                node_name = None,
                configs_dict = None,
                params_dict = None,
                services_dict = None,
                pubs_dict = None,
                subs_dict = None,
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        if node_name is not None:
            nepi_sdk.init_node(name = node_name) # Can be overwitten by luanch command
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.msg_if.pub_debug("Starting Node IF Initialization Processes" )

        ##############################  
        # Create Sub Classes
           
        ##############################  
        # Create Config Class After Params
        if configs_dict is None:
            self.configs_dict = configs_dict
        else:
            # Need to inject our own config callback functions that call the params_if functions first
            self.configs_dict = configs_dict
            # Copy and override rather than rebuild from scratch, so alt_namespace,
            # clear_params, manage_configs and anything else the caller set survive the
            # injection. The rebuilt literal this replaced carried only namespace and
            # silently dropped alt_namespace, which reset_params and delete_config_all
            # both read. Copying also keeps NodeConfigsIF's own 'namespace' default
            # assignment off the caller's dict.
            configs_dict = dict(configs_dict)
            configs_dict['init_callback'] = self._initConfigCb
            configs_dict['reset_callback'] = self._resetConfigCb
            configs_dict['factory_reset_callback'] = self._factoryResetConfigCb


        self.configs_if = NodeConfigsIF(configs_dict = configs_dict,
                                        
                                        msg_if = self.msg_if
                                        )
        nepi_sdk.sleep(1)


        self.params_if = NodeParamsIF(params_dict = params_dict, msg_if = self.msg_if )
        self.services_if = NodeServicesIF(services_dict = services_dict, msg_if = self.msg_if )

        self.pubs_if = NodePublishersIF(pubs_dict = pubs_dict, msg_if = self.msg_if )
        self.subs_if = NodeSubscribersIF(subs_dict = subs_dict, msg_if = self.msg_if )

        nepi_sdk.sleep(0.1)
      
        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("Node IF Initialization Complete" )
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready" )
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out" )
        else:
            self.msg_if.pub_debug("Ready" )
        return self.ready

    def get_namespace(self):
        return self.node_namespace

    # Config Methods ####################
    # 

    def add_configs(self, configs_dict):
        if self.configs_if is not None:
            self.configs_if.add_configs(configs_dict)

    def init_config(self):
        if self.configs_if is not None:
            self.configs_if.init_config()

    def reset_config(self):
        if self.configs_if is not None:
            self.configs_if.reset_config()

    def factory_reset_config(self):
        if self.configs_if is not None:
            self.configs_if.factory_reset_config()
    def save_config(self):
        if self.configs_if is not None:
            self.configs_if.save_config()

    def save_config_all(self):
        if self.configs_if is not None:
            self.configs_if.save_config_all()




    # Param Methods ####################
    def add_param(self,param_key, name, namespace, value):
        if self.params_if is not None:
            params = self.params_if.add_param(param_key, name, namespace, value)

    def add_params(self,params_dict):
        if self.params_if is not None:
            self.params_if.add_params(params_dict)

    def get_params(self):
        params = None
        if self.params_if is not None:
            params = self.params_if.get_params()
        return params
    
    def load_params(self, file_path):
        if self.params_if is not None:
            self.params_if.load_params(file_path)

    def initialize_params(self):
        if self.params_if is not None:
            self.params_if.initialize_params()


    def reset_params(self, param_keys = None):
        if self.params_if is not None:
            self.params_if.reset_params(param_keys)

    def factory_reset_params(self, param_keys = None):
        if self.params_if is not None:
            self.params_if.factory_reset_params(param_keys)


    def save_params(self,  param_keys = None):
        if self.params_if is not None:
            self.params_if.save_params(param_keys)



    def has_param(self, param_key):
        exists = False
        if self.params_if is not None:
            exists = self.params_if.has_param(param_key)
        return exists

    def get_param(self, param_key, fallback = None):
        value = None
        if self.params_if is not None:
            value = self.params_if.get_param(param_key, fallback)

        return value

    def set_param(self, param_key, value):
        success = False
        if self.params_if is not None:
            self.params_if.set_param(param_key,value)
        return success

    def reset_param(self, param_key):
        success = False
        if self.params_if is not None:
            self.params_if.reset_param(param_key)
        return success

    def factory_reset_param(self, param_key):
        success = False
        if self.params_if is not None:
            self.params_if.reset_param(param_key)

        return success

    # Service Methods ####################
    def get_services(self):
        srvs = None
        if self.services_if is not None:
            srvs = self.services_if.get_services()
        return srvs

    def register_service(self,service_name, service_dict):
        if self.services_if is not None:
            self.services_if.register_service(service_name, service_dict)

    def register_services(self, services_dict):
        if self.services_if is not None and services_dict is not None:
            self.services_if.register_services( services_dict)


    def unregister_service(self,service_name):
        if self.services_if is not None:
            self.services_if.unregister_service(service_name)

    def unregister_services(self):
        service_names = list(self.srvs_dict.keys())  
        for service_name in service_names:  
            self.services_if.unregister_services()


    # Publisher Methods ####################
    def get_pubs(self):
        pubs = []
        if self.pubs_if is not None:
            pubs = self.pubs_if.get_pubs()
        return pubs

    def pub_has_subscribers(self,pub_name):
        has_subs = False
        if self.pubs_if is not None:
            has_subs = self.pubs_if.has_subscribers_check(pub_name)
        return has_subs

    def register_pub(self,pub_name, pub_dict):
        if self.pubs_if is not None:
            self.pubs_if.register_pub(pub_name, pub_dict)

    def register_pubs(self,pubs_dict = None):
        if self.pubs_if is not None:
            self.pubs_if.register_pubs(pubs_dict)


    def unregister_pub(self,pub_name):
        if self.pubs_if is not None:
            self.pubs_if.unregister_pub(pub_name)

    def unregister_pubs(self):
        if self.pubs_if is not None:
            self.pubs_if.unregister_pubs()

    def publish_pub(self,pub_name, pub_msg):
        success = False
        if self.pubs_if is not None and not nepi_sdk.is_shutdown():
            succes = self.pubs_if.publish_pub(pub_name, pub_msg)   
        return success
            
    # Subscriber Methods ####################
    def get_subs(self):
        subs = []
        if self.subs_if is not None:
            subs = self.subs_if.get_subs()
        return subs


    def register_sub(self,sub_name, sub_dict):
        if self.subs_if is not None:
            self.subs_if.register_sub(sub_name, sub_dict)

    def register_subs(self, subs_dict):
        if self.subs_if is not None and subs_dict is not None:
            self.subs_if.register_subs(subs_dict)

    def unregister_sub(self,sub_name):
        if self.subs_if is not None:
            self.subs_if.unregister_sub(sub_name)

    def unregister_subs(self):
        if self.subs_if is not None:
            self.subs_if.unregister_subs()

    # Class Methods ####################
    def unregister_class(self):
        if self.services_if is not None:
            self.services_if.unregister_services()
        if self.pubs_if is not None:
            self.pubs_if.unregister_pubs()
        if self.subs_if is not None:
            self.subs_if.unregister_subs()

    ###############################
    # Class Private Methods
    ###############################

    def _initConfigCb(self, do_updates = False):
        self.initialize_params()
        if self.configs_dict is not None:
            if 'init_callback' in self.configs_dict.keys():
                if self.configs_dict['init_callback'] is not None:
                    self.configs_dict['init_callback'](do_updates = do_updates)
            

    def _resetConfigCb(self):
        self.msg_if.pub_debug("Node Got Reset Params Request" )
        self.reset_params()
        if self.configs_dict is not None:
            if 'reset_callback' in self.configs_dict.keys():
                if self.configs_dict['reset_callback'] is not None:
                    self.configs_dict['reset_callback']()

    def _factoryResetConfigCb(self):
        self.msg_if.pub_debug("Node Got Factory Reset Params Request" )
        self.factory_reset_params()
        if self.configs_dict is not None:
            if 'factory_reset_callback' in self.configs_dict.keys():
                if self.configs_dict['factory_reset_callback'] is not None:
                    self.configs_dict['factory_reset_callback']()


