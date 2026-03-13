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
import copy
import inspect

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_triggers
from nepi_sdk import nepi_states

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import Reset

from nepi_interfaces.msg import SaveDataRate, SaveDataStatus, FilenameConfig
from nepi_interfaces.srv import SaveDataCapabilitiesQuery, SaveDataCapabilitiesQueryRequest, SaveDataCapabilitiesQueryResponse



from nepi_interfaces.msg import Setting, SettingsStatus
from nepi_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryRequest, SettingsCapabilitiesQueryResponse

from nepi_interfaces.msg import SystemState
from nepi_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_interfaces.msg import SystemTrigger
from nepi_interfaces.srv import SystemTriggersQuery, SystemTriggersQueryRequest, SystemTriggersQueryResponse


from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF



class ConnectSaveDataIF:

    ready = False

    namespace = None
    
    status_msg = None
    status_connected = False


    #######################
    ### IF Initialization
    def __init__(self, namespace = None , timeout = float('inf'),
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting SaveData IF Initialization Processes", log_name_list = self.log_name_list)
        ############################## 
        

        #############################
        # Initialize Class Variables
        if namespace is None:
            namespace = '~'
        namespace = nepi_sdk.create_namespace(namespace,'save_data')
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.msg_if.pub_warn("Using save data namespace: " + self.namespace, log_name_list = self.log_name_list)

        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'capabilities_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': SaveDataCapabilitiesQuery,
                'req': SaveDataCapabilitiesQueryRequest(),
                'resp': SaveDataCapabilitiesQueryResponse()
            }
        }


        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'prefix': {
                'msg': String,
                'namespace': self.namespace,
                'topic': 'save_data_prefix',
                'qsize': 1,
                'latch': False
            },
            'snapshot': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'snapshot_trigger',
                'qsize': 1,
                'latch': False
            },
            'reset': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'save_data_reset',
                'qsize': 1,
                'latch': False
            },
            'factory_reset': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'save_data_factory_reset',
                'qsize': 1,
                'latch': False
            },            
            'prefix_all': {
                'msg': String,
                'namespace': self.base_namespace,
                'topic': 'save_data_prefix',
                'qsize': 1,
                'latch': False
            },
            'rate_all': {
                'msg': SaveDataRate,
                'namespace': self.base_namespace,
                'topic': 'save_data_rate',
                'qsize': 1,
                'latch': False
            },
            'save_all_data': {
                'msg': Bool,
                'namespace': self.base_namespace,
                'topic': 'save_data_enable',
                'qsize': 1,
                'latch': False
            },
            'snapshot_all': {
                'msg': Empty,
                'namespace': self.base_namespace,
                'topic': 'snapshot_trigger',
                'qsize': 1,
                'latch': False
            },
            'reset_all': {
                'msg': Empty,
                'namespace': self.base_namespace,
                'topic': 'reset',
                'qsize': 1,
                'latch': False
            },
            'factory_reset_all': {
                'msg': Empty,
                'namespace': self.base_namespace,
                'topic': 'factory_reset',
                'qsize': 1,
                'latch': False
            }
        }



        # Subs Config Dict ####################
        self.SUBS_DICT = {    
            'status_sub': {
                'msg': SaveDataStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'callback': self._statusCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(services_dict = self.SRVS_DICT,
                                            pubs_dict = self.PUBS_DICT,
                                            subs_dict = self.SUBS_DICT,
                                            log_name_list = self.log_name_list,
                                            msg_if = self.msg_if
                                            )


        #self.con_node_if.wait_for_ready()
        nepi_sdk.wait()

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready   
        

    def get_namespace(self):
        return self.namespace

    def get_data_products(self):
        srv_name =  'data_products_query'
        req = DataProductQueryRequest()
        self.con_node_if.call_service(srv_name,req)

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        else:
            self.msg_if.publish_warn("Status Listener Not ready", log_name_list = self.log_name_list)
        return status_dict

    def save_data_pub(self,enable):
        pub_name = 'save_data'
        msg = enable
        self.con_node_if.publish_pub(pub_name,msg)

    def save_data_prefix_pub(self,prefix):
        pub_name = 'prefix'
        msg = prefix
        self.con_node_if.publish_pub(pub_name,msg)

    def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        pub_name = 'rate'
        msg = SaveDataRate()
        msg.data_product = data_product
        msg.save_rate_hz = rate_hz
        self.con_node_if.publish_pub(pub_name,msg)

    def snapshot_pub(self):
        pub_name = 'snapshot'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def reset_pub(self):
        pub_name = 'reset'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def factory_reset_pub(self):
        pub_name = 'factory_reset'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)
       
    def unregister_save_data_if(self): 
        success = False
        self.con_node_if.unregister()
        time.sleep(1)
        self.ready = False
        self.status_connected = False
        success = True
        return success  

    #################
    ## Save Config Functions

    def call_save_config(self):
        self.con_node_if.publish_pub('save_config',Empty())

    def call_reset_config(self):
        self.con_node_if.publish_pub('reset_config',Empty())

    def call_factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config',Empty())

    ###############################
    # Class Private Methods
    ###############################

    def _statusCb(self,msg):
        self.status_msg = msg
        self.status_connected = True




SETTING_TYPES = ["Menu","Discnamespacee","String","Bool","Int","Float"]

'''
EXAMPLE_CAP_SETTINGS_DICT = {"None":{"name":"None","type":"None","optons":[]}}
EXAMPLE_SETTINGS_DICT = {"None":{"name":"None","type":"None","value":"None"}}
'''

class ConnectSettingsIF:

    msg_if = None
    ready = False
    namespace = '~'

    settings_dict = None
    cap_settings_dict = None
    has_cap_updates = False

    #######################
    ### IF Initialization
    def __init__(self, namespace , timeout = float('inf'),
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting Settings IF Initialization Processes", log_name_list = self.log_name_list)
        ############################## 
        

        #############################
        # Initialize Class Variables
        if namespace is None:
            namespace = '~'
        namespace = nepi_sdk.create_namespace(namespace,'settings')
        self.namespace = nepi_sdk.get_full_namespace(namespace)


        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'setting_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'msg': SettingsCapabilitiesQuery
            }
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'cap_update_pub': {
                'msg': Setting,
                'namespace': self.namespace,
                'topic': 'update_cap_setting',
                'qsize': 1,
                'latch': False
            },
            'setting_update_pub': {
                'msg': Setting,
                'namespace': self.namespace,
                'topic': 'update_setting',
                'qsize': 1,
                'latch': False
            },
            'reset_pub': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset',
                'qsize': 1,
                'latch': False
            },
            'factory_reset_pub': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'factory_reset',
                'qsize': 1,
                'latch': False
            }
        }


        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'msg': SettingsStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'callback': self._settingsCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                                            log_name_list = self.log_name_list,
                                            msg_if = self.msg_if
                                            )

        #self.con_node_if.wait_for_ready()
        nepi_sdk.wait()

        ##############################
        # Run Initialization Processes
        self.cap_settings_dict = self._getCapSettings()   
        #self.msg_if.pub_info("Cap Settings Message: " + str(self.cap_settings_dict))      

        ##############################
        # Complete Initialization
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def get_namespace(self):
        return self.namespace

    def get_settings_capabilities_dict(self):
        return self.cap_settings_dict

    def get_has_capabilities_updating(self):
        return self.has_cap_updates

    def get_settings_dict(self):
        return self.settings_dict

    def update_cap_setting(self, cap_setting_dict):
        success = False
        if self.has_cap_updates == True:
            msg = nepi_settings.create_msg_from_cap_setting(cap_setting_dict)
            success = self.con_node_if.publish_pub('cap_update_pub',msg)
        return success

    def update_setting(self, setting_dict):
        msg = nepi_settings.create_msg_from_setting(setting_dict)
        success = self.con_node_if.publish_pub('setting_update_pub',msg)
        return success


    def reset_settings(self):
        msg = Empty()
        success = self.con_node_if.publish_pub('reset_pub',msg)
        return success

    def factory_reset_settings(self):
        msg = Empty()
        success = self.con_node_if.publish_pub('factory_reset_pub',msg)
        return success

     #################
    ## Save Config Functions

    def call_save_config(self):
        self.con_node_if.publish_pub('save_config',Empty())

    def call_reset_config(self):
        self.con_node_if.publish_pub('reset_config',Empty())

    def call_factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config',Empty())

    ###############################
    # Class Private Methods
    ###############################

    def _getCapSettings(self):
        req = SettingsCapabilitiesQueryRequest()
        response = self.con_node_if.call_service('capabilities_query',req)
        if response is not None:
            [self.cap_settings_dict,self.has_cap_updates] = nepi_settings.parse_capabilities_response(response)
        return self.cap_settings_dict

    def _settingsCb(self,msg):
        [self.settings_dict,self.cap_settings_dict,self.has_cap_updates] = nepi_settings.parse_status_msg(msg)






STATE_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]
'''
EXAMPLE_STATE_DICT = {"name":"None",
                        "description": "None",
                        "type":"Bool",
                        "options": [],
                        "value":"False"
}

EXAMPLE_STATES_DICT = {"example_state":EXAMPLE_STATE_DICT}
'''


class ConnectStatesIF:

    msg_if = None
    ready = False
    namespace = '~'

    status_msg = None


    #######################
    ### IF Initialization
    def __init__(self,
                namespace = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting States IF Initialization Processes", log_name_list = self.log_name_list)
        ############################## 
        
        #############################
        # Initialize Class Variables
        if namespace is None:
            namespace = '~'
        self.namespace = nepi_sdk.get_full_namespace(namespace)

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
        self.con_node_if = ConnectNodeClassIF(subs_dict = self.SUBS_DICT,
                                            log_name_list = self.log_name_list,
                                            msg_if = self.msg_if
                                            )

        #self.con_node_if.wait_for_ready()
        nepi_sdk.wait()

        ##############################
        # Complete Initialization
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and self.status_msg is None and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  


    def get_states_names(self):
        state_names = []
        if self.status_msg is not None:
                state_names = self.status_msg.states_names_list
        else:
            self.msg_if.pub_warn(self,":" + self.class_name + ": state Status listener has not received any data yet: ", log_name_list = self.log_name_list)
        return state_names


    def get_states_status_dict(self):
        states_dict = None
        if self.status_msg is not None:
                states_dict = nepi_states.parse_trigger_status_msg(self.status_msg)
        else:
            self.msg_if.pub_warn(self,":" + self.class_name + ": state Status listener has not received any data yet: ", log_name_list = self.log_name_list)
        return states_dict


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



'''
EXAMPLE_TRIGGER_DICT = {"name":"None",
                        "description": "None",
                        "data_str_list":["None"],
                        "time":nepi_utils.get_time(),
                        "node":"None"
}

EXAMPLE_TRIGGERS_DICT = {"example_trigger":EXAMPLE_TRIGGER_DICT}
'''


class ConnectTriggersIF:

    msg_if = None
    ready = False

    trigger_handlers_dict = dict()

    status_msg = None

    #######################
    ### IF Initialization
    def __init__(self,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting Triggers IF Initialization Processes", log_name_list = self.log_name_list)
        ############################## 
        

        #############################
        # Initialize Class Variables

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
                'topic': 'system_triggers',
                'qsize': 1,
                'callback': self._triggerCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(subs_dict = self.SUBS_DICT,
                                            log_name_list = self.log_name_list,
                                            msg_if = self.msg_if
                                            )

        #self.con_node_if.wait_for_ready()
        nepi_sdk.wait()

        ##############################
        # Complete Initialization
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and self.status_msg is None and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  


    def get_triggers_names(self):
        trigger_names = []
        if self.status_msg is not None:
            trigger_names = self.status_msg.triggers_name_list
        else:
            self.msg_if.pub_warn(self,":" + self.class_name + ": Trigger Status listener has not received any data yet: ", log_name_list = self.log_name_list)
        return trigger_names


    def get_triggers_status_dict(self):
        triggers_dict = None
        if self.status_msg is not None:
            [name_list, triggered_list] = nepi_triggers.parse_trigger_status_msg(self.status_msg)
            for i, name in enumerate(name_list):
                triggers_dict[name] = triggered_list[i]
        else:
            self.msg_if.pub_warn(self,":" + self.class_name + ": Trigger Status listener has not received any data yet: ", log_name_list = self.log_name_list)
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
            self.msg_if.pub_warn("No trigger handler registed for trigger: " + trigger_name + " " + str(e))
        return success


    def get_last_trigger_time(self, trigger_name):
        last_time = None
        try:
            last_time = self.trigger_handlers_dict[trigger_name]['last_time']
        except Exception as e:
            self.msg_if.pub_warn("No trigger handler registed for trigger: " + trigger_name + " " + str(e))
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
                self.msg_if.pub_warn("Failed to call trigger handler function: " + str(e))
        if trigger_name in self.trigger_handlers_dict.keys():
            self.trigger_handlers_dict[trigger_name]['last_time'] = nepi_utils.get_time()
            try:
                self.trigger_handlers_dict[trigger_name]['handler'](trigger_dict)
            except:
                self.msg_if.pub_warn("Failed to call trigger handler function: " + str(e))

    # Update System Status
    def _statusCb(self,msg):
        self.status_msg = msg
        self.ready = True