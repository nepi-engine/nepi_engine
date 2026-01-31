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

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils


from std_msgs.msg import String, Empty

from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyRequest as EmptySrvRequest
from std_srvs.srv import EmptyResponse as EmptySrvResponse

from nepi_interfaces.msg import Reset

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodePublishersIF, NodeSubscribersIF





##################################################
### Node Connect Configs Class
'''
EXAMPLE_CFGS_CONFIG_DICT = {
        'namespace': '~'
}
'''

class ConnectNodeConfigsIF:


    msg_if = None
    ready = False
    configs_dict = None
    namespace = '~'

    save_pub = None
    reset_pub = None
    factory_reset_pub = None

    #######################
    ### IF Initialization
    def __init__(self, configs_dict , timeout = float('inf'),
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
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_debug("Starting Connect Node Configs IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################  
        # Initialize Class Variables
        namespace = configs_dict['namespace']
        if namespace is None:
            namespace = self.node_namespace
        else:
            namespace = namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)


        ##############################  
        # Setup Class Publishers for Node Namespace
        self.msg_if.pub_debug("Creating config publishers on namespace: " + self.namespace, log_name_list = self.log_name_list)
        self.save_pub = nepi_sdk.create_publisher(self.namespace + '/save_config', Empty, queue_size=1) 
        self.reset_pub = nepi_sdk.create_publisher(self.namespace + '/reset_config', Empty, queue_size=1) 
        self.factory_reset_pub = nepi_sdk.create_publisher(self.namespace + '/factory_reset_config', Empty, queue_size=1) 
        self.sys_reset_pub = nepi_sdk.create_publisher(self.namespace + '/system_reset', Reset, queue_size=1)

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete", log_name_list = self.log_name_list)
        ##############################   


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_debug("ready", log_name_list = self.log_name_list)
        return self.ready


    def get_namespace(self):
        return self.namespace

    def save(self):
        success = False
        if self.save_pub is not None:
            try:
                msg = Empty()
                self.save_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send update config: " + str(e), log_name_list = self.log_name_list)
        return success


    def reset(self):
        success = False
        if self.reset_pub is not None:
            try:
                msg = Empty()
                self.reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success

    def factory_reset(self):
        success = False
        if self.factory_reset_pub is not None:
            try:
                msg = Empty()
                self.factory_reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success

    def system_user_reset(self):
        success = False
        if self.sys_reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.USER_RESET
                self.sys_reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success

    def system_factory_reset(self):
        success = False
        if self.sys_reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.FACTORY_RESET
                self.sys_reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success

    def system_software_reset(self):
        success = False
        if self.sys_reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.SOFTWARE_RESET
                self.sys_reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success

    def system_hardware_reset(self):
        success = False
        if self.sys_reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.HARDWARE_RESET
                self.sys_reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e), log_name_list = self.log_name_list)
        return success




    def unregister_save_if(self): 
        success = False
        if self.ready is False or self.namespace is None:
            self.msg_if.pub_warn("Save Config IF not running", log_name_list = self.log_name_list)
            success = True
        else:
            self.msg_if.pub_warn("Killing Save Confg IF for trigger: " + self.namespace, log_name_list = self.log_name_list)
            try:
                self.save_pub.unregister()
                self.reset_pub.unregister()
                self.factory_reset_pub.unregister()
                self.syst_reset_pub.unregister()


                success = True
            except:
                pass
            time.sleep(1)
            self.ready = False
            self.save_pub = None
            self.reset_pub = None
            self.factory_reset_pub = None
            self.sys_reset_pub = None
            self.namespace = None
        return success       

    ###############################
    # Class Private Methods
    ###############################





##################################################
### Node Connect Services Class

# Services Dict ####################
EXAMPLE_SRVS_DICT = {
    'service_name': {
        'namespace': '~',
        'topic': 'empty_query',
        'srv': EmptySrv,
        'req': EmptySrvRequest(),
        'resp': EmptySrvResponse()
    }
}

class ConnectNodeServicesIF:


    msg_if = None
    ready = False
    srvs_dict = dict()


    #######################
    ### IF Initialization
    def __init__(self, 
                services_dict = None,
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
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_debug("Starting Connect Node Services IF Initialization Processes", log_name_list = self.log_name_list)
        ##############################   

        ##############################  
        # Initialize Services System
        self.srvs_dict = services_dict
        if self.srvs_dict is None:
            self.srvs_dict = dict()
        self._initialize_services()

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete", log_name_list = self.log_name_list)
        ##############################   

    ###############################
    # Class Public Methods
    ###############################
    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_debug("ready", log_name_list = self.log_name_list)
        return self.ready


    def get_services(self):
        return list(self.srvs_dict.keys())

    def create_request_msg(self, service_name):
        req = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'req' in srv_dict.keys():
                req = srv_dict['req']
        return req

    def create_response_msg(self, service_name):
        resp = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'resp' in srv_dict.keys():
                resp = srv_dict['resp']
        return resp


    def call_service(self, service_name, request, verbose = True):
        resp = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'service' in srv_dict.keys():
                service = srv_dict['service']
                if service is not None:
                    resp = nepi_sdk.call_service(service, request, verbose = verbose, log_name_list = self.log_name_list)
        return resp


    def register_service(self,service_name, service_dict):
        self.srvs_dict[service_name] = service_dict
        self._initialize_services()

    def unregister_service(self,service_name):
        self._unregister_service(service_name)

    def unregister_services(self):
        for service_name in self.srvs_dict.keys():
            self._unregister_service(service_name)

    def add_services(self,services_dict):
        self.services_dict.update(services_dict)
        self.initialize_services()

    ###############################
    # Class Private Methods
    ###############################

    def _initialize_services(self):
        for service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'service' not in srv_dict.keys():
                srv_namespace = os.path.join(srv_dict['namespace'],srv_dict['topic'])
                srv_msg = srv_dict['srv']
                self.msg_if.pub_debug("Creating service for: " + service_name, log_name_list = self.log_name_list)
                service = None
                try:
                    service = nepi_sdk.connect_service(srv_namespace, srv_msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get service connection: " + service_name + " " + str(e), log_name_list = self.log_name_list) 
                self.srvs_dict[service_name]['service'] = service

    def _unregister_service(self, service_name):
        purge = False
        if service_name in self.srvs_dict.keys():
            purge = True
            if 'service' in srv_dict.keys():
                try:
                    self.srvs_dict[service_name]['service'].shutdown()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister service: " + service_name + " " + str(e), log_name_list = self.log_name_list) 
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

class ConnectNodePublishersIF:

    msg_if = None
    ready = False
    pubs_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                pubs_dict = None,
                log_name = None,
                log_class_name = True,
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
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
            
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_debug("Starting Node Pubs IF Initialization Processes", log_name_list = self.log_name_list)
        ##############################   
        ##############################  
        # Initialize Publishers System
        self.pubs_dict = pubs_dict
        if self.pubs_dict is None:
            self.pubs_dict = dict()
        self._initialize_pubs()

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete", log_name_list = self.log_name_list)
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_debug("Ready", log_name_list = self.log_name_list)
        return self.ready

        

    def get_pubs(self):
        return list(self.pubs_dict.keys())

    def has_subscribers_check(self,pub_name):
        has_subs = False
        if pub_name in self.pubs_dict.keys() and not nepi_sdk.is_shutdown():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                has_subs = pub_dict['pub'].get_num_connections() > 0
                #self.msg_if.pub_warn("Pub has subscribers: " + pub_dict['namespace'] + "/" + pub_dict['topic'] + " " + str(has_subs), log_name_list = self.log_name_list)
        return has_subs

    def publish_pub(self,pub_name,pub_msg):
        success = False
        if pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                if pub_dict['pub'] is not None and not nepi_sdk.is_shutdown():
                    try:
                        nepi_sdk.publish_pub(pub_dict['pub'], pub_msg, log_name_list = self.log_name_list)
                        success = True
                    except Exception as e:
                        namespace =  pub_dict['namespace']
                        self.msg_if.pub_warn("Failed to publish msg: " + pub_name + \
                            " " + str(namespace)  + " " + str(pub_msg) + str(e), throttle_s = 5.0, log_name_list = self.log_name_list)   
        return success
                    
    def register_pub(self,pub_name, pub_dict):
        self.pubs_dict[pub_name] = pub_dict
        self._initialize_pubs()

    def unregister_pub(self,pub_name):
        self._unregister_pub(pub_name)

    def unregister_pubs(self):
        for pub_name in self.pubs_dict.keys():
            self._unregister_pub(pub_name)

    def add_pubs(self,pubs_dict):
        self.pubs_dict.update(pubs_dict)
        self.initialize_pubs()

    ###############################
    # Class Private Methods
    ###############################
    def _initialize_pubs(self):
        for pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' not in pub_dict.keys():
                if 'topic' in pub_dict.keys() and 'msg' in pub_dict.keys() and not nepi_sdk.is_shutdown():
                    pub_namespace = nepi_sdk.create_namespace(pub_dict['namespace'] ,pub_dict['topic'])
                    self.msg_if.pub_debug("Creating pub for: " + pub_name + " with namespace: " + pub_namespace , log_name_list = self.log_name_list) 
                    pub = None
                    if 'qsize' not in pub_dict.keys():
                        self.pubs_dict[pub_name]['qsize'] = 1
                        pub_dict['qsize'] = 1
                    if 'latch' not in pub_dict.keys():
                        self.pubs_dict[pub_name]['latch'] = False
                        pub_dict['latch'] = False
                    try:
                        pub = nepi_sdk.create_publisher(pub_namespace, pub_dict['msg'], queue_size = pub_dict['qsize'],  latch = pub_dict['latch'])
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to create publisher: " + pub_name + " " + str(e), log_name_list = self.log_name_list) 
                    self.pubs_dict[pub_name]['pub'] = pub


    def _unregister_pub(self, pub_name):
        purge = False
        if pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            purge = True
            if 'pub' in pub_dict.keys() and not nepi_sdk.is_shutdown():
                try:
                    self.pubs_dict[pub_name]['pub'].unregister()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister pub: " + pub_name + " " + str(e), log_name_list = self.log_name_list) 
        if purge == True:
            del self.pubs_dict[pub_name]


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
class ConnectNodeSubscribersIF:

    msg_if = None
    ready = False
    subs_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                subs_dict = None,
                log_name = None,
                log_class_name = True,
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
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_debug("Starting Node Subs IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################   

        self.subs_dict = subs_dict
        if self.subs_dict is None:
            self.subs_dict = dict()
        self._initialize_subs()


        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete", log_name_list = self.log_name_list)
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_debug("Ready", log_name_list = self.log_name_list)
        return self.ready

        
    def get_subs(self):
        return list(self.subs_dict.keys())


    def register_sub(self,sub_name, sub_dict):
        self.subs_dict[sub_name] = sub_dict
        self._initialize_subs()

    def unregister_sub(self,sub_name):
        self._unregister_sub(sub_name)

    def unregister_subs(self):
        for sub_name in self.subs_dict.keys():
            self._unregister_sub(sub_name)

    def add_subs(self,subs_dict):
        self.subs_dict.update(subs_dict)
        self._initialize_subs()
    ###############################
    # Class Private Methods
    ###############################
    def _initialize_subs(self):
        for sub_name in self.subs_dict.keys():
            sub_dict = self.subs_dict[sub_name]
            #self.msg_if.pub_warn("Will try to create sub for: " + sub_name, log_name_list = self.log_name_list)
            if 'sub' not in sub_dict.keys() and sub_dict['callback'] is not None and not nepi_sdk.is_shutdown():
                sub_namespace = nepi_sdk.create_namespace(sub_dict['namespace'],sub_dict['topic'])
                self.msg_if.pub_debug("Creating sub for: " + sub_name + " with namespace: " + sub_namespace, log_name_list = self.log_name_list)
                if 'callback_args' not in sub_dict.keys():
                    sub_dict['callback_args'] = ()
                if sub_dict['callback_args'] is None:
                    sub_dict['callback_args'] = ()
                try:
                    if len(sub_dict['callback_args']) == 0:
                        sub = nepi_sdk.create_subscriber(sub_namespace, sub_dict['msg'],sub_dict['callback'], queue_size = sub_dict['qsize'])
                    else:
                        sub = nepi_sdk.create_subscriber(sub_namespace, sub_dict['msg'],sub_dict['callback'], queue_size = sub_dict['qsize'], callback_args=sub_dict['callback_args'])
                    self.subs_dict[sub_name]['sub'] = sub
                    success = True
                    #self.msg_if.pub_warn("Created sub for: " + sub_name + " with namespace: " + sub_namespace, log_name_list = self.log_name_list)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to create subscriber: " + sub_name + " " + str(e), log_name_list = self.log_name_list)  
                    self.subs_dict[sub_name]['sub'] = None
            

    def _unregister_sub(self, sub_name):
        purge = False
        if sub_name in self.subs_dict.keys():
            sub_dict = self.subs_dict[sub_name]
            purge = True
            if 'sub' in sub_dict.keys() and not nepi_sdk.is_shutdown():
                try:
                    self.subs_dict[sub_name]['sub'].unregister()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister sub: " + sub_name + " " + str(e), log_name_list = self.log_name_list)
        if purge == True:
            del self.subs_dict[sub_name]



##################################################
### Node Class

# Configs Dict ####################
'''
EXAMPLE_CFGS_DICT = {
        'init_callback': None,
        'reset_callback': None,
        'factory_reset_callback': None,
        'init_configs': True,
        'namespace':  self.node_namespace
}


# Params Dict ####################
EXAMPLE_PARAMS_DICT = {
    'param1_name': {
        'namespace':  self.node_namespace,
        'factory_val': 100
    },
    'param2_name': {
        'namespace':  self.node_namespace,
        'factory_val': "Something"
    }
}

# Services Dict ####################
EXAMPLE_SRVS_DICT = {
    'service_name': {
        'namespace': '~',
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
        'namespace': '~',
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'latch': False
    }
}


# Subscribers Dict ####################
EXAMPLE_SUBS_DICT = {
    'sub_name': {
        'namespace': '~',
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'callback': EXAMPLE_SUB_CALLBACK,
        'callback_args': ()
    }
}


# Create Node Class ####################
EXAMPLE_NODE_IF = ConnectNodeClassIF(
                configs_dict = EXAMPLE_CFGS_DICT,
                services_dict = EXAMPLE_SRVS_DICT,
                pubs_dict = EXAMPLE_PUBS_DICT,
                subs_dict = EXAMPLE_SUBS_DICT,
                log_class_name = True
)
'''

class ConnectNodeClassIF:

    ready = False

    node_if = None
    msg_if = None

    services_if = None
    pubs_if = None
    subs_if = None

    #######################
    ### IF Initialization
    def __init__(self, 
                configs_dict = None,
                services_dict = None,
                pubs_dict = None,
                subs_dict = None,
                node_name = None,
                do_wait = False,
                log_name_list = [],
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
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_info("Starting Connect Node IF Initialization Processes", log_name_list = self.log_name_list)
  
        ##############################  
        # Create Sub Classes
        self.services_if = ConnectNodeServicesIF(services_dict = services_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)
        self.pubs_if = ConnectNodePublishersIF(pubs_dict = pubs_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)
        self.subs_if = ConnectNodeSubscribersIF(subs_dict = subs_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)

        ############################## 
        # Wait for ready
        if do_wait == True:
            if services_dict is not None:
                ready = self.services_if.wait_for_ready()
            if pubs_dict is not None:
                ready = self.pubs_if.wait_for_ready()
            if subs_dict is not None:
                ready = self.subs_if.wait_for_ready()
        else:
            nepi_sdk.sleep(0.1)

        ###############################
        self.ready = True
        self.msg_if.pub_info("Node IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_debug("Waiting for Ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_debug("Wait for Ready Timed Out", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_debug("ready", log_name_list = self.log_name_list)
        return self.ready


    # Config Methods ####################
    def get_config_namespace(self):
        namespace = None
        if self.configs_if is not None:
            namespace = self.configs_if.get_namespace()
        return namespace

    def save_config(self, config_dict):
        success = False
        if self.configs_if is not None:
            success = self.configs_if.save()
        return success

    def reset_config(self, config_dict):
        success = False
        if self.configs_if is not None:
            success = self.configs_if.reset()
        return success

    def factory_reset_config(self, config_dict):
        success = False
        if self.configs_if is not None:
            success = self.configs_if.factory_reset()
        return success


    # Service Methods ####################
    def get_services(self):
        srvs_names = None
        if self.services_if is not None:
            srvs_names = self.services_if.get_services()
        return srvs_names


    def create_request_msg(self,service_name):
        req = None
        if self.services_if is not None:
            req = self.services_if.create_request_msg(service_name)
        return req

    def create_response_msg(self,service_name):
        msg = None
        if self.services_if is not None:
            msg = self.services_if.create_response_msg(service_name)
        return msg

    def call_service(self,service_name,request, verbose = True):
        resp = None
        if self.services_if is not None:
            resp = self.services_if.call_service(service_name,request, verbose = verbose)
        return resp

    def register_service(self,service_name, service_dict):
        if self.services_if is not None:
            self.services_if.register_service(service_name, service_dict)


    def unregister_service(self,service_name):
        if self.services_if is not None:
            self.services_if.unregister_service(service_name)

    def unregister_services(self):
        if self.services_if is not None:
            self.services_if.unregister_services()


    # Publisher Methods ####################
    def get_pubs(self):
        pubs = None
        if self.pubs_if is not None:
            pubs = self.pubs_if.get_pubs()
        return pubs



    def publish_pub(self,pub_name, pub_msg):
        success = False
        if self.pubs_if is not None and not nepi_sdk.is_shutdown():
            succes = self.pubs_if.publish_pub(pub_name, pub_msg)   
        return success


    def register_pub(self,pub_name, pub_dict):
        if self.services_if is not None:
            self.services_if.register_pub(pub_name, pub_dict)


    def unregister_pub(self,pub_name):
        if self.services_if is not None:
            self.services_if.unregister_pub(pub_name)

    def unregister_pubs(self):
        if self.services_if is not None:
            self.services_if.unregister_pubs()
                    
    # Subsciber Methods ####################
    def get_subs(self):
        subs = None
        if self.subs_if is not None:
            subs = self.subs_if.get_subs()
        return subs


    def register_sub(self,sub_name, sub_dict):
        if self.services_if is not None:
            self.services_if.register_sub(sub_name, sub_dict)


    def unregister_sub(self,sub_name):
        if self.services_if is not None:
            self.services_if.unregister_sub(sub_name)

    def unregister_subs(self):
        if self.services_if is not None:
            self.services_if.unregister_subs()

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



