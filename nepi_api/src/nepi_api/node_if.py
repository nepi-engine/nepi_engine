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

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_system

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_interfaces.msg import Reset

from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyRequest as EmptySrvRequest
from std_srvs.srv import EmptyResponse as EmptySrvResponse

# from nepi_interfaces.msg import
from nepi_interfaces.srv import ParamsReset, ParamsResetRequest, ParamsResetResponse

from nepi_api.messages_if import MsgIF


##################################################
### Node Config Class

'''
EXAMPLE_CONFIGS_DICT = {
        'init_callback': None,
        'reset_callback': None,
        'factory_reset_callback': None,
        'software_reset_callback': None,
        'hardware_reset_callback': None,
        'init_configs': True
}
'''


class NodeConfigsIF:

    msg_if = None
    ready = False
    configs_dict = None
    namespace = '~'

    reset_sub = None
    request_msg = ParamsResetRequest()
    response_msg = ParamsResetResponse()


    initCb = None
    sysResetCb = None
    resetCb = None
    factoryResetCb = None

    ### IF Initialization
    def __init__(self, 
                configs_dict,
                wait_cfg_mgr = True,
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
        self.msg_if.pub_debug("Starting Node Configs IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################    
        self.msg_if.pub_debug("Got Config Dict: " + str(configs_dict), log_name_list = self.log_name_list)

        if 'init_callback' in configs_dict.keys():  
            self.initCb = configs_dict['init_callback']

        if 'reset_callback' in configs_dict.keys():
            self.resetCb = configs_dict['reset_callback']

        if 'factory_reset_callback' in configs_dict.keys():
            self.factoryResetCb = configs_dict['factory_reset_callback']

        if 'software_reset_callback' in configs_dict.keys(): 
            self.softwareResetCb = configs_dict['software_reset_callback']

        if 'hardware_reset_callback' in configs_dict.keys(): 
            self.hardwareResetCb = configs_dict['hardware_reset_callback']
        
        namespace = configs_dict['namespace']
        if namespace is None:
            namespace = self.node_namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        #self.msg_if.pub_warn("Using Config namespace: " + str(self.namespace), log_name_list = self.log_name_list)
        #self.msg_if.pub_warn("Using Base namespace: " + str(self.base_namespace), log_name_list = self.log_name_list)

        # Create reset serivces
        self.request_msg.namespace = self.namespace
        ns = nepi_sdk.create_namespace(self.namespace,'user_reset')
        self.reset_service = nepi_sdk.connect_service(ns, ParamsReset)
        ns = nepi_sdk.create_namespace(self.namespace,'factory_reset')
        self.factory_reset_service = nepi_sdk.connect_service(ns, ParamsReset)

        ns = nepi_sdk.create_namespace(self.base_namespace,'user_reset')
        self.reset_service = nepi_sdk.connect_service(ns, ParamsReset)
        ns = nepi_sdk.create_namespace(self.base_namespace,'factory_reset')
        self.factory_reset_service = nepi_sdk.connect_service(ns, ParamsReset)
        ns = nepi_sdk.create_namespace(self.base_namespace,'store_params')
        self.save_params_pub = nepi_sdk.create_publisher(ns, String, queue_size=1)

        if wait_cfg_mgr == True:
            self.msg_if.pub_debug("Waiting for Config Mgr", log_name_list = self.log_name_list)
            config_folders = nepi_system.get_config_folders()


        # Subscribe to save config for node namespace
        nepi_sdk.create_subscriber(self.namespace + '/save_config', Empty, self._saveCb)
        nepi_sdk.create_subscriber(self.namespace + '/init_config', Empty, self._initCb)
        nepi_sdk.create_subscriber(self.namespace + '/reset_config', Empty, self._resetCb)
        nepi_sdk.create_subscriber(self.namespace + '/factory_reset_config', Empty, self._factoryResetCb)
        nepi_sdk.create_subscriber(self.namespace + '/system_reset', Reset, self._systemResetCb)

        # Global Topic Subscribers
        nepi_sdk.create_subscriber('save_config', Empty, self._saveCb)
        nepi_sdk.create_subscriber('init_config', Empty, self._initCb)
        nepi_sdk.create_subscriber('reset_config', Empty, self._resetCb)
        nepi_sdk.create_subscriber('factory_reset_config', Empty, self._factoryResetCb)
        nepi_sdk.create_subscriber('system_reset', Reset, self._systemResetCb)

        self.msg_if.pub_debug("Resetting Params", log_name_list = self.log_name_list)
        self.reset_config()
        nepi_sdk.wait()

        params_dict = nepi_sdk.get_param(self.namespace, dict())
        self.msg_if.pub_warn("##### Got Reset Params: " + str(params_dict), log_name_list = self.log_name_list)
        
        if 'init_configs' in configs_dict.keys():
            
            init_configs = configs_dict['init_configs']
            if init_configs == True:
                self.msg_if.pub_debug("Calling parent init function", log_name_list = self.log_name_list)
                self.init_config(do_updates = False)

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_debug("Node Configs IF Initialization Complete", log_name_list = self.log_name_list)
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

    def init_config(self, do_updates = False):
        if (self.initCb):
            try:
                self.initCb(do_updates = do_updates) # Callback provided by the container class to set init values to current values, etc.
            except:
                self.initCb()

    def save_config(self):
        self.msg_if.pub_debug("Saving Config: " + str(self.namespace))
        self.save_params_pub.publish(self.namespace)
        #if self.initCb is not None and not nepi_sdk.is_shutdown():
        #    self.initCb() # Callback provided by container class to update based on param server, etc.

    def reset_config(self):
        self.msg_if.pub_warn("Reseting Config: " + str(self.request_msg))
        success = False
        success = nepi_sdk.call_service(self.reset_service,self.request_msg)
        nepi_sdk.sleep(1)
        #if (self.resetCb and success == True) and not nepi_sdk.is_shutdown():
        #    self.resetCb() # Callback provided by container class to update based on param server, etc.
        return success

    def factory_reset_config(self):
        self.msg_if.pub_debug("Factory Reseting Config: " + str(self.request_msg))
        success = False
        success = nepi_sdk.call_service(self.factory_reset_service,self.request_msg)
        nepi_sdk.sleep(1)
        if (self.factoryResetCb) and not nepi_sdk.is_shutdown():
            self.factoryResetCb() # Callback provided by container class to update based on param server, etc.
        return success

    def software_reset_config(self):
        if self.softwareResetCb:
            self.softwareResetCb() # Callback provided by container class to update based on param server, etc.
        else:
            self.msg_if.pub_warn("Does not have software reset support", log_name_list = self.log_name_list, throttle_s = 5.0)  
        return success

    def hardware_reset_config(self):
        if self.hardwareResetCb:
            self.hardwareResetCb() # Callback provided by container class to update based on param server, etc.
        else:
            self.msg_if.pub_warn("Does not have hardware reset support", log_name_list = self.log_name_list, throttle_s = 5.0)  
        return success


    ###############################
    # Class Private Methods
    ###############################

    def _saveCb(self,msg):
        self.save_config()

    def _initCb(self,msg):
        self.init_config(do_updates = True)

    def _resetCb(self,msg):
        self.reset_config()

    def _factoryResetCb(self,msg):
        self.factory_reset_config()

    def _systemResetCb(self,msg):
        reset_type = msg.reset_type
        if reset_type == 0:
            self.reset_config()
        elif reset_type == 1:
            self.factory_reset_config()
        elif reset_type == 2:
            self.software_reset_config()
        elif reset_type == 3:
            self.hardware_reset_config()




##################################################
### Node Params Class
'''
EXAMPLE_PARAMS_DICT = {
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
'''


class NodeParamsIF:

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
        self.msg_if.pub_debug("Starting Node Params IF Initialization Processes", log_name_list = self.log_name_list)
        ##############################   

        ##############################  
        # Initialize Params System

        self.params_dict = params_dict
        if self.params_dict is None:
            self.params_dict = dict()
        self.initialize_params()


        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
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

    def load_params(self, file_path):
        self.nepi_sdk.load_params_from_file(file_path,self.namespace)        

    def initialize_params(self):
        ns_params_dict = dict()
        for param_name in self.params_dict.keys():
            namespace = self.params_dict[param_name]['namespace']
            if namespace not in ns_params_dict.keys():
                ns_params_dict[namespace] = nepi_sdk.get_params(namespace)
        for param_name in self.params_dict.keys():
            param_dict = self.params_dict[param_name]
            namespace = param_dict['namespace']
            init_val = None
            if namespace in ns_params_dict.keys():
                ns_param_dict = ns_params_dict[namespace]
                if param_name in ns_param_dict.keys():
                    init_val = ns_param_dict[param_name]
            if init_val is None:
                init_val = self.get_param(param_name)
                self.set_param(param_name, init_val)
            self.params_dict[param_name]['init_val'] = init_val
            


    def reset_params(self):
        for param_name in self.params_dict.keys():
            init_val = self.params_dict[param_name]['init_val']
            self.set_param(param_name, init_val)

    def factory_reset_params(self):
        for param_name in self.params_dict.keys():
            factory_val = self.params_dict[param_name]['factory_val']
            self.set_param(param_name, factory_val)

    def save_params(self, file_path):
        if not nepi_sdk.is_shutdown():
            self.nepi_sdk.save_params_to_file(file_path,self.namespace)       

    def has_param(self, param_name):
        namespace = self.get_param_namespace(param_name)
        return nepi_sdk.has_param(namespace)

    def get_param(self, param_name):
        value = None
        if param_name in self.params_dict.keys():
            param_dict = self.params_dict[param_name]
            namespace = self.get_param_namespace(param_name)
            if 'init_val' in param_dict.keys():
                fallback = param_dict['init_val']
            else:
                fallback = param_dict['factory_val']

            if fallback is not None:  
                if namespace in self.params_ns_dict.keys():
                    value = self.params_ns_dict[namespace]
                else:
                    value = fallback
        return value

    def set_param(self, param_name, value):
        if not nepi_sdk.is_shutdown():
            namespace = self.get_param_namespace(param_name)
            nepi_sdk.set_param(namespace,value)
            self.params_ns_dict[namespace] = value

    def reset_param(self, param_name):
        if param_name in self.params_dict.keys():
            init_val = self.params_dict[param_name]['init_val']
            self.set_param(param_name, init_val)

    def factory_reset_param(self, param_name):
        if param_name in self.params_dict.keys():
            factory_val = self.params_dict[param_name]['factory_val']
            self.set_param(param_name, factory_val)

    def get_params(self):
        return list(self.params_dict.keys())


    def get_param_namespace(self,param_name):
        namespace = ""
        if param_name in self.params_dict.keys() and not nepi_sdk.is_shutdown():
            param_dict = self.params_dict[param_name]
            namespace = nepi_sdk.create_namespace(param_dict['namespace'],param_name)
        return namespace

    def add_params(self,params_dict):
        self.params_dict.update(params_dict)
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
EXAMPLE_SRVS_DICT = {
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
        self.msg_if.pub_debug("Starting Node Services IF Initialization Processes", log_name_list = self.log_name_list)
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
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
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
        self.srvs_dict[service_name] = service_dict
        self._initialize_services()

    def unregister_service(self,service_name):
        self._unregister_service(service_name)

    def unregister_services(self):
        service_names = list(self.srvs_dict.keys())
        for service_name in service_names:
            self._unregister_service(service_name)

    def add_services(self,services_dict):
        self.srvs_dict.update(services_dict)
        self._initialize_services()
    ###############################
    # Class Private Methods
    ###############################

    def _initialize_services(self):
        for service_name in self.srvs_dict.keys():
            self.msg_if.pub_debug("Will try to create service for: " + service_name )
            srv_dict = self.srvs_dict[service_name]
            if 'service' not in srv_dict.keys() and srv_dict['callback'] is not None:
                srv_callback = None
                try:
                    srv_namespace = nepi_sdk.create_namespace(srv_dict['namespace'],srv_dict['topic'])
                    srv_msg = srv_dict['srv']
                    srv_callback = srv_dict['callback']
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get service info from dict: " + service_name + " " + str(e), throttle_s = 5.0, log_name_list = self.log_name_list) 
                if srv_callback is not None and not nepi_sdk.is_shutdown():
                    self.msg_if.pub_debug("Created service for: " + service_name + " with namespace: " + str(srv_namespace), log_name_list = self.log_name_list)
                    service = None
                    try:
                        service = nepi_sdk.create_service(srv_namespace, srv_msg, srv_callback, log_name_list = self.log_name_list)   
                        self.srvs_dict[service_name]['service'] = service
                        self.srvs_dict[service_name]['namespace'] = srv_namespace
                        self.msg_if.pub_debug("Created service for: " + service_name + " with namespace: " + str(srv_namespace), \
                                        throttle_s = 5.0, log_name_list = self.log_name_list)                 
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to get service connection: " + service_name + " " + str(e), log_name_list = self.log_name_list)  
                    

    def _unregister_service(self, service_name):
        purge = False
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            purge = True
            if 'service' in srv_dict.keys() and not nepi_sdk.is_shutdown():
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

class NodePublishersIF:

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
        self.msg_if.pub_debug("Initializing with pub dict: " + str(self.pubs_dict) )
        self._initialize_pubs()

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
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
                self.msg_if.pub_debug("Pub has subscribers: " + pub_dict['namespace'] + "/" + pub_dict['topic'] + " " + str(has_subs), \
                                        throttle_s = 5.0, log_name_list = self.log_name_list) 
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

    def register_pubs(self,pubs_dict):
        for pub_name in pubs_dict.keys():
            pub_dict = pubs_dict[pub_name]
            self.pubs_dict[pub_name] = pub_dict
        self._initialize_pubs()

    def register_pub(self,pub_name, pub_dict):
        self.pubs_dict[pub_name] = pub_dict
        self._initialize_pubs()

    def unregister_pub(self,pub_name):
        self._unregister_pub(pub_name)

    def unregister_pubs(self):
        pub_names = list(self.pubs_dict.keys())
        for pub_name in pub_names:
            self._unregister_pub(pub_name)

    def add_pubs(self,pubs_dict):
        self.msg_if.pub_debug("Adding pubs dict: " + str(pubs_dict) , log_name_list = self.log_name_list) 
        self.pubs_dict.update(pubs_dict)
        #self.msg_if.pub_debug("Updated pubs dict: " + str(pubs_dict) , log_name_list = self.log_name_list) 
        self._initialize_pubs()

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
                    try:
                        pub = nepi_sdk.create_publisher(pub_namespace, pub_dict['msg'], queue_size = pub_dict['qsize'], \
                                latch = pub_dict['latch'], log_name_list = self.log_name_list)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to create publisher: " + pub_name + " " + str(e), log_name_list = self.log_name_list) 
                    self.pubs_dict[pub_name]['namespace'] = pub_namespace
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
class NodeSubscribersIF:

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
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
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
        sub_names = list(self.subs_dict.keys())
        for sub_name in sub_names:
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
            self.msg_if.pub_debug("Will try to create sub for: " + sub_name )
            if 'sub' not in sub_dict.keys() and sub_dict['callback'] is not None and not nepi_sdk.is_shutdown():
                sub_namespace = nepi_sdk.create_namespace(sub_dict['namespace'],sub_dict['topic'])
                self.msg_if.pub_debug("Creating sub for: " + sub_name + " with namespace: " + sub_namespace, log_name_list = self.log_name_list) 
                if 'callback_args' not in sub_dict.keys():
                    sub_dict['callback_args'] = ()
                if sub_dict['callback_args'] is None:
                    sub_dict['callback_args'] = ()
                try:
                    if len(sub_dict['callback_args']) == 0:
                        sub = nepi_sdk.create_subscriber(sub_namespace, sub_dict['msg'], sub_dict['callback'], queue_size = sub_dict['qsize'], \
                            log_name_list = self.log_name_list)
                    else:
                        sub = nepi_sdk.create_subscriber(sub_namespace, sub_dict['msg'],sub_dict['callback'], queue_size = sub_dict['qsize'], \
                             callback_args=sub_dict['callback_args'], log_name_list = self.log_name_list)
                    self.subs_dict[sub_name]['sub'] = sub
                    self.subs_dict[sub_name]['namespace'] = sub_namespace
                    success = True
                    self.msg_if.pub_debug("Created sub for: " + sub_name + " with namespace: " + sub_namespace, log_name_list = self.log_name_list) 
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
                    self.msg_if.pub_warn("Failed to get unregister sub: " + sub_name + " " + str(e), throttle_s = 5.0)  
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
EXAMPLE_PARAMS_DICT = {
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
EXAMPLE_SRVS_DICT = {
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
                params_dict = EXAMPLE_PARAMS_DICT,
                services_dict = EXAMPLE_SRVS_DICT,
                pubs_dict = EXAMPLE_PUBS_DICT,
                subs_dict = EXAMPLE_SUBS_DICT,
                log_class_name = True
)
'''

class NodeClassIF:

    ready = False
    node_if = None
    msg_if = None
    log_name = None

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
                wait_cfg_mgr = True,
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
        self.msg_if.pub_info("Starting Node IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################  
        # Create Sub Classes
           
        ##############################  
        # Create Config Class After Params
        if configs_dict is not None:
            # Need to inject our own config callback functions that call the params_if functions first
            self.configs_dict = configs_dict
            configs_dict = {
                    'init_callback': self._initConfigCb,
                    'reset_callback': self._resetConfigCb,
                    'factory_reset_callback': self._factoryResetConfigCb,
                    'init_configs': True,
                    'namespace': configs_dict['namespace']
            }
            self.configs_if = NodeConfigsIF(configs_dict = configs_dict, 
                                            wait_cfg_mgr = wait_cfg_mgr, 
                                            msg_if = self.msg_if, 
                                            log_name_list = self.log_name_list)
        nepi_sdk.sleep(1)

        self.params_if = NodeParamsIF(params_dict = params_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)
        self.services_if = NodeServicesIF(services_dict = services_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)
        self.pubs_if = NodePublishersIF(pubs_dict = pubs_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)
        self.subs_if = NodeSubscribersIF(subs_dict = subs_dict, msg_if = self.msg_if, log_name_list = self.log_name_list)
        
        nepi_sdk.sleep(0.1)
      
        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("Node IF Initialization Complete", log_name_list = self.log_name_list)
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

    # Config Methods ####################
    def save_config(self):
        if self.configs_if is not None:
            self.configs_if.save_config()

    def reset_config(self):
        if self.configs_if is not None:
            self.configs_if.reset_config()

    def factory_reset_config(self):
        if self.configs_if is not None:
            self.configs_if.factory_reset_config()


    # Param Methods ####################
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


    def reset_params(self):
        if self.params_if is not None:
            self.params_if.reset_params()

    def factory_reset_params(self):
        if self.params_if is not None:
            self.params_if.factory_reset_params()


    def save_params(self):
        if self.params_if is not None:
            self.params_if.save_params()



    def has_param(self, param_name):
        exists = False
        if self.params_if is not None:
            exists = self.params_if.has_param(param_name)
        return exists

    def get_param(self, param_name):
        value = None
        if self.params_if is not None:
            value = self.params_if.get_param(param_name)

        return value

    def set_param(self, param_name, value):
        success = False
        if self.params_if is not None:
            self.params_if.set_param(param_name,value)
        return success

    def reset_param(self, param_name):
        success = False
        if self.params_if is not None:
            self.params_if.reset_param(param_name)
        return success

    def factory_reset_param(self, param_name):
        success = False
        if self.params_if is not None:
            self.params_if.reset_param(param_name)

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
            if self.configs_dict['init_callback'] is not None:
                self.configs_dict['init_callback'](do_updates = do_updates)
            

    def _resetConfigCb(self):
        self.reset_params()
        if self.configs_dict is not None:
            if self.configs_dict['reset_callback'] is not None:
                self.configs_dict['reset_callback']()

    def _factoryResetConfigCb(self):
        self.factory_reset_params()
        if self.configs_dict is not None:
            if self.configs_dict['factory_reset_callback'] is not None:
                self.configs_dict['factory_reset_callback']()