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

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_ros_interfaces.msg import Reset

from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyRequest as EmptySrvRequest
from std_srvs.srv import EmptyResponse as EmptySrvResponse

# from nepi_ros_interfaces.msg import
from nepi_ros_interfaces.srv import FileReset, FileResetRequest, FileResetResponse

from nepi_api.messages_if import MsgIF


##################################################
### Node Config Class

'''
EXAMPLE_CFGS_DICT = {
        'init_callback': None,
        'reset_callback': None,
        'factory_reset_callback': None,
        'software_reset_callback': None,
        'hardware_reset_callback': None,
        'init_configs': True,
        'namespace':  self.node_namespace
}
'''

class NodeConfigsIF:

    msg_if = None
    ready = False
    configs_dict = None
    namespace = '~'

    reset_sub = None
    factory_reset_service = None
    request_msg = FileResetRequest()
    response_msg = FileResetResponse()


    initCb = None
    sysResetCb = None
    resetCb = None
    factoryResetCb = None

    ### IF Initialization
    def __init__(self, 
                configs_dict,
                log_name = None,
                log_class_name = True):
        #################################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        if log_name is None:
            log_name = ''
        elif log_class_name == True:
            log_name = log_name + ": "
        if log_class_name == True:
          log_name = log_name + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################    

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
        else:
            namespace = namespace
        self.namespace = nepi_ros.get_full_namespace(namespace)


        # Create reset serivces
        self.request_msg.node_name = self.namespace
        self.reset_service = nepi_ros.connect_service('~user_reset', FileReset)
        self.factory_reset_service = nepi_ros.connect_service('~factory_reset', FileReset)

        self.reset_service = nepi_ros.connect_service('user_reset', FileReset)
        self.factory_reset_service = nepi_ros.connect_service('factory_reset', FileReset)

        time.sleep(1)

        self.save_params_pub = nepi_ros.create_publisher('store_params', String, queue_size=1)

        self.msg_if.pub_info("Loading saved config data")
        self._resetCb('user_reset')


        # Subscribe to save config for node namespace
        nepi_ros.create_subscriber('~save_config', Empty, self._saveCb)
        nepi_ros.create_subscriber('~init_config', Empty, self._initCb)
        nepi_ros.create_subscriber('~reset_config', Empty, self._resetCb)
        nepi_ros.create_subscriber('~factory_reset_config', Empty, self._factoryResetCb)
        nepi_ros.create_subscriber('~system_reset', Reset, self._systemResetCb)

        # Global Topic Subscribers
        nepi_ros.create_subscriber('save_config', Empty, self._saveCb)
        nepi_ros.create_subscriber('init_config', Empty, self._initCb)
        nepi_ros.create_subscriber('reset_config', Empty, self._resetCb)
        nepi_ros.create_subscriber('factory_reset_config', Empty, self._factoryResetCb)
        nepi_ros.create_subscriber('system_reset', Reset, self._systemResetCb)

        if 'init_configs' in configs_dict.keys():
            init_configs = configs_dict['init_configs']
            if init_configs == True:
                self.init_config(do_updates = False)

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        self.msg_if.pub_info("Waiting for Ready")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Wait for Ready Timed Out")
        else:
            self.msg_if.pub_info("Ready")
        return self.ready

    def init_config(self, do_updates = False):
        if (self.initCb):
            try:
                self.initCb(do_updates = do_updates) # Callback provided by the container class to set init values to current values, etc.
            except:
                self.initCb()

    def save_config(self):
        self.save_params_pub.publish(self.node_namespace)
        if self.initCb is not None and not nepi_ros.is_shutdown():
            self.initCb() # Callback provided by container class to update based on param server, etc.

    def reset_config(self):
        success = False
        success = nepi_ros.call_service(self.reset_service,self.request_msg)
        nepi_ros.sleep(1)
        if (self.resetCb and success == True) and not nepi_ros.is_shutdown():
            self.resetCb() # Callback provided by container class to update based on param server, etc.
        return success

    def factory_reset_config(self):
        success = False
        success = nepi_ros.call_service(self.factory_reset_service,self.request_msg)
        nepi_ros.sleep(1)
        if (self.factoryResetCb) and not nepi_ros.is_shutdown():
            self.factoryResetCb() # Callback provided by container class to update based on param server, etc.
        return success

    def software_reset_config(self):
        if self.softwareResetCb:
            self.softwareResetCb() # Callback provided by container class to update based on param server, etc.
        else:
            self.msg_if.pub_warn("Does not have software reset support")
        return success

    def hardware_reset_config(self):
        if self.hardwareResetCb:
            self.hardwareResetCb() # Callback provided by container class to update based on param server, etc.
        else:
            self.msg_if.pub_warn("Does not have hardware reset support")
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
        'factory_val': 100
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

    #######################
    ### IF Initialization
    def __init__(self, 
                params_dict = None,
                log_name = None,
                log_class_name = True
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################
        # Create Msg Class
        if log_name is None:
            log_name = ''
        elif log_class_name == True:
            log_name = log_name + ": " 
        if log_class_name == True:
          log_name = log_name + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
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
        self.msg_if.pub_info("IF Initialization Complete")
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        self.msg_if.pub_info("Waiting for Ready")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Wait for Ready Timed Out")
        else:
            self.msg_if.pub_info("Ready")
        return self.ready

    def load_params(self, file_path):
        self.nepi_ros.load_params_from_file(file_path,self.namespace)        

    def initialize_params(self):
        for param_name in self.params_dict.keys():
            param_dict = self.params_dict[param_name]
            if 'factory_val' in param_dict:
                factory_val = param_dict['factory_val']
                init_val = self.get_param(param_name)
                #self.msg_if.pub_warn("Got init param value: " + param_name + " : " + str(init_val))
                if init_val is None:
                    init_val = factory_val
                self.params_dict[param_name]['init_val'] = init_val
                self.set_param(param_name, init_val)

    def reset_params(self):
        for param_name in self.params_dict.keys() and not nepi_ros.is_shutdown():
            init_val = self.params_dict[param_name]['init_val']
            self.set_param(param_name, init_val)

    def factory_reset_params(self):
        for param_name in self.params_dict.keys() and not nepi_ros.is_shutdown():
            factory_val = self.params_dict[param_name]['factory_val']
            self.set_param(param_name, factory_val)

    def save_params(self, file_path):
        if not nepi_ros.is_shutdown():
            self.nepi_ros.save_params_to_file(file_path,self.namespace)       

    def has_param(self, param_name):
        namespace = self.get_param_namespace(param_name)
        return nepi_ros.has_param(namespace)

    def get_param(self, param_name):
        value = None
        namespace = self.get_param_namespace(param_name)
        if param_name in self.params_dict.keys() and not nepi_ros.is_shutdown():
            param_dict = self.params_dict[param_name]
            if 'init_val' in param_dict.keys():
                fallback = param_dict['init_val']
            else:
                fallback = param_dict['factory_val']

            if fallback is not None:       
                 value = nepi_ros.get_param(namespace,fallback)
        return value

    def set_param(self, param_name, value):
        if not nepi_ros.is_shutdown():
            namespace = self.get_param_namespace(param_name)
            nepi_ros.set_param(namespace,value)

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
        if param_name in self.params_dict.keys() and not nepi_ros.is_shutdown():
            param_dict = self.params_dict[param_name]
            namespace = nepi_ros.create_namespace(param_dict['namespace'],param_name)
        return namespace



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
                log_class_name = True
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()
        # Create Msg Class
        if log_name is None:
            log_name = ''
        elif log_class_name == True:
            log_name = log_name + ": " 
        if log_class_name == True:
          log_name = log_name + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
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
        self.msg_if.pub_info("IF Initialization Complete")
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        self.msg_if.pub_info("Waiting for Ready")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Wait for Ready Timed Out")
        else:
            self.msg_if.pub_info("Ready")
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
        for service_name in self.srvs_dict.keys():
            self._unregister_service(service_name)


    ###############################
    # Class Private Methods
    ###############################

    def _initialize_services(self):
        for service_name in self.srvs_dict.keys():
            #self.msg_if.pub_warn("Will try to create service for: " + service_name )
            srv_dict = self.srvs_dict[service_name]
            if 'service' not in srv_dict.keys() and srv_dict['callback'] is not None:
                srv_callback = None
                try:
                    srv_namespace = nepi_ros.create_namespace(srv_dict['namespace'],srv_dict['topic'])
                    srv_msg = srv_dict['srv']
                    srv_callback = srv_dict['callback']
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get service info from dict: " + service_name + " " + str(e))
                if srv_callback is not None and not nepi_ros.is_shutdown():
                    self.msg_if.pub_info("Created service for: " + service_name + " with namespace: " + str(srv_namespace))
                    service = None
                    try:
                        service = nepi_ros.create_service(srv_namespace, srv_msg, srv_callback)   
                        self.srvs_dict[service_name]['service'] = service
                        self.msg_if.pub_info("Created service for: " + service_name + " with namespace: " + str(srv_namespace))                 
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to get service connection: " + service_name + " " + str(e))  
                    

    def _unregister_service(self, service_name):
        purge = False
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            purge = True
            if 'service' in srv_dict.keys() and not nepi_ros.is_shutdown():
                try:
                    self.srvs_dict[service_name]['service'].shutdown()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister service: " + service_name + " " + str(e))
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
                do_wait = True
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()
        # Create Msg Class
        if log_name is None:
            log_name = ''
        elif log_class_name == True:
            log_name = log_name + ": " 
        if log_class_name == True:
          log_name = log_name + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting Node Class IF Initialization Processes")
        ##############################   

        self.do_wait = do_wait
        ##############################  
        # Initialize Publishers System
        self.pubs_dict = pubs_dict
        if self.pubs_dict is None:
            self.pubs_dict = dict()
        self._initialize_pubs()

        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        self.msg_if.pub_info("Waiting for Ready")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Wait for Ready Timed Out")
        else:
            self.msg_if.pub_info("Ready")
        return self.ready

        

    def get_pubs(self):
        return list(self.pubs_dict.keys())

    def has_subscribers_check(self,pub_name):
        has_subs = False
        if pub_name in self.pubs_dict.keys() and not nepi_ros.is_shutdown():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                has_subs = pub_dict['pub'].get_num_connections() > 0
                #self.msg_if.pub_warn("Pub has subscribers: " + pub_dict['namespace'] + "/" + pub_dict['topic'] + " " + str(has_subs))
        return has_subs

    def publish_pub(self,pub_name,pub_msg):
        success = False
        if pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                if pub_dict['pub'] is not None and not nepi_ros.is_shutdown():
                    try:
                        pub_dict['pub'].publish(pub_msg)
                        success = True
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to publish msg: " + pub_name + " " + str(e))  
        return success
                    
    def register_pub(self,pub_name, pub_dict):
        self.pubs_dict[pub_name] = pub_dict
        self._initialize_pubs()

    def unregister_pub(self,pub_name):
        self._unregister_pub(pub_name)

    def unregister_pubs(self):
        for pub_name in self.pubs_dict.keys():
            self._unregister_pub(pub_name)



    ###############################
    # Class Private Methods
    ###############################
    def _initialize_pubs(self):
        for pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' not in pub_dict.keys():
                if 'topic' in pub_dict.keys() and 'msg' in pub_dict.keys() and not nepi_ros.is_shutdown():
                    pub_namespace = nepi_ros.create_namespace(pub_dict['namespace'] ,pub_dict['topic'])
                    self.msg_if.pub_info("Creating pub for: " + pub_name + " with namespace: " + pub_namespace )
                    pub = None
                    try:
                        pub = nepi_ros.create_publisher(pub_namespace, pub_dict['msg'], queue_size = pub_dict['qsize'],  latch = pub_dict['latch'])
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to create publisher: " + pub_name + " " + str(e))  
                    self.pubs_dict[pub_name]['pub'] = pub


    def _unregister_pub(self, pub_name):
        purge = False
        if pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            purge = True
            if 'pub' in pub_dict.keys() and not nepi_ros.is_shutdown():
                try:
                    self.pubs_dict[pub_name]['pub'].unregister()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister pub: " + pub_name + " " + str(e))
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
                do_wait = True
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()
        ############################## 
        # Create Msg Class
        if log_name is None:
            log_name = ''
        elif log_class_name == True:
            log_name = log_name + ": " 
        if log_class_name == True:
          log_name = log_name + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################   
        self.do_wait = do_wait


        self.subs_dict = subs_dict
        if self.subs_dict is None:
            self.subs_dict = dict()
        self._initialize_subs()


        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        self.msg_if.pub_info("Waiting for Ready")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Wait for Ready Timed Out")
        else:
            self.msg_if.pub_info("Ready")
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

    ###############################
    # Class Private Methods
    ###############################
    def _initialize_subs(self):
        for sub_name in self.subs_dict.keys():
            sub_dict = self.subs_dict[sub_name]
            #self.msg_if.pub_warn("Will try to create sub for: " + sub_name )
            if 'sub' not in sub_dict.keys() and sub_dict['callback'] is not None and not nepi_ros.is_shutdown():
                sub_namespace = nepi_ros.create_namespace(sub_dict['namespace'],sub_dict['topic'])
                self.msg_if.pub_info("Creating sub for: " + sub_name + " with namespace: " + sub_namespace)
                if 'callback_args' not in sub_dict.keys():
                    sub_dict['callback_args'] = ()
                if sub_dict['callback_args'] is None:
                    sub_dict['callback_args'] = ()
                try:
                    if len(sub_dict['callback_args']) == 0:
                        sub = nepi_ros.create_subscriber(sub_namespace, sub_dict['msg'],sub_dict['callback'], queue_size = sub_dict['qsize'])
                    else:
                        sub = nepi_ros.create_subscriber(sub_namespace, sub_dict['msg'],sub_dict['callback'], queue_size = sub_dict['qsize'], callback_args=sub_dict['callback_args'])
                    self.subs_dict[sub_name]['sub'] = sub
                    success = True
                    #self.msg_if.pub_warn("Created sub for: " + sub_name + " with namespace: " + sub_namespace)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to create subscriber: " + sub_name + " " + str(e))  
                    self.subs_dict[sub_name]['sub'] = None
            

    def _unregister_sub(self, sub_name):
        purge = False
        if sub_name in self.subs_dict.keys():
            sub_dict = self.subs_dict[sub_name]
            purge = True
            if 'sub' in sub_dict.keys() and not nepi_ros.is_shutdown():
                try:
                    self.subs_dict[sub_name]['sub'].unregister()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister sub: " + sub_name + " " + str(e))
        if purge == True:
            del self.subs_dict[sub_name]


##################################################
### Node Class

# Configs Config Dict ####################
'''
EXAMPLE_CFGS_DICT = {
        'init_callback': None,
        'reset_callback': None,
        'factory_reset_callback': None,
        'init_configs': True,
        'namespace':  self.node_namespace
}


# Params Config Dict ####################
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


# Services Config Dict ####################
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


# Publishers Config Dict ####################
EXAMPLE_PUBS_DICT = {
    'pub_name': {
        'namespace':  self.node_namespace,
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'latch': False
    }
}


# Subscribers Config Dict ####################
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
                configs_dict = EXAMPLE_CFGS_DICT,
                params_dict = EXAMPLE_PARAMS_DICT,
                services_dict = EXAMPLE_SRVS_DICT,
                pubs_dict = EXAMPLE_PUBS_DICT,
                subs_dict = EXAMPLE_SUBS_DICT,
                log_class_name = True
)
'''

class NodeClassIF:

    ready = False

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
                configs_dict = None,
                params_dict = None,
                services_dict = None,
                pubs_dict = None,
                subs_dict = None,
                log_name = None,
                log_class_name = False,
                node_name = None,
                do_wait = True
                ):
        ####  IF INIT SETUP ####
        if node_name is not None:
            nepi_ros.init_node(name = node_name) # Can be overwitten by luanch command
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        if log_name is None:
            log_name = ''
        elif log_class_name == True:
            log_name = log_name + ": " 
        if log_class_name == True:
          log_name = log_name + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting Node Class IF Initialization Processes")
        ##############################   
        self.do_wait = do_wait


        ##############################  
        # Create Params Class
        if params_dict is not None:
            self.msg_if.pub_info("Starting Node Params IF Initialization Processes")
            self.params_if = NodeParamsIF(params_dict = params_dict, log_name = log_name)


        ##############################  
        # Create Config Class After Params
        if configs_dict is not None:
            self.msg_if.pub_info("Starting Node Configs IF Initialization Processes")
            # Need to inject our own config callback functions that call the params_if functions first
            self.configs_dict = configs_dict
            configs_dict = {
                    'init_callback': self._initConfigCb,
                    'reset_callback': self._resetConfigCb,
                    'factory_reset_callback': self._factoryResetConfigCb,
                    'init_configs': True,
                    'namespace': self.node_namespace
            }
            self.configs_if = NodeConfigsIF(configs_dict = configs_dict, log_name = log_name)


        ##############################  
        # Create Services Class
        if services_dict is not None:
            self.msg_if.pub_info("Starting Node Services IF Initialization Processes")
            self.services_if = NodeServicesIF(services_dict = services_dict, log_name = log_name)


        ##############################  
        # Create Publisers Class
        if pubs_dict is not None:
            self.msg_if.pub_info("Starting Node Publishers IF Initialization Processes")
            self.pubs_if = NodePublishersIF(pubs_dict = pubs_dict, log_name = log_name, do_wait = self.do_wait)


        ##############################  
        # Create Subscribers Class
        if subs_dict is not None:
            self.msg_if.pub_info("Starting Node Subscribers IF Initialization Processes")
            self.subs_if = NodeSubscribersIF(subs_dict = subs_dict, log_name = log_name)

            
        ############################## 
        # Wait for ready
        if params_dict is not None:
            ready = self.params_if.wait_for_ready()
        if configs_dict is not None:
            ready = self.configs_if.wait_for_ready()
        if services_dict is not None:
            ready = self.services_if.wait_for_ready()
        if pubs_dict is not None:
            ready = self.pubs_if.wait_for_ready()
        if subs_dict is not None:
            ready = self.subs_if.wait_for_ready()



        ##############################  
        # Complete Initialization Process
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ##############################  


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        self.msg_if.pub_info("Waiting for Ready")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Wait for Ready Timed Out")
        else:
            self.msg_if.pub_info("Ready")
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
        if self.services_if is not None:
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
        if self.pubs_if is not None and not nepi_ros.is_shutdown():
            self.pubs_if.publish_pub(pub_name, pub_msg)   
            
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

    def _initConfigCb(self):
        self.initialize_params()
        if self.configs_dict is not None:
            if self.configs_dict['init_callback'] is not None:
                self.configs_dict['init_callback']()
            

    def _resetConfigCb(self,msg):
        self.reset_params()
        if self.configs_dict is not None:
            if self.configs_dict['reset_callback'] is not None:
                self.configs_dict['reset_callback']()

    def _factoryResetConfigCb(self,msg):
        self.factory_reset_params()
        if self.configs_dict is not None:
            if self.configs_dict['factory_reset_callback'] is not None:
                self.configs_dict['factory_reset_callback']()