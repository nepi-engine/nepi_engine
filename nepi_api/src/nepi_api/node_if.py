#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import rospy
import time

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv

from nepi_ros_interfaces.msg import Reset
from nepi_ros_interfaces.srv import *

from nepi_api.sys_if_msg import MsgIF


##################################################
### Node Config Class


EXAMPLE_CONFIGS_CONFIG_DICT = {
        'init_callback': None,
        'reset_callback': None,
        'factory_reset_callback': None,
        'init_configs': True,
        'namespace': '~'
}



class NodeConfigsIF(object):

    ready = False
    namespace = None

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
        # Create Msg Class
        if log_name is None:
            log_name = ''
        if log_class_name == True:
          log_name = log_name + ": " + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################    
        self.initCb = configs_dict['init_callback']
        self.resetCb = configs_dict['reset_callback']
        self.factoryResetCb = configs_dict['factory_callback']
        
        namespace = configs_dict['namespace']
        if namespace is None:
            self.namespace = self.node_namespace
        else:
            self.namespace = namespace

        self.save_params_pub = rospy.Publisher('store_params', String, queue_size=1)

        self.msg_if.pub_info("Loading saved config data")
        self._resetCb('user_reset')

        # Subscribe to provided save config namespace
        save_topic = os.path.join(self.namespace,'save_config')
        reset_topic = os.path.join(self.namespace,'reset_config')
        factory_reset_topic = os.path.join(self.namespace,'factory_reset_config')
        rospy.Subscriber(save_topic, Empty, self._saveConfigCb)
        rospy.Subscriber(reset_topic, Reset, self.sendResetMsgCb)
        rospy.Subscriber(reset_topic, Reset, self.sendResetMsgCb)
        rospy.Subscriber(factory_reset_topic, Reset, self.sendResetMsgCb)

        # Global Topic Subscribers
        rospy.Subscriber('save_config', Empty, self._saveConfigCb)
        rospy.Subscriber('reset_config', Reset, self.sendResetMsgCb)
        rospy.Subscriber('factory_reset_config', Reset, self.sendResetMsgCb)

        nepi_ros.sleep(1)

        init_configs = configs_dict['init_configs']
        if init_configs == True:
            nepi_ros.sleep(1)
            self.init()

        ###############################
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

    def init(self):
        if (self.initCb):
            self.initCb() # Callback provided by the container class to set values to param server, etc.

    def save(self):
        self.init()
        # Save the current nodes params
        self.save_params_pub.publish(self.node_namespace)

    def saveConfig(self): 
        self.save()

    def reset(self):
        self._resetCb('user_reset')
        nepi_ros.sleep(1)
        if (self.resetCb and ret_val == True):
            self.resetCb() # Callback provided by container class to update based on param server, etc.
        return ret_val


    def factory_reset(self):
        self._resetCb('factory_reset')
        nepi_ros.sleep(1)
        if (self.factoryResetCb):
            self.factoryResetCb() # Callback provided by container class to update based on param server, etc.
        return ret_val

    ###############################
    # Class Private Methods
    ###############################




##################################################
### Node Params Class

EXAMPLE_PARAMS_DICT = {
    'param1_name': {
        'factory_val': 100
    },
    'param2_name': {
        'factory_val': "Something"
    }
}

EXAMPLE_PARAMS_CONFIG_DICT = {
        'namespace': '~',
        'params_dict': EXAMPLE_PARAMS_DICT
}



class NodeParamsIF(object):

    ready = False

    msg_if = None
    
    params_dict = dict()
    params_namespace = '~'

    initCb = None
    resetCb = None
    factoryResetCb = None

    #######################
    ### IF Initialization
    def __init__(self, 
                params_config_dict = None,
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
        if log_class_name == True:
          log_name = log_name + ": " + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        ##############################   

        ##############################  
        # Initialize Params System

        if params_config_dict['namespace'] is None:
            self.params_namespace = self.base_namespace
        else:
            self.params_namespace = params_config_dict['namespace']
        self.msg_if.pub_info("Using param namespace: " + self.params_namespace )

        self.initCb = params_config_dict['init_callback']
        self.resetCb = params_config_dict['reset_callback']
        self.factoryResetCb = params_config_dict['factory_reset_callback']

        # Initialize Params
        self.params_dict = params_config_dict['params_dict']
        if self.params_dict is None:
            self.params_dict = dict()
        self.initialize_params()
        ##############################   


        ###############################
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

        

    def initialize_params(self):
        for param_name in self.params_dict.keys():
            param_dict = self.params_dict[param_name]
            if 'factory_val' in param_dict:
                factory_val = param_dict['factory_val']
                init_val = self.get_param(param_name)
                if init_val is not None:
                    init_val = factory_val
                self.params_dict[param_name]['init_val'] = init_val
                self.set_param(param_name, init_val)

    def load_params(self, file_path):
        self.nepi_ros.load_params_from_file(file_path,self.namespace)

 

    def has_param(self, param_name):
        namespace = self.get_param_namespace(param_name)
        return rospy.has_param(namespace)

    def get_param(self, param_name):
        value = None
        namespace = self.get_param_namespace(param_name)
        if param_name in self.params_dict.keys():
            param_dict = self.params_dict[param_name]
            if 'init_val' in param_dict.keys():
                fallback = param_dict['init_val']
            else:
                try:
                    fallback = param_dict['factory_val']
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get factory val: " + param_name + " " + str(e))  
                    fallback = None
            if fallback is not None:       
                try:
                    value = rospy.get_param(namespace,fallback)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get param val: " + param_name + " " + str(e))  
        return value

    def set_param(self, param_name, value):
        namespace = self.get_param_namespace(param_name)
        rospy.set_param(namespace,value)


    def get_params(self):
        return list(self.params_dict.keys())


    def get_param_namespace(self,param_name):
        return nepi_ros.create_namespace(self.params_namespace,param_name)



    ###############################
    # Class Private Methods
    ###############################

    def _initCb(self):
        if self.initCb is not None:
            self.initCb()

    def _resetCb(self):
        for param_name in self.params_dict.keys():
            init_val = self.params_dict[param_name]['init_val']
            self.set_param(param_name, init_val)
        if self.resetCb is not None:
            self.resetCb()

    def _factoryResetCb(self):
        for param_name in self.params_dict.keys():
            factory_val = self.params_dict[param_name]['factory_val']
            self.set_param(param_name, factory_val)
        if self.factoryResetCb is not None:
            self.factoryResetCb()



##################################################
### Node Services Class

def EXAMPLE_CALLBACK_FUNCTION(request):
    response = EmptySrvResponse()
    return response


EXAMPLE_SRVS_DICT = {
    'service_name': {
        'topic': 'empty_query',
        'msg': EmptySrv,
        'callback': EXAMPLE_CALLBACK_FUNCTION
    }
}

EXAMPLE_SRVS_CONFIG_DICT = {
        'namespace': '~',
        'srvs_dict': EXAMPLE_SRVS_DICT
}

class NodeServicesIF(object):

    ready = False

    msg_if = None
    
    srvs_dict = dict()
    srvs_namespace = "~"

    #######################
    ### IF Initialization
    def __init__(self, 
                srvs_config_dict = None,
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
        if log_class_name == True:
          log_name = log_name + ": " + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        ##############################   

        ##############################  
        # Initialize Services System
        if srvs_config_dict['namespace'] is None:
            self.srvs_namespace = self.base_namespace
        else:
            self.srvs_namespace = srvs_config_dict['namespace']
        self.srvs_dict = srvs_config_dict['srvs_dict']
        if self.srvs_dict is None:
            self.srvs_dict = dict()
        self._initialize_srvs()

        ###############################
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

        
    def get_srvs(self):
        return list(self.srvs_dict.keys())

    ###############################
    # Class Private Methods
    ###############################

    def _initialize_srvs(self):
        for service_name in self.srvs_dict.keys():
            try:
                srv_dict = self.srvs_dict[service_name]
                srv_namespace = nepi_ros.create_namespace(self.srvs_namespace,srv_dict['topic'])
                srv_msg = srv_dict['msg']
                srv_callback = srv_dict['callback']
            except Exception as e:
                self.msg_if.pub_warn("Failed to get service info from dict: " + service_name + " " + str(e))
            if srv_callback is not None:
                self.msg_if.pub_info("Creating service for: " + service_name + " with namespace: " + str(srv_namespace))
                service = None
                try:
                    service = rospy.Service(srv_namespace, srv_msg, srv_callback)   
                    self.msg_if.pub_info("Created service for: " + service_name + " with namespace: " + str(srv_namespace))                 
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get service connection: " + service_name + " " + str(e))  
                self.srvs_dict[service_name]['service'] = service







##################################################
### Node Publishers Class


EXAMPLE_PUBS_DICT = {
    'pub_name': {
        'msg': EmptyMsg,
        'topic': 'set_empty',
        'qsize': 1,
        'latch': False
    }
}

EXAMPLE_PUBS_CONFIG_DICT = {
    'namespace': '~',
    'pubs_dict': EXAMPLE_PUBS_DICT
}


class NodePublishersIF(object):

    ready = False

    msg_if = None
    
    pubs_dict = dict()
    pubs_namespace = "~"

    #######################
    ### IF Initialization
    def __init__(self, 
                pubs_config_dict = None,
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
        if log_class_name == True:
          log_name = log_name + ": " + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting Node Class IF Initialization Processes")
        ##############################   

        self.do_wait = do_wait
        ##############################  
        # Initialize Publishers System
        if pubs_config_dict is not None:
            if pubs_config_dict['namespace'] is None:
                self.pubs_namespace = self.base_namespace
            else:
                self.pubs_namespace = pubs_config_dict['namespace']
            self.pubs_dict = pubs_config_dict['pubs_dict']
            if self.pubs_dict is None:
                self.pubs_dict = dict()
            self._initialize_pubs()
        ##############################   


        ###############################
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

        

    def get_pubs(self):
        return list(self.pubs_dict.keys())

    def publish(self,pub_name,pub_msg):
        success = False
        if pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                if pub_dict['pub'] is not None:
                    try:
                        pub_dict['pub'].publish(pub_msg)
                        success = True
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to publish msg: " + pub_name + " " + str(e))  
        return success
                    
    def unregister(self,pub_name):
        success = False
        if pub_name in self.pubs_dict.keys():
            exists = True
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                if pub_dict['pub'] is not None:
                    pub_dict['pub'].unregister()
                    time.sleep(1)
                    success = True
            if success == True:
                pub_dict['pub'] = None
        return success


    def register(self,pub_name):
        success = False
        if pub_name in self.pubs_dict.keys():
            exists = True
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                if pub_dict['pub'] is None:
                    self._initialize_pub(pub_name)
                    time.sleep(1)
                    success = True
            if success == True:
                pub_dict['pub'] = None
        return success


    ###############################
    # Class Private Methods
    ###############################
    def _initialize_pubs(self):
        for pub_name in self.pubs_dict.keys():
            self._initialize_pub(pub_name)

    def _initialize_pub(self,pub_name):
        if pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            if 'topic' in pub_dict.keys() and 'msg' in pub_dict.keys():
                pub_namespace = nepi_ros.create_namespace(self.pubs_namespace ,pub_dict['topic'])
                self.msg_if.pub_warn("Creating pub for: " + pub_name + " with namespace: " + pub_namespace )
                pub = None
                try:
                    pub = rospy.Publisher(pub_namespace, pub_dict['msg'], queue_size = pub_dict['qsize'],  latch = pub_dict['latch'])
                    if self.do_wait == True:
                        time.sleep(1)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to create publisher: " + pub_name + " " + str(e))  
                self.pubs_dict[pub_name]['pub'] = pub
            else:
                self.pubs_dict[pub_name]['pub'] = None





##################################################
### Node Subscribers Class

def EXAMPLE_SUB_CALLBACK(msg):
    return msg

EXAMPLE_SUBS_DICT = {
    'sub_name': {
        'msg': nepi_ros.Empty,
        'topic': 'set_empty',
        'qsize': 1,
        'callback': EXAMPLE_SUB_CALLBACK,  # Use None to skip
        'callback_args': ()
    }
}

EXAMPLE_SUBS_CONFIG_DICT = {
    'namespace': '~',
    'subs_dict': EXAMPLE_SUBS_DICT
}

class NodeSubscribersIF(object):

    ready = False
    msg_if = None

    subs_dict = dict()
    subs_namespace = "~"

    #######################
    ### IF Initialization
    def __init__(self, 
                subs_config_dict = None,
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
        if log_class_name == True:
          log_name = log_name + ": " + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        ##############################   
        self.do_wait = do_wait
        ##############################  
        # Initialize Params System
        if subs_config_dict is not None:
            # Initialize Params
            if subs_config_dict['namespace'] is None:
                self.subs_namespace = self.base_namespace
            else:
                self.subs_namespace = subs_config_dict['namespace']
            self.subs_dict = subs_config_dict['subs_dict']
            if self.subs_dict is None:
                self.subs_dict = dict()
            self._initialize_subs()


        ###############################
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

        
    def get_subs(self):
        return list(self.subs_dict.keys())

    def get_subs_registered(self):
        reg_subs = []
        for sub_name in self.subs_dict.keys():
            sub_dict = self.subs_dict[sub_name]
            if 'sub' in sub_dict.keys():
                if sub_dict['sub'] is not None:
                    reg_subs.append(sub_name)
        return reg_subs


    def unregister(self,sub_name):
        success = False
        if sub_name in self.subs_dict.keys():
            exists = True
            sub_dict = self.subs_dict[sub_name]
            if 'sub' in sub_dict.keys():
                if sub_dict['sub'] is not None:
                    sub_dict['sub'].unregister()
                    time.sleep(1)
                    success = True
            if success == True:
                sub_dict['sub'] = None
        return success


    def register(self,sub_name, sub_dict = None):
        success = False
        success = self._initialize_sub(sub_name, sub_dict = sub_dict)
        return success

    ###############################
    # Class Private Methods
    ###############################
    def _initialize_subs(self):
        for sub_name in self.subs_dict.keys():
            self._initialize_sub(sub_name)

    def _initialize_sub(self,sub_name, sub_dict = None):
        success = False
        sub = None
        if sub_dict is not None:
            if sub_name in self.subs_dict.keys():
                self.msg_if.pub_warn("Subscriber allready registered: " + sub_name ) 
            else:
                self.subs_dict[sub_name] = sub_dict

        if sub_name in self.subs_dict.keys():
            sub_dict = self.subs_dict[sub_name]
            sub_exists = False
            if 'sub' in sub_dict.keys():
                if sub_dict['sub'] is not None:
                    sub_exists = True
            if sub_exists == True:
                    self.msg_if.pub_warn("Subscriber allready registered: " + sub_name )  
                    success = True
            else:
                if 'callback_args' not in sub_dict.keys():
                    sub_dict['callback_args'] = ()
                if sub_dict['callback_args'] is None:
                    sub_dict['callback_args'] = ()
                try:
                    sub_namespace = nepi_ros.create_namespace(self.subs_namespace,sub_dict['topic'])
                    self.msg_if.pub_info("Creating sub for: " + sub_name + " with namespace: " + sub_namespace)
                    if len(sub_dict['callback_args']) == 0:
                        sub = rospy.Subscriber(sub_namespace, sub_dict['msg'],sub_dict['callback'], queue_size = sub_dict['qsize'])
                    else:
                        sub = rospy.Subscriber(sub_namespace, sub_dict['msg'],sub_dict['callback'], queue_size = sub_dict['qsize'], callback_args=sub_dict['callback_args'])
                    self.subs_dict[sub_name]['sub'] = sub
                    success = True
                except Exception as e:
                    self.msg_if.pub_warn("Failed to create subscriber: " + sub_name + " " + str(e))  
                    self.subs_dict[sub_name]['sub'] = None
        return success
            




##################################################
### Node Class Class

class NodeClassIF(object):

    ready = False

    msg_if = None
    log_name = None

    configs_if = None
    params_if = None
    srvs_if = None
    pubs_if = None
    subs_if = None

    #######################
    ### IF Initialization
    def __init__(self, 
                configs_config_dict = None,
                params_config_dict = None,
                srvs_config_dict = None,
                pubs_config_dict = None,
                subs_config_dict = None,
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
        if log_class_name == True:
          log_name = log_name + ": " + self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting Node Class IF Initialization Processes")
        ##############################   
        self.do_wait = do_wait
        ##############################  
        # Create Params Class

        if configs_config_dict is not None:
            self.msg_if.pub_info("Starting Node Configs IF Initialization Processes")
            self.configs_if = NodeConfigsIF(configs_config_dict = configs_config_dict, log_name = log_name)
            ready = self.params_if.wait_for_ready()

        if params_config_dict is not None:
            self.msg_if.pub_info("Starting Node Params IF Initialization Processes")
            self.params_if = NodeParamsIF(params_config_dict = params_config_dict, log_name = log_name)
            ready = self.params_if.wait_for_ready()

        ##############################  
        # Create Services Class
        if srvs_config_dict is not None:
            self.msg_if.pub_info("Starting Node Services IF Initialization Processes")
            self.srvs_if = NodeServicesIF(srvs_config_dict = srvs_config_dict, log_name = log_name)
            ready = self.srvs_if.wait_for_ready()

        ##############################  
        # Create Publisers Class
        if pubs_config_dict is not None:
            self.msg_if.pub_info("Starting Node Publishers IF Initialization Processes")
            self.pubs_if = NodePublishersIF(pubs_config_dict = pubs_config_dict, log_name = log_name, do_wait = self.do_wait)
            ready = self.pubs_if.wait_for_ready()

        ##############################  
        # Create Subscribers Class
        if subs_config_dict is not None:
            self.msg_if.pub_info("Starting Node Subscribers IF Initialization Processes")
            self.subs_if = NodeSubscribersIF(subs_config_dict = subs_config_dict, log_name = log_name)
            ready = self.subs_if.wait_for_ready()
            
        ############################## 
        # Wait for ready
        if params_config_dict is not None:
            ready = self.params_if.wait_for_ready()
        if srvs_config_dict is not None:
            ready = self.srvs_if.wait_for_ready()
        if pubs_config_dict is not None:
            ready = self.pubs_if.wait_for_ready()
        if subs_config_dict is not None:
            ready = self.subs_if.wait_for_ready()



        ###############################
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


   def save_config(self):
        if self.configs_if is not None:
            self.configs_if.save()

    def reset_config(self):
        if self.configs_if is not None:
            self.configs_if.reset()

    def factory_reset_config(self):
        if self.configs_if is not None:
            self.configs_if.factory_reset()



    def get_params(self):
        params = None
        if self.params_if is not None:
            params = self.params_if.get_params()
        return params

    def initialize_params(self):
        if self.params_if is not None:
            self.params_if.initialize_params()

    def load_params(self, file_path):
        if self.params_if is not None:
            self.params_if.load_params(file_path)

    def save_params(self):
        if self.params_if is not None:
            self.params_if.save_params()

    def reset_params(self):
        if self.params_if is not None:
            self.params_if.reset_params()

    def factory_reset_params(self):
        if self.params_if is not None:
            self.params_if.factory_reset_params()

    def has_param(self, param_name):
        exists = False
        if self.params_if is not None:
            exists = self.params_if.has_param(param_name)
        return exists

    def get_param(self, param_name):
        value = None
        if self.params_if is not None:
            try:
                value = self.params_if.get_param(param_name)
            except:
                pass
        return value

    def set_param(self, param_name, value):
        success = False
        if self.params_if is not None:
            try:
                self.params_if.set_param(param_name,value)
                success = True
            except:
                pass
        return success


    def get_srvs(self):
        srvs = None
        if self.srvs_if is not None:
            srvs = self.srvs_if.get_srvs()
        return srvs



    def get_pubs(self):
        pubs = []
        if self.pubs_if is not None:
            pubs = self.pubs_if.get_pubs()
        return pubs


    def unregister_pub(self, pub_name):
        if self.pubs_if is not None:
            try:
                self.pubs_if.unregister(pub_name)
            except:
                pass 

    def register_pub(self, pub_name):
        if self.pubs_if is not None:
            try:
                self.pubs_if.register(pub_name)
            except:
                pass 

    def publish_pub(self, pub_name, pub_msg):
        success = False
        if self.pubs_if is not None:
            try:
                success = self.pubs_if.publish(pub_name, pub_msg)
            except:
                pass 
        return success
                    

    def get_subs(self):
        subs = []
        if self.subs_if is not None:
            subs = self.subs_if.get_subs()
        return subs


    def unregister_sub(self, pub_name):
        if self.subs_if is not None:
            try:
                self.subs_if.unregister(sub_name)
            except:
                pass 

    def register_sub(self,pub_name, callback = None):
        if self.subs_if is not None:
            try:
                self.subs_if.register(sub_name, callback = callback)
            except:
                pass 

    def unregister_class(self):
        pubs = self.get_pubs()
        for pub in pubs:
            self.unregister_pub(pub)

        subs = self.get_subs()
        for sub in subs:
            self.unregister_sub(pub)

    ###############################
    # Class Private Methods
    ###############################

