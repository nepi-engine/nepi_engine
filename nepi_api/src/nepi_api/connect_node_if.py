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
from nepi_sdk import nepi_utils


from std_msgs.msg import String, Empty

from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyRequest as EmptySrvRequest
from std_srvs.srv import EmptyResponse as EmptySrvResponse


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
    def __init__(self, configs_dict , timeout = float('inf')):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################  
        # Initialize Class Variables
        namespace = configs_dict['namespace']
        if namespace is None:
            namespace = self.node_namespace
        else:
            namespace = namespace
        self.namespace = nepi_ros.get_full_namespace(namespace)


        ##############################  
        # Setup Class Publishers for Node Namespace
        self.msg_if.pub_info("Creating config publishers on namespace: " + self.namespace)
        self.save_pub = nepi_ros.create_publisher('~save_config', Empty, queue_size=1) 
        self.reset_pub = nepi_ros.create_publisher('~reset_config', Empty, queue_size=1) 
        self.factory_reset_pub = nepi_ros.create_publisher('~factory_reset_config', Empty, queue_size=1) 

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
            self.msg_if.pub_info("ready")
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
                self.msg_if.pub_warn("Failed send update config: " + str(e))
        return success


    def reset(self):
        success = False
        if self.reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.USER_RESET
                self.reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e))
        return success

    def factory_reset(self):
        success = False
        if self.reset_pub is not None:
            try:
                msg = Reset()
                msg.reset_type = Reset.FACTORY_RESET
                self.reset_pub.publish(msg)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed send config reset: "  + str(e))
        return success

    def unregister_save_if(self): 
        success = False
        if self.ready is False or self.namespace is None:
            self.msg_if.pub_warn("Save Config IF not running")
            success = True
        else:
            self.msg_if.pub_info("Killing Save Confg IF for trigger: " + self.namespace)
            try:
                self.save_pub.unregister()
                self.reset_pub.unregister()

                success = True
            except:
                pass
            time.sleep(1)
            self.ready = False
            self.save_pub = None
            self.reset_pub = None
            self.namespace = None
        return success       

    ###############################
    # Class Private Methods
    ###############################





##################################################
### Node Connect Services Class

# Services Config Dict ####################
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
                do_wait = False
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting Connect Node Services IF Initialization Processes")
        ##############################   

        ##############################  
        # Initialize Services System
        self.srvs_dict = services_dict
        if self.srvs_dict is None:
            self.srvs_dict = dict()
        self._initialize_services()

        if do_wait == True:
            time.sleep(1)

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
            self.msg_if.pub_info("ready")
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


    def call_service(self, service_name, request):
        resp = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'service' in srv_dict.keys():
                service = srv_dict['service']
                if service is not None:
                    resp = nepi_ros.call_service(service, request)
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
            srv_dict = self.srvs_dict[service_name]
            if 'service' not in srv_dict.keys():
                srv_namespace = os.path.join(srv_dict['namespace'],srv_dict['topic'])
                srv_msg = srv_dict['srv']
                self.msg_if.pub_info("Creating service for: " + service_name)
                service = None
                try:
                    service = nepi_ros.connect_service(srv_namespace, srv_msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get service connection: " + service_name + " " + str(e))  
                self.srvs_dict[service_name]['service'] = service

    def _unregister_service(self, service_name):
        purge = False
        if service_name in self.srvs_dict.keys():
            purge = True
            if 'service' in srv_dict.keys():
                try:
                    self.srvs_dict[service_name]['service'].shutdown()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister service: " + service_name + " " + str(e))
        if purge == True:
            del self.srvs_dict[service_name]



'''

##################################################
### Node Connect Publishers Class


# Publishers Config Dict ####################

EXAMPLE_PUBS_DICT = {
    'pub_name': {
        'namespace': '~',
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'latch': False
    }
}


class ConnectNodePublishersIF(NodePublishersIF):

    #######################
    ### IF Initialization
    def __init__(self, 
                pubs_dict = None,
                log_class_name = True
                ):
        ####  IF INIT SETUP ####
        super().__init__(pubs_dict = None,
                log_class_name = True)

    ###############################
    # Class Public Methods
    ###############################


    ###############################
    # Class Private Methods
    ###############################






##################################################
### Node Connect Subscribers Class

def EXAMPLE_SUB_CALLBACK(msg):
    return msg

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


class ConnectNodeSubscribersIF(NodeSubscribersIF):

    msg_if = None
    subs_dict = dict()
    subs_namespace = "~"

    #######################
    ### IF Initialization
    def __init__(self, 
                subs_dict = None,
                log_class_name = True
                ):
        ####  IF INIT SETUP ####
        super().__init__(subs_dict = None,
                log_class_name = True)


    ###############################
    # Class Public Methods
    ###############################


    ###############################
    # Class Private Methods
    ###############################

'''




##################################################
### Node Publishers Class


EXAMPLE_PUBS_DICT = {
    'pub_name': {
        'namespace':  '~',
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'latch': False
    }
}


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

    def has_subscribers(self,pub_name):
        has_subs = False
        if pub_name in self.pubs_dict.keys():
            pub_dict = self.pubs_dict[pub_name]
            if 'pub' in pub_dict.keys():
                has_subs = pub_dict['pub'].get_num_connections() > 0
        return has_subs

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
                if 'topic' in pub_dict.keys() and 'msg' in pub_dict.keys():
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
            purge = True
            if 'pub' in pub_dict.keys():
                try:
                    self.pubs_dict[pub_name]['pub'].unregister()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister pub: " + pub_name + " " + str(e))
        if purge == True:
            del self.pubs_dict[pub_name]


##################################################
### Node Subscribers Class

def EXAMPLE_SUB_CALLBACK(msg):
    return msg

EXAMPLE_SUBS_DICT = {
    'sub_name': {
        'namespace':  '~',
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'callback': EXAMPLE_SUB_CALLBACK, 
        'callback_args': ()
    }
}

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
            if 'sub' not in sub_dict.keys() and sub_dict['callback'] is not None:
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
            purge = True
            if 'sub' in sub_dict.keys():
                try:
                    self.subs_dict[sub_name]['sub'].unregister()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get unregister sub: " + sub_name + " " + str(e))
        if purge == True:
            del self.subs_dict[sub_name]



##################################################
### Node Connect Class

# Configs Config Dict ####################
EXAMPLE_CFGS_DICT = {
        'namespace': '~'
}

def EXAMPLE_CALLBACK_FUNCTION(request):
    response = EmptySrvResponse()
    return response

# Services Config Dict ####################
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


# Publishers Config Dict ####################
EXAMPLE_PUBS_DICT = {
    'pub_name': {
        'namespace': '~',
        'topic': 'set_empty',
        'msg': EmptyMsg,
        'qsize': 1,
        'latch': False
    }
}


# Subscribers Config Dict ####################
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
'''
EXAMPLE_NODE_IF = NodeClassIF(
                configs_dict = EXAMPLE_CFGS_DICT,
                services_dict = EXAMPLE_SRVS_DICT,
                pubs_dict = EXAMPLE_PUBS_DICT,
                subs_dict = EXAMPLE_SUBS_DICT,
                log_class_name = True
)
'''

class ConnectNodeClassIF:

    ready = False

    msg_if = None

    srvs_if = None
    pubs_if = None
    subs_if = None

    #######################
    ### IF Initialization
    def __init__(self, 
                configs_dict = None,
                services_dict = None,
                pubs_dict = None,
                subs_dict = None,
                log_class_name = False,
                node_name = None
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
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################    
        # Create Configs Class
        if configs_dict is not None:
            self.msg_if.pub_info("Starting Connect Node Configs IF Initialization Processes")
            self.configs_if = ConnectNodeConfigsIF(configs_dict = configs_dict)

        ##############################    
        # Create Services Class
        if services_dict is not None:
            self.msg_if.pub_info("Starting Connect Node Services IF Initialization Processes")
            self.srvs_if = ConnectNodeServicesIF(services_dict = services_dict)

        ##############################  
        # Create Subscribers Class
        if pubs_dict is not None:
            self.msg_if.pub_info("Starting Connect Node Publishers IF Initialization Processes")
            self.pubs_if = ConnectNodePublishersIF(pubs_dict = pubs_dict)

        ##############################  
        # Create Subscribers Class
        if subs_dict is not None:
            self.msg_if.pub_info("Starting Connect Node Subscribers IF Initialization Processes")
            self.subs_if = ConnectNodeSubscribersIF(subs_dict = subs_dict)

        ############################## 
        # Wait for ready
        if services_dict is not None:
            ready = self.srvs_if.wait_for_ready()
        if pubs_dict is not None:
            ready = self.pubs_if.wait_for_ready()
        if subs_dict is not None:
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
        self.msg_if.pub_info("Waiting for Ready")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Wait for Ready Timed Out")
        else:
            self.msg_if.pub_info("ready")
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
        if self.srvs_if is not None:
            srvs_names = self.srvs_if.get_services()
        return srvs_names


    def create_request_msg(self,service_name):
        req = None
        if self.srvs_if is not None:
            req = self.srvs_if.create_request_msg(service_name)
        return req

    def create_response_msg(self,service_name):
        msg = None
        if self.srvs_if is not None:
            msg = self.srvs_if.create_response_msg(service_name)
        return msg

    def call_service(self,service_name,request):
        resp = None
        if self.srvs_if is not None:
            resp = self.srvs_if.call_service(service_name,request)
        return resp

    def register_service(self,service_name, service_dict):
        if self.srvs_if is not None:
            self.srvs_if.register_service(service_name, service_dict)


    def unregister_service(self,service_name):
        if self.srvs_if is not None:
            self.srvs_if.unregister_service(service_name)

    def unregister_services(self):
        if self.srvs_if is not None:
            self.srvs_if.unregister_services()


    # Publisher Methods ####################
    def get_pubs(self):
        pubs = None
        if self.pubs_if is not None:
            pubs = self.pubs_if.get_pubs()
        return pubs



    def publish_pub(self, pub_name, pub_msg):
        if self.pubs_if is not None:
            try:
                self.pubs_if.publish(pub_name, pub_msg)
            except:
                pass 

    def register_pub(self,pub_name, pub_dict):
        if self.srvs_if is not None:
            self.srvs_if.register_pub(pub_name, pub_dict)


    def unregister_pub(self,pub_name):
        if self.srvs_if is not None:
            self.srvs_if.unregister_pub(pub_name)

    def unregister_pubs(self):
        if self.srvs_if is not None:
            self.srvs_if.unregister_pubs()
                    
    # Subsciber Methods ####################
    def get_subs(self):
        subs = None
        if self.subs_if is not None:
            subs = self.subs_if.get_subs()
        return subs


    def register_sub(self,sub_name, sub_dict):
        if self.srvs_if is not None:
            self.srvs_if.register_sub(sub_name, sub_dict)


    def unregister_sub(self,sub_name):
        if self.srvs_if is not None:
            self.srvs_if.unregister_sub(sub_name)

    def unregister_subs(self):
        if self.srvs_if is not None:
            self.srvs_if.unregister_subs()

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



