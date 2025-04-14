#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg


from std_msgs.msg import String, Empty

from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyRequest as EmptySrvRequest
from std_srvs.srv import EmptyResponse as EmptySrvResponse


from nepi_api.sys_if_msg import MsgIF
from nepi_api.node_if import NodePublishersIF, NodeSubscribersIF



##################################################
### Node Connect Configs Class


EXAMPLE_CFGS_CONFIG_DICT = {
        'namespace': '~'
}

class ConnectConfigsIF(object):


    msg_if = None
    ready = False
    configs_dict = None
    namespace = '~'

    save_pub = None
    reset_pub = None
    factory_reset_pub = None

    #######################
    ### IF Initialization
    log_name = "ConnectSaveConfigIF"
    def __init__(self, configs_dict , timeout = float('inf')):
        ####  IF INIT SETUP ####
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
        self.msg_if.pub_info("Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Failed to Connect")
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
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed send update config: " + str(e))
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
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed send config reset: "  + str(e))
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
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed send config reset: "  + str(e))
        return success

    def unregister_save_if(self): 
        success = False
        if self.ready is False or self.namespace is None:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Save Config IF not running")
            success = True
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Killing Save Confg IF for trigger: " + self.namespace)
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
        'svr': EmptySrv,
        'req': EmptySrvRequest(),
        'resp': EmptySrvResponse()
    }
}

class ConnectNodeServicesIF(object):


    msg_if = None
    ready = False
    srvs_dict = dict()


    #######################
    ### IF Initialization
    def __init__(self, 
                services_dict = None,
                log_class_name = True,
                do_wait = True
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()
        # Create Msg Class
        log_name = ''
        if log_class_name == True:
          log_name = self.class_name
        self.msg_if = MsgIF(log_name = log_name)
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
        self.msg_if.pub_info("Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Failed to Connect")
        else:
            self.msg_if.pub_info("ready")
        return self.ready


    def get_services(self):
        return list(self.srvs_dict.keys())

    def create_request_msg(service_name):
        req = None
        if service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            if 'req' in srv_dict.keys():
                req = srv_dict['req']
        return req

    def create_response_msg(service_name):
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
                service = self.srv_dict['service']
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
                srv_msg = service_dict['svr']
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Creating service for: " + service_name)
                service = None
                try:
                    service = nepi_ros.create_service(service_namespace, srv_msg,srv_callback)
                    time.sleep(1)
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to get service connection: " + service_name + " " + str(e))  
                self.srv_dict[service_name]['service'] = service

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


class ConnectNodePublishersIF(NodePublisherIF):

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

class ConnectNodeSubscribersIF(object):

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



##################################################
### Node Connect Class Class

# Configs Config Dict ####################
EXAMPLE_CFGS_DICT = {
        'namespace': '~'
}


# Services Config Dict ####################
EXAMPLE_SRVS_DICT = {
    'service_name': {
        'namespace': '~',
        'topic': 'empty_query',
        'svr': EmptySrv,
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
EXAMPLE_NODE_IF = NodeClassIF(self,
                configs_dict = EXAMPLE_CFGS_DICT,
                services_dict = EXAMPLE_SRVS_DICT,
                pubs_dict = EXAMPLE_PUBS_DICT,
                subs_dict = EXAMPLE_SUBS_DICT,
                log_class_name = True
)
'''

class ConnectNodeClassIF(object):

    ready = False

    msg_if = None
    log_name = None

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
            self.configs_if = ConnectNodeConifgsIF(configs_dict = configs_dict)

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
        self.msg_if.pub_info("Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Failed to Connect")
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
        srvs = None
        if self.srvs_if is not None:
            srvs = self.srvs_if.get_services()
        return srvs


    def create_request_msg(service_name):
        req = None
        if self.srvs_if is not None:
            req = self.srvs_if.create_request_msg(service_name)
        return req

    def create_response_msg(service_name):
        resp = None
        if self.srvs_if is not None:
            resp = self.srvs_if.create_response_msg(service_name)
        return resp

    def call_service(self,service_name,request):
        resp = None
        if self.srvs_if is not None:
            resp = self.srvs_if.call_service(service_name,request)
        return srvs

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



