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

from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv

from nepi_api.sys_if_msg import MsgIF
from nepi_api.node_if import NodePublishersIF, NodeSubscribersIF


##################################################
### Node Services Class

EXAMPLE_SRVS_DICT = {
    'service_name': {
        'topic': 'empty_query',
        'msg': EmptySrv,
        'callback': None
    }
}

EXAMPLE_SRVS_CONFIG_DICT = {
        'namespace': '~',
        'srvs_dict': EXAMPLE_SRVS_DICT
}

class ConnectNodeServicesIF(object):

    msg_if = None
    
    srvs_dict = dict()
    srvs_namespace = "~"

    #######################
    ### IF Initialization
    def __init__(self, 
                srvs_config_dict = None,
                log_class_name = True
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
        if srvs_config_dict['srvs_namespace'] is None:
            self.srvs_namespace = self.base_namespace
        else:
            self.srvs_namespace = srvs_config_dict['srvs_namespace']
        self.srvs_dict = srvs_config_dict['srvs_dict']
        if self.srvs_dict is None:
            self.srvs_dict = dict()
        self._initialize_srvs()
        ##############################   

    ###############################
    # Class Public Methods
    ###############################
    def get_srvs(self):
        return list(self.srvs_dict.keys())

    def call_srv(self, srv_name, request):
        resp = None
        if srv_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[srv_name]
            if 'service' in srv_dict.keys():
                service = self.srv_dict['service']
                if service is not None:
                    resp = nepi_ros.call_service(service, request)
        return resp

    ###############################
    # Class Private Methods
    ###############################

    def _initialize_srvs(self):
        for service_name in self.srvs_dict.keys():
            srv_dict = self.srvs_dict[service_name]
            srv_namespace = os.path.join(self.srvs_namespace,srv_dict['topic'])
            srv_msg = service_dict['msg']
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Creating service for: " + service_name)
            service = None
            try:
                
                service = nepi_ros.create_service(service_namespace, srv_msg,srv_callback)
                time.sleep(1)
            except Exception as e:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to get service connection: " + service_name + " " + str(e))  
            self.srv_dict[service_name]['service'] = service






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


class ConnectNodePublishersIF(NodePublisherIF):

    #######################
    ### IF Initialization
    def __init__(self, 
                pubs_config_dict = None,
                log_class_name = True
                ):
        ####  IF INIT SETUP ####
        super().__init__(pubs_config_dict = None,
                log_class_name = True)

    ###############################
    # Class Public Methods
    ###############################


    ###############################
    # Class Private Methods
    ###############################






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

class ConnectNodeSubscribersIF(object):

    msg_if = None

    subs_dict = dict()
    subs_namespace = "~"

    #######################
    ### IF Initialization
    def __init__(self, 
                subs_config_dict = None,
                log_class_name = True
                ):
        ####  IF INIT SETUP ####
        super().__init__(subs_config_dict = None,
                log_class_name = True)


    ###############################
    # Class Public Methods
    ###############################

    def wait_for_sub_pub(self,sub_name,timeout = float('inf')):
        sub_dict = self.subs_dict[service_name]
        sub_namespace = os.path.join(self.subs_namespace,sub_dict['topic'])
        return nepi_ros.wait_for_service(sub_namespace, timeout = timeout )

    ###############################
    # Class Private Methods
    ###############################



##################################################
### Node Class Class

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
                params_config_dict = None,
                srvs_config_dict = None,
                pubs_config_dict = None,
                subs_config_dict = None,
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
        # Create Services Class
        if srvs_config_dict is not None:
            self.msg_if.pub_info("Starting Node Services IF Initialization Processes")
            self.srvs_if = ConnectNodeServicesIF(srvs_config_dict = srvs_config_dict)

        ##############################  
        # Create Subscribers Class
        if pubs_config_dict is not None:
            self.msg_if.pub_info("Starting Node Publishers IF Initialization Processes")
            self.pubs_if = ConnectNodePublishersIF(pubs_config_dict = pubs_config_dict)

        ##############################  
        # Create Subscribers Class
        if subs_config_dict is not None:
            self.msg_if.pub_info("Starting Node Subscribers IF Initialization Processes")
            self.subs_if = ConnectNodeSubscribersIF(subs_config_dict = subs_config_dict)

        ############################## 
        # Wait for ready
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

        


    def get_srvs(self):
        srvs = None
        if self.srvs_if is not None:
            srvs = self.srvs_if.get_srvs()
        return srvs



    def call_srv(self,srv_name,request):
        resp = None
        if self.srvs_if is not None:
            resp = self.srvs_if.call_srv(srv_name,request)
        return srvs


    def get_pubs(self):
        pubs = None
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
        if self.pubs_if is not None:
            try:
                self.pubs_if.publish(pub_name, pub_msg)
            except:
                pass 
                    

    def get_subs(self):
        subs = None
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



