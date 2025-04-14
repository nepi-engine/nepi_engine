#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

from nepi_sdk import nepi_ros

from std_msgs.msg import String, Empty
from nepi_ros_interfaces.msg import Reset

from nepi_ros_interfaces.msg import SaveData, SaveDataRate, SaveDataStatus
from nepi_ros_interfaces.srv import DataProductQuery, DataProductQueryRequest, DataProductQueryResponse

from nepi_api.connect_con_node_if import ConnectNodeClassIF
from nepi_api.sys_if_msg import MsgIF

class ConnectSettingsIF(object):

    ready = False

    namespace = None
    
    status_msg = None
    status_connected = False


    #######################
    ### IF Initialization
    def __init__(self, namespace = None , timeout = float('inf')):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        

        #############################
        # Initialize Class Variables
        if namespace is None:
            namespace = '~'
        self.namespace = nepi_ros.get_full_namespace(namespace)

        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'data_products_query': {
                'msg': DataProductQuery,
                'namespace': self.namespace,
                'topic': 'query_data_products',
                'service': None
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
            'rate': {
                'msg': SaveDataRate,
                'namespace': self.namespace,
                'topic': 'save_data_rate',
                'qsize': 1,
                'latch': False
            },
            'save_data': {
                'msg': SaveData,
                'namespace': self.namespace,
                'topic': 'save_data',
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
            }            

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
                'msg': SaveData,
                'namespace': self.base_namespace,
                'topic': 'save_data',
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
                'topic': 'save_data_reset',
                'qsize': 1,
                'latch': False
            },
            'factory_reset_all': {
                'msg': Empty,
                'namespace': self.base_namespace,
                'topic': 'save_data_factory_reset',
                'qsize': 1,
                'latch': False
            }
        }



        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'msg': SaveDataStatus,
                'namespace': self.namespace,
                'topic': 'save_data_status',
                'qsize': 1,
                'callback': self._statusCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(services_dict = self.SRVS_DICT,
                                            pubs_dict = self.PUBS_DICT,
                                            subs_dict = self.SUBS_DICT
                                            )


        self.con_node_if.wait_for_ready()

        ##############################
        # Complete Initialization
        self.publish_status()
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
        if self.ready is not None:
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
        

    def get_namespace(self):
        return self.namespace

    def get_data_products(self):
        srv_name =  'data_products_query'
        req = DataProductQueryRequest()
        self.con_node_if.call_service(srv_name,req)

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_ros.convert_msg2dict(self.status_msg)
        else:
            self.msg_if.publish_warn("Status Listener Not ready")
        return status_dict

    def save_data_pub(self,enable):
        pub_name = 'save_data'
        msg = SaveData()
        msg.save_continuous = enable
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



    ###############################
    # Class Private Methods
    ###############################

    def _statusCb(self,msg):
        self.status_msg = msg
        self.status_connected = True


