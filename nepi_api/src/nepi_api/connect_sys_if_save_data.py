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

from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.sys_if_msg import MsgIF

class ConnectSettingsIF(object):

    connected = False

    namespace = None
    
    status_msg = None
    status_connected = False


    #######################
    ### IF Initialization
    def __init__(self, namespace , timeout = float('inf')):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        log_name = self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        

        #############################

        self.namespace = namespace

        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'data_products': {
                'msg': DataProductQuery,
                'topic': 'query_data_products',
                'service': None
            }
        }
        self.SRVS_CONFIG_DICT = {
                'namespace': self.namespace,
                'srvs_dict': self.SRVS_DICT
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'save_data': {
                'msg': SaveData,
                'topic': 'save_data',
                'qsize': 1,
                'latch': False
            },
            'snapshot': {
                'msg': Empty,
                'topic': 'snapshot_trigger',
                'qsize': 1,
                'latch': False
            },
            'prefix': {
                'msg': String,
                'topic': 'save_data_prefix',
                'qsize': 1,
                'latch': False
            },
            'rate': {
                'msg': SaveDataRate,
                'topic': 'save_data_rate',
                'qsize': 1,
                'latch': False
            },
            'reset': {
                'msg': Empty,
                'topic': 'save_data_reset',
                'qsize': 1,
                'latch': False
            },
            'reset_factory': {
                'msg': Empty,
                'topic': 'save_data_reset_factory',
                'qsize': 1,
                'latch': False
            }
        }

        self.PUBS_CONFIG_DICT = {
            'namespace': self.namespace,
            'pubs_dict': self.PUBS_DICT
        }


        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'msg': SaveDataStatus,
                'topic': 'save_data_status',
                'qsize': 1,
                'callback': self._statusCb,
                'callback_args': ()
            }
        }

        self.SUBS_CONFIG_DICT = {
            'namespace': self.settings_namespace,
            'subs_dict': self.SUBS_DICT
        }


        self.class_if = ConnectNodeClassIF(srvs_config_dict = self.SRVS_CONFIG_DICT,
                        pubs_config_dict = self.PUBS_CONFIG_DICT,
                        subs_config_dict = self.SUBS_CONFIG_DICT
                        )


        time.sleep(1)
        self.connected = True

        ##############################   
        self.msg_if.pub_info("IF Initialization Complete")


    ###############################
    # Class Public Methods
    ###############################

    def check_connection(self):
        return self.connected

    def get_namespace(self):
        return self.namespace

    def get_data_products(self):
        srv_name = ''
        req = DataProductQueryRequest()
        self.class_if.call_service(srv_name,req)

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_ros.convert_msg2dict(self.status_msg)
        else:
            self.msg_if.publish_warn("Status Listener Not connected")
        return status_dict

    def save_data_pub(self,enable):
        pub = 'save_data'
        msg = SaveData()
        msg.save_continuous = enable
        self.class_if.publish_pub(pub,msg)

    def save_data_prefix_pub(self,prefix):
        pub = 'prefix'
        msg = prefix
        self.class_if.publish_pub(pub,msg)

    def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        pub = 'rate'
        msg = SaveDataRate()
        msg.data_product = data_product
        msg.save_rate_hz = rate_hz
        self.class_if.publish_pub(pub,msg)

    def snapshot_pub(self):
        pub = 'snapshot'
        msg = Empty()
        self.class_if.publish_pub(pub,msg)

    def reset_pub(self):
        pub = 'reset'
        msg = Empty()
        self.class_if.publish_pub(pub,msg)

    def factory_reset_pub(self):
        pub = 'reset_factory'
        msg = Empty()
        self.class_if.publish_pub(pub,msg)
       
    def unregister_save_data_if(self): 
        success = False
        self.class_if.unregister()
        time.sleep(1)
        self.connected = False
        self.status_connected = False
        success = True
        return success  



    ###############################
    # Class Private Methods
    ###############################

    def _statusCb(self,msg):
        self.status_msg = msg
        self.status_connected = True


