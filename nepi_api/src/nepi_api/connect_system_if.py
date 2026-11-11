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
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_triggers
from nepi_sdk import nepi_states

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import Reset

from nepi_interfaces.msg import NavPose, NavPoseStatus

from nepi_interfaces.msg import SaveDataRate, SaveDataStatus, FilenameConfig
from nepi_interfaces.srv import SaveDataCapabilitiesQuery, SaveDataCapabilitiesQueryRequest, SaveDataCapabilitiesQueryResponse



from nepi_interfaces.msg import ControlsStatus, UpdateControl

from nepi_interfaces.msg import SystemState, SystemStates
from nepi_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_interfaces.msg import SystemTrigger



from nepi_api.messages_if import MsgIF


from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.connect_node_if import ConnectNodeIF


#########################################
# Connect Data Product IF Class
#########################################


# connect_status_msg is the status msg TYPE NAME, as a string. ConnectNodeIF
# hands it to nepi_sdk.find_topics_by_msg, which string-compares it against the
# ROS type names of live topics, and publishes it as ConnectIFStatus.status_msg_type,
# which is a string field. connect_status_msg_class is the matching msg CLASS,
# which subscribe_topic needs as the subscriber msg type. The two are separate
# parameters because neither consumer accepts the other's form, and ConnectSystemIF
# is generic so it cannot hardcode the class the way connect_targets_if.py does.
CONNECT_ID='connect_system'
CONNECT_STATUS_MSG=None
CONNECT_STATUS_MSG_CLASS=None
CONNECT_DATA_MSG=None
CONNECT_NAME='connect_system'


CONNECTED_TIMEOUT = 2


class ConnectSystemIF(ConnectNodeIF):

    # ADD Additional Connect Callback Functions


    msg_if = None
    ready = False
    namespace = '~'

    node_if = None

    connect_name = CONNECT_NAME
    status_msg = None
    connected = False
    last_status_time = 0

    connect_status_msg = None
    connect_status_msg_class = None
    status_callback = None # Backwards Compatibility

    # Data pipeline state. The connected data source publishes an Image on its
    # base namespace and an ImageStatus on <namespace>/status. Retrieved image
    # frames are cached here (thread-safe) for polling consumers, or handed
    # straight to data_callback when one is provided.
    # data_lock is created per instance in __init__. As a class attribute it
    # would be one lock shared by every connect instance in the process, needlessly
    # serialising instances that only ever guard their own data_dict.
    data_msg = None
    data_dict = None
    data_lock = None

    connect_data_msg = None
    connect_data = True
    data_callback = None

    connect_topic_subs_dict = None
    connect_topic_pubs_dict = None

    connect_topic_controls_dict = None

    #######################
    ### IF Initialization
    def __init__(self,
                connect_id = CONNECT_ID,
                connect_name = CONNECT_NAME,
                namespace = None,
                auto_select_enabled = True,
                selected_topic = None,
                connect_status_msg = CONNECT_STATUS_MSG,
                connect_status_msg_class = CONNECT_STATUS_MSG_CLASS,
                status_callback = None,
                connect_data = True,
                connect_data_msg = CONNECT_DATA_MSG,
                data_callback = None,
                filter_topic_list = [],
                connect_topic_controls_dict = None,
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):

        # Raise rather than return. A bare return here left a half-built object with
        # msg_if still None, so the real failure surfaced much later as an
        # AttributeError on NoneType somewhere unrelated to the missing argument.
        if connect_status_msg is None:
            raise ValueError(str(type(self).__name__) +
                    ": connect_status_msg is required (status msg type name, as a string)")
        if connect_status_msg_class is None:
            raise ValueError(str(type(self).__name__) +
                    ": connect_status_msg_class is required (status msg class). "
                    "subscribe_topic always creates the status subscriber, so this "
                    "is required even when connect_data is False.")

        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  

        
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.msg_if.pub_info("Starting " + str(self.class_name) + " Initialization Processes")


        self.msg_if = msg_if
        self.node_if = node_if
        self.connect_name = connect_name
        self.connect_topic_controls_dict = connect_topic_controls_dict

        if namespace is None:
            namespace = self.node_namespace
        namespace = namespace.replace('/' + self.connect_name, '')
        self.namespace = nepi_sdk.create_namespace(namespace,self.connect_name)
        self.node_if_prefix = self.namespace.replace(self.node_namespace + '/','') + '/'

        super().__init__(
                connect_id = connect_id,
                connect_name = connect_name,
                namespace = namespace,
                connect_status_msg = connect_status_msg,
                auto_select_enabled = auto_select_enabled,
                selected_topic = selected_topic,
                filter_topics_list = filter_topic_list,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = self.msg_if,
                node_if = self.node_if
                )
        ####  IF INIT SETUP ####

        self.wait_for_connect_ready()



        ##############################
        # Initialize Class Variables
        self.connect_status_msg = connect_status_msg
        self.connect_status_msg_class = connect_status_msg_class
        self.status_callback = status_callback
        self.connect_data_msg = connect_data_msg
        self.connect_data = connect_data and connect_data_msg is not None
        self.data_callback = data_callback
        self.data_lock = threading.Lock()


        ##############################
        # Start updater process

        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is ConnectSystemIF:
            self.start()


    #######################
    # Class Public Methods
    #######################



    def get_data_topic(self):
        """Return the image data topic of the connected data source.

        Returns:
            str: The selected data source namespace, which is also the Image topic
                the source publishes on, or 'None' if no source is selected.
        """
        return self.selected_topic



    def save_config(self):
        """Publish a save configuration command to persist current settings on the data source.
        """
        self.node_if.publish_pub('save_config',Empty())

    def reset_config(self):
        """Publish a reset configuration command to restore the last saved settings on the data source.
        """
        self.node_if.publish_pub('reset_config',Empty())

    def factory_reset_config(self):
        """Publish a factory reset command to restore factory default settings on the data source.
        """
        self.node_if.publish_pub('factory_reset_config',Empty())

    #################
    ## Save Data Functions

    def get_save_data_products(self):
        """Return the list of available save data products for this data source.

        Returns:
            list: A list of data product identifiers supported by the save data interface.
        """
        data_products = self.con_save_data_if.get_data_products()
        return data_products

    def get_save_data_status_dict(self):
        """Return the current save data status as a dictionary.

        Returns:
            dict: A dictionary representation of the save data interface status.
        """
        status_dict = self.con_save_data_if.get_status_dict()
        return status_dict

    def save_data_enable_pub(self,enable):
        """Enable or disable data saving on the data source.

        Args:
            enable (bool): True to enable data saving, False to disable it.
        """
        self.con_save_data_if.save_data_pub(enable)

    def save_data_prefix_pub(self,prefix):
        """Publish an updated filename prefix for saved data files.

        Args:
            prefix (str): The prefix string to prepend to saved data filenames.
        """
        self.con_save_data_if.save_data_prefix_pub(prefix)

    def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        """Publish an updated save rate for a data product.

        Args:
            rate_hz (float): Desired save rate in Hz.
            data_product (int, optional): Identifier for the specific data product to update.
                Defaults to SaveDataRate.ALL_DATA_PRODUCTS.
        """
        self.con_save_data_if.publish_pub(rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS)

    def save_data_snapshot_pub(self):
        """Trigger a one-shot snapshot save of current data on the data source.
        """
        self.con_save_data_if.publish_pub()

    def save_data_factory_reset_pub(self):
        """Publish a factory reset command to restore the save data configuration to defaults.
        """
        pub_name = 'factory_reset'
        msg = Empty()
        self.con_save_data_if.publish_pub(pub_name,msg)

    ###############################
    # Class Private Methods
    ###############################

    def updaterCb(self,timer):
        cur_time = nepi_utils.get_time()
        last_time = copy.deepcopy(self.last_status_time )
        if self.connected == True:
            if (cur_time - last_time) > CONNECTED_TIMEOUT:
                self.connected = False
                self.status_msg = None

        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)




    def subscribe_topic(self, topic):
        self.msg_if.pub_debug("subscribe_data_topic Called")

        success = False
        success = self.unsubscribe_topic()

        # Subscribers Config Dict ####################
        self.connect_topic_subs_dict = {
            self.node_if_prefix + 'status_sub': {
                'namespace': self.selected_topic,
                'topic': 'status',
                'msg': self.connect_status_msg_class,
                'qsize': 10,
                'callback': self._statusCb
            }
        }
        if self.connect_data == True:
             self.connect_topic_subs_dict[self.node_if_prefix + 'data_sub'] = {
                'namespace': self.selected_topic,
                'topic': '',
                'msg': self.connect_data_msg,
                'qsize': 1,
                'callback': self._dataCb
            }


        # Publishers Config Dict ####################
        self.connect_topic_pubs_dict = {
            self.node_if_prefix + 'save_config': {
                'namespace': self.selected_topic,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': 1,
            },
            self.node_if_prefix + 'reset_config': {
                'namespace': self.selected_topic,
                'topic': 'reset_config',
                'msg': Empty,
                'qsize': 1,
            },
            self.node_if_prefix + 'factory_reset_config': {
                'namespace': self.selected_topic,
                'topic': 'factory_reset_config',
                'msg': Empty,
                'qsize': 1,
            }
        }

        # Controls Config Dict ####################
        # Each subclass authors its controls dict in its own constructor, before the
        # connected namespace is known, so every entry carries the 'unknown' namespace
        # sentinel meaning "fill this in with the connected topic". Resolve into a
        # per-entry copy. Writing self.selected_topic back into
        # self.connect_topic_controls_dict strips the sentinel permanently, and the
        # next subscribe -- connect_node_if._updaterCb calls subscribe_topic on every
        # topic change -- would then re-register every control publisher against the
        # previously connected namespace, silently commanding the wrong data source.
        if self.connect_topic_controls_dict is not None:
            for control_name in self.connect_topic_controls_dict.keys():
                control_dict = dict(self.connect_topic_controls_dict[control_name])
                if control_dict['namespace'] == 'unknown':
                    control_dict['namespace'] = self.selected_topic
                self.connect_topic_pubs_dict[control_name] = control_dict

        if self.node_if is not None:
            self.node_if.register_pubs(self.connect_topic_pubs_dict)
            self.node_if.register_subs(self.connect_topic_subs_dict)
            self.connecting = True
            self.connected = False
            self.connected_topic = 'None'
            self.status_msg = None

        return success




    def unsubscribe_topic(self):
        success = False
        if self.connecting == True or self.connected == True:
            self.msg_if.pub_debug("unsubscribe_topic Called")

            if self.node_if is not None:
                if self.connect_topic_subs_dict is not None:
                    for sub_name in self.connect_topic_subs_dict.keys():
                        self.node_if.unregister_sub(sub_name)
            self.connect_topic_subs_dict = None

            if self.node_if is not None:
                if self.connect_topic_pubs_dict is not None:
                    for pub_name in self.connect_topic_pubs_dict.keys():
                        self.node_if.unregister_pub(pub_name)
            self.connect_topic_pubs_dict = None

            nepi_sdk.sleep(1)
            self.connecting = False
            self.connected = False
            self.connected_topic = 'None'
            self.status_msg = None
            self.data_dict = None
            success = True
        return success


    def _statusCb(self,status_msg):
        self.last_status_time = nepi_utils.get_time()
        if self.connected == False:
            self._announceConnected(self.connect_id)
            self.connecting = False
            self.connected_topic = self.selected_topic
        self.connected = True
        self.status_msg = status_msg

        if self.status_callback is not None:
            status_dict = self.get_status_dict()
            self.status_callback(status_dict)


    def _dataCb(self,data_msg):
        pass



    def _navposeCb(self,navpose_msg):
       
        navpose_dict = nepi_nav.convert_navposes_msg2dict(navpose_msg)


        self.navpose_dict_lock.acquire()
        self.navpose_dict = navpose_dict
        self.navpose_dict_lock.release()




#########################################
# Connect NavPose IF Class
#########################################
# Client-side connect class mirroring data_if.NavPoseIF. NavPoseIF is a
# standalone (non-image) data interface, so this connect class subclasses
# ConnectNodeIF directly and diverges from the image-oriented ConnectDataIF:
# the connected source publishes a NavPose on its base namespace and a
# NavPoseStatus on <namespace>/status. Retrieved NavPose messages are
# converted to nav pose dictionaries and cached (thread-safe) for polling
# consumers, or handed straight to data_callback when one is provided.


NAVPOSE_CONNECT_ID = 'connect_navpose'
NAVPOSE_CONNECT_STATUS_MSG = 'NavPoseStatus'
NAVPOSE_CONNECT_STATUS_MSG_CLASS = NavPoseStatus
NAVPOSE_CONNECT_DATA_MSG = NavPose
NAVPOSE_CONNECT_NAME = 'connect_navpose'


class ConnectNavPoseIF(ConnectSystemIF):

    # ADD Additional Connect Callback Functions
    navpose_callback = None
    #######################
    ### IF Initialization
    def __init__(self,
                auto_select_enabled = True,
                selected_topic = None,
                status_callback = None,
                connect_data = True,
                data_callback = None,
                navpose_callback = None,
                filter_topic_list = [],
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  

        
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.msg_if.pub_info("Starting " + str(self.class_name) + " Initialization Processes")

        # Forward by keyword. These arguments used to be passed positionally into a
        # signature whose 3rd and 6th slots are connect_status_msg/connect_data_msg,
        # so every argument landed in the wrong slot and connect_status_msg received
        # None, which tripped the guard in ConnectDataIF.__init__ and left the
        # instance dead. msg_if and node_if never arrived at all.

        connect_name = NAVPOSE_CONNECT_NAME
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,connect_name)
        self.node_if_prefix = self.namespace.replace(self.node_namespace + '/','') + '/'

        self.navpose_callback = navpose_callback

        # Controls Config Dict ####################
        # Mirrors data_if.NavPoseIF.SUBS_DICT, which advertises exactly one
        # subscriber. The 'unknown' namespace is the sentinel ConnectDataIF
        # replaces with the connected topic at subscribe time.
        connect_topic_controls_dict = {
            self.node_if_prefix + self.node_if_prefix + 'navpose_reset': {
                'namespace': 'unknown',
                'topic': 'reset',
                'msg': Empty,
                'qsize': 1,
            }
        }

        super().__init__(
                connect_id = NAVPOSE_CONNECT_ID,
                connect_name = connect_name,
                auto_select_enabled = auto_select_enabled,
                selected_topic = selected_topic,
                connect_status_msg = NAVPOSE_CONNECT_STATUS_MSG,
                connect_status_msg_class = NAVPOSE_CONNECT_STATUS_MSG_CLASS,
                status_callback = status_callback,
                # The NavPose data topic is what feeds navpose_callback and
                # data_callback (through _dataCb); the status topic carries no pose
                connect_data = connect_data,
                connect_data_msg = NAVPOSE_CONNECT_DATA_MSG,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                connect_topic_controls_dict = connect_topic_controls_dict,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = msg_if,
                node_if = node_if
                )
        ####  IF INIT SETUP ####

        self.msg_if.pub_info("IF Initialization Complete")
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is ConnectNavPoseIF:
            self.start()


    #######################
    # Class Public Methods
    #######################

    #################
    ## Data Functions


    def get_navpose_msg(self):
        """Return a deep copy of the most recently received nav pose dictionary.

        Mirrors data_if.NavPoseIF.get_navpose_dict, which is the contract the
        data_if image, depth map and pointcloud classes call against whichever
        navpose interface they hold -- a server-side NavPoseIF when one is passed
        in, or a ConnectNavPoseIF built from a namespace when one is not.

        Returns:
            dict: The last nav pose data dictionary, or None if no NavPose message
                has arrived from the connected source yet.
        """
        if self.data_msg is None:
            return None
        self.data_lock.acquire()
        data_msg = copy.deepcopy(self.data_msg)
        self.data_lock.release()
        return data_msg

    def get_navpose_dict(self):
        """Return a deep copy of the most recently received nav pose dictionary.

        Mirrors data_if.NavPoseIF.get_navpose_dict, which is the contract the
        data_if image, depth map and pointcloud classes call against whichever
        navpose interface they hold -- a server-side NavPoseIF when one is passed
        in, or a ConnectNavPoseIF built from a namespace when one is not.

        Returns:
            dict: The last nav pose data dictionary, or None if no NavPose message
                has arrived from the connected source yet.
        """
        if self.data_dict is None:
            return None
        self.data_lock.acquire()
        navpose_dict = copy.deepcopy(self.data_dict)
        self.data_lock.release()
        return navpose_dict


    #################
    ## Control Functions

    def reset(self):
        """Publish a reset command to the connected navpose source.

        Restores the navpose interface to its initialized state. Mirrors
        data_if.NavPoseIF.reset.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'navpose_reset', Empty())


    ###############################
    # Class Private Methods
    ###############################

    def _dataCb(self,data_msg):
        # ConnectSystemIF.subscribe_topic registers the data subscriber with _dataCb
        self._navposeCb(data_msg)

    def _navposeCb(self,navpose_msg):

        navpose_dict = nepi_nav.convert_navpose_msg2dict(navpose_msg)
        #self.msg_if.pub_warn("Got navpose dict: " + str(navpose_dict), throttle_s = 5)
        if self.data_callback is not None:
            self.data_callback(navpose_dict)
        if self.navpose_callback is not None:
            try:
                self.navpose_callback(navpose_dict)
            except:
                pass
        
        self.data_lock.acquire()
        self.data_msg = navpose_msg
        self.data_dict = navpose_dict
        self.navpose_dict = navpose_dict
        self.data_lock.release()
        self.got_data = True







################################
# Connect Save Data IF
################################

class ConnectSaveDataIF:

    ready = False

    namespace = None
    
    status_msg = None
    status_connected = False


    #######################
    ### IF Initialization
    def __init__(self, namespace = None , timeout = float('inf'),
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
        self.msg_if.pub_info("Starting SaveData IF Initialization Processes" )
        ############################## 
        

        #############################
        # Initialize Class Variables
        if namespace is None:
            namespace = '~'
        namespace = nepi_sdk.create_namespace(namespace,'save_data')
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.msg_if.pub_warn("Using save data namespace: " + self.namespace )

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
                                            
                                            msg_if = self.msg_if
                                            )


        #self.con_node_if.wait_for_ready()
        nepi_sdk.wait()

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return the current ready state of this save data interface."""
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout elapses, then return the ready state."""
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection" )
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect" )
            else:
                self.msg_if.pub_info("Connected" )
        return self.ready


    def get_namespace(self):
        """Return the fully resolved ROS namespace for this save data interface."""
        return self.namespace

    def get_data_products(self):
        """Query and return the available save data products from the remote node."""
        srv_name =  'data_products_query'
        req = DataProductQueryRequest()
        self.con_node_if.call_service(srv_name,req)

    def get_status_dict(self):
        """Return the latest save data status as a dictionary, or None if no status has been received."""
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        else:
            self.msg_if.publish_warn("Status Listener Not ready" )
        return status_dict

    def save_data_pub(self,enable):
        """Publish a boolean command to enable or disable data saving on the remote node."""
        pub_name = 'save_data'
        msg = enable
        self.con_node_if.publish_pub(pub_name,msg)

    def save_data_prefix_pub(self,prefix):
        """Publish a filename prefix string to the remote node's save data prefix topic."""
        pub_name = 'prefix'
        msg = prefix
        self.con_node_if.publish_pub(pub_name,msg)

    def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        """Publish a save rate command in Hz for the specified data product."""
        pub_name = 'rate'
        msg = SaveDataRate()
        msg.data_product = data_product
        msg.save_rate_hz = rate_hz
        self.con_node_if.publish_pub(pub_name,msg)

    def snapshot_pub(self):
        """Publish a snapshot trigger to capture a single save of all active data products."""
        pub_name = 'snapshot'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def reset_pub(self):
        """Publish a reset command to clear save data state on the remote node."""
        pub_name = 'reset'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def factory_reset_pub(self):
        """Publish a factory reset command to restore save data defaults on the remote node."""
        pub_name = 'factory_reset'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def unregister_save_data_if(self):
        """Unregister all publishers and subscribers for this save data interface and mark it not ready."""
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
        """Publish a save config command to persist current parameters to disk."""
        self.con_node_if.publish_pub('save_config',Empty())

    def call_reset_config(self):
        """Publish a reset config command to reload parameters from the last saved configuration."""
        self.con_node_if.publish_pub('reset_config',Empty())

    def call_factory_reset_config(self):
        """Publish a factory reset config command to restore all parameters to factory defaults."""
        self.con_node_if.publish_pub('factory_reset_config',Empty())

    ###############################
    # Class Private Methods
    ###############################

    def _statusCb(self,msg):
        self.status_msg = msg
        self.status_connected = True




# Settings are controls dicts now, so setting types are the control types
SETTING_TYPES = nepi_controls.CONTROL_TYPES

'''
EXAMPLE_CAP_SETTINGS_DICT = {"None":{"name":"None","type":"None","options":[]}}
EXAMPLE_SETTINGS_DICT = {"None":{"name":"None","type":"None","value":"None"}}
'''

class ConnectSettingsIF:
    """Client side of a remote node's SettingsIF.

    Settings are published as a nepi_controls ControlsStatus message, so both
    the current values and their capabilities (type, options, bounds, default)
    arrive in the same message. There is no capabilities query service to call.
    """

    msg_if = None
    ready = False
    namespace = '~'

    controls_dict = dict()
    settings_dict = None
    cap_settings_dict = None
    has_cap_updates = False

    #######################
    ### IF Initialization
    def __init__(self, namespace , timeout = float('inf'),
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
        self.msg_if.pub_info("Starting Settings IF Initialization Processes" )
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
        # No capabilities_query. The capability report arrives in the status
        # message, so there is nothing to call.
        self.SRVS_DICT = None

        # Pubs Config Dict ####################
        # 'reset_settings' is the topic SettingsIF subscribes; the previous
        # 'reset'/'factory_reset' topics had no subscriber on the far end.
        self.PUBS_DICT = {
            'setting_update_pub': {
                'msg': UpdateControl,
                'namespace': self.namespace,
                'topic': 'update_setting',
                'qsize': 1,
                'latch': False
            },
            'reset_pub': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset_settings',
                'qsize': 1,
                'latch': False
            }
        }


        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'msg': ControlsStatus,
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
                                            
                                            msg_if = self.msg_if
                                            )

        #self.con_node_if.wait_for_ready()
        nepi_sdk.wait()

        ##############################
        # Complete Initialization
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of this settings interface."""
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout elapses, then return the ready state."""
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection" )
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect" )
            else:
                self.msg_if.pub_info("Connected" )
        return self.ready

    def get_namespace(self):
        """Return the fully resolved ROS namespace for this settings interface."""
        return self.namespace

    def get_settings_capabilities_dict(self):
        """Return the capabilities dictionary describing available settings and their types.

        Read from the last status message, not from a service call.

        Returns:
            dict: setting name -> {'name', 'type', 'options', 'default_value'},
                or None if no status has arrived yet.
        """
        return self.cap_settings_dict

    def get_has_capabilities_updating(self):
        """Return True if the remote node supports runtime capability setting updates.

        Always False: the runtime cap-update topic was retired along with the
        capabilities query service. Capability changes are made node-side and
        appear in the next status message.
        """
        return self.has_cap_updates

    def get_settings_dict(self):
        """Return the current settings dictionary as last received from the remote node.

        Returns:
            dict: setting name -> {'name', 'type', 'value'}, or None if no
                status has arrived yet.
        """
        return self.settings_dict

    def get_controls_dict(self):
        """Return the raw controls dict parsed from the last status message.

        Returns:
            dict: control name -> control dict, in nepi_controls form.
        """
        return copy.deepcopy(self.controls_dict)

    def update_cap_setting(self, cap_setting_dict):
        """Publish a capability setting update if the remote node supports it, and return success.

        Always returns False. Kept so a caller written against the old client
        still resolves; the remote cap-update topic no longer exists.

        Args:
            cap_setting_dict (dict): Ignored.

        Returns:
            bool: False.
        """
        self.msg_if.pub_warn("Runtime cap setting updates are retired; ignoring " + str(cap_setting_dict) )
        return False

    def update_setting(self, setting_dict):
        """Publish a setting update to the remote node and return success.

        Args:
            setting_dict (dict): {'name', 'type', 'value'} with a string value,
                in the device settings form.

        Returns:
            bool: True if the message was published.
        """
        string_options = []
        try:
            name = setting_dict['name']
            if name in self.controls_dict.keys():
                string_options = self.controls_dict[name]['string_options']
        except Exception as e:
            self.msg_if.pub_warn("Failed to read options for setting " + str(setting_dict) + " : " + str(e) )
        msg = nepi_controls.create_update_control_msg_from_setting(setting_dict, string_options)
        success = self.con_node_if.publish_pub('setting_update_pub',msg)
        return success

    def reset_settings(self):
        """Publish a reset command to restore all settings to their last saved values."""
        msg = Empty()
        success = self.con_node_if.publish_pub('reset_pub',msg)
        return success

    def factory_reset_settings(self):
        """Publish a factory reset command to restore all settings to their factory defaults.

        SettingsIF exposes factory reset through the node config surface
        (factory_reset_config), not through its own settings namespace, so this
        routes there.
        """
        self.call_factory_reset_config()
        return True

     #################
    ## Save Config Functions

    def call_save_config(self):
        """Publish a save config command to persist current parameters to disk."""
        self.con_node_if.publish_pub('save_config',Empty())

    def call_reset_config(self):
        """Publish a reset config command to reload parameters from the last saved configuration."""
        self.con_node_if.publish_pub('reset_config',Empty())

    def call_factory_reset_config(self):
        """Publish a factory reset config command to restore all parameters to factory defaults."""
        self.con_node_if.publish_pub('factory_reset_config',Empty())

    ###############################
    # Class Private Methods
    ###############################

    def _settingsCb(self,msg):
        # One message carries both the values and their capabilities.
        controls_dict = nepi_controls.parse_controls_status_msg(msg)
        self.controls_dict = controls_dict
        self.settings_dict = nepi_controls.get_settings_from_controls_dict(controls_dict)
        self.cap_settings_dict = nepi_controls.get_cap_settings_from_controls_dict(controls_dict)
        self.ready = True






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
        self.msg_if.pub_info("Starting States IF Initialization Processes" )
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
                'msg': SystemStates,
                'topic': 'system_states_status',
                'qsize': 1,
                'callback': self._statusSubCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(subs_dict = self.SUBS_DICT,
                                            
                                            msg_if = self.msg_if
                                            )

        #self.con_node_if.wait_for_ready()
        nepi_sdk.wait()

        ##############################
        # Complete Initialization
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of this states interface."""
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout elapses, then return the ready state.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready before the timeout, False otherwise.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection" )
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and self.status_msg is None and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect" )
            else:
                self.msg_if.pub_info("Connected" )
        return self.ready


    def get_states_names(self):
        """Return the list of state names from the most recent system states status message.

        Returns:
            list: State name strings, or an empty list if no status has been received.
        """
        state_names = []
        if self.status_msg is not None:
                state_names = self.status_msg.states_names_list
        else:
            self.msg_if.pub_warn(self,":" + self.class_name + ": state Status listener has not received any data yet: " )
        return state_names


    def get_states_status_dict(self):
        """Return the current system states as a parsed dictionary.

        Returns:
            dict: Mapping of state name to state dict, or None if no status has been received.
        """
        states_dict = None
        if self.status_msg is not None:
                states_dict = nepi_states.parse_trigger_status_msg(self.status_msg)
        else:
            self.msg_if.pub_warn(self,":" + self.class_name + ": state Status listener has not received any data yet: " )
        return states_dict


    def get_state_value(self, state_name):
        """Return the current value for a named state, or None if unavailable.

        Args:
            state_name (str): The name of the state to retrieve.

        Returns:
            object: The state value extracted from the state dict, or None if the state
                name is not found or no status has been received.
        """
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
        self.msg_if.pub_info("Starting Triggers IF Initialization Processes" )
        ############################## 
        

        #############################
        # Initialize Class Variables

        ##############################  
        # Create NodeClassIF Class  

        # Subs Config Dict ####################
        self.SUBS_DICT = {
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
                                            
                                            msg_if = self.msg_if
                                            )

        #self.con_node_if.wait_for_ready()
        nepi_sdk.wait()

        ##############################
        # Complete Initialization
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of this triggers interface."""
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout elapses, then return the ready state.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready before the timeout, False otherwise.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection" )
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and self.status_msg is None and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect" )
            else:
                self.msg_if.pub_info("Connected" )
        return self.ready


    def get_trigger_names(self):
        """Return the list of trigger names from the most recent system triggers status message.

        Returns:
            list: Trigger name strings, or an empty list if no status has been received.
        """
        trigger_names = []
        if self.status_msg is not None:
            trigger_names = self.status_msg.triggers_name_list
        else:
            self.msg_if.pub_warn(self,":" + self.class_name + ": Trigger Status listener has not received any data yet: " )
        return trigger_names


    def get_registered_triggers(self):
        """Return the names of all currently registered trigger handlers.

        Returns:
            KeysView: The trigger name keys from the internal handler dictionary.
        """
        return self.trigger_handlers_dict.keys()

    def register_trigger(self, trigger_handler_function, trigger_name = "All"):
        """Register a callback function to be invoked when the named trigger fires.

        Args:
            trigger_handler_function (callable): Function to call when the trigger fires.
                It will receive a trigger dict as its only argument.
            trigger_name (str, optional): Name of the specific trigger to listen for.
                Pass "All" to receive every trigger event. Defaults to "All".

        Returns:
            bool: True if registration succeeded.
        """
        success = False
        th_dict = dict()
        th_dict['handler'] = trigger_handler_function
        th_dict['last_time'] = nepi_utils.get_time()
        self.trigger_handlers_dict[trigger_name] = th_dict
        success = True
        return success

    def unregister_trigger(self, trigger_name):
        """Remove the registered handler for the named trigger.

        Args:
            trigger_name (str): Name of the trigger whose handler should be removed.

        Returns:
            bool: True if the handler was removed, False if no handler was registered
                for that trigger name.
        """
        success = False
        try:
            del self.trigger_handlers_dict[trigger_name]
            success = True
        except Exception as e:
            self.msg_if.pub_warn("No trigger handler registed for trigger: " + trigger_name + " " + str(e))
        return success


    def get_last_trigger_time(self, trigger_name):
        """Return the timestamp of the most recent firing of the named trigger.

        Args:
            trigger_name (str): Name of the registered trigger to query.

        Returns:
            float: ROS time of the last trigger event, or None if no handler is
                registered for the given trigger name.
        """
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

