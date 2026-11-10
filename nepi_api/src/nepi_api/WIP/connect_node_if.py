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
import threading
from pathlib import Path

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav


from std_msgs.msg import String, Empty

from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyRequest as EmptySrvRequest
from std_srvs.srv import EmptyResponse as EmptySrvResponse




from nepi_interfaces.msg import ConnectNodeIFStatus, MgrSystemStatus
from nepi_interfaces.srv import ConnectNodeIFStatusQuery, ConnectNodeIFStatusQueryRequest, ConnectNodeIFStatusQueryResponse

from nepi_interfaces.msg import Messages, SettingsStatus, SaveDataStatus, TransformStatus, NavPoseStatus
from nepi_interfaces.msg import SystemStatesStatus, SystemTriggersStatus
from nepi_interfaces.msg import Transform, NavPose, SystemStates, SystemTriggers

from nepi_interfaces.msg import Value, ValuesDict, TopicDict, TopicsDict, ControlGroup, ControlGroupsDict


from nepi_interfaces.msg import Reset

from nepi_api.messages_if import MsgIF

# Reuse the full node config/param machinery so NodeClassIF can persist
# params (e.g. the connect selected_topic) via the config manager, exactly the
# way NodeClassIF does. node_if.py does not import this module, so there is no
# circular import.
# from nepi_api.node_if import NodeConfigsIF, NodeParamsIF
from nepi_api.node_if import NodeIF




#########################################
# Connect Node IF Class
#########################################

#################
# node_if Configs Updates
#################

# Overrides _initCb(self, doUpdates = False)

#################
# node_if Params Updates
#################
## Creates or Adds the following params to node_if,
## depending on whether a node_if is passed in the system_if_dict

# 'selected_topic' - self.selected_topic

#################
# node_if Services Updates
#################
## Creates or Adds the following services-callback_functions to node_if,
## depending on whether a node_if is passed in the system_if_dict
## The callback functions can be overrided by Child Class

# NONE

#################
# node_if Publishers Updates
#################
## Creates or Adds the following publishers to node_if,
## depending on whether a node_if is passed in the system_if_dict

# 'status_pub' - Msg class updated to ConnectNodeIFStatus

#################
# node_if Subscribers Updates
#################
## Creates or Adds the following subscribers-callback_functions to node_if,
## depending on whether a node_if is passed in the system_if_dict
## The callback functions can be overrided by Child Class

# self.namespace/set_connect_namespace - self._setNamespaceCb
# self.namespace/select_topic - self._selectTopicCb


#############################################################

#################
# Node IF Function Overrides
#################

# self.get_if_status_msg = Change to update class values based on connection
# self.get_pub_status_msg - Changed to return the ConndectNodeIFStatus message
# self._nodeIFUpdater = Change to update class values based on connection
#############################################################

#################
# Info UPDATES
#################

UPDATE_IF_INFO_DICT = dict()

#################
# Capabilities UPDATES
#################
UPDATE_IF_CAPS_DICT = dict()

#################
# Config  UPDATES
#################

UPDATE_CONFIG_CONTROLS_DICT = {}

UPDATE_CONFIG_DATA_DICT = {}

UPDATE_IF_CONFIG_DICT = dict(
    name = '',
    id = '',

    status_msg_type = '',
    data_msg_type = '',
    data_rate = 0,

    available_topics = [],
    available_names = [],
    auto_select_enabled = True,

    selected_topic = '',
    selected_name = '',
    connecting = False,
    connected = False,
    connected_topic = '',
    connect_msg = '',
)




#################
# Controls Groups UPDATES
#################

'''
########################################
##### EXAMPLE UPDATE_CONTROL_GROUPS_DICT
self.none_config_dict = dict(
    none_key = 'None'
)

none_control_group = dict(
    name = 'none_control',
    display_name = 'None Control',
    controls_if = self.none_controls_if,
    data_if = None,
    config_dict = self.none_config_dict
)

UPDATE_CONTROL_GROUP_DICT = dict(
    none_control = none_control_group
)
########################################
'''


#################
# Stats UPDATES
#################

UPDATE_IF_STATS_DICT = dict()


#################
# RUI UPDATES
#################

UPDATE_IF_RUI_DICT = dict()
UPDATE_IF_RUI_DICT['show_namespaces'] = True
UPDATE_IF_RUI_DICT['show_selector'] = True


UPDATE_IF_CALLBACK_DICT = dict()




class ConnectNodeIF(NodeIF):

    CONNECTED_TIMEOUT = 2

    if_status_msg = ConnectNodeIFStatus()
    if_status_msg_class = ConnectNodeIFStatus
    if_query_class = ConnectNodeIFStatusQuery
    if_request_class = ConnectNodeIFStatusQueryRequest
    if_response_class = ConnectNodeIFStatusQueryResponse

    if_ready = False


    init_config_controls_dict = copy.deepcopy(UPDATE_CONFIG_CONTROLS_DICT)
    init_config_data_dict = copy.deepcopy(UPDATE_CONFIG_DATA_DICT)
    ######

    connect_namespace = ''
    connect_id = None
    connect_status_msg = None
    connect_name = None

    filter_topics_list = []
    exclude_namespaces_list = []
    available_topics = []
    available_names = []
    auto_select_enabled = True

    selected_topic_param = 'None'
    selected_topic = "None"
    connecting = False
    connected = False
    connected_topic = 'None'
    connect_msg = 'Not Selected'

    connect_node_pubs_dict = None
    connect_node_subs_dict = None

    #######################
    ### IF Initialization
    def __init__(self, 
                name = '',
                namespace = None,
                status_msg = None,
                data_msg = None,
                params_dict = None,
                srvs_dict = None,
                pubs_dict = None,
                subs_dict = None,
                info_dict = None, # self.info_dict can be created/updated later
                caps_dict = None, # self.caps_dict can be created/updated later
                config_controls_dict = None, # self.config_controls_if can be created/updated later
                config_data_dict = None, # self.config_data_if can be created/updated later
                config_dict = None, # self.config_dict can be created/updated later
                control_groups_dict = None, # self.stats_dict dict can be created/updated later
                stats_dict = None, # self.config_controls_if can be created/updated later
                rui_dict = None, # self.rui_dict can be created/updated later
                callback_dict = None, # Use class set_callback and clear_callback functions to update later
                system_if_dict = None, # self.system_if_dict can be created/updated later
                ):


        ########################################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        if name is None:
            name = ''
        if name.strip() == '':
            name = 'nepi_node'
        name = nepi_sdk.get_clean_name(str(name))
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,name)
        self.node_if_prefix = self.namespace.replace(self.node_namespace + '/','') + '/'
        ##############################  

        
        update_rui_dict = copy.deepcopy(UPDATE_IF_RUI_DICT)

        ########################################

        ##################
        update_info_dict = copy.deepcopy(UPDATE_IF_INFO_DICT)
        try:
            update_info_dict.update(info_dict)
        except:
            pass

        ##################
        update_caps_dict = copy.deepcopy(UPDATE_IF_CAPS_DICT)
        try:
            update_caps_dict.update(caps_dict)
        except:
            pass

        ##################
        update_config_dict = copy.deepcopy(UPDATE_IF_CONFIG_DICT)
        try:
            update_config_dict.update(config_dict)
        except:
            pass

        ###################
        update_control_groups_dict = control_groups_dict

        ##################
        update_stats_dict = copy.deepcopy(UPDATE_IF_STATS_DICT)
        try:
            update_stats_dict.update(stats_dict)
        except:
            pass

        ##################
        update_rui_dict = copy.deepcopy(UPDATE_IF_RUI_DICT)
        try:
            update_rui_dict.update(rui_dict)
        except:
            pass

        ##################
        update_callback_dict = copy.deepcopy(UPDATE_IF_CALLBACK_DICT)
        try:
            update_callback_dict.update(callback_dict)
        except:
            pass


     
        #######################
        # Params Config Dict ####################
        update_params_dict = {
            self.node_if_prefix + 'selected_topic': {
                'namespace': self.connect_namespace,
                'factory_val': self.selected_topic
            }
        }
        if params_dict is not None:
            try:
                update_params_dict.update(params_dict)
            except:
                pass


        #######################
        # Services Config Dict ####################
        update_srvs_dict = {}
        if srvs_dict is not None:
            try:
                update_srvs_dict.update(srvs_dict)
            except:
                pass

       

        #######################
        # Services Config Dict ####################
        update_pubs_dict = {
            self.node_if_prefix + 'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': self.if_status_msg_class,
                'qsize': 1,
                'latch': True
            }
        }
        if pubs_dict is not None:
            try:
                update_pubs_dict.update(pubs_dict)
            except:
                pass

        #######################
        # Services Config Dict ####################
        update_subs_dict =  {
            self.node_if_prefix + 'set_connect_namespace': {
                'namespace': self.namespace,
                'topic': 'set_connect_namespace',
                'msg': String,
                'qsize': None,
                'callback': self._setNamespaceCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'select_topic': {
                'namespace': self.namespace,
                'topic': 'select_topic',
                'msg': String,
                'qsize': None,
                'callback': self._selectTopicCb, 
                'callback_args': ()
            }
        }


        if subs_dict is not None:
            try:
                update_subs_dict.update(subs_dict)
            except:
                pass


        # self.node_subs_dict 



        ###########
        # NODE IF SETUP

       # Call the parent class constructor
        super().__init__(name = name,
                namespace = namespace,
                status_msg = status_msg,
                data_msg = data_msg,
                params_dict = update_params_dict,
                srvs_dict = update_srvs_dict,
                pubs_dict = update_pubs_dict,
                subs_dict = update_subs_dict,
                info_dict = update_info_dict,
                caps_dict = update_caps_dict, 
                config_controls_if = None, 
                config_data_if = None, 
                config_dict = update_config_dict, 
                control_groups_dict = update_control_groups_dict, 
                stats_dict = update_stats_dict, 
                rui_dict = update_rui_dict, 
                callback_dict = update_callback_dict, 
                system_if_dict = system_if_dict
                )


        nepi_sdk.sleep(1)


        ##############################
        #### Create Config IFs

        if config_controls_dict is not None:
            try:
                self.init_config_controls_dict.update(config_controls_dict)
            except:
                pass
        self.config_controls_if = ControlsIF()

        if config_data_dict is not None:
            try:
                self.init_config_data_dict.update(config_data_dict)
            except:
                pass
        self.config_controls_if = DataIF()




        # ########################################
        # # Setup Connect Navpose Settings
        # self.connect_navpose == connect_navpose
        # self.clear_navpose = clear_navpose
        # self.callback_dict['navpose'] = navpose_callback

        # ########################################
        # # Setup Connect Settings

        # # Check ID Class
        # if connect_id is None:
        #     self.msg_if.pub_debug("Connect ID Not Provided") 
        #     return 
        # self.connect_id = connect_id



        # self.connect_name = nepi_utils.get_clean_name(connect_name)
        # if self.connect_name is None or self.connect_name == '':
        #     self.msg_if.pub_debug("Topic Name Not Valid: " + str(connect_name)) 
        #     return
        # self.msg_if.pub_debug("Using Topic Name: " + self.connect_name)


        # # Set Connect Namespace
        # if connect_namespace is None:
        #     connect_namespace = self.base_namespace
        # self.connect_namespace = nepi_sdk.get_full_namespace(connect_namespace)        
        
        # ##############################    
        # # Initialize Class Variables

        # if selected_topic is None:
        #     selected_topic = "None"
        # if selected_topic != "None":
        #     sselected_topic = nepi_sdk.get_full_namespace(selected_topic)
        # if selected_topic == '' or selected_topic == '/':
        #     selected_topic = "None"
        # self.selected_topic = selected_topic

        # self.auto_select_enabled = auto_select_enabled
        # self.msg_if.pub_debug("Auto select enabled: " + str(self.auto_select_enabled))

        # self.show_namespaces = show_namespaces
        # self.show_selector = show_selector
        # self.show_controls = show_controls
        # self.show_data = show_data

        # self.filter_topics_list = filter_topics_list

        # # Namespaces discovery never offers. A node that HOSTS a device of the
        # # same type it connects to (an app that builds an RBX, which publishes
        # # its own NPX, beside an NPX connect) passes its own node namespace here
        # # so its selector cannot pick the node's own device.
        # self.exclude_namespaces_list = []
        # if exclude_namespaces_list is not None:
        #     for exclude_namespace in exclude_namespaces_list:
        #         exclude_namespace = str(exclude_namespace).rstrip('/')
        #         if exclude_namespace != '':
        #             self.exclude_namespaces_list.append(exclude_namespace)


        # self.connect_topic_controls_dict = connect_topic_controls_dict

        # ################################################
        # # Restore any persisted selection. When no explicit topic was requested
        # # (selected_topic == "None"), use the value the config manager restored
        # # for this connect namespace. Otherwise honor the explicit request.
        # self.selected_topic_param = self.node_if_prefix + 'selected_topic'
        # if selected_topic == "None":
        #     persisted = self.node_if.get_param(self.selected_topic_param)
        #     if persisted is not None and persisted != '' and persisted != "None":
        #         selected_topic = persisted
        # self.selected_topic = selected_topic
        # self.msg_if.pub_debug("Init Selected Topic: " + str(self.selected_topic))


        ##############################
        # Start updater process
        nepi_sdk.start_timer_process(1.0, self._nodeIFUpdater, oneshot = True)

        ##############################
        # Complete Initialization

        self.if_ready = True
        self.msg_if.pub_debug(str(self.class_name) + " Initialization Complete")
        ###############################
    

    #######################
    # Class Public Methods
    #######################



    # ###############################

    # def get_available_namespaces(self):
    #     return self.available_namespaces

            
    # def get_connect_namespace(self):
    #     return self.connect_namespace

    
    # def set_connect_namespace(self, connect_namespace):
    #     connect_namespace = nepi_sdk.get_full_namespace(connect_namespace)
    #     self.connect_namespace = connect_namespace
    #     self.publish_status()
   
    

    # ###############################
    # def get_available_topics(self):
    #     return self.available_topics

    # def set_auto_connect_enable(self, enabled):
    #     self.auto_select_enabled = enabled
            
    # def get_selected_topic(self):
    #     return self.selected_topic

    
    # def set_selected_topic(self, selected_topic):
    #     # An undiscovered topic cannot be selected. Reject it WITHOUT persisting:
    #     # the old code fell through and wrote the param plus save_config() even
    #     # though self.selected_topic was left unchanged. Callers that re-assert a
    #     # derived selection every cycle (applyDerivedTargetsImageSelection in the
    #     # obstacles app) guard on get_selected_topic(), which never reports the
    #     # rejected value, so each cycle re-entered here -- hammering the param
    #     # server and rewriting the config file at 1 Hz, and filling the log with
    #     # "Failed to set param ... Connection refused" whenever the master was
    #     # down. Rejection must be cheap and quiet; the caller's re-assert loop is
    #     # intentional and self-corrects once the topic is discovered.
    #     if selected_topic not in self.available_topics and selected_topic != "None":
    #         self.msg_if.pub_debug("Ignoring unavailable selected_topic: " + str(selected_topic),
    #                               throttle_s = 5.0)
    #         self.publish_status()
    #         return
    #     self.selected_topic = selected_topic
    #     self.publish_status()
    #     # Persist the selection so it survives a node restart. set_param writes
    #     # the ROS param; save_config asks the config manager to save it to file.
    #     if self.node_if is not None:
    #         self.msg_if.pub_debug("selected_topic: " + str(selected_topic))
    #         self.node_if.set_param(self.node_if_prefix + 'selected_topic', self.selected_topic)
    #         self.node_if.save_config()
    

    # def check_connection(self):
    #     """Check whether the device is currently connected.

    #     Returns:
    #         bool: True if a status message has been received within the connection timeout window,
    #             False otherwise.
    #     """
    #     return self.connected


    # def get_connected_topic(self):
    #     topic = ''
    #     if self.check_connection() == True:
    #         topic = self.selected_topic
    #     return topic

    # def wait_for_connection(self, timeout = float('inf') ):
    #     """Block until the device is connected or the timeout expires.

    #     Args:
    #         timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

    #     Returns:
    #         bool: True if connection was established, False if the timeout was reached.
    #     """
    #     if self.node_if is not None:
    #         self.msg_if.pub_debug("Waiting for connection")
    #         timer = 0
    #         time_start = nepi_sdk.get_time()
    #         while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
    #             nepi_sdk.sleep(.1)
    #             timer = nepi_sdk.get_time() - time_start
    #         if self.connected == False:
    #             self.msg_if.pub_debug("Failed to Connect")
    #         else:
    #             self.msg_if.pub_debug("Connected")
    #     return self.connected


    # def subscribe_topic(self):
    #     pass

    # def unsubscribe_topic(self):
    #     pass


    # def get_available_path_names(self, available_topics = []):
    #     """Label each available topic with its node name AND its data product.

    #     Args:
    #         available_topics (list): Fully-qualified topic namespaces to label.

    #     Returns:
    #         list: One label per entry in available_topics, in the same order.
    #             Falls back to the full topic for a namespace too shallow to hold
    #             a node name.
    #     """
    #     available_names = []
    #     for topic in available_topics:
    #         name = topic
    #         topic_split = topic[1:].split('/')
    #         if len(topic_split) > 2:
    #             path_tokens = []
    #             for token in topic_split[2:]:
    #                 if token not in IF_NAMESPACE_TOKENS:
    #                     path_tokens.append(token)
    #             if len(path_tokens) > 0:
    #                 name = '/'.join(path_tokens)
    #         available_names.append(name)
    #     return available_names

    ###############################
    def get_data_dict(self):
        """Retrieve and consume the latest captured pointcloud data dictionary.

        Thread-safe. Clears the stored data after retrieval so subsequent calls
        return None until a new frame arrives.

        Returns:
            dict: The pointcloud data dictionary containing the Open3D point cloud,
                point count, timestamps, and latency metrics, or None if no
                pointcloud is available.
        """
        self.data_lock.acquire()
        data_dict = copy.deepcopy(self.data_dict)
        if self.clear_data == True:
            self.data_dict = None
        self.data_lock.release()
        return data_dict


    ###############################
    def get_navpose_namespace(self):
        return self.system_topics['navpose']['topic']
    
    def get_navpose_msg(self):
        """Return a deep copy of the most recently received nav pose dictionary.
        Returns:
            dict: The last nav pose data dictionary, or None if no NavPose message
                has arrived from the connected source yet.
        """
        navpose_msg = copy.deepcopy(self.navpose_msg)

        if navpose_msg is None:
            navpose_msg = NavPose()
        return navpose_msg

    def get_navpose_dict(self):
        """Return a deep copy of the most recently published nav pose dictionary.

        Returns:
            dict: The last nav pose data dictionary.
        """
        self.navpose_lock.acquire()
        navpose_dict = copy.deepcopy(self.navpose_dict)
        if self.clear_navpose == True:
            self.navpose_dict = None
        self.navpose_lock.release()

        if navpose_dict is None and self.clear_navpose == False:
            navpose_dict = nepi_sdk.convert_msg2dict(NavPose())

        return navpose_dict

    

    # #################
    # ## Save Data Functions

    # def get_save_data_products(self):
    #     """Return the list of available save data products for this data source.

    #     Returns:
    #         list: A list of data product identifiers supported by the save data interface.
    #     """
    #     data_products = []      
    #     if self.save_data_connect_if is not None:
    #         data_products = self.save_data_connect_if.get_data_products()
    #     return data_products

    # def get_save_data_status_dict(self):
    #     """Return the current save data status as a dictionary.

    #     Returns:
    #         dict: A dictionary representation of the save data interface status.
    #     """
    #     status_dict = SaveDataStatus()
    #     if self.save_data_connect_if is not None:
    #         status_dict = self.save_data_connect_if.get_status_dict()
    #     return status_dict

    # def save_data_enable_pub(self,enable):
    #     """Enable or disable data saving on the data source.

    #     Args:
    #         enable (bool): True to enable data saving, False to disable it.
    #     """
       
    #     if self.save_data_connect_if is not None:
    #         self.save_data_connect_if.save_data_pub(enable)

    # def save_data_prefix_pub(self,prefix):
    #     """Publish an updated filename prefix for saved data files.

    #     Args:
    #         prefix (str): The prefix string to prepend to saved data filenames.
    #     """
      
    #     if self.save_data_connect_if is not None:
    #         self.save_data_connect_if.save_data_prefix_pub(prefix)

    # def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
    #     """Publish an updated save rate for a data product.

    #     Args:
    #         rate_hz (float): Desired save rate in Hz.
    #         data_product (int, optional): Identifier for the specific data product to update.
    #             Defaults to SaveDataRate.ALL_DATA_PRODUCTS.
    #     """
    #     if self.save_data_connect_if is not None:
    #         self.save_data_connect_if.publish_pub(rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS)

    # def save_data_snapshot_pub(self):
    #     """Trigger a one-shot snapshot save of current data on the data source.
    #     """
    #     if self.save_data_connect_if is not None:
    #         self.save_data_connect_if.publish_pub()

    # def save_data_factory_reset_pub(self):
    #     """Publish a factory reset command to restore the save data configuration to defaults.
    #     """
    #     if self.save_data_connect_if is not None:
    #         pub_name = 'factory_reset'
    #         msg = Empty()
    #         self.save_data_connect_if.publish_pub(pub_name,msg)




    #######################
    # Node Class Public Override Methods
    #######################
 
    def get_if_status_msg(self):
        #####################################
        self.if_status_msg.info_dict = nepi_sdk.convert_values_dict2msg(self.info_dict)

        #####################################
        self.if_status_msg.caps_dict = nepi_sdk.convert_values_dict2msg(self.caps_dict)

        #####################################
        self.if_status_msg.config_controls_topic = self.config_controls_topic
        self.if_status_msg.config_data_topic = self.config_data_topic
        self.if_status_msg.config_dict = nepi_sdk.convert_values_dict2msg(self.config_dict)

        #####################################
        control_groups_msg = ControlGroupsDict()
        key_names = []
        entries = []
        if self.control_groups_dict is not None:
            for key in self.control_groups_dict.keys:
                
                entry_msg = ControlGroup()

                try:
                    name = self.control_groups_dict[key]['name']
                except:
                    name = str(key)
                entry_msg.name = name

                try:
                    entry_msg.controls_topic = self.control_groups_dict[key]['controls_if'].get_namespace()
                except:
                    pass

                try:
                    entry_msg.data_topic = self.control_groups_dict[key]['data_if'].get_namespace()
                except:
                    pass

                try:
                    entry_dict = self.control_groups_dict[key]['config_dict']
                    entry_msg.data_topic = nepi_sdk.convert_values_dict2msg(entry_dict)
                except:
                    pass

                key_names.append(str(key))
                entries.append(entry_msg)
        control_groups_msg.key_names = key_names
        control_groups_msg.entries = entries
        self.if_status_msg.control_groups_dict = control_groups_msg
        

        #####################################
        self.if_status_msg.node_topic_dict = nepi_sdk.convert_topic_dict2msg(self.node_topic_dict)
        self.if_status_msg.system_topics_dict = nepi_sdk.convert_topic_dict2msg(self.system_topics_dict)
        self.if_status_msg.data_product_topics_dict = nepi_sdk.convert_topic_dict2msg(self.data_product_topics_dict)

        #####################################
        self.if_status_msg.stats_dict = nepi_sdk.convert_values_dict2msg(self.stats_dict)

        self.if_status_msg.rui_dict = nepi_sdk.convert_values_dict2msg(self.rui_dict)

        return self.status_msg




     


    #     # status_msg.settings_topic = self.system_topics['settings']['topic']
    #     # status_msg.settings_connected = self.system_topics['settings']['connected']

    #     # status_msg.save_data_topic = self.system_topics['save_data']['topic']
    #     # status_msg.save_data_connected = self.system_topics['save_data']['connected']

    #     # status_msg.transform_topic = self.system_topics['transform']['topic']
    #     # status_msg.transform_connected = self.system_topics['transform']['connected']

    #     # status_msg.navpose_topic = self.system_topics['navpose']['topic']
    #     # status_msg.navpose_connected = self.system_topics['navpose']['connected']

    #     # status_msg.states_topic = self.system_topics['states']['topic']
    #     # status_msg.states_connected = self.system_topics['states']['connected']

    #     # status_msg.triggers_topic = self.system_topics['triggers']['topic']
    #     # status_msg.triggers_connected = self.system_topics['triggers']['connected']

    #     # status_msg.triggers_topic = self.system_topics['status']['topic']
    #     # status_msg.triggers_connected = self.system_topics['status']['connected']

    #     # status_msg.triggers_topic = self.system_topics['data']['topic']
    #     # status_msg.triggers_connected = self.system_topics['data']['connected']




    #     ###########
    #     if self.node_if is not None:
    #         if self.status_has_published == False:
    #             #self.msg_if.pub_debug("Publishing Status: " + str(status_msg))
    #             self.status_has_published = True
    #         self.node_if.publish_pub(self.node_if_prefix + 'status_pub', status_msg) 
    #         #self.node_if.save_config()



    def get_pub_status_msg(self):
        return self.get_if_status_msg()
    
    #######################
    # Class Private Methods
    #######################
    def _initCb(self,do_updates = False):
      if self.node_if is not None:

        # Restore any persisted selection. When no explicit topic was requested
        # (selected_topic == "None"), use the value the config manager restored
        # for this connect namespace. Otherwise honor the explicit request.
        self.selected_topic_param = self.node_if_prefix + 'selected_topic'
        if selected_topic == "None":
            persisted = self.node_if.get_param(self.selected_topic_param)
            if persisted is not None and persisted != '' and persisted != "None":
                selected_topic = persisted
        self.selected_topic = selected_topic
        self.msg_if.pub_debug("Init Selected Topic: " + str(self.selected_topic))

        try:
            self.initCb(do_updates)
        except:
            try:
                self.initCb()
            except:
                pass
      if do_updates == True:
        pass
      self.publish_status()




    def _setNamespaceCb(self,msg):
        namespace = msg.data
        self.set_connect_namespace(namespace)


    def _selectTopicCb(self,msg):
        topic = msg.data
        self.set_selected_topic(topic)

    # # Announce a new connection ONCE per topic, from every subclass's _statusCb.
    # #
    # # Each subclass's status callback runs its announcement inside
    # # 'if self.connected == False:', which reads like a first-connection guard but
    # # is not one: the staleness check in _nodeIFUpdater clears self.connected whenever
    # # no status arrives inside CONNECTED_TIMEOUT, without touching the
    # # subscriptions. Any source publishing its status slower than that timeout --
    # # or any source whose status stalls briefly -- therefore re-enters that branch
    # # on every message and re-announces the same connection at warn level forever.
    # #
    # # The remembered value is the announced TOPIC rather than a bare once-only
    # # flag, because connecting to a DIFFERENT topic is still news worth a warn:
    # # these IFs are re-pointed at runtime, by an operator's selector or by a
    # # consumer node driving set_selected_topic() (nepi_app_obstacles derives all
    # # five of its selections that way). It is deliberately never cleared, so a
    # # deselect and reselect of the same topic stays quiet.
    # #
    # # Reconnects still log, at debug, so the churn is visible when it is being
    # # looked for.
    def _announceConnected(self, connect_label):
        if self.selected_topic != self.announced_connected_topic:
            self.announced_connected_topic = self.selected_topic
            self.msg_if.pub_debug("Connected to " + str(connect_label) + " Status:  " + str(self.selected_topic))
        else:
            self.msg_if.pub_debug("Reconnected to " + str(connect_label) + " Status:  " + str(self.selected_topic))
    


    # True when topic is an exclude_namespaces_list namespace or sits below one.
    # Matches whole namespace levels, so excluding /nepi/dev/app_robot does not
    # also hide /nepi/dev/app_robot2.
    def _isExcludedTopic(self, topic):
        for exclude_namespace in self.exclude_namespaces_list:
            if topic == exclude_namespace or topic.startswith(exclude_namespace + '/'):
                return True
        return False


    def _nodeIFUpdater(self,timer):
        needs_publish = False
        ##############

        #########################
        # Update Config Data
        #########################
        
        try:
            config_controls_topic = self.config_controls_if.get_namespace()
        except:
            config_controls_topic = ''
        if config_controls_topic is None:
            config_controls_topic = ''
        self.config_controls_topic = config_controls_topic

        try:
            config_data_topic = self.config_data_if.get_namespace()
        except:
            config_data_topic = ''
        if config_data_topic is None:
            config_data_topic = ''
        self.config_data_topic = config_data_topic

        ###################################################

        # An excluded selection is never discovered, so it would stay selected
        # and unconnectable forever, and block auto-select, which only fills an
        # empty selection. Clear it, persisted param included, so a config saved
        # before the exclusion existed recovers on its own.
        if self.selected_topic != 'None' and self._isExcludedTopic(self.selected_topic):
            self.msg_if.pub_debug("Clearing excluded selected_topic: " + str(self.selected_topic))
            self.set_selected_topic('None')

        selected_topic = copy.deepcopy(self.selected_topic)
        last_available = copy.deepcopy(self.available_topics)

        topics = nepi_sdk.find_topics_by_msg(self.status_msg_type, topics_list = self.active_topics, types_list = self.active_topic_types)
        available_namespaces = []
        available_topics = []
        for topic in topics:
            namespace = str(Path(topic).parent.parent)
            if self.base_namespace not in namespace:
                namespace = self.base_namespace
            if namespace not in available_namespaces:
                available_namespaces.append(namespace)
            valid = True
            if self.connect_namespace in topic and self.selected_topic != topic:
                valid = False
            elif len(self.filter_topics_list) > 0 and self.selected_topic != topic:
                valid = False
                for filter in self.filter_topics_list:
                    if filter in topic:
                        valid = True
                        break
            if valid == True and self._isExcludedTopic(topic):
                valid = False
            if valid == True:
                available_topics.append(topic.replace('/status',''))
        if available_topics != last_available:
            self.available_topics = available_topics
            needs_publish = True

        ####################
        if self.connected_topic is not None:
            if self.connected_topic not in self.available_topics:
                success = self.unsubscribe_topic()
        # Honor auto_select_enabled here too. This path ignored the flag, so a
        # consumer that disabled auto-select (a pan/tilt pair, where auto-selecting
        # the first servo is guaranteed wrong for at least one axis) still got one
        # picked for it one second after startup.
        if selected_topic == 'None' and len(self.available_topics) > 0 and self.auto_select_enabled == True:
            self.selected_topic = self.available_topics[0]
        needs_publish = True

        was_connected = copy.deepcopy(self.connected)
        if self.selected_topic in self.available_topics and self.connected_topic != selected_topic:
            success = self.subscribe_topic(self.selected_topic)
        elif self.connect_if is not None:
            self.connected = self.connect_if.check_connection()
        elif self.selected_topic not in self.available_topics:
            self.connected = False
        # else: already subscribed to the selected topic -- leave self.connected
        # to the status callback (sets True on each msg) and the staleness check
        # below, so it does not get clobbered False every cycle.

        ##################
        cur_time = nepi_utils.get_time()
        last_time = copy.deepcopy(self.last_status_time )
        if self.connected == True:
            if (cur_time - last_time) > self.CONNECTED_TIMEOUT:
                self.connecting = False 
                self.connected = False 
                self.connected_topic = ''
                self.status_msg = None
                self.status_dict = None
                self.data_msg = None
                self.data_dict = None
                self.navpose_msg = None
                self.navpose_dict = None
                self.reset_topics_dict()

        if self.connected == False:
            self.reset_topics_dict()


        #################
        # Get settings from param server
        if needs_publish == True:
          self.publish_status()

        ##################################
        # Update config_dict
        available_topics = copy.deepcopy(self.available_topics)
        selected_topic = copy.deepcopy(self.selected_topic)

        self.config_dict['name'] = self.connect_name
        self.config_dict['id'] = self.connect_id

        self.config_dict['status_msg_type'] = self.connect_status_msg

        self.config_dict['available_topics'] = available_topics
        # Path names, not node names: a selector must distinguish one device's
        # data products from each other. Device connect classes see no change --
        # their topics reduce to the node name either way. See
        # get_available_path_names().
        available_names = self.get_available_path_names(available_topics)
        self.config_dict['available_names'] = available_names

        selected_name = 'None'
        if selected_topic not in available_topics:
            # Auto-select fills an EMPTY selection only. A selection that IS set but
            # not yet discovered must be left alone: available_topics is populated
            # incrementally as sibling device nodes come up, so grabbing
            # available_topics[0] here silently repointed a restored selection at
            # whichever device happened to appear first. On a two-axis node that
            # landed both axes on the same servo -- ptx_servos_node restored pan
            # channel 0 and tilt channel 1 from config, then this method moved tilt
            # to channel 0 before channel 1 was discovered, and the node adopted the
            # wrong topic on its next updateConnectsHandler cycle. Nothing rewrote
            # the param when that happened, so the config file still held the
            # operator's real choice while the live selection had already moved on,
            # which is why the selection looked like it never saved.
            if selected_topic == 'None' and len(available_topics) > 0 and self.auto_select_enabled == True:
                selected_topic = available_topics[0]
                self.selected_topic = selected_topic

        if selected_topic in available_topics:
            selected_ind = available_topics.index(selected_topic)
            selected_name = available_names[selected_ind]
        elif selected_topic != 'None':
            # Selected but not discovered yet. Report the selection with a derived
            # label rather than 'None' so the status does not read as unselected
            # while discovery catches up.
            selected_names = self.get_available_path_names([selected_topic])
            if len(selected_names) > 0:
                selected_name = selected_names[0]

        self.config_dict['selected_topic'] = selected_topic
        self.config_dict['selected_name'] = selected_name

        self.config_dict['connecting'] = self.connecting
        self.config_dict['connected'] = self.connected
        connected_topic = self.connected_topic
        if connected_topic is None:
            connected_topic = 'None'
        self.config_dict['connected_topic'] = connected_topic

        connect_msg = "Not Selected"
        if self.selected_topic != "None":
            connect_msg = "Selected"
            if self.connecting == True:
                connect_msg = "Connecting"
            if self.connected == True:
                connect_msg = "Connected"
        self.config_dict['connect_msg'] = connect_msg

        #######################
        # Update Topics Dict
        # if self.system_topics_lock.locked() == False:
        #     for key_name in self.system_if_dict.keys():
        #         if self.system_if_dict[key_name] is not None:
        #             topic_key = key_name.replace('_if')
        #             status_msg_type = self.system_topics_dict[topic_key].get('status_msg_type', '')
        #             if self.status_msg_type == status_msg_type:
        #                 self.system_topics_dict[topic_key]['topic_locked'] = True
        #                 self.system_topics_dict[topic_key]['topic'] = self.namespace 
        #                 self.system_topics_dict[topic_key]['connecting'] = True
        #                 self.system_topics_dict[topic_key]['connected'] = True
        #             else:
        #                 self.system_topics_dict[topic_key]['topic_locked'] = False
        #                 try:
        #                     self.system_topics_dict[topic_key]['topic'] = self.system_if_dict[key_name].get_namespace() 
        #                     self.system_topics_dict[topic_key]['connecting'] = True
        #                     self.system_topics_dict[topic_key]['connected'] = True
        #                 except:
        #                     self.system_topics_dict[topic_key]['topic'] = ''
        #                     self.system_topics_dict[topic_key]['connecting'] = False
        #                     self.system_topics_dict[topic_key]['connected'] = False

            

        #######################
        # Update Data Products Dict
        for key_name in self.data_products_if_dict.keys():
            if self.data_products_if_dict[key_name] is not None:
                topic_key = key_name.replace('_if')
                topic = self.data_products_topics_dict.get(topic_key,'')
                if topic == '':
                    try:
                        self.data_products_topics_dict[topic_key]['topic'] = self.data_products_if_dict[key_name].get_namespace() 
                        self.data_products_topics_dict[topic_key]['connecting'] = True
                        self.data_products_topics_dict[topic_key]['connected'] = True
                    except:
                        pass
        
        nepi_sdk.start_timer_process(1.0, self._nodeIFUpdater, oneshot = True)



    #     #######################
    #     # Update Topics Dict
    #     for key in self.system_if_dict.keys():
    #         if self.system_if_dict[key] is not None:
    #             topic_key = key.replace('_if')
    #             topic = self.system_topics_dict.get(topic_key,'')
    #             if topic == '':
    #                 try:
                        
    #                     self.system_topics_dict[topic_key]['topic'] = self.system_if_dict[key].get_namespace() 
    #                     self.system_topics_dict[topic_key]['connecting'] = True
    #                     self.system_topics_dict[topic_key]['connected'] = True
    #                 except:
    #                     pass

        





    def subscribe_topic(self, topic):
        self.msg_if.pub_debug("Subscribing from Pubs and Subs for topic: " + str(topic))

        success = False
        success = self.unsubscribe_topic()
        self.reset_topics_dict()
        # Subscribers Config Dict ####################
        self.msg_if.pub_debug("Starting Status Topic Subscription: " + str([self.selected_topic + '/status', self.status_msg_class]))
        self.system_topics['status']['topic'] = self.selected_topic + '/status'
        self.connect_topic_subs_dict = {
            self.node_if_prefix + 'status_sub': {
                'namespace': self.selected_topic,
                'topic': 'status',
                'msg': self.status_msg_class,
                'qsize': 10,
                'callback': self._statusCb
            }
        }



        if self.connect_data == True:
             self.msg_if.pub_debug("Starting Data Topic Subscription: " + str([self.selected_topic, self.connect_data_msg_class]))
             self.system_topics['data']['topic'] = self.selected_topic
             self.connect_topic_subs_dict[self.node_if_prefix + 'data_sub'] = {
                'namespace': self.selected_topic,
                'topic': '',
                'msg': self.connect_data_msg_class,
                'qsize': 1,
                'callback': self._baseDataCb
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
        # next subscribe -- node_if._updaterCb calls subscribe_topic on every
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
            self.status_dict = None
            self.data_dict = None
            self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
            


        return success




    def unsubscribe_topic(self):
        success = False
        if self.connecting == True or self.connected == True:
            self.reset_topics_dict()
            self.msg_if.pub_debug("Unsubscribing from Pubs and Subs")

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
            self.status_dict = None
            self.data_msg = None
            self.data_dict = None

            self.reset_topics_dict()
            self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

            if self.save_data_connect_if is not None:
                self.msg_if.pub_debug("Disconnecting Save Data IF")
                self.save_data_connect_if.unregister()
                self.save_data_connect_if = None
                nepi_sdk.sleep(1)
                self.system_topics['save_data']['connected'] = False
                self.system_topics['save_data']['connecting'] = False

            success = True
        return success

    ###############################
    # Class Private Methods
    ###############################



    def _statusCb(self,status_msg):
       # Update Connection Status
        self.system_topics['status']['connected'] = True
        self.system_topics['status']['connecting'] = False

        self.last_status_time = nepi_utils.get_time()
        if self.connected == False:
            self._announceConnected(self.connect_id)
            self.connecting = False
            self.connected_topic = self.selected_topic
        self.connected = True
        self.status_msg = status_msg
        
        # Process NavPose Status Dict
        status_navpose_dict = None
        try:
            navpose_msg = status_msg.navpose_msg
            status_navpose_dict = nepi_nav.convert_navpose_msg2dict(navpose_msg)
        except:
            pass
        if status_navpose_dict is None:
            status_navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

        status_dict = nepi_sdk.convert_msg2dict(status_msg)
        try:
            del status_dict['navpose_msg']
        except:
            pass


        # Call Callback
        for key in self.callback_dict['status'].keys():
            callback = self.callback_dict['status'][key]
            if callback is not None:
                try:
                    callback(status_dict)
                except:
                    pass


        # Update Dict
        self.status_dict = status_dict
        self.status_navpose_dict = status_navpose_dict


        # Update Connection Topics from Status
        if status_dict is not None:
            try:
                for key in self.system_topics.keys():
                    if key != 'status' and key != 'data':
                        name_key = key + '_topic'
                        self.system_topics[key]['topic'] = status_dict.get(name_key,'')
            except:
                pass

        # Setup NavPose Data Connection
        navpose_topic = status_dict.get('navpose_topic','')
        self.system_topics['navpose']['topic'] = navpose_topic
        topic_dict = self.system_topics['navpose']
        if navpose_topic != '' and self.connect_navpose == True: 
            if topic_dict['connecting'] == False and topic_dict['connected'] == False:
                self.msg_if.pub_debug("Starting Navpose Topic Subscription: " + str([navpose_topic, NavPose]))

                topic_dict['connecting'] = True

                sub_name = self.node_if_prefix + 'navpose_sub'
                sub_dict = {
                    'namespace': navpose_topic,
                    'topic': 'navpose',
                    'msg': NavPose,
                    'qsize': 1,
                    'callback': self._navposeCb
                }    
                self.connect_topic_subs_dict[sub_name] = sub_dict
                self.node_if.register_subs(sub_name,sub_dict)



        # Setup Save Data IF Connection
        save_data_topic = status_dict.get('save_data_topic','')
        self.system_topics['save_data']['topic'] = save_data_topic
        topic_dict = self.system_topics['save_data']
        if save_data_topic != '' and self.save_data_connect_if is None: 
             if topic_dict['connecting'] == False and topic_dict['connected'] == False:
                 self.msg_if.pub_debug("Connecting to Save Data IF with topic: " + str(save_data_topic))
                 self.system_topics['save_data']['connecting'] = True
                 self.save_data_connect_if = ConnectSaveDataIF(namespace = save_data_topic,
                                                                msg_if = self.msg_if,
                                                                node_if = self.node_if
                 )
                 nepi_sdk.sleep(1)

        connected = False
        if self.save_data_connect_if is not None:
            connected = self.save_data_connect_if.check_connection()
        if connected == True:
            self.system_topics['save_data']['connected'] = True
            self.system_topics['save_data']['connecting'] = False


    def _dataCb(data_msg):
        data_dict = None
        try:
            data_dict = nepi_sdk.convert_msg2dict(data_msg)
        except:
            pass
        return data_dict

    def _baseDataCb(self,data_msg):
        self.data_msg = data_msg
        # Update Connection Status
        self.system_topics['data']['connected'] = True
        self.system_topics['data']['connecting'] = False

        # Create Data Dict
        data_dict = None
        try:
            data_dict = self._dataCb(data_msg)
        except Exception as e:
            self.msg_if.pub_debug("_dataCb failed to return data dict:  " + str(e), trottle_s = 5)    

        # Call Callback
        if data_dict is not None:
            for key in self.callback_dict['data'].keys():
                callback = self.callback_dict['data'][key]
                if callback is not None:
                    try:
                        callback(data_dict)
                    except:
                        pass

        # Update Dict
        if data_dict is not None:
            self.data_lock.acquire()
            self.data_dict = data_dict
            self.data_lock.release()




    def _navposeCb(self,navpose_msg):  
        # Update Connection Status
        self.system_topics['navpose']['connected'] = True
        self.system_topics['navpose']['connecting'] = False

        # Create Navpose Dict
        navpose_dict = nepi_nav.convert_navpose_msg2dict(navpose_msg)

        for key in self.callback_dict['navpose'].keys():
            callback = self.callback_dict['navpose'][key]
            if callback is not None:
                try:
                    callback(self.navpose_dict)
                except:
                    pass

        self.navpose_dict_lock.acquire()
        self.navpose_msg = navpose_msg
        self.navpose_dict = navpose_dict
        self.navpose_dict_lock.release()






