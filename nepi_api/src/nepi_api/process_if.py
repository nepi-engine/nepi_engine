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
import copy
import time 
import copy
import numpy as np
import math
import threading
import cv2

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from nepi_interfaces.msg import StringArray

from nepi_interfaces.msg import ProcessStatus, MgrSystemStatus


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_ais
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeParamsIF, NodeClassIF
from nepi_api.system_if import SaveDataIF




#########################################
# Process IF Class
#########################################


CONNECTED_TIMEOUT = 2

class ProcessIF:
    
    msg_if = None
    node_if = None
    node_if_shared = False
    save_data_if = None

    process_name = None
    process_namespace = ''
    process_data_products = []
    process_status_msg = ProcessStatus
    process_node_pubs_dict = None
    process_node_subs_dict = None
    max_proc_rate_hz = 10
    process_ready = False

    active_nodes = []
    active_topics = []
    active_topic_types =  []
    active_services =  []  

    source_status_msg_type = None  

    available_sources = []
    available_names = []

    auto_select_enabled = True
    multi_source_enabled = True
    exclude_source_filters = []


    selected_sources_param = []
    selected_sources = []
    sources_connecting = []
    sources_connected = []
    sources_connected_topics = []
    sources_status_sub_dict = dict()
    sources_status_dict = dict()
    sources_data_sub_dict = dict()
    sources_data_dict = dict()
    sources_pubs_dict = dict()
    sources_stats_dict = dict()

    source_selected = False
    source_connected = False

    show_selector = True
    show_controls = True
    show_data = True


    has_imaging = False
    max_image_pub_rate_hz = 10
    imaging_enabled = True
    use_last_image = False
    imaging_if_api = None
    imaging_if_class = None
    imaging_source_topics = []
    imaging_pub_topics = []

    status_has_published = False

    #######################
    ### IF Initialization
    def __init__(self, 
                process_name = None,
                process_group = 'PROCESS',
                process_description = 'Process',
                process_status_msg = ProcessStatus,
                process_data_products = [],
                max_process_rate_hz = 10,
                source_status_msg_type = None,
                source_data_msg_type = None,       
                source_callback_dict = None,       
                auto_select_enabled = True,
                muti_source_enabled = True,
                exclude_source_filters = [],
                selected_sources = [],
                has_imaging = False,
                max_image_pub_rate_hz = 10,
                imaging_if_api = None,
                imaging_if_class = None,
                show_selector = True,
                show_controls = True,
                show_data = True,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None,
                save_data_if = None
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
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            log_name = nepi_utils.get_clean_name(log_name)
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        # Create Namespace
        self.process_name = nepi_utils.get_clean_name(process_name)
        if self.process_name is None or self.process_name == '':
            self.msg_if.pub_warn("Process Name Not Valid: " + str(process_name)) 
            return
        self.msg_if.pub_info("Using Process Name: " + self.process_name)
        self.process_namespace = nepi_sdk.create_namespace(self.node_name,self.process_name)




        ##############################    
        # Initialize Class Variables

        self.process_group = str(process_group)
        self.process_description = str(process_description)
        # Check Process Status Msg Type
        if process_status_msg is not None:
            self.process_status_msg = process_status_msg

        self.max_process_rate_hz = max_process_rate_hz

        # Check Status Msg Type
        if source_status_msg_type is None:
            self.msg_if.pub_warn("Source Status Msg Not Provided") 
            return
        self.source_status_msg_type = source_status_msg_type

        # Check Status Msg Type
        if source_data_msg_type is None:
            self.msg_if.pub_warn("Source Data Msg Not Provided") 
            return
        self.source_data_msg_type = source_data_msg_type

        if source_callback_dict is not None:
            for key in source_callback_dict.keys():
                self.source_callback_dict[key] = source_callback_dict[key]
      
        self.auto_select_enabled = auto_select_enabled
        self.muti_source_enabled = muti_source_enabled
        self.exclude_source_filters = exclude_source_filters

        clean_sources = []
        for source_topic in selected_sources:
            clean_sources = nepi_sdk.get_full_namespace(source_topic)
        self.selected_sources = clean_sources

        if has_imaging == True and imaging_if_api is not None and imaging_if_class is not None:
            self.has_imaging = has_imaging,
            self.max_image_pub_rate_hz = max_image_pub_rate_hz,
            self.imaging_if_api = imaging_if_api,
            self.imaging_if_class = imaging_if_class,

        
        self.show_selector = show_selector,
        self.show_controls = show_controls,
        self.show_data = show_data,

                   
        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        CFGS_DICT = {
                'namespace': self.process_namespace
        }

        # Params Config Dict ####################
        # Persist the selected topic under the connect namespace so the
        # selection survives node restarts (via the config manager). Passing a
        # params_dict is what enables config management on NodeClassIF.
        PARAMS_DICT = {
            'selected_sources': {
                'namespace': self.process_namespace,
                'factory_val': self.selected_sources
            }
        }


        # Publishers Config Dict ####################
        self.process_node_pubs_dict = {
            'status_pub': {
                'namespace': self.process_namespace,
                'topic': 'status',
                'msg': self.process_status_msg,
                'qsize': 1,
                'latch': True
            }
        }



        # Subscribers Config Dict ####################
        self.process_node_subs_dict = {
            'set_source': {
                'namespace': self.process_namespace,
                'topic': 'set_source',
                'msg': String,
                'qsize': None,
                'callback': self._setSourceCb, 
                'callback_args': ()
            },
            'remove_source': {
                'namespace': self.process_namespace,
                'topic': 'clear_source',
                'msg': String,
                'qsize': None,
                'callback': self._clearSourceCb, 
                'callback_args': ()
            },
            'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self._systemStatusCb
            },
        }

        if self.multi_source_enabled == True:
            self.process_node_subs_dict['set_sources'] = {
                'namespace': self.process_namespace,
                'topic': 'set_sources',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.setSourcesCb, 
                'callback_args': ()
            },
            self.process_node_subs_dict['add_sources'] = {
                'namespace': self.process_namespace,
                'topic': 'add_sources',
                'msg': String,
                'qsize': 10,
                'callback': self.addSourcesCb, 
                'callback_args': ()
            },
            self.process_node_subs_dict['remove_sources'] = {
                'namespace': self.process_namespace,
                'topic': 'add_sources',
                'msg': Empty,
                'qsize': 10,
                'callback': self.removeSourcesCb, 
                'callback_args': ()
            },
            self.process_node_subs_dict['clear_sources'] = {
                'namespace': self.process_namespace,
                'topic': 'clear_sources',
                'msg': Empty,
                'qsize': 10,
                'callback': self.clearSourcesCb, 
                'callback_args': ()
            },
        
        if node_if is None:
            self.node_if = NodeClassIF(
                            configs_dict = CFGS_DICT,
                            params_dict = PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.process_node_pubs_dict,
                            subs_dict = self.process_node_subs_dict,
                            log_name_list = [],
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()
        else:
            self.node_if_shared = True
            try:
                self.node_if = node_if
                self.node_if.register_pubs(self.process_node_pubs_dict)
                self.node_if.register_subs(self.process_node_subs_dict)
                # Register the persisted selection param on the shared node_if too.
                self.node_if.add_param('selected_sources', self.process_namespace, self.selected_sources)
                nepi_sdk.sleep(1)
            except Exception as e:
                self.msg_if.pub_info("Failed to register pubs and subs: " + str(e))
                return


        # Restore any persisted selection. When no explicit topic was requested
        # (selected_sources == "None"), use the value the config manager restored
        # for this connect namespace. Otherwise honor the explicit request.
        self.selected_sources_param = 'selected_sources'
        if selected_sources == "None":
            persisted = self.node_if.get_param(self.selected_sources_param)
            if persisted is not None and persisted != '' and persisted != "None":
                selected_sources = persisted
        self.selected_sources = selected_sources
        self.msg_if.pub_info("Init Selected Topic: " + str(self.selected_sources))

        ###############################
        self.process_data_products = process_data_products
        self.msg_if.pub_info("####################", log_name_list = self.log_name_list)
        self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
        if save_data_if is not None and save_data_if != 'None':
            self.save_data_if = save_data_if
            data_products = self.save_data_if.get_data_products()
            for data_product in self.process_data_products:
                if data_product not in data_products:
                    self.save_data_if.register_data_product(data_product)
        elif save_data_if != 'None' and len(self.process_data_products) > 0:
            # Setup Save Data IF Class 
            self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
            factory_data_rates= dict()
            for data_product in self.process_data_products:
                factory_data_rates[data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

            factory_filename_dict = {
                'prefix': "", 
                'add_timestamp': True, 
                'add_ms': True,
                'add_us': False,
                'suffix': "",
                'add_node_name': True
                }

            sd_namespace = self.namespace
            self.save_data_if = SaveDataIF(namespace = sd_namespace,
                                    data_products = [self.data_product],
                                    factory_rate_dict = factory_data_rates,
                                    factory_filename_dict = factory_filename_dict,
                                    log_name_list = self.log_name_list,
                                    msg_if = self.msg_if,
                                    node_if = self.node_if)
            nepi_sdk.sleep(1)

        if self.save_data_if is not None:
            self.status_msg.save_data_topic = self.save_data_if.get_namespace()
            self.msg_if.pub_warn("Using save_data namespace: " + str(self.status_msg.save_data_topic), log_name_list = self.log_name_list)




        ##############################
        # Start updater process
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

        ##############################
        # Complete Initialization
        self.process_ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete")
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_process_ready_state(self):
        """Return the ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.process_ready

    def wait_for_process_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.process_ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.process_ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.process_ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.process_ready  

    def get_namespace(self):
        """Return the fully-resolved ROS namespace for the sources_connected PTX device.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.process_namespace
    

    def get_available_sources(self):
        return self.available_sources
    

    def get_available_names(self, available_sources = []):
        available_names = []
        for topic in available_sources:
            name = topic
            topic = topic[1:]
            topic_split = topic.split('/')
            if len(topic_split) > 2:
                name = topic_split[2]
            available_names.append(name)
        return available_names

    
    def get_selected_sources(self):
        return self.selected_sources
    
    def set_selected_sources(self, selected_sources):
        if selected_sources in self.available_sources or selected_sources == "None":
            self.selected_sources = selected_sources
        self.publish_status()
        # Persist the selection so it survives a node restart. set_param writes
        # the ROS param; save_config asks the config manager to save it to file.
        if self.node_if is not None:
            self.msg_if.pub_warn("selected_sources: " + str(selected_sources))
            self.node_if.set_param('selected_sources', self.selected_sources)
            self.node_if.save_config()
    

    def check_connection(self):
        """Check whether the device is currently sources_connected.

        Returns:
            bool: True if a status message has been received within the connection timeout window,
                False otherwise.
        """
        return self.sources_connected
    
    def check_connections(self, source_topic):
        """Check whether the device is currently sources_connected.

        Returns:
            bool: True if a status message has been received within the connection timeout window,
                False otherwise.
        """
        connected = False
        if source_topic in self.sources_connected_topics
        return self.sources_connected

    def wait_for_connection(self, timeout = float('inf') ):
        """Block until the device is sources_connected or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if connection was established, False if the timeout was reached.
        """
        if self.node_if is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.sources_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.sources_connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.sources_connected


    def unregister(self):
        success = False
        self.unsubscribe_topic()
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.sleep(1)
            else:
                self.unsubscribe_topic()

                if self.node_if is not None:
                    if self.process_node_subs_dict is not None:
                        for sub_name in self.process_node_subs_dict.keys():
                            self.node_if.unregister_sub(sub_name)
                self.process_node_subs_dict = None

                if self.node_if is not None:
                    if self.process_node_pubs_dict is not None:
                        for pub_name in self.process_node_pubs_dict.keys():
                            self.node_if.unregister_pub(pub_name)
                self.process_node_pubs_dict = None
                
        time.sleep(1)
        try:
            self.node_if = None
            self.selected_sources = 'None'
            self.connecting = False 
            self.sources_connected = False 
            self.sources_connected_topics = 'None'
            success = True
        except Exception as e:
            self.msg_if.pub_warn("Failed to unregister:  " + str(e))
        return success



    def get_process_status_msg(self):

        available_sources = copy.deepcopy(self.available_sources)
        selected_sources = copy.deepcopy(self.selected_sources)
        status_msg = ProcessStatus()

        status_msg.name = self.process_name
        status_msg.id = self.process_id

        status_msg.status_msg_type = self.process_status_msg

        status_msg.available_sources = available_sources
        available_names = self.get_available_names(available_sources)
        status_msg.available_names = available_names

        selected_name = 'None'
        if selected_sources not in available_sources:
            if len(available_sources) > 0 and self.auto_select_enabled == True:
                selected_sources = available_sources[0]
                self.selected_sources = selected_sources
            else:
                selected_sources = 'None' 

        if selected_sources in available_sources:
            selected_ind = available_sources.index(selected_sources)
            selected_name = available_names[selected_ind]

        status_msg.selected_sources = selected_sources
        status_msg.selected_name = selected_name

        status_msg.connecting = self.connecting
        status_msg.sources_connected = self.sources_connected
        sources_connected_topics = self.sources_connected_topics
        if sources_connected_topics is None:
            sources_connected_topics = 'None'
        status_msg.sources_connected_topics = sources_connected_topics

        connect_msg = "Not Selected"
        if self.selected_sources != "None":
            connect_msg = "Selected"
            if self.connecting == True:
                connect_msg = "Connecting"
            if self.sources_connected == True:
                connect_msg = "Connected"
        status_msg.connect_msg = connect_msg


        status_msg.show_selector = self.show_selector
        status_msg.show_controls = self.show_controls
        status_msg.show_data = self.show_data


        # ###########
        # if self.node_if is not None:
        #     if self.status_has_published == False:
        #         self.msg_if.pub_warn("Publishing Status: " + str(status_msg))
        #         self.status_has_published = True
        #     self.node_if.publish_pub('status_pub', status_msg) 
        #     #self.node_if.save_config()
        return status_msg

    def publish_status(self, status_msg):
        ###########
        if self.node_if is not None:
            if self.status_has_published == False:
                self.msg_if.pub_warn("Publishing Status: " + str(status_msg))
                self.status_has_published = True
            self.node_if.publish_pub('status_pub', status_msg) 
        return status_msg


    #######################
    # Class Private Methods
    #######################

    # ROS callback for the system status msg. Populates the active topic/type
    # lists that discovery searches. NOTE: this MUST NOT share a name with the
    # discovery timer below -- a duplicate name silently shadows this method, so
    # active_topics never gets populated and discovery finds nothing.
    def _systemStatusCb(self,msg):
            self.active_nodes = msg.active_nodes
            self.active_topics = msg.active_topics
            self.active_topic_types = msg.active_topic_types
            self.active_services = msg.active_services


    # Discovery/connection timer. Finds available topics of the connect status
    # msg type among the active topics, auto-selects, and subscribes.
    def _updaterCb(self,timer):
        needs_publish = False
        ##############

        selected_sources = copy.deepcopy(self.selected_sources)
        last_available = copy.deepcopy(self.available_sources)

        topics = nepi_sdk.find_topics_by_msg(self.connect_status_msg, topics_list = self.active_topics, types_list = self.active_topic_types)
        available_sources = []
        for topic in topics:
            valid = True
            for filter in self.exclude_source_filters:
                if filter in topic:
                    valid = False
            if valid == True:
                available_sources.append(topic.replace('/status',''))
        if available_sources != last_available:
            self.available_sources = available_sources
            needs_publish = True

        ####################
        if self.sources_connected_topics is not None:
            if self.sources_connected_topics not in self.available_sources:
                success = self.unsubscribe_topic()
        if selected_sources == 'None' and len(self.available_sources) > 0:
            self.selected_sources = self.available_sources[0]
        needs_publish = True

        was_sources_connected = copy.deepcopy(self.sources_connected)
        if self.selected_sources in self.available_sources and self.sources_connected_topics != selected_sources:
            success = self.subscribe_source(self.selected_sources)
        elif self.selected_sources not in self.available_sources:
            self.sources_connected = False
        # else: already subscribed to the selected topic -- leave self.sources_connected
        # to the status callback (sets True on each msg) and the staleness check
        # below, so it does not get clobbered False every cycle.

        ##################
        cur_time = nepi_utils.get_time()
        last_time = copy.deepcopy(self.last_status_time )
        for i, source_topic in enumerate(self.sources_connected_topics):
            connected = self.sources_connected[i]
            if connected == True:
                if (cur_time - last_time) > CONNECTED_TIMEOUT:
                    self.sources_connecting[i] = False 
                    self.sources_connected[i] = False 
                    self.sources_status_dict[i] = None
                    self.sources_status_msg[i] = None



        ##################
        # Get settings from param server
        # if needs_publish == True:
        #   self.publish_status()
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)


