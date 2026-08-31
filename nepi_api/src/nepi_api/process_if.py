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


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image


from nepi_interfaces.msg import UpdateOrder, UpdateRangeWindow, UpdateFloat, UpdateFloats, UpdateInt, UpdateBool, UpdateString, UpdateStringArray, UpdateTrigger


from nepi_interfaces.msg import ProcessStatus, MgrSystemStatus
from nepi_interfaces.msg import Control, ControlsStatus, UpdateControl
from nepi_interfaces.msg import Datum, DataStatus
from nepi_interfaces.msg import Result, ResultsStatus
from nepi_interfaces.msg import ImageStatus
from nepi_interfaces.msg import Detections, DetectorStatus
from nepi_interfaces.msg import Targets, TargetingStatus





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

    process_status_dict = dict()
    has_process_data = False
    process_data_msg = DataStatus()
    process_data_dict = dict()

    has_process_controls = False
    process_controls_msg = ControlsStatus()
    process_controls_dict = dict()

    has_process_results = False
    process_results_namespace = ''

    
    process_node_pubs_dict = None
    process_node_subs_dict = None

    max_process_rate_hz = 10
    process_ready = False

    # Resolved process namespace. Every pub, sub and param this IF registers
    # hangs off it, and it is what ProcessStatus.namespace reports -- the RUI's
    # Nepi_IF_ConnectProcess matches incoming status on this field, so it has to
    # be the same string the RUI subscribed with.
    namespace = ''

    # Run state. enabled is the operator's request, running is what the owning
    # node reports back after acting on it. They are deliberately separate: an
    # enabled process whose sources drop out is enabled and not running.
    has_enable = False
    enabled = False
    running = False
    state = False
    msg_str = ''

    enable_callback = None
    controls_updated_callback = None
    data_updated_callback = None

    # Source management is not implemented on this IF yet. The fields are
    # reported as empty rather than left undefined so publish_status() and the
    # check_connection() helpers cannot raise.
    manages_sources = False
    available_sources = []
    selected_sources = []
    sources_connected = False
    sources_connected_topics = []

    controls_hidden = False
    data_hidden = False

    status_pub_rate_hz = 1.0
    last_pub_time = None
    time_list = [1.0] * 10

    active_nodes = []
    active_topics = []
    active_topic_types =  []
    active_services =  []  


    show_selector = True
    show_controls = True
    show_data = True
    show_results = True



    has_image_pub = False
    image_pub_name = 'image'
    max_image_pub_rate_hz = 10
    image_pub_enabled = True
    use_last_image = False
    imaging_source_topics = []
    imaging_pub_topics = []

    status_has_published = False

    #######################
    ### IF Initialization
    def __init__(self, 
                process_name = None,
                process_group = 'PROCESS',
                process_description = 'Process',
                process_data_dict = None,
                process_controls_dict = None,
                results_msg = None,
                results_name = 'results',
                show_data = True,
                show_controls = True,
                show_results = True,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None,
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

        # Create Process Name
        self.process_name = nepi_utils.get_clean_name(process_name)
        if self.process_name is None or self.process_name == '':
            self.msg_if.pub_warn("Process Name Not Valid: " + str(process_name)) 
            return
        self.msg_if.pub_info("Using Process Name: " + self.process_name)
        self.process_namespace = nepi_sdk.create_namespace(self.node_namespace,self.process_name)
        # namespace is the name the rest of this class registers and reports
        # against; process_namespace is kept as the historical accessor.
        self.namespace = self.process_namespace
        # Registry keys on a shared node_if must be domain-unique, so every key
        # this IF adds carries the process name. Param wire names ARE
        # namespace + key, so the prefix is part of the external param surface.
        self.node_if_prefix = self.process_name + '_'

       
        ##############################    
        # Initialize Class Variables

        self.process_group = str(process_group)
        self.process_description = str(process_description)
        # Check Process Status Msg Type

        # The caller passes an init dict (the nepi_controls / nepi_data authoring
        # form). Every accessor below and update_status_msg() operate on the
        # created form, so the conversion happens once, here.
        if process_controls_dict is None:
            self.process_controls_dict = dict()
            self.has_process_controls = False
            show_controls = False
        else:
            self.process_controls_dict = nepi_controls.create_controls_dict(
                                            copy.deepcopy(process_controls_dict))
            self.has_process_controls = len(self.process_controls_dict.keys()) > 0
        self.show_controls = show_controls
        self.process_controls_msg = nepi_controls.create_status_msg(
                                        name = self.process_name,
                                        description = self.process_description,
                                        show_controls = self.show_controls)

        if process_data_dict is None:
            self.process_data_dict = dict()
            self.has_process_data = False
            show_data = False
        else:
            self.process_data_dict = nepi_data.create_data_dict(
                                            copy.deepcopy(process_data_dict))
            self.has_process_data = len(self.process_data_dict.keys()) > 0
        self.show_data = show_data
        self.process_data_msg = nepi_data.create_status_msg(
                                        name = self.process_name,
                                        description = self.process_description,
                                        show_data = self.show_data)

                   
        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        # Configs Config Dict ####################
        CFGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.process_namespace
        }

        # Params Config Dict ####################
        # Persist the selected topic under the connect namespace so the
        # selection survives node restarts (via the config manager). Passing a
        # params_dict is what enables config management on NodeClassIF.
        self.controls_param_name = self.node_if_prefix + 'process_controls_dict'
        # set_selected_sources wrote a bare 'selected_sources', which was never
        # registered here, so set_param resolved to no namespace and the write was
        # dropped silently. Prefixed like controls_param_name so it cannot collide
        # with the ControlsIF or detector params of the same name on a shared node_if.
        self.sources_param_name = self.node_if_prefix + 'selected_sources'
        PARAMS_DICT = {
            self.controls_param_name: {
                'namespace': self.process_namespace,
                'factory_val': self.process_controls_dict
            },
            self.sources_param_name: {
                'namespace': self.process_namespace,
                'factory_val': self.selected_sources
            }
        }


        # Publishers Config Dict ####################
        self.process_node_pubs_dict = dict()


        # The status publisher is unconditional. Nepi_IF_ConnectProcess renders
        # nothing at all until a ProcessStatus arrives, so a process with no
        # custom status message still has to publish the generic one.

        self.process_node_pubs_dict[self.node_if_prefix + 'status_pub'] = {
            'namespace': self.process_namespace,
            'topic': 'status',
            'msg': ProcessStatus,
            'qsize': 1,
            'latch': True
        }

        
        if results_msg is not None and results_name is not None:
            results_name = nepi_utils.get_clean_name(results_name)
            if results_name != '':
                self.process_node_pubs_dict[self.node_if_prefix + 'results_pub'] = {
                    'namespace': self.process_namespace,
                    'topic': results_name,
                    'msg': results_msg,
                    'qsize': 1,
                    'latch': True
                }
                self.process_results_namespace = self.process_namespace + '/results_name'
                self.has_process_results = True
                self.show_results = show_results

        # Subscribers Config Dict ####################
        # The full command surface registers here, once, regardless of
        # has_enable or of whether the process has controls. set_enable is
        # guarded inside its callback instead, so which topics exist never
        # depends on run state.
        self.process_node_subs_dict = {
             self.node_if_prefix + 'set_menu_control_value': {
                'msg': UpdateInt,
                'namespace': self.namespace,
                'topic': 'set_menu_control_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_selection_control_value': {
                'msg': UpdateString,
                'namespace': self.process_namespace,
                'topic': 'set_selection_control_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_selections_control_value': {
                'msg': UpdateStringArray,
                'namespace': self.process_namespace,
                'topic': 'set_selections_control_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_int_control_value': {
                'msg': UpdateInt,
                'namespace': self.process_namespace,
                'topic': 'set_int_control_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_float_control_value': {
                'msg': UpdateFloat,
                'namespace': self.process_namespace,
                'topic': 'set_float_control_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_floatslider_control_value': {
                'msg': UpdateFloat,
                'namespace': self.process_namespace,
                'topic': 'set_floatslider_control_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_floatsliders_control_value': {
                'msg': UpdateRangeWindow,
                'namespace': self.process_namespace,
                'topic': 'set_floatsliders_control_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_trigger_control_value': {
                'msg': UpdateTrigger,
                'namespace': self.process_namespace,
                'topic': 'set_trigger_control_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_bool_control_value': {
                'msg': UpdateBool,
                'namespace': self.process_namespace,
                'topic': 'set_bool_control_value',
                'qsize': 5,
                'callback': self._setValueCb
            },
             self.node_if_prefix + 'set_string_control_value': {
                'msg': UpdateString,
                'namespace': self.process_namespace,
                'topic': 'set_string_control_value',
                'qsize': 5,
                'callback': self._setValueCb
            }
        }




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
                # Register this IF's params on the shared node_if too, or
                # get_param/set_param below resolve to no namespace and the
                # controls dict and enable state never persist.
                self.node_if.add_params(PARAMS_DICT)
                nepi_sdk.sleep(1)
            except Exception as e:
                self.msg_if.pub_info("Failed to register pubs and subs: " + str(e))
                return


        ##############################
        # Complete Initialization
        self.process_ready = True
        # Without this the status topic is advertised and never written, and the
        # RUI process panel stays blank forever.
        nepi_sdk.start_timer_process(float(1) / self.status_pub_rate_hz, self._publishStatusCb)
        self.publish_status()
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
            self.node_if.set_param(self.sources_param_name, self.selected_sources)
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
        if source_topic in self.sources_connected_topics:
            connected = True
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

    ##################
    # Controls Functions

    def get_controls_dict(self):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        return controls_dict

    def get_control_value(self, control_name):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        value = nepi_controls.get_control_value(controls_dict, control_name)
        return value

    def set_control_value(self, control_name, update_value):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        controls_dict = nepi_controls.set_control_value(controls_dict, control_name, update_value)
        self.process_controls_dict = controls_dict
        self.publish_status
        if self.controls_updated_callback is not None:
            self.controls_updated_callback(control_name)
        if self.node_if is not None:
            self.node_if.set_param(self.controls_param_name, self.process_controls_dict)

    def get_control_default_value(self, control_name):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        value = nepi_controls.get_control_default_value(controls_dict, control_name)
        return value

    def set_control_default_value(self, control_name, update_value):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        controls_dict = nepi_controls.set_control_default_value(controls_dict, control_name, update_value)
        self.process_controls_dict = controls_dict

    def get_control_factory_value(self, control_name):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        value = nepi_controls.get_control_factory_value(controls_dict, control_name)
        return value

    def set_control_factory_value(self, control_name, update_value):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        controls_dict = nepi_controls.set_control_factory_value(controls_dict, control_name, update_value)
        self.process_controls_dict = controls_dict

    def reset_control_value(self, control_name):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        controls_dict = nepi_controls.reset_control_value(controls_dict, control_name)
        self.process_controls_dict = controls_dict

    def reset_control_values(self):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        controls_dict = nepi_controls.reset_control_values(controls_dict)
        self.process_controls_dict = controls_dict

    def factory_reset_control_value(self, control_name):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        controls_dict = nepi_controls.factory_reset_control_value(controls_dict, control_name)
        self.process_controls_dict = controls_dict

    def factory_reset_control_values(self):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        controls_dict = nepi_controls.factory_reset_control_values(controls_dict)
        self.process_controls_dict = controls_dict

    def get_control_options(self, control_name):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        options = nepi_controls.get_control_options(controls_dict, control_name)
        return options

    def set_control_options(self, control_name, options):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        controls_dict = nepi_controls.set_control_options(controls_dict, control_name, options)
        self.process_controls_dict = controls_dict
        self.publish_status
        if self.node_if is not None:
            self.node_if.set_param(self.controls_param_name, self.process_controls_dict)

    def get_control_bounds(self, control_name):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        bounds = nepi_controls.get_control_bounds(controls_dict, control_name)
        return bounds

    def set_control_bounds(self, control_name, bounds = []):
        controls_dict = copy.deepcopy(self.process_controls_dict)
        controls_dict = nepi_controls.set_control_options(controls_dict, control_name, bounds)
        self.process_controls_dict = controls_dict
        self.publish_status
        if self.node_if is not None:
            self.node_if.set_param(self.controls_param_name, self.process_controls_dict)


    ##################
    # Data Functions

    def get_data_dict(self):
        """Return a copy of the full data dict, keyed by datum name.

        Returns:
            dict: A deep copy of the data dict.
        """
        process_data_dict = copy.deepcopy(self.process_data_dict)
        return process_data_dict

    def get_datum_value(self, datum_name):
        """Return the current value of one datum, read from its type-correct field.

        Args:
            datum_name (str): The datum key name.

        Returns:
            The datum value, or None if the datum is not registered.
        """
        process_data_dict = copy.deepcopy(self.process_data_dict)
        value = nepi_data.get_datum_value(process_data_dict, datum_name)
        return value

    def set_datum_value(self, datum_name, update_value, timestamp = None):
        """Write one datum value, stamp its timestamp, and publish status.

        The node that owns this interface is the only writer of record; the RUI
        has no publish path to this method.

        Args:
            datum_name (str): The datum key name.
            update_value: The new value. Coerced to the datum's declared type.
            timestamp (float, optional): Write time. Defaults to now.
        """
        process_data_dict = copy.deepcopy(self.process_data_dict)
        process_data_dict = nepi_data.set_datum_value(process_data_dict, datum_name, update_value, timestamp = timestamp)
        self.process_data_dict = process_data_dict
        self.publish_status()
        if self.data_updated_callback is not None:
            self.data_updated_callback(datum_name)

    def get_datum_timestamp(self, datum_name):
        """Return the time one datum's value was last written.

        Args:
            datum_name (str): The datum key name.

        Returns:
            float: The datum timestamp in seconds, or 0.0 if not registered.
        """
        process_data_dict = copy.deepcopy(self.process_data_dict)
        timestamp = nepi_data.get_datum_timestamp(process_data_dict, datum_name)
        return timestamp

    ##################
    # Misc Functions

    def publish_status(self):


        status_msg = ProcessStatus()

        status_msg.name = self.process_name
        status_msg.group = self.process_group
        status_msg.description = self.process_description

        status_msg.node_name = self.node_name
        status_msg.namespace = self.namespace
        status_msg.has_config = False

        # Run state. enabled is what the operator asked for and running is what
        # the owning node reports back; the RUI shows both so an enable that
        # could not take effect is visible rather than silently cosmetic.
        status_msg.enabled = self.enabled
        status_msg.running = self.running
        status_msg.state = self.state
        status_msg.msg_str = self.msg_str
        status_msg.process_ready = self.process_ready

        status_msg.manages_sources = self.manages_sources
        status_msg.available_source_topics = self.available_sources
        status_msg.selected_sources = self.selected_sources
        status_msg.source_connected = self.sources_connected

        # has_process_data / has_process_controls gate whether the RUI renders
        # those blocks at all, so an empty dict has to report False -- reporting
        # True for an empty dict draws an empty panel with a toggle that does
        # nothing.
        status_msg.has_process_data = self.has_process_data
        if self.has_process_data == True:
            process_data_dict = copy.deepcopy(self.process_data_dict)
            self.process_data_msg = nepi_data.update_status_msg(self.process_data_msg, process_data_dict)
            status_msg.process_data = self.process_data_msg
        status_msg.show_data = self.show_data

        status_msg.has_process_controls = self.has_process_controls
        if self.has_process_controls == True:
            process_controls_dict = copy.deepcopy(self.process_controls_dict)
            self.process_controls_msg = nepi_controls.update_status_msg(self.process_controls_msg, process_controls_dict)
            status_msg.process_controls = self.process_controls_msg
        status_msg.show_controls = self.show_controls

        status_msg.has_process_results = self.has_process_results
        if self.has_process_results == True:
            status_msg.process_results_namespace = self.process_results_namespace

        status_msg.show_results = self.show_results and self.has_process_results

        status_msg.show_selector = self.show_selector



        ###########
        if self.node_if is not None:
            if self.status_has_published == False:
                self.msg_if.pub_info("Publishing first status for process: " + str(self.process_name))
                self.status_has_published = True
            self.node_if.publish_pub(self.node_if_prefix + 'status_pub', status_msg) 
        return status_msg


    def publish_results(self, results_msg):

        ###########
        if self.node_if is not None and self.has_process_results == True and results_msg is not None:
            self.node_if.publish_pub(self.node_if_prefix + 'results_pub', results_msg) 


    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this interface."""
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.process_node_pubs_dict is not None:
                    for pub_name in self.process_node_pubs_dict.keys():
                        self.node_if.unregister_pub(pub_name)

    def unsubscribe(self):
        """Shut down this interface, unregister all owned ROS resources, and clear state."""
        self.ready = False
        if self.node_if is not None and self.node_if_shared == False:
            self.node_if.unregister_class()
        else:
            self.unregister_pubs()
        time.sleep(1)
        self.process_namespace = None

    def init(self, do_updates = False):
        """Initialize or re-initialize interface state and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            controls_dict = self.node_if.get_param(self.controls_param_name)
            if controls_dict is not None:
                self.process_controls_dict = controls_dict
            selected_sources = self.node_if.get_param(self.sources_param_name)
            if selected_sources is not None:
                self.selected_sources = selected_sources
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the interface to its initialized state."""
        self.init()

    def factory_reset(self):
        """Reset the interface to factory defaults."""
        self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _updatePubStats(self):
        if self.last_pub_time is None:
            pub_time_sec = 1.0
            self.last_pub_time = nepi_utils.get_time()
        else:
            cur_time = nepi_utils.get_time()
            pub_time_sec = cur_time - self.last_pub_time
            self.last_pub_time = cur_time
        self.time_list.pop(0)
        self.time_list.append(pub_time_sec)

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)


    def _setValueCb(self,msg):
            control_name = msg.name
            # The value setters share this single callback. Most Update* msgs carry
            # a 'value' field; UpdateRangeWindow (FloatSliders) carries start/stop_range
            # and UpdateTrigger (Trigger) carries no value at all.
            if hasattr(msg, 'value'):
                control_value = msg.value
            elif hasattr(msg, 'start_range'):
                control_value = [msg.start_range, msg.stop_range]
            else:
                control_value = nepi_utils.get_time()
            self.set_control_value(control_name, control_value)

    def _publishStatusCb(self, timer):
        self.publish_status()
       



# class ProcessIF:
    
#     msg_if = None
#     node_if = None
#     node_if_shared = False
#     save_data_if = None

#     process_name = None
#     process_namespace = ''
#     process_data_products = []
#     process_status_msg = ProcessStatus
#     process_node_pubs_dict = None
#     process_node_subs_dict = None
#     max_process_rate_hz = 10
#     process_ready = False

#     active_nodes = []
#     active_topics = []
#     active_topic_types =  []
#     active_services =  []  

#     source_status_msg_type = None  

#     available_sources = []
#     available_names = []

#     auto_select_enabled = True
#     auto_select_active = True
#     multi_source_enabled = True
#     exclude_source_filters = []


#     selected_sources_param = []
#     selected_sources = []
#     sources_connecting = []
#     sources_connected = []
#     sources_connected_topics = []
#     sources_status_sub_dict = dict()
#     sources_status_dict = dict()
#     sources_data_sub_dict = dict()
#     sources_data_dict = dict()
#     sources_pubs_dict = dict()
#     sources_stats_dict = dict()

#     source_selected = False
#     source_connected = False

#     show_selector = True
#     show_controls = True
#     show_data = True


#     has_imaging = False
#     max_image_pub_rate_hz = 10
#     imaging_enabled = True
#     use_last_image = False
#     imaging_if_api = None
#     imaging_if_class = None
#     imaging_source_topics = []
#     imaging_pub_topics = []

#     status_has_published = False

#     #######################
#     ### IF Initialization
#     def __init__(self, 
#                 process_name = None,
#                 process_group = 'PROCESS',
#                 process_description = 'Process',
#                 process_status_msg = ProcessStatus,
#                 process_data_products = [],
#                 max_process_rate_hz = 10,
#                 updater_process_enabled = True,
#                 source_status_msg_type = None,
#                 source_data_msg_type = None,       
#                 source_callback_dict = None,       
#                 auto_select_enabled = True,
#                 muti_source_enabled = True,
#                 exclude_source_filters = [],
#                 selected_sources = [],
#                 has_imaging = False,
#                 max_image_pub_rate_hz = 10,
#                 imaging_if_api = None,
#                 imaging_if_class = None,
#                 show_selector = True,
#                 show_controls = True,
#                 show_data = True,
#                 log_name = None,
#                 log_name_list = [],
#                 msg_if = None,
#                 node_if = None,
#                 save_data_if = None
#                 ):
#         ####  IF INIT SETUP ####
#         self.class_name = type(self).__name__
#         self.base_namespace = nepi_sdk.get_base_namespace()
#         self.node_name = nepi_sdk.get_node_name()
#         self.node_namespace = nepi_sdk.get_node_namespace()

#         ##############################  

        
#         # Create Msg Class
#         if msg_if is not None:
#             self.msg_if = msg_if
#         else:
#             self.msg_if = MsgIF()
#         self.log_name_list = copy.deepcopy(log_name_list)
#         self.log_name_list.append(self.class_name)
#         if log_name is not None:
#             log_name = nepi_utils.get_clean_name(log_name)
#             self.log_name_list.append(log_name)
#         self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

#         # Create Namespace
#         self.process_name = nepi_utils.get_clean_name(process_name)
#         if self.process_name is None or self.process_name == '':
#             self.msg_if.pub_warn("Process Name Not Valid: " + str(process_name)) 
#             return
#         self.msg_if.pub_info("Using Process Name: " + self.process_name)
#         self.process_namespace = nepi_sdk.create_namespace(self.node_namespace,self.process_name)
#         self.node_if_prefix = self.process_name + '_'



#         ##############################    
#         # Initialize Class Variables

#         self.process_group = str(process_group)
#         self.process_description = str(process_description)
#         # Check Process Status Msg Type
#         if process_status_msg is not None:
#             self.process_status_msg = process_status_msg

#         self.max_process_rate_hz = max_process_rate_hz

#         # Check Status Msg Type
#         if source_status_msg_type is None:
#             self.msg_if.pub_warn("Source Status Msg Not Provided") 
#         self.source_status_msg_type = source_status_msg_type

#         # Check Status Msg Type
#         if source_data_msg_type is None:
#             self.msg_if.pub_warn("Source Data Msg Not Provided") 
#         self.source_data_msg_type = source_data_msg_type

#         if source_callback_dict is not None:
#             for key in source_callback_dict.keys():
#                 self.source_callback_dict[key] = source_callback_dict[key]
      
#         self.auto_select_enabled = auto_select_enabled
#         self.muti_source_enabled = muti_source_enabled
#         self.exclude_source_filters = exclude_source_filters

#         clean_sources = []
#         for source_topic in selected_sources:
#             clean_sources = nepi_sdk.get_full_namespace(source_topic)
#         self.selected_sources = clean_sources

#         if has_imaging == True and imaging_if_api is not None and imaging_if_class is not None:
#             self.has_imaging = has_imaging
#             self.max_image_pub_rate_hz = max_image_pub_rate_hz
#             self.imaging_if_api = imaging_if_api
#             self.imaging_if_class = imaging_if_class


#         self.show_selector = show_selector
#         self.show_controls = show_controls
#         self.show_data = show_data

                   
#         ##############################   
#         ## Node Setup

#         # Configs Config Dict ####################
#         CFGS_DICT = {
#                 'namespace': self.process_namespace
#         }

#         # Params Config Dict ####################
#         # Persist the selected topic under the connect namespace so the
#         # selection survives node restarts (via the config manager). Passing a
#         # params_dict is what enables config management on NodeClassIF.
#         PARAMS_DICT = {
#             'selected_sources': {
#                 'namespace': self.process_namespace,
#                 'factory_val': self.selected_sources
#             }
#         }


#         # Publishers Config Dict ####################
#         self.process_node_pubs_dict = {
#             'status_pub': {
#                 'namespace': self.process_namespace,
#                 'topic': 'status',
#                 'msg': self.process_status_msg,
#                 'qsize': 1,
#                 'latch': True
#             }
#         }



#         # Subscribers Config Dict ####################

#         self.process_node_subs_dict = {
#             'set_source': {
#                 'namespace': self.process_namespace,
#                 'topic': 'set_source',
#                 'msg': String,
#                 'qsize': None,
#                 'callback': self._setSourceCb, 
#                 'callback_args': ()
#             },
#             'remove_source': {
#                 'namespace': self.process_namespace,
#                 'topic': 'clear_source',
#                 'msg': String,
#                 'qsize': None,
#                 'callback': self._clearSourceCb, 
#                 'callback_args': ()
#             },
#             'system_status': {
#                 'msg': MgrSystemStatus,
#                 'namespace': self.base_namespace,
#                 'topic': 'status',
#                 'qsize': 5,
#                 'callback': self._systemStatusCb
#             },
#         }



#         if self.source_status_msg_type is not None:
#             self.process_node_subs_dict['set_source'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'set_source',
#                 'msg': String,
#                 'qsize': None,
#                 'callback': self._setSourceCb, 
#                 'callback_args': ()
#             }
#             self.process_node_subs_dict['remove_source'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'clear_source',
#                 'msg': String,
#                 'qsize': None,
#                 'callback': self._clearSourceCb, 
#                 'callback_args': ()
#             }
#             self.process_node_subs_dict['system_status'] = {
#                 'msg': MgrSystemStatus,
#                 'namespace': self.base_namespace,
#                 'topic': 'status',
#                 'qsize': 5,
#                 'callback': self._systemStatusCb
#             }

#         if self.multi_source_enabled == True:
#             self.process_node_subs_dict['set_sources'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'set_sources',
#                 'msg': StringArray,
#                 'qsize': 10,
#                 'callback': self.setSourcesCb, 
#                 'callback_args': ()
#             },
#             self.process_node_subs_dict['add_sources'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'add_sources',
#                 'msg': String,
#                 'qsize': 10,
#                 'callback': self.addSourcesCb, 
#                 'callback_args': ()
#             },
#             self.process_node_subs_dict['remove_sources'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'add_sources',
#                 'msg': Empty,
#                 'qsize': 10,
#                 'callback': self.removeSourcesCb, 
#                 'callback_args': ()
#             },
#             self.process_node_subs_dict['clear_sources'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'clear_sources',
#                 'msg': Empty,
#                 'qsize': 10,
#                 'callback': self.clearSourcesCb, 
#                 'callback_args': ()
#             },
        
#         if node_if is None:
#             self.node_if = NodeClassIF(
#                             configs_dict = CFGS_DICT,
#                             params_dict = PARAMS_DICT,
#                             services_dict = None,
#                             pubs_dict = self.process_node_pubs_dict,
#                             subs_dict = self.process_node_subs_dict,
#                             log_name_list = [],
#                             msg_if = self.msg_if
#             )
#             self.node_if.wait_for_ready()
#         else:
#             self.node_if_shared = True
#             try:
#                 self.node_if = node_if
#                 self.node_if.register_pubs(self.process_node_pubs_dict)
#                 self.node_if.register_subs(self.process_node_subs_dict)
#                 # Register the persisted selection param on the shared node_if too.
#                 self.node_if.add_param('selected_sources', self.process_namespace, self.selected_sources)
#                 nepi_sdk.sleep(1)
#             except Exception as e:
#                 self.msg_if.pub_info("Failed to register pubs and subs: " + str(e))
#                 return


#         # Restore any persisted selection. When no explicit topic was requested
#         # (selected_sources == "None"), use the value the config manager restored
#         # for this connect namespace. Otherwise honor the explicit request.
#         self.selected_sources_param = 'selected_sources'
#         if selected_sources == "None":
#             persisted = self.node_if.get_param(self.selected_sources_param)
#             if persisted is not None and persisted != '' and persisted != "None":
#                 selected_sources = persisted
#         self.selected_sources = selected_sources
#         self.msg_if.pub_info("Init Selected Topic: " + str(self.selected_sources))

#         ###############################
#         self.process_data_products = process_data_products
#         self.msg_if.pub_info("####################", log_name_list = self.log_name_list)
#         self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
#         if save_data_if is not None and save_data_if != 'None':
#             self.save_data_if = save_data_if
#             data_products = self.save_data_if.get_data_products()
#             for data_product in self.process_data_products:
#                 if data_product not in data_products:
#                     self.save_data_if.register_data_product(data_product)
#         elif save_data_if != 'None' and len(self.process_data_products) > 0:
#             # Setup Save Data IF Class 
#             self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
#             factory_data_rates= dict()
#             for data_product in self.process_data_products:
#                 factory_data_rates[data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

#             factory_filename_dict = {
#                 'prefix': "", 
#                 'add_timestamp': True, 
#                 'add_ms': True,
#                 'add_us': False,
#                 'suffix': "",
#                 'add_node_name': True
#                 }

#             sd_namespace = self.node_namespace
#             self.save_data_if = SaveDataIF(namespace = sd_namespace,
#                                     data_products = list(self.process_data_products),
#                                     factory_rate_dict = factory_data_rates,
#                                     factory_filename_dict = factory_filename_dict,
#                                     log_name_list = self.log_name_list,
#                                     msg_if = self.msg_if,
#                                     node_if = self.node_if)
#             nepi_sdk.sleep(1)

#         if self.save_data_if is not None:
#             self.save_data_topic = self.save_data_if.get_namespace()
#             self.msg_if.pub_warn("Using save_data namespace: " + str(self.save_data_topic), log_name_list = self.log_name_list)




#         ##############################
#         # Start updater process
#         if updater_process_enabled == True:
#             nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)
#         nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

#         ##############################
#         # Complete Initialization
#         self.process_ready = True
#         self.msg_if.pub_info(str(self.class_name) + " Initialization Complete")
#         ###############################
    

#     #######################
#     # Class Public Methods
#     #######################


#     def get_process_ready_state(self):
#         """Return the ready state of the interface.

#         Returns:
#             bool: True if the interface has completed initialization, False otherwise.
#         """
#         return self.process_ready

#     def wait_for_process_ready(self, timeout = float('inf') ):
#         """Block until the interface is ready or the timeout expires.

#         Args:
#             timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

#         Returns:
#             bool: True if the interface became ready, False if the timeout was reached.
#         """
#         success = False
#         if self.process_ready is not None:
#             self.msg_if.pub_info("Waiting for connection")
#             timer = 0
#             time_start = nepi_sdk.get_time()
#             while self.process_ready == False and timer < timeout and not nepi_sdk.is_shutdown():
#                 nepi_sdk.sleep(.1)
#                 timer = nepi_sdk.get_time() - time_start
#             if self.process_ready == False:
#                 self.msg_if.pub_info("Failed to Connect")
#             else:
#                 self.msg_if.pub_info("Connected")
#         return self.process_ready  

#     def get_namespace(self):
#         """Return the fully-resolved ROS namespace for the sources_connected PTX device.

#         Returns:
#             str: The fully-qualified namespace string used for topic and service resolution.
#         """
#         return self.process_namespace
    

#     def get_available_sources(self):
#         return self.available_sources
    

#     def get_available_names(self, available_sources = []):
#         available_names = []
#         for topic in available_sources:
#             name = topic
#             topic = topic[1:]
#             topic_split = topic.split('/')
#             if len(topic_split) > 2:
#                 name = topic_split[2]
#             available_names.append(name)
#         return available_names

    
#     def get_selected_sources(self):
#         return self.selected_sources
    
#     def set_selected_sources(self, selected_sources):
#         if selected_sources in self.available_sources or selected_sources == "None":
#             self.selected_sources = selected_sources
#         self.publish_status()
#         # Persist the selection so it survives a node restart. set_param writes
#         # the ROS param; save_config asks the config manager to save it to file.
#         if self.node_if is not None:
#             self.msg_if.pub_warn("selected_sources: " + str(selected_sources))
#             self.node_if.set_param('selected_sources', self.selected_sources)
#             self.node_if.save_config()
    

#     def check_connection(self):
#         """Check whether the device is currently sources_connected.

#         Returns:
#             bool: True if a status message has been received within the connection timeout window,
#                 False otherwise.
#         """
#         return self.sources_connected
    
#     def check_connections(self, source_topic):
#         """Check whether the device is currently sources_connected.

#         Returns:
#             bool: True if a status message has been received within the connection timeout window,
#                 False otherwise.
#         """
#         connected = False
#         if source_topic in self.sources_connected_topics:
#             connected = True
#         return self.sources_connected

#     def wait_for_connection(self, timeout = float('inf') ):
#         """Block until the device is sources_connected or the timeout expires.

#         Args:
#             timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

#         Returns:
#             bool: True if connection was established, False if the timeout was reached.
#         """
#         if self.node_if is not None:
#             self.msg_if.pub_info("Waiting for connection")
#             timer = 0
#             time_start = nepi_sdk.get_time()
#             while self.sources_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
#                 nepi_sdk.sleep(.1)
#                 timer = nepi_sdk.get_time() - time_start
#             if self.sources_connected == False:
#                 self.msg_if.pub_info("Failed to Connect")
#             else:
#                 self.msg_if.pub_info("Connected")
#         return self.sources_connected


#     def unregister(self):
#         success = False
#         self.unsubscribe_topic()
#         if self.node_if is not None:
#             if self.node_if_shared == False:
#                 self.node_if.unregister_class()
#                 nepi_sdk.sleep(1)
#             else:
#                 self.unsubscribe_topic()

#                 if self.node_if is not None:
#                     if self.process_node_subs_dict is not None:
#                         for sub_name in self.process_node_subs_dict.keys():
#                             self.node_if.unregister_sub(sub_name)
#                 self.process_node_subs_dict = None

#                 if self.node_if is not None:
#                     if self.process_node_pubs_dict is not None:
#                         for pub_name in self.process_node_pubs_dict.keys():
#                             self.node_if.unregister_pub(pub_name)
#                 self.process_node_pubs_dict = None
                
#         time.sleep(1)
#         try:
#             self.node_if = None
#             self.selected_sources = 'None'
#             self.connecting = False 
#             self.sources_connected = False 
#             self.sources_connected_topics = 'None'
#             success = True
#         except Exception as e:
#             self.msg_if.pub_warn("Failed to unregister:  " + str(e))
#         return success



#     def get_process_status_msg(self):

#         available_sources = copy.deepcopy(self.available_sources)
#         selected_sources = copy.deepcopy(self.selected_sources)
#         status_msg = ProcessStatus()

#         status_msg.name = self.process_name
#         status_msg.id = self.process_id

#         status_msg.status_msg_type = self.process_status_msg

#         status_msg.available_sources = available_sources
#         available_names = self.get_available_names(available_sources)
#         status_msg.available_names = available_names

#         selected_name = 'None'
#         if selected_sources not in available_sources:
#             if len(available_sources) > 0 and self.auto_select_enabled == True and self.auto_select_active == True:
#                 selected_sources = [available_sources[0]]
#                 self.selected_sources = selected_sources
#             else:
#                 selected_sources = 'None' 

#         if selected_sources in available_sources:
#             selected_ind = available_sources.index(selected_sources)
#             selected_name = available_names[selected_ind]

#         status_msg.selected_sources = selected_sources
#         status_msg.selected_name = selected_name

#         status_msg.connecting = self.connecting
#         status_msg.sources_connected = self.sources_connected
#         sources_connected_topics = self.sources_connected_topics
#         if sources_connected_topics is None:
#             sources_connected_topics = 'None'
#         status_msg.sources_connected_topics = sources_connected_topics

#         connect_msg = "Not Selected"
#         if self.selected_sources != "None":
#             connect_msg = "Selected"
#             if self.connecting == True:
#                 connect_msg = "Connecting"
#             if self.sources_connected == True:
#                 connect_msg = "Connected"
#         status_msg.connect_msg = connect_msg


#         status_msg.show_selector = self.show_selector
#         status_msg.show_controls = self.show_controls
#         status_msg.show_data = self.show_data


#         # ###########
#         # if self.node_if is not None:
#         #     if self.status_has_published == False:
#         #         self.msg_if.pub_warn("Publishing Status: " + str(status_msg))
#         #         self.status_has_published = True
#         #     self.node_if.publish_pub('status_pub', status_msg) 
#         #     #self.node_if.save_config()
#         return status_msg

#     def publish_status(self, status_msg):
#         ###########
#         if self.node_if is not None:
#             if self.status_has_published == False:
#                 self.msg_if.pub_warn("Publishing Status: " + str(status_msg))
#                 self.status_has_published = True
#             self.node_if.publish_pub('status_pub', status_msg) 
#         return status_msg


#     #######################
#     # Class Private Methods
#     #######################

#     # ROS callback for the system status msg. Populates the active topic/type
#     # lists that discovery searches. NOTE: this MUST NOT share a name with the
#     # discovery timer below -- a duplicate name silently shadows this method, so
#     # active_topics never gets populated and discovery finds nothing.
#     def _systemStatusCb(self,msg):
#             self.active_nodes = msg.active_nodes
#             self.active_topics = msg.active_topics
#             self.active_topic_types = msg.active_topic_types
#             self.active_services = msg.active_services


#     # Discovery/connection timer. Finds available topics of the connect status
#     # msg type among the active topics, auto-selects, and subscribes.
#     def _updaterCb(self,timer):
#         needs_publish = False
#         ##############

#         selected_sources = copy.deepcopy(self.selected_sources)
#         last_available = copy.deepcopy(self.available_sources)

#         topics = nepi_sdk.find_topics_by_msg(self.connect_status_msg, topics_list = self.active_topics, types_list = self.active_topic_types)
#         available_sources = []
#         for topic in topics:
#             valid = True
#             for filter in self.exclude_source_filters:
#                 if filter in topic:
#                     valid = False
#             if valid == True:
#                 available_sources.append(topic.replace('/status',''))
#         if available_sources != last_available:
#             self.available_sources = available_sources
#             needs_publish = True

#         ####################
#         if self.sources_connected_topics is not None:
#             if self.sources_connected_topics not in self.available_sources:
#                 success = self.unsubscribe_topic()
#         if selected_sources == 'None' and len(self.available_sources) > 0:
#             self.selected_sources = self.available_sources[0]
#         needs_publish = True

#         was_sources_connected = copy.deepcopy(self.sources_connected)
#         if self.selected_sources in self.available_sources and self.sources_connected_topics != selected_sources:
#             success = self.subscribe_source(self.selected_sources)
#         elif self.selected_sources not in self.available_sources:
#             self.sources_connected = False
#         # else: already subscribed to the selected topic -- leave self.sources_connected
#         # to the status callback (sets True on each msg) and the staleness check
#         # below, so it does not get clobbered False every cycle.

#         ##################
#         cur_time = nepi_utils.get_time()
#         last_time = copy.deepcopy(self.last_status_time )
#         for i, source_topic in enumerate(self.sources_connected_topics):
#             connected = self.sources_connected[i]
#             if connected == True:
#                 if (cur_time - last_time) > CONNECTED_TIMEOUT:
#                     self.sources_connecting[i] = False 
#                     self.sources_connected[i] = False 
#                     self.sources_status_dict[i] = None
#                     self.sources_status_msg[i] = None



#         ##################
#         # Get settings from param server
#         # if needs_publish == True:
#         #   self.publish_status()
#         nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)


#########################################
# Detections Data Product IF Class
#########################################

class DetectionsIF:
    """Per-data-product interface for the AI detector 'detections' product.

    Owns the publishers, status message, and save-data registration for a
    single detections data product. Publishes a nepi_interfaces/Detections
    message on the data topic and a nepi_interfaces/DetectorStatus message on
    the status topic, and saves detection results through a SaveDataIF. Follows
    the standalone per-data-product IF convention used by the data_if.py
    classes (NavPoseIF, DepthMapIF).
    """

    ready = False

    data_product = 'detections'

    namespace = '~/detections'

    node_if = None
    node_if_shared = True

    save_data_if = None
    save_data_enabled = True

    status_msg = DetectorStatus()

    data_msg_type = Detections
    status_msg_type = DetectorStatus

    data_pub_name = 'detections_pub'
    status_pub_name = 'detections_status_pub'

    needs_data = False
    last_pub_time = None
    time_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    PUBS_DICT = dict()

    def __init__(self, namespace = None,
                data_product = None,
                save_data_if = None,
                save_data_enabled = True,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
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

        ##############################
        # Initialize Class Variables
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product

        if namespace is None:
            namespace = self.node_namespace
        if os.path.basename(namespace) != self.data_product:
            namespace = nepi_sdk.create_namespace(namespace, self.data_product)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        # Initialize status message
        self.status_msg = self.status_msg_type()
        self.status_msg.process_status.node_name = self.node_name
        self.status_msg.process_status.namespace = self.namespace

        ##############################
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.status_pub_name: {
                'msg': self.status_msg_type,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': False
            },
            self.data_pub_name: {
                'msg': self.data_msg_type,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = None

        # Update or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            self.node_if.register_pubs(self.PUBS_DICT)
        else:
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()

        ####################
        # Save Data Setup
        self.save_data_enabled = save_data_enabled
        if self.save_data_enabled == True:
            self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
            if save_data_if is not None and save_data_if != 'None':
                self.save_data_if = save_data_if
                data_products = self.save_data_if.get_data_products()
                if self.data_product not in data_products:
                    self.save_data_if.register_data_product(self.data_product)
            elif save_data_if != 'None':
                # Setup Save Data IF Class
                self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
                factory_data_rates = dict()
                factory_data_rates[self.data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

                factory_filename_dict = {
                    'prefix': "",
                    'add_timestamp': True,
                    'add_ms': True,
                    'add_us': False,
                    'suffix': "",
                    'add_node_name': True
                }

                self.save_data_if = SaveDataIF(namespace = self.node_namespace,
                                        data_products = [self.data_product],
                                        factory_rate_dict = factory_data_rates,
                                        factory_filename_dict = factory_filename_dict,
                                        log_name_list = self.log_name_list,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if)
                nepi_sdk.sleep(1)

            if self.save_data_if is not None:
                self.status_msg.process_status.save_data_topic = self.save_data_if.get_namespace()
                self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.process_status.save_data_topic), log_name_list = self.log_name_list)

        ##############################
        # Update vals and publish first status
        self.init(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return the current ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf')):
        """Block until the interface is ready or a timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for ready", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace for this data product interface.

        Returns:
            str: The fully-qualified ROS namespace.
        """
        return self.namespace

    def get_data_product(self):
        """Return the data product name for this interface.

        Returns:
            str: The data product identifier string (e.g. 'detections').
        """
        return self.data_product

    def get_status_dict(self):
        """Return the current status message converted to a plain dictionary.

        Returns:
            dict: Status fields as a dictionary, or None if no status message exists.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def needs_data_check(self):
        """Return whether downstream consumers currently need detections data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        return copy.deepcopy(self.needs_data)

    def publish_data(self, data_msg, timestamp = None):
        """Publish a detections message and save it if saving is enabled.

        Publishes the provided data message on the data product topic, updates
        the publish-rate statistics, and — if a SaveDataIF is registered and the
        data product is due to be saved or snapshotted — converts the message to
        a dictionary and saves it.

        Args:
            data_msg (nepi_interfaces/Detections): The detections message to publish.
            timestamp (float, optional): Acquisition timestamp in seconds. Defaults
                to the current time when None.

        Returns:
            bool: True if the message was published, False otherwise.
        """
        if self.node_if is None or data_msg is None:
            return False
        self.node_if.publish_pub(self.data_pub_name, data_msg)
        self._updatePubStats()
        if self.save_data_if is not None:
            should_save = self.save_data_if.data_product_should_save(self.data_product) == True
            snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(self.data_product) == True
            if should_save or snapshot_enabled:
                if timestamp is None:
                    timestamp = nepi_utils.get_time()
                try:
                    data_dict = nepi_sdk.convert_msg2dict(data_msg)
                    self.save_data_if.save(self.data_product, data_dict, timestamp = timestamp)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to save " + self.data_product + " data: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
        return True

    def publish_status(self, status_msg = None):
        """Publish the data product status message.

        Args:
            status_msg (optional): A pre-built status message to publish and store.
                When None, the interface's current status message is published.
        """
        if self.node_if is None:
            return
        if status_msg is not None:
            self.status_msg = status_msg
        if self.status_msg is not None:
            self.node_if.publish_pub(self.status_pub_name, self.status_msg)

    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this interface."""
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.PUBS_DICT is not None:
                    for pub_name in self.PUBS_DICT.keys():
                        self.node_if.unregister_pub(pub_name)

    def unsubscribe(self):
        """Shut down this interface, unregister all owned ROS resources, and clear state."""
        self.ready = False
        if self.node_if is not None and self.node_if_shared == False:
            self.node_if.unregister_class()
        else:
            self.unregister_pubs()
        time.sleep(1)
        self.namespace = None

    def init(self, do_updates = False):
        """Initialize or re-initialize interface state and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the interface to its initialized state."""
        self.init()

    def factory_reset(self):
        """Reset the interface to factory defaults."""
        self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _updatePubStats(self):
        if self.last_pub_time is None:
            pub_time_sec = 1.0
            self.last_pub_time = nepi_utils.get_time()
        else:
            cur_time = nepi_utils.get_time()
            pub_time_sec = cur_time - self.last_pub_time
            self.last_pub_time = cur_time
        self.time_list.pop(0)
        self.time_list.append(pub_time_sec)

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _needsDataCheckCb(self, timer):
        has_subs = self.node_if.pub_has_subscribers(self.data_pub_name)
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
            needs_data = has_subs or needs_save or needs_snapshot
        else:
            needs_data = has_subs
        self.needs_data = needs_data
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)

    def _publishStatusCb(self, timer):
        self.publish_status()


#########################################
# Detections Image Data Product IF Class
#########################################

class DetectionsImageIF:
    """Per-data-product interface for the AI detector 'detections_image' product.

    Owns the publishers, status message, and save-data registration for the
    detections overlay image data product. Publishes a sensor_msgs/Image message
    on the data topic and a nepi_interfaces/ImageStatus message on the status
    topic, and saves the rendered image through a SaveDataIF. Mirrors the
    standalone image data-product convention used by the data_if.py image
    classes.
    """

    ready = False

    data_product = 'detections_image'

    namespace = '~/detections_image'

    node_if = None
    node_if_shared = True

    save_data_if = None
    save_data_enabled = True

    status_msg = ImageStatus()

    data_msg_type = Image
    status_msg_type = ImageStatus

    data_pub_name = 'detections_image_pub'
    status_pub_name = 'detections_image_status_pub'

    needs_data = False
    last_pub_time = None
    time_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    PUBS_DICT = dict()

    def __init__(self, namespace = None,
                data_product = None,
                save_data_if = None,
                save_data_enabled = True,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
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

        ##############################
        # Initialize Class Variables
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product

        if namespace is None:
            namespace = self.node_namespace
        if os.path.basename(namespace) != self.data_product:
            namespace = nepi_sdk.create_namespace(namespace, self.data_product)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        # Initialize status message
        self.status_msg = self.status_msg_type()
        self.status_msg.node_name = self.node_name
        self.status_msg.image_topic = self.namespace

        ##############################
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.status_pub_name: {
                'msg': self.status_msg_type,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': False
            },
            self.data_pub_name: {
                'msg': self.data_msg_type,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = None

        # Update or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            self.node_if.register_pubs(self.PUBS_DICT)
        else:
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()

        ####################
        # Save Data Setup
        self.save_data_enabled = save_data_enabled
        if self.save_data_enabled == True:
            self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
            if save_data_if is not None and save_data_if != 'None':
                self.save_data_if = save_data_if
                data_products = self.save_data_if.get_data_products()
                if self.data_product not in data_products:
                    self.save_data_if.register_data_product(self.data_product)
            elif save_data_if != 'None':
                # Setup Save Data IF Class
                self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
                factory_data_rates = dict()
                factory_data_rates[self.data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

                factory_filename_dict = {
                    'prefix': "",
                    'add_timestamp': True,
                    'add_ms': True,
                    'add_us': False,
                    'suffix': "",
                    'add_node_name': True
                }

                self.save_data_if = SaveDataIF(namespace = self.node_namespace,
                                        data_products = [self.data_product],
                                        factory_rate_dict = factory_data_rates,
                                        factory_filename_dict = factory_filename_dict,
                                        log_name_list = self.log_name_list,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if)
                nepi_sdk.sleep(1)

            if self.save_data_if is not None:
                self.status_msg.save_data_topic = self.save_data_if.get_namespace()
                self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic), log_name_list = self.log_name_list)

        ##############################
        # Update vals and publish first status
        self.init(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return the current ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf')):
        """Block until the interface is ready or a timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for ready", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace for this data product interface.

        Returns:
            str: The fully-qualified ROS namespace.
        """
        return self.namespace

    def get_data_product(self):
        """Return the data product name for this interface.

        Returns:
            str: The data product identifier string (e.g. 'detections_image').
        """
        return self.data_product

    def get_status_dict(self):
        """Return the current status message converted to a plain dictionary.

        Returns:
            dict: Status fields as a dictionary, or None if no status message exists.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def needs_data_check(self):
        """Return whether downstream consumers currently need image data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        return copy.deepcopy(self.needs_data)

    def publish_data(self, data_msg, timestamp = None):
        """Publish a detections image message and save it if saving is enabled.

        Publishes the provided ROS Image message on the data product topic,
        updates the publish-rate statistics, and — if a SaveDataIF is registered
        and the data product is due to be saved or snapshotted — converts the
        image to a cv2 image and saves it.

        Args:
            data_msg (sensor_msgs/Image): The image message to publish.
            timestamp (float, optional): Acquisition timestamp in seconds. Defaults
                to the current time when None.

        Returns:
            bool: True if the message was published, False otherwise.
        """
        if self.node_if is None or data_msg is None:
            return False
        self.node_if.publish_pub(self.data_pub_name, data_msg)
        self._updatePubStats()
        if self.save_data_if is not None:
            should_save = self.save_data_if.data_product_should_save(self.data_product) == True
            snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(self.data_product) == True
            if should_save or snapshot_enabled:
                if timestamp is None:
                    timestamp = nepi_utils.get_time()
                try:
                    cv2_img = nepi_img.rosimg_to_cv2img(data_msg)
                    self.save_data_if.save(self.data_product, cv2_img, timestamp = timestamp)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to save " + self.data_product + " data: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
        return True

    def publish_cv2_image(self, cv2_img, encoding = 'bgr8', timestamp = None):
        """Convert a cv2 image to a ROS Image message and publish it.

        Args:
            cv2_img (numpy.ndarray): The OpenCV image to publish.
            encoding (str, optional): Image encoding to use for conversion.
                Defaults to 'bgr8'.
            timestamp (float, optional): Acquisition timestamp in seconds. Defaults
                to the current time when None.

        Returns:
            bool: True if the image was converted and published, False otherwise.
        """
        if cv2_img is None:
            return False
        try:
            data_msg = nepi_img.cv2img_to_rosimg(cv2_img, encoding = encoding)
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert cv2 image to ros image: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
            return False
        return self.publish_data(data_msg, timestamp = timestamp)

    def publish_status(self, status_msg = None):
        """Publish the image data product status message.

        Args:
            status_msg (optional): A pre-built status message to publish and store.
                When None, the interface's current status message is published.
        """
        if self.node_if is None:
            return
        if status_msg is not None:
            self.status_msg = status_msg
        if self.status_msg is not None:
            self.node_if.publish_pub(self.status_pub_name, self.status_msg)

    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this interface."""
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.PUBS_DICT is not None:
                    for pub_name in self.PUBS_DICT.keys():
                        self.node_if.unregister_pub(pub_name)

    def unsubscribe(self):
        """Shut down this interface, unregister all owned ROS resources, and clear state."""
        self.ready = False
        if self.node_if is not None and self.node_if_shared == False:
            self.node_if.unregister_class()
        else:
            self.unregister_pubs()
        time.sleep(1)
        self.namespace = None

    def init(self, do_updates = False):
        """Initialize or re-initialize interface state and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the interface to its initialized state."""
        self.init()

    def factory_reset(self):
        """Reset the interface to factory defaults."""
        self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _updatePubStats(self):
        if self.last_pub_time is None:
            pub_time_sec = 1.0
            self.last_pub_time = nepi_utils.get_time()
        else:
            cur_time = nepi_utils.get_time()
            pub_time_sec = cur_time - self.last_pub_time
            self.last_pub_time = cur_time
        self.time_list.pop(0)
        self.time_list.append(pub_time_sec)

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _needsDataCheckCb(self, timer):
        has_subs = self.node_if.pub_has_subscribers(self.data_pub_name)
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
            needs_data = has_subs or needs_save or needs_snapshot
        else:
            needs_data = has_subs
        self.needs_data = needs_data
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)

    def _publishStatusCb(self, timer):
        self.publish_status()




#########################################
# Targets Data Product IF Class
#########################################

class TargetsIF:
    """Per-data-product interface for the AI detector 'targets' product.

    Owns the publishers, status message, and save-data registration for a
    single targets data product. Publishes a nepi_interfaces/Targets message on
    the data topic and a nepi_interfaces/TargetingStatus message on the status
    topic, and saves targeting results through a SaveDataIF. Mirrors DetectionsIF
    using the targeting message types and the targeting namespace.
    """

    ready = False

    data_product = 'targets'

    namespace = '~/targets'

    node_if = None
    node_if_shared = True

    save_data_if = None
    save_data_enabled = True

    status_msg = TargetingStatus()

    data_msg_type = Targets
    status_msg_type = TargetingStatus

    data_pub_name = 'targets_pub'
    status_pub_name = 'targets_status_pub'

    needs_data = False
    last_pub_time = None
    time_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    PUBS_DICT = dict()

    def __init__(self, namespace = None,
                data_product = None,
                save_data_if = None,
                save_data_enabled = True,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
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

        ##############################
        # Initialize Class Variables
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product

        if namespace is None:
            namespace = self.node_namespace
        if os.path.basename(namespace) != self.data_product:
            namespace = nepi_sdk.create_namespace(namespace, self.data_product)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        # Initialize status message
        self.status_msg = self.status_msg_type()
        self.status_msg.process_status.node_name = self.node_name
        self.status_msg.process_status.namespace = self.namespace

        ##############################
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.status_pub_name: {
                'msg': self.status_msg_type,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': False
            },
            self.data_pub_name: {
                'msg': self.data_msg_type,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = None

        # Update or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            self.node_if.register_pubs(self.PUBS_DICT)
        else:
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()

        ####################
        # Save Data Setup
        self.save_data_enabled = save_data_enabled
        if self.save_data_enabled == True:
            self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
            if save_data_if is not None and save_data_if != 'None':
                self.save_data_if = save_data_if
                data_products = self.save_data_if.get_data_products()
                if self.data_product not in data_products:
                    self.save_data_if.register_data_product(self.data_product)
            elif save_data_if != 'None':
                # Setup Save Data IF Class
                self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
                factory_data_rates = dict()
                factory_data_rates[self.data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

                factory_filename_dict = {
                    'prefix': "",
                    'add_timestamp': True,
                    'add_ms': True,
                    'add_us': False,
                    'suffix': "",
                    'add_node_name': True
                }

                self.save_data_if = SaveDataIF(namespace = self.node_namespace,
                                        data_products = [self.data_product],
                                        factory_rate_dict = factory_data_rates,
                                        factory_filename_dict = factory_filename_dict,
                                        log_name_list = self.log_name_list,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if)
                nepi_sdk.sleep(1)

            if self.save_data_if is not None:
                self.status_msg.process_status.save_data_topic = self.save_data_if.get_namespace()
                self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.process_status.save_data_topic), log_name_list = self.log_name_list)

        ##############################
        # Update vals and publish first status
        self.init(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return the current ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf')):
        """Block until the interface is ready or a timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for ready", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace for this data product interface.

        Returns:
            str: The fully-qualified ROS namespace.
        """
        return self.namespace

    def get_data_product(self):
        """Return the data product name for this interface.

        Returns:
            str: The data product identifier string (e.g. 'targets').
        """
        return self.data_product

    def get_status_dict(self):
        """Return the current status message converted to a plain dictionary.

        Returns:
            dict: Status fields as a dictionary, or None if no status message exists.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def needs_data_check(self):
        """Return whether downstream consumers currently need targets data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        return copy.deepcopy(self.needs_data)

    def publish_data(self, data_msg, timestamp = None):
        """Publish a targets message and save it if saving is enabled.

        Publishes the provided data message on the data product topic, updates
        the publish-rate statistics, and — if a SaveDataIF is registered and the
        data product is due to be saved or snapshotted — converts the message to
        a dictionary and saves it.

        Args:
            data_msg (nepi_interfaces/Targets): The targets message to publish.
            timestamp (float, optional): Acquisition timestamp in seconds. Defaults
                to the current time when None.

        Returns:
            bool: True if the message was published, False otherwise.
        """
        if self.node_if is None or data_msg is None:
            return False
        self.node_if.publish_pub(self.data_pub_name, data_msg)
        self._updatePubStats()
        if self.save_data_if is not None:
            should_save = self.save_data_if.data_product_should_save(self.data_product) == True
            snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(self.data_product) == True
            if should_save or snapshot_enabled:
                if timestamp is None:
                    timestamp = nepi_utils.get_time()
                try:
                    data_dict = nepi_sdk.convert_msg2dict(data_msg)
                    self.save_data_if.save(self.data_product, data_dict, timestamp = timestamp)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to save " + self.data_product + " data: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
        return True

    def publish_status(self, status_msg = None):
        """Publish the data product status message.

        Args:
            status_msg (optional): A pre-built status message to publish and store.
                When None, the interface's current status message is published.
        """
        if self.node_if is None:
            return
        if status_msg is not None:
            self.status_msg = status_msg
        if self.status_msg is not None:
            self.node_if.publish_pub(self.status_pub_name, self.status_msg)

    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this interface."""
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.PUBS_DICT is not None:
                    for pub_name in self.PUBS_DICT.keys():
                        self.node_if.unregister_pub(pub_name)

    def unsubscribe(self):
        """Shut down this interface, unregister all owned ROS resources, and clear state."""
        self.ready = False
        if self.node_if is not None and self.node_if_shared == False:
            self.node_if.unregister_class()
        else:
            self.unregister_pubs()
        time.sleep(1)
        self.namespace = None

    def init(self, do_updates = False):
        """Initialize or re-initialize interface state and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the interface to its initialized state."""
        self.init()

    def factory_reset(self):
        """Reset the interface to factory defaults."""
        self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _updatePubStats(self):
        if self.last_pub_time is None:
            pub_time_sec = 1.0
            self.last_pub_time = nepi_utils.get_time()
        else:
            cur_time = nepi_utils.get_time()
            pub_time_sec = cur_time - self.last_pub_time
            self.last_pub_time = cur_time
        self.time_list.pop(0)
        self.time_list.append(pub_time_sec)

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _needsDataCheckCb(self, timer):
        has_subs = self.node_if.pub_has_subscribers(self.data_pub_name)
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
            needs_data = has_subs or needs_save or needs_snapshot
        else:
            needs_data = has_subs
        self.needs_data = needs_data
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)

    def _publishStatusCb(self, timer):
        self.publish_status()


#########################################
# Targets Image Data Product IF Class
#########################################

class TargetsImageIF:
    """Per-data-product interface for the AI detector 'targets_image' product.

    Owns the publishers, status message, and save-data registration for the
    targets overlay image data product. Publishes a sensor_msgs/Image message on
    the data topic and a nepi_interfaces/ImageStatus message on the status
    topic, and saves the rendered image through a SaveDataIF. Mirrors
    DetectionsImageIF using the targeting namespace.
    """

    ready = False

    data_product = 'targets_image'

    namespace = '~/targets_image'

    node_if = None
    node_if_shared = True

    save_data_if = None
    save_data_enabled = True

    status_msg = ImageStatus()

    data_msg_type = Image
    status_msg_type = ImageStatus

    data_pub_name = 'targets_image_pub'
    status_pub_name = 'targets_image_status_pub'

    needs_data = False
    last_pub_time = None
    time_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    PUBS_DICT = dict()

    def __init__(self, namespace = None,
                data_product = None,
                save_data_if = None,
                save_data_enabled = True,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
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

        ##############################
        # Initialize Class Variables
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product

        if namespace is None:
            namespace = self.node_namespace
        if os.path.basename(namespace) != self.data_product:
            namespace = nepi_sdk.create_namespace(namespace, self.data_product)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        # Initialize status message
        self.status_msg = self.status_msg_type()
        self.status_msg.node_name = self.node_name
        self.status_msg.image_topic = self.namespace

        ##############################
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.status_pub_name: {
                'msg': self.status_msg_type,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': False
            },
            self.data_pub_name: {
                'msg': self.data_msg_type,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = None

        # Update or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            self.node_if.register_pubs(self.PUBS_DICT)
        else:
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()

        ####################
        # Save Data Setup
        self.save_data_enabled = save_data_enabled
        if self.save_data_enabled == True:
            self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
            if save_data_if is not None and save_data_if != 'None':
                self.save_data_if = save_data_if
                data_products = self.save_data_if.get_data_products()
                if self.data_product not in data_products:
                    self.save_data_if.register_data_product(self.data_product)
            elif save_data_if != 'None':
                # Setup Save Data IF Class
                self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
                factory_data_rates = dict()
                factory_data_rates[self.data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

                factory_filename_dict = {
                    'prefix': "",
                    'add_timestamp': True,
                    'add_ms': True,
                    'add_us': False,
                    'suffix': "",
                    'add_node_name': True
                }

                self.save_data_if = SaveDataIF(namespace = self.node_namespace,
                                        data_products = [self.data_product],
                                        factory_rate_dict = factory_data_rates,
                                        factory_filename_dict = factory_filename_dict,
                                        log_name_list = self.log_name_list,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if)
                nepi_sdk.sleep(1)

            if self.save_data_if is not None:
                self.status_msg.save_data_topic = self.save_data_if.get_namespace()
                self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic), log_name_list = self.log_name_list)

        ##############################
        # Update vals and publish first status
        self.init(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return the current ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf')):
        """Block until the interface is ready or a timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for ready", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace for this data product interface.

        Returns:
            str: The fully-qualified ROS namespace.
        """
        return self.namespace

    def get_data_product(self):
        """Return the data product name for this interface.

        Returns:
            str: The data product identifier string (e.g. 'targets_image').
        """
        return self.data_product

    def get_status_dict(self):
        """Return the current status message converted to a plain dictionary.

        Returns:
            dict: Status fields as a dictionary, or None if no status message exists.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def needs_data_check(self):
        """Return whether downstream consumers currently need image data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        return copy.deepcopy(self.needs_data)

    def publish_data(self, data_msg, timestamp = None):
        """Publish a targets image message and save it if saving is enabled.

        Publishes the provided ROS Image message on the data product topic,
        updates the publish-rate statistics, and — if a SaveDataIF is registered
        and the data product is due to be saved or snapshotted — converts the
        image to a cv2 image and saves it.

        Args:
            data_msg (sensor_msgs/Image): The image message to publish.
            timestamp (float, optional): Acquisition timestamp in seconds. Defaults
                to the current time when None.

        Returns:
            bool: True if the message was published, False otherwise.
        """
        if self.node_if is None or data_msg is None:
            return False
        self.node_if.publish_pub(self.data_pub_name, data_msg)
        self._updatePubStats()
        if self.save_data_if is not None:
            should_save = self.save_data_if.data_product_should_save(self.data_product) == True
            snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(self.data_product) == True
            if should_save or snapshot_enabled:
                if timestamp is None:
                    timestamp = nepi_utils.get_time()
                try:
                    cv2_img = nepi_img.rosimg_to_cv2img(data_msg)
                    self.save_data_if.save(self.data_product, cv2_img, timestamp = timestamp)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to save " + self.data_product + " data: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
        return True

    def publish_cv2_image(self, cv2_img, encoding = 'bgr8', timestamp = None):
        """Convert a cv2 image to a ROS Image message and publish it.

        Args:
            cv2_img (numpy.ndarray): The OpenCV image to publish.
            encoding (str, optional): Image encoding to use for conversion.
                Defaults to 'bgr8'.
            timestamp (float, optional): Acquisition timestamp in seconds. Defaults
                to the current time when None.

        Returns:
            bool: True if the image was converted and published, False otherwise.
        """
        if cv2_img is None:
            return False
        try:
            data_msg = nepi_img.cv2img_to_rosimg(cv2_img, encoding = encoding)
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert cv2 image to ros image: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
            return False
        return self.publish_data(data_msg, timestamp = timestamp)

    def publish_status(self, status_msg = None):
        """Publish the image data product status message.

        Args:
            status_msg (optional): A pre-built status message to publish and store.
                When None, the interface's current status message is published.
        """
        if self.node_if is None:
            return
        if status_msg is not None:
            self.status_msg = status_msg
        if self.status_msg is not None:
            self.node_if.publish_pub(self.status_pub_name, self.status_msg)

    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this interface."""
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.PUBS_DICT is not None:
                    for pub_name in self.PUBS_DICT.keys():
                        self.node_if.unregister_pub(pub_name)

    def unsubscribe(self):
        """Shut down this interface, unregister all owned ROS resources, and clear state."""
        self.ready = False
        if self.node_if is not None and self.node_if_shared == False:
            self.node_if.unregister_class()
        else:
            self.unregister_pubs()
        time.sleep(1)
        self.namespace = None

    def init(self, do_updates = False):
        """Initialize or re-initialize interface state and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the interface to its initialized state."""
        self.init()

    def factory_reset(self):
        """Reset the interface to factory defaults."""
        self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _updatePubStats(self):
        if self.last_pub_time is None:
            pub_time_sec = 1.0
            self.last_pub_time = nepi_utils.get_time()
        else:
            cur_time = nepi_utils.get_time()
            pub_time_sec = cur_time - self.last_pub_time
            self.last_pub_time = cur_time
        self.time_list.pop(0)
        self.time_list.append(pub_time_sec)

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _needsDataCheckCb(self, timer):
        has_subs = self.node_if.pub_has_subscribers(self.data_pub_name)
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
            needs_data = has_subs or needs_save or needs_snapshot
        else:
            needs_data = has_subs
        self.needs_data = needs_data
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)

    def _publishStatusCb(self, timer):
        self.publish_status()


# class ProcessIF:
    
#     msg_if = None
#     node_if = None
#     node_if_shared = False
#     save_data_if = None

#     process_name = None
#     process_namespace = ''
#     process_data_products = []
#     process_module = None

#     process_base_msg = ProcessStatus()
#     process_status_msg = None
#     process_status_dict = dict()
#     process_data_msg = DataStatus()
#     process_data_dict = dict()
#     process_controls_msg = ControlsStatus()
#     process_controls_dict = dict()
#     prpcess_results_msg = ResultsStatus()
#     process_results_dict = dict()

#     process_node_pubs_dict = None
#     process_node_subs_dict = None
#     max_process_rate_hz = 10
#     process_updater_function = None
#     process_ready = False

#     active_nodes = []
#     active_topics = []
#     active_topic_types =  []
#     active_services =  []  

#     source_status_msg_type = None  


#     available_sources = []
#     available_names = []

#     auto_select_enabled = True
#     auto_select_active = True
#     multi_source_enabled = True
#     exclude_source_filters = []


#     selected_sources_param = []
#     selected_sources = []
#     sources_connecting = []
#     sources_connected = []
#     sources_connected_topics = []
#     sources_status_sub_dict = dict()
#     sources_status_dict = dict()
#     sources_data_sub_dict = dict()
#     sources_data_dict = dict()
#     sources_pubs_dict = dict()
#     sources_stats_dict = dict()

#     source_selected = False
#     source_connected = False

#     show_selector = True
#     show_controls = True
#     show_data = True
#     show_results = True

#     available_processes = []
#     selected_process = 'None'
#     controls_dict = dict()
#     data_dict = dict()
#     results_dict = dict()


#     has_image_pub = False
#     image_pub_name = 'image'
#     max_image_pub_rate_hz = 10
#     image_pub_enabled = True
#     use_last_image = False
#     imaging_source_topics = []
#     imaging_pub_topics = []

#     status_has_published = False

#     #######################
#     ### IF Initialization
#     def __init__(self, 
#                 process_name = None,
#                 process_group = 'PROCESS',
#                 process_description = 'Process',
#                 process_module = None,
#                 process_status_msg = None,
#                 get_status_dict_callback = None,
#                 process_data_msg = None,
#                 process_data_products = [],
#                 process_results_msg = None,
#                 max_process_rate_hz = 10,
#                 source_status_msg_type = None,
#                 source_data_msg_type = None,       
#                 source_callback_dict = None,       
#                 auto_select_enabled = True,
#                 muti_source_enabled = True,
#                 exclude_source_filters = [],
#                 selected_sources = [],
#                 has_image_pub = False,
#                 image_pub_name = 'image',
#                 max_image_pub_rate_hz = 10,
#                 show_selector = True,
#                 show_controls = True,
#                 show_data = True,
#                 show_results = True,
#                 log_name = None,
#                 log_name_list = [],
#                 msg_if = None,
#                 node_if = None,
#                 save_data_if = None
#                 ):
#         ####  IF INIT SETUP ####
#         self.class_name = type(self).__name__
#         self.base_namespace = nepi_sdk.get_base_namespace()
#         self.node_name = nepi_sdk.get_node_name()
#         self.node_namespace = nepi_sdk.get_node_namespace()

#         ##############################  

        
#         # Create Msg Class
#         if msg_if is not None:
#             self.msg_if = msg_if
#         else:
#             self.msg_if = MsgIF()
#         self.log_name_list = copy.deepcopy(log_name_list)
#         self.log_name_list.append(self.class_name)
#         if log_name is not None:
#             log_name = nepi_utils.get_clean_name(log_name)
#             self.log_name_list.append(log_name)
#         self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

#         # Create Process Name
#         self.process_name = nepi_utils.get_clean_name(process_name)
#         if self.process_name is None or self.process_name == '':
#             self.msg_if.pub_warn("Process Name Not Valid: " + str(process_name)) 
#             return
#         self.msg_if.pub_info("Using Process Name: " + self.process_name)
#         self.process_namespace = nepi_sdk.create_namespace(self.node_namespace,self.process_name)
#         self.node_if_prefix = self.process_name + '_'

#         # Load Process Module
        
#         if process_module is None:
#             self.msg_if.pub_warn("Process Module Not Provided: " + str(process_name)) 
#             return
#         self.process_module = process_module
#         self.msg_if.pub_info("Using Process Module: " + self.process_module)


#         ##############################    
#         # Initialize Class Variables

#         self.process_group = str(process_group)
#         self.process_description = str(process_description)
#         # Check Process Status Msg Type


#         self.max_process_rate_hz = max_process_rate_hz

#         # Check Status Msg Type
#         if source_status_msg_type is None:
#             self.msg_if.pub_warn("Source Status Msg Not Provided") 
#             return
#         self.source_status_msg_type = source_status_msg_type

#         # Check Status Msg Type
#         if source_data_msg_type is None:
#             self.msg_if.pub_warn("Source Data Msg Not Provided") 
#             return
#         self.source_data_msg_type = source_data_msg_type

#         if source_callback_dict is not None:
#             for key in source_callback_dict.keys():
#                 self.source_callback_dict[key] = source_callback_dict[key]

#         self.process_updater_function = process_updater_function
      
#         self.auto_select_enabled = auto_select_enabled
#         self.muti_source_enabled = muti_source_enabled
#         self.exclude_source_filters = exclude_source_filters

#         clean_sources = []
#         for source_topic in selected_sources:
#             clean_sources = nepi_sdk.get_full_namespace(source_topic)
#         self.selected_sources = clean_sources

#         if has_image_pub == True:
#             self.has_image_pub = has_image_pub
#             self.max_image_pub_rate_hz = max_image_pub_rate_hz



#         self.show_selector = show_selector
#         self.show_controls = show_controls
#         self.show_data = show_data
#         self.show_results = show_results

                   
#         ##############################   
#         ## Node Setup

#         # Configs Config Dict ####################
#         CFGS_DICT = {
#                 'namespace': self.process_namespace
#         }

#         # Params Config Dict ####################
#         # Persist the selected topic under the connect namespace so the
#         # selection survives node restarts (via the config manager). Passing a
#         # params_dict is what enables config management on NodeClassIF.
#         PARAMS_DICT = {
#             self.node_if_prefix + 'selected_sources': {
#                 'namespace': self.process_namespace,
#                 'factory_val': self.selected_sources
#             }
#         }


#         # Publishers Config Dict ####################
#         self.process_node_pubs_dict = dict()


#         if process_status_msg is not None:
#             self.process_status_msg = process_status_msg
#             self.process_node_pubs_dict[self.node_if_prefix + 'status_pub'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'status',
#                 'msg': self.process_status_msg,
#                 'qsize': 1,
#                 'latch': True
#             }

#         # Check Process Status Msg Type
#         if process_data_msg is not None:
#             self.process_data_msg = process_data_msg
#             self.process_node_pubs_dict[self.node_if_prefix + 'data_pub'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'data',
#                 'msg': self.process_data_msg,
#                 'qsize': 1,
#                 'latch': True
#             }

#         # Check Process Status Msg Type
#         if process_results_msg is not None:
#             self.process_results_msg = process_results_msg
#             self.process_node_pubs_dict[self.node_if_prefix + 'results_pub'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'results',
#                 'msg': self.process_results_msg,
#                 'qsize': 1,
#                 'latch': True
#             }

        

#         # Subscribers Config Dict ####################
#         self.process_node_subs_dict = {
#             self.node_if_prefix + 'set_source': {
#                 'namespace': self.process_namespace,
#                 'topic': 'set_source',
#                 'msg': String,
#                 'qsize': None,
#                 'callback': self._setSourceCb, 
#                 'callback_args': ()
#             },
#             self.node_if_prefix + 'remove_source': {
#                 'namespace': self.process_namespace,
#                 'topic': 'clear_source',
#                 'msg': String,
#                 'qsize': None,
#                 'callback': self._clearSourceCb, 
#                 'callback_args': ()
#             },
#             self.node_if_prefix + 'system_status': {
#                 'msg': MgrSystemStatus,
#                 'namespace': self.base_namespace,
#                 'topic': 'status',
#                 'qsize': 5,
#                 'callback': self._systemStatusCb
#             },
#         }

#         if self.multi_source_enabled == True:
#             self.process_node_subs_dict[self.node_if_prefix + 'set_sources'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'set_sources',
#                 'msg': StringArray,
#                 'qsize': 10,
#                 'callback': self._setSourcesCb, 
#                 'callback_args': ()
#             }
#             self.process_node_subs_dict[self.node_if_prefix + 'add_sources'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'add_sources',
#                 'msg': String,
#                 'qsize': 10,
#                 'callback': self._addSourcesCb, 
#                 'callback_args': ()
#             }
#             self.process_node_subs_dict[self.node_if_prefix + 'remove_sources'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'add_sources',
#                 'msg': Empty,
#                 'qsize': 10,
#                 'callback': self._removeSourcesCb, 
#                 'callback_args': ()
#             }
#             self.process_node_subs_dict[self.node_if_prefix + 'clear_sources'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'clear_sources',
#                 'msg': Empty,
#                 'qsize': 10,
#                 'callback': self._clearSourcesCb, 
#                 'callback_args': ()
#             }

#         if process_module is not None:
#             self.process_node_subs_dict[self.node_if_prefix + 'reload_process'] = {
#                 'namespace': self.node_namespace,
#                 'topic': 'reload_process',
#                 'msg': Empty,
#                 'qsize': 10,
#                 'callback': self.reloadProcessCb, 
#                 'callback_args': ()
#             }

#             self.process_node_subs_dict[self.node_if_prefix + 'set_process'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'set_process',
#                 'msg': String,
#                 'qsize': 10,
#                 'callback': self._setProcessCb, 
#                 'callback_args': ()
#             }

#             self.process_node_subs_dict[self.node_if_prefix + 'set_process_max_rate'] = {
#                 'namespace': self.process_namespace,
#                 'topic': 'set_auto_update_rate',
#                 'msg': Float32,
#                 'qsize': 1,
#                 'callback': self._setProcessMaxRateCb,
#                 'callback_args': ()
#             }     
#             self.process_node_subs_dict[self.node_if_prefix + 'set_process_value'] = {
#                 'namespace': self.node_namespace,
#                 'topic': 'set_process_value',
#                 'msg': UpdateControl,
#                 'qsize': 1,
#                 'callback': self._setProcessControlCb,
#                 'callback_args': ()
#             }

#         if node_if is None:
#             self.node_if = NodeClassIF(
#                             configs_dict = CFGS_DICT,
#                             params_dict = PARAMS_DICT,
#                             services_dict = None,
#                             pubs_dict = self.process_node_pubs_dict,
#                             subs_dict = self.process_node_subs_dict,
#                             log_name_list = [],
#                             msg_if = self.msg_if
#             )
#             self.node_if.wait_for_ready()
#         else:
#             self.node_if_shared = True
#             try:
#                 self.node_if = node_if
#                 self.node_if.register_pubs(self.process_node_pubs_dict)
#                 self.node_if.register_subs(self.process_node_subs_dict)
#                 # Register the persisted selection param on the shared node_if too.
#                 self.node_if.add_param('selected_sources', self.process_namespace, self.selected_sources)
#                 nepi_sdk.sleep(1)
#             except Exception as e:
#                 self.msg_if.pub_info("Failed to register pubs and subs: " + str(e))
#                 return


#         # Restore any persisted selection. When no explicit topic was requested
#         # (selected_sources == "None"), use the value the config manager restored
#         # for this connect namespace. Otherwise honor the explicit request.
#         self.selected_sources_param = 'selected_sources'
#         if selected_sources == "None":
#             persisted = self.node_if.get_param(self.selected_sources_param)
#             if persisted is not None and persisted != '' and persisted != "None":
#                 selected_sources = persisted
#         self.selected_sources = selected_sources
#         self.msg_if.pub_info("Init Selected Topic: " + str(self.selected_sources))

#         ###############################
#         self.process_data_products = process_data_products
#         self.msg_if.pub_info("####################", log_name_list = self.log_name_list)
#         self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
#         if save_data_if is not None and save_data_if != 'None':
#             self.save_data_if = save_data_if
#             data_products = self.save_data_if.get_data_products()
#             for data_product in self.process_data_products:
#                 if data_product not in data_products:
#                     self.save_data_if.register_data_product(data_product)
#         elif save_data_if != 'None' and len(self.process_data_products) > 0:
#             # Setup Save Data IF Class 
#             self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
#             factory_data_rates= dict()
#             for data_product in self.process_data_products:
#                 factory_data_rates[data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

#             factory_filename_dict = {
#                 'prefix': "", 
#                 'add_timestamp': True, 
#                 'add_ms': True,
#                 'add_us': False,
#                 'suffix': "",
#                 'add_node_name': True
#                 }

#             sd_namespace = self.process_namespace
#             self.save_data_if = SaveDataIF(namespace = sd_namespace,
#                                     data_products = list(self.process_data_products),
#                                     factory_rate_dict = factory_data_rates,
#                                     factory_filename_dict = factory_filename_dict,
#                                     log_name_list = self.log_name_list,
#                                     msg_if = self.msg_if,
#                                     node_if = self.node_if)
#             nepi_sdk.sleep(1)

#         if self.save_data_if is not None:
#             self.save_data_topic = self.save_data_if.get_namespace()
#             self.msg_if.pub_warn("Using save_data namespace: " + str(self.save_data_topic), log_name_list = self.log_name_list)




#         ##############################
#         # Start updater process
#         if process_updater_enabled == True:
#             nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)
#         if process__msg is not None:
#             nepi_sdk.start_timer_process(1.0, self._publishStatusCb)
#         if process_status_msg is not None:
#             nepi_sdk.start_timer_process(1.0, self._publishStatusCb)
#                 process_file = None,
#                 process_controls_msg = None,
#                 process_data_msg = None,
#                 process_results_msg = None,
#                 process_status_msg = None,
#         ##############################
#         # Complete Initialization
#         self.process_ready = True
#         self.msg_if.pub_info(str(self.class_name) + " Initialization Complete")
#         ###############################
    

#     #######################
#     # Class Public Methods
#     #######################


#     def get_process_ready_state(self):
#         """Return the ready state of the interface.

#         Returns:
#             bool: True if the interface has completed initialization, False otherwise.
#         """
#         return self.process_ready

#     def wait_for_process_ready(self, timeout = float('inf') ):
#         """Block until the interface is ready or the timeout expires.

#         Args:
#             timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

#         Returns:
#             bool: True if the interface became ready, False if the timeout was reached.
#         """
#         success = False
#         if self.process_ready is not None:
#             self.msg_if.pub_info("Waiting for connection")
#             timer = 0
#             time_start = nepi_sdk.get_time()
#             while self.process_ready == False and timer < timeout and not nepi_sdk.is_shutdown():
#                 nepi_sdk.sleep(.1)
#                 timer = nepi_sdk.get_time() - time_start
#             if self.process_ready == False:
#                 self.msg_if.pub_info("Failed to Connect")
#             else:
#                 self.msg_if.pub_info("Connected")
#         return self.process_ready  

#     def get_namespace(self):
#         """Return the fully-resolved ROS namespace for the sources_connected PTX device.

#         Returns:
#             str: The fully-qualified namespace string used for topic and service resolution.
#         """
#         return self.process_namespace
    

#     def get_available_sources(self):
#         return self.available_sources
    

#     def get_available_names(self, available_sources = []):
#         available_names = []
#         for topic in available_sources:
#             name = topic
#             topic = topic[1:]
#             topic_split = topic.split('/')
#             if len(topic_split) > 2:
#                 name = topic_split[2]
#             available_names.append(name)
#         return available_names

    
#     def get_selected_sources(self):
#         return self.selected_sources
    
#     def set_selected_sources(self, selected_sources):
#         if selected_sources in self.available_sources or selected_sources == "None":
#             self.selected_sources = selected_sources
#         self.publish_status()
#         # Persist the selection so it survives a node restart. set_param writes
#         # the ROS param; save_config asks the config manager to save it to file.
#         if self.node_if is not None:
#             self.msg_if.pub_warn("selected_sources: " + str(selected_sources))
#             self.node_if.set_param('selected_sources', self.selected_sources)
#             self.node_if.save_config()
    

#     def check_connection(self):
#         """Check whether the device is currently sources_connected.

#         Returns:
#             bool: True if a status message has been received within the connection timeout window,
#                 False otherwise.
#         """
#         return self.sources_connected
    
#     def check_connections(self, source_topic):
#         """Check whether the device is currently sources_connected.

#         Returns:
#             bool: True if a status message has been received within the connection timeout window,
#                 False otherwise.
#         """
#         connected = False
#         if source_topic in self.sources_connected_topics:
#             connected = True
#         return self.sources_connected

#     def wait_for_connection(self, timeout = float('inf') ):
#         """Block until the device is sources_connected or the timeout expires.

#         Args:
#             timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

#         Returns:
#             bool: True if connection was established, False if the timeout was reached.
#         """
#         if self.node_if is not None:
#             self.msg_if.pub_info("Waiting for connection")
#             timer = 0
#             time_start = nepi_sdk.get_time()
#             while self.sources_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
#                 nepi_sdk.sleep(.1)
#                 timer = nepi_sdk.get_time() - time_start
#             if self.sources_connected == False:
#                 self.msg_if.pub_info("Failed to Connect")
#             else:
#                 self.msg_if.pub_info("Connected")
#         return self.sources_connected




#     ##########################
#     ### Process Functions
#     ##########################  


#     def setProcessCb(self, msg):
#         value = msg.data
#         self.setProcess(value)

#     def setProcess(self,value):
#             self.msg_if.pub_info("Setting process process topic to: " + str(value))
#             if value in self.process_processes_dict.keys():
#                 self.selected_process_process = value
#                 self.publish_status()
#                 if self.node_if is not None:
#                     self.node_if.set_param('selected_process_process', self.selected_process_process)
#                     #self.node_if.save_config()


#     def setProcessUpdateRateCb(self, msg):
#         rate = msg.data
#         self.setProcessUpdateRate(rate)

#     def setProcessUpdateRate(self, rate):
#             if rate < 0:
#                 rate = 1
#             rate = round(rate,1)
#             self.msg_if.pub_info("Setting process update rate to: " + str(rate))
#             self.process_processes_dict[self.selected_process_process]['process_update_rate'] = rate
#             self.publish_status()
#             if self.node_if is not None:
#                 self.node_if.set_param('process_processes_dict', self.process_processes_dict)
#                 #self.node_if.save_config()

#     def setProcessControlCb(self, msg):
#         self.msg_if.pub_info("Got Process Control update message " + str(msg))
#         control = msg.name
#         value = msg.value
#         self.setProcessControl(control,value)

#     def setProcessControl(self, control,value):
#             process_process = self.selected_process_process
#             process_controls_dict = self.process_processes_dict[process_process]['process_controls_dict']
#             if control in process_controls_dict.keys():
#                 self.msg_if.pub_info("Setting process control " + str(control) + " : " + str(value))
#                 process_controls_dict[control] = value
#                 self.process_processes_dict[process_process]['process_controls_dict'] = process_controls_dict
#                 self.publish_status()
#                 if self.node_if is not None:
#                     self.node_if.set_param('process_processes_dict', self.process_processes_dict)
#                     #self.node_if.save_config()



#     def reloadProcesssCb(self,msg):
#         self.process_process_ready = False
#         nepi_sdk.sleep(1)
#         try:
#             importlib.reload(nepi_process_pt)
#             self.process_processes_dict = nepi_process_pt.update_processes_dict(self.process_processes_dict)
#             process_processes = list(self.process_processes_dict.keys())
#             if self.selected_process_process not in process_processes:
#                 self.selected_process_process = process_processes[0]
#             self.msg_if.pub_info("Processs reloaded")
#             self.process_process_ready = True
#         except Exception as e:
#             self.msg_if.pub_info("Failed to reload process module: " + str(e)) 






#     def unregister(self):
#         success = False
#         self.unsubscribe_topic()
#         if self.node_if is not None:
#             if self.node_if_shared == False:
#                 self.node_if.unregister_class()
#                 nepi_sdk.sleep(1)
#             else:
#                 self.unsubscribe_topic()

#                 if self.node_if is not None:
#                     if self.process_node_subs_dict is not None:
#                         for sub_name in self.process_node_subs_dict.keys():
#                             self.node_if.unregister_sub(sub_name)
#                 self.process_node_subs_dict = None

#                 if self.node_if is not None:
#                     if self.process_node_pubs_dict is not None:
#                         for pub_name in self.process_node_pubs_dict.keys():
#                             self.node_if.unregister_pub(pub_name)
#                 self.process_node_pubs_dict = None
                
#         time.sleep(1)
#         try:
#             self.node_if = None
#             self.selected_sources = 'None'
#             self.connecting = False 
#             self.sources_connected = False 
#             self.sources_connected_topics = 'None'
#             success = True
#         except Exception as e:
#             self.msg_if.pub_warn("Failed to unregister:  " + str(e))
#         return success



#     def get_process_status_msg(self):

#         available_sources = copy.deepcopy(self.available_sources)
#         selected_sources = copy.deepcopy(self.selected_sources)
#         status_msg = ProcessStatus()

#         status_msg.name = self.process_name
#         status_msg.id = self.process_id

#         status_msg.status_msg_type = self.process_status_msg

#         status_msg.available_sources = available_sources
#         available_names = self.get_available_names(available_sources)
#         status_msg.available_names = available_names

#         selected_name = 'None'
#         if selected_sources not in available_sources:
#             if len(available_sources) > 0 and self.auto_select_enabled == True and self.auto_select_active == True:
#                 selected_sources = [available_sources[0]]
#                 self.selected_sources = selected_sources
#             else:
#                 selected_sources = 'None' 

#         if selected_sources in available_sources:
#             selected_ind = available_sources.index(selected_sources)
#             selected_name = available_names[selected_ind]

#         status_msg.selected_sources = selected_sources
#         status_msg.selected_name = selected_name

#         status_msg.connecting = self.connecting
#         status_msg.sources_connected = self.sources_connected
#         sources_connected_topics = self.sources_connected_topics
#         if sources_connected_topics is None:
#             sources_connected_topics = 'None'
#         status_msg.sources_connected_topics = sources_connected_topics

#         connect_msg = "Not Selected"
#         if self.selected_sources != "None":
#             connect_msg = "Selected"
#             if self.connecting == True:
#                 connect_msg = "Connecting"
#             if self.sources_connected == True:
#                 connect_msg = "Connected"
#         status_msg.connect_msg = connect_msg


#         status_msg.show_selector = self.show_selector
#         status_msg.show_controls = self.show_controls
#         status_msg.show_data = self.show_data
#         status_msg.show_results = self.show_results


#         return status_msg

#     def publish_status(self, status_msg):
#         ###########
#         if self.node_if is not None:
#             if self.status_has_published == False:
#                 self.msg_if.pub_warn("Publishing Status: " + str(status_msg))
#                 self.status_has_published = True
#             self.node_if.publish_pub('status_pub', status_msg) 
#         return status_msg


#     #######################
#     # Class Private Methods
#     #######################

#     # ROS callback for the system status msg. Populates the active topic/type
#     # lists that discovery searches. NOTE: this MUST NOT share a name with the
#     # discovery timer below -- a duplicate name silently shadows this method, so
#     # active_topics never gets populated and discovery finds nothing.
#     def _systemStatusCb(self,msg):
#             self.active_nodes = msg.active_nodes
#             self.active_topics = msg.active_topics
#             self.active_topic_types = msg.active_topic_types
#             self.active_services = msg.active_services


#     # Discovery/connection timer. Finds available topics of the connect status
#     # msg type among the active topics, auto-selects, and subscribes.
#     def _updaterCb(self,timer):
#         needs_publish = False
#         ##############

#         selected_sources = copy.deepcopy(self.selected_sources)
#         last_available = copy.deepcopy(self.available_sources)

#         topics = nepi_sdk.find_topics_by_msg(self.connect_status_msg, topics_list = self.active_topics, types_list = self.active_topic_types)
#         available_sources = []
#         for topic in topics:
#             valid = True
#             for filter in self.exclude_source_filters:
#                 if filter in topic:
#                     valid = False
#             if valid == True:
#                 available_sources.append(topic.replace('/status',''))
#         if available_sources != last_available:
#             self.available_sources = available_sources
#             needs_publish = True

#         ####################
#         if self.sources_connected_topics is not None:
#             if self.sources_connected_topics not in self.available_sources:
#                 success = self.unsubscribe_topic()
#         if selected_sources == 'None' and len(self.available_sources) > 0:
#             self.selected_sources = self.available_sources[0]
#         needs_publish = True

#         was_sources_connected = copy.deepcopy(self.sources_connected)
#         if self.selected_sources in self.available_sources and self.sources_connected_topics != selected_sources:
#             success = self.subscribe_source(self.selected_sources)
#         elif self.selected_sources not in self.available_sources:
#             self.sources_connected = False
#         # else: already subscribed to the selected topic -- leave self.sources_connected
#         # to the status callback (sets True on each msg) and the staleness check
#         # below, so it does not get clobbered False every cycle.

#         ##################
#         cur_time = nepi_utils.get_time()
#         last_time = copy.deepcopy(self.last_status_time )
#         for i, source_topic in enumerate(self.sources_connected_topics):
#             connected = self.sources_connected[i]
#             if connected == True:
#                 if (cur_time - last_time) > CONNECTED_TIMEOUT:
#                     self.sources_connecting[i] = False 
#                     self.sources_connected[i] = False 
#                     self.sources_status_dict[i] = None
#                     self.sources_status_msg[i] = None


#         ##################
#         # Call Provided Updater Function
#         if self.process_updater_function is not None:
#             self.process_updater_function()

#         ##################
#         # Get settings from param server
#         # if needs_publish == True:
#         #   self.publish_status()
#         nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)


#     ##########################
#     ### Process
#     ##########################  

#     def setProcessUpdateRateCb(self, msg):
#         rate = msg.data
#         self.setProcessUpdateRate(rate)

#     def setProcessUpdateRate(self, rate):
#             if rate < 0:
#                 rate = 1
#             rate = round(rate,1)
#             self.msg_if.pub_info("Setting process update rate to: " + str(rate))
#             self.process_processes_dict[self.selected_process_process]['process_update_rate'] = rate
#             self.publish_status()
#             if self.node_if is not None:
#                 self.node_if.set_param('process_processes_dict', self.process_processes_dict)
#                 #self.node_if.save_config()

#     def setProcessControlCb(self, msg):
#         self.msg_if.pub_info("Got Process Control update message " + str(msg))
#         control = msg.name
#         value = msg.value
#         self.setProcessControl(control,value)

#     def setProcessControl(self, control,value):
#             process_process = self.selected_process_process
#             process_controls_dict = self.process_processes_dict[process_process]['process_controls_dict']
#             if control in process_controls_dict.keys():
#                 self.msg_if.pub_info("Setting process control " + str(control) + " : " + str(value))
#                 process_controls_dict[control] = value
#                 self.process_processes_dict[process_process]['process_controls_dict'] = process_controls_dict
#                 self.publish_status()
#                 if self.node_if is not None:
#                     self.node_if.set_param('process_processes_dict', self.process_processes_dict)
#                     #self.node_if.save_config()



#     def reloadProcesssCb(self,msg):
#         self.process_process_ready = False
#         nepi_sdk.sleep(1)
#         try:
#             importlib.reload(nepi_process_pt)
#             self.process_processes_dict = nepi_process_pt.update_processes_dict(self.process_processes_dict)
#             process_processes = list(self.process_processes_dict.keys())
#             if self.selected_process_process not in process_processes:
#                 self.selected_process_process = process_processes[0]
#             self.msg_if.pub_info("Processs reloaded")
#             self.process_process_ready = True
#         except Exception as e:
#             self.msg_if.pub_info("Failed to reload process module: " + str(e)) 



#     def setProcessCb(self, msg):
#         value = msg.data
#         self.setProcess(value)

#     def setProcess(self,value):
#             self.msg_if.pub_info("Setting process process topic to: " + str(value))
#             if value in self.process_processes_dict.keys():
#                 self.selected_process_process = value
#                 self.publish_status()
#                 if self.node_if is not None:
#                     self.node_if.set_param('selected_process_process', self.selected_process_process)
#                     #self.node_if.save_config()



#     def _processLoop(self):
       

