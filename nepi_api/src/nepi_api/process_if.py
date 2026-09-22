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
import importlib


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data
from nepi_sdk import nepi_process
from nepi_sdk import nepi_system

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image

from nepi_interfaces.msg import StringArray


from nepi_interfaces.msg import Control, ControlsStatus, SettingsStatus, UpdateControl, MgrSystemStatus
from nepi_interfaces.msg import Datum, DataStatus

from nepi_interfaces.msg import ProcessStatus


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.data_if import ColorImageIF





#########################################
# Process IF Class
#########################################



BLANK_CALLBACK_DICT = dict(
        process_update_callback = None,
        selection_updated_callback = None,
        controls_updated_callback = None,
    )


BLANK_CONFIG_DICT = dict(
        has_sources = False,
        multi_source_enabled = False,
        auto_select_enabled = True,

        has_enable = True,
        enable_requires_admin = False,

        has_process_pub = True,
        has_process_enable = True,
        has_process_reload = False,
        has_process_rate = False,
        min_max_process_rates = [1,20],
        default_process_rate = 10,

        has_results_pub = True,

        has_save_data = True,

        has_config = True,
        config_requires_admin = False,

        has_image_pub = True,
        has_image_rate = False,
        min_max_image_rates = [1,20],
        default_image_rate = 10,
        has_use_last_image = False,
        use_last_image = False,
    )



BLANK_SHOW_DICT = dict(        
        show_sources = True,
        show_sources_restricted = True,
        show_enable = True,
        show_enable_restricted = True,
        show_rates = True,
        show_rates_restricted = True,
        show_process = True,
        show_process_restricted = True,
        show_reload = True,
        show_reload_restricted = True,
        show_data = True,
        show_data_restricted = True,
        show_controls = True,
        show_controls_restricted = True,
        show_results = True,
        show_results_restricted = True,
        show_stats = True,
        show_stats_restricted = True,
        show_save_data = True,
        show_save_data_restricted = True,
        show_config = True,
        show_config_restricted = True,
    )



CONNECTED_TIMEOUT = 2
class ProcessIF:
    
    msg_if = None
    node_if = None
    config_topic = ''
    node_if_shared = False
    ready = False

    admin_enabled = False

    save_data_if = None
    data_products = None

    status_msg = ProcessStatus()
    save_data_topic = ''


    active_nodes = []
    active_topics = []
    active_topic_types =  []
    active_services =  []  

    process_image_name = None
    namespace = ''

    data_dict = dict()
    
    has_controls = False
    controls_msg = ControlsStatus()
    controls_dict = dict()

    states_dict = dict()

    has_results = False    
    results_dict = None

    results_display_dict = None
    results_display_msg = DataStatus()

    has_results_pub = True
    results_pub_msg = None
    results_pub_topic = ''

    
    process_node_pubs_dict = None
    process_node_subs_dict = None

    # Resolved process namespace. Every pub, sub and param this IF registers
    # hangs off it, and it is what ProcessStatus.namespace reports -- the RUI's
    # Nepi_IF_ConnectProcess matches incoming status on this field, so it has to
    # be the same string the RUI subscribed with.
    namespace = ''

    # Run state. enabled is the operator's request, running is what the owning
    # node reports back after acting on it. They are deliberately separate: an
    # enabled process whose sources drop out is enabled and not running.

    enabled = True
    running = False
    state = False
    msg_str = ''

    connected_source_topics = []

    min_max_process_rates = [0.1,100]
    set_process_rate = 10.0



    process_module = None
    has_process_reload = True
    processes_dict = dict()
    processes_controls_dict = dict()
    processes_functions_dict = dict()
    available_processes = []
    selected_process = 'None'
    process_function = None
    process_ready = False
    process_busy = False
    process_times = [1.0] * 10
    last_process_time = None

    
    image_pub_name = ''
    min_max_image_rates = [1,20]
    set_image_rate = 10.0
    image_pub_topics = []
    image_pub_enabled = True
    

    callback_dict = copy.deepcopy(BLANK_CALLBACK_DICT)
    config_dict = copy.deepcopy(BLANK_CONFIG_DICT)
    show_dict = copy.deepcopy(BLANK_SHOW_DICT)

    status_has_published = False

    #######################
    ### IF Initialization
    def __init__(self, 
                process_name = 'process',
                process_group = 'PROCESS',
                process_description = 'Process',
                process_module = None,              
                callback_dict = None,
                config_dict = None,
                show_dict = None,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None,
                save_data_if = None,
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
        if log_name is not None:
            log_name = nepi_utils.get_clean_name(log_name)
            self.log_name_list.append(log_name)
        self.log_name_list.append(self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        # Create Process Name
        self.process_name = nepi_utils.get_clean_name(process_name)
        if self.process_name is None or self.process_name == '':
            self.msg_if.pub_warn("Process Name Not Valid: " + str(process_name)) 
            return
        self.msg_if.pub_info("Using Process Name: " + self.process_name)
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,self.process_name)
        self.data_products = [self.process_name]




        if callback_dict is not None:
            try:
                for key in callback_dict.keys():
                    if key in self.show_dict.keys():
                        self.callback_dict[key] = callback_dict[key]
            except:
                pass

        if config_dict is not None:
            try:
                for key in config_dict.keys():
                    if key in self.config_dict.keys():
                        self.config_dict[key] = config_dict[key]
            except:
                pass

        if show_dict is not None:
            try:
                for key in show_dict.keys():
                    if key in self.show_dict.keys():
                        self.show_dict[key] = show_dict[key]
            except:
                pass



        if process_module is None:
            self.msg_if.pub_warn("No Process Module Provided")
            return

        self.process_module = process_module

        self.has_results_pub = self.config_dict['has_results_pub']

        has_image_pub = self.config_dict['has_image_pub']
        try:
            image_pub_topic = process_module.IMAGE_PUB_TOPIC
        except:
            image_pub_topic = process_name.replace('_image','') + '_image'
        image_pub_topic = nepi_utils.get_clean_name(image_pub_topic)
        if image_pub_topic is not None and image_pub_topic != '':
            self.image_pub_name = image_pub_topic
            if image_pub_topic not in self.data_products and has_image_pub == True:
                self.data_products.append(self.image_pub_name)


        # Registry keys on a shared node_if must be domain-unique, so every key
        # this IF adds carries the process name. Param wire names ARE
        # namespace + key, so the prefix is part of the external param surface.
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

       
        ##############################    
        # Initialize Class Variables

        self.process_group = str(process_group)
        self.process_description = str(process_description)
        # Check Process Status Msg Type



        success = self._reloadProcesses()
        if success == False:
            self.msg_if.pub_warn("INITIAL PROCESS LOAD FAILED: " + str(self.processes_functions_dict))
        else:
            self.msg_if.pub_warn("INITIAL PROCESS LOAD SUCCEEDED: " + str(self.processes_functions_dict))





        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        # Configs Config Dict ####################
        CFGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        # Persist the selected topic under the connect namespace so the
        # selection survives node restarts (via the config manager). Passing a
        # params_dict is what enables config management on NodeClassIF.
        self.processes_param_name = self.node_if_prefix + 'processes_dict'
        PARAMS_DICT = {
            self.processes_param_name: {
                'name': 'processes_dict',
                'namespace': self.namespace,
                'factory_val': self.processes_controls_dict
            },
            self.node_if_prefix + 'selected_process': {
                'name': 'selected_process',
                'namespace': self.namespace,
                'factory_val': self.selected_process
            },
            self.node_if_prefix + 'enabled': {
                'name': 'enabled',
                'namespace': self.namespace,
                'factory_val': self.enabled
            },
            self.node_if_prefix + 'set_process_rate': {
                'name': 'set_process_rate',
                'namespace': self.namespace,
                'factory_val': self.set_process_rate
            },
            self.node_if_prefix +  'set_image_rate': {
                'name': 'set_image_rate',
                'namespace': self.namespace,
                'factory_val': self.set_image_rate
            },
        }


        # Publishers Config Dict ####################
        self.process_node_pubs_dict = dict()


        # The status publisher is unconditional. Nepi_IF_ConnectProcess renders
        # nothing at all until a ProcessStatus arrives, so a process with no
        # custom status message still has to publish the generic one.

        self.process_node_pubs_dict[self.node_if_prefix + 'status_pub'] = {
            'namespace': self.namespace,
            'topic': 'status',
            'msg': ProcessStatus,
            'qsize': 1,
            'latch': True
        }

        
        if self.has_results_pub == True:
            self.process_node_pubs_dict[self.node_if_prefix + 'results_pub'] = {
                'namespace': self.namespace.replace('/' + process_name,''),
                'topic': process_name,
                'msg': self.results_pub_msg,
                'qsize': 1,
                'latch': True
            }
            self.results_pub_topic = self.namespace + '/' + process_name
            self.has_results_pub = True


        # Subscribers Config Dict ####################
      
        self.process_node_subs_dict = {
            self.node_if_prefix + 'reload_process': {
                'namespace': self.namespace,
                'topic': 'reload_process',
                'msg': Empty,
                'qsize': 10,
                'callback': self._reloadProcessesCb
            },
            self.node_if_prefix + 'set_process': {
                'namespace': self.namespace,
                'topic': 'set_process',
                'msg': String,
                'qsize': 10,
                'callback': self._setProcessCb
            },
            self.node_if_prefix + 'set_enable': {
                'namespace': self.namespace,
                'topic': 'set_enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self._setEnableCb
            },
            self.node_if_prefix + 'update_control': {
                'msg': UpdateControl,
                'namespace': self.namespace,
                'topic': 'update_control',
                'qsize': 5,
                'callback': self._updateControlCb
            },
            self.node_if_prefix + 'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self._systemStatusCb
            },
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
            self.config_if = self.namespace
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

        ####################
        # Config
        has_config = self.config_dict['has_config']
        if has_config == True:
            self.status_msg.has_config = has_config 
            self.status_msg.config_topic = self.node_if.get_namespace()
            self.status_msg.show_config = self.show_dict['show_config'] == True

        ####################
        # Save Data
        has_save_data = self.config_dict['has_save_data']
        if has_save_data == False or len(self.data_products) == 0:
            self.save_data_topic = ''
            self.data_products = []
        else:
            if self.save_data_if is not None:
                self.msg_if.pub_info("####################", log_name_list = self.log_name_list)
                self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
                if save_data_if is not None and save_data_if != 'None':
                    self.save_data_if = save_data_if
                    data_products = self.save_data_if.get_data_products()
                    for data_product in self.data_products:
                        if data_product not in data_products:
                            self.save_data_if.register_data_product(data_product)
                elif save_data_if != 'None':
                    
                    # Setup Save Data IF Class 
                    self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
                    factory_data_rates= dict()

                    factory_filename_dict = {
                        'prefix': "", 
                        'add_timestamp': True, 
                        'add_ms': True,
                        'add_us': False,
                        'suffix': "",
                        'add_node_name': True
                        }

                    sd_namespace = self.node_namespace
                    self.save_data_if = SaveDataIF(namespace = sd_namespace,
                                            data_products = [self.data_products],
                                            factory_rate_dict = factory_data_rates,
                                            factory_filename_dict = factory_filename_dict,
                                            log_name_list = self.log_name_list,
                                            msg_if = self.msg_if,
                                            node_if = self.node_if)
                    nepi_sdk.sleep(1)

                if self.save_data_if is not None:
                    self.save_data_topic = self.save_data_if.get_namespace()
                    self.msg_if.pub_info("Using save_data namespace: " + str(self.save_data_topic), log_name_list = self.log_name_list)
                else:
                    has_save_data = False

            self.status_msg.has_save_data = has_save_data 
            self.status_msg.save_data_topic = self.save_data_topic
            self.status_msg.data_products = self.data_products
            self.status_msg.show_save_data = has_save_data == True and self.show_dict['show_save_data'] == True



        self.init(do_updates = True)

        ##############################
        # Complete Initialization
        self.ready = True
        # Without this the status topic is advertised and never written, and the
        # RUI process panel stays blank forever.
        nepi_sdk.start_timer_process(1, self._publishStatusCb)
        self.publish_status()
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete")
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_ready(self):
        """Return the ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready  

    def get_namespace(self):
        """Return the fully-resolved ROS namespace for the sources_connected PTX device.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.namespace
    


    def get_available_processes(self):
        return self.available_processes
    
    
    def get_selected_process(self):
        return self.selected_process
    
    def set_selected_process(self, process_name, check_updates = True):
        success = False
        if process_name in self.available_processes:
            cur_process = copy.deepcopy(self.selected_process)
            if process_name != cur_process or check_updates == False:
                self.msg_if.pub_warn("Process Selected: " + str(process_name))
                self.process_ready = False
                self.selected_process = process_name
                self.publish_status()
                nepi_sdk.sleep(1)
                processes_dict = copy.deepcopy(self.processes_dict)
                [self.data_dict,self.controls_dict,self.results_display_dict,self.states_dict] = nepi_process.get_process_dicts(processes_dict,process_name)
                self.process_function = self.processes_functions_dict[process_name]
                nepi_sdk.sleep(1)
                success = True
                self.msg_if.pub_warn("Process Ready: " + str(process_name))
                self.msg_if.pub_warn("Process Dictionaries: " + str([self.data_dict.keys(),self.controls_dict.keys(),self.results_display_dict.keys(),self.states_dict.keys(),self.process_function]))

        self.process_ready = self.selected_process in self.available_processes
        self.enabled = True
        return success

    def set_enable_process(self, enabled):
        self.enabled = enabled

    def get_process_ready(self):
        """Return the ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        process_ready = self.ready and self.process_ready and self.enabled
        return process_ready

    def wait_for_process_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        #self.msg_if.pub_info("Waiting for process ready")
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.get_process_ready() == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        return self.get_process_ready()  


    def set_process_busy(self, is_busy = False):
        self.process_busy = is_busy

    def get_process_busy(self):
        return self.process_busy


    def wait_on_process_busy(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False

        #self.msg_if.pub_info("Waiting for process not busy")
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.get_process_busy() == True and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        return self.get_process_busy()
    


    
    def set_connected_source_topics(self, connected_source_topics):
        if self.connected_source_topics != connected_source_topics:
            self.connected_source_topics = connected_source_topics
            self.publish_status()
        
    def set_image_pub_topics(self, image_pub_topics):
        if self.image_pub_topics != image_pub_topics:
            self.image_pub_topics = image_pub_topics
            self.publish_status()


    ##################
    # Data Dict Functions

    def get_data(self):
        """Return a copy of the full data dict, keyed by datum name.

        Returns:
            dict: A deep copy of the data dict.
        """
        data_dict = copy.deepcopy(self.data_dict)
        return data_dict

    def get_datum(self, datum_name):
        """Return the current value of one datum, read from its type-correct field.

        Args:
            datum_name (str): The datum key name.

        Returns:
            The datum value, or None if the datum is not registered.
        """
        value = None
        data_dict = copy.deepcopy(self.data_dict)
        if self.data_dict is not None:
            if datum_name in data_dict.keys():
                value = data_dict[datum_name]
        return value

    def set_data_value(self, datum_name, update_value):
        """Write one datum value, stamp its timestamp, and publish status.

        The node that owns this interface is the only writer of record; the RUI
        has no publish path to this method.

        Args:
            datum_name (str): The datum key name.
            update_value: The new value. Coerced to the datum's declared type.
            timestamp (float, optional): Write time. Defaults to now.
            publish (bool, optional): Publish status after update
        """
        if self.data_dict is not None:
            self.data_dict[datum_name] = update_value
            
    def set_data_values(self, data_dict):
        """Write multiple datum values, stamp its timestamp, and publish status.

        The node that owns this interface is the only writer of record; the RUI
        has no publish path to this method.

        Args:
            data_dict (dict): dictionary of datums to update
            timestamp (float, optional): Write time. Defaults to now.
            publish (bool, optional): Publish status after update
        """
        if data_dict is not None:
            if self.data_dict is not None:
                for datum_name in data_dict.keys():
                    update_value = data_dict[datum_name]
                    self.data_dict[datum_name] = update_value
 


    ##################
    # Controls Dict Functions



    def get_control_value(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        value = None
        if controls_dict is not None:
            value = nepi_controls.get_value(controls_dict, control_name)
        return value

    def get_controls_values(self):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_values_dict = None
        if controls_dict is not None:
            controls_values_dict = get_controls_values_dict = nepi_controls.get_values_dict(controls_dict)
        return controls_values_dict

    def set_control_value(self, control_name, update_value, index = None):
        if self.get_process_ready() == True:
            process_name = copy.deepcopy(self.selected_process)
            controls_dict = copy.deepcopy(self.controls_dict)
            if controls_dict is not None:
                if control_name in controls_dict.keys():
                    controls_dict = nepi_controls.set_value(controls_dict, control_name, update_value, index = index)
                    if controls_dict != self.controls_dict:
                        self.controls_dict = controls_dict
                        self.publish_status()
                        if process_name in self.processes_dict.keys():
                            self.processes_dict[process_name]['controls_dict'] = self.controls_dict
                            processes_controls_dict = copy.deepcopy(self.processes_controls_dict)
                            processes_controls_dict[process_name] = nepi_controls.get_values_dict(self.processes_dict[process_name]['controls_dict'])
                            if self.node_if is not None and processes_controls_dict != self.processes_controls_dict:
                                self.processes_controls_dict = processes_controls_dict
                                self.node_if.set_param(self.processes_param_name, self.processes_controls_dict)
                        try:
                            self.msg_if.pub_warn("Updated Control Value: " + str([ control_name, update_value, self.controls_dict[control_name] ]), throttle_s = 5)
                        except Exception as e:
                            self.msg_if.pub_info("Failed pub Updated Control Value msg: " + str(e), throttle_s = 5)
                else:
                    self.msg_if.pub_info("Failed pub Updated Control Options msg. Control Name not In Controls.keys: " + str([control_name,controls_dict.keys()]), throttle_s = 5)

    def set_control_options(self, control_name, update_options):
        if self.get_process_ready() == True:
            process_name = copy.deepcopy(self.selected_process)
            controls_dict = copy.deepcopy(self.controls_dict)
            if controls_dict is not None:
                if control_name in controls_dict.keys():
                    controls_dict = nepi_controls.set_options(controls_dict, control_name, update_options)
                    if controls_dict != self.controls_dict:
                        self.controls_dict = controls_dict
                        self.publish_status()

                        if process_name in self.processes_dict.keys():
                            self.processes_dict[process_name]['controls_dict'] = self.controls_dict
                            processes_controls_dict = copy.deepcopy(self.processes_controls_dict)
                            processes_controls_dict[process_name] = nepi_controls.get_values_dict(self.processes_dict[process_name]['controls_dict'])
                            if self.node_if is not None and processes_controls_dict != self.processes_controls_dict:
                                self.processes_controls_dict = processes_controls_dict
                                self.node_if.set_param(self.processes_param_name, self.processes_controls_dict)
                        try:
                            self.msg_if.pub_warn("Updated Control Options: " + str([ control_name, update_options, self.controls_dict[control_name] ]), throttle_s = 5)
                        except Exception as e:
                            self.msg_if.pub_info("Failed pub Updated Control Options msg: " + str(e), throttle_s = 5)
                else:
                    self.msg_if.pub_info("Failed pub Updated Control Options msg. Control Name not In Controls.keys: " + str([control_name,controls_dict.keys()]), throttle_s = 5)

    def get_control_options(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        options = None
        if controls_dict is not None:
            options = nepi_controls.get_options(controls_dict, control_name)
        return options

    def set_control_bounds(self, control_name, min_bound = None, max_bound = None):
        if self.get_process_ready() == True:
            process_name = copy.deepcopy(self.selected_process)
            controls_dict = copy.deepcopy(self.controls_dict)
            if controls_dict is not None:
                if control_name in controls_dict.keys():
                    controls_dict = nepi_controls.set_bounds(controls_dict, control_name, min_bound = min_bound, max_bound = max_bound)
                    if controls_dict != self.controls_dict:
                        self.controls_dict = controls_dict
                        self.self.publish_status()

                        if process_name in self.processes_dict.keys():
                            self.processes_dict[process_name]['controls_dict'] = self.controls_dict
                            processes_controls_dict = copy.deepcopy(self.processes_controls_dict)
                            processes_controls_dict[process_name] = nepi_controls.get_values_dict(self.processes_dict[process_name]['controls_dict'])
                            if self.node_if is not None and processes_controls_dict != self.processes_controls_dict:
                                self.processes_controls_dict = processes_controls_dict
                                self.node_if.set_param(self.processes_param_name, self.processes_controls_dict)
                        try:
                            self.msg_if.pub_warn("Updated Control Bounds: " + str([ control_name, update_bounds, self.controls_dict[control_name] ]), throttle_s = 5)
                        except Exception as e:
                            self.msg_if.pub_info("Failed pub Updated Control Bounds msg: " + str(e), throttle_s = 5)
                else:
                    self.msg_if.pub_info("Failed pub Updated Control Options msg. Control Name not In Controls.keys: " + str([control_name,controls_dict.keys()]), throttle_s = 5)


    def get_control_bounds(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        bounds = None
        if controls_dict is not None:
            bounds = nepi_controls.get_bounds(controls_dict, control_name)
        return bounds
    
    ##################
    # Process Results Functions


    # def get_results(self):
    #     values_dict = None
    #     results_dict = copy.deepcopy(self.results_display_dict)
    #     if results_dict is not None:
    #         values_dict = nepi_data.get_values_dict(results_dict)
    #     return values_dict


    def process_results(self, source_topic = ''):
        if source_topic != '' and source_topic not in self.connected_source_topics:
            self.connected_source_topics.append(source_topic)
        results_pub_msg = None
        #self.msg_if.pub_warn("Processing results: " + str( [self.data_dict, self.controls_dict, self.results_display_dict, self.process_function]), throttle_s = 5)
        last_results_dict = copy.deepcopy(self.results_dict)
        results_dict = None
        if self.enabled == True:
            process_ready = self.wait_for_process_ready()
            if process_ready == True:
                try:
                    [self.data_dict, self.controls_dict, self.states_dict, results_dict] = self.process_function(self.data_dict, self.controls_dict, self.states_dict, self.results_dict)
                    self.results_dict = results_dict
                    if self.results_dict is not None:
                        [self.data_dict, self.controls_dict, self.states_dict, results_dict] = self.process_function(self.data_dict, self.controls_dict, self.states_dict, self.results_dict)
                        
                    else:
                        self.results_display_dict = nepi_data.reset_values(self.results_display_dict)
                    #self.msg_if.pub_warn("Processed results: " + str( [self.results_display_dict, results_pub_msg]), throttle_s = 5)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to process results: " + str(e), throttle_s = 5) 

                self.results_dict = results_dict
                if self.results_dict is not None:
                    self.results_display_dict = nepi_data.set_values(self.results_display_dict,self.results_dict)
                    #self.msg_if.pub_warn("Updated results display results: " + str( [self.results_display_dict, results_dict]), throttle_s = 5)
                else:
                    self.results_display_dict = nepi_data.reset_values(self.results_display_dict)
                #self.msg_if.pub_warn("Processed results: " + str( [self.results_display_dict, results_pub_msg]), throttle_s = 5)

                if self.has_results_pub == True and self.results_dict is not None:
                    self._publishResults(self.results_dict, source_topic)
            else:
                self.msg_if.pub_warn("Processes Not Ready", throttle_s = 10)
            
            
        else:
            #self.msg_if.pub_warn("Process Not Ready. Can't Pub Results", throttle_s = 5)
            pass
        if last_results_dict != self.results_dict:
            self.publish_status()
        return self.results_dict


    def get_states(self):
        values_dict = None
        states_dict = copy.deepcopy(self.states_dict)
        if states_dict is not None:
            values_dict = nepi_data.get_values_dict(states_dict)
        return values_dict

    ##################
    # Misc Functions


    def publish_status(self):

        status_msg = ProcessStatus()

        status_msg.name = self.process_name
        status_msg.group = self.process_group
        status_msg.description = self.process_description

        status_msg.node_name = self.node_name
        status_msg.namespace = self.namespace

        status_msg.save_data_topic = self.save_data_topic
        status_msg.config_topic = self.config_topic

        # Run state. enabled is what the operator asked for and running is what
        # the owning node reports back; the RUI shows both so an enable that
        # could not take effect is visible rather than silently cosmetic.
        status_msg.enabled = self.enabled
        status_msg.running = self.running
        status_msg.state = self.state
        status_msg.msg_str = self.msg_str

        status_msg.connected_source_topics = self.connected_source_topics

        status_msg.min_max_process_rates = self.min_max_process_rates
        status_msg.set_process_rate = self.set_process_rate

        status_msg.has_process_reload = True
        status_msg.available_processes = self.available_processes
        status_msg.selected_process = self.selected_process
        status_msg.process_ready = self.get_process_ready()

        controls_dict = copy.deepcopy(self.controls_dict)
        if controls_dict is not None:
            has_controls = len(list(controls_dict.keys())) > 0
            status_msg.has_controls = has_controls
            if has_controls == True:
                self.controls_msg = nepi_controls.update_status_msg(self.controls_msg, controls_dict)
                status_msg.controls = self.controls_msg

        results_display_dict = copy.deepcopy(self.results_display_dict)
        if results_display_dict is not None:
            has_results = len(list(results_display_dict.keys())) > 0
            status_msg.has_results = has_results
            if has_results == True:
                self.results_display_msg = nepi_data.update_status_msg(self.results_display_msg, results_display_dict)
                status_msg.results = self.results_display_msg

        status_msg.has_results_pub = self.has_results_pub
        if self.has_results_pub == True:
            status_msg.results_pub_topic = self.results_pub_topic


        status_msg.image_pub_name = self.image_pub_name
        status_msg.min_max_image_rates = self.min_max_image_rates
        status_msg.set_image_rate = self.set_image_rate
        status_msg.image_pub_topics = self.image_pub_topics


        for key in self.show_dict.keys():
            if '_restricted' not in key:
                try:
                    restricted = self.show_dict[key + '_restricted'] and self.admin_enabled == True
                    show = self.show_dict[key] and restricted == False
                    setattr(status_msg, key, show)
                except:
                    pass


        ###########
        if self.node_if is not None:
            if self.status_has_published == False:
                self.msg_if.pub_info("Publishing first status for process: " + str(self.process_name))
                self.status_has_published = True
            self.node_if.publish_pub(self.node_if_prefix + 'status_pub', status_msg) 
        return status_msg




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
        self.namespace = None

    def init(self, do_updates = False):
        """Initialize or re-initialize interface state and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            processes_controls_dict =  self.node_if.get_param(self.processes_param_name)
            if processes_controls_dict is not None:
                self.processes_controls_dict = processes_controls_dict
            selected_process =  self.node_if.get_param(self.node_if_prefix + 'selected_process')
            if selected_process is not None:
                self.selected_process = selected_process
            self.enabled =  self.node_if.get_param(self.node_if_prefix + 'enabled')
        if do_updates == True:
            success = self._reloadProcesses()
            if success == False:
                self.msg_if.pub_warn("PROCESS LOAD FAILED: " + str(self.processes_functions_dict))
            else:
                self.msg_if.pub_warn("Processes Functions Updated: " + str(self.processes_functions_dict.keys()))
                # self.msg_if.pub_warn("Processes Dict Updated: " + str(self.processes_dict))
                # self.msg_if.pub_warn("Process Selected: " + str(self.selected_process))
        self.publish_status()

    def reset(self):
        """Reset the interface to its initialized state."""   
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        nepi_sdk.sleep(1)     
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset the interface to factory defaults."""
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Factory resetting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
        self.init(do_updates = True)

    ###############################
    # Class Private Methods
    ###############################


    # def _addProcessSubs(self, subs_dict, namespace = None, add_prefix = ''):

    #     if subs_dict is None:
    #         subs_dict = dict()
    #     if namespace is None:
    #         namespace = self.namespace
    #     # Subs Config Dict ####################
    #     if self.config_dict['has_enable'] == True:
    #         subs_dict[self.node_if_prefix + add_prefix + 'enable'] = {
    #             'namespace': namespace,
    #             'topic': 'enable',
    #             'msg': Bool,
    #             'qsize': 10,
    #             'callback': self.setEnableCb, 
    #             'callback_args': ()
    #         },
    #     if self.config_dict['has_sources'] == True:
    #         subs_dict[self.node_if_prefix + add_prefix + 'set_source_topic'] = {
    #             'namespace': namespace,
    #             'topic': 'set_source_topic',
    #             'msg': String,
    #             'qsize': 10,
    #             'callback': self.setImageTopicCb, 
    #             'callback_args': ()
    #         }
    #         subs_dict[self.node_if_prefix + add_prefix + 'set_source_topics']  = {         
    #             'namespace': namespace,
    #             'topic': 'set_source_topics',
    #             'msg': StringArray,
    #             'qsize': 10,
    #             'callback': self.setImageTopicsCb, 
    #             'callback_args': ()
    #         }
    #         subs_dict[self.node_if_prefix + add_prefix + 'add_source_topic']  = {          
    #             'namespace': namespace,
    #             'topic': 'add_source_topic',
    #             'msg': String,
    #             'qsize': 10,
    #             'callback': self.addImageTopicCb, 
    #             'callback_args': ()
    #         }
    #         subs_dict[self.node_if_prefix + add_prefix + 'add_source_topics']  = {          
    #             'namespace': namespace,
    #             'topic': 'add_source_topics',
    #             'msg': StringArray,
    #             'qsize': 10,
    #             'callback': self.addImageTopicsCb, 
    #             'callback_args': ()
    #         }
    #         subs_dict[self.node_if_prefix + add_prefix + 'remove_source_topic']  = {           
    #             'namespace': namespace,
    #             'topic': 'remove_source_topic',
    #             'msg': String,
    #             'qsize': 10,
    #             'callback': self.removeImageTopicCb, 
    #             'callback_args': ()
    #         }
    #         subs_dict[self.node_if_prefix + add_prefix + 'remove_source_topics']  = {            
    #             'namespace': namespace,
    #             'topic': 'remove_source_topics',
    #             'msg': StringArray,
    #             'qsize': 10,
    #             'callback': self.removeImageTopicsCb, 
    #             'callback_args': ()
    #         }
    #         subs_dict[self.node_if_prefix + add_prefix + 'process_source_file']  = {           
    #             'namespace': namespace,
    #             'topic': 'process_source_file',
    #             'msg': String,
    #             'qsize': 10,
    #             'callback': self.processFileCb, 
    #             'callback_args': ()
    #         }

         
    #     if self.config_dict['has_process_rate'] == True:
    #         subs_dict[self.node_if_prefix + add_prefix + 'set_max_process_rate'] = {
    #             'namespace': namespace,
    #             'topic': 'set_max_process_rate',
    #             'msg': Float32,
    #             'qsize': 10,
    #             'callback': self.setMaxProcessRateCb, 
    #             'callback_args': ()
    #         }
    #     if self.config_dict['has_image_pub'] == True:
    #         subs_dict[self.node_if_prefix + add_prefix + 'set_image_pub'] = {
    #             'namespace': namespace,
    #             'topic': 'set_image_pub',
    #             'msg': Bool,
    #             'qsize': 10,
    #             'callback': self.setPubImageCb, 
    #             'callback_args': ()
    #         }
    #     if self.config_dict['has_image_rate'] == True:
    #         subs_dict[self.node_if_prefix + add_prefix + 'set_max_image_pub_rate'] = {
    #             'namespace': namespace,
    #             'topic': 'set_max_image_pub_rate',
    #             'msg': Float32,
    #             'qsize': 10,
    #             'callback': self.setMaxImgRateCb, 
    #             'callback_args': ()
    #         }
    #     if self.config_dict['has_use_last_image'] == True:
    #         subs_dict[self.node_if_prefix + add_prefix + 'set_use_last_image'] = {
    #             'namespace': namespace,
    #             'topic': 'set_use_last_image',
    #             'msg':Bool,
    #             'qsize': 10,
    #             'callback': self.setUseLastImageCb, 
    #             'callback_args': ()
    #         }


    # def setEnableCb(self,msg):
    #     #self.msg_if.pub_warn("Received AI enable msg: " + str(msg))
    #     enabled = msg.data
    #     self.setEnable(enabled)



    # def setEnable(self,enabled):
    #     if self.enabled != enabled:
    #         self.enabled = enabled
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('enabled',self.enabled)
                
    #         if enabled == False and not nepi_sdk.is_shutdown():
    #             self.next_source_topic = "None"

            

    # def setAutoSelectEnableCb(self,msg):
    #     #self.msg_if.pub_warn("Received AI auto select source topic msg: " + str(msg))
    #     enabled = msg.data
    #     self.setAutoSelectEnable(enabled)


    # def setAutoSelectEnable(self, enabled):
    #     self.selected_sources = []
    #     self.auto_select_active = enabled
    #     self.auto_select_enabled = enabled
    #     self.publish_status()
    #     if self.node_if is not None:
    #         self.node_if.set_param('auto_select_enabled',self.auto_select_enabled)
            
       

    # def setImageTopicCb(self,msg):
    #     #self.msg_if.pub_info("Received Set Image Topic: " + msg.data)
    #     source_topic = msg.data
    #     self.setImageTopic(source_topic)


    # def setImageTopic(self, source_topic):
    #     #self.msg_if.pub_info("Set Image Topic: " + source_topic)     
    #     if self.selected_sources != [source_topic]:    
    #         self.selected_sources = [source_topic]
    #         self.auto_select_active = False
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('selected_sources',self.selected_sources)
                

    # def setImageTopicsCb(self,msg):
    #     #self.msg_if.pub_info("Received Set Image Topic: " + msg.data)
    #     source_topics = msg.data
    #     self.setImageTopics(source_topics)


    # def setImageTopics(self, source_topics):
    #     #self.msg_if.pub_info("Set Image Topics: " + str(source_topics))     
    #     if self.selected_sources != source_topics:    
    #         self.selected_sources = source_topics
    #         self.auto_select_active = False
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('selected_sources',self.selected_sources)
                


    # def addImageTopicCb(self,msg):
    #     #self.msg_if.pub_info("Received Add Image Topic: " + msg.data)
    #     source_topic = msg.data
    #     self.addImageTopic(source_topic)


    # def addImageTopicsCb(self,msg):
    #     #self.msg_if.pub_info("Received Add Image Topics: " + str(msg))
    #     source_topic_list = msg.array
    #     for source_topic in source_topic_list:
    #         self.addImageTopic(source_topic)


    # def addImageTopic(self,source_topic):   
    #     #self.msg_if.pub_info("Adding Image Topic: " + source_topic)
    #     if source_topic not in self.selected_sources: 
    #         source_topics = copy.deepcopy(self.selected_sources)
    #         if source_topic not in source_topics:
    #             source_topics.append(source_topic)
    #         else:
    #             self.msg_if.pub_warn('Image topic allready selected')
    #         self.selected_sources = source_topics
    #         self.auto_select_active = False
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('selected_sources',self.selected_sources)
                


    # def removeImageTopicCb(self,msg):
    #     #self.msg_if.pub_info("Received Remove Image Topic: " + str(msg))
    #     source_topic = msg.data
    #     self.removeImageTopic(source_topic)


    # def removeImageTopicsCb(self,msg):
    #     #self.msg_if.pub_info("Received Remove Image Topic: " + str(msg))
    #     source_topic_list = msg.array
    #     for source_topic in source_topic_list:
    #         self.removeImageTopic(source_topic)



    # def removeImageTopic(self,source_topic):
    #     #self.msg_if.pub_info("Removing Image Topic: " + source_topic)  
    #     if source_topic in self.selected_sources:       
    #         source_topics = copy.deepcopy(self.selected_sources)
    #         if source_topic in source_topics:
    #             source_topics.remove(source_topic)
    #         self.selected_sources = source_topics
    #         self.auto_select_active = False
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('selected_sources',self.selected_sources)
                


    # ###################
    # # Process Functions

    # def setClassCb(self,msg):
    #     #self.msg_if.pub_info("Received Set class: " + msg.data)
    #     class_name = msg.data
    #     self.setClass(class_name)


    # def setClass(self, class_name):
    #     #self.msg_if.pub_info("Set Class: " + class_name)      
    #     if self.selected_classes != [class_name]:  
    #         self.selected_classes = [class_name]
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('selected_classes',self.selected_classes)
                

    # def setClassesCb(self,msg):
    #     #self.msg_if.pub_info("Received Set classes: " + msg.data)
    #     class_names = msg.data
    #     self.setClasses(class_names)


    # def setClasses(self, class_names):
    #     #self.msg_if.pub_info("Set Class: " + class_name) 
    #     if self.selected_classes != class_names:        
    #         self.selected_classes = class_names
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('selected_classes',self.selected_classes)
                


    # def addAllClassesCb(self,msg):
    #     #self.msg_if.pub_info('Got add all classes msg: ' + str(msg))
    #     self.addAllClasses()

    # def addAllClasses(self):
    #     self.publish_status() # Updated Here
    #     if self.selected_classes != self.classes:
    #         self.selected_classes = self.classes
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('selected_classes', self.classes)
                


    # def removeAllClassesCb(self,msg):
    #     #self.msg_if.pub_info('Got remove all classes msg: ' + str(msg))
    #     if len(self.selected_classes) > 0:
    #         self.selected_classes = []
    #         self.publish_status() # Updated Here
    #         if self.node_if is not None:
    #             self.node_if.set_param('selected_classes',[])
                


    # def addClassCb(self,msg):
    #     #self.msg_if.pub_info('Got add class msg: ' + str(msg))
    #     class_name = msg.data
    #     if class_name in self.classes and class_name not in self.selected_classes:
    #         sel_classes = copy.deepcopy(self.selected_classes)
    #         if class_name not in sel_classes:
    #             sel_classes.append(class_name)
    #         self.selected_classes = sel_classes
    #         self.publish_status() # Updated Here
    #         if self.node_if is not None:
    #             self.node_if.set_param('selected_classes', sel_classes)
                


    # def removeClassCb(self,msg):
    #     #self.msg_if.pub_info('Got remove class msg: ' + str(msg))
    #     class_name = msg.data
        
    #     if class_name in self.selected_classes:
    #         sel_classes = copy.deepcopy(self.selected_classes)
    #         sel_classes.remove(class_name)
    #         self.selected_classes = sel_classes
    #         self.publish_status() # Updated Here
    #         if self.node_if is not None:
    #             self.node_if.set_param('selected_classes', sel_classes)
                



    # def setThresholdCb(self,msg):
    #     threshold = msg.data
    #     self.setThreshold(threshold)

    # def setThreshold(self,threshold):
    #     #self.msg_if.pub_info("Received Threshold Update: " + str(threshold))
    #     if threshold <  MIN_THRESHOLD:
    #         threshold = MIN_THRESHOLD
    #     elif threshold > MAX_THRESHOLD:
    #         threshold = MAX_THRESHOLD
    #     if self.threshold != threshold:
    #         self.threshold = threshold
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('threshold',self.threshold)
                


    # def setMaxProcessRateCb(self,msg):
    #     max_rate = msg.data
    #     if max_rate <  MIN_MAX_RATE:
    #         max_rate = MIN_MAX_RATE
    #     elif max_rate > MAX_MAX_RATE:
    #         max_rate = MAX_MAX_RATE
    #     if max_rate != self.set_process_rate:
    #         self.set_process_rate = max_rate
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('set_process_rate',self.set_process_rate)
                

 


    # def setMaxImgRateCb(self,msg):
    #     max_rate = msg.data
    #     if max_rate <  MIN_MAX_RATE:
    #         max_rate = MIN_MAX_RATE
    #     elif max_rate > MAX_MAX_RATE:
    #         max_rate = MAX_MAX_RATE
    #     if max_rate != self.set_image_rate:
    #         self.set_image_rate = max_rate
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('set_image_rate',self.set_image_rate)
                

    # def setUseLastImageCb(self,msg):
    #     enable = msg.data
    #     if self.use_last_image != enable:
    #         self.use_last_image = enable
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('use_last_image',self.use_last_image)
                





    # def setPubImageCb(self,msg):
    #     enable = msg.data
    #     self.set_pub_image(enable)




    # def set_pub_image(self,enable):
    #     """Enables or disables image publishing and persists the setting.

    #     Args:
    #         enable (bool): True to enable detections image publishing,
    #             False to disable it.
    #     """
    #     if self.imaging_enabled != enable:
    #         self.imaging_enabled = enable
    #         self.publish_status()
    #         if self.node_if is not None:
    #             self.node_if.set_param('imaging_enabled',self.imaging_enabled)
                


    def _systemStatusCb(self,msg):
            self.active_nodes = msg.active_nodes
            self.active_topics = msg.active_topics
            self.active_topic_types = msg.active_topic_types
            self.active_services = msg.active_services



    def _updatePubStats(self):
        if self.last_process_time is None:
            pub_time_sec = 1.0
            self.last_process_time = nepi_utils.get_time()
        else:
            cur_time = nepi_utils.get_time()
            pub_time_sec = cur_time - self.last_process_time
            self.last_process_time = cur_time
        self.process_times.pop(0)
        self.process_times.append(pub_time_sec)

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.reset(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.factory_reset(do_updates = do_updates)


    def _reloadProcessesCb(self,msg):
        self.msg_if.pub_warn("Got Process Reload Msg")
        self._reloadProcesses()
    
    def _setProcessCb(self,msg):
        process_name = msg.data
        self.set_selected_process(process_name)

    def _setEnableCb(self,msg):
        enabled = msg.data
        self.set_enable_process(enabled)

    def _reloadProcesses(self):
        success = False
        if self.process_module is not None:
            self.process_ready = False           
            nepi_sdk.sleep(1)
            process_busy = self.wait_on_process_busy()
            if process_busy == True:
                self.msg_if.pub_info("Failed to load process. Process Busy: " + str(process_busy))
            else:
                processes_controls_dict = copy.deepcopy(self.processes_controls_dict)
                try:
                    importlib.reload(self.process_module)
                    processes_dict = self.process_module.PROCESSES_DICT
                    self.msg_if.pub_warn("################################")
                    self.msg_if.pub_warn("Process Reloaded")
                    self.msg_if.pub_warn("Updating Process Dictionaries")


                    try:
                        self.results_pub_msg = self.process_module.RESULTS_PUB_MSG
                        self.has_results_pub = self.has_results_pub == True and self.results_pub_msg is not None
                    except:
                        self.has_results_pub = False

                    available_processes = []
                    for process_name in processes_dict.keys():
                        available_processes.append(process_name)
                        if process_name in processes_controls_dict.keys():

                                if 'controls_dict' in processes_dict[process_name].keys():
                                    for control_name in processes_controls_dict[process_name].keys():
                                        #self.msg_if.pub_warn("Updating Processes control_name: " + str([control_name]))
                                        if control_name in processes_dict[process_name]['controls_dict'].keys():
                                            control_value = processes_controls_dict[process_name][control_name]
                                            nepi_controls.set_value(processes_dict[process_name]['controls_dict'], control_name, control_value )

                    self.available_processes = available_processes
                    self.processes_dict = processes_dict
                    self.processes_functions_dict = self.process_module.FUNCTIONS_DICT
                    #self.msg_if.pub_warn("Processes Functions Updated: " + str(self.processes_functions_dict))

                    processes_controls_dict = dict()
                    for process_name in processes_dict.keys():
                        try:
                            processes_controls_dict[process_name] = processes_dict[process_name]['controls_dict']
                        except:
                            pass




                    #self.msg_if.pub_warn("")
                    #self.msg_if.pub_warn("Processes Dict Updated: " + str(self.processes_dict))
                    #self.msg_if.pub_warn("################################")
                    if self.selected_process is None:
                        self.selected_process = 'None'
                    selected_process = self.selected_process    
                    if selected_process == 'None' or selected_process not in self.available_processes:
                        selected_process = self.available_processes[0]
                        try:
                            selected_process = self.process_module.DEFAULT_PROCESS
                        except:
                            pass
                    self.selected_process = selected_process
                    #self.msg_if.pub_warn("Process Selected: " + str(self.selected_process))
                    success = self.set_selected_process(self.selected_process, check_updates = False)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to reload process class: " + str(e)) 
        if success == False:
            self.process_ready = False
        return success


    def _updateControlCb(self,msg):
        self.msg_if.pub_info("Received control update msg: " + str(msg), log_name_list = self.log_name_list)
        control_name = msg.name
        # Same fix as ControlsIF._updateControlCb: apply_update_msg writes
        # through the dict it is handed.
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.apply_update_msg(controls_dict, msg)
        control_value = nepi_controls.get_value(controls_dict, control_name )
        self.set_control_value(control_name, control_value)

    def _publishResults(self, results_dict, source_topic = ''):
        #self.msg_if.pub_warn("Starting Pub Result Process with Results Dict and Results Msg: " + str([results_pub_msg, self.results_pub_msg]), throttle_s = 10) 
        try:
            results_pub_dict = copy.deepcopy(self.process_module.RESULTS_PUB_DICT)
        except:
            results_pub_dict = None
        if results_pub_dict is not None and results_dict is not None:
            for key in results_pub_dict.keys():
                if key in results_dict.keys():
                    results_pub_dict[key] = results_dict[key]


            if 'data_header' in results_pub_dict.keys():
                process_timestamp = nepi_utils.get_time()
                data_header = dict()
                data_header['process_name'] = self.node_name
                data_header['process_namespace'] = self.node_namespace
                data_header['process_timestamp'] = process_timestamp
                data_header['sour= nepi_utils.get_time() ce_topic'] = source_topic
                if 'timestamp' in results_dict.keys():
                    source_timestamp = results_dict['timestamp']
                else:
                    source_timestamp = process_timestamp
                data_header['source_timestamp'] = source_timestamp
                for key in data_header.keys():
                    if key in results_pub_dict['data_header'].keys():
                        results_pub_dict['data_header'][key] = data_header[key]

            try:          
                msg = self.process_module.RESULTS_PUB_MSG
                msg_type = self.process_module.RESULTS_PUB_TYPE
                results_pub_msg = nepi_process.convert_results_pub_dict2msg(msg, msg_type, results_pub_dict)
            except:
                results_pub_msg = None
                
            if self.node_if is not None and self.results_pub_msg is not None:
                self.node_if.publish_pub(self.node_if_prefix + 'results_pub', results_pub_msg) 
            else:
                #self.msg_if.pub_warn("Failed to Pub. Results Msg is None: " + str([results_pub_dict, self.results_pub_msg]), throttle_s = 10) 
                pass


    def _publishStatusCb(self, timer):
        self.admin_enabled = nepi_system.get_admin_mode()
        self.publish_status()


# class ProcessIF:
    
#     msg_if = None
#     node_if = None
#     node_if_shared = False
#     save_data_if = None

#     process_image_name = None
#     namespace = ''
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
#     set_process_rate = 10
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

#     show_sources = True
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
#     set_image_rate = 10
#     image_pub_enabled = True
#     use_last_image = False
#     imaging_source_topics = []
#     imaging_pub_topics = []

#     status_has_published = False

#     #######################
#     ### IF Initialization
#     def __init__(self, 
#                 process_image_name = None,
#                 process_group = 'PROCESS',
#                 process_description = 'Process',
#                 process_module = None,
#                 process_status_msg = None,
#                 get_status_dict_callback = None,
#                 process_data_msg = None,
#                 process_data_products = [],
#                 process_results_msg = None,
#                 set_process_rate = 10,
#                 source_status_msg_type = None,
#                 source_data_msg_type = None,       
#                 source_callback_dict = None,       
#                 auto_select_enabled = True,
#                 muti_source_enabled = True,
#                 exclude_source_filters = [],
#                 selected_sources = [],
#                 has_image_pub = False,
#                 image_pub_name = 'image',
#                 set_image_rate = 10,
#                 show_sources = True,
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
#         self.process_image_name = nepi_utils.get_clean_name(process_image_name)
#         if self.process_image_name is None or self.process_image_name == '':
#             self.msg_if.pub_warn("Process Name Not Valid: " + str(process_image_name)) 
#             return
#         self.msg_if.pub_info("Using Process Name: " + self.process_image_name)
#         self.namespace = nepi_sdk.create_namespace(self.node_namespace,self.process_image_name)
#         self.node_if_prefix = self.process_image_name + '_'

#         # Load Process Module
        
#         if process_module is None:
#             self.msg_if.pub_warn("Process Module Not Provided: " + str(process_image_name)) 
#             return
#         self.process_module = process_module
#         self.msg_if.pub_info("Using Process Module: " + self.process_module)


#         ##############################    
#         # Initialize Class Variables

#         self.process_group = str(process_group)
#         self.process_description = str(process_description)
#         # Check Process Status Msg Type


#         self.set_process_rate = set_process_rate

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
#             self.set_image_rate = set_image_rate



#         self.show_sources = show_sources
#         self.show_controls = show_controls
#         self.show_data = show_data
#         self.show_results = show_results

                   
#         ##############################   
#         ## Node Setup

#         # Configs Config Dict ####################
#         CFGS_DICT = {
#                 'namespace': self.namespace
#         }

#         # Params Config Dict ####################
#         # Persist the selected topic under the connect namespace so the
#         # selection survives node restarts (via the config manager). Passing a
#         # params_dict is what enables config management on NodeClassIF.
#         PARAMS_DICT = {
#             self.node_if_prefix + 'selected_sources': {
#                 'namespace': self.namespace,
#                 'factory_val': self.selected_sources
#             }
#         }


#         # Publishers Config Dict ####################
#         self.process_node_pubs_dict = dict()


#         if process_status_msg is not None:
#             self.process_status_msg = process_status_msg
#             self.process_node_pubs_dict[self.node_if_prefix + 'status_pub'] = {
#                 'namespace': self.namespace,
#                 'topic': 'status',
#                 'msg': self.process_status_msg,
#                 'qsize': 1,
#                 'latch': True
#             }

#         # Check Process Status Msg Type
#         if process_data_msg is not None:
#             self.process_data_msg = process_data_msg
#             self.process_node_pubs_dict[self.node_if_prefix + 'data_pub'] = {
#                 'namespace': self.namespace,
#                 'topic': 'data',
#                 'msg': self.process_data_msg,
#                 'qsize': 1,
#                 'latch': True
#             }

#         # Check Process Status Msg Type
#         if process_results_msg is not None:
#             self.process_results_msg = process_results_msg
#             self.process_node_pubs_dict[self.node_if_prefix + 'results_pub'] = {
#                 'namespace': self.namespace,
#                 'topic': 'results',
#                 'msg': self.process_results_msg,
#                 'qsize': 1,
#                 'latch': True
#             }

        

#         # Subscribers Config Dict ####################
#         self.process_node_subs_dict = {
#             self.node_if_prefix + 'set_source': {
#                 'namespace': self.namespace,
#                 'topic': 'set_source',
#                 'msg': String,
#                 'qsize': None,
#                 'callback': self._setSourceCb, 
#                 'callback_args': ()
#             },
#             self.node_if_prefix + 'remove_source': {
#                 'namespace': self.namespace,
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
#                 'namespace': self.namespace,
#                 'topic': 'set_sources',
#                 'msg': StringArray,
#                 'qsize': 10,
#                 'callback': self._setSourcesCb, 
#                 'callback_args': ()
#             }
#             self.process_node_subs_dict[self.node_if_prefix + 'add_sources'] = {
#                 'namespace': self.namespace,
#                 'topic': 'add_sources',
#                 'msg': String,
#                 'qsize': 10,
#                 'callback': self._addSourcesCb, 
#                 'callback_args': ()
#             }
#             self.process_node_subs_dict[self.node_if_prefix + 'remove_sources'] = {
#                 'namespace': self.namespace,
#                 'topic': 'add_sources',
#                 'msg': Empty,
#                 'qsize': 10,
#                 'callback': self._removeSourcesCb, 
#                 'callback_args': ()
#             }
#             self.process_node_subs_dict[self.node_if_prefix + 'clear_sources'] = {
#                 'namespace': self.namespace,
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
#                 'namespace': self.namespace,
#                 'topic': 'set_process',
#                 'msg': String,
#                 'qsize': 10,
#                 'callback': self._setProcessCb, 
#                 'callback_args': ()
#             }

#             self.process_node_subs_dict[self.node_if_prefix + 'set_process_max_rate'] = {
#                 'namespace': self.namespace,
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
#                 self.node_if.add_param('selected_sources', self.namespace, self.selected_sources)
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

#             sd_namespace = self.namespace
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
#         return self.namespace
    

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

#         status_msg.name = self.process_image_name
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


#         status_msg.show_sources = self.show_sources
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
       





# #########################################
# # Process Image IF Class
# #########################################


# WATCHDOG_DELAY=60
# WATCHDOG_TIMEOUT=3



# NONE_IMG_DICT = {       
#     'width': 0,
#     'height': 0,
#     'timestamp': nepi_sdk.get_time(),
#     'source_topic': 'None',
#     'ros_img_header': Header(),
#     'source_timestamp': Header().stamp
# }


# BLANK_SIZE_DICT = { 'h': 350, 'w': 700, 'c': 3}

# BLANK_CV2_IMAGE = nepi_img.create_blank_image((BLANK_SIZE_DICT['h'],BLANK_SIZE_DICT['w'],BLANK_SIZE_DICT['c']))

# BLANK_IMG_DICT = {       
#     'width': 0,
#     'height': 0,
#     'timestamp': nepi_sdk.get_time(),
#     'source_topic': 'None',
#     'source_timestamp': nepi_sdk.get_time()
# }




# class ProcessImageIF:
#     process_name = 'process'
#     process_image_name = 'process_image'
#     process_image_function = None

#     process_ready = False

#     options_dict = None


#     DATA_PRODUCTS = ['targets','targets_image']
#     TARGETS_IMG_DATA_PRODUCT = 'targets_image'

#     OUTPUT_IMG_PRODUCTS = [ TARGETS_IMG_DATA_PRODUCT]


#     target_sub_names = ['targets']

#     node_if = None


#     self_managed = True
#     model_name = "None"


#     save_data_if = None

#     cv2_img = None
#     cv2_img_lock = threading.Lock()

#     img_node_dict = dict()
#     img_node_lock = threading.Lock()


#     imgs_info_dict = dict()
#     imgs_info_lock = threading.Lock()
#     imgs_img_proc_dict = dict()

#     save_cfg_if = None

#     state_str_msg = 'Loading'

#     cur_source_topic = "None"
#     get_source_topic = "None"
#     last_get_image_time = 0
#     got_source_topic = "None"

#     clear_img_time = 1.0

#     clear_targets_time = 1.0

#     detection_state = False

#     classes_list = []
#     classes_colors_list = []
#     selected_classes = []
    
#     last_status_time=None

#     data_products = DATA_PRODUCTS

#     connected = False

#     watchdog_timeout = None    
#     #######################
#     ### IF Initialization
#     def __init__(self, 
#                 process_name = 'process',
#                 process_image_name = 'process_image',
#                 process_module = None,              
#                 log_name = None,
#                 log_name_list = [],
#                 msg_if = None,
#                 node_if = None,
#                 navpose_if = None,
#                 save_data_if = None,
#                 ):
#         ####  IF INIT SETUP ####
#         self.class_name = type(self).__name__
#         self.base_namespace = nepi_sdk.get_base_namespace()
#         self.node_name = nepi_sdk.get_node_name()
#         self.node_namespace = nepi_sdk.get_node_namespace()

    



#         # Create Msg Class
#         if msg_if is not None:
#             self.msg_if = msg_if
#         else:
#             self.msg_if = MsgIF()
#         self.log_name_list = copy.deepcopy(log_name_list)
#         if log_name is not None:
#             log_name = nepi_utils.get_clean_name(log_name)
#             self.log_name_list.append(log_name)
#         self.log_name_list.append(self.class_name)
#         self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

#         # Create Process Name
#         self.process_image_name = nepi_utils.get_clean_name(process_image_name)
#         if self.process_image_name is None or self.process_image_name == '':
#             self.msg_if.pub_warn("Process Name Not Valid: " + str(process_image_name)) 
#             return
#         self.msg_if.pub_info("Using Process Name: " + self.process_image_name)

#         if process_image_namespace is None:
#             process_image_namespace = self.node_namespace
#         self.namespace = nepi_sdk.create_namespace(process_image_namespace,self.process_image_name)
#         self.data_products = [self.process_image_name]



#         if process_module is None:
#             self.msg_if.pub_warn("No Process Module Provided")
#             return

#         self.process_module = process_module

#         # Registry keys on a shared node_if must be domain-unique, so every key
#         # this IF adds carries the process name. Param wire names ARE
#         # namespace + key, so the prefix is part of the external param surface.
#         self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

       
#         ##############################    
#         # Initialize Class Variables

#         success = self._reloadProcesses()
#         if success == False:
#             self.msg_if.pub_warn("INITIAL PROCESS LOAD FAILED" + str(self.process_image_function))
#         else:
#             self.msg_if.pub_warn("INITIAL PROCESS LOAD SUCCEEDED: " + str(self.process_image_function))

#         option_keys = None
#         if self.options_dict is not None:
#             option_keys = list(self.options_dict.keys())
#         self.msg_if.pub_warn("Got Image Options" + str(option_keys))

#         # if data_product is not None:
#         #     data_product = nepi_utils.get_clean_name(data_product)
#         #     if data_product is not None:
#         #         self.data_product = data_product
#         self.save_data_if = save_data_if
#         self.navpose_if = navpose_if
#         self.node_if = node_if
#         # # Call the parent class constructor
#         # super().__init__(namespace = self.namespace ,
#         #         data_product = self.process_image_name,
#         #         data_source_description = self.process_name,
#         #         data_ref_description = self.process_name,
#         #         perspective = 'pov',
#         #         init_overlay_text_list = [],
#         #         options_dict = self.options_dict,
#         #         callback_dict = self.callback_dict,
#         #         navpose_if = self.navpose_if,
#         #         navpose_namespace = None,
#         #         transform_namespace = None,
#         #         save_data_if = self.save_data_if,
#         #         live_adjustments_disabled = False,
#         #         aspect_adjustment_disabled = False,
#         #         log_name = None,
#         #         log_name_list = [],
#         #         msg_if = self.msg_if,
#         #         node_if = self.node_if
#         # )



#         self.msg_if.pub_warn("Starting with Data Products: " + str(self.data_products))
        
#         self.process_namespace = self.node_namespace.replace("_img_pub","")
#         self.msg_if.pub_warn("Starting with Process Namespace: " + str(self.process_namespace))


#         self.status_msg = ProcessStatus()
#         self.model_name = 'None'
#         self.enabled = False
#         self.state_str_msg = "Unknown"
#         self.set_image_rate = 10
#         self.use_last_image = True

#         self.imaging_enabled = True
#         self.overlay_labels = True
#         self.overlay_range_bearing = True
#         # self.overlay_clf_name = False
#         # self.overlay_img_name = False

#         self.selected_source_topics = []
#         self.selected_img_navpose_topics = []
#         self.img_process_namespaces = []
#         self.img_targets_states = []

        

#         ##############################  
#         # Create NodeClassIF Class  

#         # Configs Dict ########################
#         self.CONFIGS_DICT = None
#         '''
#                 {
#                 'init_callback': self.initCb,
#                 'reset_callback': self.resetCb,
#                 'factory_reset_callback': self.factoryResetCb,
#                 'init_configs': True,
#                 'namespace':  self.process_namespace,
#         }
#         '''


#         # Params Config Dict ####################
#         self.PARAMS_DICT = None


#         # Services Config Dict ####################
#         self.SRVS_DICT = None


#         self.PUBS_DICT = None


#         # Subs Config Dict ####################
#         self.SUBS_DICT = {
#             self.node_if_prefix + 'reload_process': {
#                 'namespace': self.namespace,
#                 'topic': 'reload_process',
#                 'msg': Empty,
#                 'qsize': 10,
#                 'callback': self._reloadProcessesCb
#             },
#             'targets_status_sub': {
#                 'msg': TargetsStatus,
#                 'namespace': self.process_namespace + '/targets',
#                 'topic': 'status',
#                 'qsize': 10,
#                 'callback': self.targetsStatusCb,
#                 'callback_args': ()
#             },
#             'targets': {
#                 'msg': Targets,
#                 'namespace': self.process_namespace,
#                 'topic': 'targets',
#                 'qsize': 10,
#                 'callback': self.targetsCb,
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







#     if node_if is None:
#         self.node_if = NodeClassIF(
#                         configs_dict = CFGS_DICT,
#                         params_dict = PARAMS_DICT,
#                         services_dict = None,
#                         pubs_dict = self.process_node_pubs_dict,
#                         subs_dict = self.process_node_subs_dict,
#                         log_name_list = [],
#                         msg_if = self.msg_if
#         )
#         self.node_if.wait_for_ready()
#     else:
#         self.config_if = self.namespace
#         self.node_if_shared = True
#         try:
#             self.node_if = node_if
#             self.node_if.register_pubs(self.process_node_pubs_dict)
#             self.node_if.register_subs(self.process_node_subs_dict)
#             # Register this IF's params on the shared node_if too, or
#             # get_param/set_param below resolve to no namespace and the
#             # controls dict and enable state never persist.
#             self.node_if.add_params(PARAMS_DICT)
#             nepi_sdk.sleep(1)
#         except Exception as e:
#             self.msg_if.pub_info("Failed to register pubs and subs: " + str(e))
#             return





#         ###############################
#         # Create System IFs

       
#         # Setup Save Data IF
#         factory_data_rates= {}
#         for d in self.data_products:
#             factory_data_rates[d] = [1.0, 0.0, 100]            
#         self.save_data_if = SaveDataIF(data_products = self.data_products, pub_status = False, factory_rate_dict = factory_data_rates, namespace = self.process_namespace,
#                         msg_if = self.msg_if,
#                             node_if = self.node_if
#                         )
        
#         nepi_sdk.sleep(1)
#         if self.save_data_if is not None:
#             self.status_msg.save_data_topic = self.save_data_if.get_namespace()
#             self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic))


#         time.sleep(1)


#         ##########################
#         # Complete Initialization

#         # Start Timer Processes
        
#         nepi_sdk.start_timer_process((0.1), self.updaterCb, oneshot = True)
#         #nepi_sdk.start_timer_process((0.1), self.updateImgSubsCb, oneshot = True)
#         self.last_status_time=nepi_utils.get_time()
#         nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)
#         nepi_sdk.on_shutdown(self.shutdownCb)
        
#         #########################################################
#         ## Initiation Complete
#         self.msg_if.pub_info("Initialization Complete")
#         # Spin forever (until object is detected)
#         nepi_sdk.spin()
#         #########################################################


#     def initCb(self,do_updates = False):
#         self.msg_if.pub_info(" Setting init values to param values")
#         if do_updates == True:
#             pass


#     def resetCb(self,do_updates = True):
#         self.last_targets_dict_list = []
#         if do_updates == True:
#             pass
#         self.initCb()

#     def factoryResetCb(self,do_updates = True):
#         self.last_targets_dict_list = []
#         if do_updates == True:
#             pass
#         self.initCb()



#     #######################
#     # Class Public Methods
#     #######################

#     ###############################
#     # Class Private Methods
#     ###############################


#     def unregister_pubs(self):
#         """Unregister all ROS publishers managed by this interface."""
#         if self.node_if is not None:
#             if self.node_if_shared == False:
#                 self.node_if.unregister_pubs()
#             else:
#                 if self.process_node_pubs_dict is not None:
#                     for pub_name in self.process_node_pubs_dict.keys():
#                         self.node_if.unregister_pub(pub_name)

#     def unsubscribe(self):
#         """Shut down this interface, unregister all owned ROS resources, and clear state."""
#         self.ready = False
#         if self.node_if is not None and self.node_if_shared == False:
#             self.node_if.unregister_class()
#         else:
#             self.unregister_pubs()
#         time.sleep(1)
#         self.namespace = None

#     def init(self, do_updates = False):
#         """Initialize or re-initialize interface state and publish status.

#         Args:
#             do_updates (bool, optional): Reserved for future use. Defaults to False.
#         """
#         if self.node_if is not None:
#             processes_controls_dict =  self.node_if.get_param(self.processes_param_name)
#             if processes_controls_dict is not None:
#                 self.processes_controls_dict = processes_controls_dict
#             selected_process =  self.node_if.get_param(self.node_if_prefix + 'selected_process')
#             if selected_process is not None:
#                 self.selected_process = selected_process
#             self.enabled =  self.node_if.get_param(self.node_if_prefix + 'enabled')
#         if do_updates == True:
#             success = self._reloadProcesses()
#             if success == False:
#                 self.msg_if.pub_warn("PROCESS LOAD FAILED: " + str(self.processes_functions_dict))
#             else:
#                 self.msg_if.pub_warn("Processes Functions Updated: " + str(self.processes_functions_dict.keys()))
#                 # self.msg_if.pub_warn("Processes Dict Updated: " + str(self.processes_dict))
#                 # self.msg_if.pub_warn("Process Selected: " + str(self.selected_process))
#         self.publish_status()

#     def reset(self):
#         """Reset the interface to its initialized state."""   
#         if self.node_if is not None and self.node_if_shared == False:
#             self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
#             self.node_if.reset_params()
#         nepi_sdk.sleep(1)     
#         self.init(do_updates = True)

#     def factory_reset(self):
#         """Reset the interface to factory defaults."""
#         if self.node_if is not None and self.node_if_shared == False:
#             self.msg_if.pub_info("Factory resetting params", log_name_list = self.log_name_list)
#             self.node_if.factory_reset_params()
#         self.init(do_updates = True)

#     ###############################
#     # Class Private Methods
#     ###############################







#     def getImgInfoDict(self):
#         self.imgs_info_lock.acquire()
#         imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
#         self.imgs_info_lock.release()
#         return imgs_info_dict

#     def getActiveImgTopics(self):
#         self.imgs_info_lock.acquire()
#         imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
#         self.imgs_info_lock.release()
#         source_topics = list(imgs_info_dict.keys())
#         #self.msg_if.pub_warn("Updating active topics: " +  str(source_topics))
#         active_source_topics = []
#         for source_topic in source_topics:
#             if 'active' in imgs_info_dict[source_topic].keys():
#                 if imgs_info_dict[source_topic]['active'] == True:
#                     #self.msg_if.pub_warn("Found active topic: " +  str(source_topic))
#                     active_source_topics.append(source_topic)
#         return active_source_topics


#     def updaterCb(self,timer):
#         # Clear boxes if stall
#         selected_source_topics = copy.deepcopy(self.selected_source_topics)
#         active_source_topics = self.getActiveImgTopics()
#         #self.msg_if.pub_warn("")
#         #self.msg_if.pub_warn("Updating with image topics: " +  str(selected_source_topics))
#         #self.msg_if.pub_warn("Updating with active image topics: " +  str(active_source_topics))
#         current_time = nepi_utils.get_time()
#         for source_topic in self.imgs_info_dict.keys():
#             last_time = self.imgs_info_dict[source_topic]['last_targets_time']
#             check_time = current_time - last_time
#             '''
#             if check_time > self.clear_targets_time or self.enabled == False or self.state_str_msg != 'Detecting':
#                 try:
#                     self.imgs_info_dict[source_topic]['target_dict_list'] = None
#                 except:
#                     pass
#             '''


#         # Do Image Subs updating

#         #self.msg_if.pub_warn("Subscriber Check with selected image topics: " +  str(selected_source_topics))
#         #self.msg_if.pub_warn("Subscriber Check  with active image topics: " +  str(active_source_topics))
#         purge_list = []
#         if self.set_image_rate == -1 :
#             purge_list = selected_source_topics
#         elif self.imaging_enabled == True:
#             # Update Image subscribers
#             found_source_topics = []
#             for source_topic in selected_source_topics:
#                 if os.path.basename(source_topic) in self.OUTPUT_IMG_PRODUCTS:
#                     continue
#                 source_topic = nepi_sdk.find_topic(source_topic, exact = True)
#                 if source_topic != '':
#                     found_source_topics.append(source_topic)
#                     if source_topic not in active_source_topics:
#                         self.msg_if.pub_warn('Will subscribe to image topic: ' + source_topic)
#                         success = self.subscribeImgTopic(source_topic)
#                         #self.msg_if.pub_warn('Subscribe process returned: ' + str(success))
                       
#             # Update Image Subs purge list      
#             active_source_topics = self.getActiveImgTopics() 
#             #self.msg_if.pub_warn('Purge Check with active topics: ' + str(active_source_topics))       
#             #self.msg_if.pub_warn('Purge Check with found topics: ' + str(active_source_topics)) 
#             for source_topic in active_source_topics:
#                 if source_topic not in found_source_topics:
#                     purge_list.append(source_topic)
#         elif self.imaging_enabled == False:
#              purge_list = copy.deepcopy(list(self.imgs_info_dict.keys()))

#         # Do image sub purging if required
#         #self.msg_if.pub_warn('Purging image topics: ' + str(purge_list))
#         #self.msg_if.pub_warn("")
#         for source_topic in purge_list:
#                 if source_topic not in active_source_topics:
#                     purge_list.remove(source_topic)
#         if len(purge_list) > 0:
#             self.msg_if.pub_warn('Purging image topics: ' + str(purge_list))
#         for source_topic in purge_list:
#             self.msg_if.pub_warn('Will unsubscribe from image topic: ' + source_topic)
#             success = self.unsubscribeImgTopic(source_topic)
#             self.msg_if.pub_warn('Unsubsribe process returned: ' + str(success))
#             nepi_sdk.sleep(1)

#         '''
#         self.pub_img_if.publish_msg_img("Detector not Enabled")

#         self.pub_img_if.publish_msg_img("Detector Sleeping")

#         self.pub_img_if.publish_msg_img("Waiting for Image")

#         self.pub_img_if.publish_msg_img("Image not Connected")
#         '''
#         nepi_sdk.start_timer_process((1), self.updaterCb, oneshot = True)


#     def watchdogCb(self,timer):
#         cur_time=nepi_utils.get_time()
#         timer=cur_time-self.last_status_time
#         if self.watchdog_timeout is None:
#             self.watchdog_timeout = WATCHDOG_TIMEOUT
#             nepi_sdk.sleep(WATCHDOG_DELAY)
#         else:
#             if timer > WATCHDOG_TIMEOUT:
                
#                 msg="Lost connection to parent node status msg.  Shutting down"
#                 self.msg_if.pub_warn(msg)
#                 nepi_sdk.signal_shutdown(msg)
            
#         nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)


#     def subscribeImgTopic(self,source_topic):
#         success = False
#         #self.msg_if.pub_warn('Subscribing with image dict keys: ' + str(self.imgs_info_dict.keys()))
#         if source_topic == "None" or source_topic == "":
#             self.msg_if.pub_warn('Skipping subscribe, Image topic is None: ' + str(source_topic))
#             return False
        
#         # Create Publishers
#         self.msg_if.pub_warn('Subscribing to image topic: ' + source_topic)

#         img_source_topic = os.path.dirname(source_topic)
#         targets_name = os.path.basename(self.process_namespace)
#         #self.msg_if.pub_warn('Creating namespace for image name: ' + img_source_topic)

#         pub_namespace = img_source_topic #os.path.join(os.path.dirname(source_topic),targets_name)
#         img_pub_topic = os.path.join(pub_namespace,self.TARGETS_IMG_DATA_PRODUCT)
#         self.msg_if.pub_warn('Publishing imgage ' + img_source_topic + ' on namespace: ' + img_pub_topic)


#         if source_topic in self.imgs_info_dict.keys():

#             imgs_info_dict = self.getImgInfoDict()
#             if imgs_info_dict[source_topic]['active'] == True:
#                 self.msg_if.pub_warn('Skipping subscribe, Image Topic is active: ' + source_topic)
#                 return  False
#             self.img_node_lock.acquire()
#             if source_topic in self.img_node_dict.keys():
#                 self.imgs_info_dict[source_topic]['active'] = True
#                 self.img_node_dict[source_topic]['img_pub'] = nepi_sdk.create_publisher(img_pub_topic,Image, queue_size = 1, log_name_list = [])
#                 nepi_sdk.sleep(1)
#                 self.img_node_dict[source_topic]['img_sub'] = nepi_sdk.create_subscriber(source_topic,Image, self.imageCb, queue_size = 1, callback_args= (source_topic), log_name_list = [])
#                 self.img_node_dict[source_topic]['img_if'].register_pubs()
#             self.img_node_lock.release()

#             return True


#         img_pub = nepi_sdk.create_publisher(img_pub_topic,Image, queue_size = 1, log_name_list = [])
#         nepi_sdk.sleep(1)
#         img_sub = nepi_sdk.create_subscriber(source_topic,Image, self.imageCb, queue_size = 1, callback_args= (source_topic), log_name_list = [])
#         img_status_topic = nepi_sdk.create_namespace(source_topic, 'status')
#         self.msg_if.pub_warn('Subscribing to image status topic: ' + img_status_topic)
#         img_stutus_sub = nepi_sdk.create_subscriber(img_status_topic,ImageStatus, self.imageStatusCb, queue_size = 1, callback_args= (source_topic), log_name_list = [])


#         # Create detections image publisher
#         img_if = ColorImageIF(namespace = pub_namespace ,
#                         data_product = 'targets_image',
#                         data_source_description = 'image',
#                         data_ref_description = 'image',
#                         perspective = 'pov',
#                         save_data_if = self.save_data_if,
#                         init_overlay_text_list = [],
#                         live_adjustments_disabled = True,
#                         aspect_adjustment_disabled = True,
#                         log_name = 'targets_image',
#                         log_name_list = [],
#                         msg_if = self.msg_if,
#                         node_if = self.node_if
#                         )


#         # Subscribe to new image topic
#         self.img_node_lock.acquire()
#         self.img_node_dict[source_topic] = {
#                                         'img_sub': img_sub,
#                                         'img_status_sub': img_stutus_sub,
#                                         'img_pub': img_pub,
#                                         'img_if': img_if,

#                                         }
#         self.img_node_lock.release()

#         ####################
#         # Create img info dict
#         img_info_dict = dict()  
#         img_info_dict['active'] = True
#         img_info_dict['img_connected'] = False
#         img_info_dict['img_published'] = False
#         img_info_dict['targets_img_published'] = False
#         img_info_dict['status_dict'] = None
#         img_info_dict['pub_namespace'] = pub_namespace

#         img_info_dict['connected'] = False
#         img_info_dict['publishing'] = False
#         img_info_dict['get_latency_time'] = 0
#         img_info_dict['pub_latency_time'] = 0
#         img_info_dict['process_time'] = 0
#         img_info_dict['last_img_time'] = 0
#         img_info_dict['last_targets_time'] = 0
#         img_info_dict['target_dict_list'] = []
#         img_info_dict['last_img'] = None

#         self.imgs_info_lock.acquire()
#         self.imgs_info_dict[source_topic] = img_info_dict
#         self.imgs_info_lock.release()
#         #self.msg_if.pub_warn('Subscribed with image dict key: ' + str(img_info_dict.keys()))
#         #self.msg_if.pub_warn('Subscribed with images dict: ' + str(self.imgs_info_dict))
       
#         return True
    


#     def unsubscribeImgTopic(self,source_topic):
#         if source_topic in self.imgs_info_dict.keys():
#             if self.imgs_info_dict[source_topic]['active'] == True:
#                 self.msg_if.pub_warn('Unsubscribing from image topic: ' + source_topic)


#                 # Unsubscribe
#                 self.img_node_lock.acquire()
#                 if source_topic in self.img_node_dict.keys():
#                     if self.img_node_dict[source_topic]['img_sub'] is not None:
#                         self.img_node_dict[source_topic]['img_sub'].unregister()
#                     if self.img_node_dict[source_topic]['img_status_sub'] is not None:
#                         self.img_node_dict[source_topic]['img_status_sub'].unregister
#                     if self.img_node_dict[source_topic]['img_pub'] is not None:
#                         self.img_node_dict[source_topic]['img_pub'].unregister()
#                     if self.img_node_dict[source_topic]['img_if'] is not None:
#                         self.img_node_dict[source_topic]['img_if'].unregister_pubs()
#                     nepi_sdk.sleep(1)
#                     self.img_node_dict[source_topic]['img_sub'] = None
#                     self.img_node_dict[source_topic]['img_status_sub'] = None
#                     self.img_node_dict[source_topic]['img_pub'] = None
#                 self.img_node_lock.release()

#                 if source_topic in self.imgs_info_dict.keys():
#                     self.msg_if.pub_warn('Setting image topic inactive: ' + source_topic)
#                     self.imgs_info_lock.acquire()
#                     self.imgs_info_dict[source_topic]['active'] = False
#                     self.imgs_info_dict[source_topic]['status_dict'] = None
#                     self.imgs_info_dict[source_topic]['connected'] = False 
#                     self.imgs_info_dict[source_topic]['publishing'] = False
#                     self.imgs_info_dict[source_topic]['img_connected'] = False
#                     self.imgs_info_dict[source_topic]['img_published'] = False
#                     self.imgs_info_dict[source_topic]['targets_img_published'] = False
#                     self.imgs_info_dict[source_topic]['last_img'] = None

#                     self.imgs_info_lock.release()
#                     #self.msg_if.pub_warn('Unubscribed with images dict: ' + str(self.imgs_info_dict))

#                 nepi_sdk.sleep(1)

                    
#                 return True
#         return False




#     def imageStatusCb(self, status_msg, args):   
#             source_topic = args  
#             if source_topic not in self.imgs_info_dict.keys():
#                 return
#             self.imgs_info_lock.acquire()
#             if source_topic in self.imgs_info_dict.keys():
#                 status_dict = nepi_sdk.convert_msg2dict(status_msg)
#                 if self.imgs_info_dict[source_topic]['status_dict'] is None:
#                     self.msg_if.pub_warn('Connected to image status topic: ' + source_topic + '/status')
#                     self.msg_if.pub_warn('Got width,height degs: ' + str([status_dict['width_deg'],status_dict['height_deg']]))
#                 self.imgs_info_dict[source_topic]['status_dict'] = status_dict
#             self.imgs_info_lock.release()                


#     def imageCb(self, image_msg, args):   
#             source_topic = args  

#             if source_topic not in self.imgs_info_dict.keys():
#                 return


#             if self.imgs_info_dict[source_topic]['img_connected'] == False:
#                 self.msg_if.pub_warn('Connected to image topic: ' + source_topic)
#             self.imgs_info_dict[source_topic]['img_connected'] = True



#             needs_img = False
#             needs_targets_img = False
#             if source_topic in self.imgs_info_dict.keys():
#                 if  self.img_node_dict[source_topic]['img_if'] is not None:
#                     needs_img = self.img_node_dict[source_topic]['img_if'].needs_data_check()
#                 if 'publishing' in self.imgs_info_dict[source_topic].keys():
#                     if self.imgs_info_dict[source_topic]['publishing'] == False:
#                         pass

#             if ( needs_img or needs_targets_img ) and self.imaging_enabled:
#                 start_time = nepi_sdk.get_time()
                


#                 sel_imgs = copy.deepcopy(self.selected_source_topics) 
#                 set_image_rate = copy.deepcopy(self.set_image_rate)
#                 if source_topic in self.imgs_info_dict.keys() and source_topic in sel_imgs and set_image_rate > .01:
#                     if self.imgs_info_dict[source_topic]['connected'] == False:
#                         self.msg_if.pub_warn("Got image topic: " + str(source_topic))
#                     self.imgs_info_dict[source_topic]['connected'] = True
#                     if self.enabled == True and self.state_str_msg == 'Detecting':

#                         # if self.imgs_info_dict[source_topic]['publishing'] == False:
#                         #     self.msg_if.pub_warn("Processing image topic: " + str(source_topic))

#                         # Check if time to publish
#                         delay_time = float(1) / set_image_rate 
#                         last_img_time = self.imgs_info_dict[source_topic]['last_img_time']
#                         current_time = nepi_utils.get_time()
#                         timer = round((current_time - last_img_time), 3)
#                         # if self.imgs_info_dict[source_topic]['publishing'] == False:   
#                         #     self.msg_if.pub_warn("Process Delay and Timer: " + str(delay_time) + " " + str(timer))
#                         if timer > delay_time: 
#                             self.imgs_info_dict[source_topic]['last_img_time'] = current_time


#                             stamp = image_msg.header.stamp
#                             timestamp = copy.deepcopy(float(stamp.to_sec()))

#                             ros_frame_id = image_msg.header.frame_id

#                             current_time = nepi_utils.get_time()
#                             latency = (current_time - timestamp )
#                             self.imgs_info_dict[source_topic]['get_latency_time'] = latency

#                             if self.use_last_image == False:
#                                 # process image for next time
#                                 use_cv2_img = nepi_img.rosimg_to_cv2img(image_msg)
#                             else: 
#                                 # Use last image to align with detection data
#                                 self.cv2_img_lock.acquire()
#                                 use_cv2_img = copy.deepcopy(self.imgs_info_dict[source_topic]['last_img'])
#                                 self.cv2_img_lock.release()
#                                 #self.msg_if.pub_info("Image updated is None: " + str(use_cv2_img is None))


#                             target_dict_list = copy.deepcopy(self.imgs_info_dict[source_topic]['target_dict_list'])
#                             if target_dict_list == None:
#                                 target_dict_list = []
#                             if use_cv2_img is not None:
#                                 # if self.imgs_info_dict[source_topic]['publishing'] == False:
#                                 #     self.msg_if.pub_warn("Will process img with shape: " + str(use_cv2_img.shape) )
#                                 # Symmetric targets overlay image, built from the
#                                 # same source image and the latest targets list.
#                                 success = self.processTargetsImage(source_topic,
#                                                             use_cv2_img,
#                                                             target_dict_list,
#                                                             timestamp = timestamp,
#                                 )

#                                 current_time = nepi_utils.get_time()
#                                 latency = (current_time - timestamp )
#                                 self.imgs_info_dict[source_topic]['pub_latency_time'] = latency

                                
#                             if self.use_last_image == True:
#                                 # process image for next time
#                                 self.imgs_info_dict[source_topic]['last_img'] = nepi_img.rosimg_to_cv2img(image_msg)

                           

#     def processFileImg(self, img_file,target_dict_list):   
#         source_topic = 'img_file'      
#         set_image_rate = copy.deepcopy(self.set_image_rate)
#         if set_image_rate > .01:
#             if self.enabled == True and self.state_str_msg == 'Detecting':


#                 # Check if time to publish
#                 delay_time = float(1) / set_image_rate 
#                 last_img_time = 0
#                 if 'last_img_time' in self.imgs_info_dict['img_file'].keys():
#                     last_img_time = self.imgs_info_dict['img_file']['last_img_time']
#                 current_time = nepi_utils.get_time()
#                 timer = round((current_time - last_img_time), 3)
#                 #self.msg_if.pub_warn("Delay and Timer: " + str(delay_time) + " " + str(timer))
#                 if timer > delay_time: 
#                     cv2_img = cv2.imread(img_file)
#                     if cv2_img is not None:
#                         self.imgs_info_dict['img_file']['last_img_time'] = current_time


#                         timestamp = nepi_utils.get_time()
#                         ros_frame_id = 'nepi_base'

#                         current_time = nepi_utils.get_time()
#                         latency = (current_time - timestamp)
#                         self.imgs_info_dict['img_file']['get_latency_time'] = latency

#                         if target_dict_list == None:
#                             target_dict_list = []           
#                         success = self.processTargetsImage(source_topic, 
#                                                     cv2_img, 
#                                                     target_dict_list, 
#                                                     timestamp = timestamp,  
#                                                     )

#                         current_time = nepi_utils.get_time()
#                         latency = (current_time - timestamp)
#                         self.imgs_info_dict['img_file']['pub_latency_time'] = latency                              

                           

#     def processTargetsImage(self,source_topic, cv2_img, target_dict_list, timestamp = None):
#         # Symmetric mirror of processDetImage for the targets_image data product.
#         # Post process image with overlays
#         if target_dict_list is not None:
#             cv2_img = self.apply_targets_overlay(source_topic, target_dict_list, cv2_img)

#             add_overlay_text_list = []

#             self.publishImgData(source_topic,
#                                 cv2_img,
#                                 timestamp = timestamp,
#                                 add_overlay_text_list = add_overlay_text_list,
#                                 )

#             if self.imgs_info_dict[source_topic]['targets_img_published'] == False:
#                 namespace = self.imgs_info_dict[source_topic]['pub_namespace']
#                 topic = os.path.join(namespace,'targets_image')
#                 self.msg_if.pub_warn('Published image topic: ' + topic)
#             self.imgs_info_dict[source_topic]['targets_img_published'] = True

#             # Save Image Data if needed
#             data_product = 'targets_image'
#             if self.save_data_if is not None:
#                 self.save_data_if.save(data_product,cv2_img,timestamp)
#         return True


#     def publishImgData(self, source_topic, cv2_img, encoding = "bgr8", timestamp = None, add_overlay_text_list = []):


#             if self.imgs_info_dict[source_topic]['publishing'] == False:
#                 pass
#             if self.imaging_enabled:

#                 if source_topic in self.imgs_info_dict.keys():

#                     # if self.imgs_info_dict[source_topic]['publishing'] == False:
#                     #     self.msg_if.pub_warn("Publishing image topic: " + str(source_topic))
#                     self.imgs_info_dict[source_topic]['publishing'] = True
#                     status_dict = copy.deepcopy(self.imgs_info_dict[source_topic]['status_dict'])
#                     if status_dict is not None:
#                         width_deg = status_dict['width_deg']
#                         height_deg = status_dict['height_deg']
#                         #self.msg_if.pub_warn('Using status provided image width,height degs: ' + str([width_deg,height_deg]))
#                     else:
#                         width_deg = 100
#                         height_deg = 70

#                     # try/finally: any raise between acquire and release (a
#                     # publish_cv2_img signature drift, a missing dict key) would
#                     # otherwise leave the lock held and wedge image publishing for
#                     # the life of the node. Degrade to a logged error instead.
#                     self.img_node_lock.acquire()
#                     try:
#                         img_if = self.img_node_dict[source_topic]['img_if']
#                         img_pub = self.img_node_dict[source_topic]['img_pub']

#                         img_if_ready = img_if.ready
#                         if img_if_ready == False:
#                             img_msg = nepi_img.cv2img_to_rosimg(cv2_img)
#                             nepi_sdk.publish_pub(img_pub,img_msg)
#                         else:
#                             img_if.publish_cv2_img(cv2_img,
#                                                 encoding = encoding,
#                                                 timestamp = timestamp,
#                                                 width_deg = width_deg,
#                                                 height_deg = height_deg,
#                                                 add_overlay_text_list = add_overlay_text_list
#                                                 )
#                     except Exception as e:
#                         self.msg_if.pub_warn("Failed to publish image for source: " + str(source_topic) + " : " + str(e))
#                     finally:
#                         self.img_node_lock.release()

#                     # img_pub = self.img_node_dict[source_topic]['img_pub']
#                     # img_msg = nepi_img.cv2img_to_rosimg(cv2_img)
#                     # nepi_sdk.publish_pub(img_pub,img_msg)

               

#     def apply_targets_overlay(self,source_topic, targets_dict_list, cv2_img):
#         cv2_targets_img = copy.deepcopy(cv2_img)
#         cv2_shape = cv2_img.shape
#         img_width = cv2_shape[1] 
#         img_height = cv2_shape[0] 

#         for i, target_dict in enumerate(targets_dict_list):
#             img_size = cv2_img.shape[:2]

#             # Overlay text data on OpenCV image
#             font = cv2.FONT_HERSHEY_DUPLEX
#             scale = 1.5e-3 - 0.1e-3 * math.ceil(max([img_height, img_width])/700)
#             fontScale, fontThickness  = nepi_img.get_optimal_font_dims(cv2_img,font_scale = scale, thickness_scale = scale) 
#             fontColor = (255, 255, 255)
#             fontColorBk = (0,0,0)
#             lineType = cv2.LINE_AA


#             ###### Apply Image Overlays and Publish Image ROS Message
#             # Overlay adjusted detection boxes on image 
#             class_name = target_dict['name']
#             xmin = target_dict['xmin_pixel']
#             ymin = target_dict['ymin_pixel']
#             xmax = target_dict['xmax_pixel']
#             ymax = target_dict['ymax_pixel']

#             if xmin <= 0:
#                 xmin = 5
#             if ymin <= 0:
#                 ymin = 5
#             if xmax >= img_size[1]:
#                 xmax = img_size[1] - 5
#             if ymax >= img_size[0]:
#                 ymax = img_size[0] - 5


#             bot_left_box = (xmin, ymin)
#             top_right_box = (xmax, ymax)


#             class_color = (0,0,127)
#             if class_name in self.classes_list:
#                 class_ind = self.classes_list.index(class_name)
#                 #self.msg_if.pub_warn("Got Class Index: " + str(class_ind))
#                 if class_ind < len(self.classes_colors_list):
#                     class_color = self.classes_colors_list[class_ind]

#             #self.msg_if.pub_warn("Got Class Color: " + str(class_color) + ' type: ' + str(type(class_color)) + " type: " + str(type(class_color[0])) )
#             line_thickness = 1 + math.ceil(max([img_height, img_width])/2000)
            

#             success = False
#             try:
#                 cv2.rectangle(cv2_targets_img, bot_left_box, top_right_box, class_color, thickness=line_thickness)
#                 success = True
#             except Exception as e:
#                 self.msg_if.pub_warn("Failed to create bounding box rectangle: " + str(e))

#             # Overlay text data on OpenCV image
#             if success == True:


#                 ## Overlay Text
#                 overlay_labels =  self.overlay_labels
#                 overlay_range_bearing =  self.overlay_range_bearing

#                 overlay_text = ""

#                 if overlay_labels:
#                     overlay_text = overlay_text + class_name + " "
#                 if overlay_range_bearing:
#                     rb_text = ''
#                     if target_dict['range_m'] != -999 and target_dict['range_m'] != '':
#                         rb_text = rb_text + str(round(target_dict['range_m'],1)) + 'm :'
#                     if target_dict['azimuth_deg'] != -999 and target_dict['elevation_deg'] != -999:
#                         rb_text = rb_text + str(round(target_dict['azimuth_deg'],1)) + 'deg '
#                         rb_text = rb_text + str(round(target_dict['elevation_deg'],1)) + 'deg '
#                     if len(rb_text) > 0:
#                         overlay_text = overlay_text + rb_text



#                 if len(overlay_text) > 0:
#                     text2overlay=overlay_text
#                     text_size = cv2.getTextSize(text2overlay, 
#                         font, 
#                         fontScale,
#                         fontThickness)
#                     #self.msg_if.pub_warn("Text Size: " + str(text_size))
#                     line_height = text_size[0][1]
#                     line_width = text_size[0][0]
#                     x_padding = int(line_height*0.4)
#                     y_padding = int(line_height*0.4)
                    
#                     center = bot_left_box[0] + int(( top_right_box[0] - bot_left_box[0]) / 2 )
#                     #bot_left_text = (xmin + (line_thickness * 2) + x_padding , ymin + line_height + (line_thickness * 2) + y_padding)
#                     bot_left_text = (center + x_padding , ymin - (line_thickness * 2) - y_padding)
#                     # Create Text Background Box
#                     #bot_left_box =  (bot_left_text[0] - x_padding , bot_left_text[1] + y_padding)
#                     bot_left_box =  ( center - x_padding, bot_left_text[1] + y_padding)
#                     top_right_box = (center + line_width + x_padding, bot_left_text[1] - line_height - y_padding )
#                     box_color = [0,0,0]

#                     try:
#                         cv2.rectangle(cv2_targets_img, bot_left_box, top_right_box, box_color , -1)
#                         cv2.putText(cv2_targets_img,text2overlay, 
#                             bot_left_text, 
#                             font, 
#                             fontScale,
#                             fontColor,
#                             fontThickness,
#                             lineType)
#                     except Exception as e:
#                         self.msg_if.pub_warn("Failed to apply overlay label text: " + str(e))

#                     # Start name overlays    
#                     x_start = int(img_width * 0.05)
#                     y_start = int(img_height * 0.05)


#         return cv2_targets_img



#     def targetsCb(self,msg):
#         self.connected = True
#         img_stamp = msg.source_timestamp
#         source_topic = msg.source_topic
#         current_time = nepi_utils.get_time()
#         targets_dict = nepi_sdk.convert_msg2dict(msg)
#         targets_dict_list = targets_dict['targets']
#         if source_topic in self.imgs_info_dict.keys():
#             self.imgs_info_dict[source_topic]['target_dict_list'] = targets_dict_list
#             self.imgs_info_dict[source_topic]['img_stamp'] = img_stamp
#             self.imgs_info_dict[source_topic]['last_targets_time'] = current_time
#         else:
#             if os.path.exists(source_topic):
#                 self.imgs_info_dict['img_file'] = dict()
#                 self.imgs_info_dict['img_file']['img_stamp'] = img_stamp
#                 self.imgs_info_dict['img_file']['last_targets_time'] = current_time
#                 self.processFileImg(source_topic,targets_dict_list)




#     def targetsStatusCb(self,msg):
#         self.last_status_time=nepi_utils.get_time()

#         self.status_msg = msg.process_status

#         self.name = self.status_msg.name
#         self.enabled = self.status_msg.enabled
#         self.state_str_msg = self.status_msg.msg_str
#         self.set_image_rate = self.status_msg.set_image_rate
#         self.use_last_image = self.status_msg.use_last_image


#         self.imaging_enabled = self.status_msg.image_pub_enabled
#         last_sel_imgs = copy.deepcopy(self.selected_source_topics)
#         self.selected_source_topics = self.status_msg.selected_sources
#         if last_sel_imgs != self.selected_source_topics:
#             self.msg_if.pub_warn("Updating selected images topics: " + str(self.selected_source_topics))
        
#         self.classes_list = msg.available_classes
#         self.selected_classes = msg.selected_classes

#         if len(self.classes_colors_list) != len(self.classes_list) :
#             #self.msg_if.pub_warn("Detector provided classes list: " + str(self.classes))
#             num_colors = len(self.classes_list)
#             self.classes_colors_list = nepi_img.create_bgr_jet_colormap_list(num_colors)
#             #self.msg_if.pub_warn("Created classes color list: " + str(self.classes_colors_list))

#     def _reloadProcesses(self):
#         success = False
#         if self.process_module is not None:
#             self.process_ready = False           
#             nepi_sdk.sleep(1)
#             process_busy = self.wait_on_process_busy()
#             if process_busy == True:
#                 self.msg_if.pub_info("Failed to load process. Process Busy: " + str(process_busy))
#             else:
#                 processes_controls_dict = copy.deepcopy(self.processes_controls_dict)
#                 try:
#                     importlib.reload(self.process_module)
#                     processes_dict = self.process_module.PROCESSES_DICT
#                     self.msg_if.pub_warn("################################")
#                     self.msg_if.pub_warn("Process Reloaded")
#                     self.msg_if.pub_warn("Updating Process Dictionaries")


#                     try:
#                         self.results_pub_msg = self.process_module.RESULTS_PUB_MSG
#                         self.has_results_pub = self.has_results_pub == True and self.results_pub_msg is not None
#                     except:
#                         self.has_results_pub = False

#                     available_processes = []
#                     for process_image_name in processes_dict.keys():
#                         available_processes.append(process_image_name)
#                         if process_image_name in processes_controls_dict.keys():

#                                 if 'controls_dict' in processes_dict[process_image_name].keys():
#                                     for control_name in processes_controls_dict[process_image_name].keys():
#                                         #self.msg_if.pub_warn("Updating Processes control_name: " + str([control_name]))
#                                         if control_name in processes_dict[process_image_name]['controls_dict'].keys():
#                                             control_value = processes_controls_dict[process_image_name][control_name]
#                                             nepi_controls.set_value(processes_dict[process_image_name]['controls_dict'], control_name, control_value )

#                     self.available_processes = available_processes
#                     self.processes_dict = processes_dict
#                     self.processes_functions_dict = self.process_module.FUNCTIONS_DICT
#                     #self.msg_if.pub_warn("Processes Functions Updated: " + str(self.processes_functions_dict))

#                     processes_controls_dict = dict()
#                     for process_image_name in processes_dict.keys():
#                         try:
#                             processes_controls_dict[process_image_name] = processes_dict[process_image_name]['controls_dict']
#                         except:
#                             pass




#                     #self.msg_if.pub_warn("")
#                     #self.msg_if.pub_warn("Processes Dict Updated: " + str(self.processes_dict))
#                     #self.msg_if.pub_warn("################################")
#                     if self.selected_process is None:
#                         self.selected_process = 'None'
#                     selected_process = self.selected_process    
#                     if selected_process == 'None' or selected_process not in self.available_processes:
#                         selected_process = self.available_processes[0]
#                         try:
#                             selected_process = self.process_module.DEFAULT_PROCESS
#                         except:
#                             pass
#                     self.selected_process = selected_process
#                     #self.msg_if.pub_warn("Process Selected: " + str(self.selected_process))
#                     success = self.set_selected_process(self.selected_process, check_updates = False)
#                 except Exception as e:
#                     self.msg_if.pub_warn("Failed to reload process class: " + str(e)) 
#         if success == False:
#             self.process_ready = False
#         return success


#     def _updateControlCb(self,msg):
#         self.msg_if.pub_info("Received control update msg: " + str(msg), log_name_list = self.log_name_list)
#         control_name = msg.name
#         # Same fix as ControlsIF._updateControlCb: apply_update_msg writes
#         # through the dict it is handed.
#         controls_dict = copy.deepcopy(self.controls_dict)
#         controls_dict = nepi_controls.apply_update_msg(controls_dict, msg)
#         control_value = nepi_controls.get_value(controls_dict, control_name )
#         self.set_control_value(control_name, control_value)



#     def shutdownCb(self):
#         pass

