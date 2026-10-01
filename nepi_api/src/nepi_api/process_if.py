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

DEFAULT_SETTINGS_INIT_DICT = dict(

    enabled = {
        'type': 'Toggle', 'value': True,
        'display_name': 'Enabled', 'description': 'Enabled', 'display_hidden': True},

    auto_select_enabled = {
        'type': 'Toggle', 'value': False,
        'display_name': 'Enable Auto Select', 'description': 'Auto Select Source', 'display_hidden': True},

    select_source = {
        'type': 'Selection', 'default': 'None', 'options': ['None'],
        'display_name': 'Select Source', 'description': 'Select Source', 'display_hidden': True},

    select_sources = {
        'type': 'Selections', 'default': 'None', 'options': ['None'],
        'display_name': 'Select Sources', 'description': 'Select Sources', 'display_hidden': True},

    process_rate = {
        'type': 'FloatSlider', 'value': 10, 'bounds': [1,20], 'round_value': 3,
        'display_name': 'Max Process Rate',
        'description': 'Max Process Rate', 'display_hidden': True},

    image_rate = {
        'type': 'FloatSlider', 'value': 10, 'bounds': [1,20], 'round_value': 3,
        'display_name': 'Max Image Rate',
        'description': 'Max Image Rate', 'display_hidden': True},

    use_last_image = {
        'type': 'Toggle', 'value': False,
        'display_name': 'Use Last Image', 'description': 'Use Last Image', 'display_hidden': True},

)




BLANK_CONFIG_DICT = dict(
        has_sources = False,
        multi_source_enabled = False,
        has_auto_select = True,
        auto_select_enabled = True,
        source_filter_list = [],
        selected_sources = [],
        
        has_enable = True,
        has_process_rate = False,
        min_max_process_rates = [1,20],
        has_image_rate = False,
        min_max_image_rates = [1,20],
        has_use_last_image = False,


        has_process_pub = True,
        has_process_enable = True,
        has_process_reload = False,

        has_results_pub = True,

        has_save_data = True,
        has_config = True,

        has_image_pub = True,




        has_status_pub = True,
        throttle_status_sec = 0.1
    )

BLANK_CALLBACK_DICT = dict(
        process_update_callback = None,
        settings_updated_callback = None,
        controls_updated_callback = None,
    )

BLANK_SHOW_DICT = dict(        
        show_settings = True,
        show_settings_restricted = False,
        show_process = True,
        show_process_restricted = False,
        show_reload = True,
        show_reload_restricted = False,
        show_data = True,
        show_data_restricted = False,
        show_controls = True,
        show_controls_restricted = False,
        show_results = True,
        show_results_restricted = False,
        show_stats = True,
        show_stats_restricted = False,
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

    settings_init_dict = copy.deepcopy(DEFAULT_SETTINGS_INIT_DICT)
    settings_controls_dict = dict()
    settings_msg = ControlsStatus()

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


    multi_source_enabled = False
    auto_select_enabled = False
    auto_select_active = False
    available_source_topics = []
    selected_sources = []
    sources_connected = []
    connected_source_topics = []
    sources_pub_namespaces =[]


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
    image_pub_topics = []
    image_pub_enabled = True


    min_max_process_rates = [0.1,100]
    set_process_rate = 10.0
    min_max_image_rates = [1,20]
    set_image_rate = 10.0

    callback_dict = copy.deepcopy(BLANK_CALLBACK_DICT)
    config_dict = copy.deepcopy(BLANK_CONFIG_DICT)
    show_dict = copy.deepcopy(BLANK_SHOW_DICT)

    status_has_published = False
    last_status_time = 0
    #######################
    ### IF Initialization
    def __init__(self, 
                process_name = 'process',
                process_group = 'PROCESS',
                process_description = 'Process',
                process_module = None,              
                callback_dict = None,
                config_dict = None,
                settings_init_dict = None,
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
                    if key in self.callback_dict.keys():
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
        
        self.min_max_process_rates = self.config_dict['min_max_process_rates']
        self.min_max_image_rates = self.config_dict['min_max_image_rates']


 


        if show_dict is not None:
            try:
                for key in show_dict.keys():
                    if key in self.show_dict.keys():
                        self.show_dict[key] = show_dict[key]
            except:
                pass


        if settings_init_dict is not None:
            try:
                self.settings_init_dict |= settings_init_dict
            except:
                pass
        self.settings_controls_dict = nepi_controls.create_controls_dict(self.settings_init_dict)
        self.settings_controls_dict = nepi_controls.set_hidden(self.settings_controls_dict,'enabled',self.config_dict['has_enable'] == False)

        has_sources = self.config_dict['has_sources']
        self.settings_controls_dict = nepi_controls.set_value(self.settings_controls_dict,'auto_select_enabled',self.config_dict['auto_select_enabled'])
        self.settings_controls_dict = nepi_controls.set_hidden(self.settings_controls_dict,'auto_select_enabled',self.config_dict['has_auto_select'] == False)
        multi_source_enabled = self.config_dict['has_sources']
        if has_sources == True:
            self.settings_controls_dict = nepi_controls.set_hidden(self.settings_controls_dict,'select_source',multi_source_enabled == False)
            self.settings_controls_dict = nepi_controls.set_hidden(self.settings_controls_dict,'select_sources',multi_source_enabled == True)

        self.settings_controls_dict = nepi_controls.set_bounds(self.settings_controls_dict,'process_rate',self.min_max_process_rates)
        self.settings_controls_dict = nepi_controls.set_hidden(self.settings_controls_dict,'process_rate',self.config_dict['has_process_rate'] == False)
        
        self.settings_controls_dict = nepi_controls.set_bounds(self.settings_controls_dict,'image_rate',self.min_max_image_rates)
        self.settings_controls_dict = nepi_controls.set_hidden(self.settings_controls_dict,'image_rate',self.config_dict['has_image_rate'] == False)

        self.settings_controls_dict = nepi_controls.set_hidden(self.settings_controls_dict,'use_last_image',self.config_dict['has_use_last_image'] == False)

        self.updateSettingsValues()

        self.status_msg.has_sources = has_sources
        self.status_msg.multi_source_enabled = has_sources
        

          

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
        self.settings_param_name = self.node_if_prefix + 'settings_controls_dict'
        PARAMS_DICT = {
            self.processes_param_name: {
                'name': 'processes_dict',
                'namespace': self.namespace,
                'factory_val': self.processes_controls_dict
            },
            self.settings_param_name: {
                'name': 'settings_controls_dict',
                'namespace': self.namespace,
                'factory_val': self.settings_controls_dict
            }
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
            self.node_if_prefix + 'update_setting': {
                'msg': UpdateControl,
                'namespace': self.namespace,
                'topic': 'update_setting',
                'qsize': 5,
                'callback': self._updateSettingCb
            },
            self.node_if_prefix + 'set_process': {
                'namespace': self.namespace,
                'topic': 'set_process',
                'msg': String,
                'qsize': 10,
                'callback': self._setProcessCb
            },
            self.node_if_prefix + 'reload_process': {
                'namespace': self.namespace,
                'topic': 'reload_process',
                'msg': Empty,
                'qsize': 10,
                'callback': self._reloadProcessesCb
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
    


    def updateSettingsValues(self):
            self.enabled = nepi_controls.get_value(self.settings_controls_dict,'enabled') 
            self.auto_select_enabled = nepi_controls.get_value(self.settings_controls_dict,'auto_select_enabled')
            self.selected_sources = self.get_selected_sources()      
            self.set_process_rate = nepi_controls.get_value(self.settings_controls_dict,'process_rate')            
            self.min_max_process_rates = nepi_controls.get_bounds(self.settings_controls_dict,'process_rate')
            self.set_image_rate = nepi_controls.get_value(self.settings_controls_dict,'image_rate')            
            self.min_max_image_rates = nepi_controls.get_bounds(self.settings_controls_dict,'image_rate')
            self.use_last_image = nepi_controls.get_value(self.settings_controls_dict,'use_last_image') 


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
        return success

    def set_enable(self, value):
        self.settings_controls_dict = nepi_controls.set_value(self.settings_controls_dict,'enabled',value)

    def get_enable(self):
        return nepi_controls.get_value(self.settings_controls_dict,'enabled')


    def set_auto_select_enable(self, value):
        self.settings_controls_dict = nepi_controls.set_value(self.settings_controls_dict,'auto_select_enabled',value)

    def get_auto_select_enable(self):
        return nepi_controls.get_value(self.settings_controls_dict,'auto_select_enabled')


    def set_available_source_topics(self, value):
        if value is not None:
            if isinstance(value, list) == False:
                value = [str(value)]
            self.available_source_topics = value

    def set_selected_source(self, value):
        if value is not None:
            multi_source_enabled = self.config_dict['has_sources']
            if multi_source_enabled == False:
                if isinstance(value, list) == True:
                    if len(value) > 0:
                        value = str(value[0])
                nepi_controls.set_value(self.settings_controls_dict,'select_source', value)
            else:
                nepi_controls.set_value(self.settings_controls_dict,'select_sources', value) 

    def set_selected_sources(self, value):
        if value is not None:
            if isinstance(value, list) == False:
                value = [str(value)]
            multi_source_enabled = self.config_dict['has_sources']
            if multi_source_enabled == False and len(value) > 0:
                try:
                    nepi_controls.set_value(self.settings_controls_dict,'select_source', value[0])
                except:
                    pass
            else:
                nepi_controls.set_value(self.settings_controls_dict,'select_sources', value) 



    def get_selected_sources(self):
        selected_sources = []
        multi_source_enabled = self.config_dict['has_sources']
        if multi_source_enabled == False:
            selected_sources = [nepi_controls.get_value(self.settings_controls_dict,'select_source')]
        else:
            selected_sources = nepi_controls.get_value(self.settings_controls_dict,'select_sources')   
        return selected_sources

    def set_max_process_rate(self, value):
        self.settings_controls_dict = nepi_controls.set_value(self.settings_controls_dict,'process_rate',value)

    def get_max_process_rate(self):
        return nepi_controls.get_value(self.settings_controls_dict,'process_rate')

    def set_max_image_rate(self, value):
        self.settings_controls_dict = nepi_controls.set_value(self.settings_controls_dict,'image_rate',value)

    def get_max_image_rate(self):
        return nepi_controls.get_value(self.settings_controls_dict,'image_rate')

    def set_use_last_image(self, value):
        self.settings_controls_dict = nepi_controls.set_value(self.settings_controls_dict,'use_last_image',value)

    def get_use_last_image(self):
        return nepi_controls.get_value(self.settings_controls_dict,'use_last_image')



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

    def get_data_dict(self):
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
    # Settings Dict Functions



    def get_setting_value(self, setting_name):
        settings_controls_dict = copy.deepcopy(self.settings_controls_dict)
        value = None
        if settings_controls_dict is not None:
            value = nepi_controls.get_value(settings_controls_dict, setting_name)
        return value

    def get_settings_values(self):
        settings_controls_dict = copy.deepcopy(self.settings_controls_dict)
        settings_values_dict = None
        if settings_controls_dict is not None:
            settings_values_dict = nepi_controls.get_values_dict(settings_controls_dict)
        return settings_values_dict

    def set_setting_value(self, setting_name, update_value, index = None):
        if self.get_process_ready() == True:
            settings_controls_dict = copy.deepcopy(self.settings_controls_dict)
            if settings_controls_dict is not None:
                if setting_name in settings_controls_dict.keys():
                    settings_controls_dict = nepi_controls.set_value(settings_controls_dict, setting_name, update_value, index = index)
                    if settings_controls_dict != self.settings_controls_dict:
                        self.settings_controls_dict = settings_controls_dict
                        if self.node_if is not None and settings_controls_dict != self.settings_controls_dict:
                            self.settings_controls_dict = settings_controls_dict
                            self.node_if.set_param(self.settings_param_name, self.settings_controls_dict)
                else:
                    self.msg_if.pub_info("Failed pub Updated Setting Options msg. Setting Name not In Settings.keys: " + str([setting_name,settings_controls_dict.keys()]), throttle_s = 5)

    def set_setting_options(self, setting_name, update_options):
        if self.get_process_ready() == True:
            process_name = copy.deepcopy(self.selected_process)
            settings_controls_dict = copy.deepcopy(self.settings_controls_dict)
            if settings_controls_dict is not None:
                if setting_name in settings_controls_dict.keys():
                    settings_controls_dict = nepi_controls.set_options(settings_controls_dict, setting_name, update_options)
                    if settings_controls_dict != self.settings_controls_dict:
                        self.settings_controls_dict = settings_controls_dict
                        if self.node_if is not None and settings_controls_dict != self.settings_controls_dict:
                            self.settings_controls_dict = settings_controls_dict
                            self.node_if.set_param(self.settings_param_name, self.settings_controls_dict)
                else:
                    self.msg_if.pub_info("Failed pub Updated Setting Options msg. Setting Name not In Settings.keys: " + str([setting_name,settings_controls_dict.keys()]), throttle_s = 5)

    def get_setting_options(self, setting_name):
        settings_controls_dict = copy.deepcopy(self.settings_controls_dict)
        options = None
        if settings_controls_dict is not None:
            options = nepi_controls.get_options(settings_controls_dict, setting_name)
        return options

    def set_setting_bounds(self, setting_name, min_bound = None, max_bound = None):
        if self.get_process_ready() == True:
            process_name = copy.deepcopy(self.selected_process)
            settings_controls_dict = copy.deepcopy(self.settings_controls_dict)
            if settings_controls_dict is not None:
                if setting_name in settings_controls_dict.keys():
                    settings_controls_dict = nepi_controls.set_bounds(settings_controls_dict, setting_name, min_bound = min_bound, max_bound = max_bound)
                    if settings_controls_dict != self.settings_controls_dict:
                        self.settings_controls_dict = settings_controls_dict
                        if self.node_if is not None and settings_controls_dict != self.settings_controls_dict:
                            self.settings_controls_dict = settings_controls_dict
                            self.node_if.set_param(self.settings_param_name, self.settings_controls_dict)
                else:
                    self.msg_if.pub_info("Failed pub Updated Setting Options msg. Setting Name not In Settings.keys: " + str([setting_name,settings_controls_dict.keys()]), throttle_s = 5)


    def get_setting_bounds(self, setting_name):
        settings_controls_dict = copy.deepcopy(self.settings_controls_dict)
        bounds = None
        if settings_controls_dict is not None:
            bounds = nepi_controls.get_bounds(settings_controls_dict, setting_name)
        return bounds
    



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
            controls_values_dict = nepi_controls.get_values_dict(controls_dict)
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
                        # self.publish_status()
                        if process_name in self.processes_dict.keys():
                            self.processes_dict[process_name]['controls_dict'] = self.controls_dict
                            processes_controls_dict = copy.deepcopy(self.processes_controls_dict)
                            processes_controls_dict[process_name] = nepi_controls.get_values_dict(self.processes_dict[process_name]['controls_dict'])
                            if self.node_if is not None and processes_controls_dict != self.processes_controls_dict:
                                self.processes_controls_dict = processes_controls_dict
                                self.node_if.set_param(self.processes_param_name, self.processes_controls_dict)
                        try:
                            #self.msg_if.pub_warn("Updated Control Value: " + str([ control_name, update_value, self.controls_dict[control_name] ]), throttle_s = 5)
                            pass
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
                        #self.publish_status()

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

    def get_status_msg(self):
        self.updateSettingsValues()

        self.status_msg.name = self.process_name
        self.status_msg.group = self.process_group
        self.status_msg.description = self.process_description

        self.status_msg.node_name = self.node_name
        self.status_msg.namespace = self.namespace

        self.status_msg.save_data_topic = self.save_data_topic
        self.status_msg.config_topic = self.config_topic

        # Run state. enabled is what the operator asked for and running is what
        # the owning node reports back; the RUI shows both so an enable that
        # could not take effect is visible rather than silently cosmetic.
        self.status_msg.enabled = self.enabled
        self.status_msg.running = self.running
        self.status_msg.state = self.state
        self.status_msg.msg_str = self.msg_str

        settings_controls_dict = copy.deepcopy(self.settings_controls_dict)
        self.settings_msg = nepi_controls.update_status_msg(self.settings_msg, settings_controls_dict)
        self.status_msg.settings = self.settings_msg


        self.status_msg.auto_select_enabled = self.auto_select_enabled
        self.status_msg.auto_select_active = self.auto_select_active
        self.status_msg.available_source_topics = self.available_source_topics
        self.status_msg.selected_sources = self.selected_sources
        self.status_msg.sources_connected = self.sources_connected
        self.status_msg.connected_source_topics = self.connected_source_topics
        self.status_msg.sources_pub_namespaces = self.sources_pub_namespaces

        self.status_msg.min_max_process_rates = self.min_max_process_rates
        self.status_msg.set_process_rate = self.set_process_rate

        self.status_msg.has_process_reload = True
        self.status_msg.available_processes = self.available_processes
        self.status_msg.selected_process = self.selected_process
        self.status_msg.process_ready = self.get_process_ready()

        controls_dict = copy.deepcopy(self.controls_dict)
        if controls_dict is not None:
            has_controls = len(list(controls_dict.keys())) > 0
            self.status_msg.has_controls = has_controls
            if has_controls == True:
                self.controls_msg = nepi_controls.update_status_msg(self.controls_msg, controls_dict)
                self.status_msg.controls = self.controls_msg

        results_display_dict = copy.deepcopy(self.results_display_dict)
        if results_display_dict is not None:
            has_results = len(list(results_display_dict.keys())) > 0
            self.status_msg.has_results = has_results
            if has_results == True:
                self.results_display_msg = nepi_data.update_status_msg(self.results_display_msg, results_display_dict)
                self.status_msg.results = self.results_display_msg

        self.status_msg.has_results_pub = self.has_results_pub
        if self.has_results_pub == True:
            self.status_msg.results_pub_topic = self.results_pub_topic


        self.status_msg.image_pub_name = self.image_pub_name
        self.status_msg.min_max_image_rates = self.min_max_image_rates
        self.status_msg.set_image_rate = self.set_image_rate
        self.status_msg.image_pub_topics = self.image_pub_topics


        for key in self.show_dict.keys():
            if '_restricted' not in key:
                try:
                    restricted = self.show_dict[key + '_restricted'] and self.admin_enabled == True
                    show = self.show_dict[key] and restricted == False
                    setattr(self.status_msg, key, show)
                except:
                    pass
        return self.status_msg

    def publish_status(self):

        status_msg = self.get_status_msg()

        ###########

        if self.node_if is not None:
            cur_time = nepi_utils.get_time()
            timer = cur_time - self.last_status_time
            if self.config_dict['has_status_pub'] == True and timer >= self.config_dict['throttle_status_sec']:
                self.last_status_time = nepi_utils.get_time()

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
            settings_controls_dict =  self.node_if.get_param(self.settings_param_name)
            if settings_controls_dict is not None:
                try:
                    settings_values_dict = nepi_controls.get_values_dict(settings_controls_dict)
                    self.settings_controls_dict = nepi_controls.set_values(self.settings_controls_dict, settings_values_dict)
                except:
                    pass



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
        self.set_enable(enabled)

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


    def _updateSettingCb(self,msg):
        #self.msg_if.pub_info("Received setting update msg: " + str(msg), log_name_list = self.log_name_list)
        setting_name = msg.name
        # Same fix as SettingsIF._updateSettingCb: apply_update_msg writes
        # through the dict it is handed.

        settings_controls_dict = copy.deepcopy(self.settings_controls_dict)
        cur_value = nepi_controls.get_value(settings_controls_dict, setting_name)
        if cur_value is not None:
            settings_controls_dict = nepi_controls.apply_update_msg(settings_controls_dict, msg)
            update_value = nepi_controls.get_value(settings_controls_dict, setting_name )
            if cur_value != update_value:
                self.set_setting_value(setting_name, update_value)
                self.publish_status()
                setting_value = self.get_setting_value(setting_name)
                if self.callback_dict['settings_updated_callback'] is not None:
                    try:
                        self.callback_dict['settings_updated_callback'](setting_name, update_value)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to call settings_updated_callback: " + str(e), throttle_s = 1) 
                        pass


    def _updateControlCb(self,msg):
        #self.msg_if.pub_info("Received control update msg: " + str(msg), log_name_list = self.log_name_list)
        control_name = msg.name
        # Same fix as ControlsIF._updateControlCb: apply_update_msg writes
        # through the dict it is handed.

        controls_dict = copy.deepcopy(self.controls_dict)
        cur_value = nepi_controls.get_value(controls_dict, control_name)
        if cur_value is not None:
            controls_dict = nepi_controls.apply_update_msg(controls_dict, msg)
            update_value = nepi_controls.get_value(controls_dict, control_name )
            if cur_value != update_value:
                self.set_control_value(control_name, update_value)
                self.publish_status()
                control_value = self.get_control_value(control_name)
                if self.callback_dict['controls_updated_callback'] is not None:
                    try:
                        self.callback_dict['controls_updated_callback'](control_name, update_value)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to call controls_updated_callback: " + str(e), throttle_s = 1) 
                        pass

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

