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
from nepi_interfaces.msg import ProcessIFStatus
from nepi_interfaces.srv import ProcessIFStatusQuery, ProcessIFStatusQueryRequest, ProcessIFStatusQueryResponse


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeIF
from nepi_api.system_if import SaveDataIF
from nepi_api.utils_if import ControlsIF
from nepi_api.data_product_if_image import ColorImageIF



logger = nepi_sdk.sdk_logger

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
        default_process_rate = 10,

        has_image_rate = False,
        min_max_image_rates = [1,20],
        default_image_rate = 10,

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


class ProcessIF(NodeIF):

    #########################
    ## NODE IF Required
    ## Class Variables Updated by NodeIF Init Class
    #########################

    STATUS_MSG = ProcessStatus()
    DATA_MSG = None

    ID = 'process'
    DISPLAY_NAME = 'Process'
    DESCRIPTION = 'Process'

    IF_DICT = dict()

    #########################
    ## CUSTOM
    #########################
    msg_if = None
    node_if = None

    ready = False

    data_products = None

    process_status_msg = ProcessStatus()

    status_msg = ProcessStatus()
    status_msg_class = status_msg.__class__
    status_msg_type = status_msg_class.__name__

    admin_enabled = False

    process_image_if = None
    process_image_function = None
    process_image_controls_if = None
    last_process_image_controls_dict = dict()
    process_image_dict = dict()
    process_image_dict['controls_dict'] = dict()
    process_image_dict['data_dict'] = dict()
    process_image_dict['config_dict'] = dict()

    data_dict = dict()
    blank_data_dict = dict()

    settings_init_dict = copy.deepcopy(DEFAULT_SETTINGS_INIT_DICT)
    settings_dict = dict()
    settings_controls_dict = dict()

    has_controls = False
    controls_dict = dict()
    states_dict = dict()

    has_results = False    
    results_dict = None

    results_display_dict = None
    results_display_msg = DataStatus()


    has_results_pub = True
    results_pub_msg = None
    results_pub_msg_type = None
    results_pub_dict = None
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


    min_max_process_rates = [1,20]
    max_process_rate = 10.0
    min_max_image_rates = [1,20]
    max_image_rate = 10.0

    callback_dict = copy.deepcopy(BLANK_CALLBACK_DICT)
    config_dict = copy.deepcopy(BLANK_CONFIG_DICT)
    show_dict = copy.deepcopy(BLANK_SHOW_DICT)

    status_has_published = False
    last_status_time = 0

    process_if_dict = dict()

    debug = False

    #######################
    ### IF Initialization
    def __init__(self, 
                process_name = 'process',
                process_group = 'PROCESS',
                process_description = 'Process',
                process_module = None,
                process_if_dict = None,  
                process_image_if = None,            
                callback_dict = None,
                config_dict = None,
                settings_init_dict = None,
                show_dict = None,
                msg_if = None,
                node_if = None,
                save_data_if = None,
                ):


        ##############################  
        # Update Class Variables
        ##############################  

        self.process_name = nepi_utils.get_clean_name(process_name)


        if self.process_name is None or self.process_name == '':
            logger.log_error("Process Name Not Valid: " + str(process_name))
            return
        logger.log_debug("Using Process Name: " + self.process_name )
        self.namespace = nepi_sdk.create_namespace(nepi_sdk.get_node_namespace(),self.process_name)
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


        self.multi_source_enabled = self.config_dict['multi_source_enabled']
        self.auto_select_enabled = self.config_dict['auto_select_enabled']
        self.min_max_process_rates = self.config_dict['min_max_process_rates']
        self.max_process_rate = self.config_dict['default_process_rate']
        self.min_max_image_rates = self.config_dict['min_max_image_rates']
        self.max_image_rate = self.config_dict['default_image_rate']





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
        self.settings_dict = nepi_controls.create_controls_dict(self.settings_init_dict, self.process_name + '_settings', 'process_settings')

        self.settings_dict = nepi_controls.set_hidden(self.settings_dict,'enabled',self.config_dict['has_enable'] == False)

        has_sources = self.config_dict['has_sources']
        self.settings_dict = nepi_controls.set_value(self.settings_dict,'auto_select_enabled',self.config_dict['auto_select_enabled'])
        self.settings_dict = nepi_controls.set_hidden(self.settings_dict,'auto_select_enabled',self.config_dict['has_auto_select'] == False)

        if has_sources == True:
            self.settings_dict = nepi_controls.set_hidden(self.settings_dict,'select_source',self.multi_source_enabled == False)
            self.settings_dict = nepi_controls.set_hidden(self.settings_dict,'select_sources',self.multi_source_enabled == True)

        self.settings_dict = nepi_controls.set_bounds(self.settings_dict,'process_rate',self.min_max_process_rates)
        self.settings_dict = nepi_controls.set_value(self.settings_dict,'process_rate',self.max_process_rate)
        self.settings_dict = nepi_controls.set_hidden(self.settings_dict,'process_rate',self.config_dict['has_process_rate'] == False)
        
        self.settings_dict = nepi_controls.set_bounds(self.settings_dict,'image_rate',self.min_max_image_rates)
        self.settings_dict = nepi_controls.set_value(self.settings_dict,'image_rate',self.max_image_rate)
        self.settings_dict = nepi_controls.set_hidden(self.settings_dict,'image_rate',self.config_dict['has_image_rate'] == False)

        self.settings_dict = nepi_controls.set_hidden(self.settings_dict,'use_last_image',self.config_dict['has_use_last_image'] == False)

        self.process_status_msg.has_sources = has_sources
        self.process_status_msg.multi_source_enabled = self.multi_source_enabled
        
        self.updateSettingsValues()
        

        if process_module is None:
            logger.log_warn("No Process Module Provided" )
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



        if process_if_dict is not None:
            self.process_if_dict = process_if_dict

        self.process_image_if = process_image_if
        if self.process_image_if is not None:
            self.process_image_controls_if = self.process_image_if.options_controls_if
            # self.process_image_controls_if = ControlsIF( name = 'image_options',
            #                 namespace = None,
            #                 controls_name = 'image_options',
            #                 controls_display_name = 'Image Options',
            #                 controls_description = 'Custom Image Options',
            #                 controls_init_dict = dict(),
            #                 save_params = True,
            #                 controls_updated_callback = None, # if not None: Calls function with with control_name when msg is recieved, after changine controls_dict and publishing status
            #                 info_dict = dict(),
            #                 msg_if = None,
            #                 node_if = None,)
            # self.process_image_if.controls_options_topic = self.process_image_controls_if.get_namespace()
        ##############################    
        # Initialize Class Variables

        self.process_group = str(process_group)
        self.process_description = str(process_description)
        # Check Process Status Msg Type





        ##############################  
        # Setup NodeIF Updates
        ##############################  

        #######################
        # Sync System IF inputs to if_dict

        self.msg_if = msg_if
        self.IF_DICT['msg_if'] = msg_if
        self.node_if = node_if
        self.IF_DICT['node_if'] = node_if


        ##################
        if_info_dict = dict(
            description = self.DESCRIPTION,
            display_name = self.DISPLAY_NAME,
            type = 'system'
        )
        update_info_dict = dict()
        ##################        
        update_caps_dict = dict()
        ##################        
        update_config_control_dict = dict()
        ##################       
        update_node_controls_dict = dict()
        ##################
        update_stats_dict = dict()
        ##################
        update_rui_dict = show_dict
        ##################
        update_callback_dict =  callback_dict

        ###########
        # NODE IF SETUP

        # Call the parent class constructor
        super().__init__(id = self.ID,
                name = process_name,
                namespace = None,
                status_msg = copy.deepcopy(self.STATUS_MSG),
                data_msg = copy.deepcopy(self.DATA_MSG),
                info_dict = update_info_dict,
                caps_dict = update_caps_dict, 
                stats_dict = update_stats_dict, 
                rui_dict = update_rui_dict, 
                config_control_dict = update_config_control_dict, 
                node_controls_dict = update_node_controls_dict, 
                callback_dict = update_callback_dict,
                if_dict = dict(self.IF_DICT)
                )

        ready = self.wait_for_ready()

        # The process load logs through msg_if and publishes status, which only
        # exist once NodeIF is set up, and the params and results pub below
        # read what it loads
        success = self._reloadProcesses()
        if success == False:
            logger.log_warn("INITIAL PROCESS LOAD FAILED: " + str(self.processes_functions_dict) )
        else:
            logger.log_debug("Initial Process Load Succeeded: " + str(self.processes_functions_dict) )


       #######################
        # Add Services
        #######################

        self.sprocess_param_name = self.node_if_prefix + 'selected_process'
        self.processes_param_name = self.node_if_prefix + 'processes_dict'
        self.settings_param_name = self.node_if_prefix + 'settings_dict'

        params_dict = {
            'selected_process': {
                'name': 'selected_process',
                'namespace': self.namespace,
                'factory_val': self.selected_process
            },
            'processes_dict': {
                'name': 'processes_dict',
                'namespace': self.namespace,
                'factory_val': self.processes_controls_dict
            },
            'settings_dict': {
                'name': 'settings_dict',
                'namespace': self.namespace,
                'factory_val': self.settings_controls_dict
            }
        }
        

        #######################
        # Add Services
        #######################
        services_dict = {
            'process_if_status_query': {
                'namespace': self.namespace,
                'topic': 'process_if_status_query',
                'srv': ProcessIFStatusQuery,
                'req': ProcessIFStatusQueryRequest(),
                'resp': ProcessIFStatusQueryResponse(),
                'callback': self._processIFStatusQueryCb
            }
        }
        

        #######################
        # Add Pubs 
        #######################
        pubs_dict = {}


        if self.has_results_pub == True:
            pubs_dict['results_pub'] = {
                'namespace': self.namespace.replace('/' + process_name,''),
                'topic': process_name,
                'msg': self.results_pub_msg,
                'qsize': 1,
                'latch': False
            }
            self.results_pub_topic = self.namespace + '/' + process_name
            self.has_results_pub = True

        #######################
        # Add Subs 
        #######################
        subs_dict =  {
             'update_setting': {
                 'msg': UpdateControl,
                 'namespace': self.namespace,
                 'topic': 'update_setting',
                 'qsize': 5,
                 'callback': self._updateSettingCb
             },
             'set_process': {
                 'namespace': self.namespace,
                 'topic': 'set_process',
                 'msg': String,
                 'qsize': 10,
                 'callback': self._setProcessCb
             },
             'reload_process': {
                 'namespace': self.namespace,
                 'topic': 'reload_process',
                 'msg': Empty,
                 'qsize': 10,
                 'callback': self._reloadProcessesCb
             },
             'update_control': {
                 'msg': UpdateControl,
                 'namespace': self.namespace,
                 'topic': 'update_control',
                 'qsize': 5,
                 'callback': self._updateControlCb
             },
             'reset_controls': {
                 'msg': Empty,
                 'namespace': self.namespace,
                 'topic': 'reset_controls',
                 'qsize': 5,
                 'callback': self._resetControlsCb
             },
         }


        #####################
        self.add_params(params_dict)
        self.add_services(services_dict)
        self.add_pubs(pubs_dict)
        self.add_subs(subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################

        #########################
        # Update Status Message
        ##############################




        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is ProcessIF:
            self.start()


    def _processIFStatusQueryCb(self, _):
        # The response carries a ProcessIFStatus, which wraps the NodeIFStatus
        if_status_msg = ProcessIFStatus()
        if_status_msg.if_status = self.get_if_status_msg()
        return if_status_msg


    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):
        # A ProcessStatus status is the process status itself, built whole by get_process_status
        if isinstance(status_msg, ProcessStatus):
            status_msg = self.get_process_status()
        try:
            status_msg = self.process_module.update_status_msg(status_msg)
        except:
            pass
        return status_msg


    def initCb(self, do_updates = False):
        """Initialize or re-initialize interface state and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        self.msg_if.pub_debug("Initializing params" )
        if self.node_if is not None:
                processes_controls_dict =  self.node_if.get_param(self.processes_param_name)
                if processes_controls_dict is not None:
                    self.processes_controls_dict = processes_controls_dict
                self.msg_if.pub_debug("Init Processes Dict: " + str(self.processes_controls_dict))

                selected_process =  self.node_if.get_param(self.sprocess_param_name)
                if selected_process is not None:
                    if selected_process in self.processes_controls_dict.keys():
                        self.selected_process = selected_process

                settings_controls_dict =  self.node_if.get_param(self.settings_param_name)
                if settings_controls_dict is not None:
                    for setting_name in settings_controls_dict.keys():
                        if setting_name in self.settings_dict.keys():
                            setting_value = settings_controls_dict[setting_name]
                            self.settings_dict = nepi_controls.set_value(self.settings_dict,setting_name, setting_value)
                self.settings_controls_dict = nepi_controls.get_values_dict(self.settings_dict)

        if do_updates == True:

            success = self._reloadProcesses()
            if success == False:
                self.msg_if.pub_warn("PROCESS LOAD FAILED: " + str(self.processes_functions_dict) )
            else:
                self.msg_if.pub_debug("Processes Functions Updated: " + str(self.processes_functions_dict.keys()) )
                # self.msg_if.pub_debug("Processes Dict Updated: " + str(self.processes_dict) )
                # self.msg_if.pub_debug("Process Selected: " + str(self.selected_process) )
        self.publish_status()

    ###############################
    # Class Public Methods
    ###############################





    #######################
    # Class Public Methods
    #######################



    def get_available_processes(self):
        return self.available_processes
    
    
    def get_selected_process(self):
        return self.selected_process
    
    def set_selected_process(self, process_name, check_updates = True):
        success = False
        if process_name in self.available_processes:
            cur_process = copy.deepcopy(self.selected_process)
            if process_name != cur_process or check_updates == False:
                self.msg_if.pub_info("Process Selected: " + str(process_name) )
                self.process_ready = False
                self.selected_process = process_name
                self.publish_status()
                nepi_sdk.sleep(1)
                processes_dict = copy.deepcopy(self.processes_dict)
                [self.data_dict,controls_dict,self.results_display_dict,self.states_dict] = nepi_process.get_process_dicts(processes_dict,process_name)
                self.blank_data_dict = copy.deepcopy(self.data_dict)
                processes_controls_dict = copy.deepcopy(self.processes_controls_dict)
                if process_name in processes_controls_dict.keys():
                        if 'controls_dict' in processes_dict[process_name].keys():
                            for control_name in controls_dict.keys():
                                #self.msg_if.pub_debug("Updating Processes control_name: " + str([control_name]) )
                                if control_name in processes_controls_dict[process_name].keys():
                                    control_value = processes_controls_dict[process_name][control_name]
                                    nepi_controls.set_value(controls_dict, control_name, control_value )
    
                self.controls_dict = controls_dict
                # self.processes_controls_dict[process_name] = nepi_controls.get_values_dict(self.controls_dict)
                # self.node_if.set_param(self.processes_param_name, self.processes_controls_dict)
                # self.node_if.set_param(self.sprocess_param_name, self.selected_process)
                self.process_function = self.processes_functions_dict[process_name]
                nepi_sdk.sleep(1)
                success = True
                self.msg_if.pub_info("Process Ready: " + str(process_name) )
                self.msg_if.pub_debug("Process Dictionaries: " + str([self.data_dict.keys(),self.controls_dict.keys(),self.results_display_dict.keys(),self.states_dict.keys(),self.process_function]) )

        self.process_ready = self.selected_process in self.available_processes
        return success

    def set_enable(self, value):
        self.settings_dict = nepi_controls.set_value(self.settings_dict,'enabled',value)

    def get_enable(self):
        return nepi_controls.get_value(self.settings_dict,'enabled')


    def set_auto_select_enable(self, value):
        self.settings_dict = nepi_controls.set_value(self.settings_dict,'auto_select_enabled',value)

    def get_auto_select_enable(self):
        return nepi_controls.get_value(self.settings_dict,'auto_select_enabled')


    def set_available_source_topics(self, value):
        if value is not None:
            if isinstance(value, list) == False:
                value = [str(value)]
            self.available_source_topics = value

    def set_selected_source(self, value):
        if value is not None:
            if self.multi_source_enabled == False:
                if isinstance(value, list) == True:
                    if len(value) > 0:
                        value = str(value[0])
                nepi_controls.set_value(self.settings_dict,'select_source', value)
            else:
                nepi_controls.set_value(self.settings_dict,'select_sources', value) 

    def set_selected_sources(self, value):
        if value is not None:
            if isinstance(value, list) == False:
                value = [str(value)]
            if self.multi_source_enabled == False and len(value) > 0:
                try:
                    nepi_controls.set_value(self.settings_dict,'select_source', value[0])
                except:
                    pass
            else:
                nepi_controls.set_value(self.settings_dict,'select_sources', value) 



    def get_selected_sources(self):
        selected_sources = []
        if self.multi_source_enabled == False:
            selected_sources = [nepi_controls.get_value(self.settings_dict,'select_source')]
        else:
            selected_sources = nepi_controls.get_value(self.settings_dict,'select_sources')   
        return selected_sources

    def set_process_rate(self, value):
        self.settings_dict = nepi_controls.set_value(self.settings_dict,'process_rate',value)

    def get_process_rate(self):
        return nepi_controls.get_value(self.settings_dict,'process_rate')

    def set_image_rate(self, value):
        self.settings_dict = nepi_controls.set_value(self.settings_dict,'image_rate',value)

    def get_image_rate(self):
        return nepi_controls.get_value(self.settings_dict,'image_rate')

    def set_use_last_image(self, value):
        self.settings_dict = nepi_controls.set_value(self.settings_dict,'use_last_image',value)

    def get_use_last_image(self):
        return nepi_controls.get_value(self.settings_dict,'use_last_image')



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
        #self.msg_if.pub_debug("Waiting for process ready" )
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

        #self.msg_if.pub_debug("Waiting for process not busy" )
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.get_process_busy() == True and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        return self.get_process_busy()
    


    
    def set_connected_source_topics(self, connected_source_topics):
        if connected_source_topics is not None:
            topics = []
            for topic in connected_source_topics:
                topics.append(str(topic))
            if self.connected_source_topics != topics:
                self.connected_source_topics = topics
                self.publish_status()
            
    def set_image_pub_topics(self, image_pub_topics):
        if self.image_pub_topics != image_pub_topics:
            self.image_pub_topics = image_pub_topics
            self.publish_status()


    ##############################
    # IF and Topic Dict Support Methods



    def update_process_if_dict(self, update_dict):
        self.process_if_dict.update(update_dict)

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
        settings_dict = copy.deepcopy(self.settings_dict)
        value = None
        if settings_dict is not None:
            value = nepi_controls.get_value(settings_dict, setting_name)
        return value

    def get_settings_values(self):
        settings_dict = copy.deepcopy(self.settings_dict)
        settings_values_dict = None
        if settings_dict is not None:
            settings_values_dict = nepi_controls.get_values_dict(settings_dict)
        return settings_values_dict

    def set_setting_value(self, setting_name, update_value, index = None):
        if self.get_process_ready() == True:
            settings_dict = copy.deepcopy(self.settings_dict)
            if settings_dict is not None:
                if setting_name in settings_dict.keys():
                    settings_dict = nepi_controls.set_value(settings_dict, setting_name, update_value, index = index)
                    if settings_dict != self.settings_dict:
                        self.settings_dict = settings_dict
                        if self.node_if is not None:
                            self.settings_controls_dict = nepi_controls.get_values_dict(self.settings_dict)
                            self.node_if.set_param(self.settings_param_name, self.settings_controls_dict)
                else:
                    self.msg_if.pub_debug("Failed pub Updated Setting Options msg. Setting Name not In Settings.keys: " + str([setting_name,self.settings_controls_dict.keys()]), throttle_s = 5 )

    def set_setting_options(self, setting_name, update_options):
        if self.get_process_ready() == True:
            process_name = copy.deepcopy(self.selected_process)
            settings_dict = copy.deepcopy(self.settings_dict)
            if settings_dict is not None:
                if setting_name in settings_dict.keys():
                    settings_dict = nepi_controls.set_options(settings_dict, setting_name, update_options)
                    if settings_dict != self.settings_dict:
                        self.settings_dict = settings_dict
                        if self.node_if is not None:
                            self.settings_controls_dict = nepi_controls.get_values_dict(self.settings_dict)
                            self.node_if.set_param(self.settings_param_name, self.settings_controls_dict)
                else:
                    self.msg_if.pub_debug("Failed pub Updated Setting Options msg. Setting Name not In Settings.keys: " + str([setting_name,self.settings_controls_dict.keys()]), throttle_s = 5 )

    def get_setting_options(self, setting_name):
        settings_dict = copy.deepcopy(self.settings_dict)
        options = None
        if settings_dict is not None:
            options = nepi_controls.get_options(settings_dict, setting_name)
        return options

    def set_setting_bounds(self, setting_name, min_bound = None, max_bound = None):
        if self.get_process_ready() == True:
            process_name = copy.deepcopy(self.selected_process)
            settings_dict = copy.deepcopy(self.settings_dict)
            if settings_dict is not None:
                if setting_name in settings_dict.keys():
                    settings_dict = nepi_controls.set_bounds(settings_dict, setting_name, [min_bound, max_bound])
                    if settings_dict != self.settings_dict:
                        self.settings_dict = settings_dict
                        if self.node_if is not None:
                            self.settings_controls_dict = nepi_controls.get_values_dict(self.settings_dict)
                            self.node_if.set_param(self.settings_param_name, self.settings_controls_dict)
                else:
                    self.msg_if.pub_debug("Failed pub Updated Setting Options msg. Setting Name not In Settings.keys: " + str([setting_name,self.settings_controls_dict.keys()]), throttle_s = 5 )


    def get_setting_bounds(self, setting_name):
        settings_dict = copy.deepcopy(self.settings_dict)
        bounds = None
        if settings_dict is not None:
            bounds = nepi_controls.get_bounds(settings_dict, setting_name)
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
                        # try:
                        #     #self.msg_if.pub_debug("Updated Control Value: " + str([ control_name, update_value, self.controls_dict[control_name] ]), throttle_s = 5 )
                        #     pass
                        # except Exception as e:
                        #     self.msg_if.pub_debug("Failed pub Updated Control Value msg: " + str(e), throttle_s = 5 )
                else:
                    self.msg_if.pub_debug("Failed pub Updated Control Options msg. Control Name not In Controls.keys: " + str([control_name,controls_dict.keys()]), throttle_s = 5 )

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
                        # try:
                        #     self.msg_if.pub_debug("Updated Control Options: " + str([ control_name, update_options, self.controls_dict[control_name] ]), throttle_s = 5 )
                        # except Exception as e:
                        #     self.msg_if.pub_debug("Failed pub Updated Control Options msg: " + str(e), throttle_s = 5 )
                else:
                    self.msg_if.pub_debug("Failed pub Updated Control Options msg. Control Name not In Controls.keys: " + str([control_name,controls_dict.keys()]), throttle_s = 5 )

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
                    controls_dict = nepi_controls.set_bounds(controls_dict, control_name, [min_bound, max_bound])
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
                        # try:
                        #     self.msg_if.pub_debug("Updated Control Bounds: " + str([ control_name, [min_bound,max_bound], self.controls_dict[control_name] ]), throttle_s = 5 )
                        # except Exception as e:
                        #     self.msg_if.pub_debug("Failed pub Updated Control Bounds msg: " + str(e), throttle_s = 5 )
                else:
                    self.msg_if.pub_debug("Failed pub Updated Control Options msg. Control Name not In Controls.keys: " + str([control_name,controls_dict.keys()]), throttle_s = 5 )


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
        #self.msg_if.pub_debug("Processing results: " + str( [self.data_dict, self.controls_dict, self.results_display_dict, self.process_function]), throttle_s = 5 )
        #self.msg_if.pub_debug("Processing results with source topic: " + str( [source_topic]), throttle_s = 5 )
        last_results_dict = copy.deepcopy(self.results_dict)
        results_dict = None
        if self.enabled == True:
            process_ready = self.wait_for_process_ready()
            if process_ready == True:
                try:
                    [self.data_dict, self.controls_dict, self.states_dict, results_dict] = self.process_function(self.data_dict, self.controls_dict, self.states_dict, self.results_dict, self.process_if_dict)
                    self.results_dict = results_dict
                except Exception as e:
                    self.msg_if.pub_warn("Failed to process results: " + str(e), throttle_s = 10 )

                self.results_dict = results_dict
                if self.results_dict is not None:
                    self.results_display_dict = nepi_data.set_values(self.results_display_dict,self.results_dict)
                    #self.msg_if.pub_debug("Updated results display results: " + str( [self.results_display_dict, results_dict]), throttle_s = 5 )
                else:
                    self.results_display_dict = nepi_data.reset_values(self.results_display_dict)
                #self.msg_if.pub_debug("Processed results: " + str( [self.results_display_dict, results_pub_msg]), throttle_s = 5 )

                if self.has_results_pub == True and self.results_dict is not None:
                    self._publishResults(self.results_dict, source_topic)
            else:
                self.msg_if.pub_debug("Processes Not Ready", throttle_s = 10)
            
            
        else:
            #self.msg_if.pub_debug("Process Not Ready. Can't Pub Results", throttle_s = 5 )
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


    ###################
    # Process Results Image



    def process_results_image(self, cv2_img, cv2_img_status_dict):
        if self.process_image_function is None:
            results_cv2_img = cv2_img
        else:
            results_cv2_img = copy.deepcopy(cv2_img)
            process_image_dict = self.process_image_dict
            if self.process_image_controls_if is not None:
                try:
                    process_image_dict['controls_dict'] = nepi_controls.get_values_dict(self.process_image_controls_if.get_controls_dict())
                except:
                    pass
            try:
                [results_cv2_img, process_image_dict] = self.process_image_function(results_cv2_img, cv2_img_status_dict,
                                                               process_image_dict,
                                                               self.results_dict, self.process_image_if)

                
                try:
                    controls_dict = self.process_image_dict['controls_dict'] 
                    for control_name in controls_dict.keys():
                        self.process_image_controls_if.set_control_value(control_name, controls_dict[control_name])
                except:
                    pass

                self.process_image_dict['data_dict'] = process_image_dict['data_dict']
                self.process_image_dict['config_dict'] = process_image_dict['config_dict']
            except Exception as e:
                self.msg_if.pub_warn("Failed to process results image: " + str(e), throttle_s = 10 )
        return results_cv2_img



    ##################
    # Misc Functions






    def _resetControlsCb(self,msg):
        self.msg_if.pub_debug("Received reset controls msg" )
        if self.node_if is not None:

            self.process_ready = False
            self.node_if.reset_params([self.processes_param_name])
            nepi_sdk.sleep(1)
            processes_controls_dict =  self.node_if.get_param(self.processes_param_name)
            
            self.msg_if.pub_debug("Got Reset Params Dict: " + str(processes_controls_dict) )   

            process_name = self.selected_process
            processes_dict = copy.deepcopy(self.processes_dict)
            if process_name in processes_controls_dict.keys():
                
                if 'controls_dict' in processes_dict[process_name].keys():
                    for control_name in processes_controls_dict[process_name].keys():
                       
                        if control_name not in processes_dict[process_name]['controls_dict'].keys():
                            self.msg_if.pub_debug("control name not in Processes dict keys: " + str([control_name,processes_dict[process_name]['controls_dict']]) )
                        else:
                            control_value = processes_controls_dict[process_name][control_name]
                            #self.msg_if.pub_debug("Updating Processes control_name,value: " + str([control_name,control_value] )
                            processes_dict[process_name]['controls_dict'] = nepi_controls.set_value(processes_dict[process_name]['controls_dict'], control_name, control_value )
                    processes_values_dict = nepi_controls.get_values_dict(processes_dict[process_name]['controls_dict'])
                    self.msg_if.pub_debug("Updated Processes values dict: " + str(processes_values_dict) )
                    self.processes_controls_dict[process_name] = processes_values_dict
                    self.processes_dict = processes_dict
                    self.controls_dict = processes_dict[process_name]['controls_dict']
            self.data_dict = copy.deepcopy(self.blank_data_dict)
            self.process_ready = True

        self.publish_status()


    ##########################
    # Process Status 
    ########################

    def get_process_status(self):

        self.updateSettingsValues()

        self.process_status_msg.name = self.process_name
        self.process_status_msg.group = self.process_group
        self.process_status_msg.description = self.process_description

        self.process_status_msg.node_name = self.node_name
        self.process_status_msg.namespace = self.namespace

        # NodeIF keeps these in node_topics_dict (SYSTEM_TOPICS_STATUS_FIELDS keys save_data, save_config)
        self.process_status_msg.save_data_topic = self.node_topics_dict.get('save_data', dict()).get('topic', '')
        self.process_status_msg.config_topic = self.node_topics_dict.get('save_config', dict()).get('topic', '')
        self.process_status_msg.data_products = self.data_products

        # Run state. enabled is what the operator asked for and running is what
        # the owning node reports back; the RUI shows both so an enable that
        # could not take effect is visible rather than silently cosmetic.
        self.process_status_msg.enabled = self.enabled
        self.process_status_msg.running = self.running
        self.process_status_msg.state = self.state
        self.process_status_msg.msg_str = self.msg_str

        settings_dict = copy.deepcopy(self.settings_dict)
        self.process_status_msg.settings = nepi_controls.convert_controls_dict2msg(self.settings_dict)


        # NodeIF fills multi_source_enabled, max_process_rate, image_pub_enabled,
        # use_last_image and data_products by name, but this message replaces
        # its status, so they are set here too
        self.process_status_msg.multi_source_enabled = self.multi_source_enabled
        self.process_status_msg.auto_select_enabled = self.auto_select_enabled
        self.process_status_msg.auto_select_active = self.auto_select_active
        self.process_status_msg.available_source_topics = self.available_source_topics
        self.process_status_msg.selected_sources = self.selected_sources
        self.process_status_msg.sources_connected = self.sources_connected
        self.process_status_msg.connected_source_topics = self.connected_source_topics
        self.process_status_msg.sources_pub_namespaces = self.sources_pub_namespaces

        self.process_status_msg.min_max_process_rates = self.min_max_process_rates
        self.process_status_msg.set_process_rate = self.max_process_rate
        self.process_status_msg.max_process_rate = self.max_process_rate

        self.process_status_msg.has_process_reload = True
        self.process_status_msg.available_processes = self.available_processes
        self.process_status_msg.selected_process = self.selected_process
        self.process_status_msg.process_ready = self.get_process_ready()

        controls_dict = copy.deepcopy(self.controls_dict)

        if controls_dict is not None:
            has_controls = len(list(controls_dict.keys())) > 0
            self.process_status_msg.has_controls = has_controls
            if has_controls == True:
                controls_msg = nepi_controls.convert_controls_dict2msg(controls_dict)
                controls_msg.name = self.process_name
                controls_msg.description = self.process_description
                self.process_status_msg.controls = controls_msg
        #self.msg_if.pub_debug("Publishing status for process controls: " + str([ControlsStatus(),Control(),self.controls_msg]) , throttle_s = 10)
 
 
 
 
 
        results_display_dict = copy.deepcopy(self.results_display_dict)
        self.results_display_msg.name = self.process_name
        self.results_display_msg.description = self.process_description
        if results_display_dict is not None:
            has_results = len(list(results_display_dict.keys())) > 0
            self.process_status_msg.has_results = has_results
            if has_results == True:
                self.results_display_msg = nepi_data.convert_data_dict2msg(results_display_dict, name = self.process_name, description = self.process_description)
                self.process_status_msg.results = self.results_display_msg

        self.process_status_msg.has_results_pub = self.has_results_pub
        if self.has_results_pub == True:
            self.process_status_msg.results_pub_topic = self.results_pub_topic


        self.process_status_msg.image_pub_name = self.image_pub_name
        self.process_status_msg.image_pub_enabled = self.image_pub_enabled
        self.process_status_msg.use_last_image = self.use_last_image
        self.process_status_msg.min_max_image_rates = self.min_max_image_rates
        self.process_status_msg.set_image_rate = self.max_image_rate
        self.process_status_msg.image_pub_topics = self.image_pub_topics


        for key in self.show_dict.keys():
            if '_restricted' not in key:
                try:
                    restricted = self.show_dict[key + '_restricted'] and self.admin_enabled == True
                    show = self.show_dict[key] and restricted == False
                    setattr(self.process_status_msg, key, show)
                except:
                    pass
        return self.process_status_msg


    ###############################
    # Class Private Methods
    ###############################



    def _systemStatusCb(self,msg):
            self.active_nodes = msg.active_nodes
            self.active_topics = msg.active_topics
            self.active_topic_types = msg.active_topic_types
            self.active_services = msg.active_services
            
            namespace = msg.sys_debug_namespace
            enabled = msg.sys_debug_enabled
            self.debug = enabled and (namespace == 'ALL' or namespace == self.node_namespace)

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

    def _reloadProcessesCb(self,msg):
        self.msg_if.pub_debug("Got Process Reload Msg" )
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
                self.msg_if.pub_debug("Failed to load process. Process Busy: " + str(process_busy) )
            else:
                
                try:
                    importlib.reload(self.process_module)
                    processes_dict = self.process_module.PROCESSES_DICT
                    self.msg_if.pub_debug("################################" )
                    self.msg_if.pub_debug("Process Reloaded" )
                    self.msg_if.pub_debug("Updating Process Dictionaries" )

                    # Update Status Info
                    try:
                        status_msg = self.process_module.STATUS_MSG 
                        status_msg_class = status_msg.__class__()
                        status_msg_type = status_msg_class.__name__
                        self.status_msg = self.process_module.STATUS_MSG
                        self.status_msg_class = status_msg_class
                        self.status_msg_type = status_msg_type
                        self.node_topic_dict['status_msg_type'] = self.status_msg_type
                    except Exception as e:
                        pass

                    # Update Results Pub Info
                    try:
                        results_pub_msg = self.process_module.RESULTS_PUB_MSG 
                        results_pub_msg_class = results_pub_msg.__class__()
                        results_pub_msg_type = results_pub_msg_class.__name__
                    except Exception as e:
                        results_pub_msg = None
                        results_pub_msg_class = None
                        results_pub_msg_type = None
                    self.results_pub_msg = self.process_module.RESULTS_PUB_MSG
                    self.results_pub_msg_class = results_pub_msg_class
                    self.results_pub_msg_type = results_pub_msg_type
                    self.has_results_pub = self.has_results_pub == True and self.results_pub_msg is not None
                   
                        
                    # Update Avaialable Processes
                    available_processes = []
                    processes_controls_dict = copy.deepcopy(self.processes_controls_dict)
                    self.msg_if.pub_debug("Got Process Params Dict: " + str(processes_controls_dict) )                  
                    for process_name in processes_dict.keys():
                        available_processes.append(process_name)

                        if process_name in processes_controls_dict.keys():

                            if 'controls_dict' in processes_dict[process_name].keys():
                                for control_name in processes_controls_dict[process_name].keys():
                                    #self.msg_if.pub_debug("Updating Processes control_name: " + str([control_name]) )
                                    if control_name in processes_dict[process_name]['controls_dict'].keys():
                                        control_value = processes_controls_dict[process_name][control_name]
                                        processes_dict[process_name]['controls_dict'] = nepi_controls.set_value(processes_dict[process_name]['controls_dict'], control_name, control_value )
                        processes_controls_dict[process_name] = nepi_controls.get_values_dict(processes_dict[process_name]['controls_dict'])
                    self.msg_if.pub_debug("Updated Process Params Dict: " + str(processes_controls_dict) ) 
                    self.available_processes = available_processes
                    self.processes_dict = processes_dict
                    self.processes_functions_dict = self.process_module.FUNCTIONS_DICT
                    #self.msg_if.pub_debug("Processes Functions Updated: " + str(self.processes_functions_dict) )


                    #self.msg_if.pub_debug("" )
                    #self.msg_if.pub_debug("Processes Dict Updated: " + str(self.processes_dict) )
                    #self.msg_if.pub_debug("################################" )
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
                    #self.msg_if.pub_debug("Process Selected: " + str(self.selected_process) )
                    success = self.set_selected_process(self.selected_process, check_updates = False)

                    # Update Process Image Data
                    if success == True and self.process_image_if is not None:
                        try:
                            self.process_image_function = self.process_module.process_results_image
                        except:
                            pass

                        try:
                            process_image_dict = self.process_module.process_image_dict
                            last_process_image_controls_dict = process_image_dict.get('controls_dict', dict()).keys()
                            self.process_image_controls_if.remove_controls(last_process_image_controls_dict)
                            init_image_controls = process_image_dict.get('controls_dict', dict())
                            self.process_image_controls_if.add_controls(init_image_controls)
                        except:
                            pass
                        
                        try:
                            self.process_image_settings_dict = self.process_module.process_image_settings
                        except:
                            pass


                except Exception as e:
                    self.msg_if.pub_warn("Failed to reload process class: " + str(e) )
        if success == False:
            self.process_ready = False
        return success



    def updateSettingsValues(self):
            self.enabled = nepi_controls.get_value(self.settings_dict,'enabled') 
            self.auto_select_enabled = nepi_controls.get_value(self.settings_dict,'auto_select_enabled')
            self.selected_sources = self.get_selected_sources()      
            self.max_process_rate = nepi_controls.get_value(self.settings_dict,'process_rate')   
            self.min_max_process_rates = nepi_controls.get_bounds(self.settings_dict,'process_rate')
            self.max_image_rate = nepi_controls.get_value(self.settings_dict,'image_rate')            
            self.min_max_image_rates = nepi_controls.get_bounds(self.settings_dict,'image_rate')
            self.use_last_image = nepi_controls.get_value(self.settings_dict,'use_last_image') 

    def _updateSettingCb(self,msg):
        #self.msg_if.pub_debug("Received setting update msg: " + str(msg) )
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
                        self.msg_if.pub_debug("Failed to call settings_updated_callback: " + str(e), throttle_s = 1 )
                        pass


    def _updateControlCb(self,msg):
        self.msg_if.pub_debug("Received control update msg: " + str(msg)  )
        control_name = msg.name
        # Same fix as ControlsIF._updateControlCb: apply_update_msg writes
        # through the dict it is handed.

        controls_dict = copy.deepcopy(self.controls_dict)
        cur_value = nepi_controls.get_value(controls_dict, control_name)
        if cur_value is not None:
            controls_dict = nepi_controls.apply_update_msg(controls_dict, msg, debug = self.debug)
            update_value = nepi_controls.get_value(controls_dict, control_name)
            self.msg_if.pub_debug("Checking for update:current Change: " + str([control_name,update_value,cur_value]) )
            if cur_value != update_value:
                self.set_control_value(control_name, update_value)
                self.publish_status()
                control_value = self.get_control_value(control_name)
                if self.callback_dict['controls_updated_callback'] is not None:
                    try:
                        self.callback_dict['controls_updated_callback'](control_name, update_value)
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to call controls_updated_callback: " + str(e), throttle_s = 1) 
                        pass

    def _publishResults(self, results_dict, source_topic = ''):
        results_pub_dict = self.results_pub_dict
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
                data_header['source_topic'] = source_topic
                if 'timestamp' in results_dict.keys():
                    source_timestamp = results_dict['timestamp']
                else:
                    source_timestamp = process_timestamp
                data_header['source_timestamp'] = source_timestamp
                for key in data_header.keys():
                    if key in results_pub_dict['data_header'].keys():
                        results_pub_dict['data_header'][key] = data_header[key]




                pub_msg = None
                if self.results_pub_msg is not None and self.results_pub_msg_type is not None:
                    try:          
                        pub_msg = nepi_process.convert_results_pub_dict2msg(self.results_pub_msg, self.results_pub_msg_type, results_pub_dict)
                    except:
                        pass

                    if self.node_if is not None and pub_msg is not None:
                        self.node_if.publish_pub(self.node_if_prefix + 'results_pub', pub_msg) 
                    else:
                        self.msg_if.pub_debug("Failed to Pub. Results Msg is None: " + str([results_pub_dict, self.results_pub_msg]), throttle_s = 10 )
                        pass



