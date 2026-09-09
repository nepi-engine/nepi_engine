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
import numpy as np
import copy
import threading
import importlib

os.environ['EGL_PLATFORM'] = 'surfaceless'   # Ubuntu 20.04+
import open3d as o3d

import cv2


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_states
from nepi_sdk import nepi_triggers
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data
from nepi_sdk import nepi_process

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64


from nepi_interfaces.msg import Control, ControlsStatus, SettingsStatus, UpdateControl, MgrSystemStatus
from nepi_interfaces.msg import Datum, DataStatus

from nepi_interfaces.msg import ProcessStatus

from nepi_interfaces.msg import SaveDataRate, SaveDataStatus, FilenameConfig
from nepi_interfaces.srv import SaveDataCapabilitiesQuery, SaveDataCapabilitiesQueryRequest, SaveDataCapabilitiesQueryResponse


from nepi_interfaces.msg import Transform, TransformStatus

from nepi_interfaces.msg import SystemState, SystemStates, SystemStatesStatus
from nepi_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_interfaces.msg import SystemTrigger

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import  NodeClassIF






#########################################
# Controls IF Class
#########################################


class ControlsIF:
    
    msg_if = None
    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'controls_'
    namespace = ''

    controls_name = 'controls'
    controls_display_name = ''
    controls_description = ''
    controls_dict = dict()
    controls_status_msg = ControlsStatus()

    controls_node_pubs_dict = None
    controls_node_subs_dict = None
    controls_ready = False

    status_has_published = False

    controls_updated_callback = None, # if not None: Calls function with with control_name when msg is recieved, after changine controls_dict and publishing status

    pub_status = True
    save_params = True

    #######################
    ### IF Initialization
    def __init__(self, 
                controls_name = 'controls',
                controls_display_name = 'Controls',
                controls_description = 'Controls',
                controls_init_dict = dict(),
                controls_updated_callback = None, # if not None: Calls function with with control_name when msg is recieved, after changine controls_dict and publishing status
                pub_status = True,
                save_params = True,
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
        self.msg_if.pub_info("Starting IF Initialization Controls " + str(controls_name), log_name_list = self.log_name_list)

        # Create Namespace
        self.controls_name = nepi_utils.get_clean_name(controls_name)
        if controls_name is None or controls_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(controls_name)) 
            return
        self.msg_if.pub_info("Using Name: " + self.controls_name)
        # Built from the sanitized self.controls_name, not the raw argument -- the
        # namespace must match the name reported in ControlsStatus.
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,self.controls_name)
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

        ##############################    
        # Initialize Class Variables

        self.controls_display_name = str(controls_display_name)
        self.controls_description = str(controls_description)
        self.controls_dict = nepi_controls.create_controls_dict(controls_init_dict)
        self.controls_status_msg = nepi_controls.create_status_msg(self.controls_name, self.controls_display_name, self.controls_description)

        self.controls_updated_callback = controls_updated_callback

        self.pub_status = pub_status
        self.save_params = save_params

        # if source_callback_dict is not None:
        #     for key in source_callback_dict.keys():
        #         self.source_callback_dict[key] = source_callback_dict[key]
      


        ##############################   
        ## Node Setup
        # Configs Config Dict ####################
        # The persisted value stays the string-valued settings dict it has
        # always been, under the param key it has always used. The controls
        # dict is derived state, so no deployed config file needs migrating.
        if self.save_params == True:

            # Configs Config Dict ####################
            self.CONFIGS_DICT = {
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
            self.PARAMS_DICT = {
                self.node_if_prefix + 'controls_dict': {
                    'name': 'controls_dict',
                    'namespace': self.namespace,
                    'factory_val': self.controls_dict
                },
            }
        else:
            self.CONFIGS_DICT = None
            self.PARAMS_DICT = None

        # Publishers Config Dict ####################
        if pub_status == False:
            self.controls_node_pubs_dict = dict()
        else:
            self.controls_node_pubs_dict = {
                self.node_if_prefix + 'status_pub': {
                    'namespace': self.namespace,
                    'topic': 'status',
                    'msg': ControlsStatus,
                    'qsize': 1,
                    'latch': True
                }
            }



        # Subscribers Config Dict ####################
        self.controls_node_subs_dict = {
            #####################
            # Control Subs
            ####################
            self.node_if_prefix + 'update_control': {
                'msg': UpdateControl,
                'namespace': self.namespace,
                'topic': 'update_control',
                'qsize': 5,
                'callback': self._updateControlCb
            },
        }

    
        
        if node_if is None:
            self.config_topic = self.namespace
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.controls_node_pubs_dict,
                            subs_dict = self.controls_node_subs_dict,
                            log_name_list = [],
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()
        else:
            self.node_if_shared = True
            try:
                self.node_if = node_if
                self.node_if.register_pubs(self.controls_node_pubs_dict)
                self.node_if.register_subs(self.controls_node_subs_dict)
                self.node_if.add_params(self.PARAMS_DICT)
                nepi_sdk.sleep(1)
            except Exception as e:
                self.msg_if.pub_info("Failed to register pubs and subs: " + str(e))
                return


        ##############################
        # Start updater controls

        if pub_status == True:
            nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

        ##############################
        # Complete Initialization
        self.controls_ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete")
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_controls_ready_state(self):
        """Return the ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.controls_ready

    def wait_for_controls_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.controls_ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.controls_ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.controls_ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.controls_ready  

    def get_namespace(self):
        """Return the fully-resolved ROS namespace this controls set publishes under.

        This is create_namespace(node_namespace, controls_name) -- the same
        namespace the status publisher and every set_*_control_value subscriber
        are registered on, so it is what a remote subscriber should use.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.namespace
    
   
    def unregister(self):
        """Shut down this settings interface and release its ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)
                self.PUBS_DICT = None
                
        time.sleep(1)
        try:
            self.node_if = None
            self.controls_ready = False
            success = True
        except Exception as e:
            self.msg_if.pub_warn("Failed to unregister:  " + str(e))
        return success


    ##################
    # Controls Functions
    def get_controls_dict(self):
        controls_dict = copy.deepcopy(self.controls_dict)
        return controls_dict

    def get_control_value(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        value = nepi_controls.get_value(controls_dict, control_name)
        return value

    def set_control_value(self, control_name, update_value, index = None):
        controls_dict = copy.deepcopy(self.controls_dict)
        if controls_dict is not None:
            if control_name in controls_dict.keys():
                controls_dict = nepi_controls.set_value(controls_dict, control_name, update_value, index = index)
                if controls_dict != self.controls_dict:
                    self.controls_dict = controls_dict
                    self.publish_status()
                    if self.controls_updated_callback is not None:
                        self.controls_updated_callback(control_name)
                    self.save_params_dict()


    def reset_control_value(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.reset_value(controls_dict, control_name)
        self.controls_dict = controls_dict

    def reset_control_values(self):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.reset_values(controls_dict)
        self.controls_dict = controls_dict

    def get_control_labels(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        labels = nepi_controls.get_labels(controls_dict, control_name)
        return labels

    def set_control_labels(self, control_name, labels):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.set_labels(controls_dict, control_name, labels)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()



    def get_control_options(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        options = nepi_controls.get_options(controls_dict, control_name)
        return options

    def set_control_options(self, control_name, options):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.set_options(controls_dict, control_name, options)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()


    def get_control_bounds(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        bounds = nepi_controls.get_bounds(controls_dict, control_name)
        return bounds


    def set_control_min_bound(self, control_name, min_bound = None):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.set_min_bound(controls_dict, control_name, min_bound = min_bound)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()

    def set_control_max_bound(self, control_name, max_bound = None):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.set_min_bound(controls_dict, control_name, max_bound = max_bound)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()


    def set_control_bounds(self, control_name, bounds = [None,None]):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.set_bounds(controls_dict, control_name, bounds)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()



    ##################
    # Display Functions

    def get_control_display_name(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        display_name = nepi_controls.get_display_name(controls_dict, control_name)
        return display_name

    def set_control_display_name(self, control_name, display_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.set_display_name(controls_dict, control_name, display_name)
        self.controls_dict = controls_dict


    def get_control_description(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        description = nepi_controls.get_description(controls_dict, control_name)
        return description

    def set_control_description(self, control_name, description):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.set_description(controls_dict, control_name, description)
        self.controls_dict = controls_dict

    def get_control_hidden(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        hidden = nepi_controls.get_hidden(controls_dict, control_name)
        return hidden

    def set_control_hidden(self, control_name, hidden):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.set_hidden(controls_dict, control_name, hidden)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()


    def get_control_disabled(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        disabled = nepi_controls.get_disabled(controls_dict, control_name)
        return disabled

    def set_control_disabled(self, control_name, disabled):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.set_disabled(controls_dict, control_name, disabled)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()


    def get_control_display_order(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        order = nepi_controls.get_display_order(controls_dict, control_name)
        return order

    def set_control_display_order(self, control_name, update_order = 0):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.set_display_order(controls_dict, control_name, update_order)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()

    def move_control_display_top(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.move_control_top(controls_dict, control_name)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()

    def move_control_display_bottom(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.move_control_bottom(controls_dict, control_name)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()

    def move_control_display_up(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.move_control_up(controls_dict, control_name)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()

    def move_control_display_down(self, control_name):
        controls_dict = copy.deepcopy(self.controls_dict)
        self.controls_dict = nepi_controls.move_control_down(controls_dict, control_name)
        if self.controls_dict != controls_dict:
            self.publish_status()
            self.save_params_dict()
    



    def save_params_dict(self):
        controls_dict = copy.deepcopy(self.controls_dict)
        params_dict = nepi_controls.get_params_dict(controls_dict)
        if self.node_if is not None and self.save_params == True and controls_dict is not None:
            param_name = self.node_if_prefix + 'controls_dict'
            self.node_if.set_param(param_name, params_dict)


    ##################
    # Misc Functions

    def publish_status(self, status_msg = None):
        ###########
        if self.pub_status == True:
            controls_dict = copy.deepcopy(self.controls_dict)
            self.controls_status_msg = nepi_controls.update_status_msg(self.controls_status_msg, controls_dict)
            self.controls_status_msg.config_topic = self.config_topic
            if self.node_if is not None:
                if self.status_has_published == False:
                    self.msg_if.pub_warn("Publishing Status: " + str(self.controls_status_msg))
                    self.status_has_published = True
                self.node_if.publish_pub(self.node_if_prefix + 'status_pub', self.controls_status_msg) 
            return

    def init(self, do_updates = False):
        """Initialize or re-initialize controls from the parameter server and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            # Prefixed key, matching how the param is registered and how
            # set_control_value()/set_control_options()/set_control_bounds()
            # write it back. get_param() returns None for a name it does not
            # know, so the unprefixed name wiped the controls dict on every
            # config init, reset and factory reset, leaving ControlsStatus with
            # empty control lists and the RUI with an empty controls box.

            param_name = self.node_if_prefix + 'controls_dict'
            controls_params_dict = nepi_controls.get_params_dict(self.controls_dict)

            controls_params_dict = self.node_if.get_param(param_name)
            if controls_params_dict is not None:
                for control_name in controls_params_dict.keys():
                    control_value = controls_params_dict[control_name]
                    if control_value is not None:
                        self.controls_dict = nepi_controls.set_value(self.controls_dict, control_name, control_value)

        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset parameters to their last-saved (user) values and reinitialize.

        Calls node_if.reset_params() to reload the user configuration tier, then
        reinitializes from the param server.
        """
        self.controls_dict = nepi_controls.reset_values(self.controls_dict)
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset parameters to factory defaults and reinitialize.

        Calls node_if.factory_reset_params() to restore factory values, then
        reinitializes from the param server.
        """
        self.controls_dict = nepi_controls.reset_values(self.controls_dict)
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Factory resetting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
        self.init(do_updates = True)

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)


    def _updateControlCb(self,msg):
        self.msg_if.pub_info("Received control update msg: " + str(msg), log_name_list = self.log_name_list)
        control_name = msg.name
        controls_dict = nepi_controls.apply_update_msg(self.controls_dict, msg)
        control_value = nepi_controls.get_value(controls_dict, control_name )
        self.set_value(control_name, control_value)
    

    def _setHiddenValueCb(self,msg):
            self.set_hidden(msg.name, msg.value)

    def _setControlsHiddenCb(self,msg):
            self.sets_hidden(msg.value)

    def _setOrderValueCb(self,msg):
            self.set_display_order(msg.name, msg.value)

    def _setOrderTopCb(self,msg):
            self.move_control_display_top(msg.name)

    def _setOrderBottomCb(self,msg):
            self.move_control_display_bottom(msg.name)

    def _setOrderDownCb(self,msg):
            self.move_control_display_down(msg.name)

    def _publishStatusCb(self,timer):
            self.publish_status()





#######################################
# SettingsIF
#######################################

# The device-side settings contract, unchanged by the move onto nepi_controls.
# A driver (or system_mgr, for the system config) describes its settings as
# string-valued dicts and hands SettingsIF two functions to read and write them.
# SettingsIF converts that form to and from a nepi_controls controls dict, which
# is what it publishes -- so the capability report that used to require a
# SettingsCapabilitiesQuery round trip now rides the status message.


def SET_NONE_SETTINGS_FUNCTION(setting):
    return False, "No settings update function available", dict()

def GET_NONE_SETTINGS_FUNCTION():
    return dict()


class SettingsIF:
    """Publishes a node's settings as a nepi_settings settings set.

    A device's settings are settings: named, typed, bounded values with a
    factory, default and set tier. This interface keeps the string-valued
    settings dict contract that drivers implement, holds the live state as a
    nepi_settings settings dict, and publishes it as a SettingsStatus message
    on '<namespace>/settings/status'. The capability information that the
    retired SettingsCapabilitiesQuery service used to return (type, options,
    bounds, default) is carried in that status message.

    ROS interface, all under '<namespace>/<settings_name>':
        status              (SettingsStatus, latched)  the settings and their capabilities
        update_setting_value      (UpdateSetting)            change one setting
        reset_settings      (Empty)                    restore last-saved values
    """

    # Class Vars ####################

    msg_if = None
    ready = False
    namespace = '~'

    node_if = None
    node_if_shared = False
    config_topic = ''

    node_if_prefix = ''

    settings_dict_values = None
    getCapSettingsFunction = None
    getSettingsFunction = None
    setSettingFunction = None
    callback_arg = None

    settings_dict = dict()
    settings_status_msg = None

    settings_name = 'settings'


    save_params = True

    #######################
    ### IF Initialization
    def __init__(self, 
                namespace = None,
                settings_name = 'settings',
                getSettingsFunction=None, 
                setSettingFunction=None, 
                callback_arg = None,
                save_params = True,
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
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting Settings IF Initialization Processes", log_name_list = self.log_name_list)
        

        #############################

        self.callback_arg = callback_arg


        # Create Namespace
        settings_name = nepi_utils.get_clean_name(settings_name)
        if settings_name is None or settings_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(settings_name)) 
            return
        self.settings_name = settings_name
        self.msg_if.pub_info("Using Settings Name: " + settings_name)
        # An explicit namespace lets a caller place this interface somewhere other than
        # its own node namespace. system_mgr passes the base namespace so the system
        # config settings own the global '<base>/settings' namespace the RUI subscribes
        # to, and the device IFs pass their device namespace ('<node>/idx', '<node>/ptx',
        # ...) so each device type's settings sit under its own device namespace.
        if namespace is None:
            namespace = self.node_namespace
        self.namespace = nepi_sdk.create_namespace(namespace,settings_name)

        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_' 
       
        self.save_params = save_params


        if getSettingsFunction is None:
            self.getSettingsFunction = GET_NONE_SETTINGS_FUNCTION
        else:
            self.getSettingsFunction = getSettingsFunction


        if setSettingFunction is None:
            self.setSettingFunction = SET_NONE_SETTINGS_FUNCTION
        else:
            self.setSettingFunction = setSettingFunction
        
        # Build the settings dict. The factory and default tiers come from the
        # device's factory settings (or a cap setting's own declared default);
        # the set tier is seeded from whatever the device currently reports.

        if self.callback_arg is None:
            settings_dict = self.getSettingsFunction()
        else:
            settings_dict = self.getSettingsFunction(self.callback_arg)

        if settings_dict is not None:
            self.settings_dict = settings_dict
            #self.msg_if.pub_warn("Get Settings function returned Settings Dict: " + str(settings_dict), log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_warn("Setting update function returned None Settings Dict", log_name_list = self.log_name_list)

        self.settings_status_msg = nepi_settings.create_status_msg(
                                    self.settings_name, 'Settings', 'Device Settings')

        ##############################  
        # Create NodeClassIF Class  

        # Configs Config Dict ####################
        # The persisted value stays the string-valued settings dict it has
        # always been, under the param key it has always used. The controls
        # dict is derived state, so no deployed config file needs migrating.
        if self.save_params == True:
            self.CONFIGS_DICT = {
                'init_callback': self._initCb,
                'reset_callback': self._resetCb,
                'factory_reset_callback': self._factoryResetCb,
                'init_configs': True,
                'namespace': self.namespace
            }
            self.PARAMS_DICT = {
                self.node_if_prefix + 'settings': {
                    'name': 'settings',
                    'namespace': self.namespace,
                    'factory_val': self.settings_dict_values
                }
            }
        else:
            self.CONFIGS_DICT = None
            self.PARAMS_DICT = None

        # Services Config Dict ####################
        # No capabilities service. Everything _capabilitiesHandler used to
        # return is published in the status message.
        self.SRVS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.node_if_prefix + 'status_pub': {
                'namespace': self.namespace,
                'msg': SettingsStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            self.node_if_prefix + 'update_setting': {
                'msg': UpdateControl,
                'namespace': self.namespace,
                'topic': 'update_setting',
                'qsize': 5,
                'callback': self._updateSettingCb
            },
            self.node_if_prefix + 'reset_settings': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset_settings',
                'qsize': 5,
                'callback': self._resetSettingsCb,
                'callback_args': None
            }
        }

        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if_shared = True
            self.node_if = node_if
            if self.PARAMS_DICT is not None:
               self.node_if.add_params(self.PARAMS_DICT) 
            self.node_if.register_pubs(self.PUBS_DICT)
            self.node_if.register_subs(self.SUBS_DICT)
        else:
            self.config_topic = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = self.SRVS_DICT,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                                                )

   

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status() 
    
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

  
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the SettingsIF.

        Returns:
            bool: True if initialization completed successfully, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the SettingsIF is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace used by this SettingsIF.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace
    
    def unregister(self):
        """Shut down this settings interface and release its ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)
                self.PUBS_DICT = None


    def get_settings_dict(self):
        """Return the current settings in the string-valued device settings form.

        Returns:
            dict: setting name -> {'name', 'type', 'value'}.
        """
        return nepi_controls.gets_values_dict(self.getSettingsDict())

    def publish_status(self):
        """Build and publish the settings SettingsStatus message.

        Refreshes the settings dict from the device (its live values, and its
        capability report if it provides a getCapSettingsFunction), then
        publishes. No-ops if node_if is None.
        """
        if self.node_if is None:
            return
        self.settings_status_msg.config_topic = self.config_topic
        settings_dict = self.getSettingsDict()

        self.settings_status_msg = nepi_settings.update_status_msg(self.settings_status_msg, settings_dict)
        #self.msg_if.pub_warn("Publishing Status Msg: " + str([settings_dict,self.settings_status_msg]), log_name_list = self.log_name_list, throttle_s = 10)
        self.node_if.publish_pub(self.node_if_prefix + 'status_pub', self.settings_status_msg)



    def update_setting_value(self,setting_name, setting_value, index = None):
        """Apply a single setting update using the registered setSettingFunction.

        Compares the incoming value against the value currently held in the
        settings dict and only calls down to the device if it has changed.
        Optionally persists the settings dict to the ROS param server and
        publishes an updated status message.

        Args:
            setting (dict): Setting dictionary with 'name', 'type', and 'value' keys.
            do_updates (bool, optional): If True, publish status after a successful
                update. Defaults to True.
            update_param (bool, optional): If True, write the updated settings to the
                ROS param server. Defaults to True.

        Returns:
            bool: True if the setting was successfully applied, False otherwise.
        """
        success = False
        
        if self.setSettingFunction is None:
            self.msg_if.pub_debug("Settings updates ignored. No settings update function defined ", log_name_list = self.log_name_list)
            return success

        settings_dict = self.getSettingsDict()
        if setting_name not in settings_dict.keys():
            self.msg_if.pub_warn("Ignoring update for unknown setting: " + str(setting_name), log_name_list = self.log_name_list)
            return

        if nepi_controls.get_clean_value(settings_dict, setting_name, setting_value) is None:
            self.msg_if.pub_warn("Setting update rejected as invalid: " + str([setting_name, setting_value, index]), log_name_list = self.log_name_list)

        current_value = nepi_controls.get_value(self.settings_dict, setting_name)
        if current_value == setting_value:
            #self.msg_if.pub_warn("Setting allready set: " + str([current_value, setting_value]), log_name_list = self.log_name_list)
            return True

        self.msg_if.pub_info("Updating setting : " + str([setting_name,setting_value]), log_name_list = self.log_name_list)
        try:
            if self.callback_arg is None:
                [success, msg, settings_dict] = self.setSettingFunction(setting_name, setting_value)
            else:
                [success, msg, settings_dict] = self.setSettingFunction(setting_name, setting_value, self.callback_arg)
        except Exception as e:
            self.msg_if.pub_warn("setSettingFunction callback failed: " + str(e), log_name_list = self.log_name_list)
            success = False
            msg = str(e)
        if settings_dict is not None:
            self.settings_dict = settings_dict
        else:
            self.msg_if.pub_warn("Setting update function returned None Settings Dict", log_name_list = self.log_name_list)
        #self.msg_if.pub_warn("setSettingFunction returned: " + str(self.settings_dict), log_name_list = self.log_name_list)
 
        self.publish_status()
        if success == True:
            self.msg_if.pub_info("Setting Updated: " + str([setting_name, setting_value]), log_name_list = self.log_name_list)
            self.save_params_dict()
        else:
            self.msg_if.pub_warn("Setting update failed: " + str([setting_name, setting_value]) + " : " + str(msg), log_name_list = self.log_name_list)
        return success


    def init(self, do_updates = True):
        """Load settings from the ROS param server and optionally apply them.

        Reads the persisted string-valued settings dict from the param server,
        seeds the settings dict default and set tiers from it, and if
        do_updates is True pushes each stored value down to the device before a
        final status publish.

        Args:
            do_updates (bool, optional): If True, apply all stored settings to the
                hardware after loading. Defaults to True.
        """
        settings_params_dict = nepi_controls.get_params_dict(self.settings_dict)
        if self.node_if is not None and self.save_params == True:
            settings_params_dict = self.node_if.get_param(self.node_if_prefix + 'settings')            
        #self.msg_if.pub_warn("Init start settings params: " + str(settings_params_dict), log_name_list = self.log_name_list)

        if self.callback_arg is None:
            init_settings_dict = self.getSettingsFunction()
        else:
            init_settings_dict = self.getSettingsFunction(self.callback_arg)

        #self.msg_if.pub_warn("Init start settings dict: " + str(init_settings_dict), log_name_list = self.log_name_list)
        if type(init_settings_dict) == dict:
            if type(settings_params_dict) == dict:
                for setting_name in settings_params_dict.keys():
                    if setting_name in init_settings_dict.keys():
                        setting_value = settings_params_dict[setting_name]
                        init_settings_dict = nepi_controls.set_value(init_settings_dict, setting_name, setting_value, setting_value)
        else:
            init_settings_dict = dict()

        self.settings_dict = init_settings_dict


        #self.msg_if.pub_warn("Init update settings dict: " + str(init_settings_dict), log_name_list = self.log_name_list)

        if do_updates == True:
            settings_dict = self.getSettingsDict()
            for setting_name in settings_dict.keys():
                setting_value = nepi_controls.get_value(settings_dict, setting_name)
                try:
                    if self.callback_arg is None:
                        [success, msg, settings_dict] = self.setSettingFunction(setting_name, setting_value)
                    else:
                        [success, msg, settings_dict] = self.setSettingFunction(setting_name, setting_value, self.callback_arg)
                    if settings_dict is not None:
                        self.settings_dict = settings_dict
                        #self.msg_if.pub_warn("Init Got Settings update: " + str(settings_dict[setting_name]), log_name_list = self.log_name_list)
                    else:
                        self.msg_if.pub_warn("Init Settings update function returned None Settings Dict", log_name_list = self.log_name_list)

                except Exception as e:
                    self.msg_if.pub_warn("Init setSettingFunction callback failed: " + str(e), log_name_list = self.log_name_list)
                    success = False
                    msg = str(e)
            self.save_params_dict()
            init_settings_values_dict = nepi_controls.get_values_dict(self.settings_dict)
            self.msg_if.pub_warn("Init Settings Complete: " + str(init_settings_values_dict), log_name_list = self.log_name_list)
        self.publish_status()

    def reset(self):
        """Reset settings to their last-saved (user) values and reinitialize.

        Restores the default tier of every setting, reloads the user
        configuration tier via node_if.reset_params(), then reapplies.
        """
        self.settings_dict = nepi_controls.reset_values(self.settings_dict)
        if self.node_if is not None and self.save_params == True and self.node_if_shared == False:
            self.node_if.reset_params()
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset settings to factory defaults and reinitialize.

        Restores the factory tier of every setting (factory -> default -> set),
        restores the factory param values via node_if.factory_reset_params(),
        then reapplies.
        """
        self.settings_dict = nepi_controls.reset_values(self.settings_dict)
        if self.node_if is not None and self.save_params == True and self.node_if_shared == False:
            self.node_if.factory_reset_params()
        self.init(do_updates = True)


    ###############################
    # Class Private Methods
    ###############################

    def getSettingsDict(self):
        return copy.deepcopy(self.settings_dict)


    def save_params_dict(self):
        settings_dict = copy.deepcopy(self.settings_dict)
        params_dict = nepi_controls.get_params_dict(settings_dict)
        if self.node_if is not None and self.save_params == True and settings_dict is not None:
            self.node_if.set_param(self.node_if_prefix + 'settings', params_dict)

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.reset()

    def _factoryResetCb(self, do_updates = True):
        self.factory_reset()

    def _resetSettingsCb(self, msg):
        self.reset()

    def _publishStatusCb(self, timer):
        self.publish_status()


    def _updateSettingCb(self,msg):
            #self.msg_if.pub_info("Received setting update msg: " + str(msg), log_name_list = self.log_name_list)
            setting_name = msg.name
            settings_dict = copy.deepcopy(self.settings_dict)
            cur_value = nepi_controls.get_value(settings_dict, setting_name )
            settings_dict = nepi_controls.apply_update_msg(settings_dict, msg)
            setting_value = nepi_controls.get_value(settings_dict, setting_name )
            #self.msg_if.pub_info("Sending Updated Val from/to: " + str([cur_value, setting_value]), log_name_list = self.log_name_list)
            self.update_setting_value(setting_name, setting_value)






#########################################
# Data IF Class
#########################################


class DataIF:
    
    msg_if = None
    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'data_'
    namespace = ''

    data_name = 'data'
    data_display_name = ''
    data_description = ''
    data_dict = dict()
    data_status_msg = DataStatus()

    data_node_pubs_dict = None
    data_node_subs_dict = None
    data_ready = False

    status_has_published = False

    pub_status = True

    #######################
    ### IF Initialization
    def __init__(self, 
                data_name = 'data',
                data_display_name = 'Data',
                data_description = 'Data',
                data_init_dict = dict(),
                pub_status = True,
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
        self.msg_if.pub_info("Starting IF Initialization Data " + str(data_name), log_name_list = self.log_name_list)

        # Create Namespace
        self.data_name = nepi_utils.get_clean_name(data_name)
        if data_name is None or data_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(data_name)) 
            return
        self.msg_if.pub_info("Using Name: " + self.data_name)
        # Built from the sanitized self.data_name, not the raw argument -- the
        # namespace must match the name reported in DataStatus.
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,self.data_name)
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

        ##############################    
        # Initialize Class Variables

        self.data_display_name = str(data_display_name)
        self.data_description = str(data_description)
        self.data_dict = nepi_data.create_data_dict(data_init_dict)
        self.data_status_msg = nepi_data.create_status_msg(self.data_name, self.data_display_name, self.data_description)

        self.pub_status = pub_status


        ##############################   
        ## Node Setup
        # Configs Config Dict ####################
        # The persisted value stays the string-valued settings dict it has
        # always been, under the param key it has always used. The data
        # dict is derived state, so no deployed config file needs migrating.

        self.CONFIGS_DICT = None
        self.PARAMS_DICT = None

        # Publishers Config Dict ####################
        if pub_status == False:
            self.data_node_pubs_dict = dict()
        else:
            self.data_node_pubs_dict = {
                self.node_if_prefix + 'status_pub': {
                    'namespace': self.namespace,
                    'topic': 'status',
                    'msg': DataStatus,
                    'qsize': 1,
                    'latch': True
                }
            }



        # Subscribers Config Dict ####################
        self.data_node_subs_dict = dict()

    
        
        if node_if is None:
            self.config_topic = self.namespace
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.data_node_pubs_dict,
                            subs_dict = self.data_node_subs_dict,
                            log_name_list = [],
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()
        else:
            self.node_if_shared = True
            try:
                self.node_if = node_if
                self.node_if.register_pubs(self.data_node_pubs_dict)
                self.node_if.register_subs(self.data_node_subs_dict)
                nepi_sdk.sleep(1)
            except Exception as e:
                self.msg_if.pub_info("Failed to register pubs and subs: " + str(e))
                return


        ##############################
        # Start updater data

        if pub_status == True:
            nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

        ##############################
        # Complete Initialization
        self.data_ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete")
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_data_ready_state(self):
        """Return the ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.data_ready

    def wait_for_data_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.data_ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.data_ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.data_ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.data_ready  

    def get_namespace(self):
        """Return the fully-resolved ROS namespace this data set publishes under.

        This is create_namespace(node_namespace, data_name) -- the same
        namespace the status publisher and every set_*_datum_value subscriber
        are registered on, so it is what a remote subscriber should use.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.namespace
    
   
    def unregister(self):
        """Shut down this settings interface and release its ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)
                self.PUBS_DICT = None
                
        time.sleep(1)
        try:
            self.node_if = None
            self.data_ready = False
            success = True
        except Exception as e:
            self.msg_if.pub_warn("Failed to unregister:  " + str(e))
        return success


    ##################
    # Data Functions
    def get_data_dict(self):
        data_dict = copy.deepcopy(self.data_dict)
        return data_dict

    def get_datum_value(self, datum_name):
        data_dict = copy.deepcopy(self.data_dict)
        value = nepi_data.get_value(data_dict, datum_name)
        return value

    def set_datum_value(self, datum_name, update_value, index = None):
        data_dict = copy.deepcopy(self.data_dict)
        if data_dict is not None:
            if datum_name in data_dict.keys():
                data_dict = nepi_data.set_value(data_dict, datum_name, update_value, index = index)
                if data_dict != self.data_dict:
                    self.data_dict = data_dict
                    self.publish_status()


    def get_datum_labels(self, datum_name):
        data_dict = copy.deepcopy(self.data_dict)
        labels = nepi_data.get_labels(data_dict, datum_name)
        return labels

    def set_datum_labels(self, datum_name, labels):
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = nepi_data.set_labels(data_dict, datum_name, labels)
        if self.data_dict != data_dict:
            self.publish_status()



    def get_datum_bounds(self, datum_name):
        data_dict = copy.deepcopy(self.data_dict)
        bounds = nepi_data.get_bounds(data_dict, datum_name)
        return bounds


    def set_datum_min_bound(self, datum_name, min_bound = None):
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = nepi_data.set_min_bound(data_dict, datum_name, min_bound = min_bound)
        if self.data_dict != data_dict:
            self.publish_status()


    def set_datum_max_bound(self, datum_name, max_bound = None):
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = nepi_data.set_min_bound(data_dict, datum_name, max_bound = max_bound)
        if self.data_dict != data_dict:
            self.publish_status()



    def set_datum_bounds(self, datum_name, bounds = [None,None]):
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = nepi_data.set_bounds(data_dict, datum_name, bounds)
        if self.data_dict != data_dict:
            self.publish_status()




    ##################
    # Display Functions

    def get_datum_display_name(self, datum_name):
        data_dict = copy.deepcopy(self.data_dict)
        display_name = nepi_data.get_display_name(data_dict, datum_name)
        return display_name

    def set_datum_display_name(self, datum_name, display_name):
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.set_display_name(data_dict, datum_name, display_name)
        self.data_dict = data_dict


    def get_datum_description(self, datum_name):
        data_dict = copy.deepcopy(self.data_dict)
        description = nepi_data.get_description(data_dict, datum_name)
        return description

    def set_datum_description(self, datum_name, description):
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.set_description(data_dict, datum_name, description)
        self.data_dict = data_dict

    def get_datum_hidden(self, datum_name):
        data_dict = copy.deepcopy(self.data_dict)
        hidden = nepi_data.get_hidden(data_dict, datum_name)
        return hidden

    def set_datum_hidden(self, datum_name, hidden):
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = nepi_data.set_hidden(data_dict, datum_name, hidden)
        if self.data_dict != data_dict:
            self.publish_status()



    def get_datum_display_order(self, datum_name):
        data_dict = copy.deepcopy(self.data_dict)
        order = nepi_data.get_display_order(data_dict, datum_name)
        return order

    def set_datum_display_order(self, datum_name, update_order = 0):
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.set_display_order(data_dict, datum_name, update_order)
        self.data_dict = data_dict


    def move_datum_display_top(self, datum_name):
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.move_datum_top(data_dict, datum_name)
        self.data_dict = data_dict


    def move_datum_display_bottom(self, datum_name):
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.move_datum_bottom(data_dict, datum_name)
        self.data_dict = data_dict


    def move_datum_display_up(self, datum_name):
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.move_datum_up(data_dict, datum_name)
        self.data_dict = data_dict


    def move_datum_display_down(self, datum_name):
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.move_datum_down(data_dict, datum_name)
        self.data_dict = data_dict



    ##################
    # Misc Functions

    def publish_status(self, status_msg = None):
        ###########
        if self.pub_status == True:
            data_dict = copy.deepcopy(self.data_dict)
            self.data_status_msg = nepi_data.update_status_msg(self.data_status_msg, data_dict)
            self.data_status_msg.config_topic = self.config_topic
            if self.node_if is not None:
                if self.status_has_published == False:
                    self.msg_if.pub_warn("Publishing Status: " + str(self.data_status_msg))
                    self.status_has_published = True
                self.node_if.publish_pub(self.node_if_prefix + 'status_pub', self.data_status_msg) 
            return

    def init(self, do_updates = False):
        """Initialize or re-initialize data from the parameter server and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset data
        """
        pass
        self.init(do_updates = True)

    def factory_reset(self):
        pass
        self.init(do_updates = True)

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)


    def _updateDatumCb(self,msg):
        self.msg_if.pub_info("Received datum update msg: " + str(msg), log_name_list = self.log_name_list)
        datum_name = msg.name
        data_dict = nepi_data.apply_update_msg(self.data_dict, msg)
        datum_value = nepi_data.get_value(data_dict, datum_name )
        self.set_datum_value(datum_name, datum_value)
    

    def _setHiddenValueCb(self,msg):
            self.set_datum_hidden(msg.name, msg.value)

    def _setDataHiddenCb(self,msg):
            self.set_data_hidden(msg.value)

    def _setOrderValueCb(self,msg):
            self.set_datum_display_order(msg.name, msg.value)

    def _setOrderTopCb(self,msg):
            self.move_datum_display_top(msg.name)

    def _setOrderBottomCb(self,msg):
            self.move_datum_display_bottom(msg.name)

    def _setOrderDownCb(self,msg):
            self.move_datum_display_down(msg.name)

    def _publishStatusCb(self,timer):
            self.publish_status()









#########################################
# Process IF Class
#########################################

CONNECTED_TIMEOUT = 2
class ProcessIF:
    
    msg_if = None
    node_if = None
    config_topic = ''
    node_if_shared = False
    ready = False

    save_data_if = None
    data_products = None

    status_msg = ProcessStatus()
    save_data_topic = ''


    active_nodes = []
    active_topics = []
    active_topic_types =  []
    active_services =  []  

    process_name = None
    namespace = ''

    data_dict = dict()
    
    has_controls = False
    controls_msg = ControlsStatus()
    controls_dict = dict()

    has_results = False
    results_msg = DataStatus()
    results_dict = None
    has_results_pub = False
    results_pub_msg = None
    results_pub_topic = None
    results_pub_namespace = ''

    has_results = False
    results_msg = DataStatus()
    results_dict = None
    
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

    process_callback = None

    min_max_process_rates = [0.1,100]
    max_process_rate_hz = 10.0



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
    min_max_image_pub_rates = [1,20]
    max_image_pub_rate_hz = 10.0
    image_pub_topics = []


    show_enable = False
    show_rates = True
    show_selector = True
    show_process = False
    show_data = True
    show_controls = True
    show_results = True
    show_stats = True
    show_save_data = False

    status_has_published = False

    #######################
    ### IF Initialization
    def __init__(self, 
                process_name = 'process',
                process_group = 'PROCESS',
                process_description = 'Process',
                process_module = None,
                process_image_name = None,
                show_enable = False,
                show_rates = True,
                show_selector = True,
                show_process = True,
                show_controls = True,
                show_results = True,
                show_stats = True,
                show_save_data = False,
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
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,self.process_name)

        self.data_products = [self.process_name]
        if process_image_name is not None:
            self.image_pub_name = nepi_utils.get_clean_name(process_image_name)
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

        if process_module is None:
            self.msg_if.pub_warn("No Process Module Provided")
            return

        self.process_module = process_module


        success = self._reloadProcesses()
        if success == False:
            self.msg_if.pub_warn("INITIAL PROCESS LOAD FAILED: " + str(self.processes_functions_dict))
        else:
            self.msg_if.pub_warn("INITIAL PROCESS LOAD SUCCEEDED: " + str(self.processes_functions_dict))



        self.show_enable = show_enable
        self.show_rates = show_rates
        self.show_selector = show_selector
        self.show_process = show_process
        self.show_controls = show_controls
        self.show_results = show_results
        self.show_stats = show_stats
        self.show_save_data = show_save_data

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
            self.node_if_prefix + 'max_process_rate_hz': {
                'name': 'max_process_rate_hz',
                'namespace': self.namespace,
                'factory_val': self.max_process_rate_hz
            },
            self.node_if_prefix +  'max_image_pub_rate_hz': {
                'name': 'max_image_pub_rate_hz',
                'namespace': self.namespace,
                'factory_val': self.max_image_pub_rate_hz
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

        
        if self.results_pub_msg is not None and self.results_pub_topic is not None:
            results_pub_topic = nepi_utils.get_clean_name(self.results_pub_topic)
            if results_pub_topic != '':
                self.process_node_pubs_dict[self.node_if_prefix + 'results_pub'] = {
                    'namespace': self.namespace,
                    'topic': results_pub_topic,
                    'msg': self.results_pub_msg,
                    'qsize': 1,
                    'latch': True
                }
                self.results_pub_namespace = self.namespace + '/' + results_pub_topic
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
        if len(self.data_products) > 0:
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
                    self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic), log_name_list = self.log_name_list)




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
                [self.data_dict,self.controls_dict,self.results_dict,self.states_dict] = nepi_process.get_process_dicts(processes_dict,process_name)
                self.process_function = self.processes_functions_dict[process_name]
                nepi_sdk.sleep(1)
                success = True
                self.msg_if.pub_warn("Process Ready: " + str(process_name))
                #self.msg_if.pub_warn("Process Dictionaries: " + str([self.data_dict,self.controls_dict,self.results_dict,self.process_function]))

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
            controls_values_dict = get_controls_values_dict = nepi_controls.gets_values_dict(controls_dict)
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
                            processes_controls_dict[process_name] = nepi_controls.gets_values_dict(self.processes_dict[process_name]['controls_dict'])
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
                            processes_controls_dict[process_name] = nepi_controls.gets_values_dict(self.processes_dict[process_name]['controls_dict'])
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
                            processes_controls_dict[process_name] = nepi_controls.gets_values_dict(self.processes_dict[process_name]['controls_dict'])
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


    def get_results(self):
        values_dict = None
        results_dict = copy.deepcopy(self.results_dict)
        if results_dict is not None:
            values_dict = nepi_data.get_values_dict(results_dict)
        return values_dict


    def process_results(self, source_topic = ''):
        if source_topic != '' and source_topic not in self.connected_source_topics:
            self.connected_source_topics.append(source_topic)
        results_pub_msg = None
        #self.msg_if.pub_warn("Processing results: " + str( [self.data_dict, self.controls_dict, self.results_dict, self.process_function]), throttle_s = 5)
        if self.enabled == True:
            process_ready = self.wait_for_process_ready()
            if process_ready == True:
                try:
                    [self.data_dict, self.controls_dict, self.results_dict, self.states_dict, results_pub_dict] = self.process_function(self.data_dict, self.controls_dict, self.results_dict, self.states_dict)
                    #self.msg_if.pub_warn("Processed results: " + str( [self.results_dict, results_pub_msg]), throttle_s = 5)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to process results: " + str(e), throttle_s = 5) 
                self._publishResults(results_pub_dict, source_topic)
            else:
                self.msg_if.pub_warn("Processes Not Ready", throttle_s = 10)
            
            
        else:
            #self.msg_if.pub_warn("Process Not Ready. Can't Pub Results", throttle_s = 5)
            pass
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
        status_msg.max_process_rate_hz = self.max_process_rate_hz

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
                self.controls_msg.show_controls = self.show_controls
                status_msg.controls = self.controls_msg

        results_dict = copy.deepcopy(self.results_dict)
        if results_dict is not None:
            has_results = len(list(results_dict.keys())) > 0
            status_msg.has_results = has_results
            if has_results == True:
                self.results_msg = nepi_data.update_status_msg(self.results_msg, results_dict)
                self.results_msg.show_data = self.show_results
                status_msg.results = self.results_msg

        status_msg.has_results_pub = self.has_results_pub
        if self.has_results_pub == True:
            status_msg.results_pub_namespace = self.results_pub_namespace


        status_msg.image_pub_name = self.image_pub_name
        status_msg.min_max_image_pub_rates = self.min_max_image_pub_rates
        status_msg.max_image_pub_rate_hz = self.max_image_pub_rate_hz
        status_msg.image_pub_topics = self.image_pub_topics


        status_msg.show_enable = self.show_enable
        status_msg.show_rates = self.show_rates
        status_msg.show_selector = self.show_selector
        status_msg.show_process = self.show_process
        status_msg.show_controls = self.show_controls
        status_msg.show_results = self.show_results
        status_msg.show_stats = self.show_stats
        status_msg.show_save_data = self.show_save_data


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
        if self.process_module is not None:
            self.process_ready = False           
            nepi_sdk.sleep(1)
            process_busy = self.wait_on_process_busy()
            success = True
            if process_busy == True:
                self.msg_if.pub_info("Failed to load process. Process Busy: " + str(process_busy))
            else:
                processes_controls_dict = copy.deepcopy(self.processes_controls_dict)
                try:
                    success = False
                    importlib.reload(self.process_module)
                    processes_dict = self.process_module.PROCESSES_DICT
                    self.msg_if.pub_warn("################################")
                    self.msg_if.pub_warn("Process Reloaded")
                    self.msg_if.pub_warn("Updating Process Dictionaries")

                    try:
                        self.data_products[0] = self.process_module.RESULTS_PUB_TOPIC
                    except:
                        pass
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


                    try:
                        self.results_pub_msg = self.process_module.RESULTS_PUB_MSG
                        self.results_pub_topic = self.process_module.RESULTS_PUB_TOPIC
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
        success = self.selected_process in self.available_processes
        self.process_ready = success
        return success


    def _updateControlCb(self,msg):
        self.msg_if.pub_info("Received control update msg: " + str(msg), log_name_list = self.log_name_list)
        control_name = msg.name
        controls_dict = nepi_controls.apply_update_msg(self.controls_dict, msg)
        control_value = nepi_controls.get_value(controls_dict, control_name )
        self.set_control_value(control_name, control_value)
        
    def _publishResults(self, results_pub_dict, source_topic = ''):
        #self.msg_if.pub_warn("Starting Pub Result Process with Results Dict and Results Msg: " + str([results_pub_msg, self.results_pub_msg]), throttle_s = 10) 
        try:
            msg = self.process_module.RESULTS_PUB_MSG
            msg_type = self.process_module.RESULTS_PUB_TYPE
            results_pub_msg = nepi_process.convert_results_pub_dict2msg(msg, msg_type, results_pub_dict)
        except:
            results_pub_msg = None
        if self.node_if is not None and self.results_pub_msg is not None and results_pub_msg is not None:
            results_pub_msg.results_header.timestamp = nepi_utils.get_time()
            results_pub_msg.results_header.process_name = self.node_name
            results_pub_msg.results_header.process_namespace = self.node_namespace
            results_pub_msg.results_header.source_topic = source_topic
            results_pub_msg.results_header.source_timestamp = nepi_utils.get_time() 
            #self.msg_if.pub_warn("Publishing Results Msg: " + str(results_pub_msg), throttle_s = 5) 
            self.node_if.publish_pub(self.node_if_prefix + 'results_pub', results_pub_msg) 
        else:
            #self.msg_if.pub_warn("Failed to Pub. Results Msg is None: " + str([results_dict, self.results_pub_msg]), throttle_s = 10) 
            pass


    def _publishStatusCb(self, timer):
        self.publish_status()
       


           


################################################
## ReadWriteIF


class ReadWriteIF:

    ready = False
    node_name = ''
    # Save data variables
    filename_dict = {
        'prefix': "",
        'suffix': "",
        'add_timestamp': True, 
        'use_utc_tz': True,
        'add_ms': True,
        'add_us': False,
        'add_tz': True,
        'add_node_name': True
        }

    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'read_write_'


    #######################
    ### IF Initialization
    def __init__(self,
                filename_dict = None,
                node_name = None,
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
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)
        



        #############################
        # Initialize Class Variables

        if node_name is not None:
            self.node_name = node_name.replace(' ','-')
        self.msg_if.pub_info("Using node_name: " + self.node_name, log_name_list = self.log_name_list)

        if filename_dict is not None:
            for key in self.filename_dict.keys():
                if key in filename_dict.keys():
                    self.filename_dict[key] = filename_dict[key]

        self.data_dict = {
            'dict': {
                'data_type': dict,
                'file_types': ['yaml'],
                'read_function': self.read_dict_file,
                'write_function': self.write_dict_file
            },
            'array': {
                'data_type': np.ndarray,
                'file_types': ['npy','csv','txt'],
                'read_function': self.read_array_file,
                'write_function': self.write_array_file 
            },
            'image': {
                'data_type': np.ndarray,
                'file_types': ['png','PNG','jpg','jpeg','JPG'],
                'read_function': self.read_image_file,
                'write_function': self.write_image_file 
            },
            'pointcloud': {
                'data_type': o3d.geometry.PointCloud,
                'file_types': ['pcd'],
                'read_function': self.read_pointcloud_file,
                'write_function': self.write_pointcloud_file 
            },
        }

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
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

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready


    def get_namespace(self):
        """Return the ROS namespace used by this interface.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace

    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                # if self.SRVS_DICT is not None:
                #         for service_name in self.SRVS_DICT.keys():
                #             self.node_if.unregister_service(service_name)
                # self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                # if self.node_if is not None:
                #     if self.PUBS_DICT is not None:
                #         for pub_name in self.PUBS_DICT.keys():
                #             self.node_if.unregister_pub(pub_name)


    def get_supported_data_types(self):
        """Return the list of data type keys supported for file I/O.

        Returns:
            list: List of supported data type strings (e.g. 'dict', 'array', 'image',
                'pointcloud').
        """
        return list(self.data_dict.keys())


    def get_filename_prefix(self):
        """Return the current filename prefix string.

        Returns:
            str: The prefix prepended to generated filenames.
        """
        return self.filename_dict['prefix']

    def set_filename_prefix(self, prefix = ''):
        """Set the filename prefix string used when generating file names.

        Args:
            prefix (str, optional): Prefix string to prepend to filenames.
                Defaults to ''.
        """
        self.filename_dict['prefix'] = prefix

    def get_use_utc_tz(self):
        """Return whether UTC timezone is used for file timestamps.

        Returns:
            bool: True if UTC is used, False if local timezone is used.
        """
        return self.filename_dict['use_utc_tz']

    def set_use_utc_tz(self, use_utc_tz):
        """Set whether timestamps in filenames use UTC or local timezone.

        Args:
            use_utc_tz (bool): True to use UTC, False to use local timezone.
        """
        self.filename_dict['use_utc_tz'] = use_utc_tz

    def get_add_timestamp(self):
        """Return whether a timestamp is appended to generated filenames.

        Returns:
            bool: True if a timestamp is included in filenames.
        """
        return self.filename_dict['add_timestamp']

    def set_add_timestamp(self, add_timestamp):
        """Set whether a timestamp is included in generated filenames.

        Args:
            add_timestamp (bool): True to include a timestamp in filenames.
        """
        self.filename_dict['add_timestamp'] = add_timestamp

    def get_add_ms(self):
        """Return whether milliseconds are included in the filename timestamp.

        Returns:
            bool: True if milliseconds are appended to the timestamp portion.
        """
        return self.filename_dict['add_ms']

    def set_add_ms(self, add_ms):
        """Set whether milliseconds are included in the filename timestamp.

        Args:
            add_ms (bool): True to append milliseconds to the timestamp.
        """
        self.filename_dict['add_ms'] = add_ms

    def get_add_us(self):
        """Return whether microseconds are included in the filename timestamp.

        Returns:
            bool: True if microseconds are appended to the timestamp portion.
        """
        return self.filename_dict['add_us']

    def set_add_us(self, add_us):
        """Set whether microseconds are included in the filename timestamp.

        Args:
            add_us (bool): True to append microseconds to the timestamp.
        """
        self.filename_dict['add_us'] = add_us

    def get_add_tz(self):
        """Return whether the timezone abbreviation is included in filenames.

        Returns:
            bool: True if the timezone string is appended to the timestamp.
        """
        return self.filename_dict['add_tz']

    def set_add_tz(self, add_tz):
        """Set whether the timezone abbreviation is included in filenames.

        Args:
            add_tz (bool): True to append the timezone string to the timestamp.
        """
        self.filename_dict['add_tz'] = add_tz


    def get_filename_dict(self):
        """Return the full filename configuration dictionary.

        Returns:
            dict: Dictionary containing all filename formatting options (prefix,
                suffix, add_timestamp, use_utc_tz, add_ms, add_us, add_tz,
                add_node_name).
        """
        return self.filename_dict

    def set_filename_dict(self,filename_dict):
        """Merge a partial filename configuration dict into the current settings.

        Any keys missing from the provided dict are filled in from the current
        filename_dict, ensuring all required keys remain present.

        Args:
            filename_dict (dict): Dictionary with one or more filename config keys
                to update.

        Returns:
            dict: The updated filename configuration dictionary.
        """
        for key in self.filename_dict.keys():
            if key not in filename_dict.keys():
                filename_dict[key] = self.filename_dict[key]

        self.filename_dict = filename_dict
        return self.filename_dict


    def get_folder_files(self, path, ext_str = ""):
        """Return a list of files in a folder, optionally filtered by extension.

        Args:
            path (str): Filesystem path of the directory to list.
            ext_str (str, optional): File extension filter (e.g. 'yaml'). Defaults
                to '' (all files).

        Returns:
            list: List of file paths matching the extension filter.
        """
        file_list = nepi_utils.get_file_list(path,ext_str=ext_str)
        return file_list

    def get_time_from_filename(self,filename):
        """Extract and return the timestamp embedded in a NEPI-style filename.

        Args:
            filename (str): Filename string containing an embedded datetime token.

        Returns:
            float: Unix timestamp (seconds) parsed from the filename, or None if
                parsing fails.
        """
        file_time = None
        dt_str = self._getDtStr(filename)
        file_time = nepi_utils.get_time_from_datetime_str(dt_str)
        return file_time


    def get_data_type(self,data):
        """Infer the NEPI data type category of a data object.

        Inspects the Python type and, for numpy arrays, the dtype and number of
        dimensions to distinguish images from generic arrays.

        Args:
            data: The data object to classify.

        Returns:
            str: One of 'dict', 'pointcloud', 'image', 'array', or the string
                representation of the object's type if unrecognized.
        """
        dtype = type(data)

        if dtype == dict:
            return 'dict'
        elif dtype == o3d.geometry.PointCloud:
            return 'pointcloud'
        elif dtype == np.ndarray:
            # Check for valid image dtypes (uint8 or float32 are common)
            image_dtypes = [np.uint8]
            if data.dtype not in image_dtypes:
                return 'array'
            elif data.ndim == 2 or data.ndim == 3 :
                if data.dtype == np.uint8:
                    return 'image'
                else:
                    return 'array'
            else:
                return 'array'
        else:
            return str(dtype)
            





    def write_data_file(self, filepath, data, data_name, timestamp = None, timezone = None, filename = None, key_name = None):
        """Write a data object to disk using the appropriate format for its type.

        The data type is inferred automatically via get_data_type(). Supported types
        are 'dict' (YAML), 'array' (npy/csv), 'image' (png/jpg), and 'pointcloud'
        (pcd). Unsupported types are logged as warnings.

        Args:
            filepath (str): Directory path where the file will be written.
            data: Data object to save. Must be one of the supported types.
            data_name (str): Name token embedded in the generated filename.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the filename timestamp.
                Defaults to None (UTC).
        """
        data_type = self.get_data_type(data)
        found_type = False
        if data_type in self.data_dict.keys():
                save_function = self.data_dict[data_type]['write_function']
                #self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone), log_name_list = self.log_name_list, throttle_s = 5.0)
                filename = save_function(filepath, data, data_name, timestamp = timestamp, timezone = timezone, filename = filename, key_name = key_name)
                found_type = True
        if found_type == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type) + ' for data name: ' + str(data_name) + ' with data: ' + str(data), log_name_list = self.log_name_list, throttle_s = 5.0)
        return filename


    def read_dict_file(self, filepath, filename):
        """Read a YAML file and return its contents as a dictionary.

        Args:
            filepath (str): Directory path containing the file.
            filename (str): Name of the YAML file to read (must have a supported
                extension such as 'yaml').

        Returns:
            dict: Parsed dictionary from the YAML file, or None if the file type
                is unsupported or reading fails.
        """
        data_key = 'dict'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            try:
                data = nepi_utils.read_yaml_2_dict(file_path)
            except:
                self.msg_if.pub_warn("Failed to read file: " + file_path, log_name_list = self.log_name_list)
        return data

    def write_dict_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'yaml', filename = None, key_name = None):
        """Write a dictionary to a YAML file using a generated filename.

        Args:
            filepath (str): Directory path where the file will be written.
            data (dict): Dictionary to serialize.
            data_name (str): Name token embedded in the generated filename.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the filename timestamp.
                Defaults to None (UTC).
            ext_str (str, optional): File extension. Defaults to 'yaml'.

        Returns:
            bool: True if the file was written successfully, False otherwise.
        """
        data_key = 'dict'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                if filename is None:
                    filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    try:
                        success = nepi_utils.add_dict_2_yaml(file_path, data, key_name = key_name)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5)
                else:
                    try:
                        data_to_write = {key_name: data} if key_name is not None else data
                        success = nepi_utils.write_dict_2_yaml(file_path, data_to_write)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5)
        return filename


    def read_array_file(self, filepath, filename):
        """Read a numeric array from a .npy, .csv, or .txt file.

        Args:
            filepath (str): Directory path containing the file.
            filename (str): Name of the array file to read (must have a supported
                extension such as 'npy', 'csv', or 'txt').

        Returns:
            numpy.ndarray: Array loaded from the file, or None if the file type is
                unsupported or reading fails.
        """
        data_key = 'array'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            try:
                if ext_str == 'npy':
                    data = np.genfromtxt(file_path, delimiter=',', dtype=float, filling_values=-999)
                else:
                    data = np.genfromtxt(file_path, delimiter=',', dtype=float, filling_values=-999)
            except:
                self.msg_if.pub_warn("Failed to read file: " + file_path, log_name_list = self.log_name_list)
                
        return data

    def write_array_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'npy', filename = None, key_name = None):
        """Write a numpy array to a file using a generated filename.

        Args:
            filepath (str): Directory path where the file will be written.
            data (numpy.ndarray): Array to save.
            data_name (str): Name token embedded in the generated filename.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the filename timestamp.
                Defaults to None (UTC).
            ext_str (str, optional): File extension ('npy', 'csv', or 'txt').
                Defaults to 'npy'.

        Returns:
            bool: True if the file was written successfully, False otherwise.
        """
        data_key = 'array'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(type(data)), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    try:
                        if ext_str == 'npy':
                            file_path = file_path.replace('.' + ext_str, '')
                            success = np.save(file_path, data)
                        else:
                            success = np.savetxt(file_path, data)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5)

        return success


    def read_image_file(self, filepath, filename):
        """Read an image from disk and return it as a numpy array.

        Args:
            filepath (str): Directory path containing the file.
            filename (str): Name of the image file to read (must have a supported
                extension such as 'png', 'jpg', or 'jpeg').

        Returns:
            numpy.ndarray: Image array, or None if the file type is unsupported or
                reading fails.
        """
        data_key = 'image'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            try:
                data = nepi_img.read_image_file(file_path)
            except:
                self.msg_if.pub_warn("Failed to read file: " + file_path, log_name_list = self.log_name_list)
                
        return data

    def write_image_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'png', filename = None, key_name = None):
        """Write a numpy image array to an image file using a generated filename.

        Args:
            filepath (str): Directory path where the file will be written.
            data (numpy.ndarray): Image array (uint8) to save.
            data_name (str): Name token embedded in the generated filename.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the filename timestamp.
                Defaults to None (UTC).
            ext_str (str, optional): Image format extension ('png', 'jpg', etc.).
                Defaults to 'png'.

        Returns:
            bool: True if the file was written successfully, False otherwise.
        """
        data_key = 'image'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(type(data)), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    try:
                        success = nepi_img.write_image_file(file_path, data)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5)
        return success


    def read_pointcloud_file(self, filepath, filename):
        """Read a point cloud from a .pcd file and return an Open3D PointCloud object.

        Args:
            filepath (str): Directory path containing the file.
            filename (str): Name of the point cloud file to read (must have a
                supported extension such as 'pcd').

        Returns:
            open3d.geometry.PointCloud: Loaded point cloud, or None if the file type
                is unsupported or reading fails.
        """
        data_key = 'pointcloud'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            try:
                data = nepi_pc.read_pointcloud_file(file_path)
            except:
                self.msg_if.pub_warn("Failed to read file: " + file_path, log_name_list = self.log_name_list)
                
        return data

    def write_pointcloud_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'pcd', filename = None, key_name = None):
        """Write an Open3D point cloud to a file using a generated filename.

        Args:
            filepath (str): Directory path where the file will be written.
            data (open3d.geometry.PointCloud): Point cloud object to save.
            data_name (str): Name token embedded in the generated filename.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the filename timestamp.
                Defaults to None (UTC).
            ext_str (str, optional): Point cloud format extension. Defaults to 'pcd'.

        Returns:
            bool: True if the file was written successfully, False otherwise.
        """
        data_key = 'pointcloud'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    try:
                        success = nepi_pc.write_pointcloud_file(file_path,data)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5)
                    
        return success


    def get_example_filename(self, data_name = 'data_product', timestamp = None, timezone = None, ext_str = 'ext'):
        """Generate and return an example filename using the current naming settings.

        Useful for previewing what output filenames will look like without writing
        any data.

        Args:
            data_name (str, optional): Name token to embed in the filename. Defaults
                to 'data_product'.
            timestamp (float, optional): Unix timestamp used in the filename. Defaults
                to current time if None.
            timezone (str, optional): Timezone name for the timestamp. Defaults to
                None (UTC).
            ext_str (str, optional): File extension for the example filename. Defaults
                to 'ext'.

        Returns:
            str: The generated example filename string.
        """
        filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
        return filename
    ###############################
    # Class Private Methods
    ###############################

    def _createFileName(self, data_name_str, timestamp = None, timezone = None, ext_str = ""):
        if timestamp == None:
            timestamp = nepi_utils.get_time()
        prefix = self.filename_dict['prefix']
        if len(prefix) > 0:
            if prefix[-1] != '_':
                prefix = prefix + '_'
        suffix = self.filename_dict['suffix']
        if len(suffix) > 0:
            if suffix[0] != '_':
                suffix = '_' + suffix
        add_time = self.filename_dict['add_timestamp']
        data_time_str = ''
        if add_time == True:
            time_ns = nepi_sdk.sec_from_timestamp(timestamp)
            add_ms = self.filename_dict['add_ms']
            add_us = self.filename_dict['add_us']
            add_tz = self.filename_dict['add_tz']
            data_time_str = nepi_utils.get_datetime_str_from_timestamp(time_ns, add_ms = add_ms, add_us = add_us, add_tz = add_tz, timezone = timezone) + '_'
        node_name_str = ""
        if self.filename_dict['add_node_name'] == True:
            node_name_str = self.node_name
        if len(ext_str) > 0:
            ext_str  = '.' + ext_str
        if len(data_name_str) >  0:
            data_name_str = '-' + data_name_str
        filename = prefix + data_time_str + node_name_str + data_name_str + suffix + ext_str
        return filename

    
    def _getDtStr(self,filename):
        d_inds = nepi_utils.find_all_indexes(filename, 'D')
        dt_ind = None
        dt_str = filename
        for ind in d_inds:
            if len(filename) >= ind + 1:
                if filename[ind + 1].isdigit() == True:
                    dt_ind = ind
                    break
        dt_str = dt_str[dt_ind:]
        dt_str = dt_str.split('_')[0]
        return dt_str
                




################################################
## SaveDataIF

FALLBACK_DATA_FOLDER = '/mnt/nepi_storage/data'

SUPPORTED_DICT_FILE_TYPES = ['yaml']
SUPPORTED_IMG_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']  
SUPPORTED_PC_FILE_TYPES = ['pcd']
SUPPORTED_VID_FILE_TYPES = ['avi','AVI']

SUPPORTED_DATA_TYPES = ['dict','cv2_image','o3d_pointcloud']

'''
EXAMPLE_RATE_DICT = {
    'data_product_1' : [save_rate_hz, last_time, max_rate],
    'data_product_2' : [save_rate_hz, last_time, max_rate]
}
'''

EXAMPLE_FILENAME_DICT = {
    'prefix': "", 
    'subfolder': "",
    'add_timestamp': True, 
    'use_utc_tz': True,
    'add_ms': True,
    'add_us': False,
    'add_tz': True,
    'add_node_name': False
    }

class SaveDataIF:

    DEFAULT_TIMEZONE = 'UTC'

    ready = None
    namespace = "~"
    all_save_namespace = None
    status_msg = SaveDataStatus
    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'save_data'
    read_write_if = None
 
    snapshot_dict = dict()

    sys_mgr_if = None
    read_write_if = None

    save_data_root_directory = FALLBACK_DATA_FOLDER

    filename_dict = {
        'prefix': "", 
        'subfolder': "", 
        'add_timestamp': True, 
        'use_utc_tz': True,
        'add_ms': True,
        'add_us': False,
        'add_tz': True,
        'add_node_name': True
        }

    save_rate_dict = dict()
    save_data = False
    use_utc_tz = False

    disabled = False
    save_all_enabled = False
    save_all_rate = 0.0

    space_available = False


    file_prefix = ""
    subfolder = ""

    was_saving = False

    pub_status = False

    ### IF Initialization
    def __init__(self, 
                namespace = None,
                save_data_name = 'save_data',
                data_products = [], 
                pub_status = True,
                factory_rate_dict = None, 
                factory_filename_dict = None, 
                ignore_global_rate_updates = False,
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
            self.log_name_list.append(log_name)

        self.msg_if.pub_info("Starting SaveData IF Initialization Processes", log_name_list = self.log_name_list)
        ############################## 
        # Initialize Class Variables
        # Create Namespace
        save_data_name = nepi_utils.get_clean_name(save_data_name)
        if save_data_name is None or save_data_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(save_data_name)) 
            return
        self.msg_if.pub_info("Using States Name: " + save_data_name)
        # An explicit namespace lets a caller place this interface somewhere other than
        # its own node namespace. system_mgr passes the base namespace to own the global
        # '<base>/save_data' namespace, which is where the RUI's All selection subscribes
        # for status and where every other instance subscribes for global commands.
        if namespace is None:
            namespace = self.node_namespace
        self.namespace = nepi_sdk.create_namespace(namespace,save_data_name)
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'
        
        self.msg_if.pub_warn("Using save data namespace: " + self.namespace, log_name_list = self.log_name_list)
        
        self.pub_status = pub_status

        tzd = nepi_utils.get_timezone_description(self.DEFAULT_TIMEZONE)
        self.timezone = tzd

        all_save_namespace = nepi_sdk.create_namespace(self.base_namespace,'/save_data')
        if all_save_namespace != self.namespace:
            self.all_save_namespace = all_save_namespace

        ###############################
        # Connect Sys Mgr Services

        
        ##############################
        # Get for System Folders
        self.msg_if.pub_info("Waiting for user folders")
        user_folders = nepi_system.get_user_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got user folders: " + str(user_folders))
        if user_folders is not None and 'data' in user_folders.keys():
            self.save_data_root_directory = user_folders['data']
        self.msg_if.pub_info("Using SDK Share Folder: " + str(self.save_data_root_directory))

        # Ensure the data folder exists with proper ownership
        if not os.path.exists(self.save_data_root_directory):
            self.msg_if.pub_warn("Reported data folder does not exist... data saving is disabled", log_name_list = self.log_name_list)
            self.save_data_root_directory = None # Flag it as non-existent
            return # Don't enable any of the ROS interface stuff
        self.save_path = self.save_data_root_directory
        # And figure out user/group so that we know what ownership to create subfolders with
        stat_info = os.stat(self.save_data_root_directory)
        self.DATA_UID = stat_info.st_uid
        self.DATA_GID = stat_info.st_gid

 
 


        # Setup System IF Classes
        # Initialize with empty dict, then call update function

        self.read_write_if = ReadWriteIF(
                            filename_dict = dict(),
                            node_name = self.node_name
                            )
        nepi_sdk.sleep(1)
        self.msg_if.pub_debug("Got starting filename dict: " + str(self.filename_dict), log_name_list = self.log_name_list)
        if factory_filename_dict is not None:
            self.update_filename_dict(factory_filename_dict)
        self.msg_if.pub_debug("Got Updated filename dict: " + str(self.filename_dict), log_name_list = self.log_name_list)

        
        # Config initial data products dict
        self.msg_if.pub_debug("^^^^^^^^^^^^^^^^^^^^^^", log_name_list = self.log_name_list)
        self.msg_if.pub_debug("Starting Save_Data_IF with data products: " + str(data_products), log_name_list = self.log_name_list)
        self.msg_if.pub_debug("Starting Save_Data_IF with rate dict: " + str(factory_rate_dict), log_name_list = self.log_name_list)
        save_rate_dict = dict()
        save_rate = 0.0
        last_time = 0.0
        max_rate = 100
        for data_product in data_products:
            save_rate = 0.0
            last_time = 0.0
            max_rate = 100
            save_rate_entry = [save_rate, last_time, max_rate]
            if factory_rate_dict is not None:
                if data_product in factory_rate_dict.keys():
                    save_rate = factory_rate_dict[data_product][0]
            save_rate_entry[0] = save_rate
            save_rate_dict[data_product] = save_rate_entry
            self.snapshot_dict[data_product] = False
        self.save_rate_dict = save_rate_dict
        self.msg_if.pub_debug("Got defualt data rate dict: " + str(self.save_rate_dict), log_name_list = self.log_name_list)
            


        ##############################    
        # Node Setup
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }


        # Params Config Dict ####################
        self.PARAMS_DICT = {
            # 'disabled': {
            #     'namespace': self.namespace,
            #     'factory_val': self.disabled
            # },
            self.node_if_prefix + 'save_rate_dict': {
                'name': 'save_rate_dict',
                'namespace': self.namespace,
                'factory_val': self.save_rate_dict
            },
            self.node_if_prefix + 'filename_dict': {
                'name': 'filename_dict',
                'namespace': self.namespace,
                'factory_val': self.filename_dict
            }
        }


        # Services Config Dict ####################
        if self.namespace == self.namespace:
            self.SRVS_DICT = {
                self.node_if_prefix + 'save_data_capabilities_query': {
                    'namespace': self.namespace,
                    'topic': 'capabilities_query',
                    'srv': SaveDataCapabilitiesQuery,
                    'req': SaveDataCapabilitiesQueryRequest(),
                    'resp': SaveDataCapabilitiesQueryResponse(),
                    'callback': self._capabilitiesHandler
                }
            }
        else:
            self.SRVS_DICT = None


        # Publishers Config Dict ####################
        self.PUBS_DICT = dict()

        if self.pub_status == True:
            self.PUBS_DICT[self.node_if_prefix + 'save_data_status_pub'] = {
                'namespace': self.namespace,
                'msg': SaveDataStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            self.node_if_prefix + 'save_data_disable': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'disable',
                'qsize': 5,
                'callback': self._disableCb, 
                'callback_args': ()
            }, 
            self.node_if_prefix + 'save': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'save_data_enable',
                'qsize': 5,
                'callback': self._saveEnableCb, 
                'callback_args': ()
            },  
            self.node_if_prefix + 'prefix': {
                'namespace': self.namespace,
                'msg': String,
                'topic': 'save_data_prefix',
                'qsize': 5,
                'callback': self._setPrefixCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'subfolder': {
                'namespace': self.namespace,
                'msg': String,
                'topic': 'save_data_subfolder',
                'qsize': 5,
                'callback': self._setSubfolderCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'save_data_utc': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'save_data_utc',
                'qsize': 5,
                'callback': self._setLocalTzCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'filename': {
                'namespace': self.namespace,
                'msg': FilenameConfig,
                'topic': 'filename_config',
                'qsize': 5,
                'callback': self._setFilenameCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'rate': {
                'namespace': self.namespace,
                'msg': SaveDataRate,
                'topic': 'save_data_rate',
                'qsize': 5,
                'callback': self._saveRateCb, 
                'callback_args': ()
            },          
            self.node_if_prefix + 'snapshot': {
                'namespace': self.namespace,
                'msg': Empty,
                'topic': 'snapshot_trigger',
                'qsize': 5,
                'callback': self._snapshotCb,  
                'callback_args': ()
            },
            self.node_if_prefix + 'save_data_reset': {
                'namespace': self.namespace,
                'msg': Empty,
                'topic': 'reset_save_data',
                'qsize': 5,
                'callback': self._resetSaveDataCb,  
                'callback_args': ()
            },

        }

        if self.all_save_namespace is not None:
            ALL_SUBS_DICT =  {
                self.node_if_prefix + 'disable_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Bool,
                    'topic': 'saving_disable',
                    'qsize': 5,
                    'callback': self._disableCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'save_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Bool,
                    'topic': 'save_data_enable',
                    'qsize': 5,
                    'callback': self._saveEnableCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'prefix_all': {
                    'namespace': self.all_save_namespace,
                    'msg': String,
                    'topic': 'save_data_prefix',
                    'qsize': 5,
                    'callback': self._setPrefixCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'subfolder_all': {
                    'namespace': self.all_save_namespace,
                    'msg': String,
                    'topic': 'save_data_subfolder',
                    'qsize': 5,
                    'callback': self._setSubfolderCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'use_local_tz_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Bool,
                    'topic': 'save_data_utc',
                    'qsize': 5,
                    'callback': self._setLocalTzCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'filename_all': {
                    'namespace': self.all_save_namespace,
                    'msg': FilenameConfig,
                    'topic': 'filename_config',
                    'qsize': 5,
                    'callback': self._setFilenameCb,
                    'callback_args': ()
                },
                self.node_if_prefix + 'snapshot_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Empty,
                    'topic': 'snapshot_trigger',
                    'qsize': 5,
                    'callback': self._snapshotCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'save_all_config': {
                    'namespace': self.all_save_namespace,
                    'msg': Empty,
                    'topic': 'save_config',
                    'qsize': 5,
                    'callback': self._saveConfigCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'all_save_sub': {
                    'namespace': self.all_save_namespace,
                    'msg': SaveDataStatus,
                    'topic': 'status',
                    'qsize': 5,
                    'callback': self._saveAllStatusCb, 
                    'callback_args': ()
                }
            }
            if ignore_global_rate_updates == False:
                ALL_SUBS_DICT[self.node_if_prefix + 'rate_all'] = {
                        'namespace': self.all_save_namespace,
                        'msg': SaveDataRate,
                        'topic': 'save_data_rate',
                        'qsize': 5,
                        'callback': self._saveRateCb, 
                        'callback_args': ()
                    } 

            self.SUBS_DICT.update(ALL_SUBS_DICT)
        


        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            if self.PARAMS_DICT is not None:
               self.node_if.add_params(self.PARAMS_DICT) 
            self.node_if.register_services(self.SRVS_DICT)
            self.node_if.register_pubs(self.PUBS_DICT)
            self.node_if.register_subs(self.SUBS_DICT)
        else:
            self.config_topic = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = self.SRVS_DICT,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                                                )

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status()
        
        self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################



    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return the current ready state of the SaveDataIF.

        Returns:
            bool: True if initialization completed successfully, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the SaveDataIF is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready


    def get_namespace(self):
        """Return the ROS namespace used by this SaveDataIF.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace
    
    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SRVS_DICT is not None:
                        for service_name in self.SRVS_DICT.keys():
                            self.node_if.unregister_service(service_name)
                self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)


    def get_data_products(self):
        """Return the list of registered data product names.

        Returns:
            list: List of data product name strings currently tracked by this interface.
        """
        return list(self.save_rate_dict.keys())

    def register_data_product(self, data_product,factory_rate = 0):
        """Register a new data product with an optional factory save rate.

        If the data product is not already registered it is added to the save rate
        dictionary, the snapshot dictionary, and the ROS parameter server.

        Args:
            data_product (str): Unique name for the data product.
            factory_rate (float, optional): Default save rate in Hz. Defaults to 0
                (disabled).
        """
        save_rate_dict = self.save_rate_dict
        if data_product not in save_rate_dict.keys():
            save_rate_dict[data_product] =  [factory_rate, 0.0, 100] # Default to 1Hz save rate, max rate = 100Hz
            self.save_rate_dict = save_rate_dict
            self.snapshot_dict[data_product] = False
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix + 'save_rate_dict',save_rate_dict)

    def unregister_data_product(self, data_product):
        """Remove a previously registered data product from the save rate tracking dict.

        Args:
            data_product (str): Name of the data product to remove.
        """
        purge = False
        if data_product in self.save_rate_dict.keys():
            purge = True
        if purge == True:
            del self.save_rate_dict[data_product]

    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SRVS_DICT is not None:
                        for service_name in self.SRVS_DICT.keys():
                            self.node_if.unregister_service(service_name)
                self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)


    def update_filename_dict(self,filename_dict):
        """Apply updates from a filename configuration dictionary.

        Handles prefix sanitization, subfolder creation with proper ownership, and
        updates the underlying ReadWriteIF and ROS parameter server. Only applies
        if the new dict differs from the current one.

        Args:
            filename_dict (dict): Dictionary containing one or more filename config
                keys to update (prefix, subfolder, add_timestamp, use_utc_tz,
                add_ms, add_us, add_tz, add_node_name).
        """

        if self.filename_dict != filename_dict and filename_dict is not None:
            if 'prefix' in filename_dict.keys():
               new_prefix = filename_dict['prefix']
               self.filename_dict['prefix'] = nepi_utils.get_clean_name(new_prefix)

            if 'subfolder' in filename_dict.keys():
                new_subfolder= nepi_utils.get_clean_name(filename_dict['subfolder'])
                 

                if new_subfolder != "" and self.save_data_root_directory != None:
                    full_path = os.path.join(self.save_data_root_directory, new_subfolder)
                elif self.save_data_root_directory != None:
                    full_path = self.save_data_root_directory
                else:
                    full_path = ""

                if not os.path.exists(full_path):
                    self.msg_if.pub_debug("Creating new data subdirectory " + full_path)
                    try:
                        os.makedirs(full_path)
                    except Exception as e:
                        self.msg_if.pub_warn("Could not create save folder " + new_subfolder + str(e) )
                if os.path.exists(full_path):
                    try:
                        os.chown(full_path, self.DATA_UID, self.DATA_GID)
                    except Exception as e:
                        self.msg_if.pub_warn("Could not chmod on save folder " + full_path + str(e) )
                    self.save_path = full_path
                    filename_dict['subfolder'] = new_subfolder

            # Apply Updates
            for key in self.filename_dict.keys():
                if key in filename_dict.keys():
                    self.filename_dict[key] = filename_dict[key]  

            self.publish_status()
            if self.read_write_if is not None and self.filename_dict is not None:
                self.read_write_if.set_filename_dict(filename_dict)
            if self.node_if is not None and self.filename_dict is not None:
                self.node_if.set_param(self.node_if_prefix + 'filename_dict',self.filename_dict)
                self.node_if.save_config()

    def set_save_rate(self,data_product,save_rate_hz=0):
        """Set the save rate for one or more data products.

        Supports special sentinel values for data_product to target all products
        (ALL_DATA_PRODUCTS), none (NONE_DATA_PRODUCTS), or only currently-active
        ones (ACTIVE_DATA_PRODUCTS). Per-product rates are clamped to the
        configured max rate for that product.

        Args:
            data_product (str): Name of the data product, or a sentinel constant
                from SaveDataRate (ALL_DATA_PRODUCTS, NONE_DATA_PRODUCTS,
                ACTIVE_DATA_PRODUCTS).
            save_rate_hz (float, optional): Target save rate in Hz. Defaults to 0
                (disabled).
        """
        save_all = SaveDataRate().ALL_DATA_PRODUCTS
        save_none = SaveDataRate().NONE_DATA_PRODUCTS
        save_active = SaveDataRate().ACTIVE_DATA_PRODUCTS
        save_rate_dict = self.save_rate_dict
        if (data_product == save_active):
            for d in save_rate_dict.keys():
                # Respect the max save rate
                if save_rate_dict[d][0] > 0:
                    save_rate_dict[d][0] = save_rate_hz if save_rate_hz <= save_rate_dict[d][2] else save_rate_dict[d][2]
        elif (data_product == save_all):
            for d in save_rate_dict.keys():
                save_rate_dict[d][0] = save_rate_hz
        elif (data_product == save_none):
            for d in save_rate_dict.keys():
                save_rate_dict[d][0] = 0.0
        elif (data_product in save_rate_dict.keys()):
            save_rate_dict[data_product][0] = save_rate_hz if save_rate_hz <= save_rate_dict[data_product][2] else save_rate_dict[data_product][2]
        else:
            self.msg_if.pub_warn("Requested unknown data product: " + data_product)    
        self.save_rate_dict = save_rate_dict     
        #self.msg_if.pub_warn("Updated save rate dict: " + str(self.save_rate_dict))   
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'save_rate_dict',save_rate_dict)
            self.node_if.save_config()
        
    def disable(self, enabled):
        """Enable or disable data saving for all registered data products.

        When enabling after a disabled period, all last-save timestamps are reset
        so that each product triggers a save on the next call to data_product_should_save.

        Args:
            enabled (bool): True to enable saving, False to disable.
        """
        #self.msg_if.pub_warn("Setting Saving Disabled to: " + str(enabled))  
        self.disabled = enabled
        self.publish_status()    

    def save_data_enable(self, enabled):
        """Enable or disable data save state for all registered data products.

        When enabling after a disabled period, all last-save timestamps are reset
        so that each product triggers a save on the next call to data_product_should_save.

        Args:
            enabled (bool): True to enable save, False to disable.
        """
        if enabled == True and self.was_saving == False:
            for d in self.save_rate_dict.keys():
                self.save_rate_dict[d][1] = 0.0
            self.was_saving = True
        else:
            self.was_saving = False
        self.msg_if.pub_warn("Setting Save Enable to: " + str(enabled))  
        self.save_data = enabled
        self.publish_status()    


    def get_saving_enabled(self):
        """Return whether data saving is currently enabled.

        Returns:
            bool: True if saving is enabled, False if disabled.
        """
        return self.save_data and self.disabled == False

    def data_product_save_enabled(self, data_product):
        """Return whether a specific data product is currently configured to save.

        Returns False if global saving is disabled or if the product's save rate
        is zero.

        Args:
            data_product (str): Name of the data product to check.

        Returns:
            bool: True if saving is enabled and the product has a non-zero rate,
                False otherwise.
        """
        # If saving is disabled for this node, then no data products are saving
        try:
            save_rate_dict = self.save_rate_dict
            if self.save_data == False:
                return False

            if data_product not in save_rate_dict:
                self.msg_if.pub_warn("Unknown data product " + data_product)
                return False

            save_rate = save_rate_dict[data_product][0]
            return (save_rate > 0.0) and self.disabled == False
        except:
            return False
        
    def data_product_save_rate(self, data_product):
        """Return the configured save rate for a specific data product.

        Returns False if global saving is disabled or the product is unknown.

        Args:
            data_product (str): Name of the data product.

        Returns:
            float: Save rate in Hz, or False if saving is disabled or the product
                is unrecognized. Returns 0.0 on unexpected errors.
        """
        # If saving is disabled for this node, then no data products are saving
        try:
            save_rate_dict = self.save_rate_dict
            if self.save_data == False:
                return False

            if data_product not in save_rate_dict:
                self.msg_if.pub_warn("Unknown data product " + data_product)
                return False

            save_rate = save_rate_dict[data_product][0]
            return save_rate
        except:
            return 0.0



    def data_product_should_save(self, data_product):
        """Determine whether it is time to save a data product based on its rate and elapsed time.

        Checks the configured save rate and the time since the last save. Also
        returns True if a snapshot has been triggered for the product. Returns
        False if global saving is disabled, the product is unknown, or the rate
        is zero.

        Args:
            data_product (str): Name of the data product to check.

        Returns:
            bool: True if the product should be saved now, False otherwise.
        """
        # If saving is disabled for this node, then it is not time to save this data product!
        save_rate_dict = self.save_rate_dict
        #self.msg_if.pub_debug("Checking should save for save rate dict: " + str(save_rate_dict), log_name_list = self.log_name_list, throttle_s = 5)
        
        if self.save_data == False or  self.disabled == True:
            return False

        if data_product not in save_rate_dict.keys():
            self.msg_if.pub_warn("Unknown data product " + data_product, log_name_list = self.log_name_list, throttle_s = 5)
            return False

        save_rate = save_rate_dict[data_product][0]
        if save_rate == 0.0:
            return False

        save_period = float(1) / float(save_rate)
        now = nepi_utils.get_time()
        elapsed = now - save_rate_dict[data_product][1]
        #self.msg_if.pub_debug("Checking should save: " + str([save_period,elapsed]), log_name_list = self.log_name_list, throttle_s = 5)
        snapshot = self.snapshot_dict[data_product]
        if (elapsed >= save_period or snapshot):
            #self.msg_if.pub_debug("Should save: " + data_product + " : " + str([save_period,elapsed]), log_name_list = self.log_name_list, throttle_s = 5)
            self.save_rate_dict = save_rate_dict
            return True 
        return False



    def data_product_snapshot_enabled(self, data_product):
        """Return whether a one-shot snapshot is pending for a data product.

        Args:
            data_product (str): Name of the data product to check.

        Returns:
            bool: True if a snapshot is pending, False if not or if the product
                is unrecognized.
        """
        try:
            enabled = self.snapshot_dict[data_product]  and self.disabled == False
            return enabled
        except:
            self.msg_if.pub_warn("Unknown snapshot data product " + data_product, log_name_list = self.log_name_list, throttle_s = 5)
            return False
        return False

    def data_product_snapshot_reset(self, data_product):
        """Clear the snapshot-pending flag for a data product after it has been saved.

        Args:
            data_product (str): Name of the data product to reset.

        Returns:
            bool: True if the flag was cleared, False if the product is unrecognized.
        """
        try:
            self.snapshot_dict[data_product] = False
            return True
        except:
            self.msg_if.pub_warn("Unknown snapshot data product " + data_product, log_name_list = self.log_name_list, throttle_s = 5)
            return False
        return False


    def data_products_should_save_dict(self):
        """Return a dictionary mapping each data product to whether it should save now.

        Combines the snapshot flag and the rate-based save logic for every
        registered data product.

        Returns:
            dict: Mapping of data product name to bool (True = should save now).
        """
        dps_dict=copy.deepcopy(self.snapshot_dict)
        for dp in dps_dict.keys():
            ss =  dps_dict[dp]
            sr = self.save_rate_dict[dp][0] > 0
            dps_dict[dp] = (ss or (self.save_data and sr)) and self.disabled == False
        return dps_dict


    def get_timestamp_string(self):
        """Return the current time as a formatted datetime string with milliseconds.

        Returns:
            str: Current datetime string in NEPI format including milliseconds.
        """
        return nepi_utils.get_datetime_str_now(add_ms = True, add_us = False)

    def get_filename_path_and_prefix(self):
        """Return the full save path joined with the current filename prefix.

        Returns:
            str: Combined path string, or empty string if the save path is None.
        """
        if self.save_path is None:
            return ""
        return os.path.join(self.save_path, self.read_write_if.get_filename_prefix())




    #***************************
    # NEPI data saving utility functions
    def save(self,data_product,data,timestamp = None, save_check=True, filename = None, key_name = None):
        """Save a data object for a named data product if conditions are met.

        Checks whether disk space is available, whether the product should save
        (rate-based or snapshot), and then delegates to ReadWriteIF.write_data_file.
        Resets the snapshot flag and updates the last-save timestamp after writing.

        Args:
            data_product (str): Name of the data product being saved.
            data: Data object to write. Type is inferred automatically.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            save_check (bool, optional): When True, the rate/snapshot logic gates
                the write. When False, the data is written unconditionally (if
                space is available). Defaults to True.
        """
        if self.space_available == False:
            self.data_product_snapshot_reset(data_product)
            return ''
        else:
            if self.disabled == False and self.filename_dict is not None:
                should_save = self.data_product_should_save(data_product)
                snapshot_enabled = self.data_product_snapshot_enabled(data_product)
                # Save data if enabled
                self.msg_if.pub_debug("******", log_name_list = self.log_name_list, throttle_s = 5)
                save_check = [should_save, snapshot_enabled, save_check]
                self.msg_if.pub_debug("Checking save checks: " + data_product + " " + str(save_check) , log_name_list = self.log_name_list, throttle_s = 5)
                if (should_save or snapshot_enabled or save_check == False):
                    if self.filename_dict['use_utc_tz'] == False:
                        timezone = self.timezone
                    else:
                        timezone = 'UTC'
                    self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone) , log_name_list = self.log_name_list, throttle_s = 5)
                    if should_save == False:
                        filename = None
                    filename = self.read_write_if.write_data_file(self.save_path, data, data_product, timezone = timezone, timestamp = timestamp, filename = filename, key_name = key_name)
                    self.data_product_snapshot_reset(data_product)
                    self.save_rate_dict[data_product][1] = nepi_utils.get_time()
                self.msg_if.pub_debug("Finished Checking save data: " + data_product , log_name_list = self.log_name_list, throttle_s = 5)
                self.msg_if.pub_debug("******", log_name_list = self.log_name_list, throttle_s = 5)
                return filename


    def create_filename_msg(self):
        """Build and return a FilenameConfig ROS message from the current filename dict.

        Returns:
            nepi_interfaces.msg.FilenameConfig: Populated FilenameConfig message
                reflecting the current naming settings.
        """
        fn_msg = FilenameConfig()
        try:
            fn_dict = self.filename_dict
            fn_msg.save_prefix = fn_dict['prefix']
            fn_msg.save_subfolder = fn_dict['subfolder']
            fn_msg.add_timestamp = fn_dict['add_timestamp']
            fn_msg.use_utc_tz = fn_dict['use_utc_tz']
            fn_msg.add_ms = fn_dict['add_ms']
            fn_msg.add_us = fn_dict['add_us']
            fn_msg.add_tz = fn_dict['add_tz']
        except:
            pass
        return fn_msg


    def publish_status(self):
        """Build and publish a SaveDataStatus message on the status topic.

        Compiles current save rates, filename config, data directory, timezone,
        and an example filename into the status message and latches it to the
        'status' publisher. No-ops if pub_status is False or node_if is None.
        """
        if self.node_if is not None and self.pub_status == True:
            save_rates_msg = []
            save_rate_dict = self.save_rate_dict

            #self.msg_if.pub_debug("Status pub save_rate_dict " + str(save_rate_dict), log_name_list = self.log_name_list, throttle_s = 5)
            for name in save_rate_dict.keys():
                save_rate_msg = SaveDataRate()
                save_rate_msg.data_product = name
                save_rate_msg.save_rate_hz = 0
                if self.disabled == False:
                    save_rate_msg.save_rate_hz = save_rate_dict[name][0]
                save_rates_msg.append(save_rate_msg)
                #self.msg_if.pub_debug("data_rates_msg " + str(save_rates_msg), log_name_list = self.log_name_list, throttle_s = 5)
            status_msg = SaveDataStatus()
            status_msg.node_name = self.node_name
            status_msg.save_data_topic = self.namespace
            status_msg.config_topic = self.config_topic
            status_msg.filename_config = self.create_filename_msg()
            status_msg.data_dir = self.save_path
            if self.filename_dict is not None:
                filename_dict = copy.deepcopy(self.filename_dict)
                try:
                    status_msg.filename_prefix = filename_dict['prefix']
                    status_msg.save_subfolder = filename_dict['subfolder']
                    status_msg.save_data_utc = filename_dict['use_utc_tz']
                except Exception as e:
                    self.msg_if.pub_warn("Failed to Publish filename dict: " + str(e), log_name_list = self.log_name_list, throttle_s = 5)
                #self.msg_if.pub_warn("Publishing filename dict: " + str(filename_dict), log_name_list = self.log_name_list, throttle_s = 5)

            status_msg.timezone = self.timezone
            status_msg.data_products = list(save_rate_dict.keys())
            status_msg.save_data_rates = save_rates_msg
            status_msg.save_data_enabled = self.save_data
            status_msg.disabled = self.disabled

            if self.all_save_namespace is None:
                status_msg.save_all_enabled = self.save_data
                # Guarded: publish_status runs on a 1 Hz timer, so a KeyError here would
                # silence the global status topic entirely.
                status_msg.save_all_rate = save_rate_dict.get('All',[0.0])[0]
            else:
                status_msg.save_all_enabled = self.save_all_enabled
                status_msg.save_all_rate = self.save_all_rate   


            if self.save_data_root_directory is not None:
                status_msg.data_dir = self.save_data_root_directory

            timezone = 'UTC'
            if self.filename_dict is not None:
                if self.filename_dict['use_utc_tz'] == False:
                    timezone = self.timezone

            #self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone) , log_name_list = self.log_name_list, throttle_s = 5)
            exp_filename = self.read_write_if.get_example_filename(timezone = timezone)
            status_msg.example_filename = exp_filename
            if self.node_if is not None:
                self.node_if.publish_pub(self.node_if_prefix +  'save_data_status_pub', status_msg)

    def init(self, do_updates = False):
        """Load save rate and filename parameters from the ROS param server and publish status.

        Reads the persisted save_rate_dict from the param server, merges it into the
        current dict (preserving last-save timers at zero), writes the merged dict back,
        and then publishes the current status.

        Args:
            do_updates (bool, optional): Reserved for future use; has no effect currently.
                Defaults to False.
        """
        #self.msg_if.pub_warn("Param updated save rate dict: " + str(self.save_rate_dict))
        if self.node_if is not None:
            # Prefixed keys, matching how the params are registered in PARAMS_DICT
            # and how set_save_rate()/update_filename_dict()/register_data_product()
            # write them back. get_param() returns None for a name it does not know,
            # so the unprefixed name wiped filename_dict on every config init, reset
            # and factory reset -- and save() no-ops entirely when filename_dict is
            # None, so nothing was ever written to disk.
            save_rate_dict = self.node_if.get_param(self.node_if_prefix + 'save_rate_dict')
            if save_rate_dict is not None:
                for data_product in self.save_rate_dict.keys():
                    if data_product in save_rate_dict.keys():
                        self.save_rate_dict[data_product][0] = save_rate_dict[data_product][0]
                    self.save_rate_dict[data_product][1] = 0.0 # Reset timer
            self.node_if.set_param(self.node_if_prefix + 'save_rate_dict',self.save_rate_dict)
            filename_dict = self.node_if.get_param(self.node_if_prefix + 'filename_dict')
            if filename_dict is not None:
                self.filename_dict = filename_dict
            disabled = self.node_if.get_param('disabled')
            if disabled is not None:
                self.disabled = disabled
        self.publish_status()


    def reset(self):
        """Reset parameters to their last-saved (user) values and reinitialize.

        Calls node_if.reset_params() to reload the user configuration tier, then
        reinitializes from the param server.
        """
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset parameters to factory defaults and reinitialize.

        Calls node_if.factory_reset_params() to restore factory values, then
        reinitializes from the param server.
        """
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Factory resetting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
        self.init(do_updates = True)

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        if self.node_if is not None:
            pass
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def updaterCb(self,timer):
        tzd = nepi_system.get_timezone()
        last_tz = copy.deepcopy(self.timezone)
        self.timezone = tzd
        self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)

        self.space_available = nepi_system.get_space_available()


    def _capabilitiesHandler(self, req):
        return_list = []
        save_rate_dict = self.save_rate_dict
        for d in save_rate_dict:
            return_list.append(SaveDataRate(data_product = d, save_rate_hz = save_rate_dict[d][0]))
        return SaveDataCapabilitiesQueryResponse(return_list)





    def _logNavPoseEnableCb(self, msg):
        self.msg_if.pub_info("Recieved Log NavPose Enable Update: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data



    def _saveAllStatusCb(self,msg):
        #self.msg_if.pub_warn("Recieved save All status msg: " + str(msg), log_name_list = self.log_name_list)
        data_product_list = msg.data_products
        if 'All' in data_product_list:
            self.save_all_enabled = msg.save_data_enabled
            index = data_product_list.index('All')
            self.save_all_rate = msg.save_data_rates[index].save_rate_hz

    def _disableCb(self, msg):
        #self.msg_if.pub_info("Recieved Disable Update: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data
        self.disable(enabled)

    def _saveEnableCb(self, msg):
        self.msg_if.pub_info("Recieved Enable Update: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data
        self.save_data_enable(enabled)
        

    def _saveRateCb(self, msg):
        self.msg_if.pub_info("Recieved Rate Update: " + str(msg), log_name_list = self.log_name_list)
        data_product = msg.data_product
        save_rate_hz = msg.save_rate_hz
        self.set_save_rate(data_product,save_rate_hz)
    
        
    def _setPrefixCb(self, msg):
        prefix = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        if filename_dict is not None:
            filename_dict['prefix'] = prefix
            self.update_filename_dict(filename_dict)


    def _setSubfolderCb(self, msg):
        subfolder = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        if filename_dict is not None:
            filename_dict['subfolder'] = subfolder
            self.update_filename_dict(filename_dict)


    def _setLocalTzCb(self, msg):
        use_utc = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        if filename_dict is not None:
            filename_dict['use_utc_tz'] = use_utc
            self.update_filename_dict(filename_dict)


    def _setFilenameCb(self, msg):
        filename_dict = nepi_utils.convert_msg2dict(msg)
        if self.filename_dict is not None:
            self.update_filename_dict(filename_dict)



    def _snapshotCb(self,msg):
        self.msg_if.pub_info("Recieved Snapshot Trigger", log_name_list = self.log_name_list)
        save_rate_dict = self.save_rate_dict
        if self.disabled == False:
            for data_product in save_rate_dict.keys():
                save_rate = save_rate_dict[data_product][0]
                enabled = (save_rate > 0.0)
                if enabled:
                    self.snapshot_dict[data_product] = True

    def _saveConfigCb(self,msg):
        if self.node_if is not None:
            self.node_if.save_config()

    def _resetSaveDataCb(self,msg):
        self.reset()

    def _publishStatusCb(self, timer):
        self.publish_status()

#######################################
# Transform3DIF

# Transform_List = [x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, heading_deg]



class Transform3DIF:

    ZERO_TRANSFORM = [0,0,0,0,0,0,0]
    # Class Vars ####################

    msg_if = None
    ready = False
    namespace = '~'

    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'transform_'

    transform = copy.deepcopy(ZERO_TRANSFORM)
    source = ''
    end = ''
    has_transform = True
    supports_updates = True

    status_msg = TransformStatus()
    
    #######################
    ### IF Initialization
    def __init__(self, 
                namespace = None,
                transform_name = 'transform',
                source_ref_description = '',
                end_ref_description = '',
                get_3d_transform_function = None,
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
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting Settings IF Initialization Processes", log_name_list = self.log_name_list)
        

        #############################
        # Initialize Class Variables
        transform_name = nepi_utils.get_clean_name(transform_name)
        if transform_name is None or transform_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(transform_name)) 
            return
        self.msg_if.pub_info("Using States Name: " + transform_name)
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,transform_name)
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

        self.source = source_ref_description
        self.end = end_ref_description
        self.get_3d_transform_function = get_3d_transform_function
        if self.get_3d_transform_function is not None:
            self.supports_updates = False

        self.status_msg.has_transform = self.has_transform
        self.status_msg.supports_updates = self.supports_updates
        ##############################  
        # Create NodeClassIF Class  
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }
        
        # Params Config Dict ####################
        self.PARAMS_DICT = {
            self.node_if_prefix + 'transform': {
                'name': 'transform',
                'namespace': self.namespace,
                'factory_val': self.transform
            },
            self.node_if_prefix + 'source': {
                'name': 'source',
                'namespace': self.namespace,
                'factory_val': self.source
            },
            self.node_if_prefix + 'end': {
                'name': 'end',
                'namespace': self.namespace,
                'factory_val': self.end
            }
        }

        # Services Config Dict ####################
        self.SRVS_DICT = None
        

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.node_if_prefix + 'transform_status_pub': {
                'namespace': self.namespace,
                'msg': TransformStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            },
            self.node_if_prefix + 'transform_pub': {
                'namespace': self.namespace,
                'msg': Transform,
                'topic': '',
                'qsize': 1,
                'latch': True
            }
        }

        # Subs Config Dict ####################

        if self.supports_updates == False:
            self.SUBS_DICT = None
        else:
            self.SUBS_DICT = {
                self.node_if_prefix + 'clear_navpose_frame_transform': {
                    'namespace': self.namespace,
                    'topic': 'clear_3d_transform',
                    'msg': Empty,
                    'qsize': 5,
                    'callback': self._clearFrame3dTransformCb, 
                    'callback_args': ()
                },
                self.node_if_prefix + 'set_navpose_frame_transform': {
                    'namespace': self.namespace,
                    'topic': 'set_3d_transform',
                    'msg': Transform,
                    'qsize': 5,
                    'callback': self._setFrame3dTransformCb,
                    'callback_args': ()
                },
                self.node_if_prefix + 'set_source_ref_description': {
                    'namespace': self.namespace,
                    'topic': 'set_source_ref',
                    'msg': String,
                    'qsize': 5,
                    'callback': self._setSourceRefCb,
                    'callback_args': ()
                }
        }

        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            if self.PARAMS_DICT is not None:
               self.node_if.add_params(self.PARAMS_DICT) 
            self.node_if.register_pubs(self.PUBS_DICT)
            self.node_if.register_subs(self.SUBS_DICT)
        else:
            self.config_topic = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = self.SRVS_DICT,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                                                )
   

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status()    
    
        nepi_sdk.start_timer_process(1.0, self._publishTransformCb)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

  
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the Transform3DIF.

        Returns:
            bool: True if initialization completed successfully, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the Transform3DIF is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace used by this Transform3DIF.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace

    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SRVS_DICT is not None:
                        for service_name in self.SRVS_DICT.keys():
                            self.node_if.unregister_service(service_name)
                self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)


    def get_zero_3d_transform(self):
        """Return the zero/identity 3D transform list.

        Returns:
            list: Seven-element list of zeros [x, y, z, roll, pitch, yaw, heading].
        """
        return copy.deepcopy(self.ZERO_TRANSFORM)

    def get_3d_transform(self):
        """Return the current 3D transform as a seven-element list.

        If a get_3d_transform_function was provided at construction, it is called
        first. Falls back to the internally stored transform if the function returns
        None.

        Returns:
            list: [x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, heading_deg].
        """
        transform = None
        blank_transform = copy.deepcopy(self.ZERO_TRANSFORM)
        if self.get_3d_transform_function is not None:
            transform = self.get_3d_transform_function()

        if transform is None:
            transform = self.transform
            if isinstance(transform, list) == False:
                transform = blank_transform
                self.transform = transform
            if len(transform) != len(blank_transform):
                transform = blank_transform
                self.transform = transform

        if isinstance(transform, list) == False:
                transform = blank_transform
        if len(transform) != len(blank_transform):
            transform = blank_transform

        return transform

    def get_3d_transform_msg(self):
        """Build and return the current 3D transform as a ROS Transform message.

        Returns:
            nepi_interfaces.msg.Transform: Transform message populated with the
                current transform values and source/end reference descriptions.
        """
        transform = self.get_3d_transform()
        transform_msg = nepi_nav.convert_transform_list2msg(transform,
                source_ref_description = self.source,
                end_ref_description = self.end)
        return transform_msg

    def get_3d_transform_dict(self):
        """Return the current 3D transform as a nepi_nav transform dict.

        Suitable for passing to nepi_nav.transform_navpose_dict() to apply the
        transform to a navpose dict.

        Returns:
            dict: BLANK_TRANSFORM_DICT-shaped dict populated from the current transform.
        """
        return nepi_nav.convert_transform_list2dict(self.get_3d_transform())

    def set_3d_transform(self,transform_list):
        """Set the 3D transform from a seven-element list and publish the update.

        No-ops if updates are not supported (i.e., a get_3d_transform_function was
        supplied at construction) or if the list does not have exactly seven elements.

        Args:
            transform_list (list): [x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg,
                heading_deg].
        """
        if self.supports_updates == True:
            if len(transform_list) == 7:
                self.transform = transform_list
                self.publish_transform()
                if self.node_if is not None:
                    self.node_if.set_param(self.node_if_prefix + 'transform',transform_list)

    def clear_3d_transform(self):
        """Reset the 3D transform to the zero/identity transform and publish the update.

        No-ops if updates are not supported (i.e., a get_3d_transform_function was
        supplied at construction).
        """
        if self.supports_updates == True:
            self.transform = copy.deepcopy(self.ZERO_TRANSFORM)
            self.publish_transform()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix + 'transform',self.transform)

    def set_has_transform(self,has_transform):
        """Set whether this interface reports having a valid transform.

        Args:
            has_transform (bool): True if a valid transform is available.
        """
        self.has_trasform = has_transform

    def get_has_transform(self):
        """Return whether this interface reports having a valid transform.

        Returns:
            bool: True if a valid transform is available, False otherwise.
        """
        return self.has_transform


    def get_source_description(self):
        """Return the source reference frame description string.

        Returns:
            str: Human-readable description of the source reference frame.
        """
        return self.source

    def set_source_description(self,source):
        """Set the source reference frame description and republish the transform.

        Args:
            source (str): Human-readable description of the source reference frame.
        """
        self.source = source
        self.publish_transform()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'source',source)

    def get_end_description(self):
        """Return the end (target) reference frame description string.

        Returns:
            str: Human-readable description of the end reference frame.
        """
        return self.end

    def set_end_description(self,end):
        """Set the end (target) reference frame description and republish the transform.

        Args:
            end (str): Human-readable description of the end reference frame.
        """
        self.end = end
        self.publish_transform()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'end',end)


    def publish_transform(self):
        """Build and publish the current transform as a ROS Transform message.

        Converts the current seven-element transform list to a ROS message and
        publishes it on the 'transform_pub' topic. No-ops if node_if is None.
        """
        transform = self.get_3d_transform()
        transform_msg = nepi_nav.convert_transform_list2msg(transform,
                                                source_ref_description = self.source,
                                                end_ref_description = self.end)
        if self.node_if is not None:
            self.node_if.publish_pub(self.node_if_prefix +  'transform_pub',transform_msg)

    def publish_status(self):
        """Publish the current TransformStatus message on the 'transform_status_pub' topic.

        No-ops if node_if is None.
        """
        if self.node_if is not None:
            self.status_msg.config_topic = self.config_topic
            self.status_msg.has_transform = self.has_transform
            self.node_if.publish_pub('transform_status_pub',self.status_msg)


    def init(self, do_updates = True):
        """Load transform, source, and end parameters from the ROS param server.

        Reads the persisted values and publishes the updated status.

        Args:
            do_updates (bool, optional): Reserved for future use; has no additional
                effect currently. Defaults to True.
        """
        if self.node_if is not None:
            # Prefixed keys, matching how the params are registered in PARAMS_DICT and
            # how set_3d_transform()/set_source_description()/set_end_description()
            # write them back. get_param() returns None for a name it does not know,
            # so the unprefixed names wiped the transform on every config init, reset
            # and factory reset -- saved 3D transforms never restored.
            transform = self.node_if.get_param(self.node_if_prefix + 'transform')
            if transform is not None:
                self.transform = transform
            source = self.node_if.get_param(self.node_if_prefix + 'source')
            if source is not None:
                self.source = source
            end = self.node_if.get_param(self.node_if_prefix + 'end')
            if end is not None:
                self.end = end
            #self.msg_if.pub_debug("Setting init values to param server values: " + str(self.init_settings_dict), log_name_list = self.log_name_list)
            if do_updates:
                pass
            self.publish_status()


    def reset(self):
        """Reset transform parameters to their last-saved values and reinitialize.

        Calls node_if.reset_params() to reload the user configuration tier, then
        reinitializes from the param server.
        """
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        self.init(do_updates = True)

    def factory_reset(self):
        """Reset transform parameters to factory defaults and reinitialize.

        Calls node_if.factory_reset_params() to restore factory values, then
        reinitializes from the param server.
        """
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_info("Factory resetting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
        self.init(do_updates = True)


    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)


    def _setFrame3dTransformCb(self, msg):
        self.msg_if.pub_info("Received Frame Transform update message: " + str(msg))
        transform_msg = msg
        # The stored transform is a 7-element signed list; fold the per-axis
        # invert flags into the sign so the stored/republished value is unambiguous.
        x = -transform_msg.x_m if transform_msg.x_invert else transform_msg.x_m
        y = -transform_msg.y_m if transform_msg.y_invert else transform_msg.y_m
        z = -transform_msg.z_m if transform_msg.z_invert else transform_msg.z_m
        roll = -transform_msg.roll_deg if transform_msg.roll_invert else transform_msg.roll_deg
        pitch = -transform_msg.pitch_deg if transform_msg.pitch_invert else transform_msg.pitch_deg
        yaw = -transform_msg.yaw_deg if transform_msg.yaw_invert else transform_msg.yaw_deg
        heading = -transform_msg.heading_deg if transform_msg.heading_invert else transform_msg.heading_deg
        transform = [x, y, z, roll, pitch, yaw, heading]
        self.set_3d_transform(transform)


    def _clearFrame3dTransformCb(self, msg):
        self.msg_if.pub_info("Recived Clear 3D Transform update message: ")
        self.clear_3d_transform()

    def _setSourceRefCb(self, msg):
        self.set_source_description(msg.data)

    def _publishTransformCb(self, timer):
        self.publish_transform()

    def _publishStatusCb(self, timer):
        self.publish_status()










############################################
# StatesIF


STATE_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]

EXAMPLE_STATES_DICT = {
                    "state_name": {
                        "value": False
                    }
}


class StatesIF:

    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'states_'
    

    ready = False
    msg_if = None
    namespace = '~'

    get_states_dict_function = None



    #######################
    ### IF Initialization
    def __init__(self,
                states_name = 'states',
                get_states_dict_function = None,
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
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting States IF Initialization Processes", log_name_list = self.log_name_list)
        
        #############################
        # Initialize Class Variables

        if get_states_dict_function is None:
            self.msg_if.pub_warn("get_states_dict_function can not be None") 
            return
        self.get_states_dict_function = get_states_dict_function

        # Create Namespace
        states_name = nepi_utils.get_clean_name(states_name)
        if states_name is None or states_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(states_name)) 
            return
        self.msg_if.pub_info("Using States Name: " + states_name)
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,states_name)

        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'

        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            self.node_if_prefix + 'states_query': {
                'namespace': self.namespace,
                'topic': 'system_states_query',
                'srv': SystemStatesQuery,
                'req': SystemStatesQueryRequest(),
                'resp': SystemStatesQueryResponse(),                
                'callback': self._statesQueryHandler
            }
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.node_if_prefix + 'states_status_pub': {
                'namespace': self.namespace,
                'msg': SystemStatesStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }

        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            self.node_if.register_services(self.SRVS_DICT)

        else:
            self.config_topic = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            services_dict = self.SRVS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                                                )


        success = nepi_sdk.wait()
        states_dict = self.get_states_dict_function()
        status_msg = SystemStatesStatus()
        status_msg.state_names = list(states_dict.keys())
        self.node_if.publish_pub(self.node_if_prefix + 'states_status_pub', status_msg)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the StatesIF.

        Returns:
            bool: True if initialization completed successfully, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the StatesIF is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready


    def get_namespace(self):
        """Return the ROS namespace used by this StatesIF.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace

    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SRVS_DICT is not None:
                        for service_name in self.SRVS_DICT.keys():
                            self.node_if.unregister_service(service_name)
                self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                # if self.node_if is not None:
                #     if self.PUBS_DICT is not None:
                #         for pub_name in self.PUBS_DICT.keys():
                #             self.node_if.unregister_pub(pub_name)

    ###############################
    # Class Private Methods
    ###############################

    def _statesQueryHandler(self, req):
        resp = SystemStatesQueryResponse()
        try:
            states_dict = self.get_states_dict_function()
            resp = nepi_states.create_states_msg(states_dict)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create resp msg: " + str(e), log_name_list = self.log_name_list)
        return resp





EXAMPLE_TRIGGER_DICT = {
                    "name":"None",
                    "node_name": '~',
                    "description": "None",
                    "data_str_list":["None"],
                    "time":nepi_utils.get_time() 
}


EXAMPLE_TRIGGERS_DICT = {
                "trigger_name": {
                    "name":"None",
                    "node_name": '~',
                    "description": "None",
                    "data_str_list":["None"],
                    "time":nepi_utils.get_time() 
                    }

}



class TriggersIF:

    node_if = None
    node_if_shared = False
    config_topic = ''
    node_if_prefix = 'triggers_'

    msg_if = None
    ready = False
    namespace = '~'

    triggers_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                triggers_dict = None,
                triggers_name = 'triggers',
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
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting Triggers IF Initialization Processes", log_name_list = self.log_name_list)
        
        #############################
        # Initialize Class Variables

        if triggers_dict is None:
            self.triggers_dict = dict()
        else:
            self.triggers_dict = triggers_dict

        # Create Namespace
        triggers_name = nepi_utils.get_clean_name(triggers_name)
        if triggers_name is None or triggers_name == '':
            self.msg_if.pub_warn("Name Not Valid: " + str(triggers_name)) 
            return
        self.node_if_prefix = self.namespace.replace(self.base_namespace + '/','').replace('/','_') + '_'
        ##############################  
        # Create NodeClassIF Class  


        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.node_if_prefix + 'trigger_pub': {
                'msg': SystemTrigger,
                'namespace': self.base_namespace,
                'topic': 'system_triggers',
                'qsize': 1,
                'latch': False
            }
        }

        # Udpate or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            self.node_if.register_services(self.SRVS_DICT)
            self.node_if.register_pubs(self.PUBS_DICT)
        else:
            self.config_topic = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            pubs_dict = self.PUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                                                )

        success = nepi_sdk.wait()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################



    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the TriggersIF.

        Returns:
            bool: True if initialization completed successfully, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the TriggersIF is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace used by this TriggersIF.

        Returns:
            str: The fully-resolved ROS namespace string.
        """
        return self.namespace

    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.SRVS_DICT is not None:
                        for service_name in self.SRVS_DICT.keys():
                            self.node_if.unregister_service(service_name)
                self.service_name = None

                if self.SUBS_DICT is not None:
                        for sub_name in self.SUBS_DICT.keys():
                            self.node_if.unregister_sub(sub_name)
                self.SUBS_DICT = None

                if self.node_if is not None:
                    if self.PUBS_DICT is not None:
                        for pub_name in self.PUBS_DICT.keys():
                            self.node_if.unregister_pub(pub_name)



    def publish_trigger(self, trigger_dict):
        """Build and publish a system trigger message on the 'system_triggers' topic.

        Converts the trigger dictionary to a SystemTrigger ROS message and publishes
        it via the node_if. No-ops if node_if is None.

        Args:
            trigger_dict (dict): Trigger event dictionary with keys 'name',
                'node_name', 'description', 'data_str_list', and 'time'.
        """
        trig_msg = nepi_triggers.create_trigger_msg(self.namespace, trigger_dict)
        if self.node_if is not None:
            self.node_if.publish_pub(self.node_if_prefix +  'trigger_pub',trig_msg)


    ###############################
    # Class Private Methods
    ###############################



