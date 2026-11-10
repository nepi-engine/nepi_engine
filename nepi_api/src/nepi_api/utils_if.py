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

os.environ['EGL_PLATFORM'] = 'surfaceless'   # Ubuntu 20.04+
import open3d as o3d


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
# nepi_pc (imports open3d) must come before nepi_img (static-TLS crash on aarch64/glibc 2.31)
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_img
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

# ListIFStatus.msg was renamed SelectorStatus.msg in nepi_interfaces (776c92c, same
# fields). The old name no longer exists, so importing it made this module unimportable.
from nepi_interfaces.msg import SelectorStatus as ListIFStatus
from nepi_interfaces.msg import ControlsStatus, UpdateControl
from nepi_interfaces.msg import DataStatus

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeIF




##########################################
## ListIF

BLANK_LIST_DICT = {
    'ordered_items_list': [],
    'ordered_names_list': [],
    'active_items_list': [],
    'selected_item': None,
    'selected_item_index': -1
}
   

BLANK_CALLBACK_DICT = {
    'itemSelectCb': None,
    'listUpdatedCb': None,
    'refreshListCb': None
}

class ListIF:
    list_dict = copy.deepcopy(BLANK_LIST_DICT)
    reset_list_dict = copy.deepcopy(BLANK_LIST_DICT)
    callback_dict = copy.deepcopy(BLANK_CALLBACK_DICT)

    multi_select_enabled = False
    update_order_enabled = False

    ### IF Initialization
    def __init__(self, 
                namespace = None,
                list_dict = None,
                callback_dict = None,
                multi_select_enabled = False,
                update_order_enabled = False,
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

        self.msg_if.pub_info("Starting List IF Initialization Processes" )
        ############################## 
        # Initialize Class Variables
        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.node_namespace,'list')
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.msg_if.pub_warn("Using list namespace: " + self.namespace )
        

        self.reset_list_dict = self.update_list_dict(list_dict)

        if callback_dict is None:
            callback_dict = copy.deepcopy(BLANK_CALLBACK_DICT)
        else:
            for key in self.callback_dict():
                if key not in callback_dict:
                    callback_dict[key] = self.callback_dict[key]
        self.callback_dict = callback_dict
        

        self.multi_select_enabled = multi_select_enabled
        self.update_order_enabled = update_order_enabled

       
        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'msg': ListIFStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'select_item': {
                'namespace': self.namespace,
                'topic': 'select_item',
                'msg': String,
                'qsize': 10,
                'callback': self.selectItemCb, 
                'callback_args': ()
            },
            'reset': {
                'namespace': self.namespace,
                'topic': 'reset',
                'msg': Empty,
                'qsize': 10,
                'callback': self.resetCb, 
                'callback_args': ()
            },
            'refresh': {
                'namespace': self.namespace,
                'topic': 'refresh',
                'msg': Empty,
                'qsize': 10,
                'callback': self.refreshCb, 
                'callback_args': ()
            }
        }

        if self.multi_select_enabled == True:
            self.SUBS_DICT['enable_all'] = {
                'namespace': self.namespace,
                'topic': 'enable_all',
                'msg': Empty,
                'qsize': 10,
                'callback': self.enableAllCb, 
                'callback_args': ()
            }
            self.SUBS_DICT['disable_all'] = {
                'namespace': self.namespace,
                'topic': 'disable_all',
                'msg': Empty,
                'qsize': 10,
                'callback': self.disableAllCb, 
                'callback_args': ()
            }
            self.SUBS_DICT['update_state'] = {
                'namespace': self.namespace,
                'topic': 'update_state',
                'msg': UpdateBool,
                'qsize': 10,
                'callback': self.updateBoolCb, 
                'callback_args': ()
            },
        if self.order_updates_enabled == True:
            self.SUBS_DICT['update_order'] = {
                'namespace': self.namespace,
                'topic': 'update_order',
                'msg': UpdateOrder,
                'qsize': 10,
                'callback': self.updateOrderCb, 
                'callback_args': ()
            }


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        
                        msg_if = self.msg_if
                                            )

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.publish_status()
        
        self.updater = nepi_sdk.start_timer_process(1, self.statusPub)
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Returns the ready state of the ListIF instance.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Blocks until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to
                float('inf'), which waits indefinitely.

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
        """
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

    def statusPubCb(self,timer):
        self.publish_status


    def publish_status(self):
        """Publishes the current list state as a ListIFStatus message on the status topic.

        Constructs a ListIFStatus message from the current list_dict contents and
        publishes it via the node interface publisher. Has no effect if the node
        interface has not been initialized.
        """
        if self.node_if is not None:
            status_msg = ListIFStatus()
            status_msg.ordered_items_list = self.self.list_dict['ordered_items_list']
            status_msg.ordered_names_list = self.list_dict['ordered_names_list']
            status_msg.active_items_list = self.list_dict['active_items_list']
            status_msg.selected_item = self.list_dict['selected_item']    
            status_msg.selected_item = self.self.list_dict['selected_item_index']
            status_msg.multi_select_enabled = self.multi_select_enabled
            status_msg.update_order_enabled = self.update_order_enabled
            self.node_if.publish_pub('status_pub', status_msg)


    def get_list_dict(self):
        """Returns the current list dictionary containing all list state.

        Returns:
            dict: A dictionary with keys ``ordered_items_list``, ``ordered_names_list``,
                ``active_items_list``, ``selected_item``, and ``selected_item_index``
                reflecting the current state of the managed list.
        """
        return self.list_dict

    def update_list_dict(self, list_dict):
        """Merges a provided list dictionary into the current list state and returns it.

        Fills in any missing keys from the existing list_dict, reconciles the
        selected item and active items lists based on the current multi-select
        mode, and ensures the ordered_names_list length matches ordered_items_list.

        Args:
            list_dict (dict or None): Dictionary with list state to apply. Accepted
                keys are ``ordered_items_list``, ``ordered_names_list``,
                ``active_items_list``, ``selected_item``, and ``selected_item_index``.
                Pass None to reset to a blank list dictionary.

        Returns:
            dict: The updated internal list dictionary after applying the provided values.
        """
        if list_dict is None:
            list_dict = copy.deepcopy(BLANK_LIST_DICT)
        else:
            for key in self.list_dict():
                if key not in list_dict:
                    list_dict[key] = self.list_dict[key]

        ########
        if selected_item is not None:
            if selected_item in list_dict['ordered_items_list']:
                self.list_dict['selected_item'] = selected_item
                self.self.list_dict['selected_item_index'] = self.list_dict['ordered_items_list'].index(selected_item)
        ########
        if self.multi_select_enabled == False:
            if self.list_dict['selected_item'] in self.self.list_dict['ordered_items_list']:
                self.list_dict['active_items_list'] = [self.list_dict['selected_item']]
        else:
            for item in active_items_list:
                if item in self.self.list_dict['ordered_items_list']:
                    self.list_dict['active_items_list'].append(item)

        ######        
        ordered_names_list = list_dict['ordered_items_list']
        if ordered_names_list is None:
            list_dict['ordered_names_list'] = list_dict['ordered_items_list']
        elif len(ordered_names_list) != len(list_dict['ordered_items_list']):
            list_dict['ordered_names_list'] = list_dict['ordered_items_list']
        else:
            list_dict['ordered_names_list'] = ordered_names_list

        self.list_dict = list_dict
        return self.list_dict

    def signalSelect(self,item):
        if self.callback_dict['itemSelectCb'] is not None:
            active = item in self.list_dict['active_items_list']
            self.callback_dict['itemSelectCb'](item,active)  

    def signalUpdate(self):
        if self.callback_dict['listUpdatedCb'] is not None:
            self.callback_dict['listUpdatedCb'](self.get_list_dict())

      

    ###################
    ## List Mgr Callbacks
    def enableAllCb(self,msg):
        self.msg_if.pub_info("Got Update enable all msg: " + str(msg))
        self.list_dict['active_items_list'] = self.self.list_dict['ordered_items_list']
        self.signalUpdate()
        self.publish_status()

    def disableAllCb(self,msg):
        self.msg_if.pub_info("Got disable all msg: " + str(msg))
        self.list_dict['active_items_list'] = []
        self.signalUpdate()
        self.publish_status()


    def selectItemCb(self,msg):
        self.msg_if.pub_info("Got select msg: " + str(msg))
        item = msg.data
        self.selectItem(item)

    def selectItem(self,item):
        self.list_dict['ordered_items_list'] = copy.deepcopy(self.self.list_dict['ordered_items_list'])
        if item in self.list_dict['ordered_items_list']:
            self.list_dict['selected_item'] = item
            self.self.list_dict['selected_item_index'] = self.list_dict['ordered_items_list'].index(selected_item)
            if self.multi_select_enabled == False:
                self.list_dict['active_items_list'] = [item]
            self.signalSelect(item)
        self.publish_status()

    def updateBoolCb(self,msg):
        self.msg_if.pub_info("Got update state msg: " + str(msg))
        item = msg.name
        active_state = msg.value
        self.signalSelect(item)
        self.update_state(item,active_state)

    def update_state(self, item, active_state):
        """Adds or removes an item from the active items list when multi-select is enabled.

        Only takes effect when multi_select_enabled is True. If active_state is True
        and the item is in the ordered list but not yet active, it is appended to
        active_items_list. If active_state is False and the item is currently active,
        it is removed. Publishes an updated status message after any change.

        Args:
            item (str): The item identifier to activate or deactivate.
            active_state (bool): True to mark the item as active, False to deactivate it.
        """
        if self.multi_select_enabled == True:
            self.msg_if.pub_info("Updateding " + item + " state: " + str(active_state))
            self.list_dict['ordered_items_list'] = copy.deepcopy(self.self.list_dict['ordered_items_list'])
            active_items_list = copy.deepcopy(self.list_dict['active_items_list'])
            if active_state == False and item in self.list_dict['ordered_items_list'] and item in active_items_list:
                active_items_list.remove(item)
                #self.msg_if.pub_warn("Update state dict: " + str(self.list_dict['ordered_items_list'][item]))
                self.list_dict['active_items_list'] = active_items_list
                self.signalUpdate()
                self.publish_status()
            if active_state == True and item in self.list_dict['ordered_items_list'] and item not in active_items_list:
                active_items_list.append(item)
                #self.msg_if.pub_warn("Update state dict: " + str(self.list_dict['ordered_items_list'][item]))
                self.list_dict['active_items_list'] = active_items_list
                self.signalUpdate()
            self.publish_status()


    def updateOrderCb(self,msg):
        self.msg_if.pub_info("Got Update order msg: " + str(msg))
        item = msg.name
        move_cmd = msg.move_cmd
        moveFunction = self.getOrderUpdateFunction(move_cmd)
        if item in self.self.list_dict['ordered_items_list']:
            self.list_dict['ordered_items_list'] = copy.deepcopy(self.self.list_dict['ordered_items_list'])
            ordered_names_list = copy.deepcopy(self.list_dict['ordered_names_list'])
            [self.list_dict['ordered_items_list'],ordered_names_list] = moveFunction(item,self.list_dict['ordered_items_list'],ordered_names_list)
            self.self.list_dict['ordered_items_list'] = self.list_dict['ordered_items_list']
            self.list_dict['ordered_names_list'] = ordered_names_list
            self.signalUpdate()
        self.publish_status()
        


    def getOrderUpdateFunction(self,move_cmd):
        if move_cmd == 'top':
            updateFunction = nepi_drvs.moveDriverTop
        elif move_cmd == 'bottom':
            updateFunction = nepi_drvs.moveDriverBottom
        elif move_cmd == 'up':
            updateFunction = nepi_drvs.moveDriverUp
        elif move_cmd == 'down':
            updateFunction = nepi_drvs.moveDriverDown
        else:
            updateFunction = self.moveDriverNone
        return updateFunction

    
    def resetCb(self,msg):
        self.list_dict = copy.deepcopy(self.reset_list_dict)
        self.list_dict['ordered_names_list'] = self.reset_ordered_names_list
        self.list_dict['active_items_list'] = self.reset_active_items_list
        self.list_dict['selected_item'] = self.reset_selected_item    
        self.list_dict['selected_item'] = self.reset_selected_item_index
        self.signalUpdate()
        self.publish_status()

    def refreshCb(self,msg):
        if self.callback_dict['refreshListCb'] is not None:
            self.callback_dict['refreshListCb']()
    
    




#########################################
# Controls IF Class
#########################################


class ControlsIF(NodeIF):


    #########################
    ## NODE IF Required
    ## Class Variables Updated by NodeIF Init Class
    #########################

    STATUS_MSG = ControlsStatus()
    DATA_MSG = None

    ID = 'controls'
    DISPLAY_NAME = 'Controls'
    DESCRIPTION = 'Controls'

    IF_DICT = dict()

    #########################
    ## CUSTOM
    #########################
    msg_if = None
    node_if = None

    ready = False

    controls_dict = dict()

    save_params = True

    def __init__(self,
                name = 'controls',
                namespace = None,
                controls_name = None,
                controls_display_name = None,
                controls_description = None,
                controls_init_dict = dict(),
                save_params = True,
                controls_updated_callback = None, # if not None: Calls function with with control_name when msg is recieved, after changine controls_dict and publishing status
                info_dict = dict(),
                msg_if = None,
                node_if = None,
                
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.IF_DICT = dict(self.IF_DICT)

               
        ##############################
        # Update Class Variables
        ##############################  

        name = nepi_utils.get_clean_name(name)
        if controls_name is not None:
            controls_name = nepi_utils.get_clean_name(controls_name)
            if controls_name.strip() != '':
                name = controls_name

        if controls_display_name is not None:
            self.DISPLAY_NAME = controls_display_name
        self.DISPLAY_NAME = self.DISPLAY_NAME.replace('_','').capitalize()

        if controls_description is not None:
            self.DESCRIPTION = controls_description


        self.controls_dict = nepi_controls.create_controls_dict(controls_init_dict,
                             name = name, display_name = self.DISPLAY_NAME, description = self.DESCRIPTION)
        self.save_params = save_params        


        #########################
        # Update status message for values not in NodeIF Dictionaries
        ##############################



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
        update_info_dict = copy.deepcopy(info_dict)
        update_info_dict.update(if_info_dict)
        ##################        
        update_caps_dict = dict()
        ##################        
        update_config_control_dict = dict()
        ##################       
        update_node_controls_dict = dict()
        ##################
        update_stats_dict = dict()
        ##################
        update_rui_dict = dict()
        ##################
        update_callback_dict = dict(
            controls_updated = controls_updated_callback
        )


        ###########
        # NODE IF SETUP

       # Call the parent class constructor
        super().__init__(id = self.ID,
                name = name,
                namespace = namespace,
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


       #######################
        # Add Services
        #######################
        if self.save_params == True:
            # The {control_name: value} form, which is what save_params_dict() writes
            # and init() reads back, not the full controls dict
            params_dict = {
                'controls_dict': {
                    'name': 'controls_dict',
                    'namespace': self.namespace,
                    'factory_val': nepi_controls.get_params_dict(self.controls_dict)
                },
            }
        else:
            params_dict = {}

        self.add_params(params_dict)

        #######################
        # Add Services
        #######################
        services_dict = {}
        self.add_services(services_dict)

        #######################
        # Add Pubs 
        #######################
        pubs_dict = {}

        self.add_pubs(pubs_dict)

        #######################
        # Add Subs 
        #######################
        subs_dict = {
            #####################
            # Control Subs
            ####################
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

        self.add_subs(subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################


        #########################
        # Update status message for values not in NodeIF Dictionaries
        ##############################

        ##############################
        # Start Node Processes
        ##############################

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is ControlsIF:
            self.start()

    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_status_msg = nepi_controls.convert_controls_dict2msg(controls_dict)
        status_msg.controls_name_list = controls_status_msg.controls_name_list
        status_msg.controls_msg_list = controls_status_msg.controls_msg_list

        return status_msg

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

            param_name = 'controls_dict'
            controls_params_dict = nepi_controls.get_params_dict(self.controls_dict)

            controls_params_dict = self.get_param(param_name)
            if controls_params_dict is not None:
                for control_name in controls_params_dict.keys():
                    # Skip the meta keys and unwrap full control entries, which a param
                    # saved from the whole controls dict still holds
                    if control_name in nepi_controls.CONTROLS_META_KEYS:
                        continue
                    control_value = controls_params_dict[control_name]
                    if isinstance(control_value, dict):
                        control_value = control_value.get('value', None)
                    if control_value is not None:
                        self.controls_dict = nepi_controls.set_value(self.controls_dict, control_name, control_value)

        if do_updates == True:
            pass
        self.publish_status()

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def resetCb(self, do_updates = True):
        self.controls_dict = nepi_controls.reset_values(self.controls_dict)

    def factoryResetCb(self, do_updates = True):
        self.controls_dict = nepi_controls.reset_values(self.controls_dict)
    
    #######################
    # Class Public Methods
    #######################


    ##################
    # Controls Functions

    def remove_controls(self, controls_names):
        for name in controls_names:
            try:
                del self.controls_dict[name]
            except:
                pass
        self.publish_status()
        self.save_params_dict()

    def add_controls(self, init_controls_dict):
        add_controls_dict = nepi_controls.create_controls_dict(init_controls_dict,
                             id = self.controls_dict['controls_id'], name = self.controls_dict['controls_name'],
                             display_name = self.controls_dict['controls_display_name'], description = self.controls_dict['controls_description'])
        self.controls_dict.update(add_controls_dict)
        self.publish_status()
        self.save_params_dict()

    def get_controls_dict(self):
        controls_dict = copy.deepcopy(self.controls_dict)
        return controls_dict

    def has_control(self, control_name):
        has_control = control_name in self.controls_dict.keys()
        return has_control

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
                    if self.callback_dict['controls_updated'] is not None:
                        self.callback_dict['controls_updated'](control_name, update_value)
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
        self.controls_dict = nepi_controls.set_max_bound(controls_dict, control_name, max_bound = max_bound)
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
            param_name = 'controls_dict'
            # Param only. set_value also setattr()s any name this class declares,
            # and controls_dict is one, so it replaced the live controls dict with
            # this flat {control_name: value} form -- the next status publish then
            # indexed a str/float/int as a control entry and dropped every control
            # of the set. set_param applies the same node_if_prefix set_value does.
            self.set_param(param_name, params_dict)


    ##################
    # Misc Functions

    def get_status_pub_msg(self):
        return self.get_status_msg()


    ###############################
    # Class Private Methods
    ###############################


    def _updateControlCb(self,msg):
        self.msg_if.pub_debug("Received control update msg: " + str(msg) )
        control_name = msg.name
        # apply_update_msg writes through the dict it is handed, so passing
        # self.controls_dict applied the update before set_control_value ever
        # ran -- its "did anything change" test then compared the new dict with
        # itself, and no status, callback or param save came out of an update.
        controls_dict = copy.deepcopy(self.controls_dict)
        controls_dict = nepi_controls.apply_update_msg(controls_dict, msg)
        control_value = nepi_controls.get_value(controls_dict, control_name )
        self.set_control_value(control_name, control_value)

    def _resetControlsCb(self,msg):
        self.msg_if.pub_debug("Received reset controls msg" )
        if self.node_if is not None:

            param_name = 'controls_dict'

            self.node_if.reset_params([param_name])
            controls_params_dict = self.get_param(param_name)
            if controls_params_dict is not None:
                for control_name in controls_params_dict.keys():
                    # Skip the meta keys and unwrap full control entries, which a param
                    # saved from the whole controls dict still holds
                    if control_name in nepi_controls.CONTROLS_META_KEYS:
                        continue
                    control_value = controls_params_dict[control_name]
                    if isinstance(control_value, dict):
                        control_value = control_value.get('value', None)
                    if control_value is not None:
                        self.controls_dict = nepi_controls.set_value(self.controls_dict, control_name, control_value)
        self.publish_status()
           
        

    def _setHiddenValueCb(self,msg):
            self.set_control_hidden(msg.name, msg.value)

    def _setControlsHiddenCb(self,msg):
            self.sets_hidden(msg.value)

    def _setOrderValueCb(self,msg):
            self.set_control_display_order(msg.name, msg.value)

    def _setOrderTopCb(self,msg):
            self.move_control_display_top(msg.name)

    def _setOrderBottomCb(self,msg):
            self.move_control_display_bottom(msg.name)

    def _setOrderDownCb(self,msg):
            self.move_control_display_down(msg.name)

    def _publishStatusCb(self,timer):
            self.publish_status()




#########################################
# Data IF Class
#########################################


class DataIF(NodeIF):

    #########################
    ## NODE IF Required
    ## Class Variables Updated by NodeIF Init Class
    #########################

    STATUS_MSG = DataStatus()
    DATA_MSG = None

    ID = 'data'
    DISPLAY_NAME = 'Data'
    DESCRIPTION = 'Data'

    IF_DICT = dict()

    #########################
    ## CUSTOM
    #########################
    msg_if = None
    node_if = None

    ready = False

    data_dict = dict()


    #######################
    ### IF Initialization
    def __init__(self, 
                name = 'data',
                namespace = None,
                data_name = None,
                data_display_name = None,
                data_description = None,
                data_init_dict = dict(),
                info_dict = dict(),
                msg_if = None,
                node_if = None,
                
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.IF_DICT = dict(self.IF_DICT)


        ##############################
        # Update Class Variables
        ##############################

        name = nepi_utils.get_clean_name(name)
        if data_name is not None:
            data_name = nepi_utils.get_clean_name(data_name)
            if data_name.strip() != '':
                name = data_name

        if data_display_name is not None:
            self.DISPLAY_NAME = data_display_name
        self.DISPLAY_NAME = self.DISPLAY_NAME.replace('_','').capitalize()

        if data_description is not None:
            self.DESCRIPTION = data_description


        self.data_dict = nepi_data.create_data_dict(data_init_dict,
                             name = name, display_name = self.DISPLAY_NAME, description = self.DESCRIPTION)
   

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
        update_info_dict = copy.deepcopy(info_dict)
        update_info_dict.update(if_info_dict)
        ##################        
        update_caps_dict = dict()
        ##################        
        update_config_control_dict = dict()
        ##################       
        update_node_controls_dict = dict()
        ##################
        update_stats_dict = dict()
        ##################
        update_rui_dict = dict()
        ##################
        update_callback_dict = dict()

        ###########
        # NODE IF SETUP

        # Call the parent class constructor
        super().__init__(id = self.ID,
                name = name,
                namespace = namespace,
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


       #######################
        # Add Services
        #######################
       
        params_dict = {}
        self.add_params(params_dict)

        #######################
        # Add Services
        #######################
        services_dict = {}
        self.add_services(services_dict)

        #######################
        # Add Pubs 
        #######################
        pubs_dict = {}
        self.add_pubs(pubs_dict)

        #######################
        # Add Subs 
        #######################
        subs_dict =  {}
        self.add_subs(subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################


        #########################
        # Update status message for values not in NodeIF Dictionaries
        ##############################


        ##############################
        # Start Node Processes
        ##############################

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is DataIF:
            self.start()



    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):
        data_dict = copy.deepcopy(self.data_dict)
        data_status_msg = nepi_data.convert_data_dict2msg(data_dict)
        status_msg.data_name_list = data_status_msg.data_name_list
        status_msg.data_msg_list = data_status_msg.data_msg_list
        return status_msg

    #######################
    # Class Public Methods
    #######################


    ##################
    # Data Functions

    def remove_data(self, data_names):
        for name in data_names:
            try:
                del self.data_dict[name]
            except:
                pass
        self.publish_status()

    def add_data(self, init_data_dict):
        add_data_dict = nepi_data.create_data_dict(init_data_dict,
                             id = self.data_dict['data_id'], name = self.data_dict['data_name'],
                             display_name = self.data_dict['data_display_name'], description = self.data_dict['data_description'])
        self.data_dict.update(add_data_dict)
        self.publish_status()

    def get_data_dict(self):
        data_dict = copy.deepcopy(self.data_dict)
        return data_dict

    def has_datum(self, datum_name):
        has_datum = datum_name in self.data_dict.keys()
        return has_datum

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
        self.data_dict = nepi_data.set_max_bound(data_dict, datum_name, max_bound = max_bound)
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

    def get_status_pub_msg(self):
        return self.get_status_msg()


 

    ###############################
    # Class Private Methods
    ###############################


    def _updateDatumCb(self,msg):
        self.msg_if.pub_debug("Received datum update msg: " + str(msg) )
        datum_name = msg.name
        # Same fix as ControlsIF._updateControlCb: apply_update_msg writes
        # through the dict it is handed.
        data_dict = copy.deepcopy(self.data_dict)
        data_dict = nepi_data.apply_update_msg(data_dict, msg)
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



    #######################
    ### IF Initialization
    def __init__(self,
                filename_dict = None,
                node_name = None,
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
        self.msg_if.pub_debug("Starting IF Initialization Processes" )
        



        #############################
        # Initialize Class Variables

        if node_name is not None:
            self.node_name = node_name.replace(' ','-')
        self.msg_if.pub_debug("Using node_name: " + self.node_name )

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



        #########################
        # Update status message for values not in NodeIF Dictionaries
        ##############################


        ##############################
        # Start Node Processes
        ##############################

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ###############################


    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):
        return status_msg

    ###############################
    # Class Public Methods
    ###############################


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
                #self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone) , throttle_s = 5.0)
                filename = save_function(filepath, data, data_name, timestamp = timestamp, timezone = timezone, filename = filename, key_name = key_name)
                found_type = True
        if found_type == False:
            self.msg_if.pub_debug("Data type not supported: " + str(data_type) + ' for data name: ' + str(data_name) + ' with data: ' + str(data) , throttle_s = 5.0)
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
            self.msg_if.pub_debug("File type not supported: " + ext_str + " : " + str(file_types) )
        else:
            file_path = os.path.join(filepath,filename)
            try:
                data = nepi_utils.read_yaml_2_dict(file_path)
            except:
                self.msg_if.pub_debug("Failed to read file: " + file_path )
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
            self.msg_if.pub_debug("Data type not supported: " + str(data_type) )
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_debug("File type not supported: " + ext_str + " : " + str(file_types) )
            else:
                if filename is None:
                    filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    try:
                        success = nepi_utils.add_dict_2_yaml(file_path, data, key_name = key_name)
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5 )
                else:
                    try:
                        data_to_write = {key_name: data} if key_name is not None else data
                        success = nepi_utils.write_dict_2_yaml(file_path, data_to_write)
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5 )
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
            self.msg_if.pub_debug("File type not supported: " + ext_str + " : " + str(file_types) )
        else:
            file_path = os.path.join(filepath,filename)
            try:
                if ext_str == 'npy':
                    data = np.genfromtxt(file_path, delimiter=',', dtype=float, filling_values=-999)
                else:
                    data = np.genfromtxt(file_path, delimiter=',', dtype=float, filling_values=-999)
            except:
                self.msg_if.pub_debug("Failed to read file: " + file_path )
                
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
            self.msg_if.pub_debug("Data type not supported: " + str(type(data)) )
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_debug("File type not supported: " + ext_str + " : " + str(file_types) )
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_debug("File already exists: " + file_path )
                else:
                    try:
                        if ext_str == 'npy':
                            file_path = file_path.replace('.' + ext_str, '')
                            success = np.save(file_path, data)
                        else:
                            success = np.savetxt(file_path, data)
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5 )

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
            self.msg_if.pub_debug("File type not supported: " + ext_str + " : " + str(file_types) )
        else:
            file_path = os.path.join(filepath,filename)
            try:
                data = nepi_img.read_image_file(file_path)
            except:
                self.msg_if.pub_debug("Failed to read file: " + file_path )
                
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
            self.msg_if.pub_debug("Data type not supported: " + str(type(data)) )
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_debug("File type not supported: " + ext_str + " : " + str(file_types) )
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_debug("File already exists: " + file_path )
                else:
                    try:
                        success = nepi_img.write_image_file(file_path, data)
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5 )
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
            self.msg_if.pub_debug("File type not supported: " + ext_str + " : " + str(file_types) )
        else:
            file_path = os.path.join(filepath,filename)
            try:
                data = nepi_pc.read_pointcloud_file(file_path)
            except:
                self.msg_if.pub_debug("Failed to read file: " + file_path )
                
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
            self.msg_if.pub_debug("Data type not supported: " + str(data_type) )
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_debug("File type not supported: " + ext_str + " : " + str(file_types) )
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_debug("File already exists: " + file_path )
                else:
                    try:
                        success = nepi_pc.write_pointcloud_file(file_path,data)
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to save data type: " + data_key + " to " + file_path + str(e) , throttle_s = 5 )
                    
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
                




