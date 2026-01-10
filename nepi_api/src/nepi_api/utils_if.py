#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import time 
import copy


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_interfaces.msg import ListIFStatus

from nepi_api.messages_if import MsgIF




class ListIF:
    ordered_items_list = []
    ordered_names_list = []
    active_items_list = []
    selected_item = None  
    selected_item_index = -1
    

    reset_ordered_items_list = []
    reset_ordered_names_list = []
    reset_active_items_list = []
    reset_selected_item = None    
    reset_selected_item = -1

    itemSelectCb = None
    listUpdatedCb = None
    refreshListCb = None

    multi_select_enabled = False
    update_order_enabled = False

    ### IF Initialization
    def __init__(self, 
                namespace = None,
                ordered_items_list = [],
                ordered_names_list = None,
                active_items_list = [],
                selected_item = None,
                itemSelectCb = None,
                listUpdatedCb = None,
                refreshListCb = None,
                multi_select_enabled = False,
                update_order_enabled = False,
                log_name = None,
                log_name_list = [],
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
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)

        self.msg_if.pub_info("Starting List IF Initialization Processes", log_name_list = self.log_name_list)
        ############################## 
        # Initialize Class Variables
        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.node_namespace,'list')
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.msg_if.pub_warn("Using list namespace: " + self.namespace, log_name_list = self.log_name_list)
        
        self.reset_ordered_items_list = self.ordered_items_list
        self.reset_ordered_names_list = self.ordered_names_list
        self.reset_active_items_list = self.active_items_list
        self.reset_selected_item = self.selected_item    
        self.reset_selected_item = self.selected_item_index

        if itemSelectCb is not None:
           self.itemSelectCb = itemSelectCb
        if listUpdatedCb is not None:
           self.listUpdatedCb = listUpdatedCb
        if refreshListCb is not None:
           self.refreshListCb = refreshListCb

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
                'msg': UpdateState,
                'qsize': 10,
                'callback': self.updateStateCb, 
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
                        log_name_list = self.log_name_list,
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
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
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

    def statusPubCb(self,timer):
        self.publish_status


    def publish_status(self):
        if self.node_if is not None:
            status_msg = ListIFStatus()
            status_msg.ordered_items_list = self.ordered_items_list
            status_msg.ordered_names_list = self.ordered_names_list
            status_msg.active_items_list = self.active_items_list
            status_msg.selected_item = self.selected_item    
            status_msg.selected_item = self.selected_item_index
            status_msg.multi_select_enabled = self.multi_select_enabled
            status_msg.update_order_enabled = self.update_order_enabled
            self.node_if.publish_pub('status_pub', status_msg)


    def get_list_data(self):
        return self.ordered_items_list, self.ordered_names_list, self.active_items_list, self.selected_item

    def update_list_data(self, ordered_items_list, ordered_names_list, active_items_list, selected_item):
        self.ordered_items_list = ordered_items_list
        if ordered_names_list is None:
            self.ordered_names_list = ordered_items_list
        elif len(ordered_names_list) != len(ordered_items_list):
            self.ordered_names_list = ordered_items_list
        else:
            self.ordered_names_list = ordered_names_list

        ########
        if selected_item is not None:
            if selected_item in ordered_items_list:
                self.selected_item = selected_item
                self.selected_item_index = ordered_items_list.index(selected_item)
        ########
        if self.multi_select_enabled == False:
            if self.selected_item in self.ordered_items_list:
                self.active_items_list = [self.selected_item]
        else:
            for item in active_items_list:
                if item in self.ordered_items_list:
                    self.active_items_list.append(item)

        return self.get_list_data()

    def signalUpdate(self):
        if self.listUpdatedCb is not None:
            self.listUpdatedCb(self.get_list_data())

    def signalSelect(self,item):
        if self.signalSelectCb is not None:
            active = item in self.active_list
            self.signalSelectCb(item,ative)        

  ###################
  ## List Mgr Callbacks
  def enableAllCb(self,msg):
    self.msg_if.pub_info("Got Update enable all msg: " + str(msg))
    self.active_items_list = self.ordered_items_list
    self.signalUdpdate()
    self.publish_status()

  def disableAllCb(self,msg):
    self.msg_if.pub_info("Got disable all msg: " + str(msg))
    self.active_items_list = []
    self.signalUdpdate()
    self.publish_status()


  def selectItemCb(self,msg):
    self.msg_if.pub_info("Got select msg: " + str(msg))
    item = msg.data
    self.selectItem(item)

  def selectItem(self,item)
    ordered_items_list = copy.deepcopy(self.ordered_items_list)
    if item in ordered_items_list:
        self.selected_item = item
        self.selected_item_index = ordered_items_list.index(selected_item)
        if self.multi_select_enabled == False:
            self.active_items_list = [item]
        self.signalSelect(item)
    self.publish_status()

  def updateStateCb(self,msg):
    self.msg_if.pub_info("Got update state msg: " + str(msg))
    item = msg.name
    active_state = msg.active_state
    self.signalSelect(item)
    self.update_state(item,active_state)

  def update_state(self,item,active_state):
    if self.multi_select_enabled == True:
        self.msg_if.pub_info("Updateding " + item + " state: " + str(active_state))
        ordered_items_list = copy.deepcopy(self.ordered_items_list)
        active_items_list = copy.deepcopy(self.active_items_list)
        if active_state == False and item in ordered_items_list and item in active_items_list:
            active_items_list.remove(item)
            #self.msg_if.pub_warn("Update state dict: " + str(ordered_items_list[item]))
            self.active_items_list = active_items_list
            self.signalUdpdate()
            self.publish_status()
        if active_state == True and item in ordered_items_list and item not in active_items_list:
            active_items_list.append(item)
            #self.msg_if.pub_warn("Update state dict: " + str(ordered_items_list[item]))
            self.active_items_list = active_items_list
            self.signalUdpdate()
        self.publish_status()


  def updateOrderCb(self,msg):
    self.msg_if.pub_info("Got Update order msg: " + str(msg))
    item = msg.name
    move_cmd = msg.move_cmd
    moveFunction = self.getOrderUpdateFunction(move_cmd)
    if item in self.ordered_items_list:
        ordered_items_list = copy.deepcopy(self.ordered_items_list)
        ordered_names_list = copy.deepcopy(self.ordered_names_list)
        [ordered_items_list,ordered_names_list] = moveFunction(item,ordered_items_list,ordered_names_list)
        self.ordered_items_list = ordered_items_list
        self.ordered_names_list = ordered_names_list
        self.signalUdpdate()
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

  def moveDriverNone(self,item,ordered_items_list):
    return ordered_items_list
    self.signalUdpdate()
    self.publish_status()

 
  def resetCb(self,msg):
    self.ordered_items_list = self.reset_ordered_items_list
    self.ordered_names_list = self.reset_ordered_names_list
    self.active_items_list = self.reset_active_items_list
    self.selected_item = self.reset_selected_item    
    self.selected_item = self.reset_selected_item_index
    self.signalUdpdate()
    self.publish_status()

  def refreshCb(self,msg):
    if self.refreshListCb is not None:
        self.refreshListCb()
    
    


