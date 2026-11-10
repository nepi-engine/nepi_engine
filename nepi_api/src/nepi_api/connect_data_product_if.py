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
import math
import threading
import numpy as np


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_pc

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nepi_interfaces.msg import SaveDataRate
from nepi_interfaces.msg import NavPose, NavPoseStatus


from nepi_api.messages_if import MsgIF

from nepi_api.connect_node_if import ConnectNodeIF




#########################################
# Connect Data Product IF Class
#########################################


# connect_status_msg is the status msg TYPE NAME, as a string. ConnectNodeIF
# hands it to nepi_sdk.find_topics_by_msg, which string-compares it against the
# ROS type names of live topics, and publishes it as ConnectIFStatus.status_msg_type,
# which is a string field. connect_status_msg_class is the matching msg CLASS,
# which subscribe_topic needs as the subscriber msg type. The two are separate
# parameters because neither consumer accepts the other's form, and ConnectDataProductIF
# is generic so it cannot hardcode the class the way connect_targets_if.py does.
CONNECT_ID='DATA'
CONNECT_STATUS_MSG=None
CONNECT_STATUS_MSG_CLASS=None
CONNECT_DATA_MSG=None
CONNECT_NAME='data_connect'


CONNECTED_TIMEOUT = 2


class ConnectDataProductIF(ConnectNodeIF):

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

    navpose_dict = None
    navpose_dict_lock = None


    connect_data_msg = None
    connect_data = True
    data_callback = None

    connect_topic_subs_dict = None
    connect_topic_pubs_dict = None

    connect_topic_controls_dict = None

    has_navpose = False
    #######################
    ### IF Initialization
    def __init__(self,
                connect_id = CONNECT_ID,
                connect_name = CONNECT_NAME,
                namespace = None,
                auto_select_enabled = True,
                connect_status_msg = CONNECT_STATUS_MSG,
                connect_status_msg_class = CONNECT_STATUS_MSG_CLASS,
                status_callback = None,
                connect_data = True,
                connect_data_msg = CONNECT_DATA_MSG,
                data_callback = None,
                filter_topic_list = [],
                connect_topic_controls_dict = None,
                has_navpose = False,
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
        self.has_navpose = has_navpose

        self.namespace = nepi_sdk.create_namespace(self.node_namespace,connect_name)
        self.node_if_prefix = self.namespace.replace(self.node_namespace + '/','') + '/'

        super().__init__(
                connect_id = connect_id,
                connect_status_msg = connect_status_msg,
                connect_name = connect_name,
                selected_topic = namespace,
                auto_select_enabled = auto_select_enabled,
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
        self.navpose_dict_lock = threading.Lock()


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
        if type(self) is ConnectDataProductIF:
            self.start()


    #######################
    # Class Public Methods
    #######################


    def get_ready_state(self):
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
        """Return the fully-resolved ROS namespace for the connected data source.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.selected_topic


    def check_connection(self):
        """Check whether the data source is currently connected.

        Returns:
            bool: True if a status message has been received within the connection timeout window,
                False otherwise.
        """
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        """Block until the data source is connected or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if connection was established, False if the timeout was reached.
        """
        if self.node_if is not None and self.selected_topic != 'None':
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.connected


    def check_status_connection(self):
        """Check whether the status topic from the data source is currently connected.

        Returns:
            bool: True if status messages are being received, False otherwise.
        """
        return self.connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        """Block until the data source status topic is connected or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the status connection was established, False if the timeout was reached.
        """
        if self.node_if is not None and self.selected_topic != 'None':
            self.msg_if.pub_info("Waiting for status connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to connect to status msg")
            else:
                self.msg_if.pub_info("Status Connected")
        return self.connected

    def get_status_dict(self):
        """Return the latest data source status as a dictionary.

        Returns:
            dict: A dictionary representation of the most recent ImageStatus message,
                or None if no status has been received yet.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def get_status_msg(self):
        """Return the latest data source status as a msg.

        Returns:
            dict: A msg representation of the most recent ImageStatus message,
                or None if no status has been received yet.
        """
        return self.status_msg

    def get_data_topic(self):
        """Return the image data topic of the connected data source.

        Returns:
            str: The selected data source namespace, which is also the Image topic
                the source publishes on, or 'None' if no source is selected.
        """
        return self.selected_topic


    def get_navpose_dict(self):
        self.navpose_dict_lock.acquire()
        navpose_dict = copy.deepcopy(self.navpose_dict)
        self.navpose_dict_lock.release()
        return navpose_dict



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


        if self.has_navpose == True:
            self.connect_topic_subs_dict[self.node_if_prefix + 'navpose_sub'] = {
                'namespace': self.selected_topic,
                'topic': 'navpose',
                'msg': NavPose,
                'qsize': 1,
                'callback': self._navposeCb
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
       
        navpose_dict = nepi_nav.convert_navpose_msg2dict(navpose_msg)


        self.navpose_dict_lock.acquire()
        self.navpose_dict = navpose_dict
        self.navpose_dict_lock.release()


