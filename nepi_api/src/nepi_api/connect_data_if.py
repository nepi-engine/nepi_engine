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
from sensor_msgs.msg import Image, PointCloud2
from nepi_interfaces.msg import SaveDataRate
from nepi_interfaces.msg import ImageStatus
from nepi_interfaces.msg import NavPose, NavPoseStatus
from nepi_interfaces.msg import DepthMapStatus
from nepi_interfaces.msg import PointcloudStatus

from nepi_api.messages_if import MsgIF

from nepi_api.connect_node_if import ConnectNodeIF
from nepi_api.connect_node_if import ConnectNodeClassIF




#########################################
# Connect IF Class
#########################################


# connect_status_msg is the status msg TYPE NAME, as a string. ConnectNodeIF
# hands it to nepi_sdk.find_topics_by_msg, which string-compares it against the
# ROS type names of live topics, and publishes it as ConnectIFStatus.status_msg_type,
# which is a string field. connect_status_msg_class is the matching msg CLASS,
# which subscribe_topic needs as the subscriber msg type. The two are separate
# parameters because neither consumer accepts the other's form, and ConnectDataIF
# is generic so it cannot hardcode the class the way connect_targets_if.py does.
CONNECT_ID='DATA'
CONNECT_STATUS_MSG=None
CONNECT_STATUS_MSG_CLASS=None
CONNECT_DATA_MSG=None
CONNECT_NAME='data_connect'


CONNECTED_TIMEOUT = 2


class ConnectDataIF(ConnectNodeIF):

    # ADD Additional Connect Callback Functions


    msg_if = None
    ready = False
    namespace = '~'

    node_if = None

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
    # data_dict_lock is created per instance in __init__. As a class attribute it
    # would be one lock shared by every connect instance in the process, needlessly
    # serialising instances that only ever guard their own data_dict.
    data_dict = None
    data_dict_lock = None

    get_data = False
    got_data = False

    connect_data_msg = None
    connect_data = True
    data_callback = None

    connect_topic_subs_dict = None
    connect_topic_pubs_dict = None
    #######################
    ### IF Initialization
    def __init__(self,
                connect_id = CONNECT_ID,
                connect_name = CONNECT_NAME,
                namespace = None,
                connect_status_msg = CONNECT_STATUS_MSG,
                connect_status_msg_class = CONNECT_STATUS_MSG_CLASS,
                status_callback = None,
                connect_data = True,
                connect_data_msg = CONNECT_DATA_MSG,
                data_callback = None,
                filter_topic_list = [],
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

        self.msg_if = msg_if
        self.node_if = node_if
        super().__init__(
                connect_id = connect_id,
                connect_status_msg = connect_status_msg,
                connect_name = connect_name,
                selected_topic = namespace,
                auto_select_enabled = True,
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
        self.data_dict_lock = threading.Lock()


        ##############################
        # Start updater process
        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################


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


    def set_get_data(self, state):
        """Set the flag requesting capture of the next available image frame.

        Args:
            state (bool): True to request the next frame, False to clear the request.

        Returns:
            bool: Always True.
        """
        self.get_data = state
        return True

    def read_get_got_states(self):
        """Return the current get and got data flags.

        Returns:
            list: A two-element list [get_data, got_data] where get_data indicates
                whether a frame has been requested and got_data indicates whether a
                frame is waiting to be retrieved.
        """
        return [self.get_data, self.got_data]


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
            'status_sub': {
                'namespace': self.selected_topic,
                'topic': 'status',
                'msg': self.connect_status_msg_class,
                'qsize': 10,
                'callback': self._statusCb
            }
        }
        if self.connect_data == True:
             self.connect_topic_subs_dict['data_sub'] = {
                'namespace': self.selected_topic,
                'topic': '',
                'msg': self.connect_data_msg,
                'qsize': 1,
                'callback': self._dataCb
            }



        # Publishers Config Dict ####################
        self.connect_topic_pubs_dict = {
            'save_config': {
                'namespace': self.selected_topic,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': 1,
            },
            'reset_config': {
                'namespace': self.selected_topic,
                'topic': 'reset_config',
                'msg': Empty,
                'qsize': 1,
            },
            'factory_reset_config': {
                'namespace': self.selected_topic,
                'topic': 'factory_reset_config',
                'msg': Empty,
                'qsize': 1,
            }


        }

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
        # Only build a frame when a consumer has asked for one (get_data flag) or
        # a data_callback is registered; otherwise the incoming Image is
        # dropped cheaply. Connection state is driven by the status callback.
        get_data = (self.data_callback is not None or self.get_data == True)
        if get_data == False:
            return

        self.get_data = False

        data_dict = nepi_sdk.convert_msg2dict(data_msg)

        if self.data_callback is not None:
            self.data_callback(data_dict)
        else:
            self.data_dict_lock.acquire()
            self.data_dict = data_dict
            self.data_dict_lock.release()
            self.got_data = True




#########################################
# Connect NavPose IF Class
#########################################
# Client-side connect class mirroring data_if.NavPoseIF. NavPoseIF is a
# standalone (non-image) data interface, so this connect class subclasses
# ConnectNodeIF directly and diverges from the image-oriented ConnectDataIF:
# the connected source publishes a NavPose on its base namespace and a
# NavPoseStatus on <namespace>/status. Retrieved NavPose messages are
# converted to nav pose dictionaries and cached (thread-safe) for polling
# consumers, or handed straight to data_callback when one is provided.


NAVPOSE_CONNECT_ID = 'NAVPOSE'
NAVPOSE_CONNECT_STATUS_MSG = 'NavPoseStatus'
NAVPOSE_CONNECT_STATUS_MSG_CLASS = NavPoseStatus
NAVPOSE_CONNECT_DATA_MSG = NavPose
NAVPOSE_CONNECT_NAME = 'navpose_connect'


class ConnectNavPoseIF(ConnectDataIF):

    # ADD Additional Connect Callback Functions

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = NAVPOSE_CONNECT_NAME,
                namespace = None,
                status_callback = None,
                connect_data = True,
                data_callback = None,
                filter_topic_list = [],
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        self.msg_if = msg_if
        self.node_if = node_if
        # Forward by keyword. These arguments used to be passed positionally into a
        # signature whose 3rd and 6th slots are connect_status_msg/connect_data_msg,
        # so every argument landed in the wrong slot and connect_status_msg received
        # None, which tripped the guard in ConnectDataIF.__init__ and left the
        # instance dead. msg_if and node_if never arrived at all.
        super().__init__(
                connect_id = NAVPOSE_CONNECT_ID,
                connect_name = connect_name,
                namespace = namespace,
                connect_status_msg = NAVPOSE_CONNECT_STATUS_MSG,
                connect_status_msg_class = NAVPOSE_CONNECT_STATUS_MSG_CLASS,
                status_callback = status_callback,
                connect_data = connect_data,
                connect_data_msg = NAVPOSE_CONNECT_DATA_MSG,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = msg_if,
                node_if = node_if
                )
        ####  IF INIT SETUP ####

        self.msg_if.pub_info("IF Initialization Complete")
        ###############################


    #######################
    # Class Public Methods
    #######################



    ###############################
    # Class Private Methods
    ###############################





#########################################
# Connect BaseImage IF Class
#########################################
# Client-side connect class mirroring data_if.BaseImageIF, the shared parent of
# the image data-interface family. ConnectDataIF already implements the full
# image connect pattern (ImageStatus status + Image data), so ConnectBaseImageIF
# subclasses it and supplies only the image msg types. Every data_if.BaseImageIF
# descendant publishes the same ImageStatus/Image pair, so the image connect
# subclasses below differ from this one only in the connect name they register
# under -- and that name must differ, because it becomes the connect namespace.


BASE_IMAGE_CONNECT_ID = 'IMAGE'
BASE_IMAGE_CONNECT_STATUS_MSG = 'ImageStatus'
BASE_IMAGE_CONNECT_STATUS_MSG_CLASS = ImageStatus
BASE_IMAGE_CONNECT_DATA_MSG = Image
BASE_IMAGE_CONNECT_NAME = 'base_image_connect'


class ConnectBaseImageIF(ConnectDataIF):
    """Shared parent connect class for the image data-interface family.

    Mirrors data_if.BaseImageIF. Supplies the ImageStatus/Image msg types to
    ConnectDataIF, which provides the complete connect implementation
    (status/ready/connection handling, save/config methods, and the private ROS
    callbacks).
    """

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = BASE_IMAGE_CONNECT_NAME,
                namespace = None,
                status_callback = None,
                connect_data = True,
                data_callback = None,
                filter_topic_list = [],
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        self.msg_if = msg_if
        self.node_if = node_if
        super().__init__(
                connect_id = BASE_IMAGE_CONNECT_ID,
                connect_name = connect_name,
                namespace = namespace,
                connect_status_msg = BASE_IMAGE_CONNECT_STATUS_MSG,
                connect_status_msg_class = BASE_IMAGE_CONNECT_STATUS_MSG_CLASS,
                status_callback = status_callback,
                connect_data = connect_data,
                connect_data_msg = BASE_IMAGE_CONNECT_DATA_MSG,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = msg_if,
                node_if = node_if
                )
        ####  IF INIT SETUP ####
        ###############################




#########################################
# Connect Image IF Class
#########################################


IMAGE_CONNECT_NAME = 'image_connect'


class ConnectImageIF(ConnectBaseImageIF):
    """Connect class mirroring data_if.ImageIF.

    Adds nothing beyond the shared ConnectBaseImageIF image connect pattern; the
    connected 'image' data product is selected by namespace.
    """

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = IMAGE_CONNECT_NAME,
                namespace = None,
                status_callback = None,
                connect_data = True,
                data_callback = None,
                filter_topic_list = [],
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        self.msg_if = msg_if
        self.node_if = node_if
        super().__init__(
                connect_name = connect_name,
                namespace = namespace,
                status_callback = status_callback,
                connect_data = connect_data,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = msg_if,
                node_if = node_if
                )
        ####  IF INIT SETUP ####
        ###############################




#########################################
# Connect ColorImage IF Class
#########################################


COLOR_IMAGE_CONNECT_NAME = 'color_image_connect'


class ConnectColorImageIF(ConnectBaseImageIF):
    """Connect class mirroring data_if.ColorImageIF.

    Adds nothing beyond the shared ConnectBaseImageIF image connect pattern; the
    connected 'color_image' data product is selected by namespace.
    """

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = COLOR_IMAGE_CONNECT_NAME,
                namespace = None,
                status_callback = None,
                connect_data = True,
                data_callback = None,
                filter_topic_list = [],
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        self.msg_if = msg_if
        self.node_if = node_if
        super().__init__(
                connect_name = connect_name,
                namespace = namespace,
                status_callback = status_callback,
                connect_data = connect_data,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = msg_if,
                node_if = node_if
                )
        ####  IF INIT SETUP ####
        ###############################




#########################################
# Connect DepthMap IF Class
#########################################
# Client-side connect class mirroring data_if.DepthMapIF. DepthMapIF is a
# standalone (non-BaseImageIF) data interface that publishes its depth map as an
# Image on its base namespace and a DepthMapStatus on <namespace>/status, so
# this connect class subclasses ConnectDataIF and supplies the DepthMapStatus
# status type. Everything the connect pattern shares is inherited; what stays
# here is what ConnectDataIF has no generic form of -- the DepthMapStatus field
# accessors, the cached depth map getter, and a _dataCb that decodes the Image
# to cv2 and records latency rather than flattening the msg to a dict.


DEPTH_MAP_CONNECT_ID = 'DEPTH_MAP'
DEPTH_MAP_CONNECT_STATUS_MSG = 'DepthMapStatus'
DEPTH_MAP_CONNECT_STATUS_MSG_CLASS = DepthMapStatus
DEPTH_MAP_CONNECT_DATA_MSG = Image
DEPTH_MAP_CONNECT_NAME = 'depth_map_connect'


class ConnectDepthMapIF(ConnectDataIF):

    # ADD Additional Connect Callback Functions

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = DEPTH_MAP_CONNECT_NAME,
                namespace = None,
                status_callback = None,
                connect_data = True,
                data_callback = None,
                filter_topic_list = [],
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        self.msg_if = msg_if
        self.node_if = node_if
        super().__init__(
                connect_id = DEPTH_MAP_CONNECT_ID,
                connect_name = connect_name,
                namespace = namespace,
                connect_status_msg = DEPTH_MAP_CONNECT_STATUS_MSG,
                connect_status_msg_class = DEPTH_MAP_CONNECT_STATUS_MSG_CLASS,
                status_callback = status_callback,
                connect_data = connect_data,
                connect_data_msg = DEPTH_MAP_CONNECT_DATA_MSG,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = msg_if,
                node_if = node_if
                )
        ####  IF INIT SETUP ####
        ###############################


    #######################
    # Class Public Methods
    #######################


    def get_encoding(self):
        """Return the depth map encoding reported by the connected data source.

        Returns:
            str: The encoding string (e.g. '32FC1'), or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.encoding

    def get_image_size_px(self):
        """Return the depth map dimensions in pixels reported by the connected data source.

        Returns:
            list: A two-element list [width_px, height_px], or None if no status
                has been received.
        """
        if self.status_msg is not None:
            return [self.status_msg.width_px, self.status_msg.height_px]

    def get_depth_map_dict(self):
        """Retrieve and consume the latest captured depth map data dictionary.

        Thread-safe. Clears the stored data after retrieval so subsequent calls
        return None until a new frame arrives.

        Returns:
            dict: The depth map data dictionary containing the cv2 depth map,
                dimensions, timestamps, and latency metrics, or None if no depth
                map is available.
        """
        self.data_dict_lock.acquire()
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = None
        self.data_dict_lock.release()
        return data_dict


    ###############################
    # Class Private Methods
    ###############################

    # Overrides ConnectDataIF._dataCb, which flattens the msg with
    # convert_msg2dict. A depth map is an Image carrying float distance data, so
    # it is decoded to cv2 here and returned with its dimensions, timestamps and
    # get/process/got latency.
    def _dataCb(self,data_msg):
        # Only build a frame when a consumer has asked for one (get_data flag) or
        # a data_callback is registered; otherwise the incoming Image is
        # dropped cheaply. Connection state is driven by the status callback.
        get_data = (self.data_callback is not None or self.get_data == True)
        if get_data == False:
            return

        current_time = nepi_sdk.get_msg_stamp()
        msg_stamp = data_msg.header.stamp
        get_latency = (current_time.to_sec() - msg_stamp.to_sec())

        start_time = nepi_sdk.get_time()

        self.get_data = False

        ##############################
        ### Preprocess Depth Map
        data = nepi_img.rosimg_to_cv2img(data_msg)


        data_dict = dict()
        data_dict['namespace'] = self.selected_topic
        data_dict['data'] = data
        height, width = data.shape[:2]
        data_dict['width'] = width
        data_dict['height'] = height
        data_dict['timestamp'] = nepi_sdk.sec_from_msg_stamp(msg_stamp)
        data_dict['ros_img_header'] = data_msg.header
        data_dict['ros_img_stamp'] = msg_stamp
        data_dict['get_latency_time'] = get_latency
        ##############################

        process_time = round( (nepi_sdk.get_time() - start_time) , 3)
        data_dict['process_time'] = process_time

        got_latency = (nepi_sdk.get_msg_stamp().to_sec() - msg_stamp.to_sec())
        data_dict['got_latency_time'] = got_latency

        if self.data_callback is not None:
            self.data_callback(data_dict)
        else:
            self.data_dict_lock.acquire()
            self.data_dict = data_dict
            self.data_dict_lock.release()
            self.got_data = True




#########################################
# Connect DepthMapImage IF Class
#########################################


DEPTH_MAP_IMAGE_CONNECT_NAME = 'depth_map_image_connect'


class ConnectDepthMapImageIF(ConnectBaseImageIF):
    """Connect class mirroring data_if.DepthMapImageIF.

    Adds nothing beyond the shared ConnectBaseImageIF image connect pattern; the
    connected 'depth_map_image' data product is selected by namespace.
    """

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = DEPTH_MAP_IMAGE_CONNECT_NAME,
                namespace = None,
                status_callback = None,
                connect_data = True,
                data_callback = None,
                filter_topic_list = [],
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        self.msg_if = msg_if
        self.node_if = node_if
        super().__init__(
                connect_name = connect_name,
                namespace = namespace,
                status_callback = status_callback,
                connect_data = connect_data,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = msg_if,
                node_if = node_if
                )
        ####  IF INIT SETUP ####
        ###############################




#########################################
# Connect Pointcloud IF Class
#########################################
# Client-side connect class mirroring data_if.PointcloudIF. PointcloudIF is a
# standalone (non-BaseImageIF) data interface that publishes a PointCloud2 on
# its base namespace and a PointcloudStatus on <namespace>/status, so this
# connect class subclasses ConnectDataIF and supplies the PointcloudStatus
# status type. Everything the connect pattern shares is inherited; what stays
# here is what ConnectDataIF has no generic form of -- the PointcloudStatus
# field accessors, the cached pointcloud getter, and a _dataCb that converts the
# PointCloud2 to an Open3D cloud and records latency.


POINTCLOUD_CONNECT_ID = 'POINTCLOUD'
POINTCLOUD_CONNECT_STATUS_MSG = 'PointcloudStatus'
POINTCLOUD_CONNECT_STATUS_MSG_CLASS = PointcloudStatus
POINTCLOUD_CONNECT_DATA_MSG = PointCloud2
POINTCLOUD_CONNECT_NAME = 'pointcloud_connect'


class ConnectPointcloudIF(ConnectDataIF):

    # ADD Additional Connect Callback Functions

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = POINTCLOUD_CONNECT_NAME,
                namespace = None,
                status_callback = None,
                connect_data = True,
                data_callback = None,
                filter_topic_list = [],
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        self.msg_if = msg_if
        self.node_if = node_if
        super().__init__(
                connect_id = POINTCLOUD_CONNECT_ID,
                connect_name = connect_name,
                namespace = namespace,
                connect_status_msg = POINTCLOUD_CONNECT_STATUS_MSG,
                connect_status_msg_class = POINTCLOUD_CONNECT_STATUS_MSG_CLASS,
                status_callback = status_callback,
                connect_data = connect_data,
                connect_data_msg = POINTCLOUD_CONNECT_DATA_MSG,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = msg_if,
                node_if = node_if
                )
        ####  IF INIT SETUP ####
        ###############################


    #######################
    # Class Public Methods
    #######################


    def get_point_count(self):
        """Return the point count reported by the connected pointcloud source.

        Returns:
            int: The number of points in the pointcloud, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.point_count

    def get_has_rgb(self):
        """Return whether the connected pointcloud source provides RGB color data.

        Returns:
            bool: True if the pointcloud carries RGB data, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.has_rgb

    def get_has_intensity(self):
        """Return whether the connected pointcloud source provides intensity data.

        Returns:
            bool: True if the pointcloud carries intensity data, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.has_intensity

    def get_pointcloud_dict(self):
        """Retrieve and consume the latest captured pointcloud data dictionary.

        Thread-safe. Clears the stored data after retrieval so subsequent calls
        return None until a new frame arrives.

        Returns:
            dict: The pointcloud data dictionary containing the Open3D point cloud,
                point count, timestamps, and latency metrics, or None if no
                pointcloud is available.
        """
        self.data_dict_lock.acquire()
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = None
        self.data_dict_lock.release()
        return data_dict


    ###############################
    # Class Private Methods
    ###############################

    # Overrides ConnectDataIF._dataCb, which flattens the msg with
    # convert_msg2dict. A PointCloud2 is converted to an Open3D cloud here and
    # returned with its point count, timestamps and get/process/got latency.
    def _dataCb(self,data_msg):
        # Only build a frame when a consumer has asked for one (get_data flag) or
        # a data_callback is registered; otherwise the incoming PointCloud2 is
        # dropped cheaply. Connection state is driven by the status callback.
        get_data = (self.data_callback is not None or self.get_data == True)
        if get_data == False:
            return

        current_time = nepi_sdk.get_msg_stamp()
        msg_stamp = data_msg.header.stamp
        get_latency = (current_time.to_sec() - msg_stamp.to_sec())

        start_time = nepi_sdk.get_time()

        self.get_data = False

        ##############################
        ### Preprocess Pointcloud
        data = nepi_pc.rospc_to_o3dpc(data_msg)

        data_dict = dict()
        data_dict['namespace'] = self.selected_topic
        data_dict['data'] = data
        data_dict['point_count'] = len(data.points) if data is not None else 0
        data_dict['timestamp'] = nepi_sdk.sec_from_msg_stamp(msg_stamp)
        data_dict['ros_pc_header'] = data_msg.header
        data_dict['ros_pc_stamp'] = msg_stamp
        data_dict['get_latency_time'] = get_latency
        ##############################

        process_time = round( (nepi_sdk.get_time() - start_time) , 3)
        data_dict['process_time'] = process_time

        got_latency = (nepi_sdk.get_msg_stamp().to_sec() - msg_stamp.to_sec())
        data_dict['got_latency_time'] = got_latency

        if self.data_callback is not None:
            self.data_callback(data_dict)
        else:
            self.data_dict_lock.acquire()
            self.data_dict = data_dict
            self.data_dict_lock.release()
            self.got_data = True




#########################################
# Connect PointcloudImage IF Class
#########################################


POINTCLOUD_IMAGE_CONNECT_NAME = 'pointcloud_image_connect'


class ConnectPointcloudImageIF(ConnectBaseImageIF):
    """Connect class mirroring data_if.PointcloudImageIF.

    Adds nothing beyond the shared ConnectBaseImageIF image connect pattern; the
    connected 'pointcloud_image' data product is selected by namespace.
    """

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = POINTCLOUD_IMAGE_CONNECT_NAME,
                namespace = None,
                status_callback = None,
                connect_data = True,
                data_callback = None,
                filter_topic_list = [],
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        self.msg_if = msg_if
        self.node_if = node_if
        super().__init__(
                connect_name = connect_name,
                namespace = namespace,
                status_callback = status_callback,
                connect_data = connect_data,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = msg_if,
                node_if = node_if
                )
        ####  IF INIT SETUP ####
        ###############################
