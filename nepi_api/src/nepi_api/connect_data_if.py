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
from nepi_interfaces.msg import ColorBGR
from nepi_interfaces.msg import StringArray, UpdateBool, UpdateFloat
from nepi_interfaces.msg import RangeWindow, ImageMouseEvent
from nepi_interfaces.msg import ImageCrosshair, ImageTarget
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

    connect_topic_controls_dict = None
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
                connect_topic_controls_dict = None,
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
        self.connect_topic_controls_dict = connect_topic_controls_dict
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



        # Controls Config Dict ####################
        # Mirrors data_if.NavPoseIF.SUBS_DICT, which advertises exactly one
        # subscriber. The 'unknown' namespace is the sentinel ConnectDataIF
        # replaces with the connected topic at subscribe time.
        connect_topic_controls_dict = {
            'connect_navpose_reset': {
                'namespace': 'unknown',
                'topic': 'reset',
                'msg': Empty,
                'qsize': 1,
            }
        }

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
                connect_topic_controls_dict = connect_topic_controls_dict,
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

    #################
    ## Control Functions

    def reset(self):
        """Publish a reset command to the connected navpose source.

        Restores the navpose interface to its initialized state. Mirrors
        data_if.NavPoseIF.reset.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_navpose_reset', Empty())


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

        # Controls Config Dict ####################
        # Mirrors the data_if.BaseImageIF subscriber set. Only the subscribers
        # advertised on the data product namespace appear here -- that namespace is
        # what the connect side's selected_topic resolves to. The 'unknown' namespace
        # is the sentinel ConnectDataIF replaces with the connected topic at subscribe
        # time. Every image connect subclass inherits this set unchanged.
        connect_topic_controls_dict = {

            # Reset commands
            'connect_image_reset': {
                'namespace': 'unknown',
                'topic': 'reset',
                'msg': Empty,
                'qsize': 1,
            },
            'connect_image_reset_filters': {
                'namespace': 'unknown',
                'topic': 'reset_filters',
                'msg': Empty,
                'qsize': 1,
            },
            'connect_image_reset_overlays': {
                'namespace': 'unknown',
                'topic': 'reset_overlays',
                'msg': Empty,
                'qsize': 1,
            },
            'connect_image_reset_settings': {
                'namespace': 'unknown',
                'topic': 'reset_settings',
                'msg': Empty,
                'qsize': 1,
            },
            'connect_image_reset_renders': {
                'namespace': 'unknown',
                'topic': 'reset_renders',
                'msg': Empty,
                'qsize': 1,
            },

            # 3D render controls
            'connect_image_render_3d_controls': {
                'namespace': 'unknown',
                'topic': 'render_3d_controls',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_reset_render_3d_controls': {
                'namespace': 'unknown',
                'topic': 'reset_render_3d_controls',
                'msg': Empty,
                'qsize': 1,
            },
            'connect_image_reset_render_3d_position': {
                'namespace': 'unknown',
                'topic': 'reset_render_3d_position',
                'msg': Empty,
                'qsize': 1,
            },

            # Mouse event injection
            'connect_image_mouse_event': {
                'namespace': 'unknown',
                'topic': 'mouse_event',
                'msg': ImageMouseEvent,
                'qsize': 1,
            },

            # Overlay text controls
            'connect_image_set_overlay_text_enable': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_click_text_enable': {
                'namespace': 'unknown',
                'topic': 'click_text_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_overlay_text_size_ratio': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_size_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_overlay_text_vert_ratio': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_vert_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_overlay_text_horz_ratio': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_horz_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_overlay_text_transparency_ratio': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_transparency_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_overlay_text_color_rgb': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_color_rgb',
                'msg': ColorBGR,
                'qsize': 1,
            },
            'connect_image_set_overlay_text_source_name': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_source_name',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_overlay_text_date_time': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_date_time',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_overlay_text_nav': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_nav',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_overlay_text_pose': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_pose',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_add_overlay_text': {
                'namespace': 'unknown',
                'topic': 'add_overlay_text',
                'msg': String,
                'qsize': 1,
            },
            'connect_image_set_overlay_text_list': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_list',
                'msg': StringArray,
                'qsize': 1,
            },
            'connect_image_clear_overlay_text_list': {
                'namespace': 'unknown',
                'topic': 'clear_overlay_text_list',
                'msg': Empty,
                'qsize': 1,
            },

            # Crosshair overlay controls
            'connect_image_crosshairs_enable': {
                'namespace': 'unknown',
                'topic': 'crosshairs_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_crosshairs_size_ratio': {
                'namespace': 'unknown',
                'topic': 'set_crosshairs_size_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_crosshairs_thickness_ratio': {
                'namespace': 'unknown',
                'topic': 'set_crosshairs_thickness_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_crosshairs_text_ratio': {
                'namespace': 'unknown',
                'topic': 'set_crosshairs_text_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_crosshairs_transparency_ratio': {
                'namespace': 'unknown',
                'topic': 'set_crosshairs_transparency_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_crosshairs_color_rgb': {
                'namespace': 'unknown',
                'topic': 'set_crosshairs_color_rgb',
                'msg': ColorBGR,
                'qsize': 1,
            },
            'connect_image_overlay_crosshair_names': {
                'namespace': 'unknown',
                'topic': 'overlay_crosshair_names',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_overlay_crosshair_pixels': {
                'namespace': 'unknown',
                'topic': 'overlay_crosshair_pixels',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_overlay_crosshair_degrees': {
                'namespace': 'unknown',
                'topic': 'overlay_crosshair_degrees',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_overlay_crosshair_messages': {
                'namespace': 'unknown',
                'topic': 'overlay_crosshair_messages',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_click_crosshair_enable': {
                'namespace': 'unknown',
                'topic': 'click_crosshair_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_add_crosshair_pixel': {
                'namespace': 'unknown',
                'topic': 'add_crosshair_pixel',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            'connect_image_add_crosshair_ratios': {
                'namespace': 'unknown',
                'topic': 'add_crosshair_ratios',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            'connect_image_add_crosshair_degree_offsets': {
                'namespace': 'unknown',
                'topic': 'add_crosshair_degree_offsets',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            'connect_image_remove_crosshair': {
                'namespace': 'unknown',
                'topic': 'remove_crosshair',
                'msg': String,
                'qsize': 1,
            },
            'connect_image_clear_crosshairs': {
                'namespace': 'unknown',
                'topic': 'clear_crosshairs',
                'msg': Empty,
                'qsize': 1,
            },

            # Target overlay controls
            'connect_image_targets_enable': {
                'namespace': 'unknown',
                'topic': 'targets_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_targets_size_ratio': {
                'namespace': 'unknown',
                'topic': 'set_targets_size_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_targets_thickness_ratio': {
                'namespace': 'unknown',
                'topic': 'set_targets_thickness_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_targets_text_ratio': {
                'namespace': 'unknown',
                'topic': 'set_targets_text_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_targets_transparency_ratio': {
                'namespace': 'unknown',
                'topic': 'set_targets_transparency_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_targets_color_rgb': {
                'namespace': 'unknown',
                'topic': 'set_targets_color_rgb',
                'msg': ColorBGR,
                'qsize': 1,
            },
            'connect_image_overlay_target_names': {
                'namespace': 'unknown',
                'topic': 'overlay_target_names',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_overlay_target_pixels': {
                'namespace': 'unknown',
                'topic': 'overlay_target_pixels',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_overlay_target_degrees': {
                'namespace': 'unknown',
                'topic': 'overlay_target_degrees',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_overlay_target_messages': {
                'namespace': 'unknown',
                'topic': 'overlay_target_messages',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_click_target_enable': {
                'namespace': 'unknown',
                'topic': 'click_target_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_add_target_pixel': {
                'namespace': 'unknown',
                'topic': 'add_target_pixel',
                'msg': ImageTarget,
                'qsize': 1,
            },
            'connect_image_add_target_ratios': {
                'namespace': 'unknown',
                'topic': 'add_target_ratios',
                'msg': ImageTarget,
                'qsize': 1,
            },
            'connect_image_add_target_degree_offsets': {
                'namespace': 'unknown',
                'topic': 'add_target_degree_offsets',
                'msg': ImageTarget,
                'qsize': 1,
            },
            'connect_image_remove_target': {
                'namespace': 'unknown',
                'topic': 'remove_target',
                'msg': String,
                'qsize': 1,
            },
            'connect_image_clear_targets': {
                'namespace': 'unknown',
                'topic': 'clear_targets',
                'msg': Empty,
                'qsize': 1,
            },

            # Aspect and stream controls
            'connect_image_set_aspect_adjust_enable': {
                'namespace': 'unknown',
                'topic': 'set_aspect_adjust_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_aspect_adjust_ratio': {
                'namespace': 'unknown',
                'topic': 'set_aspect_adjust_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_aspect_adjust_by_ratio': {
                'namespace': 'unknown',
                'topic': 'set_aspect_adjust_by_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_stream_compression_enable': {
                'namespace': 'unknown',
                'topic': 'set_stream_compression_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_stream_compression_ratio': {
                'namespace': 'unknown',
                'topic': 'set_stream_compression_ratio',
                'msg': Float32,
                'qsize': 1,
            },

            # Live adjustment controls
            'connect_image_set_live_adjust_enable': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_live_adjust_rotate_ratio': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_rotate_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_live_adjust_rotate_deg': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_rotate_deg',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_live_adjust_x_ratio': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_x_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_live_adjust_x_pixel': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_x_pixel',
                'msg': Int32,
                'qsize': 1,
            },
            'connect_image_set_live_adjust_x_deg': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_x_deg',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_live_adjust_y_ratio': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_y_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_live_adjust_y_pixel': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_y_pixel',
                'msg': Int32,
                'qsize': 1,
            },
            'connect_image_set_live_adjust_y_deg': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_y_deg',
                'msg': Float32,
                'qsize': 1,
            },

            # Capability-gated controls. data_if.BaseImageIF advertises these only when

            # the matching caps_dict flag is set, and the connect side cannot know the

            # server's configuration, so they are always registered.
            'connect_image_set_resolution_ratio': {
                'namespace': 'unknown',
                'topic': 'set_resolution_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_auto_adjust_enable': {
                'namespace': 'unknown',
                'topic': 'set_auto_adjust_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_auto_adjust_ratio': {
                'namespace': 'unknown',
                'topic': 'set_auto_adjust_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_brightness_ratio': {
                'namespace': 'unknown',
                'topic': 'set_brightness_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_contrast_ratio': {
                'namespace': 'unknown',
                'topic': 'set_contrast_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_threshold_ratio': {
                'namespace': 'unknown',
                'topic': 'set_threshold_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_rotate_2d': {
                'namespace': 'unknown',
                'topic': 'rotate_2d',
                'msg': Empty,
                'qsize': 1,
            },
            'connect_image_set_rotate_2d_deg': {
                'namespace': 'unknown',
                'topic': 'set_rotate_2d_deg',
                'msg': Int32,
                'qsize': 1,
            },
            'connect_image_set_rotate_2d_swap_box': {
                'namespace': 'unknown',
                'topic': 'set_rotate_2d_swap_box',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_flip_horz': {
                'namespace': 'unknown',
                'topic': 'set_flip_horz',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_flip_vert': {
                'namespace': 'unknown',
                'topic': 'set_flip_vert',
                'msg': Bool,
                'qsize': 1,
            },
            'connect_image_set_range_ratios': {
                'namespace': 'unknown',
                'topic': 'set_range_ratios',
                'msg': RangeWindow,
                'qsize': 1,
            },
            'connect_image_set_zoom_ratio': {
                'namespace': 'unknown',
                'topic': 'set_zoom_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_pan_x_ratio': {
                'namespace': 'unknown',
                'topic': 'set_pan_x_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_pan_y_ratio': {
                'namespace': 'unknown',
                'topic': 'set_pan_y_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_zoom_3d_ratio': {
                'namespace': 'unknown',
                'topic': 'set_zoom_3d_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_rotate_3d_ratio': {
                'namespace': 'unknown',
                'topic': 'set_rotate_3d_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_tilt_3d_ratio': {
                'namespace': 'unknown',
                'topic': 'set_tilt_3d_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'connect_image_set_filter_enable': {
                'namespace': 'unknown',
                'topic': 'set_filter_enable',
                'msg': UpdateBool,
                'qsize': 1,
            },
            'connect_image_set_filter_ratio': {
                'namespace': 'unknown',
                'topic': 'set_filter_ratio',
                'msg': UpdateFloat,
                'qsize': 1,
            }
        }

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
                connect_topic_controls_dict = connect_topic_controls_dict,
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

    #################
    ## Control Functions

    # Reset commands

    def reset(self):
        """Publish a reset command, restoring all image controls to their initialized state.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_reset', Empty())

    def reset_filters(self):
        """Publish a reset command for the image filter settings.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_reset_filters', Empty())

    def reset_overlays(self):
        """Publish a reset command for the image overlay settings.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_reset_overlays', Empty())

    def reset_settings(self):
        """Publish a reset command for the image driver settings.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_reset_settings', Empty())

    def reset_renders(self):
        """Publish a reset command for the image render settings.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_reset_renders', Empty())

    # 3D render controls

    def render_3d_controls(self, enable):
        """Enable or disable the 3D render controls on the connected image source.

        Args:
            enable (bool): True to enable the 3D render controls, False to disable them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_render_3d_controls', Bool(enable))

    def reset_render_3d_controls(self):
        """Publish a reset command for the 3D render controls.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_reset_render_3d_controls', Empty())

    def reset_render_3d_position(self):
        """Publish a reset command for the 3D render camera position.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_reset_render_3d_position', Empty())

    # Mouse event injection

    def mouse_event(self, mouse_event_msg):
        """Publish a mouse event against the connected image source.

        Args:
            mouse_event_msg (ImageMouseEvent): A fully-populated ImageMouseEvent msg describing the click, drag, window or scroll event.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_mouse_event', mouse_event_msg)

    # Overlay text controls

    def set_overlay_text_enable(self, enable):
        """Enable or disable the image text overlay.

        Args:
            enable (bool): True to draw the overlay text, False to hide it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_overlay_text_enable', Bool(enable))

    def set_click_text(self, enable):
        """Enable or disable click-driven overlay text placement.

        Args:
            enable (bool): True to enable click-to-place text, False to disable it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_click_text_enable', Bool(enable))

    def set_overlay_text_size_ratio(self, ratio):
        """Set the overlay text size ratio.

        Args:
            ratio (float): Overlay text size as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_overlay_text_size_ratio', Float32(ratio))

    def set_overlay_text_vert_ratio(self, ratio):
        """Set the overlay text vertical position ratio.

        Args:
            ratio (float): Overlay text vertical position as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_overlay_text_vert_ratio', Float32(ratio))

    def set_overlay_text_horz_ratio(self, ratio):
        """Set the overlay text horizontal position ratio.

        Args:
            ratio (float): Overlay text horizontal position as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_overlay_text_horz_ratio', Float32(ratio))

    def set_overlay_text_transparency_ratio(self, ratio):
        """Set the overlay text transparency ratio.

        Args:
            ratio (float): Overlay text transparency as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_overlay_text_transparency_ratio', Float32(ratio))

    def set_overlay_text_color_rgb(self, color_msg):
        """Set the overlay text color.

        Args:
            color_msg (ColorBGR): A ColorBGR msg carrying the overlay text color.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_overlay_text_color_rgb', color_msg)

    def set_overlay_text_image_name(self, enable):
        """Enable or disable the data source name in the overlay text.

        Args:
            enable (bool): True to include the source name in the overlay text, False to omit it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_overlay_text_source_name', Bool(enable))

    def set_overlay_text_date_time(self, enable):
        """Enable or disable the date and time in the overlay text.

        Args:
            enable (bool): True to include the date and time in the overlay text, False to omit it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_overlay_text_date_time', Bool(enable))

    def set_overlay_text_nav(self, enable):
        """Enable or disable navigation data in the overlay text.

        Args:
            enable (bool): True to include navigation data in the overlay text, False to omit it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_overlay_text_nav', Bool(enable))

    def set_overlay_text_pose(self, enable):
        """Enable or disable pose data in the overlay text.

        Args:
            enable (bool): True to include pose data in the overlay text, False to omit it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_overlay_text_pose', Bool(enable))

    def set_overlay_text(self, text):
        """Append a line to the overlay text list.

        Args:
            text (str): The text line to append to the overlay text list.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_add_overlay_text', String(text))

    def set_overlay_text_list(self, text_list_msg):
        """Replace the overlay text list.

        Args:
            text_list_msg (StringArray): A StringArray msg carrying the full overlay text list.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_overlay_text_list', text_list_msg)

    def clear_overlay_text_list(self):
        """Clear the overlay text list.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_clear_overlay_text_list', Empty())

    # Crosshair overlay controls

    def set_crosshairs_enable(self, enable):
        """Enable or disable the crosshair overlays.

        Args:
            enable (bool): True to draw the crosshair overlays, False to hide them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_crosshairs_enable', Bool(enable))

    def set_crosshairs_size_ratio(self, ratio):
        """Set the crosshair size ratio.

        Args:
            ratio (float): Crosshair size as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_crosshairs_size_ratio', Float32(ratio))

    def set_crosshairs_thickness_ratio(self, ratio):
        """Set the crosshair line thickness ratio.

        Args:
            ratio (float): Crosshair line thickness as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_crosshairs_thickness_ratio', Float32(ratio))

    def set_crosshairs_text_ratio(self, ratio):
        """Set the crosshair label text size ratio.

        Args:
            ratio (float): Crosshair label text size as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_crosshairs_text_ratio', Float32(ratio))

    def set_crosshairs_transparency_ratio(self, ratio):
        """Set the crosshair transparency ratio.

        Args:
            ratio (float): Crosshair transparency as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_crosshairs_transparency_ratio', Float32(ratio))

    def set_crosshairs_color_rgb(self, color_msg):
        """Set the crosshair overlay color.

        Args:
            color_msg (ColorBGR): A ColorBGR msg carrying the crosshair color.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_crosshairs_color_rgb', color_msg)

    def set_overlay_crosshair_names(self, enable):
        """Enable or disable crosshair name labels.

        Args:
            enable (bool): True to label crosshairs with their names, False to omit the labels.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_overlay_crosshair_names', Bool(enable))

    def set_overlay_crosshair_pixels(self, enable):
        """Enable or disable crosshair pixel coordinate labels.

        Args:
            enable (bool): True to label crosshairs with their pixel coordinates, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_overlay_crosshair_pixels', Bool(enable))

    def set_overlay_crosshair_degrees(self, enable):
        """Enable or disable crosshair degree offset labels.

        Args:
            enable (bool): True to label crosshairs with their degree offsets, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_overlay_crosshair_degrees', Bool(enable))

    def set_overlay_crosshair_messages(self, enable):
        """Enable or disable crosshair message labels.

        Args:
            enable (bool): True to label crosshairs with their message strings, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_overlay_crosshair_messages', Bool(enable))

    def set_click_crosshair(self, enable):
        """Enable or disable click-driven crosshair placement.

        Args:
            enable (bool): True to enable click-to-place crosshairs, False to disable it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_click_crosshair_enable', Bool(enable))

    def add_crosshair_pixel(self, crosshair_msg):
        """Add a crosshair positioned by pixel coordinates.

        Args:
            crosshair_msg (ImageCrosshair): An ImageCrosshair msg whose x_pixel and y_pixel fields locate the crosshair.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_add_crosshair_pixel', crosshair_msg)

    def add_crosshair_ratios(self, crosshair_msg):
        """Add a crosshair positioned by image ratios.

        Args:
            crosshair_msg (ImageCrosshair): An ImageCrosshair msg whose x_ratio and y_ratio fields locate the crosshair.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_add_crosshair_ratios', crosshair_msg)

    def add_crosshair_degree_offsets(self, crosshair_msg):
        """Add a crosshair positioned by degree offsets from image center.

        Args:
            crosshair_msg (ImageCrosshair): An ImageCrosshair msg whose x_offset_deg and y_offset_deg fields locate the crosshair.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_add_crosshair_degree_offsets', crosshair_msg)

    def remove_crosshair(self, name):
        """Remove a named crosshair.

        Args:
            name (str): Name of the crosshair to remove.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_remove_crosshair', String(name))

    def clear_crosshairs(self):
        """Remove every crosshair from the connected image source.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_clear_crosshairs', Empty())

    # Target overlay controls

    def set_targets_enable(self, enable):
        """Enable or disable the target overlays.

        Args:
            enable (bool): True to draw the target overlays, False to hide them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_targets_enable', Bool(enable))

    def set_targets_size_ratio(self, ratio):
        """Set the target size ratio.

        Args:
            ratio (float): Target size as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_targets_size_ratio', Float32(ratio))

    def set_targets_thickness_ratio(self, ratio):
        """Set the target line thickness ratio.

        Args:
            ratio (float): Target line thickness as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_targets_thickness_ratio', Float32(ratio))

    def set_targets_text_ratio(self, ratio):
        """Set the target label text size ratio.

        Args:
            ratio (float): Target label text size as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_targets_text_ratio', Float32(ratio))

    def set_targets_transparency_ratio(self, ratio):
        """Set the target transparency ratio.

        Args:
            ratio (float): Target transparency as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_targets_transparency_ratio', Float32(ratio))

    def set_targets_color_rgb(self, color_msg):
        """Set the target overlay color.

        Args:
            color_msg (ColorBGR): A ColorBGR msg carrying the target color.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_targets_color_rgb', color_msg)

    def set_overlay_target_names(self, enable):
        """Enable or disable target name labels.

        Args:
            enable (bool): True to label targets with their names, False to omit the labels.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_overlay_target_names', Bool(enable))

    def set_overlay_target_pixels(self, enable):
        """Enable or disable target pixel coordinate labels.

        Args:
            enable (bool): True to label targets with their pixel coordinates, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_overlay_target_pixels', Bool(enable))

    def set_overlay_target_degrees(self, enable):
        """Enable or disable target degree offset labels.

        Args:
            enable (bool): True to label targets with their degree offsets, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_overlay_target_degrees', Bool(enable))

    def set_overlay_target_messages(self, enable):
        """Enable or disable target message labels.

        Args:
            enable (bool): True to label targets with their message strings, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_overlay_target_messages', Bool(enable))

    def set_click_target(self, enable):
        """Enable or disable click-driven target placement.

        Args:
            enable (bool): True to enable click-to-place targets, False to disable it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_click_target_enable', Bool(enable))

    def add_target_pixel(self, target_msg):
        """Add a target positioned by pixel coordinates.

        Args:
            target_msg (ImageTarget): An ImageTarget msg whose x_pixel and y_pixel fields locate the target.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_add_target_pixel', target_msg)

    def add_target_ratios(self, target_msg):
        """Add a target positioned by image ratios.

        Args:
            target_msg (ImageTarget): An ImageTarget msg whose x_ratio and y_ratio fields locate the target.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_add_target_ratios', target_msg)

    def add_target_degree_offsets(self, target_msg):
        """Add a target positioned by degree offsets from image center.

        Args:
            target_msg (ImageTarget): An ImageTarget msg whose x_offset_deg and y_offset_deg fields locate the target.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_add_target_degree_offsets', target_msg)

    def remove_target(self, name):
        """Remove a named target.

        Args:
            name (str): Name of the target to remove.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_remove_target', String(name))

    def clear_targets(self):
        """Remove every target from the connected image source.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_clear_targets', Empty())

    # Aspect and stream controls

    def set_aspect_adjust_enable(self, enable):
        """Enable or disable image aspect ratio adjustment.

        Args:
            enable (bool): True to enable aspect ratio adjustment, False to disable it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_aspect_adjust_enable', Bool(enable))

    def set_aspect_adjust_ratio(self, ratio):
        """Set the image aspect adjustment ratio.

        Args:
            ratio (float): Target aspect ratio value.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_aspect_adjust_ratio', Float32(ratio))

    def set_aspect_adjust_by_ratio(self, ratio):
        """Set the image aspect adjustment by relative ratio.

        Args:
            ratio (float): Relative aspect adjustment as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_aspect_adjust_by_ratio', Float32(ratio))

    def set_stream_compression_enable(self, enable):
        """Enable or disable image stream compression.

        Args:
            enable (bool): True to compress the published image stream, False to publish uncompressed.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_stream_compression_enable', Bool(enable))

    def set_stream_compression_ratio(self, ratio):
        """Set the image stream compression ratio.

        Args:
            ratio (float): Stream compression as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_stream_compression_ratio', Float32(ratio))

    # Live adjustment controls

    def set_live_adjust_enable(self, enable):
        """Enable or disable live image adjustments.

        Args:
            enable (bool): True to enable live image adjustments, False to disable them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_live_adjust_enable', Bool(enable))

    def set_live_adjust_rotate_ratio(self, ratio):
        """Set the live adjustment rotation ratio.

        Args:
            ratio (float): Live rotation as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_live_adjust_rotate_ratio', Float32(ratio))

    def set_live_adjust_rotate_deg(self, deg):
        """Set the live adjustment rotation in degrees.

        Args:
            deg (float): Live rotation in degrees.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_live_adjust_rotate_deg', Float32(deg))

    def set_live_adjust_x_ratio(self, ratio):
        """Set the live adjustment horizontal translation ratio.

        Args:
            ratio (float): Live horizontal translation as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_live_adjust_x_ratio', Float32(ratio))

    def set_live_adjust_x_pixel(self, pixel):
        """Set the live adjustment horizontal translation in pixels.

        Args:
            pixel (int): Live horizontal translation in pixels.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_live_adjust_x_pixel', Int32(pixel))

    def set_live_adjust_x_deg(self, deg):
        """Set the live adjustment horizontal translation in degrees.

        Args:
            deg (float): Live horizontal translation in degrees.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_live_adjust_x_deg', Float32(deg))

    def set_live_adjust_y_ratio(self, ratio):
        """Set the live adjustment vertical translation ratio.

        Args:
            ratio (float): Live vertical translation as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_live_adjust_y_ratio', Float32(ratio))

    def set_live_adjust_y_pixel(self, pixel):
        """Set the live adjustment vertical translation in pixels.

        Args:
            pixel (int): Live vertical translation in pixels.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_live_adjust_y_pixel', Int32(pixel))

    def set_live_adjust_y_deg(self, deg):
        """Set the live adjustment vertical translation in degrees.

        Args:
            deg (float): Live vertical translation in degrees.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_live_adjust_y_deg', Float32(deg))

    # Capability-gated controls. data_if.BaseImageIF advertises these only when

    # the matching caps_dict flag is set, and the connect side cannot know the

    # server's configuration, so they are always registered.

    def set_resolution_ratio(self, ratio):
        """Set the image resolution ratio.

        Args:
            ratio (float): Image resolution as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_resolution_ratio', Float32(ratio))

    def set_auto_adjust_enable(self, enable):
        """Enable or disable image auto adjustment.

        Args:
            enable (bool): True to enable auto adjustment, False to disable it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_auto_adjust_enable', Bool(enable))

    def set_auto_adjust_ratio(self, ratio):
        """Set the image auto adjustment ratio.

        Args:
            ratio (float): Auto adjustment strength as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_auto_adjust_ratio', Float32(ratio))

    def set_brightness_ratio(self, ratio):
        """Set the image brightness ratio.

        Args:
            ratio (float): Image brightness as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_brightness_ratio', Float32(ratio))

    def set_contrast_ratio(self, ratio):
        """Set the image contrast ratio.

        Args:
            ratio (float): Image contrast as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_contrast_ratio', Float32(ratio))

    def set_threshold_ratio(self, ratio):
        """Set the image threshold ratio.

        Args:
            ratio (float): Image threshold as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_threshold_ratio', Float32(ratio))

    def rotate_2d(self):
        """Publish a single 2D rotation step command.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_rotate_2d', Empty())

    def set_rotate_2d_deg(self, deg):
        """Set the 2D image rotation in degrees.

        Args:
            deg (int): 2D rotation in degrees.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_rotate_2d_deg', Int32(deg))

    def set_rotate_2d_swap_box(self, enable):
        """Enable or disable bounding box swapping on 2D rotation.

        Args:
            enable (bool): True to swap the bounding box on rotation, False to leave it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_rotate_2d_swap_box', Bool(enable))

    def set_flip_horz(self, enable):
        """Enable or disable horizontal image flip.

        Args:
            enable (bool): True to flip the image horizontally, False to leave it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_flip_horz', Bool(enable))

    def set_flip_vert(self, enable):
        """Enable or disable vertical image flip.

        Args:
            enable (bool): True to flip the image vertically, False to leave it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_flip_vert', Bool(enable))

    def set_range_ratios(self, range_msg):
        """Set the image range window ratios.

        Args:
            range_msg (RangeWindow): A RangeWindow msg carrying the start and stop range ratios.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_range_ratios', range_msg)

    def set_zoom_ratio(self, ratio):
        """Set the image zoom ratio.

        Args:
            ratio (float): Image zoom as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_zoom_ratio', Float32(ratio))

    def set_pan_x_ratio(self, ratio):
        """Set the image horizontal pan ratio.

        Args:
            ratio (float): Horizontal pan as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_pan_x_ratio', Float32(ratio))

    def set_pan_y_ratio(self, ratio):
        """Set the image vertical pan ratio.

        Args:
            ratio (float): Vertical pan as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_pan_y_ratio', Float32(ratio))

    def set_zoom_3d_ratio(self, ratio):
        """Set the 3D render zoom ratio.

        Args:
            ratio (float): 3D render zoom as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_zoom_3d_ratio', Float32(ratio))

    def set_rotate_3d_ratio(self, ratio):
        """Set the 3D render rotation ratio.

        Args:
            ratio (float): 3D render rotation as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_rotate_3d_ratio', Float32(ratio))

    def set_tilt_3d_ratio(self, ratio):
        """Set the 3D render tilt ratio.

        Args:
            ratio (float): 3D render tilt as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_tilt_3d_ratio', Float32(ratio))

    def set_filter_enable(self, update_msg):
        """Enable or disable a named image filter.

        Args:
            update_msg (UpdateBool): An UpdateBool msg naming the filter and carrying its enable value.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_filter_enable', update_msg)

    def set_filter_ratio(self, update_msg):
        """Set the ratio of a named image filter.

        Args:
            update_msg (UpdateFloat): An UpdateFloat msg naming the filter and carrying its ratio value.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub('connect_image_set_filter_ratio', update_msg)


    ###############################
    # Class Private Methods
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
