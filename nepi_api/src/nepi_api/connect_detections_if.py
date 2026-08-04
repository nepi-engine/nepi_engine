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

import copy
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty

from nepi_interfaces.msg import Detections, DetectorStatus

from nepi_api.messages_if import MsgIF

from nepi_api.connect_node_if import ConnectNodeIF
from nepi_api.connect_node_if import ConnectNodeClassIF




#########################################
# Connect Detections IF Class
#########################################
# Client-side connect class mirroring process_if.DetectionsIF. The connected
# source publishes a nepi_interfaces/Detections message on its base namespace
# and a nepi_interfaces/DetectorStatus message on <namespace>/status. This
# connect class subclasses ConnectNodeIF directly and follows the
# ConnectNavPoseIF / ConnectTargetsIF pattern: retrieved Detections messages are
# converted to detections dictionaries and cached (thread-safe) for polling
# consumers, or handed straight to dataCB when one is provided.


DETECTIONS_CONNECT_ID = 'DETECTIONS'
DETECTIONS_CONNECT_STATUS_MSG = 'DetectorStatus'
DETECTIONS_CONNECT_NAME = 'detections_connect'


CONNECTED_TIMEOUT = 2


class ConnectDetectionsIF(ConnectNodeIF):

    # ADD Additional Connect Callback Functions


    msg_if = None
    ready = False
    namespace = '~'

    node_if = None

    status_msg = None
    connected = False
    last_status_time = 0

    statusCb = None # Backwards Compatibility

    data_dict = None
    data_dict_lock = threading.Lock()

    get_data = False
    got_data = False

    preprocessFunction = None
    dataCB = None

    connect_topic_subs_dict = None
    connect_topic_pubs_dict = None
    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = DETECTIONS_CONNECT_NAME,
                namespace = None,
                statusCb = None,
                preprocess_function = None,
                dataCB = None,
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
                connect_id = DETECTIONS_CONNECT_ID,
                connect_status_msg = DETECTIONS_CONNECT_STATUS_MSG,
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

        self.statusCb = statusCb
        self.preprocessFunction = preprocess_function
        self.dataCB = dataCB


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
        """Return the fully-resolved ROS namespace for the connected detections source.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.selected_topic

    def check_connection(self):
        """Check whether the detections source is currently connected.

        Returns:
            bool: True if a status message has been received within the connection timeout window,
                False otherwise.
        """
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        """Block until the detections source is connected or the timeout expires.

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
        """Check whether the status topic from the detections source is currently connected.

        Returns:
            bool: True if status messages are being received, False otherwise.
        """
        return self.connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        """Block until the detections source status topic is connected or the timeout expires.

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
        """Return the latest detections source status as a dictionary.

        Returns:
            dict: A dictionary representation of the most recent DetectorStatus message,
                or None if no status has been received yet.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def get_status_msg(self):
        """Return the latest detections source status as a msg.

        Returns:
            DetectorStatus: The most recent DetectorStatus message,
                or None if no status has been received yet.
        """
        return self.status_msg

    def get_data_topic(self):
        """Return the detections data topic of the connected data source.

        Returns:
            str: The selected data source namespace, which is also the Detections topic
                the source publishes on, or 'None' if no source is selected.
        """
        return self.selected_topic

    def set_get_data(self, state):
        """Set the flag requesting capture of the next available detections message.

        Args:
            state (bool): True to request the next message, False to clear the request.

        Returns:
            bool: Always True.
        """
        self.get_data = state
        return True

    def read_get_got_states(self):
        """Return the current get and got data flags.

        Returns:
            list: A two-element list [get_data, got_data] where get_data indicates
                whether a detections message has been requested and got_data indicates
                whether a detections message is waiting to be retrieved.
        """
        return [self.get_data, self.got_data]

    def get_detections_dict(self):
        """Retrieve and consume the latest captured detections data dictionary.

        Thread-safe. Clears the stored data after retrieval so subsequent calls
        return None until a new detections message arrives.

        Returns:
            dict: The detections data dictionary containing the detections values,
                namespace, and timestamp, or None if no detections are available.
        """
        self.data_dict_lock.acquire()
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = None
        self.data_dict_lock.release()
        return data_dict

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
        self.msg_if.pub_warn("subscribe_data_topic Called")

        success = False
        success = self.unsubscribe_topic()

        # Subscribers Config Dict ####################
        self.connect_topic_subs_dict = {
            'status_sub': {
                'namespace': self.selected_topic,
                'topic': 'status',
                'msg': DetectorStatus,
                'qsize': 10,
                'callback': self._statusCb
            },
            'data_sub': {
                'namespace': self.selected_topic,
                'topic': '',
                'msg': Detections,
                'qsize': 1,
                'callback': self._dataCb
            }
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
            self.msg_if.pub_warn("unsubscribe_topic Called")

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
            self.msg_if.pub_warn("Connected to DETECTIONS Status:  " + str(self.selected_topic))
            self.connecting = False
            self.connected_topic = self.selected_topic
        self.connected = True
        self.status_msg = status_msg

        if self.statusCb is not None:
            status_dict = self.get_status_dict()
            self.statusCb(status_dict)


    def _dataCb(self,data_msg):
        # Only build a detections dict when a consumer has asked for one (get_data
        # flag) or a dataCB is registered; otherwise the incoming
        # Detections message is dropped cheaply. Connection state is driven by the
        # status callback. Detections carries a float64 timestamp field (no
        # std_msgs Header), so latency is not computed here.
        get_data = (self.dataCB is not None or self.get_data == True)
        if get_data == False:
            return

        start_time = nepi_sdk.get_time()

        self.get_data = False

        ##############################
        ### Preprocess Detections
        data = nepi_sdk.convert_msg2dict(data_msg)

        if self.preprocessFunction is not None:
            try:
                data = self.preprocessFunction(data)
            except Exception as e:
                self.msg_if.pub_warn("Provided Detections Preprocess Function failed:  " + str(e))

        data_dict = dict()
        data_dict['namespace'] = self.selected_topic
        data_dict['data'] = data
        data_dict['timestamp'] = data_msg.timestamp
        ##############################

        process_time = round( (nepi_sdk.get_time() - start_time) , 3)
        data_dict['process_time'] = process_time

        if self.dataCB is not None:
            self.dataCB(data_dict)
        else:
            self.data_dict_lock.acquire()
            self.data_dict = data_dict
            self.data_dict_lock.release()
            self.got_data = True
