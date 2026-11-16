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


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty

from nepi_api.messages_if import MsgIF

from nepi_api.connect_node_if import ConnectNodeIF





#########################################
# Connect Device IF Class
#########################################

# Shared base for the device connect classes (ConnectIDXDeviceIF,
# ConnectLSXDeviceIF, ConnectNPXDeviceIF, ConnectPTXDeviceIF, ConnectRBXDeviceIF,
# ConnectSVXDeviceIF), split the way ConnectDataProductIF is split from the image
# connects. ConnectNodeIF owns discovery, selection and staleness. This class adds
# what every device connect does on top of that: the device status subscription,
# the device's control publishers, the config commands, and the connection
# getters and waits.
#
# connect_status_msg is the status msg TYPE NAME, as a string, which ConnectNodeIF
# matches against live topic types. connect_status_msg_class is the matching msg
# CLASS, which the status subscriber needs. Neither consumer accepts the other's
# form, so both are passed.
#
# A subclass sets everything it owns BEFORE calling this __init__: its callbacks,
# and its connect_topic_controls_dict, one publisher entry per device control
# topic with the 'unknown' namespace sentinel (meaning "the selected device") and a
# bare key. subscribe_topic resolves the sentinel and prefixes the key with
# self.node_if_prefix on every selection. Nothing this class starts can then reach
# a half-built subclass. Device-specific subscribers, services and state extend
# subscribe_topic, unsubscribe_topic, _statusCb and updaterCb by overriding them
# and calling super().
#
# Registry keys carry self.node_if_prefix (the connect's namespace below the node,
# e.g. 'rbx_connect/'). One node_if can hold several device connects (an app with
# an RBX and an NPX connect, a pan/tilt pair of SVX connects), and its registries
# are keyed, so a bare key would let one connect overwrite or unregister another's
# entries. ROS names derive from namespace+topic, not the key, so the prefix is
# wire-invisible.

CONNECT_ID='CONNECT_DEVICE'
CONNECT_STATUS_MSG= None
CONNECT_STATUS_MSG_CLASS = None
CONNECT_NAME='connect_device'


CONNECTED_TIMEOUT = 2


class ConnectDeviceIF(ConnectNodeIF):

    # ADD Additional Connect Callback Functions


    msg_if = None
    ready = False
    namespace = '~'

    node_if = None

    status_msg = None
    connected = False
    last_status_time = 0

    statusCb = None # Backwards Compatibility

    connect_status_msg_class = None

    # Namespace of the node hosting the selected device, one level above the device
    # IF namespace: DeviceIF builds <node>/<id>, while the node's NodeConfigsIF and
    # the device's SaveDataIF sit at <node>. Set on each selection.
    device_node_namespace = 'None'

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
                filter_topics_list = None,
                exclude_namespaces_list = None,
                selected_topic = None,
                auto_select_enabled = True,
                statusCb = None,
                connect_topic_controls_dict = None,
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):

        # Raise rather than return. ConnectNodeIF returns early, debug log only, when
        # either is missing, which leaves a half-built object that fails much later.
        if connect_status_msg is None:
            raise ValueError(str(type(self).__name__) +
                    ": connect_status_msg is required (status msg type name, as a string)")
        if connect_status_msg_class is None:
            raise ValueError(str(type(self).__name__) +
                    ": connect_status_msg_class is required (status msg class)")

        if filter_topics_list is None:
            filter_topics_list = []

        ##############################
        # Initialize Class Variables
        # Set before ConnectNodeIF.__init__, which arms the discovery timer that
        # subscribes, so no callback can run before these exist.
        self.connect_status_msg_class = connect_status_msg_class
        self.connect_topic_controls_dict = connect_topic_controls_dict
        self.statusCb = statusCb

        self.msg_if = msg_if
        self.node_if = node_if
        super().__init__(
                connect_id = connect_id,
                connect_name = connect_name,
                namespace = namespace,
                connect_status_msg = connect_status_msg,
                filter_topics_list = filter_topics_list,
                exclude_namespaces_list = exclude_namespaces_list,
                selected_topic = selected_topic,
                auto_select_enabled = auto_select_enabled,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = self.msg_if,
                node_if = self.node_if
                )
        ####  IF INIT SETUP ####

        self.wait_for_connect_ready()

        # NodeIF.__init__ leaves this IF's own ConnectIFStatus in self.status_msg.
        # The device status is None until the selected device publishes one.
        self.status_msg = None


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
        if type(self) is ConnectDeviceIF:
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
        """Return the fully-resolved ROS namespace for the connected device.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.selected_topic

    def wait_for_connection(self, timeout = float('inf') ):
        """Block until the device is connected or the timeout expires.

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
        """Check whether the status topic from the device is currently connected.

        Returns:
            bool: True if status messages are being received, False otherwise.
        """
        return self.connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        """Block until the device status topic is connected or the timeout expires.

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

    def save_config(self):
        """Publish a save configuration command to persist current settings on the device.
        """
        self.node_if.publish_pub(self.node_if_prefix + 'save_config',Empty())

    def reset_config(self):
        """Publish a reset configuration command to restore the last saved settings on the device.
        """
        self.node_if.publish_pub(self.node_if_prefix + 'reset_config',Empty())

    def factory_reset_config(self):
        """Publish a factory reset command to restore factory default settings on the device.
        """
        self.node_if.publish_pub(self.node_if_prefix + 'factory_reset_config',Empty())

    ###############################
    # Class Private Methods
    ###############################

    # Staleness (no status inside CONNECTED_TIMEOUT) is ConnectNodeIF._updaterCb's
    # job: it runs that check, and also clears connected_topic so its next cycle
    # tears down and resubscribes. A second copy here raced that chain, so this
    # chain does none of it. A subclass that keeps device-specific state in step
    # with the connection overrides this and calls super(), which re-arms it.
    def updaterCb(self,timer):
        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)




    def subscribe_topic(self, topic):
        self.msg_if.pub_debug("subscribe_topic Called")

        success = False
        success = self.unsubscribe_topic()

        self.device_node_namespace = os.path.dirname(self.selected_topic)

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



        # Publishers Config Dict ####################
        # The config topics live under the device's node namespace, where the node's
        # NodeConfigsIF subscribes them, not under the device IF namespace.
        self.connect_topic_pubs_dict = {
            self.node_if_prefix + 'save_config': {
                'namespace': self.device_node_namespace,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': 1,
            },
            self.node_if_prefix + 'reset_config': {
                'namespace': self.device_node_namespace,
                'topic': 'reset_config',
                'msg': Empty,
                'qsize': 1,
            },
            self.node_if_prefix + 'factory_reset_config': {
                'namespace': self.device_node_namespace,
                'topic': 'factory_reset_config',
                'msg': Empty,
                'qsize': 1,
            }
        }

        # Controls Config Dict ####################
        # Resolve into per-entry copies, as ConnectDataProductIF does. Writing
        # self.selected_topic back into self.connect_topic_controls_dict would strip
        # the 'unknown' sentinel for good, and the next selection would re-register
        # every control publisher against the previous device.
        if self.connect_topic_controls_dict is not None:
            controls_dict = self._cleanNodeClassDict(self.connect_topic_controls_dict)
            for control_name in controls_dict.keys():
                control_dict = dict(controls_dict[control_name])
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
        # Gate on what is registered too. ConnectNodeIF's staleness check clears
        # connecting and connected without unregistering anything, and a gate on
        # those two alone let the next subscribe_topic register over live
        # subscribers, leaving the old ones running.
        if self.connecting == True or self.connected == True or \
                self.connect_topic_subs_dict is not None or self.connect_topic_pubs_dict is not None:
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
            success = True
        return success


    # Register a subclass's extra entries on top of what subscribe_topic registered,
    # and add them to the connect dicts so unsubscribe_topic tears them down too.
    def _registerConnectTopics(self, pubs_dict = None, subs_dict = None):
        if pubs_dict is not None and len(pubs_dict) > 0:
            if self.node_if is not None:
                self.node_if.register_pubs(pubs_dict)
            if self.connect_topic_pubs_dict is None:
                self.connect_topic_pubs_dict = dict()
            self.connect_topic_pubs_dict.update(pubs_dict)
        if subs_dict is not None and len(subs_dict) > 0:
            if self.node_if is not None:
                self.node_if.register_subs(subs_dict)
            if self.connect_topic_subs_dict is None:
                self.connect_topic_subs_dict = dict()
            self.connect_topic_subs_dict.update(subs_dict)


    def _statusCb(self,status_msg):
        self.last_status_time = nepi_utils.get_time()
        if self.connected == False:
            self._announceConnected(self.connect_id)
            self.connecting = False
            self.connected_topic = self.selected_topic
        self.connected = True
        self.status_msg = status_msg

        if self.statusCb is not None:
            status_dict = self.get_status_dict()
            self.statusCb(status_dict)
