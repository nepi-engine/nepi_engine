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

# Connect (client) classes for the utils_if interfaces. Each one connects to an
# interface another node is running, by that interface's namespace, reads its
# status topic and publishes on exactly the command topics the interface
# subscribes to.
#
# ReadWriteIF has no connect class: it is a local file I/O helper with no
# publishers, subscribers or services, so there is nothing to connect to.

import copy


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data


from std_msgs.msg import Empty, String

from nepi_interfaces.msg import ControlsStatus, UpdateControl
from nepi_interfaces.msg import DataStatus
from nepi_interfaces.msg import UpdateBool, UpdateOrder
# ListIF publishes the type utils_if imports as ListIFStatus, which is
# SelectorStatus on the wire (ListIFStatus.msg was renamed SelectorStatus.msg)
from nepi_interfaces.msg import SelectorStatus

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF



##########################################
## ConnectControlsIF


class ConnectControlsIF:
    """Client side of a remote node's ControlsIF.

    The controls dict, values and capabilities alike, arrives in the
    ControlsStatus message on <namespace>/status. A value change is sent as an
    UpdateControl on <namespace>/update_control, and a reset as an Empty on
    <namespace>/reset_controls -- the only two topics ControlsIF subscribes to.
    """

    msg_if = None
    ready = False
    namespace = None

    con_node_if = None

    status_msg = None
    status_connected = False

    #######################
    ### IF Initialization
    def __init__(self, namespace = None, timeout = 1.0,
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
        self.msg_if.pub_info("Starting Connect Controls IF Initialization Processes" )
        ##############################

        #############################
        # Initialize Class Variables
        # The namespace is the ControlsIF's own namespace, the one its status
        # topic sits on. ControlsIF roots itself at <namespace>/<name>, and its
        # name defaults to 'controls', so with no namespace given this connects
        # to a default-named ControlsIF on this node.
        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.node_namespace,'controls')
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.msg_if.pub_info("Using controls namespace: " + self.namespace )

        ##############################
        # Create NodeClassIF Class

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'update_control': {
                'msg': UpdateControl,
                'namespace': self.namespace,
                'topic': 'update_control',
                'qsize': 1,
                'latch': False
            },
            'reset_controls': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset_controls',
                'qsize': 1,
                'latch': False
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'msg': ControlsStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'callback': self._statusCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(pubs_dict = self.PUBS_DICT,
                                            subs_dict = self.SUBS_DICT,
                                            msg_if = self.msg_if
                                            )

        nepi_sdk.wait()

        ##############################
        # Complete Initialization
        self.ready = True
        if timeout > 0:
            self.wait_for_status_connection(timeout = timeout)
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return True once this client's publishers and subscriber are set up."""
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the client is ready or the timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: The ready state when the wait ended.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for ready" )
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Not ready" )
            else:
                self.msg_if.pub_info("Ready" )
        return self.ready

    def get_namespace(self):
        """Return the fully resolved namespace of the remote ControlsIF."""
        return self.namespace

    def check_status_connection(self):
        """Return True if a ControlsStatus message has been received."""
        return self.status_connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        """Block until the first ControlsStatus message arrives or the timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if a status message has been received.
        """
        self.msg_if.pub_info("Waiting for status connection on: " + self.namespace )
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.status_connected == False:
            self.msg_if.pub_info("No status received yet from: " + self.namespace )
        else:
            self.msg_if.pub_info("Status connected" )
        return self.status_connected

    def get_status_dict(self):
        """Return the last ControlsStatus message as a dict.

        Returns:
            dict: The status message as a dict, or None if no status has arrived yet.
        """
        status_dict = None
        status_msg = self.status_msg
        if status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(status_msg)
        return status_dict

    def get_controls_dict(self):
        """Return the remote controls dict, parsed from the last status message.

        Returns:
            dict: The controls dict in nepi_controls form (control name to
                control dict, plus the controls_* meta keys), or None if no
                status has arrived yet.
        """
        controls_dict = None
        status_msg = self.status_msg
        if status_msg is not None:
            controls_dict = nepi_controls.convert_controls_msg2dict(status_msg)
        return controls_dict

    def get_control_names(self):
        """Return the names of the remote controls, in the order the status lists them.

        Returns:
            list: Control name strings, or an empty list if no status has arrived yet.
        """
        control_names = []
        status_msg = self.status_msg
        if status_msg is not None:
            control_names = list(status_msg.controls_name_list)
        return control_names

    def has_control(self, control_name):
        """Return True if the last status message lists the named control.

        Args:
            control_name (str): Name of the control.

        Returns:
            bool: True if the control exists on the remote ControlsIF.
        """
        return control_name in self.get_control_names()

    def get_control_value(self, control_name):
        """Return a control's value as last reported by the remote ControlsIF.

        Args:
            control_name (str): Name of the control.

        Returns:
            The control's value in its native form, or None if the control is
            unknown or no status has arrived yet.
        """
        controls_dict = self.get_controls_dict()
        value = nepi_controls.get_value(controls_dict, control_name)
        return value

    def set_control_value(self, control_name, update_value, index = None):
        """Publish a value update for one control on the remote ControlsIF.

        Sends an UpdateControl carrying only the name, value and index, which is
        everything ControlsIF applies from that message.

        Args:
            control_name (str): Name of the control.
            update_value: New value. A list or tuple is sent entry by entry;
                anything else is sent as one value.
            index (int, optional): Component index for a multi-value control.
                Defaults to None, which replaces the whole value.
        """
        value_list = nepi_controls.get_value_list(update_value)
        if value_list is None:
            self.msg_if.pub_warn("No value given for control: " + str(control_name) )
            return
        msg = UpdateControl()
        msg.name = str(control_name)
        msg.value = value_list
        msg.index = '' if index is None else str(index)
        self.con_node_if.publish_pub('update_control', msg)

    def reset_controls(self):
        """Publish a reset, which reloads the remote controls from their saved params."""
        self.con_node_if.publish_pub('reset_controls', Empty())

    def unregister(self):
        """Unregister this client's publishers and subscriber and mark it not ready.

        Returns:
            bool: True when unregistering has finished.
        """
        if self.con_node_if is not None:
            self.con_node_if.unregister_class()
            nepi_sdk.sleep(1)
        self.ready = False
        self.status_connected = False
        self.status_msg = None
        return True

    ###############################
    # Class Private Methods
    ###############################

    def _statusCb(self, msg):
        self.status_msg = msg
        self.status_connected = True




##########################################
## ConnectDataIF


class ConnectDataIF:
    """Client side of a remote node's DataIF.

    DataIF publishes its data dict in the DataStatus message on
    <namespace>/status and subscribes to no command topics, so this client is
    read-only.
    """

    msg_if = None
    ready = False
    namespace = None

    con_node_if = None

    status_msg = None
    status_connected = False

    #######################
    ### IF Initialization
    def __init__(self, namespace = None, timeout = 1.0,
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
        self.msg_if.pub_info("Starting Connect Data IF Initialization Processes" )
        ##############################

        #############################
        # Initialize Class Variables
        # The namespace is the DataIF's own namespace. DataIF roots itself at
        # <namespace>/<name> with the name defaulting to 'data'.
        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.node_namespace,'data')
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.msg_if.pub_info("Using data namespace: " + self.namespace )

        ##############################
        # Create NodeClassIF Class

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'msg': DataStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'callback': self._statusCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(subs_dict = self.SUBS_DICT,
                                            msg_if = self.msg_if
                                            )

        nepi_sdk.wait()

        ##############################
        # Complete Initialization
        self.ready = True
        if timeout > 0:
            self.wait_for_status_connection(timeout = timeout)
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return True once this client's subscriber is set up."""
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the client is ready or the timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: The ready state when the wait ended.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for ready" )
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Not ready" )
            else:
                self.msg_if.pub_info("Ready" )
        return self.ready

    def get_namespace(self):
        """Return the fully resolved namespace of the remote DataIF."""
        return self.namespace

    def check_status_connection(self):
        """Return True if a DataStatus message has been received."""
        return self.status_connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        """Block until the first DataStatus message arrives or the timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if a status message has been received.
        """
        self.msg_if.pub_info("Waiting for status connection on: " + self.namespace )
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.status_connected == False:
            self.msg_if.pub_info("No status received yet from: " + self.namespace )
        else:
            self.msg_if.pub_info("Status connected" )
        return self.status_connected

    def get_status_dict(self):
        """Return the last DataStatus message as a dict.

        Returns:
            dict: The status message as a dict, or None if no status has arrived yet.
        """
        status_dict = None
        status_msg = self.status_msg
        if status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(status_msg)
        return status_dict

    def get_data_dict(self):
        """Return the remote data dict, parsed from the last status message.

        Returns:
            dict: The data dict in nepi_data form (datum name to datum dict,
                plus the data_* meta keys), or None if no status has arrived yet.
        """
        data_dict = None
        status_msg = self.status_msg
        if status_msg is not None:
            data_dict = nepi_data.convert_data_msg2dict(status_msg)
        return data_dict

    def get_datum_names(self):
        """Return the names of the remote data, in the order the status lists them.

        Returns:
            list: Datum name strings, or an empty list if no status has arrived yet.
        """
        datum_names = []
        status_msg = self.status_msg
        if status_msg is not None:
            datum_names = list(status_msg.data_name_list)
        return datum_names

    def has_datum(self, datum_name):
        """Return True if the last status message lists the named datum.

        Args:
            datum_name (str): Name of the datum.

        Returns:
            bool: True if the datum exists on the remote DataIF.
        """
        return datum_name in self.get_datum_names()

    def get_datum_value(self, datum_name):
        """Return a datum's value as last reported by the remote DataIF.

        Args:
            datum_name (str): Name of the datum.

        Returns:
            The datum's value in its native form, or None if the datum is
            unknown or no status has arrived yet.
        """
        data_dict = self.get_data_dict()
        value = nepi_data.get_value(data_dict, datum_name)
        return value

    def unregister(self):
        """Unregister this client's subscriber and mark it not ready.

        Returns:
            bool: True when unregistering has finished.
        """
        if self.con_node_if is not None:
            self.con_node_if.unregister_class()
            nepi_sdk.sleep(1)
        self.ready = False
        self.status_connected = False
        self.status_msg = None
        return True

    ###############################
    # Class Private Methods
    ###############################

    def _statusCb(self, msg):
        self.status_msg = msg
        self.status_connected = True




##########################################
## ConnectListIF

# The list state keys ListIF.get_list_dict() returns
LIST_DICT_KEYS = ['ordered_items_list','ordered_names_list','active_items_list','selected_item','selected_item_index']


class ConnectListIF:
    """Client side of a remote ListIF.

    ListIF publishes its list state on <namespace>/status and subscribes to
    select_item, reset and refresh. It subscribes to enable_all, disable_all
    and update_state only when multi-select is enabled, and to update_order
    only when order updates are enabled; the status message reports both
    flags. This client registers every topic, so a command sent to a disabled
    topic reaches no subscriber and is ignored.
    """

    msg_if = None
    ready = False
    namespace = None

    con_node_if = None

    status_msg = None
    status_connected = False

    #######################
    ### IF Initialization
    def __init__(self, namespace = None, timeout = 1.0,
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
        self.msg_if.pub_info("Starting Connect List IF Initialization Processes" )
        ##############################

        #############################
        # Initialize Class Variables
        # Same rule as ListIF: the namespace is used as given, and defaults to
        # <node namespace>/list.
        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.node_namespace,'list')
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.msg_if.pub_info("Using list namespace: " + self.namespace )

        ##############################
        # Create NodeClassIF Class

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'select_item': {
                'msg': String,
                'namespace': self.namespace,
                'topic': 'select_item',
                'qsize': 1,
                'latch': False
            },
            'reset': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset',
                'qsize': 1,
                'latch': False
            },
            'refresh': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'refresh',
                'qsize': 1,
                'latch': False
            },
            'enable_all': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'enable_all',
                'qsize': 1,
                'latch': False
            },
            'disable_all': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'disable_all',
                'qsize': 1,
                'latch': False
            },
            'update_state': {
                'msg': UpdateBool,
                'namespace': self.namespace,
                'topic': 'update_state',
                'qsize': 1,
                'latch': False
            },
            'update_order': {
                'msg': UpdateOrder,
                'namespace': self.namespace,
                'topic': 'update_order',
                'qsize': 1,
                'latch': False
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'msg': SelectorStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'callback': self._statusCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(pubs_dict = self.PUBS_DICT,
                                            subs_dict = self.SUBS_DICT,
                                            msg_if = self.msg_if
                                            )

        nepi_sdk.wait()

        ##############################
        # Complete Initialization
        self.ready = True
        if timeout > 0:
            self.wait_for_status_connection(timeout = timeout)
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return True once this client's publishers and subscriber are set up."""
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the client is ready or the timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: The ready state when the wait ended.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for ready" )
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Not ready" )
            else:
                self.msg_if.pub_info("Ready" )
        return self.ready

    def get_namespace(self):
        """Return the fully resolved namespace of the remote ListIF."""
        return self.namespace

    def check_status_connection(self):
        """Return True if a list status message has been received."""
        return self.status_connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        """Block until the first list status message arrives or the timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if a status message has been received.
        """
        self.msg_if.pub_info("Waiting for status connection on: " + self.namespace )
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.status_connected == False:
            self.msg_if.pub_info("No status received yet from: " + self.namespace )
        else:
            self.msg_if.pub_info("Status connected" )
        return self.status_connected

    def get_status_dict(self):
        """Return the last list status message as a dict.

        Returns:
            dict: The status message as a dict, or None if no status has arrived yet.
        """
        status_dict = None
        status_msg = self.status_msg
        if status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(status_msg)
        return status_dict

    def get_list_dict(self):
        """Return the remote list state, in the form ListIF.get_list_dict() returns.

        Returns:
            dict: ordered_items_list, ordered_names_list, active_items_list,
                selected_item and selected_item_index, or None if no status
                has arrived yet.
        """
        list_dict = None
        status_dict = self.get_status_dict()
        if status_dict is not None:
            list_dict = dict()
            for key in LIST_DICT_KEYS:
                list_dict[key] = copy.deepcopy(status_dict.get(key, None))
        return list_dict

    def select_item(self, item):
        """Publish a selection of one item on the remote list.

        Args:
            item (str): The item to select.
        """
        self.con_node_if.publish_pub('select_item', String(data = str(item)))

    def reset_list(self):
        """Publish a reset, which returns the remote list to its initial state."""
        self.con_node_if.publish_pub('reset', Empty())

    def refresh_list(self):
        """Publish a refresh, which asks the remote list owner to rebuild its list."""
        self.con_node_if.publish_pub('refresh', Empty())

    def enable_all(self):
        """Publish an enable-all, which marks every item active. Multi-select lists only."""
        self.con_node_if.publish_pub('enable_all', Empty())

    def disable_all(self):
        """Publish a disable-all, which clears the active items. Multi-select lists only."""
        self.con_node_if.publish_pub('disable_all', Empty())

    def update_state(self, item, active_state):
        """Publish an item's active state. Multi-select lists only.

        Args:
            item (str): The item to activate or deactivate.
            active_state (bool): True to mark the item active, False to deactivate it.
        """
        msg = UpdateBool()
        msg.name = str(item)
        msg.value = bool(active_state)
        self.con_node_if.publish_pub('update_state', msg)

    def update_order(self, item, move_cmd):
        """Publish a move of one item in the remote list order. Order-update lists only.

        Args:
            item (str): The item to move.
            move_cmd (str): One of UpdateOrder.MOVE_TO_TOP ('top'), MOVE_UP ('up'),
                MOVE_DOWN ('down') or MOVE_BOTTOM ('bottom').
        """
        msg = UpdateOrder()
        msg.name = str(item)
        msg.move_cmd = str(move_cmd)
        self.con_node_if.publish_pub('update_order', msg)

    def unregister(self):
        """Unregister this client's publishers and subscriber and mark it not ready.

        Returns:
            bool: True when unregistering has finished.
        """
        if self.con_node_if is not None:
            self.con_node_if.unregister_class()
            nepi_sdk.sleep(1)
        self.ready = False
        self.status_connected = False
        self.status_msg = None
        return True

    ###############################
    # Class Private Methods
    ###############################

    def _statusCb(self, msg):
        self.status_msg = msg
        self.status_connected = True
