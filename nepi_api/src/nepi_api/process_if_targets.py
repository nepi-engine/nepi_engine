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
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data
from nepi_sdk import nepi_process

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image



from nepi_interfaces.msg import ImageStatus
from nepi_interfaces.msg import Targets, TargetsStatus


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF


#########################################
# Targets Data Product IF Class
#########################################

class TargetsIF:
    """Per-data-product interface for the AI detector 'targets' product.

    Owns the publishers, status message, and save-data registration for a
    single targets data product. Publishes a nepi_interfaces/Targets message on
    the data topic and a nepi_interfaces/TargetsStatus message on the status
    topic, and saves targeting results through a SaveDataIF. Mirrors DetectionsIF
    using the targeting message types and the targeting namespace.
    """

    ready = False

    data_product = 'targets'

    namespace = '~/targets'

    node_if = None
    config_topic = ''
    node_if_shared = True

    save_data_if = None
    save_data_enabled = True

    status_msg = TargetsStatus()

    data_msg_type = Targets
    status_msg_type = TargetsStatus

    data_pub_name = 'targets_pub'
    status_pub_name = 'targets_status_pub'

    needs_data = False
    last_pub_time = None
    time_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    PUBS_DICT = dict()

    def __init__(self, namespace = None,
                data_product = None,
                save_data_if = None,
                save_data_enabled = True,
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
            log_name = nepi_utils.get_clean_name(log_name)
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################
        # Initialize Class Variables
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product

        if namespace is None:
            namespace = self.node_namespace
        if os.path.basename(namespace) != self.data_product:
            namespace = nepi_sdk.create_namespace(namespace, self.data_product)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        # Initialize status message
        self.status_msg = self.status_msg_type()
        self.status_msg.process_status.node_name = self.node_name
        self.status_msg.process_status.namespace = self.namespace

        ##############################
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.status_pub_name: {
                'msg': self.status_msg_type,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': False
            },
            self.data_pub_name: {
                'msg': self.data_msg_type,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = None

        # Update or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            self.node_if.register_pubs(self.PUBS_DICT)
        else:
            self.config_if = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()

        ####################
        # Save Data Setup
        self.save_data_enabled = save_data_enabled
        if self.save_data_enabled == True:
            self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
            if save_data_if is not None and save_data_if != 'None':
                self.save_data_if = save_data_if
                data_products = self.save_data_if.get_data_products()
                if self.data_product not in data_products:
                    self.save_data_if.register_data_product(self.data_product)
            elif save_data_if != 'None':
                # Setup Save Data IF Class
                self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
                factory_data_rates = dict()
                factory_data_rates[self.data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

                factory_filename_dict = {
                    'prefix': "",
                    'add_timestamp': True,
                    'add_ms': True,
                    'add_us': False,
                    'suffix': "",
                    'add_node_name': True
                }

                self.save_data_if = SaveDataIF(namespace = self.node_namespace,
                                        data_products = [self.data_product],
                                        factory_rate_dict = factory_data_rates,
                                        factory_filename_dict = factory_filename_dict,
                                        log_name_list = self.log_name_list,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if)
                nepi_sdk.sleep(1)

            if self.save_data_if is not None:
                self.status_msg.process_status.save_data_topic = self.save_data_if.get_namespace()
                self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.process_status.save_data_topic), log_name_list = self.log_name_list)

        ##############################
        # Update vals and publish first status
        self.init(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete", log_name_list = self.log_name_list)
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

    def wait_for_ready(self, timeout = float('inf')):
        """Block until the interface is ready or a timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for ready", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace for this data product interface.

        Returns:
            str: The fully-qualified ROS namespace.
        """
        return self.namespace

    def get_data_product(self):
        """Return the data product name for this interface.

        Returns:
            str: The data product identifier string (e.g. 'targets').
        """
        return self.data_product

    def get_status_dict(self):
        """Return the current status message converted to a plain dictionary.

        Returns:
            dict: Status fields as a dictionary, or None if no status message exists.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def needs_data_check(self):
        """Return whether downstream consumers currently need targets data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        return copy.deepcopy(self.needs_data)

    def publish_data(self, data_msg, timestamp = None):
        """Publish a targets message and save it if saving is enabled.

        Publishes the provided data message on the data product topic, updates
        the publish-rate statistics, and — if a SaveDataIF is registered and the
        data product is due to be saved or snapshotted — converts the message to
        a dictionary and saves it.

        Args:
            data_msg (nepi_interfaces/Targets): The targets message to publish.
            timestamp (float, optional): Acquisition timestamp in seconds. Defaults
                to the current time when None.

        Returns:
            bool: True if the message was published, False otherwise.
        """
        if self.node_if is None or data_msg is None:
            return False
        
        # data_dict = nepi_sdk.convert_msg2dict(data_msg)
        # timestamps=[data_dict['timestamp'],data_dict['source_timestamp']] 
        # targets_list = data_dict['targets']
        # for target in targets_list:
        #     timestamps.append(target['timestamp'])
        # self.msg_if.pub_warn("Pub Targets timestamps" + str(timestamps), throttle_s = 10)

        self.node_if.publish_pub(self.data_pub_name, data_msg)
        self._updatePubStats()
        if self.save_data_if is not None:
            should_save = self.save_data_if.data_product_should_save(self.data_product) == True
            snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(self.data_product) == True
            if should_save or snapshot_enabled:
                if timestamp is None:
                    timestamp = nepi_utils.get_time()
                try:
                    data_dict = nepi_sdk.convert_msg2dict(data_msg)
                    self.save_data_if.save(self.data_product, data_dict, timestamp = timestamp)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to save " + self.data_product + " data: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
        return True

    def publish_status(self, status_msg = None):
        """Publish the data product status message.

        Args:
            status_msg (optional): A pre-built status message to publish and store.
                When None, the interface's current status message is published.
        """
        if self.node_if is None:
            return
        if status_msg is not None:
            #self.status_msg = status_msg
            #self.msg_if.pub_warn("Publishing Detections Status: " + str(status_msg), log_name_list = self.log_name_list, throttle_s = 20.0)
            self.node_if.publish_pub(self.status_pub_name, status_msg)

    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this interface."""
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.PUBS_DICT is not None:
                    for pub_name in self.PUBS_DICT.keys():
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
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the interface to its initialized state."""
        self.init()

    def factory_reset(self):
        """Reset the interface to factory defaults."""
        self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _updatePubStats(self):
        if self.last_pub_time is None:
            pub_time_sec = 1.0
            self.last_pub_time = nepi_utils.get_time()
        else:
            cur_time = nepi_utils.get_time()
            pub_time_sec = cur_time - self.last_pub_time
            self.last_pub_time = cur_time
        self.time_list.pop(0)
        self.time_list.append(pub_time_sec)

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _needsDataCheckCb(self, timer):
        has_subs = self.node_if.pub_has_subscribers(self.data_pub_name)
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
            needs_data = has_subs or needs_save or needs_snapshot
        else:
            needs_data = has_subs
        self.needs_data = needs_data
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)

    def _publishStatusCb(self, timer):
        self.publish_status()


#########################################
# Targets Image Data Product IF Class
#########################################

class TargetsImageIF:
    """Per-data-product interface for the AI detector 'targets_image' product.

    Owns the publishers, status message, and save-data registration for the
    targets overlay image data product. Publishes a sensor_msgs/Image message on
    the data topic and a nepi_interfaces/ImageStatus message on the status
    topic, and saves the rendered image through a SaveDataIF. Mirrors
    DetectionsImageIF using the targeting namespace.
    """

    ready = False

    data_product = 'targets_image'

    namespace = '~/targets_image'

    node_if = None
    config_topic = ''
    node_if_shared = True

    save_data_if = None
    save_data_enabled = True

    status_msg = ImageStatus()

    data_msg_type = Image
    status_msg_type = ImageStatus

    data_pub_name = 'targets_image_pub'
    status_pub_name = 'targets_image_status_pub'

    needs_data = False
    last_pub_time = None
    time_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    PUBS_DICT = dict()

    def __init__(self, namespace = None,
                data_product = None,
                save_data_if = None,
                save_data_enabled = True,
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
            log_name = nepi_utils.get_clean_name(log_name)
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################
        # Initialize Class Variables
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product is not None:
                self.data_product = data_product

        if namespace is None:
            namespace = self.node_namespace
        if os.path.basename(namespace) != self.data_product:
            namespace = nepi_sdk.create_namespace(namespace, self.data_product)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        # Initialize status message
        self.status_msg = self.status_msg_type()
        self.status_msg.node_name = self.node_name
        self.status_msg.image_topic = self.namespace

        ##############################
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            self.status_pub_name: {
                'msg': self.status_msg_type,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': False
            },
            self.data_pub_name: {
                'msg': self.data_msg_type,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = None

        # Update or Create Node Class ####################
        if node_if is not None:
            self.node_if = node_if
            self.node_if.register_pubs(self.PUBS_DICT)
        else:
            self.config_if = self.namespace
            self.node_if_shared = False
            self.node_if = NodeClassIF(
                            configs_dict = self.CONFIGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = None,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
            )
            self.node_if.wait_for_ready()

        ####################
        # Save Data Setup
        self.save_data_enabled = save_data_enabled
        if self.save_data_enabled == True:
            self.msg_if.pub_info("Got Save Data IF is None: " + str(save_data_if is None), log_name_list = self.log_name_list)
            if save_data_if is not None and save_data_if != 'None':
                self.save_data_if = save_data_if
                data_products = self.save_data_if.get_data_products()
                if self.data_product not in data_products:
                    self.save_data_if.register_data_product(self.data_product)
            elif save_data_if != 'None':
                # Setup Save Data IF Class
                self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
                factory_data_rates = dict()
                factory_data_rates[self.data_product] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz

                factory_filename_dict = {
                    'prefix': "",
                    'add_timestamp': True,
                    'add_ms': True,
                    'add_us': False,
                    'suffix': "",
                    'add_node_name': True
                }

                self.save_data_if = SaveDataIF(namespace = self.node_namespace,
                                        data_products = [self.data_product],
                                        factory_rate_dict = factory_data_rates,
                                        factory_filename_dict = factory_filename_dict,
                                        log_name_list = self.log_name_list,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if)
                nepi_sdk.sleep(1)

            if self.save_data_if is not None:
                self.status_msg.save_data_topic = self.save_data_if.get_namespace()
                self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic), log_name_list = self.log_name_list)

        ##############################
        # Update vals and publish first status
        self.init(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info(str(self.class_name) + " Initialization Complete", log_name_list = self.log_name_list)
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

    def wait_for_ready(self, timeout = float('inf')):
        """Block until the interface is ready or a timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for ready", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace for this data product interface.

        Returns:
            str: The fully-qualified ROS namespace.
        """
        return self.namespace

    def get_data_product(self):
        """Return the data product name for this interface.

        Returns:
            str: The data product identifier string (e.g. 'targets_image').
        """
        return self.data_product

    def get_status_dict(self):
        """Return the current status message converted to a plain dictionary.

        Returns:
            dict: Status fields as a dictionary, or None if no status message exists.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def needs_data_check(self):
        """Return whether downstream consumers currently need image data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        return copy.deepcopy(self.needs_data)

    def publish_data(self, data_msg, timestamp = None):
        """Publish a targets image message and save it if saving is enabled.

        Publishes the provided ROS Image message on the data product topic,
        updates the publish-rate statistics, and — if a SaveDataIF is registered
        and the data product is due to be saved or snapshotted — converts the
        image to a cv2 image and saves it.

        Args:
            data_msg (sensor_msgs/Image): The image message to publish.
            timestamp (float, optional): Acquisition timestamp in seconds. Defaults
                to the current time when None.

        Returns:
            bool: True if the message was published, False otherwise.
        """
        if self.node_if is None or data_msg is None:
            return False
        self.node_if.publish_pub(self.data_pub_name, data_msg)
        self._updatePubStats()
        if self.save_data_if is not None:
            should_save = self.save_data_if.data_product_should_save(self.data_product) == True
            snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(self.data_product) == True
            if should_save or snapshot_enabled:
                if timestamp is None:
                    timestamp = nepi_utils.get_time()
                try:
                    cv2_img = nepi_img.rosimg_to_cv2img(data_msg)
                    self.save_data_if.save(self.data_product, cv2_img, timestamp = timestamp)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to save " + self.data_product + " data: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
        return True

    def publish_cv2_image(self, cv2_img, encoding = 'bgr8', timestamp = None):
        """Convert a cv2 image to a ROS Image message and publish it.

        Args:
            cv2_img (numpy.ndarray): The OpenCV image to publish.
            encoding (str, optional): Image encoding to use for conversion.
                Defaults to 'bgr8'.
            timestamp (float, optional): Acquisition timestamp in seconds. Defaults
                to the current time when None.

        Returns:
            bool: True if the image was converted and published, False otherwise.
        """
        if cv2_img is None:
            return False
        try:
            data_msg = nepi_img.cv2img_to_rosimg(cv2_img, encoding = encoding)
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert cv2 image to ros image: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
            return False
        return self.publish_data(data_msg, timestamp = timestamp)

    def publish_status(self, status_msg = None):
        """Publish the image data product status message.

        Args:
            status_msg (optional): A pre-built status message to publish and store.
                When None, the interface's current status message is published.
        """
        if self.node_if is None:
            return
        if status_msg is not None:
            self.status_msg = status_msg
        if self.status_msg is not None:
            self.node_if.publish_pub(self.status_pub_name, self.status_msg)

    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this interface."""
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.PUBS_DICT is not None:
                    for pub_name in self.PUBS_DICT.keys():
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
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the interface to its initialized state."""
        self.init()

    def factory_reset(self):
        """Reset the interface to factory defaults."""
        self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _updatePubStats(self):
        if self.last_pub_time is None:
            pub_time_sec = 1.0
            self.last_pub_time = nepi_utils.get_time()
        else:
            cur_time = nepi_utils.get_time()
            pub_time_sec = cur_time - self.last_pub_time
            self.last_pub_time = cur_time
        self.time_list.pop(0)
        self.time_list.append(pub_time_sec)

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _needsDataCheckCb(self, timer):
        has_subs = self.node_if.pub_has_subscribers(self.data_pub_name)
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
            needs_data = has_subs or needs_save or needs_snapshot
        else:
            needs_data = has_subs
        self.needs_data = needs_data
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)

    def _publishStatusCb(self, timer):
        self.publish_status()
