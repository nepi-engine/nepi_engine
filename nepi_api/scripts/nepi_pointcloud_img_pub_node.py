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
import sys
import copy
import time
import threading


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_img

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from nepi_interfaces.msg import PointcloudStatus


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.data_if import PointcloudImageIF



WATCHDOG_DELAY = 60
WATCHDOG_TIMEOUT = 3


class PointcloudImgPub:

    IMG_DATA_PRODUCT = 'pointcloud_image'

    node_if = None
    save_data_if = None
    image_if = None

    status_msg = None
    render_enable = True
    img_pub_enabled = True

    max_rate = 5.0
    last_img_time = 0

    connected = False
    processing = False
    publishing = False

    # Decoupled render worker state. pointcloudCb only stashes the newest cloud
    # (fast, so the subscriber socket keeps draining); _renderWorker does the
    # expensive rospc->o3d conversion + Open3D render on the most recent cloud.
    latest_msg = None
    render_lock = threading.Lock()
    render_thread = None

    last_status_time = None
    watchdog_timeout = None

    data_product = IMG_DATA_PRODUCT

    width_deg = 100
    height_deg = 70

    min_range_m = 0
    max_range_m = 1

    DEFAULT_NODE_NAME = "pointcloud_img_pub"  # Can be overwritten by launch command

    def __init__(self):
        ####  IF INIT SETUP ####
        nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################
        # Init Class Variables

        backup_data_products = ['pointcloud_image']
        self.data_products = nepi_sdk.get_param(self.node_namespace + "/data_products", backup_data_products)
        #self.data_product = self.data_products[-1]
        self.msg_if.pub_warn("Starting with Data Products: " + str(self.data_products))

        backup_pointcloud_namespace = self.node_namespace.replace("_img_pub", "")
        self.pointcloud_namespace = nepi_sdk.get_param(self.node_namespace + "/pointcloud_namespace", backup_pointcloud_namespace)
        self.msg_if.pub_warn("Starting with Pointcloud Namespace: " + str(self.pointcloud_namespace))


        backup_navpose_namespace = self.node_namespace.replace("_img_pub", "") + '/navpose'
        self.navpose_namespace = nepi_sdk.get_param(self.node_namespace + "/navpose_namespace", backup_navpose_namespace)
        self.msg_if.pub_warn("Starting with NavPose Namespace: " + str(self.navpose_namespace))

        self.pointcloud_image_namespace = nepi_sdk.create_namespace(self.pointcloud_namespace,self.data_product)
        self.msg_if.pub_warn("Starting with Pointcloud Image Namespace: " + str(self.pointcloud_image_namespace))

        self.status_msg = PointcloudStatus()

        ##############################
        # Create NodeClassIF Class

        self.CONFIGS_DICT = None
        self.PARAMS_DICT = None
        self.SRVS_DICT = None
        self.PUBS_DICT = None

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'msg': PointcloudStatus,
                'namespace': self.pointcloud_namespace,
                'topic': 'status',
                'qsize': 10,
                'callback': self.statusCb,
                'callback_args': ()
            },
            'pointcloud': {
                'msg': PointCloud2,
                'namespace': self.pointcloud_namespace,
                'topic': '',
                'qsize': 10,
                'callback': self.pointcloudCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        msg_if = self.msg_if
                        )

        nepi_sdk.wait()

        ###############################
        # Create System IFs

        # Setup Save Data IF
        factory_data_rates = {}
        for d in self.data_products:
            factory_data_rates[d] = [1.0, 0.0, 100]
        save_data_ns = os.path.dirname(self.pointcloud_namespace).split('/idx')[0]
        self.save_data_if = SaveDataIF(data_products = self.data_products, pub_status = False,
                        factory_rate_dict = factory_data_rates, namespace = save_data_ns,
                        msg_if = self.msg_if
                        )

        nepi_sdk.sleep(1)

        
        # Setup Pointcloud Image IF (renders and publishes the pointcloud image)
        self.image_if = PointcloudImageIF(namespace = self.pointcloud_namespace,
                        data_product = self.data_product,
                        data_source_description = 'pointcloud',
                        data_ref_description = 'sensor',
                        perspective = 'pov',
                        init_overlay_text_list = [],
                        save_data_if = self.save_data_if,
                        navpose_namespace = self.navpose_namespace,
                        log_name = self.data_product,
                        log_name_list = [],
                        msg_if = self.msg_if
                        )

        time.sleep(1)

        ##########################
        # Complete Initialization

        # Start Timer Processes
        self.last_status_time = nepi_utils.get_time()
        nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)
        nepi_sdk.on_shutdown(self.shutdownCb)

        # Start the decoupled render worker (see pointcloudCb / _renderWorker)
        self.render_thread = threading.Thread(target = self._renderWorker)
        self.render_thread.daemon = True
        self.render_thread.start()

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until shutdown)
        nepi_sdk.spin()
        #########################################################


    def watchdogCb(self, timer):
        cur_time = nepi_utils.get_time()
        timer = cur_time - self.last_status_time
        if self.watchdog_timeout is None:
            self.watchdog_timeout = WATCHDOG_TIMEOUT
            nepi_sdk.sleep(WATCHDOG_DELAY)
        else:
            if timer > WATCHDOG_TIMEOUT:
                msg = "Lost connection to parent node status msg.  Shutting down"
                self.msg_if.pub_warn(msg)
                nepi_sdk.signal_shutdown(msg)

        nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)





    def statusCb(self, msg):
        self.last_status_time = nepi_utils.get_time()
        self.status_msg = msg
        self.img_pub_enabled = msg.img_pub_enabled
        self.render_enable = msg.render_enable
        self.min_range_m = msg.range_min_max_m.start_range
        self.max_range_m = msg.range_min_max_m.stop_range
        self.width_deg = msg.width_deg
        self.height_deg = msg.height_deg


    def pointcloudCb(self, msg):
        # Keep this callback FAST. Rendering here would stall the subscriber
        # socket during each (~1 s) Open3D render; rospy's publisher fans out to
        # all subscribers from a single dispatch thread, so a slow read here
        # back-pressures delivery of the large PointCloud2 to EVERY subscriber
        # (including plain rostopic consumers), dragging the whole topic down to
        # the render rate. Instead just stash the newest cloud and let the render
        # worker process it; stale clouds are simply overwritten (dropped).
        self.connected = True
        with self.render_lock:
            self.latest_msg = msg

    def _renderWorker(self):
        # Continuously render the most recent pointcloud. Decoupled from the
        # subscriber callback so a slow render never back-pressures the topic.
        while not nepi_sdk.is_shutdown():
            with self.render_lock:
                msg = self.latest_msg
                self.latest_msg = None
            if msg is None:
                time.sleep(0.005)
                continue
            self._renderPointcloudMsg(msg)

    def _renderPointcloudMsg(self, msg):

        if self.image_if is None or self.img_pub_enabled == False or self.render_enable == False or self.processing == True:
            return
        
        # Only render if something downstream needs the image
        needs_img = self.image_if.needs_data_check()
        if needs_img == False:
            return

        # Check if time to publish
        max_rate = copy.deepcopy(self.max_rate)
        if max_rate < 0.01:
            return
        delay_time = float(1) / max_rate
        current_time = nepi_utils.get_time()
        if (current_time - self.last_img_time) < delay_time:
            return
        self.processing = True
        self.last_img_time = current_time

        try:
            timestamp = msg.header.stamp
            frame_id = msg.header.frame_id
            o3d_pc = nepi_pc.rospc_to_o3dpc(msg, remove_nans = True)
            if o3d_pc is not None:
                # render_dict = None on purpose: the render controls (zoom, rotate,
                # tilt, camera pose, range ratios) are owned by PointcloudImageIF,
                # not by this node. Passing None makes the interface resolve them
                # from its own control state.
                self.image_if.publish_pointcloud_img(o3d_pc,
                                    render_dict = None,
                                    width_deg = self.width_deg,
                                    height_deg = self.height_deg,
                                    min_range_m = self.min_range_m,
                                    max_range_m = self.max_range_m,
                                    timestamp = timestamp,
                                    frame_id = frame_id
                                    )
        except Exception as e:
            self.msg_if.pub_warn("Failed to render pointcloud image: " + str(e))
        self.processing = False


    def shutdownCb(self):
        pass


#########################################
# Main
#########################################
if __name__ == '__main__':
    PointcloudImgPub()
