#!/usr/bin/env python
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
import cv2

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image

from nepi_ros_interfaces.msg import StringArray, ImageStatus

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img

from nepi_api.node_if import NodeClassIF
from nepi_api.sys_if_msg import MsgIF
from nepi_api.connect_mgr_if_navpose import ConnectMgrNavPoseIF

ENCODING_OPTIONS = ["mono8",'rgb8','bgr8','32FC1','passthrough']

DEFUALT_IMG_WIDTH = 700
DEFUALT_IMG_HEIGHT = 400

class ImageIF:

    ready = False
    namespace = '~image'

    node_if = None

    status_msg = ImageStatus()

    blank_img = nepi_img.create_cv2_blank_img(DEFUALT_IMG_WIDTH, DEFUALT_IMG_HEIGHT, color = (0, 0, 0) )

    last_pub_time = None

    nav_mgr_if = None
    nav_mgr_ready = False

    has_subscibers = False


    def __init__(self, namespace = None, encoding = 'bgr8', do_wait = True ,init_overlay_list = []):
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################    
        # Initialize Class Variables
        if namespace is not None:
            self.namespace = namespace
        self.namespace = nepi_ros.get_full_namespace(self.namespace)


        self.init_overlay_list = init_overlay_list

        # Initialize Status Msg.  Updated on each publish
        status_msg = ImageStatus()
        status_msg.publishing = False
        status_msg.encoding = encoding
        status_msg.width = 0
        status_msg.height = 0
        status_msg.frame_id = "sensor_frame"
        status_msg.depth_map_topic = nepi_img.get_img_depth_map_topic(namespace)
        status_msg.pointcloud_topic = nepi_img.get_img_pointcloud_topic(namespace)
        status_msg.get_latency_time = 0
        status_msg.pub_latency_time = 0
        status_msg.process_time = 0
        self.status_msg = status_msg

        ##############################
        ## Connect NEPI NavPose Manager
        self.nav_mgr_if = ConnectMgrNavPoseIF()
        self.nav_mgr_ready = self.nav_mgr_if.wait_for_ready()

        ##############################   
        ## Node Setup

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'overlay_img_name': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'overlay_date_time': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'overlay_nav': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'overlay_pose': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'overlay_list': {
                'namespace': self.namespace,
                'factory_val': []
            },
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'image_pub': {
                'msg': Image,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            'status_pub': {
                'msg': ImageStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'overlay_img_name': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_img_name',
                'qsize': 1,
                'callback': self._setOverlayImgNameCb
            },
            'overlay_date_time': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_date_time',
                'qsize': 1,
                'callback': self._setOverlayDateTimeCb
            },
            'overlay_nav': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_nav',
                'qsize': 1,
                'callback': self._setOverlayNavCb
            },
            'overlay_pose': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_pose',
                'qsize': 1,
                'callback': self._setOverlayPoseCb
            },
            'overlay_list': {
                'msg': StringArray,
                'namespace': self.namespace,
                'topic': 'set_overlay_list',
                'qsize': 1,
                'callback': self._setOverlayListCb
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(self,
                        params_dict = self.PARAMS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        self.node_if.wait_for_ready()

        ##############################
        # Start Node Processes
        nepi_ros.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
        nepi_ros.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready  


    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_ros.convert_msg2dict(self.status_msg)
        return status_dict


    def has_subscribers_check(self):
        return self.has_subscribers


    def publish_cv2_img(self,cv2_img, encoding = "bgr8", ros_timestamp = None, frame_id = 'sensor_frame'):
        #self.msg_if.pub_warn("Got Image to Publish")
        success = False
        if self.img_pub is None:
            self.msg_if.pub_info("Can't publish on None publisher")
            return False
        if cv2_img is None:
            self.msg_if.pub_info("Can't publish None image")
            return False

        if ros_timestamp == None:
            ros_timestamp = nepi_ros.ros_time_now()


        current_time = nepi_ros.ros_time_now()
        latency = (current_time.to_sec() - ros_timestamp.to_sec())
        self.status_msg.get_latency_time = latency
        #self.msg_if.pub_info("Get Img Latency: {:.2f}".format(latency))

        # Start Img Pub Process
        start_time = nepi_ros.get_time()   

        # Publish and Save Raw Image Data if Required  
        [height,width] = cv2_img.shape[0:2]
        last_width = self.status_msg.width
        last_height = self.status_msg.height
        self.status_msg.width = width
        self.status_msg.height = height

        #self.msg_if.pub_warn("Got Image size: " + str([height,width]))

        if self.has_subscribers == False:
            if self.status_msg.publishing == True:
                self.msg_if.pub_warn("Image has no subscribers")
            self.status_msg.publishing = False

        else:
            if self.status_msg.publishing == False:
                self.msg_if.pub_warn("Image has subscribers, will publish")
            self.status_msg.publishing = True

            # Apply Overlays
            overlay_list = []
            if self.node_if.get_param('overlay_img_name') == True:
                overlay = nepi_img.getImgShortName(self.img_namespace)
                overlay_list.append(overlay)
            
            if self.node_if.get_param('overlay_date_time') == True:
                date_time = nepi_ros.get_datetime_str_from_stamp(ros_timestamp)
                overlay_list.append(overlay)

            nav_pose_dict = None
            if self.node_if.get_param('overlay_nav') == True or self.node_if.get_param('overlay_pose') == True:
                if self.nav_mgr_ready == True:
                    nav_pose_dict = self.nav_mgr_if.get_navpose_data_dict()
                    if nav_pose_dict is not None:

            if self.node_if.get_param('overlay_nav') == True and nav_pose_dict is not None:
                overlay = 'Lat: ' +  str(round(nav_pose_dict['lat'],6)) + 'Long: ' +  str(round(nav_pose_dict['long'],6)) + 'Head: ' +  str(round(nav_pose_dict['heading_deg'],2))
                overlay_list.append(overlay)

            if self.node_if.get_param('overlay_pose') == True and nav_pose_dict is not None:
                overlay = 'Roll: ' +  str(round(nav_pose_dict['roll_deg'],2)) + 'Pitch: ' +  str(round(nav_pose_dict['pitch_deg'],2)) + 'Yaw: ' +  str(round(nav_pose_dict['yaw_deg'],2))
                overlay_list.append(overlay)

            overlay_list = overlay_list + self.init_overlay_list + self.node_if.get_param('add_overlay_list')

            cv2_img = nepi_img.overlay_text_list(cv2_img, text_list = overlay_list, x_px = 10 , y_px = 10, color_rgb = (0, 255, 0), apply_shadow = True)


            #Convert to ros Image message
            ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding=encoding)
            ros_img.header.stamp = ros_timestamp
            ros_img.header.frame_id = frame_id

            process_time = round( (nepi_ros.get_time() - start_time) , 3)
            self.status_msg.process_time = process_time
            latency = (current_time.to_sec() - ros_timestamp.to_sec())
            self.status_msg.pub_latency_time = latency
            
            if not nepi_ros.is_shutdown():
                self.node_if.publish_pub('data_pub', ros_img)
            if self.last_pub_time is None:
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time
                self.status_msg.last_pub_sec = pub_time_sec
                self.status_msg.fps = float(1) / pub_time_sec

        # Update blank image if needed
        if last_width != self.status_msg.width or last_height != self.status_msg.height:
            self.blank_img = nepi_img.create_cv2_blank_img(width, height, color = (0, 0, 0) )
        return True

    def publish_cv2_msg_img(self,text):
        cv2_img = nepi_img.overlay_text_autoscale(self.blank_img, text)
        self.publish_cv2_img(cv2_img)


    def unregister(self):
        self.ready = False
        self.node_if.unregister_class()
        nepi_ros.sleep(1)
        self.namespace = '~'
        self.status_msg = None

    ###############################
    # Class Private Methods
    ###############################

    def _subscribersCheckCb(self,timer):
        self.has_subscribers = self.node_if.pub_has_subscribers('data_pub')
        #self.msg_if.pub_warn("Sub check gotsubscribers: " + str(self.has_subscribers))
        if self.has_subscribers == False:
            self.status_msg.publishing = False
        nepi_ros.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)

    def _publishStatusCb(self,timer):
        self.status_msg.overlay_img_name = self.node_if.get_param('overlay_img_name')
        self.status_msg.overlay_date_time =  self.node_if.get_param('overlay_date_time')
        self.status_msg.overlay_nav = self.node_if.get_param('overlay_nav')
        self.status_msg.overlay_pose = self.node_if.get_param('overlay_pose')  
        self.status_msg.base_overlay_list = self.init_overlay_list
        self.status_msg.add_overlay_list = add_overlays = self.node_if.get_param('add_overlay_list')

        self.node_if.publish_pub('status_pub',self.status_msg)
            self.img_status_msg.publish(self.status_msg)

    def _setOverlayImgNameCb(self,msg):
        self.node_if.set_param('overlay_img_name', msg.data)
        self.publishStatus()

    def _setOverlayDateTimeCb(self,msg):
        self.node_if.set_param('overlay_date_time', msg.data)
        self.publishStatus()

    def _setOverlayNavCb(self,msg):
        self.node_if.set_param('overlay_nav', msg.data)
        self.publishStatus()

    def _setOverlayPoseCb(self,msg):
        self.node_if.set_param('overlay_pose', msg.data)
        self.publishStatus()

    def _setAddOverlayListCb(self,msg):
        self.node_if.set_param('add_overlay_list', msg.data)
        self.publishStatus()