#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_ais

import rospy
import os
import time
import numpy as np

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes
from nepi_ros_interfaces.msg import AiDetectorInfo, AiDetectorStatus
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryRequest





EXAMPLE_BOXES_INFO_DICT_ENTRY = {
    'model_name': 'test_model',
    'image_header': Header(),
    'image_topic': 'my/test_topic',
    'src_height': 600,
    'src_width': 1000,
    'prc_height': 300,
    'prc_width': 500
}


EXAMPLE_BOX_DICT_ENTRY = {
    'name': 'chair', # Class String Name
    'id': 1, # Class Index from Classes List
    'uid': '', # Reserved for unique tracking by downstream applications
    'prob': .3, # Probability of detection
    'xmin': 10,
    'ymin': 10,
    'xmax': 100,
    'ymax': 100,
    'area_ratio': 0.054,
    'area_pixels': 8100
}

class AiDetectionIF:

    # AI Detector Vars
    connected = False
    namespace = "None"
    base_namespace = "None"

    active = False

    pub_sub_dict = dict()
    pub_sub_dict['status_sub'] = None
    pub_sub_dict['found_obj_sub'] = None
    pub_sub_dict['bounding_boxes_sub'] = None

    status_msg = None
    status_dict = dict()
    

    info_msg = None
    info_dict = dict()

    classes = []
    classes_colors = []

    proc_img_width = 0
    proc_img_height = 0
 
    found_obj_msg = None
    found_obj_count = 0

    bounding_boxes_msg = None
    bboxes_info = dict()
    bboxes_list = []

    first_det_time = None
    last_det_time = nepi_ros.get_time()

    src_img_topic = None

    #######################
    ### IF Initialization
    log_name = "AiDetectionIF"
    def __init__(self):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting IF Initialization Processes")
        ##############################  
        
        #################################
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": IF Initialization Complete")
        

    #######################
    # Class Public Methods

    def get_available_detector_namespaces(self):
        namespaces = []
        if len(self.status_dict.keys()) > 0:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Detector Status Keys: " + self.status_dict.keys())
            namespaces = self.status_dict['image_detector_namespaces']
        return namespaces


    def register_detector(self, namespace, base_namespace, timeout = 5):
        #######################
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Initialization Detection Processes")
        success = True
        namespace = nepi_ros.clear_end_slash(namespace)
        base_namespace = nepi_ros.clear_end_slash(base_namespace)

        #######################
        # Check if currently connected and unregister if so
        if self.namespace != "None":
            if namespace == self.namespace:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Specified Detector already connected, Exiting registration process")
                return True
            else:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Different Detector already connected, unregistering current detector first")
                unreged = self.unregister_detector()
                if unreged == False:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to unregister current detector: " + str(self.namespace))
        #######################
        # Start Initialize 
        nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Initializing detector with namespace: " + str(namespace) + " base_namespace: " + str(base_namespace))
        self.namespace = namespace
        self.base_namespace = base_namespace

        #######################
        # Wait for Service Call
        get_info_service = os.path.join(base_namespace,'detector_info_query')
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for detector info query service " + get_info_service)
        rospy.wait_for_service(get_info_service, timeout )
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Creating detector info query service call" + get_info_service)
        info_query_service = None
        try:
            info_query_service = rospy.ServiceProxy(get_info_service, AiDetectorInfoQuery)
            time.sleep(1)
        except Exception as e:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to obtain detector info query service " + str(e))
        if info_query_service is None:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Calling detector info query service " + get_info_service)
            req = AiDetectorInfoQueryRequest()
            try:
                response = info_query_service(req)
                detector_info = response.detector_info
                self.info_msg = detector_info
                self.info_dict = nepi_ros.convert_msg2dict(detector_info)
                self.proc_img_width = detector_info.proc_img_width
                self.proc_img_height = detector_info.proc_img_height
                self.classes = detector_info.classes
                self.classes_colors = nepi_ais.get_classes_colors_list(detector_info.classes)
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Got detector info response" + str(detector_info))
            except Exception as e:
                self.info_msg = None
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to obtain detector info service response " + str(e))
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Unregistering subscribe topic " + str(base_namespace))
                self.unregister_detector()
                time.sleep(1)
                return False

        #######################
        #Setup Detector Publishers
        DET_PUB_NS = namespace

        rospy.Publisher(DET_PUB_NS + '/add_img_topic', String, self.addImageTopicCb, queue_size=10)
        rospy.Publisher(DET_PUB_NS + '/add_img_topics', StringArray, self.addImageTopicsCb, queue_size=10)
        rospy.Publisher(DET_PUB_NS + '/remove_img_topic', String, self.removeImageTopicCb, queue_size=10)
        rospy.Publisher(DET_PUB_NS + '/remove_img_topics', StringArray, self.removeImageTopicsCb, queue_size=10)
        rospy.Publisher(DET_PUB_NS + '/set_img_tiling', Bool, self.setTileImgCb, queue_size=10)
        rospy.Publisher(DET_PUB_NS + '/set_overlay_labels', Bool, self.setOverlayLabelsCb, queue_size=10)
        rospy.Publisher(DET_PUB_NS + '/set_overlay_clf_name', Bool, self.setOverlayClfNameCb, queue_size=10)
        rospy.Publisher(DET_PUB_NS + '/set_overlay_img_name', Bool, self.setOverlayImgNameCb, queue_size=10)          
        rospy.Publisher(DET_PUB_NS + '/set_threshold', Float32, self.setThresholdCb, queue_size=10)
        rospy.Publisher(DET_PUB_NS + '/set_max_rate', Int32, self.setMaxRateCb, queue_size=10)

        rospy.Publisher(DET_PUB_NS + '/set_sleep_enable', Bool, self.setSleepEnableCb, queue_size=10) 
        rospy.Publisher(DET_PUB_NS + '/set_sleep_suspend_sec', Int32, self.setSleepSuspendTimeCb, queue_size=10)
        rospy.Publisher(DET_PUB_NS + '/set_sleep_run_sec', Int32, self.setSleepSuspendTimeCb, queue_size=10)

        rospy.Publisher(DET_PUB_NS + '/reset_factory', Empty, self.resetFactoryCb, queue_size=10) # start local callback
        rospy.Publisher(DET_PUB_NS + '/save_config', Empty, self.saveConfigCb, queue_size=10) # start local callback
        rospy.Publisher(DET_PUB_NS + '/reset_config', Empty, self.resetConfigCb, queue_size=10) # start local callback
        rospy.Publisher(DET_PUB_NS + '/enable', Bool, self.enableCb, queue_size=10) # start local callback


        #######################
        #Setup Detector Subscribers
        DET_SUB_NS = namespace
  
        found_obj_sub = rospy.Subscriber(DET_SUB_NS + "/found_object", ObjectCount, self.foundObjectCb, queue_size = 1)
        self.pub_sub_dict['found_obj_sub'] = found_obj_sub

        bounding_boxes_sub = rospy.Subscriber(DET_SUB_NS + "/bounding_boxes", BoundingBoxes, self.objectDetectedCb, queue_size = 1)
        self.pub_sub_dict['bounding_boxes_sub'] = bounding_boxes_sub

        status_sub = rospy.Subscriber(DET_SUB_NS + "/status", AiDetectorStatus, self.detectorStatusCb, queue_size=10)
        self.pub_sub_dict['status_sub'] = status_sub

        #######################
        # Wait for Status Message
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for status message")
        timer = 0
        time_start = nepi_ros.ros_time_now()
        status_msg = None
        while status_msg is None and timer < timeout and not rospy.is_shutdown():
            nepi_ros.sleep(.2)
            status_msg = self.status_msg
            timer = nepi_ros.ros_time_now() - time_start
        if self.status_msg is None:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Status msg topic subscribe timed out " + str(status_topic))
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Unregistering detector " + str(namespace))
            success = False
        else:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Got detector status message " + str(self.status_msg))

        ####################
        # Wrap Up
        if success == False:
            self.unregister_detector()
        else:
            self.connected = True
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Detector Registration Complete")
        return success


    def unregister_detector(self):
        success = True
        # Unsubsribe from pubs and subs
        for pub_sub_name in self.pub_sub_dict.keys():
            pub_sub = self.pub_sub_dict[pub_sub_name]
            if pub_sub is not None:
                try:
                    pub_sub.unregister()
                except Exception as e:
                    success = False
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to unregister: " + pub_sub_name + " " + str(e))
        time.sleep(1)
        for pub_sub_name in self.pub_sub_dict.keys():
            self.pub_sub_dict[pub_sub_name] = None
            
        # Reset some variables
        self.namespace = "None"
        self.connected = False
        self.active = False

        self.status_msg = None
        self.status_dict = dict()
        
        self.found_obj_msg = None
        self.bounding_boxes_msg = None

        return success


    #######################
    # Class Private Methods

    def detectorStatusCb(self, status_msg):
        self.status_msg = status_msg
        self.status_dict = nepi_ros.convert_msg2dict(status_msg)

        # Create some useful class objects from status message
        self.namespaces = self.status_msg.image_detector_namespaces
        self.src_img_topics = self.status_msg.image_source_topics
        topics_active = self.status_msg.image_detector_states
        if self.src_img_topic is None:
            if self.namespace in self.namespaces:
                ind = self.namespaces.index(self.namespace)
                self.src_img_topic = self.src_img_topics[ind]
                self.active = topics_active
            else:
                self.src_img_topic = None
                self.active = False
        

    ### Monitor Output of AI detector to clear detection status
    def foundObjectCb(self,found_obj_msg):
        self.found_obj_msg = found_obj_msg
        self.found_obj_count = found_obj_msg.count

    def objectDetectedCb(self,bounding_boxes_msg):
        self.object_detect_msg = bounding_boxes_msg
        self.bboxes_info = nepi_ais.get_boxes_info_from_msg(bounding_boxes_msg)
        self.bboxes_list = nepi_ais.get_boxes_list_from_msg(bounding_boxes_msg)

        time = nepi_ros.get_time()
        if self.first_det_time is None:
            self.first_det_time = time
        self.last_det_time = time





