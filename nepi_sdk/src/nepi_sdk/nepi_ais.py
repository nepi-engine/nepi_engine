#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import rospy
import os
import copy
import time
import numpy as np
import math
import threading
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cv2

from std_msgs.msg import UInt8, Float32, Bool, Empty, String
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes 
from nepi_ros_interfaces.msg import AiDetectorStatus
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, AiDetectorInfoQuery, AiDetectorInfoQueryRequest

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_img
from nepi_sdk import nepi_save

from nepi_sdk.save_data_if import SaveDataIF


#######################
# AI Model Manager IF Functions

AI_MGR_NODE_NAME = 'ai_model_mgr'
MODEL_TYPE_LIST = ['detection']


def init_ai_mgr_connection(self):
    self.ai_mgr_namespace = I_MGR_NODE_NAME
    nepi_msg.publishMsgInfo(self,"Initialization AI Manager Processes")
    base_namespace = nepi_ros.get_base_namespace()
    mgr_namespace = os.path.join(base_namespace,self.ai_mgr_namespace)
    status_topic = os.path.join(mgr_namespace,"status")
    nepi_msg.publishMsgInfo(self,"Waiting for topic: " + status_topic)
    nepi_ros.wait_for_topic(status_topic)
    nepi_msg.publishMsgInfo(self,"Starting ai model mgr status scubscriber callback")
    self.ai_mgr_status_msg = None
    rospy.Subscriber(status_topic, AiModelMgrStatus, self.ai_mgr_status_callback, queue_size=None)
    nepi_msg.publishMsgInfo(self,"Waiting for status message")
    while self.ai_mgr_status_msg is None and not rospy.is_shutdown():
        nepi_ros.sleep(1)
    nepi_msg.publishMsgInfo(self,"AI Model Manager Initialization Complate")
    return True

def ai_mgr_status_callback(self,status_msg):
    self.ai_mgr_status_msg = status_msg
    self.ai_frameworks = status_msg.ai_frameworks
    self.ai_models = status_msg.ai_models
    self.ai_models_frameworks = status_msg.ai_models_frameworks
    self.ai_models_types = status_msg.ai_models_types
    self.active_ai_framework = status_msg.active_ai_framework
    self.active_ai_models = status_msg.active_ai_models
    self.active_ai_models_types = status_msg.active_ai_models_types
    self.active_ai_models_nodes = status_msg.active_ai_models_nodes
    self.active_ai_models_namespaces = status_msg.active_ai_models_namespaces
    self.all_namespace = status_msg.all_namespace


def get_active_model_dict_by_name(self):
    model_name_dict = dict()
    for i, model_name in enumerate(self.active_ai_models):
        model_type = self.active_ai_models_types
        type_ind = MODEL_TYPE_LIST.index(model_type)
        if type_ind != -1:
            model_type_dict[model_name] = dict()
            model_type_dict[model_name]['type']=model_type
            model_type_dict[model_name]['node']=self.active_ai_models_nodes[i]
            model_type_dict[model_name]['namespace']=self.active_ai_models_namespaces[i]
    return model_types_dict


#######################
# Detection IF Functions
def get_detector_topics(self):
    mgs_type = BoundingBoxes
    topics = nepi_ros.find_topics_by_msg(msg_type)
    topics_list = []
    for i, topic in enumerate(topics):
        topics_list.append(topic.replace('/bounding_boxes',''))
    return topics_list

 def get_detector_info(self,det_topic)
    det_info = None
    det_namespaces = get_detector_namespaces(self)
    det_namespace = ""
    for namespace in det_namespaces:
        if det_topic.find(namespace) != -1:
            det_namespace = namespace
            break
    if det_namespace != "":
        # Call Detector Info Service Query
        get_info_service = os.path.join(det_namespace,'detector_info_query')
        nepi_msg.publishMsgInfo(self,"Waiting for detector info query service " + get_info_service)
        rospy.wait_for_service(get_info_service)
        nepi_msg.publishMsgInfo(self,"Calling detector info query service " + get_info_service)
        try:
            info_query_service = rospy.ServiceProxy(get_info_service, AiDetectorInfoQuery)
            time.sleep(1)
            success = True
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to obtain detector info query service " + str(e))
            success = False
        if success = True
            try:
                nepi_msg.publishMsgInfo(self,"Getting detector info query service " + get_info_service)
                req = AiDetectorInfoQueryRequest()
                det_info = info_query_service(req )
                nepi_msg.publishMsgInfo(self,"Got detector info response" + det_info)
                success = True
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to obtain detector info service response " + str(e))
                success = False
    return sucess, det_info  

def get_detector_namespaces(self):
    mgs_type = AiDetectorStatus
    topics_list = nepi_ros.find_topics_by_msg(msg_type)
    return topics_list


def get_detector_status_topics(self):
    mgs_type = AiDetectorStatus
    topics_list = nepi_ros.find_topics_by_msg(msg_type)
    return topics_list


def init_detectors_dict(self):
    self.dets_dict = dict()

def register_detector(self, det_namespace, timeout = 5):
    nepi_msg.publishMsgInfo(self,"Initialization Detection Processes")
    if det_namespace[-1] == '/'
        det_namespace = det_namespace[0:-1]
    
    try:
        keys = self.dets_dict.keys()
    except:
        nepi_msg.publishMsgInfo(self,"Class dets_dict not found, will create")
        init_detectors_dict(self)

    if det_namespace in self.dets_dict.keys():
        nepi_msg.publishMsgInfo(self,"Class dets_dict allready has det_namespace registered, will create: " + det_namespace)
        return False, det_namespace
    else:
        self.dets_dict[det_namespace] = dict()

    ## Setup Settings Callback
    status_topic = os.path.join(det_namespace,"status")
    nepi_msg.publishMsgInfo(self,"Waiting for topic: " + status_topic)
    nepi_ros.wait_for_topic(status_topic)
    nepi_msg.publishMsgInfo(self,"Starting detector status scubscriber callback")
    self.dets_dict[det_namespace]['status_msg'] = None
    det_status_sub = rospy.Subscriber(status_topic, AiDetectorStatus, detector_status_callback, queue_size=None, callback_args=(det_namespace))
    self.dets_dict[det_namespace]['status_sub'] = det_status_sub
    nepi_msg.publishMsgInfo(self,"Waiting for status message")
    success = False
    timer = 0
    time_start = time.ros_time_now()
    connected = None
    while connected is None and timer > timeout and not rospy.is_shutdown():
        nepi_ros.sleep(.2)
        connected = self.dets_dict[det_namespace]['connected'] 
        timer = time.ros_time_now() - time_start
    if connected is not None:
        success = True
    else:
        nepi_msg.publishMsgWarn(self,"Status msg topic subscribe timedout " + str(status_topic))

    if success = True:
        # Call Detector Info Service Query
        get_info_service = os.path.join(det_namespace,'detector_info_query')
        nepi_msg.publishMsgInfo(self,"Waiting for detector info query service " + get_info_service)
        rospy.wait_for_service(get_info_service)
        nepi_msg.publishMsgInfo(self,"Calling detector info query service " + get_info_service)
        try:
            info_query_service = rospy.ServiceProxy(get_info_service, AiDetectorInfoQuery)
            time.sleep(1)
            success = True
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to obtain detector info query service " + str(e))
            success = False
        if success = True
            try:
                nepi_msg.publishMsgInfo(self,"Getting detector info query service " + get_info_service)
                req = AiDetectorInfoQueryRequest()
                response = info_query_service(req )
                self.dets_dict[det_namespace]['info'] = response
                nepi_msg.publishMsgInfo(self,"Got detector info response" + response)
                success = True
            except Exception as e:
                self.det_info_response = None
                nepi_msg.publishMsgWarn(self,"Failed to obtain detector info service response " + str(e))
                success = False
    if success == False:
        nepi_msg.publishMsgWarn(self,"Status msg topic subscribe timedout " + str(det_namespace))
        nepi_msg.publishMsgWarn(self,"Unregistering subscribe topic " + str(det_namespace))
        unregister_detector(self,det_namespace)
    else:
        self.det_namespace = det_namespace
        nepi_msg.publishMsgInfo(self,"Detection Registration Complete")
    return success, det_namespace


def unregister_detector(self,det_namespace)
    try:
        if det_namespace in self.dets_dict:
        status_sub = self.dets_dict[det_namespace]['status_sub']
        status_sub.unregister()
        time.sleep(1)
        success = True
    except Exception as e:
        nepi_msg.publishMsgWarn(self,"Failed to unregister detector: " + str(e))
        success = False
    try:
        del self.dets_dict[det_namespace]
    except Exception as e:
        nepi_msg.publishMsgWarn(self,"Failed to delete dets_dict entry: " + str(e))
    return success


def detector_status_callback(self, status_msg, args):
    det_namespace = args
    if det_namespace in self.dets_dict:
        self.dets_dict[det_namespace]['status_msg'] = status_msg
        self.dets_dict[det_namespace]['connected'] = True




