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
import threading
import subprocess
import rospy
import numpy as np
import cv2
import open3d as o3d

from std_msgs.msg import UInt8, Float32, Bool, Empty, String
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from nepi_sdk.settings_if import SettingsIF
from nepi_sdk.save_data_if import SaveDataIF
from nepi_sdk.save_cfg_if import SaveCfgIF
from nepi_ros_interfaces.msg import IDXStatus, RangeWindow, SaveData, SaveDataRate, SaveDataStatus
from nepi_ros_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryResponse, NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryResponse
from nepi_ros_interfaces.msg import NavPose, Frame3DTransform
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest
from geometry_msgs.msg import Vector3

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_save
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc


#***************************
# IDX utility functions

#Factory Control Values 
DEFAULT_CONTROLS_DICT = dict( controls_enable = True,
    auto_adjust = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.5,
    resolution_mode = 3, # LOW, MED, HIGH, MAX
    framerate_mode = 1, # LOW, MED, HIGH, MAX
    start_range_ratio = 0.0,
    stop_range_ratio = 1.0,
    min_range_m = 0.0,
    max_range_m = 1.0,
    zoom_ratio = 0.5, 
    rotate_ratio = 0.5,
    frame_3d = 'nepi_center_frame'
    )


class ROSIDXSensorIF:
    # Default Global Values
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]
    RESOLUTION_MODE_MAX = 3 # LOW, MED, HIGH, MAX
    FRAMERATE_MODE_MAX = 3 # LOW, MED, HIGH, MAX
    UPDATE_NAVPOSE_RATE_HZ = 10

    ZERO_TRANSFORM = [0,0,0,0,0,0,0]
    

    NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
    NAVPOSE_SERVICE_NAME = NEPI_BASE_NAMESPACE + "nav_pose_query"


    # Define class variables
    factory_device_name = None
    init_device_name = None
    factory_controls_dict = None
    init_controls_enable = None 
    init_auto_adjust = None
    init_brightness_ratio = None
    init_contrast_ratio = None
    init_threshold_ratio = None
    init_resolution_mode = None
    init_framerate_mode = None
    init_min_range = None
    init_max_range = None
    init_frame_3d = None


    init_zoom_ratio = 0.5
    init_rotate_ratio = 0.5
    init_tilt_ratio = 0.5

    zoom_ratio = init_zoom_ratio
    rotate_ratio = init_rotate_ratio
    tilt_ratio = init_rotate_ratio
    render_controls = [zoom_ratio,rotate_ratio,tilt_ratio]


    data_products = []

    settings_if = None
    save_data_if = None
    save_cfg_if = None

    update_navpose_interval_sec = float(1)/UPDATE_NAVPOSE_RATE_HZ
    last_gps_timestamp = None
    last_odom_timestamp = None
    last_heading_timestamp = None



    cl_img_has_subs = False
    bw_img_has_subs = False
    dm_img_has_subs = False
    di_img_has_subs = False
    pc_img_has_subs = False
    pc_has_subs = False
    

    def __init__(self, device_info, capSettings=None, 
                 factorySettings=None, settingUpdateFunction=None, getSettingsFunction=None,
                 factoryControls = None, setControlsEnable=None, setAutoAdjust=None,
                 setContrast=None, setBrightness=None, setThresholding=None,
                 setResolutionMode=None, setFramerateMode=None, 
                 setRange=None, getFramerate=None,
                 getColor2DImg=None, stopColor2DImgAcquisition=None, 
                 getBW2DImg=None, stopBW2DImgAcquisition=None,
                 getDepthMap=None, stopDepthMapAcquisition=None, 
                 getDepthImg=None, stopDepthImgAcquisition=None, 
                 getPointcloud=None, stopPointcloudAcquisition=None, 
                 getPointcloudImg=None, stopPointcloudImgAcquisition=None, 
                 getGPSMsg=None,getOdomMsg=None,getHeadingMsg=None):

        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting IF Initialization Processes")
        ##############################    
        
        self.sensor_name = device_info["sensor_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.factory_device_name = device_info["sensor_name"] + "_" + device_info["identifier"]

        # Create the CV bridge. Do this early so it can be used in the threading run() methods below 
        # TODO: Need one per image output type for thread safety?

        self.capabilities_report = IDXCapabilitiesQueryResponse()
        self.navpose_capabilities_report = NavPoseCapabilitiesQueryResponse()

        # Create and update factory controls dictionary
        self.factory_controls_dict = DEFAULT_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]
        
        #nepi_msg.publishMsgWarn(self,"Starting with IDX Controls: " + str(self.factory_controls_dict))

        self.initializeParamServer(do_updates = False)

        # Set up standard IDX parameters with ROS param and subscriptions
        # Defer actually setting these on the camera via the parent callbacks... the parent may need to do some 
        # additional setup/calculation first. Parent can then get these all applied by calling updateFromParamServer()

        self.setControlsEnable = setControlsEnable
        if setControlsEnable is not None:
            rospy.Subscriber('~idx/set_controls_enable', Bool, self.setControlsEnableCb, queue_size=1) # start local callback
     
        self.setAutoAdjust = setAutoAdjust
        if setAutoAdjust is not None:
            rospy.Subscriber('~idx/set_auto_adjust', Bool, self.setAutoAdjustCb, queue_size=1) # start local callback
            self.capabilities_report.auto_adjustment = True
        else:
            self.capabilities_report.auto_adjustment = False
    
        self.setBrightness = setBrightness
        if setBrightness is not None:
            rospy.Subscriber('~idx/set_brightness', Float32, self.setBrightnessCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_brightness = True
        else:
            self.capabilities_report.adjustable_brightness = False

        self.setContrast = setContrast
        if setContrast is not None:
            rospy.Subscriber('~idx/set_contrast', Float32, self.setContrastCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_contrast = True
        else:
            self.capabilities_report.adjustable_contrast = False

        self.setThresholding = setThresholding       
        if setThresholding is not None:
            rospy.Subscriber('~idx/set_thresholding', Float32, self.setThresholdingCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_thresholding = True
        else:
            self.capabilities_report.adjustable_thresholding = False

        self.setResolutionMode = setResolutionMode
        if setResolutionMode is not None:
            rospy.Subscriber('~idx/set_resolution_mode', UInt8, self.setResolutionModeCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_resolution = True
        else:
            self.capabilities_report.adjustable_resolution = False
               
        self.setFramerateMode = setFramerateMode
        self.getFramerate = getFramerate
        if setFramerateMode is not None:
            rospy.Subscriber('~idx/set_framerate_mode', UInt8, self.setFramerateModeCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_framerate = True
        else:
            self.capabilities_report.adjustable_framerate = False

        self.setRange = setRange
        if setRange is not None:        
            rospy.Subscriber('~idx/set_range_window', RangeWindow, self.setRangeCb, queue_size=1)
            self.capabilities_report.adjustable_range = True
        else:
            self.capabilities_report.adjustable_range = False

        rospy.Subscriber('~idx/clear_frame_3d_transform', Bool, self.clearFrame3dTransformCb, queue_size=1)
        rospy.Subscriber('~idx/set_frame_3d_transform', Frame3DTransform, self.setFrame3dTransformCb, queue_size=1)
        rospy.Subscriber('~idx/set_frame_3d', String, self.setFrame3dCb, queue_size=1)

        rospy.Subscriber('~idx/reset_controls', Empty, self.resetControlsCb, queue_size=1) # start local callback
        rospy.Subscriber('~idx/reset_factory', Empty, self.resetFactoryCb, queue_size=1) # start local callback

        rospy.Subscriber('~idx/update_device_name', String, self.updateDeviceNameCb, queue_size=1) # start local callbac
        rospy.Subscriber('~idx/reset_device_name', Empty, self.resetDeviceNameCb, queue_size=1) # start local callback

        # Start the data producers  
        self.getColor2DImg = getColor2DImg
        if (self.getColor2DImg is not None):
            self.color_img_pub = rospy.Publisher('~idx/color_2d_image', Image, queue_size=1, tcp_nodelay=True)
            self.data_products.append('color_2d_image')
            self.color_img_thread = threading.Thread(target=self.runColorImgThread)
            self.color_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.color_img_subs_thread = threading.Thread(target=self.runColorImgSubsThread)
            self.color_img_subs_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopColor2DImgAcquisition = stopColor2DImgAcquisition
            self.capabilities_report.has_color_2d_image = True
        else:
            self.capabilities_report.has_color_2d_image = False
        

        self.getBW2DImg = getBW2DImg
        if (self.getBW2DImg is not None):
            self.bw_img_pub = rospy.Publisher('~idx/bw_2d_image', Image, queue_size=1, tcp_nodelay=True)
            self.data_products.append('bw_2d_image')
            self.bw_img_thread = threading.Thread(target=self.runBWImgThread)
            self.bw_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.bw_img_subs_thread = threading.Thread(target=self.runBwImgSubsThread)
            self.bw_img_subs_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopBW2DImgAcquisition = stopBW2DImgAcquisition
            self.capabilities_report.has_bw_2d_image = True
        else:
            self.capabilities_report.has_bw_2d_image = False
        

        self.getDepthMap = getDepthMap
        if (self.getDepthMap is not None):
            self.depth_map_pub = rospy.Publisher('~idx/depth_map', Image, queue_size=1, tcp_nodelay=True)
            self.data_products.append('depth_map')
            self.depth_map_thread = threading.Thread(target=self.runDepthMapThread)
            self.depth_map_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.depth_map_subs_thread = threading.Thread(target=self.runDepthMapSubsThread)
            self.depth_map_subs_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopDepthMapAcquisition = stopDepthMapAcquisition
            self.capabilities_report.has_depth_map = True
        else:
            self.capabilities_report.has_depth_map = False
            
        self.getDepthImg = getDepthImg
        if (self.getDepthImg is not None):
            self.depth_img_pub = rospy.Publisher('~idx/depth_image', Image, queue_size=1, tcp_nodelay=True)
            self.data_products.append('depth_image')
            self.depth_img_thread = threading.Thread(target=self.runDepthImgThread)
            self.depth_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.depth_img_subs_thread = threading.Thread(target=self.runDepthImgSubsThread)
            self.depth_img_subs_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopDepthImgAcquisition = stopDepthImgAcquisition
            self.capabilities_report.has_depth_image = True
        else:
            self.capabilities_report.has_depth_image = False

        self.getPointcloud = getPointcloud
        if (self.getPointcloud is not None):
            self.pointcloud_pub = rospy.Publisher('~idx/pointcloud', PointCloud2, queue_size=1, tcp_nodelay=True)
            self.data_products.append('pointcloud')
            self.pointcloud_thread = threading.Thread(target=self.runPointcloudThread)
            self.pointcloud_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.pointcloud_subs_thread = threading.Thread(target=self.runPointcloudSubsThread)
            self.pointcloud_subs_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopPointcloudAcquisition = stopPointcloudAcquisition
            self.capabilities_report.has_pointcloud = True
        else:
            self.capabilities_report.has_pointcloud = False

        self.getPointcloudImg = getPointcloudImg
        if (self.getPointcloudImg is not None):
            self.pointcloud_img_pub = rospy.Publisher('~idx/pointcloud_image', Image, queue_size=1, tcp_nodelay=True)
            self.data_products.append('pointcloud_image')
            self.pointcloud_img_thread = threading.Thread(target=self.runPointcloudImgThread)
            self.pointcloud_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.pointcloud_img_subs_thread = threading.Thread(target=self.runPointcloudImgSubsThread)
            self.pointcloud_img_subs_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopPointcloudImgAcquisition = stopPointcloudImgAcquisition
            self.capabilities_report.has_pointcloud_image = True

            rospy.Subscriber('~idx/set_zoom_ratio', Float32, self.setZoomCb, queue_size=1) # start local callback
            rospy.Subscriber('~idx/set_rotate_ratio', Float32, self.setRotateCb, queue_size=1) # start local callback
            rospy.Subscriber('~idx/set_tilt_ratio', Float32, self.setTiltCb, queue_size=1) # start local callback
        else:
            self.capabilities_report.has_pointcloud_image = False
        
        self.getGPSMsg = getGPSMsg
        if getGPSMsg is not None:
            self.idx_navpose_gps_pub = rospy.Publisher('~idx/gps_fix', NavSatFix, queue_size=1)
            self.navpose_capabilities_report.has_gps = True
        else:
            self.navpose_capabilities_report.has_gps = False

        self.getOdomMsg = getOdomMsg
        if getOdomMsg is not None:
            self.idx_navpose_odom_pub = rospy.Publisher('~idx/odom', Odometry, queue_size=1)
            self.navpose_capabilities_report.has_orientation = True
        else:
            self.navpose_capabilities_report.has_orientation = False

        self.getHeadingMsg = getHeadingMsg
        if getHeadingMsg is not None:
            self.idx_navpose_heading_pub = rospy.Publisher('~idx/heading', Float32, queue_size=1)
            self.navpose_capabilities_report.has_heading = True
        else:
            self.navpose_capabilities_report.has_heading = False

        time.sleep(1)
        # Set up the save data and save cfg i/f and launch saving threads-- Do this before launching aquisition threads so that they can check data_product_should_save() immediately
        self.capabilities_report.data_products = str(self.data_products)


        # Start NEPI IF interfaces
        self.settings_if = SettingsIF(capSettings, factorySettings, settingUpdateFunction, getSettingsFunction)
        factory_data_rates= {}
        for d in self.data_products:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if 'color_2d_image' in self.data_products:
            factory_data_rates['color_2d_image'] = [1.0, 0.0, 100.0] 
        self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates)
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initializeParamServer, paramsModifiedCallback=self.updateFromParamServer)


        # Set up additional publishers
        self.status_msg = IDXStatus()
        self.status_pub = rospy.Publisher('~idx/status', IDXStatus, queue_size=1, latch=True)

        # Start capabilities services
        rospy.Service('~idx/capabilities_query', IDXCapabilitiesQuery, self.provide_capabilities)
        rospy.Service('~idx/navpose_capabilities_query', NavPoseCapabilitiesQuery, self.provide_navpose_capabilities)

        
        # Start NavPose Updates
        if getGPSMsg != None or getOdomMsg != None or getHeadingMsg != None:
            rospy.Timer(rospy.Duration(self.update_navpose_interval_sec), self.navposeCb)

        # Launch the acquisition and saving threads
        if (self.getColor2DImg is not None):
            self.color_img_thread.start()
            self.color_img_subs_thread.start()

        if (self.getBW2DImg is not None):
            self.bw_img_thread.start()
            self.bw_img_subs_thread.start()


        if (self.getDepthMap is not None):
            self.depth_map_thread.start()
            self.depth_map_subs_thread.start()
  
        
        if (self.getDepthImg is not None):
            self.depth_img_thread.start()
            self.depth_img_subs_thread.start()
 
        
        if (self.getPointcloud is not None):
            self.pointcloud_thread.start()
            self.pointcloud_subs_thread.start()
 
        if (self.getPointcloudImg is not None):
           self.pointcloud_img_thread.start()
           self.pointcloud_img_subs_thread.start()
 
        # Update and Publish Status Message
        self.publishStatus()
        ## Initiation Complete
        nepi_msg.publishMsgInfo(self,"Initialization Complete")

    ###############################################################################################

    def resetFactoryCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived update message: " + str(msg))
        nepi_msg.publishMsgInfo(self,"Factory Resetting IDX Sensor Controls")
        self.resetFactory()

    def resetFactory(self):
        if self.settings_if is not None:
            self.settings_if.resetFactorySettings(update_status = False, update_params = True)
        rospy.set_param('~idx/device_name', self.factory_device_name)
        rospy.set_param('~idx/controls_enable',self.factory_controls_dict.get('controls_enable'))
        rospy.set_param('~idx/auto', self.factory_controls_dict.get('auto_adjust'))  
        rospy.set_param('~idx/brightness', self.factory_controls_dict.get('brightness_ratio'))      
        rospy.set_param('~idx/contrast', self.factory_controls_dict.get('contrast_ratio'))
        rospy.set_param('~idx/thresholding', self.factory_controls_dict.get('threshold_ratio'))
        rospy.set_param('~idx/resolution_mode', self.factory_controls_dict.get('resolution_mode'))
        rospy.set_param('~idx/framerate_mode', self.factory_controls_dict.get('framerate_mode'))
        rospy.set_param('~idx/range_window/start_range_ratio', self.factory_controls_dict.get('start_range_ratio'))
        rospy.set_param('~idx/range_window/stop_range_ratio', self.factory_controls_dict.get('stop_range_ratio'))
        rospy.set_param('~idx/frame_3d_transform', self.ZERO_TRANSFORM)
        rospy.set_param('~idx/frame_3d', self.factory_controls_dict.get('frame_3d'))
        self.zoom_ratio = self.init_zoom_ratio
        self.rotate_ratio = self.init_rotate_ratio
        self.tilt_ratio = self.init_rotate_ratio
        self.render_controls = [self.zoom_ratio,self.rotate_ratio,self.tilt_ratio]
        self.publishStatus()

    def updateDeviceNameCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived update message: " + str(msg))
        nepi_msg.publishMsgInfo(self,"Received Device Name update msg")
        new_device_name = msg.data
        self.updateDeviceName(new_device_name)

    def updateDeviceName(self, new_device_name):
        valid_name = True
        for char in self.BAD_NAME_CHAR_LIST:
            if new_device_name.find(char) != -1:
                valid_name = False
        if valid_name is False:
            nepi_msg.publishMsgInfo(self,"Received invalid device name update: " + new_device_name)
        else:
            rospy.set_param('~idx/device_name', new_device_name)
            self.status_msg.device_name = new_device_name
        self.publishStatus(do_updates=False) # Updated inline here 
        self.device_save_config_pub.publish(Empty())


    def resetDeviceNameCb(self,msg):
        nepi_msg.publishMsgInfo(self,"Recived update message: " + str(msg))
        nepi_msg.publishMsgInfo(self,"Received Device Name reset msg")
        self.resetDeviceName()

    def resetDeviceName(self):
        rospy.set_param('~idx/device_name', self.factory_device_name)
        self.status_msg.device_name = self.factory_device_name
        self.publishStatus(do_updates=False) # Updated inline here 
        self.device_save_config_pub.publish(Empty())


    def resetControlsCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived update message: " + str(msg))
        nepi_msg.publishMsgInfo(self,"Resetting IDX Sensor Controls")
        self.resetParamServer(do_updates = True)

    def resetParamServer(self,do_updates = False):
        rospy.set_param('~idx/device_name', self.init_device_name)
        rospy.set_param('~idx/controls_enable',self.init_controls_enable)
        rospy.set_param('~idx/auto', self.init_auto_adjust)       
        rospy.set_param('~idx/brightness', self.init_brightness_ratio)
        rospy.set_param('~idx/contrast', self.init_contrast_ratio)        
        rospy.set_param('~idx/thresholding', self.init_threshold_ratio)
        rospy.set_param('~idx/resolution_mode', self.init_resolution_mode)   
        rospy.set_param('~idx/framerate_mode', self.init_framerate_mode)
        rospy.set_param('~idx/range_window/start_range_ratio', self.init_start_range_ratio)
        rospy.set_param('~idx/range_window/stop_range_ratio', self.init_stop_range_ratio)
        rospy.set_param('~idx/frame_3d_transform', self.init_frame_3d_transform)
        rospy.set_param('~idx/frame_3d', self.init_frame_3d)
        self.zoom_ratio = self.init_zoom_ratio
        self.rotate_ratio = self.init_rotate_ratio
        self.tilt_ratio = self.init_rotate_ratio
        self.render_controls = [self.zoom_ratio,self.rotate_ratio,self.tilt_ratio]
        if do_updates:
            self.updateFromParamServer()
            self.publishStatus()

    def initializeParamServer(self,do_updates = False):
        if self.settings_if is not None:
            self.settings_if.initializeParamServer(do_updates)
        self.init_device_name = rospy.get_param('~idx/device_name', self.factory_device_name)
        self.init_controls_enable = rospy.get_param('~idx/controls_enable',  self.factory_controls_dict["controls_enable"])
        self.init_auto_adjust = rospy.get_param('~idx/auto',  self.factory_controls_dict["auto_adjust"])
        self.init_brightness_ratio = rospy.get_param('~idx/brightness',  self.factory_controls_dict["brightness_ratio"])
        self.init_contrast_ratio = rospy.get_param('~idx/contrast',  self.factory_controls_dict["contrast_ratio"])
        self.init_threshold_ratio = rospy.get_param('~idx/thresholding',  self.factory_controls_dict["threshold_ratio"])
        self.init_resolution_mode = rospy.get_param('~idx/resolution_mode',  self.factory_controls_dict["resolution_mode"])
        self.init_framerate_mode = rospy.get_param('~idx/framerate_mode',  self.factory_controls_dict["framerate_mode"])

        self.init_start_range_ratio = rospy.get_param('~idx/range_window/start_range_ratio',  self.factory_controls_dict["start_range_ratio"])
        self.init_stop_range_ratio = rospy.get_param('~idx/range_window/stop_range_ratio', self.factory_controls_dict["stop_range_ratio"])
        self.init_min_range_m = rospy.get_param('~idx/range_limits/min_range_m',  self.factory_controls_dict["min_range_m"])
        self.init_max_range_m = rospy.get_param('~idx/range_limits/max_range_m',  self.factory_controls_dict["max_range_m"])

        self.init_frame_3d_transform = rospy.get_param('~idx/frame_3d_transform', self.ZERO_TRANSFORM)

        if self.factory_controls_dict["frame_3d"] is None:
            self.init_frame_3d = rospy.get_param('~idx/frame_3d',  'nepi_center_frame')
        else:
            self.init_frame_3d = rospy.get_param('~idx/frame_3d',  self.factory_controls_dict["frame_3d"] )

        self.init_zoom_ratio = self.zoom_ratio
        self.init_rotate_ratio = self.rotate_ratio
        self.init_tilt_ratio = self.rotate_ratio
        self.resetParamServer(do_updates)

    def updateFromParamServer(self):
        if self.settings_if is not None:
            self.settings_if.updateFromParamServer()
        param_dict = rospy.get_param('~idx', {})
        if (self.setControlsEnable is not None and 'controls_enable' in param_dict):
            self.setControlsEnable(param_dict['controls_enable'])
        if (self.setAutoAdjust is not None and 'auto_adjust' in param_dict):
            self.setAutoAdjust(param_dict['auto_adjust'])
        if (self.setBrightness is not None and 'brightness' in param_dict):
            self.setBrightness(param_dict['brightness'])
        if (self.setContrast is not None and 'contrast' in param_dict):
            self.setContrast(param_dict['contrast'])
        if (self.setThresholding is not None and 'thresholding' in param_dict):
            self.setThresholding(param_dict['thresholding'])
        if (self.setResolutionMode is not None and 'resolution_mode' in param_dict):
            self.setResolutionMode(param_dict['resolution_mode'])
        if (self.setFramerateMode is not None and 'framerate_mode' in param_dict):
            self.setFramerateMode(param_dict['framerate_mode'])
        if (self.setRange is not None and 'start_range' in param_dict and 'stop_range' in param_dict):
            self.setRange(param_dict['range_window']['start_range'], param_dict['range_window']['stop_range'])



    # Define local IDX Control callbacks
    def setControlsEnableCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived IDX Controls enable update message: " + str(msg))
        new_controls_enable = msg.data
        nepi_msg.publishMsgInfo(self,"new_controls_enable")
        if self.setControlsEnable is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setControlsEnable(new_controls_enable)
            if status is True:
                rospy.set_param('~idx/controls_enable', new_controls_enable)
                self.status_msg.controls_enable = new_controls_enable
                self.publishStatus(do_updates=False) # Updated inline here
        else:
            nepi_msg.publishMsgInfo(self,"Ignoring set controls_enable.  Driver has no setControlsEnable function")
            self.publishStatus(do_updates=False) # Updated inline here

            
    def setAutoAdjustCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Auto Adjust update message: " + str(msg))
        new_auto_adjust = msg.data
        if self.setAutoAdjust is None:
            nepi_msg.publishMsgInfo(self,"Ignoring Set Auto Adjust. Driver has no setAutoAdjust function")   
        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setAutoAdjust(new_auto_adjust)
            if status is True:
                rospy.set_param('~idx/auto_adjust', new_auto_adjust)
                self.status_msg.auto_adjust = new_auto_adjust
                if new_auto_adjust:
                    nepi_msg.publishMsgInfo(self,"Enabling Auto Adjust")
                else:
                    nepi_msg.publishMsgInfo(self,"Disabling IDX Auto Adjust")
            else:
                rospy.logerr("Failed to update auto adjust: " + err_str)
        self.publishStatus(do_updates=False) # Updated inline here



    def setBrightnessCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Brightness update message: " + str(msg))
        new_brightness = msg.data
        if rospy.get_param('~idx/auto', self.init_auto_adjust):
            nepi_msg.publishMsgInfo(self,"Ignoring Set Brightness request. Auto Adjust enabled")
        else:
            if self.setBrightness is None:
                nepi_msg.publishMsgInfo(self,"Ignoring Set Brightness. Driver has no setBrightness function")
            else:
                if (new_brightness < 0.0 or new_brightness > 1.0):
                    rospy.logerr("Brightness value out of bounds")
                else:
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setBrightness(new_brightness)
                    if status is True:
                        rospy.set_param('~idx/brightness', new_brightness)
                        self.status_msg.brightness = new_brightness
                    else:
                        rospy.logerr("Failed to update brightness: " + err_str)
        self.publishStatus(do_updates=False) # Updated inline here

    def setContrastCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Contrast update message: " + str(msg))
        new_contrast = msg.data
        if rospy.get_param('~idx/auto', self.init_auto_adjust):
            nepi_msg.publishMsgInfo(self,"Ignoring Set Contrast request. Auto Adjust enabled")
        else:
            if self.setContrast is None:
                nepi_msg.publishMsgInfo(self,"Ignoring Set Contrast. Driver has no setContrast function")
            else:
                if (new_contrast < 0.0 and new_contrast != -1.0) or (new_contrast > 1.0):
                    rospy.logerr("Contrast value out of bounds")
                    self.publishStatus(do_updates=False) # No change
                    return
                else:
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setContrast(new_contrast)
                    if status is True:
                        rospy.set_param('~idx/contrast', new_contrast)
                        self.status_msg.contrast = new_contrast
                    else:
                        rospy.logerr("Failed to update contrast: " + err_str)
        self.publishStatus(do_updates=False) # Updated inline here


    def setThresholdingCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Threshold update message: " + str(msg))
        new_thresholding = msg.data
        if self.setThresholding is None:
            nepi_msg.publishMsgInfo(self,"Ignoring Set Thresholding. Driver has no setThresholding function")
        else:
            if (new_thresholding < 0.0 or new_thresholding > 1.0):
                rospy.logerr("Thresholding value out of bounds")
                self.publishStatus(do_updates=False) # No change
                return
            else:
                # Call the parent's method and update ROS param as necessary
                # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                status, err_str = self.setThresholding(new_thresholding)
                if status is True:
                    rospy.set_param('~idx/thresholding', new_thresholding)
                    self.status_msg.thresholding = new_thresholding
                else:
                    rospy.logerr("Failed to update thresholding: " + err_str)
        self.publishStatus(do_updates=False) # Updated inline here

    def setResolutionModeCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Resolution update message: " + str(msg))
        new_resolution = msg.data
        if (new_resolution > self.RESOLUTION_MODE_MAX):
                rospy.logerr("Resolution mode value out of bounds")
                self.publishStatus(do_updates=False) # No change
                return
        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setResolutionMode(new_resolution)
            if status is True:
                rospy.set_param('~idx/resolution_mode', new_resolution)
                self.status_msg.resolution_mode = new_resolution
            else:
                rospy.logerr("Failed to update resolution: " + err_str)
        self.publishStatus(do_updates=False) # Updated inline here


        
    def setFramerateModeCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Framerate update message: " + str(msg))
        new_framerate = msg.data
        if (new_framerate > self.FRAMERATE_MODE_MAX):
            rospy.logerr("Framerate mode value out of bounds")
            self.publishStatus(do_updates=False) # No change
            return
        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setFramerateMode(new_framerate)
            if status is True:
                rospy.set_param('~idx/framerate_mode', new_framerate)
                self.status_msg.framerate_mode = new_framerate
                self.status_msg.framerate_current = self.getFramerate()
            else:
                rospy.logerr("Failed to update framerate: " + err_str)
        self.publishStatus(do_updates=False) # Updated inline here

 
    def setRangeCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Range update message: " + str(msg))
        nepi_msg.publishMsgInfo(self,"Recived update message: " + str(msg))
        new_start_range_ratio = msg.start_range
        new_stop_range_ratio = msg.stop_range
        if (new_start_range_ratio < 0 or new_stop_range_ratio > 1 or new_stop_range_ratio < new_start_range_ratio):
            rospy.logerr("Range values out of bounds")
            self.publishStatus(do_updates=False) # No change
            return
        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setRange(new_start_range_ratio,new_stop_range_ratio)
            if status is True:
                rospy.set_param('~idx/range_window/start_range_ratio', new_start_range_ratio)
                rospy.set_param('~idx/range_window/stop_range_ratio', new_stop_range_ratio)
                self.status_msg.range_window.start_range = new_start_range_ratio
                self.status_msg.range_window.stop_range = new_stop_range_ratio
            else:
                rospy.logerr("Failed to update framerate: " + err_str)
        self.publishStatus(do_updates=False) # Updated inline here       

    def setZoomCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Zoom update message: " + str(msg))
        new_zoom = msg.data
        if (new_zoom < 0.0 and new_zoom != -1.0) or (new_zoom > 1.0):
            rospy.logerr("Zoom value out of bounds")
            self.publishStatus(do_updates=False) # No change
            return
        else:
            self.zoom_ratio = new_zoom
            self.status_msg.zoom = new_zoom
            self.render_controls[0] = new_zoom
        self.publishStatus(do_updates=False) # Updated inline here

    def setRotateCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Rotate update message: " + str(msg))
        new_rotate = msg.data
        if (new_rotate < 0.0 and new_rotate != -1.0) or (new_rotate > 1.0):
            rospy.logerr("rotate value out of bounds")
            self.publishStatus(do_updates=False) # No change
            return
        else:
            self.rotate_ratio = new_rotate
            self.status_msg.rotate = new_rotate
            self.render_controls[1] = new_rotate
        self.publishStatus(do_updates=False) # Updated inline here  

    def setTiltCb(self, msg):
        new_tilt = msg.data
        if (new_tilt < 0.0 and new_tilt != -1.0) or (new_tilt > 1.0):
            rospy.logerr("tilt value out of bounds")
            self.publishStatus(do_updates=False) # No change
            return
        else:
            self.tilt_ratio = new_tilt
            self.status_msg.tilt = new_tilt
            self.render_controls[2] = new_tilt
        self.publishStatus(do_updates=False) # Updated inline here  

    def setFrame3dTransformCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived 3D Transform update message: " + str(msg))
        new_transform_msg = msg
        self.setFrame3dTransform(new_transform_msg)

    def setFrame3dTransform(self, transform_msg):
        x = transform_msg.translate_vector.x
        y = transform_msg.translate_vector.y
        z = transform_msg.translate_vector.z
        roll = transform_msg.rotate_vector.x
        pitch = transform_msg.rotate_vector.y
        yaw = transform_msg.rotate_vector.z
        heading = transform_msg.heading_offset
        transform = [x,y,z,roll,pitch,yaw,heading]
        self.init_frame_3d_transform = rospy.set_param('~idx/frame_3d_transform',  transform)
        self.status_msg.frame_3d_transform = transform_msg
        self.publishStatus(do_updates=False) # Updated inline here 

    def clearFrame3dTransformCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Clear 3D Transform update message: " + str(msg))
        new_transform_msg = msg
        self.clearFrame3dTransform()

    def clearFrame3dTransform(self, transform_msg):
        transform = self.ZERO_TRANSFORM
        self.init_frame_3d_transform = rospy.set_param('~idx/frame_3d_transform',  transform)
        self.status_msg.frame_3d_transform = transform_msg
        self.publishStatus(do_updates=False) # Updated inline here 

    def setFrame3dCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Set 3D Transform update message: " + str(msg))
        new_frame_3d = msg.data
        self.setFrame3d(new_frame_3d)

    def setFrame3d(self, new_frame_3d):
        rospy.set_param('~idx/frame_3d', new_frame_3d)
        self.status_msg.frame_3d = new_frame_3d
        self.publishStatus(do_updates=False) # Updated inline here 
   
    def provide_capabilities(self, _):
        return self.capabilities_report
    
    def provide_navpose_capabilities(self, _):
        return self.navpose_capabilities_report  

           

  
    # Image from img_get_function can be CV2 or ROS image.  Will be converted as needed in the thread
    def image_thread_proccess(self,data_product,img_get_function,img_stop_function,img_publisher,has_subs_var_str):
        image = None
        cv2_img = None
        ros_img = None
        if not rospy.is_shutdown():
            nepi_msg.publishMsgInfo(self,rospy.get_name() + ": starting " + data_product + " acquisition thread")
            acquiring = False
            while (not rospy.is_shutdown()):
                has_subscribers = eval('self.' + has_subs_var_str) #(img_publisher.get_num_connections() > 0)
                saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
                snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
                #if (data_product == "color_2d_image"):
                    #nepi_msg.publishMsgWarn(self,rospy.get_name() + ": Img should save: " + str(saving_is_enabled))
                if (has_subscribers is True) or (saving_is_enabled is True) or (snapshot_enabled is True):
                    acquiring = True
                    if data_product != "pointcloud_image":
                        status, msg, image, ros_timestamp, encoding = img_get_function()
                    else:
                        status, msg, image, ros_timestamp, encoding = img_get_function(self.render_controls)
                    if (status is False):
                        #rospy.logerr_throttle(1, msg)
                        continue   
                    if (has_subscribers is True):
                        if isinstance(image,np.ndarray):  # CV2 image. Convert to ROS Image   
                            # Convert cv to ros   
                            ros_img = nepi_img.cv2img_to_rosimg(image, encoding=encoding)
                            ros_img.header.stamp = ros_timestamp
                            ros_img.header.frame_id = rospy.get_param('~idx/frame_3d',  self.init_frame_3d )
                        elif isinstance(image,Image): # ROS Image. Passthrough
                            ros_img = image
                        if ros_img is not None:
                            # Publish image
                            img_publisher.publish(ros_img)
                    should_save = self.save_data_if.data_product_should_save(data_product)
                    if ((saving_is_enabled is True and should_save is True) or snapshot_enabled is True ):
                        if isinstance(image,np.ndarray):  # CV2 image. Passthrough   
                            cv2_img = image
                        elif isinstance(image,Image): # ROS Image. Convert to CV2 Image
                            cv2_img = nepi_img.rosimg_to_cv2img(image, encoding=encoding)
                        nepi_save.save_img2file(self,data_product,cv2_img,ros_timestamp,save_check=False)
                elif acquiring is True:
                    if img_stop_function is not None:
                        nepi_msg.publishMsgInfo(self,"Stopping " + data_product + " acquisition")
                        img_stop_function()
                    acquiring = False
                else: # No subscribers and already stopped
                    acquiring = False
                    rospy.sleep(0.25)
                rospy.sleep(0.01) # Yield



    
    # Pointcloud from pointcloud_get_function can be open3D or ROS pointcloud.  Will be converted as needed in the thread
    def pointcloud_thread_proccess(self,data_product,pc_get_function,pc_stop_function,pc_publisher,has_subs_var_str):
        pc = None
        o3d_pc = None
        ros_pc = None
        if not rospy.is_shutdown():
            nepi_msg.publishMsgInfo(self,rospy.get_name() + ": starting " + data_product + " acquisition thread")
            acquiring = False
            while (not rospy.is_shutdown()):
                has_subscribers = eval('self.' + has_subs_var_str) #(img_publisher.get_num_connections() > 0)
                saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
                snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
                if (has_subscribers is True) or (saving_is_enabled is True) or (snapshot_enabled is True):
                    acquiring = True
                    status, msg, pc, ros_timestamp, ros_frame = pc_get_function()
                    if pc is not None:
                        #********************
                        set_frame = rospy.get_param('~idx/frame_3d',  self.init_frame_3d )
                        if set_frame == 'sensor_frame':
                            ros_frame = set_frame # else pass through sensor frame
                        else:
                            ros_frame = set_frame

                        transform = rospy.get_param('~idx/frame_3d_transform', self.init_frame_3d_transform)
                        zero_transform = True
                        for i in range(len(transform)):
                            if transform[i] != 0:
                                zero_transform = False
                        should_transform = (zero_transform == False) and (ros_frame == 'nepi_center_frame')
                        if should_transform:
                            if isinstance(pc,o3d.geometry.PointCloud):  # Open3d pointcloud. Passthrough   
                                o3d_pc = pc
                            elif isinstance(pc,PointCloud2): # ROS Pointcloud. Convert to Open3d pointcloud
                                o3d_pc = nepi_pc.rospc_to_o3dpc(pc, remove_nans=True)    
                            pc = self.transformPointcloud(o3d_pc,transform)
                           
                        #********************
                        if (status is False):
                            #rospy.logerr_throttle(1, msg)
                            continue   
                        if (has_subscribers is True):
                            if isinstance(pc,o3d.geometry.PointCloud):  # Open3D Pointcloud. Convert to ROS Pointcloud   
                                # Convert o3d to ros   
                                ros_pc = nepi_pc.o3dpc_to_rospc(pc, stamp = ros_timestamp, frame_id = ros_frame )
                            elif isinstance(pc,PointCloud2): # ROS Pointcloud. Passthrough
                                ros_pc = pc
                            if ros_pc is not None:
                                # Publish Pointcloud
                                pc_publisher.publish(ros_pc)
                    should_save = self.save_data_if.data_product_should_save(data_product)
                    if ((saving_is_enabled is True and should_save is True) or snapshot_enabled is True ):
                            if o3d_pc == None:
                                if isinstance(pc,o3d.geometry.PointCloud):  # Open3d pointcloud. Passthrough   
                                    o3d_pc = pc
                                elif isinstance(pc,PointCloud2): # ROS Pointcloud. Convert to Open3d pointcloud
                                    o3d_pc = nepi_pc.rospc_to_o3dpc(pc, remove_nans=True)
                            nepi_save.save_pc2file(self,data_product,o3d_pc,ros_timestamp, save_check=False)
                elif acquiring is True:
                    if pc_stop_function is not None:
                        nepi_msg.publishMsgInfo(self,"Stopping " + data_product + " acquisition")
                        pc_stop_function()
                    acquiring = False
                else: # No subscribers and already stopped
                    acquiring = False
                    rospy.sleep(0.25)
                rospy.sleep(0.01) # Yield
                

    # check for subscribers thread
    def subs_thread_proccess(self,publisher,has_subs_var_str):
        while (not rospy.is_shutdown()):
            has_subscribers = (publisher.get_num_connections() > 0)
            setattr(self,has_subs_var_str,has_subscribers)
            nepi_ros.sleep(1)


    def transformPointcloud(self, o3d_pc, transform):
        x = transform[0]
        y = transform[1]
        z = transform[2]
        translation_vector = [x, y, z]
        roll = transform[3]
        pitch = transform[4]
        yaw = transform[5]
        rotate_vector = [roll, pitch, yaw]
        o3d_pc = nepi_pc.translate_pc(o3d_pc, translation_vector)
        o3d_pc = nepi_pc.rotate_pc(o3d_pc, rotate_vector)
        return o3d_pc


    def runColorImgThread(self):
        self.image_thread_proccess('color_2d_image', self.getColor2DImg, self.stopColor2DImgAcquisition, self.color_img_pub, "cl_img_has_subs")
        
    def runBWImgThread(self):
        self.image_thread_proccess('bw_2d_image', self.getBW2DImg, self.stopBW2DImgAcquisition, self.bw_img_pub, "bw_img_has_subs" )

    def runDepthMapThread(self):
        self.image_thread_proccess('depth_map', self.getDepthMap, self.stopDepthMapAcquisition, self.depth_map_pub, "dm_img_has_subs")

    def runDepthImgThread(self):
        self.image_thread_proccess('depth_image', self.getDepthImg, self.stopDepthImgAcquisition, self.depth_img_pub, "di_img_has_subs")

    def runPointcloudThread(self):
        self.pointcloud_thread_proccess('pointcloud', self.getPointcloud, self.stopPointcloudAcquisition, self.pointcloud_pub, "pc_has_subs")

    def runPointcloudImgThread(self):
        self.image_thread_proccess('pointcloud_image', self.getPointcloudImg, self.stopPointcloudImgAcquisition, self.pointcloud_img_pub, "pc_img_has_subs")


    def runColorImgSubsThread(self):
        self.subs_thread_proccess(self.color_img_pub, "cl_img_has_subs")
        
    def runBwImgSubsThread(self):
        self.subs_thread_proccess(self.bw_img_pub, "bw_img_has_subs")

    def runDepthMapSubsThread(self):
        self.subs_thread_proccess(self.depth_map_pub, "dm_img_has_subs")

    def runDepthImgSubsThread(self):
        self.subs_thread_proccess(self.depth_img_pub, "di_img_has_subs")

    def runPointcloudSubsThread(self):
        self.subs_thread_proccess(self.pointcloud_pub, "pc_has_subs")

    def runPointcloudImgSubsThread(self):
        self.subs_thread_proccess(self.pointcloud_img_pub, "pc_img_has_subs")


                
    def navposeCb(self, timer):
        if self.getGPSMsg != None:
            gps_msg = self.getGPSMsg()
            if gps_msg is not None:
                if gps_msg.header.stamp != self.last_gps_timestamp:
                    self.idx_navpose_gps_pub.publish(gps_msg)
                    self.last_gps_timestamp = gps_msg.header.stamp
        if self.getOdomMsg != None:
            odom_msg = self.getOdomMsg()
            if odom_msg is not None:
                if odom_msg.header.stamp != self.last_odom_timestamp:
                    self.idx_navpose_odom_pub.publish(odom_msg)
                    self.last_odom_timestamp = odom_msg.header.stamp

        if self.getHeadingMsg != None:
            heading_msg = self.getHeadingMsg()
            if heading_msg is not None:
                if heading_msg.header.stamp != self.last_heading_timestamp:
                    self.idx_navpose_gps_pub.publish(heading_msg)   
                    self.last_heading_timestamp = heading_msg.header.stamp


    # Utility Functions

    def applyIDXControls2Image(self,cv2_img,IDXcontrols_dict=DEFAULT_CONTROLS_DICT,current_fps = 20):
        if IDXcontrols_dict.get("controls_enable"): 
            resolution_ratio = IDXcontrols_dict.get("resolution_mode")/3
            [cv2_img,new_res] = nepi_img.adjust_resolution(cv2_img, resolution_ratio)
            if IDXcontrols_dict.get("auto_adjust") is False:
                cv2_img = nepi_img.adjust_brightness(cv2_img,IDXcontrols_dict.get("brightness_ratio"))
                cv2_img = nepi_img.adjust_contrast(cv2_img,IDXcontrols_dict.get("contrast_ratio"))
                cv2_img = nepi_img.adjust_sharpness(cv2_img,IDXcontrols_dict.get("threshold_ratio"))
            else:
                cv2_img = nepi_img.adjust_auto(cv2_img,0.3)
        return cv2_img


    # Function to update and publish status message

    def publishStatus(self, do_updates = True):
        if do_updates is True:
            # TODO: Probably these should be queried from the parent (and through the driver) via explicit callbacks rather than via the param server
            idx_params = rospy.get_param('~idx')

            self.status_msg.device_name = idx_params['device_name'] if 'device_name' in idx_params else self.init_device_name
            self.status_msg.sensor_name = self.sensor_name
            self.status_msg.identifier = self.identifier
            self.status_msg.serial_num = self.serial_num
            self.status_msg.hw_version = self.hw_version
            self.status_msg.sw_version = self.sw_version

            self.status_msg.controls_enable = idx_params['controls_enable'] if 'controls_enable' in idx_params else True
            self.status_msg.auto_adjust = idx_params['auto_adjust'] if 'auto_adjust' in idx_params else False
            self.status_msg.resolution_mode = idx_params['resolution_mode'] if 'resolution_mode' in idx_params else 0
            self.status_msg.framerate_mode = idx_params['framerate_mode'] if 'framerate_mode' in idx_params else 0
            fr = 0.0
            if self.getFramerate is not None:
                fr =  self.getFramerate()
            self.status_msg.framerate_current = fr
            self.status_msg.contrast = idx_params['contrast'] if 'contrast' in idx_params else 0
            self.status_msg.brightness = idx_params['brightness'] if 'brightness' in idx_params else 0
            self.status_msg.thresholding = idx_params['thresholding'] if 'thresholding' in idx_params else 0
            
            self.status_msg.range_window.start_range = rospy.get_param('~idx/range_window/start_range_ratio', 0.0)
            self.status_msg.range_window.stop_range =  rospy.get_param('~idx/range_window/stop_range_ratio', 1.0)
            self.status_msg.min_range_m = rospy.get_param('~idx/range_limits/min_range_m',0.0)
            self.status_msg.max_range_m = rospy.get_param('~idx/range_limits/max_range_m',1.0)
            # The transfer frame into which 3D data (pointclouds) are transformed for the pointcloud data topic
            transform = rospy.get_param('~idx/frame_3d_transform',  self.ZERO_TRANSFORM)
            transform_msg = Frame3DTransform()
            transform_msg.translate_vector.x = transform[0]
            transform_msg.translate_vector.y = transform[1]
            transform_msg.translate_vector.z = transform[2]
            transform_msg.rotate_vector.x = transform[3]
            transform_msg.rotate_vector.y = transform[4]
            transform_msg.rotate_vector.z = transform[5]
            transform_msg.heading_offset = transform[6]
            self.status_msg.frame_3d_transform = transform_msg
            
            self.status_msg.frame_3d = idx_params['frame_3d'] if 'frame_3d' in idx_params else "nepi_center_frame"
            self.status_msg.zoom = self.zoom_ratio
            self.status_msg.rotate = self.rotate_ratio
            self.status_msg.tilt = self.tilt_ratio

        self.status_pub.publish(self.status_msg)
    
