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

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_save
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from nepi_ros_interfaces.msg import IDXStatus, RangeWindow, SaveData, SaveDataRate, SaveDataStatus
from nepi_ros_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryResponse
from nepi_ros_interfaces.msg import ImageStatus, PointcloudStatus
from nepi_ros_interfaces.msg import NavPose, Frame3DTransform

from geometry_msgs.msg import Vector3

from nepi_api.node_if import NodeClassIF
from nepi_api.sys_if_msg import MsgIF
from nepi_api.data_if_image import ImageIF
from nepi_api.data_if_pointcloud import PointcloudIF
from nepi_api.device_if_npx import NPXDeviceIF
from nepi_api.sys_if_settings import SettingsIF
from nepi_api.sys_if_save_data import SaveDataIF
from nepi_api.sys_if_save_cfg import SaveCfgIF





#***************************
# IDX utility functions

#Factory Control Values 
DEFAULT_CONTROLS_DICT = dict( controls_enable = True,
    auto_adjust = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.5,
    resolution_ratio = 1.0, # 25%, 50%, 75%, Full
    framerate_ratio = 0.25, # 25%, 50%, 75%, Full
    start_range_ratio = 0.0,
    stop_range_ratio = 1.0,
    min_range_m = 0.0,
    max_range_m = 1.0,
    zoom_ratio = 0.5, 
    rotate_ratio = 0.5,
    frame_3d = 'nepi_center_frame'
    )


class IDXDeviceIF:
    # Default Global Values
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]
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
    init_resolution_ratio = None
    init_framerate_ratio = None
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


    data_products_list = []

    settings_if = None
    save_data_if = None
    save_cfg_if = None

    update_navpose_interval_sec = float(1)/UPDATE_NAVPOSE_RATE_HZ
    last_gps_timestamp = None
    last_odom_timestamp = None
    last_heading_timestamp = None

   
    rtsp_url = None


    img_height = 0
    img_width = 0

    data_product_dict = dict()

    navpose_if = None
    
    #######################
    ### IF Initialization
    def __init__(self, device_info, capSettings=None, 
                 factorySettings=None, settingUpdateFunction=None, getSettingsFunction=None,
                 factoryControls = None, setControlsEnable=None, setAutoAdjust=None,
                 get_rtsp_url = None,
                 setContrast=None, setBrightness=None, setThresholding=None,
                 setResolutionRatio=None, setFramerateRatio=None, 
                 setRange=None, getFramerate=None,
                 getColor2DImg=None, stopColor2DImgAcquisition=None, 
                 getBW2DImg=None, stopBW2DImgAcquisition=None,
                 getDepthMap=None, stopDepthMapAcquisition=None, 
                 getDepthImg=None, stopDepthImgAcquisition=None, 
                 getPointcloud=None, stopPointcloudAcquisition=None, 
                 getPointcloudImg=None, stopPointcloudImgAcquisition=None, 
                 getNavPoseDictFunction=None):

        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        log_name = self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        


        ############################# 
        self.sensor_name = device_info["sensor_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.factory_device_name = device_info["sensor_name"] + "_" + device_info["identifier"]

        # Create the CV bridge. Do this early so it can be used in the threading run() methods below 
        # TODO: Need one per image output type for thread safety?

        self.capabilities_report = IDXCapabilitiesQueryResponse()

        # Create and update factory controls dictionary
        self.factory_controls_dict = DEFAULT_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]
        

        ############################# 
        # Init Param Server
        self.initCb(do_updates = False)

        # Set up standard IDX parameters with ROS param and subscriptions
        # Defer actually setting these on the camera via the parent callbacks... the parent may need to do some 
        # additional setup/calculation first. Parent can then get these all applied by calling ApplyConfigUpdates()

        self.get_rtsp_url = get_rtsp_url


        self.setControlsEnable = setControlsEnable
        rospy.Subscriber('~idx/set_controls_enable', Bool, self.setControlsEnableCb, queue_size=1) # start local callback
     
        self.setAutoAdjust = setAutoAdjust
        rospy.Subscriber('~idx/set_auto_adjust', Bool, self.setAutoAdjustCb, queue_size=1) # start local callback
        self.capabilities_report.auto_adjustment = True

    
        self.setBrightness = setBrightness
        rospy.Subscriber('~idx/set_brightness', Float32, self.setBrightnessCb, queue_size=1) # start local callback
        self.capabilities_report.adjustable_brightness = True


        self.setContrast = setContrast
        rospy.Subscriber('~idx/set_contrast', Float32, self.setContrastCb, queue_size=1) # start local callback
        self.capabilities_report.adjustable_contrast = True


        self.setThresholding = setThresholding       
        rospy.Subscriber('~idx/set_thresholding', Float32, self.setThresholdingCb, queue_size=1) # start local callback
        self.capabilities_report.adjustable_thresholding = True

        self.setResolutionRatio = setResolutionRatio
        rospy.Subscriber('~idx/set_resolution_ratio', Float32, self.setResolutionRatioCb, queue_size=1) # start local callback
        self.capabilities_report.adjustable_resolution = True

               
        self.setFramerateRatio = setFramerateRatio
        self.getFramerate = getFramerate
        rospy.Subscriber('~idx/set_framerate_ratio', Float32, self.setFramerateRatioCb, queue_size=1) # start local callback
        self.capabilities_report.adjustable_framerate = True


        self.setRange = setRange
        rospy.Subscriber('~idx/set_range_window', RangeWindow, self.setRangeCb, queue_size=1)
        self.capabilities_report.adjustable_range = True

        rospy.Subscriber('~idx/clear_frame_3d_transform', Bool, self.clearFrame3dTransformCb, queue_size=1)
        rospy.Subscriber('~idx/set_frame_3d_transform', Frame3DTransform, self.setFrame3dTransformCb, queue_size=1)
        rospy.Subscriber('~idx/set_frame_3d', String, self.setFrame3dCb, queue_size=1)

        rospy.Subscriber('~idx/reset_controls', Empty, self.resetControlsCb, queue_size=1) # start local callback
        rospy.Subscriber('~idx/reset_factory', Empty, self.factoryResetCbCb, queue_size=1) # start local callback

        rospy.Subscriber('~idx/update_device_name', String, self.updateDeviceNameCb, queue_size=1) # start local callbac
        rospy.Subscriber('~idx/reset_device_name', Empty, self.resetDeviceNameCb, queue_size=1) # start local callback

        # Start the data producers  
        
        if (getColor2DImg is not None):
            self.getColor2DImg = getColor2DImg
            self.stopColor2DImgAcquisition = stopColor2DImgAcquisition
            data_product = 'color_2d_image'

            start_data_function = self.getColor2DImg
            stop_data_function = self.stopColor2DImgAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread")
            self.color_img_thread = threading.Thread(target=self.runColorImgThread)
            self.color_img_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.capabilities_report.has_color_2d_image = True
        else:
            self.capabilities_report.has_color_2d_image = False
        

        if (getBW2DImg is not None):
            self.getBW2DImg = getBW2DImg
            self.stopBW2DImgAcquisition = stopBW2DImgAcquisition
            data_product = 'bw_2d_image'

            start_data_function = self.getBW2DImg
            stop_data_function = self.stopBW2DImgAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)

            self.bw_img_thread = threading.Thread(target=self.runBWImgThread)
            self.bw_img_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.capabilities_report.has_bw_2d_image = True
        else:
            self.capabilities_report.has_bw_2d_image = False
        

        if (getDepthMap is not None):
            self.getDepthMap = getDepthMap
            self.stopDepthMapAcquisition = stopDepthMapAcquisition
            data_product = 'depth_map'
            start_data_function = self.getDepthMap
            stop_data_function = self.stopDepthMapAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread")
            self.depth_map_thread = threading.Thread(target=self.runDepthMapThread)
            self.depth_map_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.capabilities_report.has_depth_map = True
        else:
            self.capabilities_report.has_depth_map = False
            
        if (getDepthImg is not None):
            self.getDepthImg = getDepthImg
            self.stopDepthImgAcquisition = stopDepthImgAcquisition
            data_product = 'depth_image'
            start_data_function = self.getDepthImg
            stop_data_function = self.stopDepthImgAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread")
            self.depth_img_thread = threading.Thread(target=self.runDepthImgThread)
            self.depth_img_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.capabilities_report.has_depth_image = True
        else:
            self.capabilities_report.has_depth_image = False

        if (getPointcloud is not None):
            self.getPointcloud = getPointcloud
            self.stopPointcloudAcquisition = stopPointcloudAcquisition
            data_product = 'pointcloud'
            start_data_function = self.getPointcloud
            stop_data_function = self.stopPointcloudAcquisition
            data_msg = PointCloud2
            data_status_msg = PointcloudStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread")
            self.pointcloud_thread = threading.Thread(target=self.runPointcloudThread)
            self.pointcloud_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.capabilities_report.has_pointcloud = True
        else:
            self.capabilities_report.has_pointcloud = False

        if (getPointcloudImg is not None):
            self.getPointcloudImg = getPointcloudImg
            self.stopPointcloudImgAcquisition = stopPointcloudImgAcquisition
            data_product = 'pointcloud_image'

            start_data_function = self.getPointcloudImg
            stop_data_function = self.stopPointcloudImgAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread")
            self.pointcloud_img_thread = threading.Thread(target=self.runPointcloudImgThread)
            self.pointcloud_img_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.capabilities_report.has_pointcloud_image = True

            rospy.Subscriber('~idx/set_zoom_ratio', Float32, self.setZoomCb, queue_size=1) # start local callback
            rospy.Subscriber('~idx/set_rotate_ratio', Float32, self.setRotateCb, queue_size=1) # start local callback
            rospy.Subscriber('~idx/set_tilt_ratio', Float32, self.setTiltCb, queue_size=1) # start local callback
        else:
            self.capabilities_report.has_pointcloud_image = False
        
        ######
        # Create a navpose publisher if supported
        self.getNavPoseDictFunction = getNavPoseDictFunction
        if getNavPoseDictFunction is not None:
            navpose_namespace = os.path.join(self.base_namespace,self.node_name,'idx')
            navpose_if = NPXDeviceIF(navpose_namespace, self.getNavPoseDictFunction, pub_rate = 10)


        time.sleep(1)
        # Set up the save data and save cfg i/f and launch saving threads-- Do this before launching aquisition threads so that they can check data_product_should_save() immediately
        self.capabilities_report.data_products = str(self.data_products_list)


        # Start NEPI IF interfaces
        self.capSettings = capSettings
        self.factorySettings = factorySettings
        self.settingUpdateFunction = settingUpdateFunction
        self.getSettingsFunction = getSettingsFunction
        self.settings_if = SettingsIF(self.capSettings, self.factorySettings, self.settingUpdateFunction, self.getSettingsFunction)


        factory_data_rates= {}
        for d in self.data_products_list:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if 'color_2d_image' in self.data_products_list:
            factory_data_rates['color_2d_image'] = [1.0, 0.0, 100.0] 
        self.save_data_if = SaveDataIF(data_product_names = self.data_products_list, factory_data_rate_dict = factory_data_rates)


        self.save_cfg_if = SaveCfgIF(initCb=self.initCb, resetCb=self.resetCb,  factoryResetCb=self.factoryResetCb)
        ready = self.save_cfg_if.wait_for_ready()




        # Set up additional publishers
        self.status_msg = IDXStatus()
        self.status_pub = rospy.Publisher('~idx/status', IDXStatus, queue_size=1, latch=True)




        self.msg_if.pub_warn("Creating IDX Capabilites Service with caps: " + str(self.capabilities_report))
        # Start capabilities services
        rospy.Service('~idx/capabilities_query', IDXCapabilitiesQuery, self.provide_capabilities)

        # Launch the acquisition and saving threads
        if (getColor2DImg is not None):
            self.color_img_thread.start()

        if (getBW2DImg is not None):
            self.bw_img_thread.start()

        if (getDepthMap is not None):
            self.depth_map_thread.start()

        if (getDepthImg is not None):
            self.depth_img_thread.start()

        if (getPointcloud is not None):
            self.pointcloud_thread.start()

        if (getPointcloudImg is not None):
           self.pointcloud_img_thread.start()

        ####################################
        self.initCb(do_updates = True)

        # Update and Publish Status Message
        self.publishStatus()
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")

    ###############################################################################################


    def addDataProduct2Dict(self,data_product,start_data_function,stop_data_function,data_msg,data_status_msg):
        success = False
        data_product = data_product
        pub_namespace = os.path.join(self.base_namespace,self.node_name,'idx')
        topic = os.path.join(pub_namespace, data_product)
        dp_dict = dict()
        dp_dict['data_product'] = data_product
        dp_dict['topic'] = topic

        dp_dict['get_data'] = start_data_function
        dp_dict['stop_data'] = stop_data_function
        if data_product != 'pointcloud':
            dp_dict['data_if'] = ImageIF(data_name = data_product, pub_namespace = pub_namespace)
        else:
            dp_dict['data_if'] = PointcloudIF(data_name = data_product, pub_namespace = pub_namespace )
        self.data_products_list.append(data_product)
        self.data_product_dict[data_product] = dp_dict


        # Do same for raw data without start stop functions for images
        if data_product != 'pointcloud':
            data_product = data_product + '_raw'
            topic = os.path.join(pub_namespace, data_product)
            dp_dict = dict()
            dp_dict['data_product'] = data_product
            dp_dict['topic'] = topic
            if data_product != 'pointcloud':
                dp_dict['data_if'] = ImageIF(data_name = data_product, pub_namespace = pub_namespace, do_wait = False)
            else:
                dp_dict['data_if'] = PointcloudIF(data_product, topic)

            time.sleep(1)

            self.data_products_list.append(data_product)
            self.data_product_dict[data_product] = dp_dict
        # do wait here for all
        time.sleep(1)
        success = True
        return success


    def factoryResetCbCb(self, msg):
        self.msg_if.pub_info("Recived update message: " + str(msg))
        self.msg_if.pub_info("Factory Resetting IDX Sensor Controls")
        self.factoryResetCb()



    def updateDeviceNameCb(self, msg):
        self.msg_if.pub_info("Recived update message: " + str(msg))
        self.msg_if.pub_info("Received Device Name update msg")
        new_device_name = msg.data
        self.updateDeviceName(new_device_name)

    def updateDeviceName(self, new_device_name):
        valid_name = True
        for char in self.BAD_NAME_CHAR_LIST:
            if new_device_name.find(char) != -1:
                valid_name = False
        if valid_name is False:
            self.msg_if.pub_info("Received invalid device name update: " + new_device_name)
        else:
            rospy.set_param('~device_name', new_device_name)
            self.status_msg.device_name = new_device_name
        self.publishStatus(do_updates=False) # Updated inline here 
        self.device_save_config_pub.publish(Empty())


    def resetDeviceNameCb(self,msg):
        self.msg_if.pub_info("Recived update message: " + str(msg))
        self.msg_if.pub_info("Received Device Name reset msg")
        self.resetDeviceName()

    def resetDeviceName(self):
        rospy.set_param('~device_name', self.factory_device_name)
        self.status_msg.device_name = self.factory_device_name
        self.publishStatus(do_updates=False) # Updated inline here 
        self.device_save_config_pub.publish(Empty())


    def resetControlsCb(self, msg):
        self.msg_if.pub_info("Recived update message: " + str(msg))
        self.resetCb(do_updates = True)

    def resetCb(self,do_updates = True):
        rospy.set_param('~device_name', self.init_device_name)
        rospy.set_param('~controls_enable',self.init_controls_enable)
        rospy.set_param('~auto_adjust', self.init_auto_adjust)       
        rospy.set_param('~brightness', self.init_brightness_ratio)
        rospy.set_param('~contrast', self.init_contrast_ratio)        
        rospy.set_param('~thresholding', self.init_threshold_ratio)
        rospy.set_param('~resolution_ratio', self.init_resolution_ratio)   
        rospy.set_param('~framerate_ratio', self.init_framerate_ratio)
        rospy.set_param('~range_window/start_range_ratio', self.init_start_range_ratio)
        rospy.set_param('~range_window/stop_range_ratio', self.init_stop_range_ratio)
        rospy.set_param('~frame_3d_transform', self.init_frame_3d_transform)
        rospy.set_param('~frame_3d', self.init_frame_3d)
        self.zoom_ratio = self.init_zoom_ratio
        self.rotate_ratio = self.init_rotate_ratio
        self.tilt_ratio = self.init_rotate_ratio
        self.render_controls = [self.zoom_ratio,self.rotate_ratio,self.tilt_ratio]
        if do_updates:
            self.ApplyConfigUpdates()
        self.publishStatus()

    def initConfig(self):
        self.initCb(do_updates = True)

    def initCb(self,do_updates = False):
        if self.settings_if is not None:
            self.settings_if.initialize_settings(do_updates)
        self.init_device_name = rospy.get_param('~device_name', self.factory_device_name)
        self.init_controls_enable = rospy.get_param('~controls_enable',  self.factory_controls_dict["controls_enable"])
        self.init_auto_adjust = rospy.get_param('~auto_adjust',  self.factory_controls_dict["auto_adjust"])
        self.init_brightness_ratio = rospy.get_param('~brightness',  self.factory_controls_dict["brightness_ratio"])
        self.init_contrast_ratio = rospy.get_param('~contrast',  self.factory_controls_dict["contrast_ratio"])
        self.init_threshold_ratio = rospy.get_param('~thresholding',  self.factory_controls_dict["threshold_ratio"])
        self.init_resolution_ratio = rospy.get_param('~resolution_ratio',  self.factory_controls_dict["resolution_ratio"])
        self.init_framerate_ratio = rospy.get_param('~framerate_ratio',  self.factory_controls_dict["framerate_ratio"])

        self.init_start_range_ratio = rospy.get_param('~range_window/start_range_ratio',  self.factory_controls_dict["start_range_ratio"])
        self.init_stop_range_ratio = rospy.get_param('~range_window/stop_range_ratio', self.factory_controls_dict["stop_range_ratio"])
        self.init_min_range_m = rospy.get_param('~range_limits/min_range_m',  self.factory_controls_dict["min_range_m"])
        self.init_max_range_m = rospy.get_param('~range_limits/max_range_m',  self.factory_controls_dict["max_range_m"])

        self.init_frame_3d_transform = rospy.get_param('~frame_3d_transform', self.ZERO_TRANSFORM)

        if self.factory_controls_dict["frame_3d"] is None:
            self.init_frame_3d = rospy.get_param('~frame_3d',  'nepi_center_frame')
        else:
            self.init_frame_3d = rospy.get_param('~frame_3d',  self.factory_controls_dict["frame_3d"] )

        self.init_zoom_ratio = self.zoom_ratio
        self.init_rotate_ratio = self.rotate_ratio
        self.init_tilt_ratio = self.rotate_ratio
        if do_updates == True:
            self.resetCb(do_updates)


    def factoryResetCb(self):
        if self.settings_if is not None:
            self.settings_if.factory_reset_settings(update_status = False, update_params = True)
        rospy.set_param('~device_name', self.factory_device_name)
        rospy.set_param('~controls_enable',self.factory_controls_dict.get('controls_enable'))
        rospy.set_param('~auto_adjust', self.factory_controls_dict.get('auto_adjust'))  
        rospy.set_param('~brightness', self.factory_controls_dict.get('brightness_ratio'))      
        rospy.set_param('~contrast', self.factory_controls_dict.get('contrast_ratio'))
        rospy.set_param('~thresholding', self.factory_controls_dict.get('threshold_ratio'))
        rospy.set_param('~resolution_ratio', self.factory_controls_dict.get('resolution_ratio'))
        rospy.set_param('~framerate_ratio', self.factory_controls_dict.get('framerate_ratio'))
        rospy.set_param('~range_window/start_range_ratio', self.factory_controls_dict.get('start_range_ratio'))
        rospy.set_param('~range_window/stop_range_ratio', self.factory_controls_dict.get('stop_range_ratio'))
        rospy.set_param('~frame_3d_transform', self.ZERO_TRANSFORM)
        rospy.set_param('~frame_3d', self.factory_controls_dict.get('frame_3d'))
        self.zoom_ratio = self.init_zoom_ratio
        self.rotate_ratio = self.init_rotate_ratio
        self.tilt_ratio = self.init_rotate_ratio
        self.render_controls = [self.zoom_ratio,self.rotate_ratio,self.tilt_ratio]
        self.publishStatus()

    def ApplyConfigUpdates(self):
        if self.settings_if is not None:
            self.settings_if.reset_settings()
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
        if (self.setResolutionRatio is not None and 'resolution_ratio' in param_dict):
            self.setResolutionRatio(param_dict['resolution_ratio'])
        if (self.setFramerateRatio is not None and 'framerate_ratio' in param_dict):
            self.setFramerateRatio(param_dict['framerate_ratio'])
        if (self.setRange is not None and 'start_range' in param_dict and 'stop_range' in param_dict):
            self.setRange(param_dict['range_window']['start_range'], param_dict['range_window']['stop_range'])



    # Define local IDX Control callbacks
    def setControlsEnableCb(self, msg):
        self.msg_if.pub_info("Recived IDX Controls enable update message: " + str(msg))
        new_controls_enable = msg.data
        self.msg_if.pub_info("new_controls_enable")
        if self.setControlsEnable is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setControlsEnable(new_controls_enable)

        rospy.set_param('~controls_enable', new_controls_enable)
        self.status_msg.controls_enable = new_controls_enable
        self.publishStatus(do_updates=False) # Updated inline here
 

            
    def setAutoAdjustCb(self, msg):
        self.msg_if.pub_info("Recived Auto Adjust update message: " + str(msg))
        new_auto_adjust = msg.data
        if self.setAutoAdjust is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setAutoAdjust(new_auto_adjust)

        if new_auto_adjust:
            self.msg_if.pub_info("Enabling Auto Adjust")
        else:
            self.msg_if.pub_info("Disabling IDX Auto Adjust")

        rospy.set_param('~auto_adjust', new_auto_adjust)
        self.status_msg.auto_adjust = new_auto_adjust
        self.publishStatus(do_updates=False) # Updated inline here



    def setBrightnessCb(self, msg):
        self.msg_if.pub_info("Recived Brightness update message: " + str(msg))
        new_brightness = msg.data
        if rospy.get_param('~auto_adjust', self.init_auto_adjust):
            self.msg_if.pub_info("Ignoring Set Brightness request. Auto Adjust enabled")
        else:
            if self.setBrightness is not None:

                if (new_brightness < 0.0 or new_brightness > 1.0):
                    rospy.logerr("Brightness value out of bounds")
                else:
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setBrightness(new_brightness)

        rospy.set_param('~brightness', new_brightness)
        self.status_msg.brightness = new_brightness
        self.publishStatus(do_updates=False) # Updated inline here

    def setContrastCb(self, msg):
        self.msg_if.pub_info("Recived Contrast update message: " + str(msg))
        new_contrast = msg.data

        if (new_contrast < 0.0 and new_contrast != -1.0) or (new_contrast > 1.0):
            rospy.logerr("Contrast value out of bounds")
            self.publishStatus(do_updates=False) # No change
            return

        if rospy.get_param('~auto_adjust', self.init_auto_adjust):
            self.msg_if.pub_info("Ignoring Set Contrast request. Auto Adjust enabled")
        else:
            if self.setContrast is not None:
                # Call the parent's method and update ROS param as necessary
                # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                status, err_str = self.setContrast(new_contrast)

        rospy.set_param('~contrast', new_contrast)
        self.status_msg.contrast = new_contrast
        self.publishStatus(do_updates=False) # Updated inline here


    def setThresholdingCb(self, msg):
        self.msg_if.pub_info("Recived Threshold update message: " + str(msg))
        new_thresholding = msg.data

        if (new_thresholding < 0.0 or new_thresholding > 1.0):
            rospy.logerr("Thresholding value out of bounds")
            self.publishStatus(do_updates=False) # No change
            return

        if self.setThresholding is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setThresholding(new_thresholding)

        rospy.set_param('~thresholding', new_thresholding)
        self.status_msg.thresholding = new_thresholding
        self.publishStatus(do_updates=False) # Updated inline here

    def setResolutionRatioCb(self, msg):
        self.msg_if.pub_info("Recived Resolution update message: " + str(msg))
        new_resolution = msg.data

        if (new_resolution < 0.0 or new_resolution > 1.0):
            rospy.logerr("Resolution value out of bounds")
            self.publishStatus(do_updates=False) # No change
            return

        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            if self.setResolutionRatio is not None:
                status, err_str = self.setResolutionRatio(new_resolution)

        rospy.set_param('~resolution_ratio', new_resolution)
        self.status_msg.resolution_ratio = new_resolution
        self.publishStatus(do_updates=False) # Updated inline here


        
    def setFramerateRatioCb(self, msg):
        self.msg_if.pub_info("Recived Framerate update message: " + str(msg))
        new_framerate = msg.data
 
        if (new_framerate < 0.0 or new_framerate > 1.0):
            rospy.logerr("Framerate value out of bounds")
            self.publishStatus(do_updates=False) # No change
            return

        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            if self.setFramerateRatio is not None:
                status, err_str = self.setFramerateRatio(new_framerate)
           
        rospy.set_param('~framerate_ratio', new_framerate)
        self.status_msg.framerate_ratio = new_framerate
        self.status_msg.framerate_current = self.getFramerate()
        self.publishStatus(do_updates=False) # Updated inline here

 
    def setRangeCb(self, msg):
        self.msg_if.pub_info("Recived Range update message: " + str(msg))
        self.msg_if.pub_info("Recived update message: " + str(msg))
        new_start_range_ratio = msg.start_range
        new_stop_range_ratio = msg.stop_range
        if (new_start_range_ratio < 0 or new_stop_range_ratio > 1 or new_stop_range_ratio < new_start_range_ratio):
            rospy.logerr("Range values out of bounds")
            self.publishStatus(do_updates=False) # No change
            return
        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            if self.setRange is not None:
                status, err_str = self.setRange(new_start_range_ratio,new_stop_range_ratio)

        rospy.set_param('~range_window/start_range_ratio', new_start_range_ratio)
        rospy.set_param('~range_window/stop_range_ratio', new_stop_range_ratio)
        self.status_msg.range_window.start_range = new_start_range_ratio
        self.status_msg.range_window.stop_range = new_stop_range_ratio

        self.publishStatus(do_updates=False) # Updated inline here       

    def setZoomCb(self, msg):
        self.msg_if.pub_info("Recived Zoom update message: " + str(msg))
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
        self.msg_if.pub_info("Recived Rotate update message: " + str(msg))
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
        self.msg_if.pub_info("Recived 3D Transform update message: " + str(msg))
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
        self.init_frame_3d_transform = rospy.set_param('~frame_3d_transform',  transform)
        self.status_msg.frame_3d_transform = transform_msg
        self.publishStatus(do_updates=False) # Updated inline here 

    def clearFrame3dTransformCb(self, msg):
        self.msg_if.pub_info("Recived Clear 3D Transform update message: " + str(msg))
        new_transform_msg = msg
        self.clearFrame3dTransform()

    def clearFrame3dTransform(self, transform_msg):
        transform = self.ZERO_TRANSFORM
        self.init_frame_3d_transform = rospy.set_param('~frame_3d_transform',  transform)
        self.status_msg.frame_3d_transform = transform_msg
        self.publishStatus(do_updates=False) # Updated inline here 

    def setFrame3dCb(self, msg):
        self.msg_if.pub_info("Recived Set 3D Transform update message: " + str(msg))
        new_frame_3d = msg.data
        self.setFrame3d(new_frame_3d)

    def setFrame3d(self, new_frame_3d):
        rospy.set_param('~frame_3d', new_frame_3d)
        self.status_msg.frame_3d = new_frame_3d
        self.publishStatus(do_updates=False) # Updated inline here 
   
    def provide_capabilities(self, _):
        return self.capabilities_report
    
  
    # Image from img_get_function can be CV2 or ROS image.  Will be converted as needed in the thread
    def image_thread_proccess(self,data_product):
        cv2_img = None
        ros_img = None
        if data_product not in self.data_product_dict.keys():
            self.msg_if.pub_warn("Can't start data product acquisition " + data_product + " , not in data product dict")
        else:
            self.msg_if.pub_warn("Starting " + data_product + " acquisition")
            acquiring = False
            while (not rospy.is_shutdown()):

                # Get Data Product Dict and Data_IF
                dp_dict = self.data_product_dict[data_product]
                dp_get_data = dp_dict['get_data']
                dp_stop_data = dp_dict['stop_data']
                dp_if = dp_dict['data_if']
                dp_has_subs = dp_if.has_subscribers_check()
                dp_should_save = (self.save_data_if.data_product_saving_enabled(data_product) and \
                    self.save_data_if.data_product_should_save(data_product) ) or \
                    self.save_data_if.data_product_snapshot_enabled(data_product)
                #self.msg_if.pub_warn("Data product " + data_product + " has subscribers: " + str(dp_has_subs), throttle_s = 1)
                # Get Data Product Raw Dict and Data_IF
                data_product_raw = data_product + '_raw'
                if data_product_raw not in self.data_product_dict.keys():
                    dp_raw_if = None
                else:
                    dp_raw_dict = self.data_product_dict[data_product_raw]
                    dp_raw_if = dp_raw_dict['data_if']
                    dp_raw_has_subs = dp_raw_if.has_subscribers_check()
                    dp_raw_should_save = (self.save_data_if.data_product_saving_enabled(data_product_raw) and \
                        self.save_data_if.data_product_should_save(data_product_raw) ) or \
                        self.save_data_if.data_product_snapshot_enabled(data_product_raw)
                    #self.msg_if.pub_warn("Data product " + data_product_raw + " has subscribers: " + str(dp_has_subs), throttle_s = 1)


                # Get new data if any required
                has_subs = dp_has_subs or dp_raw_has_subs
                should_save = dp_should_save or dp_raw_should_save
                get_data = has_subs or should_save
                if get_data == True:
                    acquiring = True
                    if data_product != "pointcloud_image":
                        status, msg, cv2_img, ros_timestamp, encoding = dp_get_data()
                    else:
                        status, msg, cv2_img, ros_timestamp, encoding = dp_get_data(self.render_controls)
                    if (status is False):
                        #rospy.logerr_throttle(1, msg)
                        continue   
                    if cv2_img is not None:

                        # Get Image Info and Pub Status if Changed
                        cur_width = self.img_width
                        cur_height = self.img_height
                        cv2_shape = cv2_img.shape
                        self.img_width = cv2_shape[1] 
                        self.img_height = cv2_shape[0] 
                        #self.msg_if.pub_warn("Got cv2_img size: " + str(self.img_width) + ":" + str(self.img_height))
                        if cur_width != self.img_width or cur_height != self.img_height:
                            self.publishStatus()

                        #############################
                        # Process Raw Data Requirements
                        if dp_raw_if is not None:
                            if (dp_raw_has_subs == True):
                                #Publish Ros Image
                                frame_id = rospy.get_param('~frame_3d',  self.init_frame_3d )
                                dp_raw_if.publish_cv2_img(cv2_img, encoding = encoding, ros_timestamp = ros_timestamp, frame_id = 'sensor_frame')
                            if (dp_raw_should_save == True):
                                nepi_save.save_img2file(self,data_product_raw,cv2_img,ros_timestamp,save_check=False)

                        #############################
                        # Apply IDX Post Processing
                        if dp_has_subs == True or dp_should_save == True: # Don't process idx image if not required
                            cv2_img = self.applyIDXControls2Image(cv2_img,data_product)
                            if (dp_has_subs == True):
                                #Publish Ros Image
                                frame_id = rospy.get_param('~frame_3d',  self.init_frame_3d )
                                dp_if.publish_cv2_img(cv2_img, encoding = encoding, ros_timestamp = ros_timestamp, frame_id = frame_id)
                            if (dp_should_save == True):
                                nepi_save.save_img2file(self,data_product,cv2_img,ros_timestamp,save_check=False)

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition")
                        dp_stop_data()
                    acquiring = False
                else: # No subscribers and already stopped
                    acquiring = False
                    rospy.sleep(0.25)
                rospy.sleep(0.01) # Yield

   
    # Pointcloud from pointcloud_get_function can be open3D or ROS pointcloud.  Will be converted as needed in the thread
    def pointcloud_thread_proccess(self,data_product):
        o3d_pc = None
        ros_pc = None
        if data_product not in self.data_product_dict.keys():
            self.msg_if.pub_warn("Can't start data product acquisition " + data_product + " , not in data product dict")
        else:
            self.msg_if.pub_warn("Starting " + data_product + " acquisition")
            acquiring = False
            while (not rospy.is_shutdown()):
 
                # Get Data Product Dict and Data_IF
                dp_dict = self.data_product_dict[data_product]
                dp_get_data = dp_dict['get_data']
                dp_stop_data = dp_dict['stop_data']
                dp_if = dp_dict['data_if']
                dp_has_subs = dp_if.has_subscribers_check()
                dp_should_save = (self.save_data_if.data_product_saving_enabled(data_product) and \
                    self.save_data_if.data_product_should_save(data_product) ) or \
                    self.save_data_if.data_product_snapshot_enabled(data_product)
                #self.msg_if.pub_warn("Data product " + data_product + " has subscribers: " + str(dp_has_subs), throttle_s = 1)
                # Get Data Product Raw Dict and Data_IF
                data_product_raw = data_product + '_raw'
                if data_product_raw not in self.data_product_dict.keys():
                    dp_raw_if = None
                else:
                    dp_raw_dict = self.data_product_dict[data_product_raw]
                    dp_raw_if = dp_raw_dict['data_if']
                    dp_raw_has_subs = dp_raw_if.has_subscribers_check()
                    dp_raw_should_save = (self.save_data_if.data_product_saving_enabled(data_product_raw) and \
                        self.save_data_if.data_product_should_save(data_product_raw) ) or \
                        self.save_data_if.data_product_snapshot_enabled(data_product_raw)
                    #self.msg_if.pub_warn("Data product " + data_product_raw + " has subscribers: " + str(dp_raw_has_subs), throttle_s = 1)
               
                # Get data if requried
                get_data = dp_has_subs or dp_should_save
                if get_data == True:
                    acquiring = True
                    status, msg, o3d_pc, ros_timestamp, ros_frame = dp_get_data()
                    if o3d_pc is not None:

                        #############################
                        # Process Raw Data Requirements
                        if dp_raw_if is not None:
                            if (dp_raw_has_subs == True):
                                #Publish Ros Image
                                frame_id = rospy.get_param('~frame_3d',  self.init_frame_3d )
                                dp_raw_if.publish_o3d_pc.publish(o3d_pc, ros_timestamp = ros_timestamp, frame_id = ros_frame)
                            if (dp_raw_should_save == True):
                                nepi_save.save_pc2file(self,data_product_raw,cv2_img,ros_timestamp,save_check=False)


                        #********************
                        set_frame = rospy.get_param('~frame_3d',  self.init_frame_3d )
                        if set_frame == 'sensor_frame':
                            ros_frame = set_frame # else pass through sensor frame
                        else:
                            ros_frame = set_frame

                        transform = rospy.get_param('~frame_3d_transform', self.init_frame_3d_transform)
                        zero_transform = True
                        for i in range(len(transform)):
                            if transform[i] != 0:
                                zero_transform = False
                        should_transform = (zero_transform == False) and (ros_frame == 'nepi_center_frame')
                        if should_transform:   
                            o3d_pc = self.transformPointcloud(o3d_pc,transform)
                           
                        #********************
                        if (dp_has_subs == True):
                            dp_if.publish_o3d_pc.publish(o3d_pc, ros_timestamp = ros_timestamp, frame_id = ros_frame )
                        if (dp_should_save == True ):
                            nepi_save.save_pc2file(self,data_product,o3d_pc,ros_timestamp, save_check=False)
                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition")
                        dp_stop_data()
                    acquiring = False
                else: # No subscribers and already stopped
                    acquiring = False
                    rospy.sleep(0.25)
                rospy.sleep(0.01) # Yield
                




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
        self.image_thread_proccess('color_2d_image')
        
    def runBWImgThread(self):
        self.image_thread_proccess('bw_2d_image')

    def runDepthMapThread(self):
        self.image_thread_proccess('depth_map')

    def runDepthImgThread(self):
        self.image_thread_proccess('depth_image')

    def runPointcloudThread(self):
        self.pointcloud_thread_proccess('pointcloud')

    def runPointcloudImgThread(self):
        self.image_thread_proccess('pointcloud_image')


 

        
    
    # Utility Functions

    def applyIDXControls2Image(self,cv2_img,data_product):
        enabled = rospy.get_param('~controls_enable',self.init_controls_enable)
        auto = rospy.get_param('~auto_adjust', self.init_auto_adjust)       
        brightness = rospy.get_param('~brightness', self.init_brightness_ratio)
        contrast = rospy.get_param('~contrast', self.init_contrast_ratio)        
        threshold = rospy.get_param('~thresholding', self.init_threshold_ratio)
        res_ratio = rospy.get_param('~resolution_ratio', self.init_resolution_ratio)   

        if enabled == True: 
            if res_ratio > 0.99:
                [cv2_img,new_res] = nepi_img.adjust_resolution_ratio(cv2_img, res_ratio)
            if data_product != 'depth_map' and data_product != 'depth_image' and data_product != 'pointcloud_image':
                if auto is False:
                    cv2_img = nepi_img.adjust_brightness(cv2_img, brightness)
                    cv2_img = nepi_img.adjust_contrast(cv2_img, contrast)
                    cv2_img = nepi_img.adjust_sharpness(cv2_img, threshold)
                else:
                    cv2_img = nepi_img.adjust_auto(cv2_img,0.3)
        return cv2_img


    # Function to update and publish status message

    def publishStatus(self, do_updates = True):
        if do_updates is True:
            # TODO: Probably these should be queried from the parent (and through the driver) via explicit callbacks rather than via the param server
            idx_params = rospy.get_param('~')

            self.status_msg.device_name = idx_params['device_name'] if 'device_name' in idx_params else self.init_device_name
            self.status_msg.sensor_name = self.sensor_name
            self.status_msg.identifier = self.identifier
            self.status_msg.serial_num = self.serial_num
            self.status_msg.hw_version = self.hw_version
            self.status_msg.sw_version = self.sw_version
            
            rtsp_url = ""
            rtsp_username = ""
            rtsp_password = ""
            if self.get_rtsp_url is not None:
                [rtsp_url,rtsp_username,rtsp_password] = self.get_rtsp_url()
                if rtsp_url is None:
                    rtsp_url = ""
            self.status_msg.rtsp_url = rtsp_url
            self.status_msg.rtsp_username = rtsp_username
            self.status_msg.rtsp_password = rtsp_password

            self.status_msg.controls_enable = idx_params['controls_enable'] if 'controls_enable' in idx_params else True
            self.status_msg.auto_adjust = idx_params['auto_adjust'] if 'auto_adjust' in idx_params else False
            
            self.status_msg.resolution_ratio = idx_params['resolution_ratio'] if 'resolution_ratio' in idx_params else 0
            res_str = str(self.img_width) + ":" + str(self.img_height)
            self.status_msg.resolution_current = res_str

            self.status_msg.framerate_ratio = idx_params['framerate_ratio'] if 'framerate_ratio' in idx_params else 0
            fr = 0.0
            if self.getFramerate is not None:
                fr =  self.getFramerate()
            self.status_msg.framerate_current = fr
            self.status_msg.contrast = idx_params['contrast'] if 'contrast' in idx_params else 0
            self.status_msg.brightness = idx_params['brightness'] if 'brightness' in idx_params else 0
            self.status_msg.thresholding = idx_params['thresholding'] if 'thresholding' in idx_params else 0
            
            self.status_msg.range_window.start_range = rospy.get_param('~range_window/start_range_ratio', 0.0)
            self.status_msg.range_window.stop_range =  rospy.get_param('~range_window/stop_range_ratio', 1.0)
            self.status_msg.min_range_m = rospy.get_param('~range_limits/min_range_m',0.0)
            self.status_msg.max_range_m = rospy.get_param('~range_limits/max_range_m',1.0)
            # The transfer frame into which 3D data (pointclouds) are transformed for the pointcloud data topic
            transform = rospy.get_param('~frame_3d_transform',  self.ZERO_TRANSFORM)
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
    
