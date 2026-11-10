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
import threading
import subprocess
import numpy as np

import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
# from nepi_sdk import nepi_system
# from nepi_sdk import nepi_pc
# from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav
# from nepi_sdk import nepi_devices

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image, PointCloud2

from nepi_interfaces.msg import DeviceIDXStatus, RangeWindow



from nepi_interfaces.srv import DeviceIDXStatusQuery, DeviceIDXStatusQueryRequest, DeviceIDXStatusQueryResponse
from nepi_interfaces.msg import ImageStatus, PointcloudStatus, NavPose



from nepi_api.data_product_if_image import ColorImageIF
from nepi_api.data_product_if_depth_map import DepthMapIF
from nepi_api.data_product_if_pointcloud import PointcloudIF

from nepi_api.device_if import DeviceIF


SUPPORTED_DATA_PRODUCTS = ['color_image','bw_image',
                            'intensity_map','depth_map','pointcloud']

PERSPECTIVE_OPTIONS = ['pov','top']






class IDXDeviceIF(DeviceIF):

    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = DeviceIDXStatus()
    DATA_MSG = None

    ID = 'idx'
    DESCRIPTION = 'Imaging Device'

    DATA_SOURCE_DESCRIPTION = 'Sensor'
    DATA_REF_DESCRIPTION = 'Sensor Lense Center'

    #Factory Control Values 
    FACTORY_CONTROLS_DICT = dict( controls_enable = True,
    auto_adjust_ebabled = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.0,
    resolution_ratio = 1.0, 
    max_framerate = 10, 
    start_range_ratio = 0.0,
    stop_range_ratio = 1.0,
    min_range_m = 0.0,
    max_range_m = 1.0,
    width_deg = 110,
    height_deg = 70,    
    zoom_ratio = 0.5, 
    rotate_ratio = 0.5,
    )

    #########################
    ## Managed by NodeIF
    #########################

    # id = ''
    # name = ''
    # namespace = '~'

    # status_msg = None
    # data_msg = None

    # msg_if = None
    # node_if = None
    # settings_if = None
    # save_data_if = None
    # transform_if = None
    # navpose_if = None    
    
    # callback_dict = dict()

    # active_topics = []
    # active_topic_types = []
    # active_services = []

    # caps_dict = dict()

    #########################
    ## Managed by DataProductIF
    #########################
    # SYSTEM_IF_DICT = dict()

    # data_source_description = 'sensor'
    # data_ref_description = 'sensor'

    # device_name = ''
    # path = ''
    # serial_num = ''
    # hw_version = ''
    # sw_version = ''

    # factory_controls_dict = dict()


    # msg_if = None
    # node_if = None
    # settings_if = None
    # save_data_if = None
    # transform_if = None
    # navpose_if = None

    # color_image_if = None
    # depthmap_if = None
    # pointcloud_if = None

    #########################
    ## CUSTOM
    #########################
    ready = False
    
    color_image_if = None
    depthmap_if = None
    pointcloud_if = None

    image_topic = ""
    depth_map_topic = ""
    pointcloud_topic = ""

    # Default Global Values
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]
    UPDATE_NAVPOSE_RATE_HZ = 10

    DEFUALT_IMG_WIDTH_DEG = 100
    DEFUALT_IMG_HEIGHT_DEG = 70

    # Per-camera 3D mount transform (metadata: where the camera is located/oriented)


    # Transformed-navpose publisher (reference frame navpose + mount transform applied)
    ref_navpose_sub = None
    ref_navpose_frame = None

    namespace_npx = ''
    npx_if = None
    device_info_dict = dict()
    navpose_update_rate = 10
    max_navpose_update_rate = 10

    

    auto_adjust_controls = []
    auto_adjust_ebabled = False
    brightness_ratio = 0.5
    contrast_ratio = 0.5
    threshold_ratio = 0.0
    resolution_ratio = 1.0  
    max_framerate = 10
    start_range_ratio = 0.0
    stop_range_ratio = 1.0

    min_range_m = 0.0
    max_range_m = 1.0

    data_products_list = []

    data_products_dict = dict()

    update_navpose_interval_sec = float(1)/UPDATE_NAVPOSE_RATE_HZ
    last_gps_timestamp = None
    last_odom_timestamp = None
    last_heading_timestamp = None

   
    rtsp_url = None

    width_px = 1
    height_px = 1

    perspective = 'pov'

    width_deg = DEFUALT_IMG_WIDTH_DEG
    height_deg = DEFUALT_IMG_HEIGHT_DEG
    aspect_ratio_deg = -999

    
    fps_queue = dict()
    current_fps = dict()
    last_data_time = dict()

    image_thread = None
    depth_map_thread = None
    pointcloud_thread = None


    device_disabled = False

    start_time = 0

    #######################
    ### IF Initialization
    def __init__(self, 
                device_info, 
                getSettingsFunction=None, setSettingFunction=None, 
                factoryControls = None, 
                data_source_description = None,
                data_ref_description = None,
                getFOV=None, perspective = 'pov',
                get_rtsp_url = None,
                setResolutionRatio=None, setMaxFramerate=None,
                setContrastRatio=None, setBrightnessRatio=None, 
                setThresholdingRatio=None, setRangeRatio=None, 
                setAutoAdjustRatio=None, autoAdjustControls=[],
                getFramerate=None,
                getColorImage=None, stopColorImageAcquisition=None, 
                getDepthMap=None, stopDepthMapAcquisition=None, 
                getPointcloud=None, stopPointcloudAcquisition=None, 
                data_products =  [],
                msg_if = None,
                node_if = None,
                settings_if = None,
                save_data_if = None,
                connect_navpose_if = None,
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.caps_dict = copy.deepcopy(self.caps_dict)
        self.data_products_dict = copy.deepcopy(self.data_products_dict)
        self.fps_queue = copy.deepcopy(self.fps_queue)
        self.current_fps = copy.deepcopy(self.current_fps)
        self.last_data_time = copy.deepcopy(self.last_data_time)


        ##############################
        # Update Class Variables
        ##############################

        data_products_list = []
        for data_product in data_products:
            #self.msg_if.pub_debug("Checking data product: " + str(data_product))
            if data_product in SUPPORTED_DATA_PRODUCTS:
                data_products_list.append(data_product)
        self.data_products_list = data_products_list




        ## Set None Capabilities Variables

        self.setMaxFramerate = setMaxFramerate
        if self.setMaxFramerate is not None:
            self.caps_dict['has_framerate'] = True
        
        self.getFramerate = getFramerate

        self.setRangeRatio = setRangeRatio
        if self.setRangeRatio is not None:
            self.caps_dict['has_range'] = True


        self.setAutoAdjustRatio = setAutoAdjustRatio
        if self.setAutoAdjustRatio is not None:
            self.caps_dict['has_auto_adjust'] = True
        self.auto_adjust_controls = autoAdjustControls

        self.setBrightnessRatio = setBrightnessRatio
        if self.setBrightnessRatio is not None:
            self.caps_dict['has_brightness'] = True

        self.setContrastRatio = setContrastRatio
        if self.setContrastRatio is not None:
            self.caps_dict['has_contrast'] = True

        self.setThresholdingRatio = setThresholdingRatio       
        if self.setThresholdingRatio is not None:
            self.caps_dict['has_threshold'] = True

        self.setResolutionRatio = setResolutionRatio
        if self.setResolutionRatio is not None:
            self.caps_dict['has_resolution'] = True


        ##############################  
        # Setup NodeIF Updates
        ##############################  

        ##################
        update_info_dict = dict()
        ##################        
        update_caps_dict = self.caps_dict
        ##################        
        update_config_control_dict = dict()
        ##################       
        update_node_controls_dict = dict()
        ##################
        update_stats_dict = dict()
        ##################
        update_rui_dict = dict()
        ##################
        update_callback_dict = dict()

        ###########
        # NODE IF SETUP

        # Call the parent class constructor
        super().__init__(                
                device_info = device_info, 
                getSettingsFunction=getSettingsFunction,
                setSettingFunction=setSettingFunction, 
                factoryControls = factoryControls, 
                data_source_description = data_source_description,
                data_ref_description = data_ref_description,
                info_dict = update_info_dict,
                caps_dict = update_caps_dict, 
                stats_dict = update_stats_dict, 
                rui_dict = update_rui_dict, 
                config_control_dict = update_config_control_dict, 
                node_controls_dict = update_node_controls_dict, 
                callback_dict = update_callback_dict,              
                msg_if = msg_if,
                node_if = node_if,
                settings_if = settings_if,
                save_data_if = save_data_if,
                connect_navpose_if = connect_navpose_if
                )

        ready = self.wait_for_ready()
        ######################################################
        
        self.msg_if.pub_debug("Using Factory Controls: " + str(self.factory_controls_dict))
        self.auto_adjust_ebabled = self.factory_controls_dict['auto_adjust_ebabled']
        self.brightness_ratio = self.factory_controls_dict['brightness_ratio']
        self.contrast_ratio = self.factory_controls_dict['contrast_ratio']
        self.threshold_ratio = self.factory_controls_dict['threshold_ratio']
        self.resolution_ratio = self.factory_controls_dict['resolution_ratio']
        self.max_framerate = self.factory_controls_dict['max_framerate']


        self.min_range_m = self.factory_controls_dict['min_range_m']
        self.max_range_m = self.factory_controls_dict['max_range_m']
        self.msg_if.pub_debug("Set Min Max Ranges: " + str([self.min_range_m,self.max_range_m]))
        self.width_deg = self.factory_controls_dict['width_deg']
        self.height_deg = self.factory_controls_dict['height_deg']

        self.start_range_ratio = self.factory_controls_dict['start_range_ratio']
        self.stop_range_ratio = self.factory_controls_dict['stop_range_ratio']

        
        # Set up standard IDX parameters with ROS param and subscriptions
        # Defer actually setting these on the camera via the parent callbacks... the parent may need to do some 
        # additional setup/calculation first. Parent can then get these all applied by calling ApplyConfigUpdates()

        self.getFOV = getFOV
        if self.getFOV is not None:
            try:
                [self.width_deg,self.height_deg] = self.getFOV()
            except:
                self.getFOV = None
        self.perspective = perspective

        self.get_rtsp_url = get_rtsp_url

          


        #######################
        # Update NodeIF NodeClass
        #######################


        # Params Config Dict ####################
        params_dict = {
            'width_deg': {
                'namespace': self.namespace,
                'factory_val': self.width_deg
            },
            'height_deg': {
                'namespace': self.namespace,
                'factory_val': self.height_deg
            },
            'aspect_ratio_deg': {
                'namespace': self.namespace,
                'factory_val': self.aspect_ratio_deg
            },
            'auto_adjust_ebabled': {
                'namespace': self.namespace,
                'factory_val': self.auto_adjust_ebabled
            },
            'brightness_ratio': {
                'namespace': self.namespace,
                'factory_val': self.brightness_ratio
            },
            'contrast_ratio': {
                'namespace': self.namespace,
                'factory_val': self.contrast_ratio
            },
            'threshold_ratio': {
                'namespace': self.namespace,
                'factory_val': self.threshold_ratio
            },
            'resolution_ratio': {
                'namespace': self.namespace,
                'factory_val': self.resolution_ratio
            },
            'max_framerate': {
                'namespace': self.namespace,
                'factory_val': self.max_framerate
            },
            'start_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.start_range_ratio
            },
            'stop_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.stop_range_ratio
            }


        }




        # Services Config Dict ####################

        services_dict = {

            'status_query': {
                'namespace': self.namespace,
                'topic': 'status_query',
                'srv': DeviceIDXStatusQuery,
                'req': DeviceIDXStatusQueryRequest(),
                'resp': DeviceIDXStatusQueryResponse(),
                'callback': self.status_query_callback
            }
        }


        # Status publishes through the NodeIF status pub, like LSX and NPX
        pubs_dict = {}
                            

        # Subscribers Config Dict ####################
        subs_dict = {
            'disable': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'disable',
                'qsize': 5,
                'callback': self._disableCb, 
                'callback_args': ()
            }, 
            'set_width_deg': {
                'namespace': self.namespace,
                'topic': 'set_width_deg',
                'msg': Int32,
                'qsize': 1,
                'callback': self.setWidthDegCb, 
                'callback_args': ()
            },
            'set_height_deg': {
                'namespace': self.namespace,
                'topic': 'set_height_deg',
                'msg': Int32,
                'qsize': 1,
                'callback': self.setHeightDegCb, 
                'callback_args': ()
            },
            'set_auto_adjust': {
                'namespace': self.namespace,
                'topic': 'set_auto_adjust_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setAutoAdjustEnableCb, 
                'callback_args': ()
            },
            'set_brightness': {
                'namespace': self.namespace,
                'topic': 'set_brightness_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setBrightnessRatioCb, 
                'callback_args': ()
            },
            'set_contrast': {
                'namespace': self.namespace,
                'topic': 'set_contrast_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setContrastRatioCb, 
                'callback_args': ()
            },
            'set_threshold': {
                'namespace': self.namespace,
                'topic': 'set_threshold_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setThresholdingRatioCb, 
                'callback_args': ()
            },
            'set_resolution_ratio': {
                'namespace': self.namespace,
                'topic': 'set_resolution_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setResolutionRatioCb, 
                'callback_args': ()
            },
            'set_max_framerate': {
                'namespace': self.namespace,
                'topic': 'set_max_framerate',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setMaxFramerateCb, 
                'callback_args': ()
            },
            'set_range_window': {
                'namespace': self.namespace,
                'topic': 'set_range_window',
                'msg': RangeWindow,
                'qsize': 1,
                'callback': self.setRangeRatioCb, 
                'callback_args': ()
            },
            'reset_controls': {
                'namespace': self.namespace,
                'topic': 'reset_controls',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetControlsCb, 
                'callback_args': ()
            }
            
        }

        #######################
        self.add_params(params_dict)
        self.add_services(services_dict)
        self.add_pubs(pubs_dict)
        self.add_subs(subs_dict)
        nepi_sdk.sleep(0.1)

        ######################################################


        #########################
        # Update Status Msg
        ##############################
        self.status_msg.device_name = self.device_name
        self.status_msg.device_path = self.path
        self.status_msg.device_node_name = self.node_name
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version

        ##############################
        # Start Node Processes
        ##############################



        # Start the data producers
        if (getColorImage is not None and 'color_image' in self.data_products_list):
            self.getColorImage = getColorImage
            self.stopColorImageAcquisition = stopColorImageAcquisition
            data_product = 'color_image'

            start_data_function = self.getColorImage
            stop_data_function = self.stopColorImageAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self._addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_debug("Starting " + data_product + " acquisition thread" )
            self.image_thread = threading.Thread(target=self.runImageThread)
            self.image_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.caps_dict['has_color_image'] = True
        else:
            self.caps_dict['has_color_image'] = False
        
        self.caps_dict['has_depth_map'] = False
        self.caps_dict['has_pointcloud'] = False

        if (getDepthMap is not None and 'depth_map' in self.data_products_list):
            self.getDepthMap = getDepthMap
            self.stopDepthMapAcquisition = stopDepthMapAcquisition
            data_product = 'depth_map'
            
            start_data_function = self.getDepthMap
            stop_data_function = self.stopDepthMapAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self._addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_debug("Starting " + data_product + " acquisition thread" )
            self.depth_map_thread = threading.Thread(target=self.runDepthMapThread)
            self.depth_map_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.caps_dict['has_depth_map'] = True
        else:
            self.caps_dict['has_depth_map'] = False

        if (getPointcloud is not None and 'pointcloud' in self.data_products_list):
            self.getPointcloud = getPointcloud
            self.stopPointcloudAcquisition = stopPointcloudAcquisition
            data_product = 'pointcloud'

            start_data_function = self.getPointcloud
            stop_data_function = self.stopPointcloudAcquisition
            data_msg = PointCloud2
            data_status_msg = PointcloudStatus

            success = self._addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_debug("Starting " + data_product + " acquisition thread" )
            self.pointcloud_thread = threading.Thread(target=self.runPointcloudThread)
            self.pointcloud_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.caps_dict['has_pointcloud'] = True
        else:
            self.caps_dict['has_pointcloud'] = False


        
        ############################
        # Start Data Get Threads


        for data_product in self.data_products_list:
            self.last_data_time[data_product] = nepi_utils.get_time()
            self.current_fps[data_product] = 0
            self.fps_queue[data_product] = [0 for _ in range(100)]

        # Launch the acquisition and saving threads
        if self.image_thread is not None:
            self.image_thread.start()

        if self.depth_map_thread is not None:
            self.depth_map_thread.start()

        if self.pointcloud_thread is not None:
            self.pointcloud_thread.start()

        nepi_sdk.sleep(1)


   
    
        ####################################
        self.ready = True
        start_delay = nepi_utils.get_time() - self.start_time
        self.msg_if.pub_info("Initialization time: " + str(start_delay) )
        self.msg_if.pub_info("Initialization Complete: "+ ": " + self.node_name, color = 'GREEN' )
        ####################################

        ##############################
        # Start NodeIF Processes
        if type(self) is IDXDeviceIF:
            self.start()


    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):
        """Builds and publishes the DeviceIDXStatus message on the status topic.

        Populates all fields of the status message from current instance state,
        including device identity, FOV angles, resolution, framerate, image
        control ratios, range-window settings, and per-data-product FPS. When
        ``do_updates`` is True, also queries the RTSP URL callback (if
        registered) and includes the result in the message.

        Args:
            do_updates (bool, optional): When True, perform live queries (e.g.
                RTSP URL) before publishing. Set to False when the caller has
                already updated state inline and a lightweight publish is
                sufficient. Defaults to True.
        """
        status_msg.device_name = self.device_name

        status_msg.device_disabled = self.device_disabled

        #############################################


        # aspect_ratio_deg = self.width_px / self.height_px
        # if self.aspect_ratio_deg == -999 or self.aspect_ratio_deg < 0.5 or self.aspect_ratio_deg > 2.5:
        #     self.aspect_ratio_deg  = round(self.width_px / self.height_px, 2)
        
        cur_aspect_ratio = self.width_px / self.height_px
        if cur_aspect_ratio < 0.5 or cur_aspect_ratio > 2.5:
            cur_aspect_ratio = 1        

        width_deg = self.width_deg / self.aspect_ratio_deg * cur_aspect_ratio
        height_deg = self.height_deg

        status_msg.width_deg = int(width_deg)
        status_msg.height_deg = int(height_deg)
        status_msg.aspect_ratio_deg = round(cur_aspect_ratio,2)

        status_msg.perspective = self.perspective
        
        status_msg.resolution_ratio = self.resolution_ratio
        res_str = str(self.width_px) + ":" + str(self.height_px)
        status_msg.resolution_current = res_str



        status_msg.max_framerate = self.max_framerate
        data_products = self.data_products_list
        framerates = []
        for dp in data_products:
            if dp in self.current_fps.keys():
                framerates.append(self.current_fps[dp])
            else:
                framerates.append(0.0)
        
        status_msg.framerates = framerates


        status_msg.auto_adjust_enabled = self.auto_adjust_ebabled
        status_msg.auto_adjust_controls = self.auto_adjust_controls
        status_msg.contrast_ratio = self.contrast_ratio
        status_msg.brightness_ratio = self.brightness_ratio
        status_msg.threshold_ratio = self.threshold_ratio
        
        status_msg.range_window_ratios.start_range = self.start_range_ratio
        status_msg.range_window_ratios.stop_range =  self.stop_range_ratio

        status_msg.min_range_m = self.min_range_m
        status_msg.max_range_m = self.max_range_m

        delta_range = self.max_range_m - self.min_range_m
        status_msg.min_range_m_adj = self.min_range_m + delta_range * self.start_range_ratio
        status_msg.max_range_m_adj = self.min_range_m + delta_range * self.stop_range_ratio

        status_msg.data_products = self.data_products_list

        # Need to move to a seperate updater process
        # rtsp_url = ""
        # rtsp_username = ""
        # rtsp_password = ""
        # if self.get_rtsp_url is not None:
        #     [rtsp_url,rtsp_username,rtsp_password] = self.get_rtsp_url()
        #     if rtsp_url is None:
        #         rtsp_url = ""
        # status_msg.rtsp_url = rtsp_url
        # status_msg.rtsp_username = rtsp_username
        # status_msg.rtsp_password = rtsp_password
       
        status_msg.navpose_msg = self.get_navpose_msg()

        ###################
        # TOPIC UPDATES
        
        ########
        # NAVPOSE DATA
        try:
           navpose_topic = self.navpose_if.get_namespace()
        except:
           navpose_topic = ''
        status_msg.navpose_topic = navpose_topic


        try:
           navpose_msg = self.navpose_if.get_navpose_msg()
           status_msg.navpose_msg = navpose_msg
        except:
           pass

        try:
           connect_navpose_topic = self.connect_navpose_if.get_namespace()
        except:
           connect_navpose_topic = ''
        status_msg.connect_navpose_topic = connect_navpose_topic
        
        return status_msg



    def initCb(self,do_updates = False):
      if self.node_if is not None:
            self.width_deg = self.node_if.get_param(self.node_if_prefix + 'width_deg')
            self.height_deg = self.node_if.get_param(self.node_if_prefix + 'height_deg')  
            aspect_ratio_deg = self.node_if.get_param(self.node_if_prefix + 'aspect_ratio_deg')
            if aspect_ratio_deg is not None:
                self.aspect_ratio_deg = round(aspect_ratio_deg,2)
            self.msg_if.pub_debug("Init degs w,h,ar:: " + str([self.width_deg,self.height_deg,self.aspect_ratio_deg]))

            self.resolution_ratio = self.node_if.get_param(self.node_if_prefix + 'resolution_ratio')
            max_framerate = self.node_if.get_param(self.node_if_prefix + 'max_framerate') 
            if max_framerate is not None:
                self.max_framerate = max_framerate
            self.auto_adjust_ebabled = self.node_if.get_param(self.node_if_prefix + 'auto_adjust_ebabled') 
            self.brightness_ratio = self.node_if.get_param(self.node_if_prefix + 'brightness_ratio')
            self.contrast_ratio = self.node_if.get_param(self.node_if_prefix + 'contrast_ratio')        
            self.threshold_ratio = self.node_if.get_param(self.node_if_prefix + 'threshold_ratio')  
            self.msg_if.pub_debug("Starting range ratios:: " + str([self.start_range_ratio,self.stop_range_ratio]))
            start_range_ratio = self.node_if.get_param(self.node_if_prefix + 'start_range_ratio')
            stop_range_ratio = self.node_if.get_param(self.node_if_prefix + 'stop_range_ratio')
            if start_range_ratio is not None and stop_range_ratio is not None:
                self.start_range_ratio = start_range_ratio
                self.stop_range_ratio = stop_range_ratio
            else:
                self.start_range_ratio = 0
                self.stop_range_ratio = 1
                self.node_if.set_param(self.node_if_prefix + 'start_range_ratio',self.start_range_ratio)
                stop_range_ratio = self.node_if.set_param(self.node_if_prefix + 'stop_range_ratio',self.stop_range_ratio)
            self.msg_if.pub_debug("Updated range ratios:: " + str([self.start_range_ratio,self.stop_range_ratio]))


            self.pt_mounted = self.node_if.get_param(self.node_if_prefix + 'pt_mounted')
            self.pt_topic = self.node_if.get_param(self.node_if_prefix + 'pt_topic')
            self.node_if.save_config()

      if do_updates == True and self.node_if is not None:
        pass        
      self.publish_status()

    def resetCb(self,do_updates = True):
      self.msg_if.pub_debug("Resetting Configs")
      if self.node_if is not None:
        self.node_if.reset_params()
        if self.getFOV is not None:
            try:
                [self.width_deg,self.height_deg] = self.getFOV()
                self.aspect_ratio_deg = -999
                self.msg_if.pub_debug("Updated degs w,h,ar:: " + str([self.width_deg,self.height_deg,self.aspect_ratio_deg]))
                if self.node_if is not None:
                    self.node_if.set_param(self.node_if_prefix + 'width_deg',self.width_deg)
                    self.node_if.set_param(self.node_if_prefix + 'height_deg',self.height_deg) 
                    self.node_if.set_param(self.node_if_prefix + 'aspect_ratio_deg',-999)
            except:
                pass
      if do_updates == True:
        pass
      self.initCb(do_updates = True)

    def factoryResetCb(self,do_updates = True):
      if self.node_if is not None:
        self.node_if.factory_reset_params()
        if self.getFOV is not None:
            try:
                [self.width_deg,self.height_deg] = self.getFOV()
                self.aspect_ratio_deg = -999
                self.msg_if.pub_debug("Updated degs w,h,ar:: " + str([self.width_deg,self.height_deg,self.aspect_ratio_deg]))
                if self.node_if is not None:
                    self.node_if.set_param(self.node_if_prefix + 'width_deg',self.width_deg)
                    self.node_if.set_param(self.node_if_prefix + 'height_deg',self.height_deg) 
                    self.node_if.set_param(self.node_if_prefix + 'aspect_ratio_deg',-999)
                    
            except:
                pass

      if do_updates == True:
        pass
      self.initCb(do_updates = True)



    ###############################
    # Class Methods
    ###############################        
 
    def runImageThread(self):
        self.image_thread_proccess('color_image')

    def runDepthMapThread(self):
        self.depth_map_thread_proccess('depth_map')
        #pass

    def runPointcloudThread(self):
        self.pointcloud_thread_proccess('pointcloud')


    def status_query_callback(self, _):
        """Handles a ROS service request for device capabilities.

        Refreshes the data-products list in the status message before
        returning it. The status reflects which optional capabilities
        (framerate, range, auto-adjust, brightness, contrast, threshold,
        resolution, color image, depth map, pointcloud) were provided at
        construction time.

        Args:
            _ (DeviceIDXStatusQueryRequest): Unused service request object.

        Returns:
            DeviceIDXStatusQueryResponse: The current DeviceIDXStatus in
                device_status.
        """
        return DeviceIDXStatusQueryResponse(device_status = self.status_msg)




    def _addDataProduct2Dict(self,data_product, start_data_function,stop_data_function,data_msg,data_status_msg):
        success = False
        data_product = data_product
        dp_dict = dict()
        dp_dict['data_product'] = data_product
        dp_dict['image_topic'] = 'None'
        

        dp_dict['get_data'] = start_data_function
        dp_dict['stop_data'] = stop_data_function

        self.data_products_dict[data_product] = dp_dict

        # do wait here for all
        success = True
        return success

    def _disableCb(self, msg):
        #self.msg_if.pub_info("Recieved Disable Update: " + str(msg) )
        enabled = msg.data
        self.disable(enabled)

    def disable(self, enabled):
        """Enable or disable data for all registered data products.

        Args:
            enabled (bool): True to enable data, False to disable.
        """
        self.msg_if.pub_debug("Setting Saving Disabled to: " + str(enabled))  
        self.device_disabled = enabled
        self.publish_status()   



    def setWidthDegCb(self, msg):
        #self.msg_if.pub_info("Recived Width Deg update message: " + str(msg))
        width_deg = msg.data
        if width_deg > 10:
            self.aspect_ratio_deg = round(self.width_px / self.height_px, 2)
            self.width_deg = width_deg     
            self.msg_if.pub_debug("Setting degs w,h,ar:: " + str([self.width_deg,self.height_deg,self.aspect_ratio_deg]))
            self.publish_status() # Updated inline here   
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix + 'width_deg', width_deg)
                self.node_if.set_param(self.node_if_prefix + 'aspect_ratio_deg',self.aspect_ratio_deg)                


 
    def setHeightDegCb(self, msg):
        #self.msg_if.pub_info("Recived Height Deg update message: " + str(msg))
        height_deg = msg.data
        if height_deg > 10:
            self.aspect_ratio_deg = round(self.width_px / self.height_px, 2)
            self.height_deg = height_deg   
            self.msg_if.pub_debug("Setting degs w,h,ar:: " + str([self.width_deg,self.height_deg,self.aspect_ratio_deg, ]))
            self.publish_status() # Updated inline here   
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix + 'height_deg', height_deg)
                self.node_if.set_param(self.node_if_prefix + 'aspect_ratio_deg',self.aspect_ratio_deg)
            
    def setAutoAdjustEnableCb(self, msg):
        #self.msg_if.pub_info("Recived Auto Adjust update message: " + str(msg))
        enabled = msg.data
        if self.setAutoAdjustRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setAutoAdjustRatio(enabled)

        if enabled:
            self.msg_if.pub_info("Enabling Auto Adjust" )
        else:
            self.msg_if.pub_info("Disabling IDX Auto Adjust" )

        self.auto_adjust_ebabled = enabled       
        self.publish_status() # Updated inline here
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'auto_adjust_ebabled', enabled)




    def setBrightnessRatioCb(self, msg):
        #self.msg_if.pub_info("Recived Brightness update message: " + str(msg))
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.brightness_ratio = ratio
        self.publish_status() # Updated inline here
        if self.setBrightnessRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setBrightnessRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'brightness_ratio', ratio)


    def setContrastRatioCb(self, msg):
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.contrast_ratio = ratio
        self.publish_status() # Updated inline here
        if self.setContrastRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setContrastRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'contrast_ratio', ratio)
        


    def setThresholdingRatioCb(self, msg):
        self.msg_if.pub_info("Received Threshold update message: " + str(msg))
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.threshold_ratio = ratio
        self.publish_status() # Updated inline here
        if self.setThresholdingRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setThresholdingRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'threshold_ratio', ratio)
        

    def setResolutionRatioCb(self, msg):
        #self.msg_if.pub_info("Recived Resolution update message: " + str(msg))
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.resolution_ratio = ratio
        self.publish_status() # Updated inline here
        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        if self.setResolutionRatio is not None:
            status, err_str = self.setResolutionRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'resolution_ratio', ratio)
        


        
    def setMaxFramerateCb(self, msg):
        #self.msg_if.pub_info("Recived Max Framerate update message: " + str(msg))
        rate = msg.data
 
        if rate < 1:
            rate = 1
        if rate > 100:
            rate = 100

        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        self.max_framerate = rate
        self.publish_status() # Updated inline here

        if self.setMaxFramerate is not None:
            #self.msg_if.pub_debug("Sending update framerate ratio to driver: ")
            status, err_str = self.setMaxFramerate(rate)
            #self.msg_if.pub_debug("Recived Framerate update: " + str(status))


        for data_product in self.data_products_list:
            self.fps_queue[data_product] = [0 for _ in range(100)]

        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'max_framerate', rate)


 
    def setRangeRatioCb(self, msg):
        #self.msg_if.pub_info("Recived Range update message: " + str(msg))
        #self.msg_if.pub_info("Recived update message: " + str(msg))
        new_start_range_ratio = msg.start_range
        new_stop_range_ratio = msg.stop_range
        if (new_start_range_ratio < 0 or new_stop_range_ratio > 1 or new_stop_range_ratio < new_start_range_ratio):
            self.msg_if.pub_error("Range values out of bounds" )
            self.publish_status() # No change
            return
        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            if self.setRangeRatio is not None:
                status, err_str = self.setRangeRatio(new_start_range_ratio,new_stop_range_ratio)

        self.start_range_ratio = new_start_range_ratio
        self.stop_range_ratio = new_stop_range_ratio
        self.publish_status() # Updated inline here  
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'start_range_ratio', new_start_range_ratio)
            self.node_if.set_param(self.node_if_prefix + 'stop_range_ratio', new_stop_range_ratio)
     
  

    def resetControlsCb(self, msg):
        #self.msg_if.pub_info("Recived reset controls message: " + str(msg))
        self.node_if.reset_param(self.node_if_prefix + 'controls_enable')
        self.node_if.reset_param(self.node_if_prefix + 'auto_adjust_ebabled')       
        self.node_if.reset_param(self.node_if_prefix + 'brightness_ratio')
        self.node_if.reset_param(self.node_if_prefix + 'contrast_ratio')        
        self.node_if.reset_param(self.node_if_prefix + 'threshold_ratio')
        self.node_if.reset_param(self.node_if_prefix + 'resolution_ratio')   
        self.node_if.reset_param(self.node_if_prefix + 'max_framerate')
        self.node_if.reset_param(self.node_if_prefix + 'start_range_ratio')
        self.node_if.reset_param(self.node_if_prefix + 'stop_range_ratio')
        self.ApplyConfigUpdates()



  
    def update_fps(self, data_product):
        """Calculates and updates FPS statistics for a data product using a rolling window.

        Records the current timestamp, computes instantaneous FPS from the
        interval since the last frame, and maintains a 100-sample rolling
        queue for averaging. Publishes a status update when the averaged FPS
        changes by more than 1 Hz or is still settling.

        Args:
            data_product (str): Key identifying the data product whose FPS
                should be updated (e.g. ``'color_image'``, ``'depth_map'``).
        """
        last_data_time = copy.deepcopy(self.last_data_time[data_product])
        self.last_data_time[data_product] = nepi_utils.get_time()
        last_fps = copy.deepcopy(self.current_fps[data_product])
        if last_data_time is not None:
            f_time = (self.last_data_time[data_product] - last_data_time)
            current_fps = float(1) / f_time
            #self.msg_if.pub_debug("Got " + data_product + " time and fps: " + str(round(f_time,3)) + " : " + str(round(current_fps,2)), throttle_s = 2)
            self.fps_queue[data_product].pop(0)
            self.fps_queue[data_product].append(current_fps)
            #self.msg_if.pub_debug("fps queue " + str(self.fps_queue[data_product]), throttle_s = 2)
            fps_queue = [x for x in self.fps_queue[data_product] if x != 0]
            if len(fps_queue) > 1:
                self.current_fps[data_product] = round(sum(fps_queue)/len(fps_queue),0)
            fps_changed = abs(self.current_fps[data_product] - last_fps) > 1
            fps_changing = 0 in self.fps_queue[data_product]
            if fps_changed or fps_changing:
                self.publish_status()
  

    # Image from img_get_function can be CV2 or ROS image.  Will be converted as needed in the thread
    def image_thread_proccess(self, data_product):
        """Continuously acquires, processes, and publishes image data for a given data product.

        Waits for the save-data interface to be ready, then enters a loop that
        checks whether any subscribers need data. When data is needed, calls the
        registered acquisition function and publishes the resulting CV2 image via
        the appropriate data interface (e.g. ``ColorImageIF``). Stops acquisition
        and resets FPS counters when no subscribers are active.

        Args:
            data_product (str): Key identifying the image data product to run
                (e.g. ``'color_image'``). Must be present in
                ``self.data_products_dict``.
        """
        cv2_img = None

        if data_product not in self.data_products_dict.keys():
            self.msg_if.pub_debug("Can't start data product acquisition " + data_product + " , not in data product dict" )
        else:
            self.msg_if.pub_debug("Starting " + data_product + " acquisition" )
            acquiring = False

            self.msg_if.pub_debug("Waiting for save_data_if: " + data_product)
            while (not nepi_sdk.is_shutdown() and self.save_data_if is None):
                nepi_sdk.sleep(1)

            dp_dict = self.data_products_dict[data_product]
            dp_get_data = dp_dict['get_data']
            dp_stop_data = dp_dict['stop_data']

            #img_pub = nepi_sdk.create_publisher(pub_namespace, Image, queue_size = 10)
            if data_product == 'color_image':
                self.msg_if.pub_debug("Creating ColorImageIF for data product: " + data_product)
                dp_namespace = self.namespace
                self.color_image_if = ColorImageIF(namespace = dp_namespace,
                            data_source_description = self.data_source_description,
                            data_ref_description = self.data_ref_description,
                            perspective = self.perspective,
                            msg_if = self.msg_if,
                            node_if = self.node_if,
                            save_data_if = self.save_data_if,
                            transform_if = self.transform_if,
                            navpose_if = self.navpose_if
                            )
                ready = self.color_image_if.wait_for_ready()
                self.data_products_dict[data_product]['image_topic'] = self.color_image_if.get_namespace()

            if self.color_image_if is None:
                self.msg_if.pub_debug("Failed to create data IF class for: " + data_product + " ** Ending thread")
                return

            # Get Data Product Dict and Data_IF

            self.msg_if.pub_debug("Starting data capture thread for data product: " + data_product)
            while (not nepi_sdk.is_shutdown()):
                # Get data if requried
                get_data = self.color_image_if.needs_data_check()
                if get_data == True and self.device_disabled == False:
                    acquiring = True
                    status, msg, cv2_img, timestamp, encoding = dp_get_data()

                    if (status is False or cv2_img is None):
                        #self.msg_if.pub_debug("No Data Recieved: " + data_product, throttle_s = 5.0)
                        pass
                    else:
                        #self.msg_if.pub_debug("Got Data: " + data_product, throttle_s = 5.0)
                        
                        # Get Image Info and Pub Status if Changed
                        cur_width = self.width_px
                        cur_height = self.height_px
                        cv2_shape = cv2_img.shape
                        self.width_px = cv2_shape[1] 
                        self.height_px = cv2_shape[0]             

                        if self.aspect_ratio_deg is None:
                            self.aspect_ratio_deg = -999
                        if self.aspect_ratio_deg == -999 or self.aspect_ratio_deg < 0.5 or self.aspect_ratio_deg > 2.5:
                            self.aspect_ratio_deg = round(self.width_px / self.height_px, 2)
                            if self.node_if is not None:
                                self.node_if.set_param(self.node_if_prefix + 'aspect_ratio_deg',self.aspect_ratio_deg)

                        cur_aspect_ratio = round(self.width_px / self.height_px, 2)
                        
                        width_deg = self.width_deg / self.aspect_ratio_deg * cur_aspect_ratio
                        height_deg = self.height_deg
                        #self.msg_if.pub_debug("Image Pub degs w,cw,ch,h,ar,cr:: " + str([self.width_deg,width_deg,self.height_deg,height_deg,self.aspect_ratio_deg,cur_aspect_ratio]), throttle_s = 5)
                        # Now process and publish image
                        cv2_img = self.color_image_if.publish_cv2_img(cv2_img, encoding = encoding,
                                                        timestamp = timestamp,
                                                        width_deg = width_deg,
                                                        height_deg = height_deg
                                                        )


                        self.update_fps(data_product)

                        #self.msg_if.pub_debug("Got cv2_img size: " + str(self.width_px) + ":" + str(self.height_px) , throttle_s = 5.0)
                        if cur_width != self.width_px or cur_height != self.height_px:
                            self.publish_status()

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition" )
                        dp_stop_data()
                    acquiring = False
                    self.current_fps[data_product] = 0.0
                    self.fps_queue[data_product] = [0,0,0,0,0,0,0,0,0,0]
                else: # No subscribers and already stopped
                    acquiring = False
                    nepi_sdk.sleep(0.25)
                #self.msg_if.pub_debug("Ending with avg fps: " + str(self.current_fps[data_product]) , throttle_s = 5.0)  
                nepi_sdk.sleep(0.01) # Yield


    def depth_map_thread_proccess(self, data_product):
        """Continuously acquires, processes, and publishes depth map data for a given data product.

        Waits for the save-data interface to be ready, then enters a loop that
        checks whether any subscribers need data. When data is needed, calls the
        registered acquisition function and publishes the resulting numpy depth
        map via ``DepthMapIF``, applying the configured range-window ratios to
        derive effective min/max range values. Stops acquisition and resets FPS
        counters when no subscribers are active.

        Args:
            data_product (str): Key identifying the depth map data product to run
                (e.g. ``'depth_map'``). Must be present in
                ``self.data_products_dict``.
        """
        np_depth_map = None

        if data_product not in self.data_products_dict.keys():
            self.msg_if.pub_debug("Can't start data product acquisition " + data_product + " , not in data product dict" )
        else:
            self.msg_if.pub_debug("Starting " + data_product + " acquisition" )
            acquiring = False

            self.msg_if.pub_debug("Waiting for save_data_if: " + data_product)
            while (not nepi_sdk.is_shutdown() and self.save_data_if is None):
                nepi_sdk.sleep(1)

            dp_dict = self.data_products_dict[data_product]
            dp_get_data = dp_dict['get_data']
            dp_stop_data = dp_dict['stop_data']


            dp_namespace = self.namespace
            self.depthmap_if = DepthMapIF(namespace = dp_namespace,
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        pub_image = True,
                        init_overlay_text_list = [],
                        msg_if = self.msg_if,
                        node_if = self.node_if,
                        save_data_if = self.save_data_if,
                        transform_if = self.transform_if,
                        navpose_if = self.navpose_if
                            )
            ready = self.depthmap_if.wait_for_ready()
            self.data_products_dict[data_product]['image_topic'] = self.depthmap_if.get_namespace()

            if self.depthmap_if is None:
                self.msg_if.pub_debug("Failed to create data IF class for: " + data_product + " ** Ending thread")
                return

            # Get Data Product Dict and Data_IF

            self.msg_if.pub_debug("Starting data capture thread for data product: " + data_product)
            while (not nepi_sdk.is_shutdown()):
                # Get data if requried
                get_data = self.depthmap_if.needs_data_check()
                if get_data == True and self.device_disabled == False:
                    #self.msg_if.pub_debug("Got Depth Map Needs Data" )

                    acquiring = True

                    status, msg, np_depth_map, timestamp, encoding = dp_get_data()
                    #print('IDX Min Max Depths: ' + str([np.nanmin(np_depth_map),np.nanmax(np_depth_map)]) )
                    if (status is False or np_depth_map is None):
                        #self.msg_if.pub_debug("No Data Recieved: " + data_product, throttle_s = 5.0)
                        pass
                    else:                       
                        # Get Image Info and Pub Status if Changed
                        cur_width = self.width_px
                        cur_height = self.height_px
                        cv2_shape = np_depth_map.shape
                        self.width_px = cv2_shape[1] 
                        self.height_px = cv2_shape[0] 

                        # if self.aspect_ratio_deg == -999 or self.aspect_ratio_deg < 0.5 or self.aspect_ratio_deg > 2.5:
                        #     self.aspect_ratio_deg  = round(self.width_px / self.height_px, 2)
                        #     if self.node_if is not None:
                        #         self.node_if.set_param(self.node_if_prefix + 'aspect_ratio_deg',self.aspect_ratio_deg)
                        cur_aspect_ratio = self.width_px / self.height_px
                        if self.aspect_ratio_deg is not None:
                            self.aspect_ratio_deg = cur_aspect_ratio
                        width_deg = self.width_deg / self.aspect_ratio_deg * cur_aspect_ratio
                        height_deg = self.height_deg

                        #Publish Ros Image
                        #self.msg_if.pub_debug("Got Min Max Ranges: " + str([self.min_range_m,self.max_range_m]))
                        range_m = self.max_range_m - self.min_range_m
                        min_range_m = self.min_range_m + self.start_range_ratio * range_m
                        max_range_m = self.max_range_m - (1-self.stop_range_ratio) * range_m
                        #self.msg_if.pub_debug("Using Min Max Ranges: " + str([self.min_range_m,self.max_range_m]))
                        np_depth_map = self.depthmap_if.publish_np_depth_map(np_depth_map,
                                                encoding = encoding,
                                                width_deg = width_deg,
                                                height_deg = height_deg,
                                                min_range_m = min_range_m,
                                                max_range_m = max_range_m,
                                                timestamp = timestamp
                                                )

                        self.update_fps(data_product)
                        if cur_width != self.width_px or cur_height != self.height_px:
                            self.publish_status()

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition" )
                        dp_stop_data()
                    acquiring = False
                    self.current_fps[data_product] = 0.0
                    self.fps_queue[data_product] = [0 for _ in range(100)]
                else: # No subscribers and already stopped
                    acquiring = False
                    nepi_sdk.sleep(0.25)
                #self.msg_if.pub_debug("Ending with avg fps: " + str(self.current_fps[data_product]) , throttle_s = 5.0)    
                nepi_sdk.sleep(0.01) # Yield


              

   
    # Pointcloud from pointcloud_get_function can be open3D or ROS pointcloud.  Will be converted as needed in the thread
    def pointcloud_thread_proccess(self, data_product):
        """Continuously acquires, processes, and publishes pointcloud data for a given data product.

        Waits for the save-data interface to be ready, then enters a loop that
        checks whether any subscribers need data. When data is needed, calls the
        registered acquisition function and publishes the resulting Open3D or ROS
        pointcloud via ``PointcloudIF``, applying the configured range-window
        ratios to derive effective min/max range values. Stops acquisition when
        no subscribers are active.

        Args:
            data_product (str): Key identifying the pointcloud data product to run
                (e.g. ``'pointcloud'``). Must be present in
                ``self.data_products_dict``.
        """
        o3d_pc = None

        if data_product not in self.data_products_dict.keys():
            self.msg_if.pub_debug("Can't start data product acquisition " + data_product + " , not in data product dict" )
        else:
            self.msg_if.pub_debug("Starting " + data_product + " acquisition" )
            acquiring = False

            self.msg_if.pub_debug("Waiting for save_data_if: " + data_product)
            while (not nepi_sdk.is_shutdown() and self.save_data_if is None):
                nepi_sdk.sleep(1)


            dp_dict = self.data_products_dict[data_product]
            dp_get_data = dp_dict['get_data']
            dp_stop_data = dp_dict['stop_data']

            #img_pub = nepi_sdk.create_publisher(pub_namespace, Image, queue_size = 10)
            dp_namespace = self.namespace
            self.pointcloud_if = PointcloudIF(namespace = dp_namespace,
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        pub_image = True,
                        init_overlay_text_list = [],
                        msg_if = self.msg_if,
                        node_if = self.node_if,
                        save_data_if = self.save_data_if,
                        transform_if = self.transform_if,
                        navpose_if = self.navpose_if
                        )
            ready = self.pointcloud_if.wait_for_ready()

            if self.pointcloud_if is None:
                self.msg_if.pub_debug("Failed to create data IF class for: " + data_product + " ** Ending thread")
                return

        
            # Get Data Product Dict and Data_IF

            self.msg_if.pub_debug("Starting data capture thread for data product: " + data_product)
            while (not nepi_sdk.is_shutdown()):
                # Get data if requried
                get_data = self.pointcloud_if.needs_data_check()
                if get_data == True and self.device_disabled == False:
                    acquiring = True
                    status, msg, o3d_pc, timestamp, pc_frame = dp_get_data()
                    if (status is False or o3d_pc is None):
                        #self.msg_if.pub_debug("No Data Recieved: " + data_product, throttle_s = 5.0)
                        pass
                    else:
                        range_m = self.max_range_m - self.min_range_m
                        min_range_m = self.min_range_m + self.start_range_ratio * range_m
                        max_range_m = self.max_range_m - (1-self.stop_range_ratio) * range_m
                        o3d_pc = self.pointcloud_if.publish_o3d_pc(o3d_pc,
                                                width_deg = self.width_deg,
                                                height_deg = self.height_deg,
                                                min_range_m = min_range_m,
                                                max_range_m = max_range_m,
                                                timestamp = timestamp
                                                )

                        self.update_fps(data_product)

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition" )
                        dp_stop_data()
                    acquiring = False
                else: # No subscribers and already stopped
                    acquiring = False
                    nepi_sdk.sleep(0.25)
                nepi_sdk.sleep(0.01) # Yield
                




