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
# from nepi_sdk import nepi_nav
# from nepi_sdk import nepi_devices

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image, PointCloud2

from nepi_interfaces.msg import DeviceIDXStatus, RangeWindow
from nepi_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryRequest, IDXCapabilitiesQueryResponse
from nepi_interfaces.msg import ImageStatus, PointcloudStatus


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SettingsIF, SaveDataIF


from nepi_api.data_if import ColorImageIF, DepthMapIF, PointcloudIF, NavPoseIF






SUPPORTED_DATA_PRODUCTS = ['color_image','bw_image',
                            'intensity_map','depth_map','pointcloud']

PERSPECTIVE_OPTIONS = ['pov','top']

#Factory Control Values 
DEFAULT_CONTROLS_DICT = dict( controls_enable = True,
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
    zoom_ratio = 0.5, 
    rotate_ratio = 0.5,
    )




class IDXDeviceIF:
    # Default Global Values
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]
    UPDATE_NAVPOSE_RATE_HZ = 10

    DEFUALT_IMG_WIDTH_DEG = 100
    DEFUALT_IMG_HEIGHT_DEG = 70

    # Define class variables
    namespace = '~'
    ready = False

    status_msg = DeviceIDXStatus()

    node_if = None
    settings_if = None
    navpose_if = None
    save_data_if = None
    npx_if = None


    device_name = ''

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

    data_products_base_list = []
    data_products_save_list = []


    update_navpose_interval_sec = float(1)/UPDATE_NAVPOSE_RATE_HZ
    last_gps_timestamp = None
    last_odom_timestamp = None
    last_heading_timestamp = None

   
    rtsp_url = None

    width_px = 0
    height_px = 0

    perspective = 'pov'

    width_deg = DEFUALT_IMG_WIDTH_DEG
    height_deg = DEFUALT_IMG_HEIGHT_DEG

    data_product_dict = dict()

    npx_if = None
    
    fps_queue = dict()
    current_fps = dict()
    last_data_time = dict()

    image_thread = None
    depth_map_thread = None
    pointcloud_thread = None

    data_source_description = 'imaging_sensor'
    data_ref_description = 'sensor'


    #######################
    ### IF Initialization
    def __init__(self, device_info, 
                 capSettings=None, factorySettings=None, 
                 settingUpdateFunction=None, getSettingsFunction=None,
                 factoryControls = None, 
                 data_source_description = 'imaging_sensor',
                 data_ref_description = 'sensor',
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
                 getNavPoseCb = None,
                 navpose_update_rate = 10,
                 data_products =  [],
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,'idx')

        ##############################  
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IDX IF Initialization Processes", log_name_list = self.log_name_list)
        self.msg_if.pub_info("Using Namespace: " + str(self.namespace), log_name_list = self.log_name_list)

        ############################# 
        # Initialize Class Variables
        self.device_id = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.device_name = self.device_id + "_" + self.identifier

        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description

        self.msg_if.pub_warn("Got driver supported data products: " + str(data_products))
        #self.msg_if.pub_warn("Supported data products: " + str(SUPPORTED_DATA_PRODUCTS))
        data_products_list = []
        for data_product in data_products:
            #self.msg_if.pub_warn("Checking data product: " + str(data_product))
            if data_product in SUPPORTED_DATA_PRODUCTS:
                data_products_list.append(data_product)
        self.data_products_base_list = data_products_list
        self.msg_if.pub_warn("Enabled data products: " + str(self.data_products_base_list))
        # Create the CV bridge. Do this early so it can be used in the threading run() methods below 
        # TODO: Need one per image output type for thread safety?

        self.caps_report = IDXCapabilitiesQueryResponse()

        # Create and update factory controls dictionary
        self.msg_if.pub_warn("Got Factory Contrls: " + str(factoryControls))
        self.factory_controls_dict = DEFAULT_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]
        
        self.msg_if.pub_warn("Using Factory Contrls: " + str(self.factory_controls_dict))
        self.min_range_m = self.factory_controls_dict['min_range_m']
        self.max_range_m = self.factory_controls_dict['max_range_m']

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

        self.getNavPoseCb = getNavPoseCb
        if navpose_update_rate < 1:
           navpose_update_rate = 1
        if navpose_update_rate > 10:
            navpose_update_rate = 10
        self.navpose_update_rate = navpose_update_rate

        ## Set None Capabilities Variables

        self.setMaxFramerate = setMaxFramerate
        if self.setMaxFramerate is not None:
            self.caps_report.has_framerate = True
        
        self.getFramerate = getFramerate

        self.setRangeRatio = setRangeRatio
        if self.setRangeRatio is not None:
            self.caps_report.has_range = True


        self.setAutoAdjustRatio = setAutoAdjustRatio
        if self.setAutoAdjustRatio is not None:
            self.caps_report.has_auto_adjust = True
        self.auto_adjust_controls = autoAdjustControls

        self.setBrightnessRatio = setBrightnessRatio
        if self.setBrightnessRatio is not None:
            self.caps_report.has_brightness = True

        self.setContrastRatio = setContrastRatio
        if self.setContrastRatio is not None:
            self.caps_report.has_contrast = True

        self.setThresholdingRatio = setThresholdingRatio       
        if self.setThresholdingRatio is not None:
            self.caps_report.has_threshold = True

        self.setResolutionRatio = setResolutionRatio
        if self.setResolutionRatio is not None:
            self.caps_report.has_resolution = True


        # Initialize status message

        self.status_msg.device_id = self.device_id
        self.status_msg.identifier = self.identifier
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version

     

        ##################################################
        ### Node Class Setup

        self.msg_if.pub_debug("Starting Node IF Initialization", log_name_list = self.log_name_list)
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.namespace
        }



        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'device_name': {
                'namespace': self.namespace,
                'factory_val': self.device_name
            },
            'width_deg': {
                'namespace': self.namespace,
                'factory_val': self.width_deg
            },
            'height_deg': {
                'namespace': self.namespace,
                'factory_val': self.height_deg
            },
            'auto_adjust_ebabled': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict["auto_adjust_ebabled"]
            },
            'brightness_ratio': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict["brightness_ratio"]
            },
            'contrast_ratio': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict["contrast_ratio"]
            },
            'threshold_ratio': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict["threshold_ratio"]
            },
            'resolution_ratio': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict["resolution_ratio"]
            },
            'max_framerate': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict["max_framerate"]
            },
            'start_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict["start_range_ratio"]
            },
            'stop_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict["stop_range_ratio"]
            }


        }




        # Services Config Dict ####################

        self.SRVS_DICT = {
            'capabilities_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': IDXCapabilitiesQuery,
                'req': IDXCapabilitiesQueryRequest(),
                'resp': IDXCapabilitiesQueryResponse(),
                'callback': self.provide_capabilities
            }
        }


        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DeviceIDXStatus,
                'qsize': 1,
                'latch': True
            }
        }
        


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'set_device_name': {
                'namespace': self.namespace,
                'topic': 'set_device_name',
                'msg': String,
                'qsize': 1,
                'callback': self.updateDeviceNameCb, 
                'callback_args': ()
            },
            'reset_device_name': {
                'namespace': self.namespace,
                'topic': 'reset_device_name',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetDeviceNameCb, 
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
                'topic': 'reset',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetControlsCb, 
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
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.initCb(do_updates = True)
        self.publish_status()


        # Start the data producers
        if (getColorImage is not None and 'color_image' in self.data_products_base_list):
            self.getColorImage = getColorImage
            self.stopColorImageAcquisition = stopColorImageAcquisition
            data_product = 'color_image'

            start_data_function = self.getColorImage
            stop_data_function = self.stopColorImageAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread", log_name_list = self.log_name_list)
            self.image_thread = threading.Thread(target=self.runImageThread)
            self.image_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.caps_report.has_color_image = True
        else:
            self.caps_report.has_color_image = False
        
        self.caps_report.has_depth_map = False
        self.caps_report.has_pointcloud = False

        if (getDepthMap is not None and 'depth_map' in self.data_products_base_list):
            self.getDepthMap = getDepthMap
            self.stopDepthMapAcquisition = stopDepthMapAcquisition
            data_product = 'depth_map'
            
            start_data_function = self.getDepthMap
            stop_data_function = self.stopDepthMapAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread", log_name_list = self.log_name_list)
            self.depth_map_thread = threading.Thread(target=self.runDepthMapThread)
            self.depth_map_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.caps_report.has_depth_map = True
        else:
            self.caps_report.has_depth_map = False
          
        '''
        if (getPointcloud is not None and 'pointcloud' in self.data_products_base_list):
            self.getPointcloud = getPointcloud
            self.stopPointcloudAcquisition = stopPointcloudAcquisition
            data_product = 'pointcloud'

            start_data_function = self.getPointcloud
            stop_data_function = self.stopPointcloudAcquisition
            data_msg = PointCloud2
            data_status_msg = PointcloudStatus

            success = self.addDataProduct2Dict(data_product,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread", log_name_list = self.log_name_list)
            self.pointcloud_thread = threading.Thread(target=self.runPointcloudThread)
            self.pointcloud_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.caps_report.has_pointcloud = True
        else:
            self.caps_report.has_pointcloud = False
        '''

        ############################
        # Start Data Get Threads
        self.caps_report.data_products = self.data_products_base_list

        for data_product in self.data_products_base_list:
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

        nepi_sdk.start_timer_process(1, self.publishStatusCb)


        ###############################
        # Setup Settings IF Class ####################
        self.msg_if.pub_debug("Starting Settings IF Initialization", log_name_list = self.log_name_list)
        settings_ns = self.namespace

        self.SETTINGS_DICT = {
                    'capSettings': capSettings, 
                    'factorySettings': factorySettings,
                    'setSettingFunction': settingUpdateFunction, 
                    'getSettingsFunction': getSettingsFunction
                    
        }

        self.settings_if = SettingsIF(namespace = settings_ns,
                        settings_dict = self.SETTINGS_DICT,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )



        ##################################
        # Setup Save Data IF Class ####################
        self.msg_if.pub_debug("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
        factory_data_rates= {}
        for d in self.data_products_save_list:
            factory_data_rates[d] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
        if 'color_image' in self.data_products_save_list:
            factory_data_rates['color_image'] = [1.0, 0.0, 100] 
        

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_us': False,
            'suffix': "idx",
            'add_node_name': True
            }

        self.msg_if.pub_debug("Starting save_rate_dict: " + str(factory_data_rates))
        sd_namespace = self.namespace
        self.save_data_if = SaveDataIF(data_products = self.data_products_save_list,
                                factory_rate_dict = factory_data_rates,
                                factory_filename_dict = factory_filename_dict,
                                namespace = sd_namespace,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )

        ####################
        # Setup Save Data IF Class 
        self.msg_if.pub_info("Starting NavPoe IF Initialization")
        self.navpose_if = NavPoseIF(namespace = self.namespace,
                                    data_product = 'navpose',    
                                    save_data_if = self.save_data_if)
            
        self.status_msg.navpose_topic = self.navpose_if.get_namespace()
        self.msg_if.pub_info("Using navpose namespace: " + str(self.status_msg.navpose_topic))

        ##################################
        # Start Node Processes
        #nepi_sdk.start_timer_process(1, self._updaterCb, oneshot = True)

    
        ####################################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ####################################

        
    

    ###############################
    # Class Methods


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready   


    def initConfig(self):
        self.initCb(do_updates = True)


    def initCb(self,do_updates = False):
      if self.node_if is not None:
            self.device_name = self.node_if.get_param('device_name')
            self.width_deg = self.node_if.get_param('width_deg')
            self.height_deg = self.node_if.get_param('height_deg')  
            self.resolution_ratio = self.node_if.get_param('resolution_ratio')
            max_framerate = self.node_if.get_param('max_framerate') 
            if max_framerate is not None:
                self.max_framerate = max_framerate
            self.auto_adjust_ebabled = self.node_if.get_param('auto_adjust_ebabled') 
            self.brightness_ratio = self.node_if.get_param('brightness_ratio')
            self.contrast_ratio = self.node_if.get_param('contrast_ratio')        
            self.threshold_ratio = self.node_if.get_param('threshold_ratio')  
            self.start_range_ratio = self.node_if.get_param('start_range_ratio')
            self.stop_range_ratio = self.node_if.get_param('stop_range_ratio')

            self.pt_mounted = self.node_if.get_param('pt_mounted')
            self.pt_topic = self.node_if.get_param('pt_topic')


      if do_updates == True:
        self.ApplyConfigUpdates()
      self.publish_status()

    def resetCb(self,do_updates = True):
      if self.node_if is not None:
        # self.node_if.reset_params()
        if self.getFOV is not None:
            try:
                [width_deg,height_deg] = self.getFOV()
                if node_if is not None:
                    self.node_if.set_param('width_deg',self.width_deg)
                    self.node_if.set_param('height_deg',self.height_deg) 
            except:
                pass
      if self.save_data_if is not None:
          self.save_data_if.reset()

      if self.settings_if is not None:
          self.settings_if.reset()
      if do_updates == True:
        pass
      self.initCb(do_updates = True)

    def factoryResetCb(self,do_updates = True):
      if self.node_if is not None:
        #self.node_if.factory_reset_params()
        if self.getFOV is not None:
            try:
                [width_deg,height_deg] = self.getFOV()
                if node_if is not None:
                    self.node_if.set_param('width_deg',self.width_deg)
                    self.node_if.set_param('height_deg',self.height_deg) 
            except:
                pass
      if self.save_data_if is not None:
          self.save_data_if.factory_reset()

      if self.settings_if is not None:
          self.settings_if.factory_reset()
      if do_updates == True:
        pass
      self.initCb(do_updates = True)



    def ApplyConfigUpdates(self):
        self.msg_if.pub_warn("Apply Auto Updates from current values")
        if (self.setAutoAdjustRatio is not None):
            self.setAutoAdjustRatio(self.auto_adjust_ebabled)
        if (self.setBrightnessRatio is not None):
            self.setBrightnessRatio(self.brightness_ratio)
        if (self.setContrastRatio is not None):
            self.setContrastRatio(self.contrast_ratio)
        if (self.setThresholdingRatio is not None):
            self.setThresholdingRatio(self.threshold_ratio)
        if (self.setResolutionRatio is not None):
            self.setResolutionRatio(self.resolution_ratio)
        if (self.setMaxFramerate is not None and self.max_framerate is not None):
            self.msg_if.pub_warn("Apply Config Framerate: " + str(self.max_framerate))
            self.setMaxFramerate(self.max_framerate)
        if (self.setRangeRatio is not None):
            self.setRangeRatio(self.start_range_ratio, self.stop_range_ratio)




    def addDataProduct2Dict(self,data_product, start_data_function,stop_data_function,data_msg,data_status_msg):
        success = False
        data_product = data_product
        namespace = os.path.join(self.base_namespace,self.node_name,'idx')
        dp_dict = dict()
        dp_dict['data_product'] = data_product
        

        dp_dict['get_data'] = start_data_function
        dp_dict['stop_data'] = stop_data_function

        self.data_products_save_list.append(data_product)
        if data_product == 'color_image':
            pass
        elif data_product == 'bw_image':
            pass
        elif data_product == 'intensity_map':
            self.data_products_save_list.append(data_product + '_image')
        elif data_product == 'depth_map':
            self.data_products_save_list.append(data_product + '_image')
        elif data_product == 'pointcloud':
            self.data_products_save_list.append(data_product + '_image')
        self.data_product_dict[data_product] = dp_dict

        # do wait here for all
        success = True
        return success






    def updateDeviceNameCb(self, msg):
        self.msg_if.pub_info("Recived update message: " + str(msg))
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
            self.status_msg.device_name = new_device_name
            self.publish_status(do_updates=False) # Updated inline here 
            self.node_if.set_param('device_name', new_device_name)



    def resetDeviceNameCb(self,msg):
        self.msg_if.pub_info("Recived update message: " + str(msg))
        self.resetDeviceName()

    def resetDeviceName(self):
        self.status_msg.device_name = self.device_name
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('device_name', self.device_name)



    def setWidthDegCb(self, msg):
        self.msg_if.pub_info("Recived Width Deg update message: " + str(msg))
        width = msg.data
        if width > 0:
            self.width_deg = width     
            self.publish_status(do_updates=False) # Updated inline here   
            self.node_if.set_param('width_deg', width)
 
    def setHeightDegCb(self, msg):
        self.msg_if.pub_info("Recived Height Deg update message: " + str(msg))
        height = msg.data
        if height > 0:
            self.width_deg = height     
            self.publish_status(do_updates=False) # Updated inline here   
            self.node_if.set_param('height_deg', height)
            
    def setAutoAdjustEnableCb(self, msg):
        self.msg_if.pub_info("Recived Auto Adjust update message: " + str(msg))
        enabled = msg.data
        if self.setAutoAdjustRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setAutoAdjustRatio(enabled)

        if enabled:
            self.msg_if.pub_info("Enabling Auto Adjust", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Disabling IDX Auto Adjust", log_name_list = self.log_name_list)

        self.auto_adjust_ebabled = enabled       
        self.publish_status(do_updates=False) # Updated inline here
        if self.node_if is not None:
            self.node_if.set_param('auto_adjust_ebabled', enabled)




    def setBrightnessRatioCb(self, msg):
        self.msg_if.pub_info("Recived Brightness update message: " + str(msg))
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.brightness_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        if self.setBrightnessRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setBrightnessRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param('brightness_ratio', ratio)


    def setContrastRatioCb(self, msg):
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.contrast_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        if self.setContrastRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setContrastRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param('contrast_ratio', ratio)
        


    def setThresholdingRatioCb(self, msg):
        self.msg_if.pub_info("Received Threshold update message: " + str(msg))
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.threshold_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        if self.setThresholdingRatio is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setThresholdingRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param('threshold_ratio', ratio)
        

    def setResolutionRatioCb(self, msg):
        self.msg_if.pub_info("Recived Resolution update message: " + str(msg))
        ratio = msg.data
 
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0

        self.resolution_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        if self.setResolutionRatio is not None:
            status, err_str = self.setResolutionRatio(ratio)
        if self.node_if is not None:
            self.node_if.set_param('resolution_ratio', ratio)
        


        
    def setMaxFramerateCb(self, msg):
        self.msg_if.pub_info("Recived Max Framerate update message: " + str(msg))
        rate = msg.data
 
        if rate < 1:
            rate = 1
        if rate > 100:
            rate = 100

        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        self.max_framerate = rate
        self.publish_status(do_updates=False) # Updated inline here

        if self.setMaxFramerate is not None:
            #self.msg_if.pub_warn("Sending update framerate ratio to driver: ")
            status, err_str = self.setMaxFramerate(rate)
            #self.msg_if.pub_warn("Recived Framerate update: " + str(status))


        for data_product in self.data_products_base_list:
            self.fps_queue[data_product] = [0 for _ in range(100)]

        if self.node_if is not None:
            self.node_if.set_param('max_framerate', rate)


 
    def setRangeRatioCb(self, msg):
        self.msg_if.pub_info("Recived Range update message: " + str(msg))
        self.msg_if.pub_info("Recived update message: " + str(msg))
        new_start_range_ratio = msg.start_range
        new_stop_range_ratio = msg.stop_range
        if (new_start_range_ratio < 0 or new_stop_range_ratio > 1 or new_stop_range_ratio < new_start_range_ratio):
            self.msg_if.pub_error("Range values out of bounds", log_name_list = self.log_name_list)
            self.publish_status(do_updates=False) # No change
            return
        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            if self.setRangeRatio is not None:
                status, err_str = self.setRangeRatio(new_start_range_ratio,new_stop_range_ratio)

        self.start_range_ratio = new_start_range_ratio
        self.stop_range_ratio = new_stop_range_ratio
        self.publish_status(do_updates=False) # Updated inline here  
        if self.node_if is not None:
            self.node_if.set_param('start_range_ratio', new_start_range_ratio)
            self.node_if.set_param('stop_range_ratio', new_stop_range_ratio)
     
  

    def resetControlsCb(self, msg):
        self.msg_if.pub_info("Recived reset controls message: " + str(msg))
        self.node_if.reset_param('device_name')
        self.node_if.reset_param('controls_enable')
        self.node_if.reset_param('auto_adjust_ebabled')       
        self.node_if.reset_param('brightness_ratio')
        self.node_if.reset_param('contrast_ratio')        
        self.node_if.reset_param('threshold_ratio')
        self.node_if.reset_param('resolution_ratio')   
        self.node_if.reset_param('max_framerate')
        self.node_if.reset_param('start_range_ratio')
        self.node_if.reset_param('stop_range_ratio')
        self.resetCb(do_updates = True)


    def factoryResetControlsCb(self, msg):
        self.msg_if.pub_info("Recived factory reset controls message: " + str(msg))
        self.node_if.factory_reset_param('device_name')
        self.node_if.factory_reset_param('controls_enable')
        self.node_if.factory_reset_param('auto_adjust_ebabled')       
        self.node_if.factory_reset_param('brightness_ratio')
        self.node_if.factory_reset_param('contrast_ratio')        
        self.node_if.factory_reset_param('threshold_ratio')
        self.node_if.factory_reset_param('resolution_ratio')   
        self.node_if.factory_reset_param('max_framerate')
        self.node_if.factory_reset_param('start_range_ratio')
        self.node_if.factory_reset_param('stop_range_ratio')

        self.factoryResetCb(do_updates = True)




    def provide_capabilities(self, _):
        return self.caps_report
    
    def update_fps(self,data_product):
        last_data_time = copy.deepcopy(self.last_data_time[data_product])
        self.last_data_time[data_product] = nepi_utils.get_time()
        last_fps = copy.deepcopy(self.current_fps[data_product])
        if last_data_time is not None:
            f_time = (self.last_data_time[data_product] - last_data_time)
            current_fps = float(1) / f_time
            #self.msg_if.pub_warn("Got " + data_product + " time and fps: " + str(round(f_time,3)) + " : " + str(round(current_fps,2)), throttle_s = 2)
            self.fps_queue[data_product].pop(0)
            self.fps_queue[data_product].append(current_fps)
            self.msg_if.pub_debug("fps queue " + str(self.fps_queue[data_product]), throttle_s = 2)
            fps_queue = [x for x in self.fps_queue[data_product] if x != 0]
            if len(fps_queue) > 1:
                self.current_fps[data_product] = round(sum(fps_queue)/len(fps_queue),0)
            fps_changed = abs(self.current_fps[data_product] - last_fps) > 1
            fps_changing = 0 in self.fps_queue[data_product]
            if fps_changed or fps_changing:
                self.publish_status()
  

    # Image from img_get_function can be CV2 or ROS image.  Will be converted as needed in the thread
    def image_thread_proccess(self,data_product):
        cv2_img = None
        ros_img = None

        if data_product not in self.data_product_dict.keys():
            self.msg_if.pub_warn("Can't start data product acquisition " + data_product + " , not in data product dict", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_warn("Starting " + data_product + " acquisition", log_name_list = self.log_name_list)
            acquiring = False

            dp_dict = self.data_product_dict[data_product]
            dp_get_data = dp_dict['get_data']
            dp_stop_data = dp_dict['stop_data']

            #img_pub = nepi_sdk.create_publisher(pub_namespace, Image, queue_size = 10)
            dp_if = None
            if data_product == 'color_image':
                self.msg_if.pub_warn("Creating ColorImageIF for data product: " + data_product)
                dp_namespace = self.namespace
                dp_if = ColorImageIF(namespace = dp_namespace, 
                            data_product_name = data_product, 
                            data_source_description = self.data_source_description,
                            data_ref_description = self.data_ref_description,
                            perspective = self.perspective,
                            navpose_if = self.navpose_if,
                            save_data_if = self.save_data_if,
                            log_name = data_product,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )
                ready = dp_if.wait_for_ready()
                
            if dp_if is None:
                self.msg_if.pub_debug("Failed to create data IF class for: " + data_product + " ** Ending thread")
                return

            # Get Data Product Dict and Data_IF
            self.msg_if.pub_warn("Starting thread with data_product dict: " + data_product + " " + str(dp_dict))
            self.msg_if.pub_debug("Waiting for save_data_if: " + data_product)
            while (not nepi_sdk.is_shutdown() and self.save_data_if is None):
                nepi_sdk.sleep(1)
            while (not nepi_sdk.is_shutdown()):
                dp_has_subs = dp_if.has_subscribers_check()
                dp_should_save = self.save_data_if.data_product_should_save(data_product)
                dp_should_save = dp_should_save or self.save_data_if.data_product_snapshot_enabled(data_product)

                # Get data if requried
                get_data = dp_has_subs or dp_should_save
                if get_data == True:
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
                        

                        # Now process and publish image
                        cv2_img = dp_if.publish_cv2_img(cv2_img, encoding = encoding,
                                                        timestamp = timestamp,
                                                        width_deg = self.width_deg,
                                                        height_deg = self.height_deg
                                                        )
                        if (dp_should_save == True):
                            self.save_data_if.save(data_product,cv2_img,timestamp = timestamp,save_check=False)


                        self.update_fps(data_product)

                        #self.msg_if.pub_debug("Got cv2_img size: " + str(self.width_px) + ":" + str(self.height_px), log_name_list = self.log_name_list, throttle_s = 5.0)
                        if cur_width != self.width_px or cur_height != self.height_px:
                            self.publish_status()

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition", log_name_list = self.log_name_list)
                        dp_stop_data()
                    acquiring = False
                    self.current_fps[data_product] = 0.0
                    self.fps_queue[data_product] = [0,0,0,0,0,0,0,0,0,0]
                else: # No subscribers and already stopped
                    acquiring = False
                    nepi_sdk.sleep(0.25)
                #self.msg_if.pub_debug("Ending with avg fps: " + str(self.current_fps[data_product]), log_name_list = self.log_name_list, throttle_s = 5.0)  
                nepi_sdk.sleep(0.01) # Yield


    def depth_map_thread_proccess(self,data_product):
        cv2_depth_map = None
        ros_img = None

        if data_product not in self.data_product_dict.keys():
            self.msg_if.pub_warn("Can't start data product acquisition " + data_product + " , not in data product dict", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_warn("Starting " + data_product + " acquisition", log_name_list = self.log_name_list)
            acquiring = False

            dp_dict = self.data_product_dict[data_product]
            dp_get_data = dp_dict['get_data']
            dp_stop_data = dp_dict['stop_data']


            range_m = self.max_range_m - self.min_range_m
            min_range_m = self.min_range_m + self.start_range_ratio * range_m
            max_range_m = self.max_range_m - (1-self.stop_range_ratio) * range_m
            dp_namespace = self.namespace
            dp_if = DepthMapIF(namespace = dp_namespace, 
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        pub_image = True,
                        init_overlay_list = [],
                        navpose_if = self.navpose_if,
                        save_data_if = self.save_data_if,
                        log_name = data_product,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )
            ready = dp_if.wait_for_ready()
            
            if dp_if is None:
                self.msg_if.pub_debug("Failed to create data IF class for: " + data_product + " ** Ending thread")
                return

            # Get Data Product Dict and Data_IF
            self.msg_if.pub_warn("Starting thread with data_product dict: " + data_product + " " + str(dp_dict))
            self.msg_if.pub_debug("Waiting for save_data_if: " + data_product)
            while (not nepi_sdk.is_shutdown() and self.save_data_if is None):
                nepi_sdk.sleep(1)
            while (not nepi_sdk.is_shutdown()):
                dp_has_subs = dp_if.has_subscribers_check()
                dp_should_save = self.save_data_if.data_product_should_save(data_product)
                dp_should_save = dp_should_save or self.save_data_if.data_product_should_save(data_product + '_image')
                dp_should_save = dp_should_save or self.save_data_if.data_product_snapshot_enabled(data_product)

                # Get data if requried
                get_data = dp_has_subs or dp_should_save
                if get_data == True:
                    acquiring = True

                    status, msg, cv2_depth_map, timestamp, encoding = dp_get_data()
                    if (status is False or cv2_depth_map is None):
                        #self.msg_if.pub_warn("No Data Recieved: " + data_product, throttle_s = 5.0)
                        pass
                    else:
                        #self.msg_if.pub_warn("Got Data: " + data_product, throttle_s = 5.0)
                        
                        # Get Image Info and Pub Status if Changed
                        cur_width = self.width_px
                        cur_height = self.height_px
                        cv2_shape = cv2_depth_map.shape
                        self.width_px = cv2_shape[1] 
                        self.height_px = cv2_shape[0] 


                        if (dp_has_subs == True):
                            #Publish Ros Image
                            range_m = self.max_range_m - self.min_range_m
                            min_range_m = self.min_range_m + self.start_range_ratio * range_m
                            max_range_m = self.max_range_m - (1-self.stop_range_ratio) * range_m
                            [cv2_depth_map, cv2_depth_map_img] = dp_if.publish_cv2_depth_map(cv2_depth_map,
                                                    encoding = encoding,
                                                    width_deg = self.width_deg,
                                                    height_deg = self.height_deg,
                                                    min_range_m = min_range_m,
                                                    max_range_m = max_range_m,
                                                    timestamp = timestamp
                                                    )
                        if (dp_should_save == True):
                            self.save_data_if.save(data_product,cv2_depth_map,timestamp = timestamp,save_check=False)
                            self.save_data_if.save(data_product + '_image',cv2_depth_map_img,timestamp = timestamp,save_check=False)

                        self.update_fps(data_product)

                        #self.msg_if.pub_debug("Got cv2_depth_map size: " + str(self.width_px) + ":" + str(self.height_px), log_name_list = self.log_name_list, throttle_s = 5.0)
                        if cur_width != self.width_px or cur_height != self.height_px:
                            self.publish_status()

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition", log_name_list = self.log_name_list)
                        dp_stop_data()
                    acquiring = False
                    self.current_fps[data_product] = 0.0
                    self.fps_queue[data_product] = [0 for _ in range(100)]
                else: # No subscribers and already stopped
                    acquiring = False
                    nepi_sdk.sleep(0.25)
                #self.msg_if.pub_debug("Ending with avg fps: " + str(self.current_fps[data_product]), log_name_list = self.log_name_list, throttle_s = 5.0)    
                nepi_sdk.sleep(0.01) # Yield


              

   
    # Pointcloud from pointcloud_get_function can be open3D or ROS pointcloud.  Will be converted as needed in the thread
    def pointcloud_thread_proccess(self,data_product):
        o3d_pc = None
        ros_pc = None

        if data_product not in self.data_product_dict.keys():
            self.msg_if.pub_warn("Can't start data product acquisition " + data_product + " , not in data product dict", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_warn("Starting " + data_product + " acquisition", log_name_list = self.log_name_list)
            acquiring = False

            dp_dict = self.data_product_dict[data_product]
            dp_get_data = dp_dict['get_data']
            dp_stop_data = dp_dict['stop_data']

            #img_pub = nepi_sdk.create_publisher(pub_namespace, Image, queue_size = 10)
            dp_namespace = self.namespace
            dp_if = PointcloudIF(namespace = dp_namespace, 
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        pub_image = True,
                        init_overlay_list = [],
                        navpose_if = self.navpose_if,
                        save_data_if = self.save_data_if,
                        log_name = data_product,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )
            ready = dp_if.wait_for_ready()
            if dp_if is None:
                self.msg_if.pub_debug("Failed to create data IF class for: " + data_product + " ** Ending thread")
                return

        
            # Get Data Product Dict and Data_IF
            self.msg_if.pub_warn("Starting thread with data_product dict: " + data_product + " " + str(dp_dict))
            self.msg_if.pub_debug("Waiting for save_data_if: " + data_product)
            while (not nepi_sdk.is_shutdown() and self.save_data_if is None):
                nepi_sdk.sleep(1)
            while (not nepi_sdk.is_shutdown()):
 
                # Get Data Product Dict and Data_IF
                dp_has_subs = dp_if.has_subscribers_check()
                dp_should_save = self.save_data_if.data_product_should_save(data_product)
                dp_should_save = dp_should_save or self.save_data_if.data_product_should_save(data_product + '_image')
                dp_should_save = dp_should_save or self.save_data_if.data_product_snapshot_enabled(data_product)

                # Get data if requried
                get_data = dp_has_subs or dp_should_save
                if get_data == True:
                    acquiring = True
                    status, msg, o3d_pc, timestamp, pc_frame = dp_get_data()
                    if o3d_pc is not None:
                        if (dp_has_subs == True):
                            range_m = self.max_range_m - self.min_range_m
                            min_range_m = self.min_range_m + self.start_range_ratio * range_m
                            max_range_m = self.max_range_m - (1-self.stop_range_ratio) * range_m
                            [o3d_pc, cv2_pc_img] = dp_if.publish_cv2_o3d_pc(o3d_pc,
                                                    width_deg = self.width_deg,
                                                    height_deg = self.height_deg,
                                                    min_range_m = min_range_m,
                                                    max_range_m = max_range_m,
                                                    timestamp = timestamp
                                                    )
                        if (dp_should_save == True ):
                            self.save_data_if.save(data_product,o3d_pc,timestamp = timestamp,save_check=False)
                            self.save_data_if.save(data_product + '_image',cv2_pc_img,timestamp = timestamp,save_check=False)

                        self.update_fps(data_product)

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition", log_name_list = self.log_name_list)
                        dp_stop_data()
                    acquiring = False
                else: # No subscribers and already stopped
                    acquiring = False
                    nepi_sdk.sleep(0.25)
                nepi_sdk.sleep(0.01) # Yield
                



    def runImageThread(self):
        self.image_thread_proccess('color_image')

    def runDepthMapThread(self):
        self.depth_map_thread_proccess('depth_map')
        #pass

    def runPointcloudThread(self):
        self.pointcloud_thread_proccess('pointcloud')

 

    # Function to update and publish status message
    def publishStatusCb(self,timer):
        self.publish_status()


    def publish_status(self, do_updates = True):
        self.status_msg.device_name = self.device_name

        self.status_msg.width_deg = self.width_deg
        self.status_msg.height_deg = self.height_deg
        self.status_msg.perspective = self.perspective
        
        self.status_msg.resolution_ratio = self.resolution_ratio
        res_str = str(self.width_px) + ":" + str(self.height_px)
        self.status_msg.resolution_current = res_str

        self.status_msg.max_framerate = self.max_framerate
        self.status_msg.data_products = self.data_products_base_list
        # framerates = []
        # for dp in self.current_fps.keys():
        #     framerates.append(self.current_fps[dp])
        # self.status_msg.framerates = framerates


        self.status_msg.auto_adjust_enabled = self.auto_adjust_ebabled
        self.status_msg.auto_adjust_controls = self.auto_adjust_controls
        self.status_msg.contrast_ratio = self.contrast_ratio
        self.status_msg.brightness_ratio = self.brightness_ratio
        self.status_msg.threshold_ratio = self.threshold_ratio
        
        self.status_msg.range_window_ratios.start_range = self.start_range_ratio
        self.status_msg.range_window_ratios.stop_range =  self.stop_range_ratio

        self.status_msg.min_range_m = self.min_range_m
        self.status_msg.max_range_m = self.max_range_m


        if do_updates == True:
           
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

        self.msg_if.pub_debug("Created status msg: " + str(self.status_msg), throttle_s = 5.0)
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub',self.status_msg)
    
