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
import copy
import threading
import subprocess
import numpy as np

import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_devices

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image, PointCloud2

from nepi_interfaces.msg import DeviceIDXStatus, RangeWindow
from nepi_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryRequest, IDXCapabilitiesQueryResponse
from nepi_interfaces.msg import ImageStatus, PointcloudStatus
from nepi_interfaces.msg import Frame3DTransform

from geometry_msgs.msg import Vector3

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SettingsIF, SaveDataIF, Transform3DIF

from nepi_api.data_if import ColorImageIF
from nepi_api.data_if import DepthMapIF
from nepi_api.data_if import PointcloudIF
from nepi_api.device_if_npx import NPXDeviceIF
from nepi_api.connect_device_if_ptx import ConnectPTXDeviceIF

from nepi_api.data_if import NavPoseIF
from nepi_api.connect_mgr_if_navpose import ConnectMgrNavPoseIF




SUPPORTED_DATA_PRODUCTS = ['color_image','bw_image',
                            'intensity_map','depth_map','pointcloud']

FRAME_3D_OPTIONS = ['sensor_frame','nepi_frame','world_frame']

#Factory Control Values 
DEFAULT_CONTROLS_DICT = dict( controls_enable = True,
    auto_adjust_ebabled = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.0,
    resolution_ratio = 1.0, 
    framerate_ratio = 0.5, 
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

    ZERO_TRANSFORM = [0,0,0,0,0,0,0]

    DEFUALT_IMG_WIDTH_DEG = 100
    DEFUALT_IMG_HEIGHT_DEG = 70

    # Define class variables
    ready = False

    status_msg = DeviceIDXStatus()

    node_if = None
    settings_if = None
    save_data_if = None
    transform_if = None
    npx_if = None
    navpose_if = None


    device_name = ''

    ctl_auto = False
    ctl_brightness = 0.5
    ctl_contrast = 0.5
    ctl_threshold = 0.0
    ctl_res_ratio = 1.0  
    ctl_fr_ratio = 1.0
    ctr_start_range_ratio = 0.0
    ctr_stop_range_ratio = 1.0

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

    width_deg = DEFUALT_IMG_WIDTH_DEG
    height_deg = DEFUALT_IMG_WIDTH_DEG

    data_product_dict = dict()

    npx_if = None
    
    fps_queue = dict()
    current_fps = dict()
    last_data_time = dict()

    image_thread = None
    depth_map_thread = None
    pointcloud_thread = None

    frame_3d = 'nepi_frame'
    tr_source_ref_description = 'sensor'
    tr_end_ref_description = 'nepi_frame'

    data_source_description = 'imaging_sensor'
    data_ref_description = 'sensor'
    device_mount_description = 'fixed'
    mount_desc = 'None'

    avail_pts = []
    pt_mounted = False
    pt_topic = ''
    connect_ptx_if = None
    pt_connected = False



    #######################
    ### IF Initialization
    def __init__(self, device_info, 
                 capSettings=None, factorySettings=None, 
                 settingUpdateFunction=None, getSettingsFunction=None,
                 factoryControls = None, setControlsEnable=None, setAutoAdjust=None,
                 data_source_description = 'imaging_sensor',
                 data_ref_description = 'sensor',
                 getFOV=None,
                 get_rtsp_url = None,
                 setContrast=None, setBrightness=None, setThresholding=None,
                 setResolutionRatio=None, setFramerateRatio=None, 
                 setRange=None, getFramerate=None,
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
        
        ## Connect NEPI NavPose Manager
        self.nav_mgr_if = ConnectMgrNavPoseIF()
        ready = self.nav_mgr_if.wait_for_ready()

        ############################# 
        # Initialize Class Variables
        self.device_id = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.device_name = device_info["device_id"] + "_" + device_info["identifier"]

        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description
        self.tr_source_ref_description = data_ref_description

        data_products_list = []
        for data_product in data_products:
            if data_product in SUPPORTED_DATA_PRODUCTS:
                data_products_list.append(data_product)
        self.data_products_base_list = data_products_list
        self.msg_if.pub_warn("Enabled data products: " + str(self.data_products_base_list))
        # Create the CV bridge. Do this early so it can be used in the threading run() methods below 
        # TODO: Need one per image output type for thread safety?

        self.caps_report = IDXCapabilitiesQueryResponse()

        # Create and update factory controls dictionary
        self.factory_controls_dict = DEFAULT_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]
        
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

        self.get_rtsp_url = get_rtsp_url

        self.getNavPoseCb = getNavPoseCb
        self.navpose_update_rate = navpose_update_rate

        ## Set None Capabilities Variables
        self.setControlsEnable = setControlsEnable

        self.setAutoAdjust = setAutoAdjust
        self.caps_report.has_auto_adjust = True

        self.setBrightness = setBrightness
        self.caps_report.has_brightness = True

        self.setContrast = setContrast
        self.caps_report.has_contrast = True

        self.setThresholding = setThresholding       
        self.caps_report.has_threshold = True

        self.setResolutionRatio = setResolutionRatio
        self.caps_report.has_resolution = True

        self.setFramerateRatio = setFramerateRatio
        self.getFramerate = getFramerate
        self.caps_report.has_framerate = True

        self.setRange = setRange
        self.caps_report.has_range = True

        self.caps_report.frame_3d_options = FRAME_3D_OPTIONS


        # Initialize status message

        self.status_msg.device_id = self.device_id
        self.status_msg.identifier = self.identifier
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version

        self.status_msg.device_mount_description = self.get_mount_description()         

        ##################################################
        ### Node Class Setup

        self.msg_if.pub_debug("Starting Node IF Initialization", log_name_list = self.log_name_list)
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.node_namespace
        }



        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'device_name': {
                'namespace': self.node_namespace,
                'factory_val': self.device_name
            },
            'width_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.width_deg
            },
            'height_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.height_deg
            },
            'auto_adjust_ebabled': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["auto_adjust_ebabled"]
            },
            'brightness_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["brightness_ratio"]
            },
            'contrast_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["contrast_ratio"]
            },
            'threshold_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["threshold_ratio"]
            },
            'resolution_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["resolution_ratio"]
            },
            'framerate_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["framerate_ratio"]
            },
            'start_range_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["start_range_ratio"]
            },
            'stop_range_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["stop_range_ratio"]
            },
            'frame_3d': {
                'namespace': self.node_namespace,
                'factory_val': self.frame_3d
            },
            'pt_mounted': {
                'namespace': self.node_namespace,
                'factory_val': False
            },
            'pt_topic': {
                'namespace': self.node_namespace,
                'factory_val': ''
            },
            'mount_desc': {
                'namespace': self.node_namespace,
                'factory_val': 'None'
            }


        }




        # Services Config Dict ####################

        self.SRVS_DICT = {
            'capabilities_query': {
                'namespace': self.node_namespace,
                'topic': 'idx/capabilities_query',
                'srv': IDXCapabilitiesQuery,
                'req': IDXCapabilitiesQueryRequest(),
                'resp': IDXCapabilitiesQueryResponse(),
                'callback': self.provide_capabilities
            }
        }


        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'idx/status',
                'msg': DeviceIDXStatus,
                'qsize': 1,
                'latch': True
            }
        }

        '''            'image_pub': {
                'namespace': self.node_namespace,
                'topic': 'idx/test_image',
                'msg': Image,
                'qsize': 1,
                'latch': True
            }
        '''
        


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'set_device_name': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_device_name',
                'msg': String,
                'qsize': 1,
                'callback': self.updateDeviceNameCb, 
                'callback_args': ()
            },
            'reset_device_name': {
                'namespace': self.node_namespace,
                'topic': 'idx/reset_device_name',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetDeviceNameCb, 
                'callback_args': ()
            },
            'set_width_deg': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_width_deg',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setWidthDegCb, 
                'callback_args': ()
            },
            'set_height_deg': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_height_deg',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setHeightDegCb, 
                'callback_args': ()
            },
            'set_auto_adjust': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_auto_adjust_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setAutoAdjustCb, 
                'callback_args': ()
            },
            'set_brightness': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_brightness_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setBrightnessCb, 
                'callback_args': ()
            },
            'set_contrast': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_contrast_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setContrastCb, 
                'callback_args': ()
            },
            'set_threshold': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_threshold_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setThresholdingCb, 
                'callback_args': ()
            },
            'set_resolution_ratio': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_resolution_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setResolutionRatioCb, 
                'callback_args': ()
            },
            'set_framerate_ratio': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_framerate_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setFramerateRatioCb, 
                'callback_args': ()
            },
            'set_range_window': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_range_window',
                'msg': RangeWindow,
                'qsize': 1,
                'callback': self.setRangeCb, 
                'callback_args': ()
            },
            'set_frame_3d': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_frame_3d',
                'msg': String,
                'qsize': 1,
                'callback': self.setFrame3dCb, 
                'callback_args': ()
            },
            'reset_controls': {
                'namespace': self.node_namespace,
                'topic': 'idx/reset',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetControlsCb, 
                'callback_args': ()
            },
            'set_pt_mounted': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_pt_mounted',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setPtMountedCb, 
                'callback_args': ()
            },
            'set_pt_topic': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_pt_topic',
                'msg': String,
                'qsize': 1,
                'callback': self.setPtTopicCb, 
                'callback_args': ()
            },
            'set_mount_desc': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_mount_description',
                'msg': String,
                'qsize': 1,
                'callback': self.setMountDescCb, 
                'callback_args': ()
            },
            'reset_mount_desc': {
                'namespace': self.node_namespace,
                'topic': 'idx/reset_mount_description',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetMountDescCb, 
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

        ready = self.node_if.wait_for_ready()


        # Setup 3D Transform IF Class ####################
        self.msg_if.pub_debug("Starting 3D Transform IF Initialization", log_name_list = self.log_name_list)
        transform_ns = nepi_sdk.create_namespace(self.node_namespace,'idx')

        self.transform_if = Transform3DIF(namespace = transform_ns,
                        source_ref_description = self.tr_source_ref_description,
                        end_ref_description = self.tr_end_ref_description,
                        supports_updates = True,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )



        # Setup Settings IF Class ####################
        self.msg_if.pub_debug("Starting Settings IF Initialization", log_name_list = self.log_name_list)
        settings_ns = nepi_sdk.create_namespace(self.node_namespace,'idx')

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

        # Create a NPX Device IF
        if self.getNavPoseCb is not None:
            self.msg_if.pub_warn("Starting NPX Device IF Initialization", log_name_list = self.log_name_list)
            npx_if = NPXDeviceIF(device_info, 
                data_source_description = self.data_source_description,
                data_ref_description = self.data_ref_description,
                getNavPoseCb = self.getNavPoseCb,
                navpose_update_rate = self.navpose_update_rate,
                log_name_list = self.log_name_list,
                msg_if = self.msg_if
                )


        # Setup navpose data IF
        np_namespace = nepi_sdk.create_namespace(self.node_namespace,'idx')
        self.navpose_if = NavPoseIF(namespace = np_namespace,
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        log_name = 'navpose',
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )

        #############################
        # Finish Initialization

        self.initCb(do_updates = True)


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
        '''
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
        self.caps_report.data_products = self.data_products_base_list

        self.msg_if.pub_debug("Starting data products list: " + str(self.data_products_base_list))

        # Setup Save Data IF Class ####################
        self.data_products_save_list.append('navpose')
        self.msg_if.pub_debug("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
        factory_data_rates= {}
        for d in self.data_products_save_list:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if 'color_image' in self.data_products_save_list:
            factory_data_rates['color_image'] = [1.0, 0.0, 100.0] 
        

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_us': False,
            'suffix': "idx",
            'add_node_name': True
            }

        self.msg_if.pub_debug("Starting save_rate_dict: " + str(factory_data_rates))
        sd_namespace = nepi_sdk.create_namespace(self.node_namespace,'idx')
        self.save_data_if = SaveDataIF(data_products = self.data_products_save_list,
                                factory_rate_dict = factory_data_rates,
                                factory_filename_dict = factory_filename_dict,
                                namespace = sd_namespace,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )

        for data_product in self.data_products_base_list:
            self.last_data_time[data_product] = nepi_utils.get_time()
            self.current_fps[data_product] = 0
            self.fps_queue[data_product] = [0,0,0,0,0,0,0,0,0,0]

        # Launch the acquisition and saving threads
        if self.image_thread is not None:
            self.image_thread.start()

        if self.depth_map_thread is not None:
            self.depth_map_thread.start()

        if self.pointcloud_thread is not None:
            self.pointcloud_thread.start()

        # Init everything
        self.initCb(do_updates = True)
        self.publish_status(do_updates = True)


        ##################################
        # Start Node Processes
        nepi_sdk.start_timer_process(1, self._updaterCb, oneshot = True)
        nepi_sdk.start_timer_process(1, self._publishNavPoseCb, oneshot = True) 

        ####################################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ####################################

    def get_3d_transform(self):
        transform = nepi_nav.ZERO_TRANSFORM
        if self.transform_if is not None:
            transform = self.transform_if.get_3d_transform()
        return transform

    def get_navpose_dict(self):
        navpose_dict = None
        if self.connect_ptx_if is not None and self.pt_connected == True:
            navpose_dict = self.connect_ptx_if.get_navpose_dict()
            if navpose_dict is not None:
                frame_3d = 'nepi_frame'
                self.transform_if.set_end_ref_description('pantilt_center_tilt_axis')
        if navpose_dict is None:
            if self.nav_mgr_if is not None:
                navpose_dict = self.nav_mgr_if.get_navpose_data_dict()
                if navpose_dict is not None:
                    frame_3d = 'nepi_frame'
                    self.transform_if.set_end_ref_description('nepi_frame')
        if navpose_dict is None:
            navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
            frame_3d = 'sensor_frame'
            self.transform_if.set_end_ref_description('sensor_frame')

        self.end_ref_description = self.transform_if.get_end_ref_description()
        return navpose_dict

    def get_mount_description(self):
        desc = self.device_mount_description
        if self.mount_desc != 'None':
            desc = self.mount_desc
        return desc

        
    def publish_navpose(self):
        self.np_status_msg.publishing = True
        np_dict = self.get_navpose_dict()
        if self.navpose_if is not None:
            transform = self.get_3d_transform()
            np_dict = self.navpose_if.publish_navpose(np_dict,
                                            frame_3d_transform = transform,
                                            device_mount_description = self.get_mount_description())
        timestamp = nepi_utils.get_time()
        self.save_data_if.save('navpose',navpose_dict,timestamp = timestamp,save_check=True)


    def _publishNavPoseCb(self,timer):
        self.publish_navpose()
        rate = 1
        if self.nav_mgr_if is not None:
            rate = self.nav_mgr_if.get_pub_rate()
        delay = float(1.0) / rate
        nepi_sdk.start_timer_process(delay, self._publishNavPoseCb, oneshot = True)
 

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




    def initCb(self,do_updates = False):

        if self.node_if is not None:
            self.device_name = self.node_if.get_param('device_name')
            self.height_deg = self.node_if.get_param('width_deg')
            self.ctl_auto = self.node_if.get_param('height_deg')       
            self.ctl_brightness = self.node_if.get_param('brightness_ratio')
            self.ctl_contrast = self.node_if.get_param('contrast_ratio')        
            self.ctl_threshold = self.node_if.get_param('threshold_ratio')
            self.ctl_res_ratio = self.node_if.get_param('resolution_ratio')  
            self.ctr_start_range_ratio = self.node_if.get_param('start_range_ratio')
            self.ctr_stop_range_ratio = self.node_if.get_param('stop_range_ratio')

            self.pt_mounted = self.node_if.get_param('pt_mounted')
            self.pt_topic = self.node_if.get_param('pt_topic')

            self.mount_desc = self.node_if.get_param('mount_desc')

            if do_updates:
                self.ApplyConfigUpdates()
            self.publish_status()

    def resetCb(self,do_updates = True):  
        if node_if is not None:
            node_if.reset_params()
        if self.save_data_if is not None:
            self.save_data_if.reset()
        if self.settings_if is not None:
            self.settings_if.reset_settings(update_status = False, update_params = True)

        if self.getFOV is not None:
            try:
                [width_deg,height_deg] = self.getFOV()
                if node_if is not None:
                    self.node_if.set_param('width_deg',self.width_deg)
                    self.node_if.set_param('height_deg',self.height_deg) 
            except:
                pass

        self.initCb(do_updates = True)



    def factoryResetCb(self, do_updates = True):
        if node_if is not None:
            node_if.factory_reset_params()
        if self.save_data_if is not None:
            self.save_data_if.factory_reset()
        if self.settings_if is not None:
            self.settings_if.factory_reset(update_status = False, update_params = True)

        if self.getFOV is not None:
            try:
                [width_deg,height_deg] = self.getFOV()
                if node_if is not None:
                    self.node_if.set_param('width_deg',self.width_deg)
                    self.node_if.set_param('height_deg',self.height_deg) 
            except:
                pass

        self.initCb(do_updates = True)

    def ApplyConfigUpdates(self):
        if self.settings_if is not None:
            self.settings_if.reset_settings()
        param_dict = nepi_sdk.get_param('~', dict())
        self.msg_if.pub_debug("Applying Config Updates from Params: " + str(param_dict))
        if (self.setAutoAdjust is not None and 'auto_adjust_ebabled' in param_dict):
            self.setAutoAdjust(param_dict['auto_adjust_ebabled'])
        if (self.setBrightness is not None and 'brightness_ratio' in param_dict):
            self.setBrightness(param_dict['brightness_ratio'])
        if (self.setContrast is not None and 'contrast_ratio' in param_dict):
            self.setContrast(param_dict['contrast_ratio'])
        if (self.setThresholding is not None and 'threshold_ratio' in param_dict):
            self.setThresholding(param_dict['threshold_ratio'])
        if (self.setResolutionRatio is not None and 'resolution_ratio' in param_dict):
            self.setResolutionRatio(param_dict['resolution_ratio'])
        if (self.setFramerateRatio is not None and 'framerate_ratio' in param_dict):
            self.setFramerateRatio(param_dict['framerate_ratio'])
        if (self.setRange is not None and 'start_range' in param_dict and 'stop_range' in param_dict):
            self.setRange(param_dict['range_window']['start_range'], param_dict['range_window']['stop_range'])




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


         


    # Define local IDX Control callbacks
    def setControlsEnableCb(self, msg):
        self.msg_if.pub_info("Recived IDX Controls enable update message: " + str(msg))
        new_controls_enable = msg.data
        if self.setControlsEnable is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setControlsEnable(new_controls_enable)
        self.ctl_enabled = new_controls_enable
        self.status_msg.controls_enable = new_controls_enable
        self.publish_status(do_updates=False) # Updated inline here        
        self.node_if.set_param('controls_enable', new_controls_enable)


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
            
    def setAutoAdjustCb(self, msg):
        self.msg_if.pub_info("Recived Auto Adjust update message: " + str(msg))
        new_auto_adjust = msg.data
        if self.setAutoAdjust is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setAutoAdjust(new_auto_adjust)

        if new_auto_adjust:
            self.msg_if.pub_info("Enabling Auto Adjust", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Disabling IDX Auto Adjust", log_name_list = self.log_name_list)

        self.ctl_auto = new_auto_adjust       
        self.status_msg.auto_adjust_ebabled = new_auto_adjust
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('auto_adjust_ebabled', new_auto_adjust)




    def setBrightnessCb(self, msg):
        self.msg_if.pub_info("Recived Brightness update message: " + str(msg))
        new_brightness = msg.data
        if self.node_if.get_param('auto_adjust_ebabled'):
            self.msg_if.pub_info("Ignoring Set Brightness request. Auto Adjust enabled", log_name_list = self.log_name_list)
        else:
            if self.setBrightness is not None:

                if (new_brightness < 0.0 or new_brightness > 1.0):
                    self.msg_if.pub_error("Brightness value out of bounds", log_name_list = self.log_name_list)
                else:
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setBrightness(new_brightness)

        self.ctl_brightness = new_brightness
        self.status_msg.brightness_ratio = new_brightness
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('brightness_ratio', new_brightness)


    def setContrastCb(self, msg):
        self.msg_if.pub_info("Recived Contrast update message: " + str(msg))
        new_contrast = msg.data

        if (new_contrast < 0.0 and new_contrast != -1.0) or (new_contrast > 1.0):
            self.msg_if.pub_error("Contrast value out of bounds", log_name_list = self.log_name_list)
            self.publish_status(do_updates=False) # No change
            return

        if self.node_if.get_param('auto_adjust_ebabled'):
            self.msg_if.pub_info("Ignoring Set Contrast request. Auto Adjust enabled", log_name_list = self.log_name_list)
        else:
            if self.setContrast is not None:
                # Call the parent's method and update ROS param as necessary
                # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                status, err_str = self.setContrast(new_contrast)

        self.ctl_contrast = new_contrast     
        self.status_msg.contrast_ratio = new_contrast
        self.publish_status(do_updates=False) # Updated inline here   
 
        self.node_if.set_param('contrast_ratio', new_contrast)
        


    def setThresholdingCb(self, msg):
        self.msg_if.pub_info("Received Threshold update message: " + str(msg))
        new_threshold = msg.data

        if (new_threshold < 0.0 or new_threshold > 1.0):
            self.msg_if.pub_error("Thresholding value out of bounds", log_name_list = self.log_name_list)
            self.publish_status(do_updates=False) # No change
            return

        if self.setThresholding is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setThresholding(new_threshold)

        self.ctl_threshold = new_threshold
        self.status_msg.threshold_ratio = new_threshold
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('threshold_ratio', new_threshold)
        

    def setResolutionRatioCb(self, msg):
        self.msg_if.pub_info("Recived Resolution update message: " + str(msg))
        new_resolution = msg.data

        if (new_resolution < 0.0 or new_resolution > 1.0):
            self.msg_if.pub_error("Resolution value out of bounds", log_name_list = self.log_name_list)
            self.publish_status(do_updates=False) # No change
            return

        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            if self.setResolutionRatio is not None:
                status, err_str = self.setResolutionRatio(new_resolution)
        self.ctl_res_ratio = new_resolution
        self.status_msg.resolution_ratio = new_resolution
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('resolution_ratio', new_resolution)
        


        
    def setFramerateRatioCb(self, msg):
        self.msg_if.pub_info("Recived Framerate update message: " + str(msg))
        new_framerate = msg.data
 
        if new_framerate < 0.1:
            new_framerate = 0.1
        if new_framerate > 1.0:
            new_framerate = 1.0

        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        if self.setFramerateRatio is not None:
            status, err_str = self.setFramerateRatio(new_framerate)
        self.status_msg.framerate_ratio = new_framerate
        self.publish_status(do_updates=False) # Updated inline here
        for data_product in self.data_products_base_list:
            self.fps_queue[data_product] = [0,0,0,0,0,0,0,0,0,0]
        self.node_if.set_param('framerate_ratio', new_framerate)


 
    def setRangeCb(self, msg):
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
            if self.setRange is not None:
                status, err_str = self.setRange(new_start_range_ratio,new_stop_range_ratio)

        self.status_msg.range_window.start_range = new_start_range_ratio
        self.status_msg.range_window.stop_range = new_stop_range_ratio
        self.publish_status(do_updates=False) # Updated inline here  

        self.node_if.set_param('start_range_ratio', new_start_range_ratio)
        self.node_if.set_param('stop_range_ratio', new_stop_range_ratio)
     
    def setFrame3dCb(self, msg):
        self.msg_if.pub_info("Recived Output Frame 3D update message: " + str(msg))
        self.setFrame3d(msg.data)

    def setFrame3d(self, frame_3d):
        if frame_3d in FRAME_3D_OPTIONS:
            self.frame_3d = frame_3d
            self.publish_status(do_updates=False) # Updated inline here 
            self.node_if.set_param('frame_3d',  self.frame_3d)



    def setPtMountedCb(self,msg):
        self.msg_if.pub_info("Recived pt mounted message: " + str(msg))
        self.pt_mounted = msg.data
        self.publish_status(do_updates=False) # Updated inline here 
        self.node_if.set_param('pt_mounted', self.pt_mounted)

    def setPtTopicCb(self,msg):
        self.msg_if.pub_info("Recived pt topic message: " + str(msg))
        self.pt_topic = msg.data
        self.publish_status(do_updates=False) # Updated inline here 
        self.node_if.set_param('pt_mounted', self.pt_mounted)

    def setMountDescCb(self,msg):
        self.msg_if.pub_info("Recived set mount description message: " + str(msg))
        self.mount_desc = msg.data
        self.publish_status(do_updates=False) # Updated inline here 
        self.node_if.set_param('mount_desc', self.mount_desc)

    def resetMountDescCb(self,msg):
        self.msg_if.pub_info("Recived reset mount description message: " + str(msg))
        self.mount_desc = 'None'
        self.publish_status(do_updates=False) # Updated inline here 
        self.node_if.set_param('mount_desc', self.mount_desc)
        
   

    def resetControlsCb(self, msg):
        self.msg_if.pub_info("Recived reset controls message: " + str(msg))
        self.node_if.reset_param('device_name')
        self.node_if.reset_param('controls_enable')
        self.node_if.reset_param('auto_adjust_ebabled')       
        self.node_if.reset_param('brightness_ratio')
        self.node_if.reset_param('contrast_ratio')        
        self.node_if.reset_param('threshold_ratio')
        self.node_if.reset_param('resolution_ratio')   
        self.node_if.reset_param('framerate_ratio')
        self.node_if.reset_param('start_range_ratio')
        self.node_if.reset_param('stop_range_ratio')
        self.node_if.reset_param('mount_desc')
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
        self.node_if.factory_reset_param('framerate_ratio')
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
            self.current_fps[data_product] = sum(self.fps_queue[data_product])/len(self.fps_queue[data_product])
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
                dp_namespace = nepi_sdk.create_namespace(self.node_namespace,'idx')
                dp_if = ColorImageIF(namespace = dp_namespace, 
                            data_product_name = data_product, 
                            data_source_description = self.data_source_description,
                            data_ref_description = self.data_ref_description,
                            get_navpose_function = self.get_navpose_dict,
                            log_name = data_product,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )
            
            # Get Data Product Dict and Data_IF
            
            self.msg_if.pub_debug("Starting thread with data_product dict: " + data_product + " " + str(dp_dict))

            while (not nepi_sdk.is_shutdown() and dp_if is not None):
                dp_has_subs = dp_if.has_subscribers_check()
                dp_should_save = self.save_data_if.data_product_should_save(data_product)
                dp_should_save = dp_should_save or self.save_data_if.data_product_snapshot_enabled(data_product)

                # Get data if requried
                get_data = dp_has_subs or dp_should_save
                if get_data == True:
                    acquiring = True
                    status, msg, cv2_img, timestamp, encoding = dp_get_data()

                    if (status is False or cv2_img is None):
                        self.msg_if.pub_debug("No Data Recieved: " + data_product, throttle_s = 5.0)
                        pass
                    else:
                        self.msg_if.pub_debug("Got Data: " + data_product, throttle_s = 5.0)
                        
                        # Get Image Info and Pub Status if Changed
                        cur_width = self.width_px
                        cur_height = self.height_px
                        cv2_shape = cv2_img.shape
                        self.width_px = cv2_shape[1] 
                        self.height_px = cv2_shape[0]             
                        

                        # Now process and publish image
                        frame_3d = 'sensor_frame'
                        cv2_img = dp_if.publish_cv2_img(cv2_img, encoding = encoding,
                                                        timestamp = timestamp,
                                                        frame_3d = frame_3d,
                                                        width_deg = self.width_deg,
                                                        height_deg = self.height_deg,
                                                        do_subscriber_check = False,
                                                        device_mount_description = self.get_mount_description())
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
            min_range_m = self.status_msg.range_window.start_range
            max_range_m = self.status_msg.range_window.stop_range
            dp_namespace = nepi_sdk.create_namespace(self.node_namespace,'idx')
            dp_if = DepthMapIF(namespace = dp_namespace, 
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        enable_image_pub = True,
                        max_image_pub_rate = 5,
                        init_overlay_list = [],
                        log_name = data_product,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )
            ready = dp_if.wait_for_ready()
            dp_dict['dp_if'] = dp_if
            
            # Get Data Product Dict and Data_IF
            
            self.msg_if.pub_debug("Starting depth map thread with data_product dict: " + data_product + " " + str(dp_dict))

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
                        self.msg_if.pub_debug("No Data Recieved: " + data_product, throttle_s = 5.0)
                        pass
                    else:
                        self.msg_if.pub_debug("Got Data: " + data_product, throttle_s = 5.0)
                        
                        # Get Image Info and Pub Status if Changed
                        cur_width = self.width_px
                        cur_height = self.height_px
                        cv2_shape = cv2_img.shape
                        self.width_px = cv2_shape[1] 
                        self.height_px = cv2_shape[0] 


                        if (dp_has_subs == True):
                            #Publish Ros Image
                            min_range_m = self.status_msg.range_window.start_range
                            max_range_m = self.status_msg.range_window.stop_range
                            frame_3d = 'sensor_frame'
                            dp_if.publish_cv2_depth_map(cv2_img,
                                                    min_range_m = min_range_m,
                                                    max_range_m = max_range_m,
                                                    width_deg = self.width_deg,
                                                    height_deg = self.height_deg,
                                                    encoding = encoding,
                                                    timestamp = timestamp,
                                                    frame_3d = frame_3d,
                                                    device_mount_description = self.get_mount_description())
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
            dp_namespace = nepi_sdk.create_namespace(self.node_namespace,'idx')
            dp_if = PointcloudIF(namespace = dp_namespace, 
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        enable_image_pub = True,
                        max_image_pub_rate = 5,
                        init_overlay_list = [],
                        log_name = data_product,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )

            while (not nepi_sdk.is_shutdown()):
 
                # Get Data Product Dict and Data_IF
                dp_has_subs = dp_if.has_subscribers_check()
                dp_should_save = self.save_data_if.data_product_should_save(data_product)
                dp_should_save = dp_should_save or self.save_data_if.data_product_snapshot_enabled(data_product)

                # Get data if requried
                get_data = dp_has_subs or dp_should_save
                if get_data == True:
                    acquiring = True
                    status, msg, o3d_pc, timestamp, frame_3d = dp_get_data()
                    if o3d_pc is not None:
                        #********************
                        frame_3d = self.frame_3d
                        if frame_3d != 'sensor_frame':
                            transform = self.get_3d_transform()
                        if transform != self.ZERO_TRANSFORM:   
                            o3d_pc = self.transformPointcloud(o3d_pc,transform)
                        if frame_3d == 'world_frame':
                            pass #  Need to implement
                        
                        #********************

                        if (dp_has_subs == True):
                            min_range_m = self.status_msg.range_window.start_range
                            max_range_m = self.status_msg.range_window.stop_range
                            dp_if.publish_cv2_o3d_pc(o3d_pc,
                                                    min_range_m = min_range_m,
                                                    max_range_m = max_range_m,
                                                    width_deg = self.width_deg,
                                                    height_deg = self.height_deg,
                                                    encoding = encoding,
                                                    timestamp = timestamp,
                                                    frame_3d = frame_3d,
                                                    device_mount_description = self.get_mount_description())
                        if (dp_should_save == True ):
                            self.save_data_if.save(data_product,o3d_pc,timestamp = timestamp, save_check=False)

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


    def runImageThread(self):
        self.image_thread_proccess('color_image')

    def runDepthMapThread(self):
        self.depth_map_thread_proccess('depth_map')

    def runPointcloudThread(self):
        self.pointcloud_thread_proccess('pointcloud')

 

    # Function to update and publish status message

    def publish_status(self, do_updates = True):
        self.status_msg.device_name = self.device_name
        self.status_msg.device_mount_description = self.get_mount_description()

        self.status_msg.width_deg = self.width_deg
        self.status_msg.height_deg = self.height_deg

        self.status_msg.min_range_m = self.min_range_m
        self.status_msg.max_range_m = self.max_range_m
        
        self.status_msg.resolution_ratio = self.ctr_res_ratio
        res_str = str(self.width_px) + ":" + str(self.height_px)
        self.status_msg.resolution_current = res_str

        self.status_msg.framerate_ratio = self.ctr_fr_ratio
        self.status_msg.data_products = self.data_products_base_list
        framerates = []
        for dp in self.current_fps.keys():
            framerates.append(self.current_fps[dp])
        self.status_msg.framerates = framerates

        self.status_msg.auto_adjust_enabled = self.ctr_auto_ratio
        self.status_msg.contrast_ratio = self.ctr_contrast_ratio
        self.status_msg.brightness_ratio = self.ctr_brightness_ratio
        self.status_msg.threshold_ratio = self.ctr_threshold_ratio
        
        self.status_msg.range_window_ratios.start_range = self.ctr_start_range_ratio
        self.status_msg.range_window_ratios.stop_range =  self.ctr_stop_range_ratio




        self.status_msg.avail_pantilt_topics = self.avail_pts

        self.status_msg.pantilt_mounted = self.pt_mounted
        if self.pt_topic in self.avail_pts and self.pt_mounted == True:
            pt_topic = self.pt_topic
            pt_name = self.pt_topic.split('/ptx')[0]
            pt_name = pt_name.replace(self.base_namespace)
            if pt_name[0] == '/':
                pt_name = pt_name[1:]
            pt_connected = self.pt_connected
        else:
            pt_topic = 'None'
            pt_name = 'None'
            pt_connected = False
        self.status_msg.sel_pantilt_device_topic = pt_topic
        self.status_msg.sel_pantilt_name = pt_name
        self.status_msg.sel_pantilt_connected = pt_connected


        # The transfer frame into which 3D data (pointclouds) are transformed for the pointcloud data topic

        
        self.status_msg.output_frame_3d = self.frame_3d
        
        self.status_msg.frame_3d_transform = self.get_3d_transform_msg()

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

        #self.msg_if.pub_debug("Created status msg: " + str(self.status_msg), throttle_s = 5.0)
        self.node_if.publish_pub('status_pub',self.status_msg)
    

    def registerPTX(self,pt_topic):
        success = False
        if self.connect_ptx_if is not None and pt_topic in self.avail_pts:
            self.connect_ptx_if = ConnectPTXDeviceIF(pt_topic)
            self.device_mount_description = 'pan_tilt'
            success = True
        return success

    def unregisterPTX(self):
        success = True
        if self.connect_ptx_if is not None:
            try:
                self.connect_ptx_if.unregister()
                nepi_sdk.sleep(1)
                self.connect_ptx_if = None
                self.pt_connected = False
                self.device_mount_description = 'fixed'
            except:
                pass
        return success

    def getPtStatus(self,status_dict):
        self.pt_connected = True
        self.pt_pan_deg = status_dict['pan_now_deg']
        self.pt_tilt_deg = status_dict['tilt_now_deg']
        tr_msg = status_dict['frame_3d_transform']
        self.pt_transform = nepi_nav.convert_transform_msg2list(tr_msg)

    def updaterCb(self,timer):
        self.avail_pts = nepi_devices.get_ptx_device_namespaces()
        sel_pt = self.pt_topic
        if sel_pt == '' or sel_pt == 'None':
            success = self.unregisterPTX()
        if sel_pt != '' and sel_pt != 'None' and sel_pt not in self.avail_pts:
            success = self.unregisterPTX()
        if sel_pt != '' and sel_pt != 'None' and sel_pt in self.avail_pts:
            success = self.registerPTX()
        nepi_sdk.start_timer_process(1, self._updaterCb, oneshot = True)