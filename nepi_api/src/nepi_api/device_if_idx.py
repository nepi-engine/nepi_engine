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
import cv2
import open3d as o3d
import copy

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image, PointCloud2

from nepi_ros_interfaces.msg import IDXStatus, RangeWindow
from nepi_ros_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryRequest, IDXCapabilitiesQueryResponse
from nepi_ros_interfaces.msg import ImageStatus, PointcloudStatus
from nepi_ros_interfaces.msg import Frame3DTransform

from geometry_msgs.msg import Vector3

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SettingsIF, SaveDataIF

from nepi_api.data_if import ImageIF
from nepi_api.data_if import DepthMapIF
from nepi_api.data_if import PointcloudIF
from nepi_api.device_if_npx import NPXDeviceIF


SUPPORTED_DATA_PRODUCTS = ['2d_color_image','2d_sonar','2d_sonar_image','depth_map','depth_image','pointcloud','pointcloud_image']

#Factory Control Values 
DEFAULT_CONTROLS_DICT = dict( controls_enable = True,
    auto_adjust = False,
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
    frame_3d = 'nepi_center_frame'
    )
EXAMPLE_HEADING_DATA_DICT = {
    'time_heading': nepi_utils.get_time(),
    # Heading should be provided in Degrees True North
    'heading_deg': 120.50,
}

EXAMPLE_POSITION_DATA_DICT = {
    'time_position': nepi_utils.get_time(),
    # Position should be provided in Meters ENU (x,y,z) with x forward, y left, and z up
    'x_m': 1.234,
    'y_m': 1.234,
    'z_m': 1.234,
}

EXAMPLE_ORIENTATION_DATA_DICT = {
    'time_orientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees ENU
    'roll_deg': 30.51,
    'pitch_deg': 30.51,
    'yaw_deg': 30.51,
}

EXAMPLE_LOCATION_DATA_DICT = {
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'lat': 47.080909,
    'long': -120.8787889,
}

EXAMPLE_ALTITUDE_DATA_DICT = {
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters WGS84
    'altitude_m': 12.321,
}

EXAMPLE_ALTITUDE_DATA_DICT = {
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive distance from surface in meters
    'altitude_m': 0
}

EXAMPLE_NAVPOSE_DATA_DICT = {
    'frame_3d': 'ENU',
    'frame_altitude': 'WGS84',

    'geoid_height_meters': 0,

    'has_heading': True,
    'time_heading': nepi_utils.get_time(),
    # Heading should be provided in Degrees True North
    'heading_deg': 120.50,

    'has_position': True,
    'time_position': nepi_utils.get_time(),
    # Position should be provided in Meters in specified 3d frame (x,y,z) with x forward, y right/left, and z up/down
    'x_m': 1.234,
    'y_m': 1.234,
    'z_m': 1.234,

    'has_orientation': True,
    'time_oreientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees in specified 3d frame
    'roll_deg': 30.51,
    'pitch_deg': 30.51,
    'yaw_deg': 30.51,

    'has_location': True,
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'lat': 47.080909,
    'long': -120.8787889,

    'has_altitude': True,
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters in specified altitude frame
    'altitude_m': 12.321,

    'has_depth': False,
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive meters
    'depth_m': 0.0
}

class IDXDeviceIF:
    # Default Global Values
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]
    UPDATE_NAVPOSE_RATE_HZ = 10

    ZERO_TRANSFORM = [0,0,0,0,0,0,0]

    # Define class variables
    ready = False

    factory_device_name = None



    ctl_enabled = True
    ctl_auto = False
    ctl_brightness = 0.5
    ctl_contrast = 0.5
    ctl_threshold = 0.0
    ctl_res_ratio = 1.0  


    init_zoom_ratio = 0.5
    init_rotate_ratio = 0.5
    init_tilt_ratio = 0.5

    zoom_ratio = init_zoom_ratio
    rotate_ratio = init_rotate_ratio
    tilt_ratio = init_rotate_ratio
    render_controls = [zoom_ratio,rotate_ratio,tilt_ratio]

    
    data_products_list = []
    data_types_list = []


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

    npx_if = None
    
    fps_queue = dict()
    current_fps = dict()
    last_data_time = dict()

    image_thread = None
    depth_map_thread = None
    pointcloud_thread = None

    frame_transform = ZERO_TRANSFORM
    #######################
    ### IF Initialization
    def __init__(self, device_info, 
                 capSettings=None, factorySettings=None, 
                 settingUpdateFunction=None, getSettingsFunction=None,
                 factoryControls = None, setControlsEnable=None, setAutoAdjust=None,
                 get_rtsp_url = None,
                 setContrast=None, setBrightness=None, setThresholding=None,
                 setResolutionRatio=None, setFramerateRatio=None, 
                 setRange=None, getFramerate=None,
                 getImage=None, stopImageAcquisition=None, 
                 getBW2DImg=None, stopBW2DImgAcquisition=None,
                 getDepthMap=None, stopDepthMapAcquisition=None, 
                 getDepthImg=None, stopDepthImgAcquisition=None,
                 getPointcloud=None, stopPointcloudAcquisition=None, 
                 getPointcloudImg=None, stopPointcloudImgAcquisition=None,
                 capSettingsNavPose=None, factorySettingsNavPose=None, 
                 settingUpdateFunctionNavPose=None, getSettingsFunctionNavPose=None,
                 getHeadingCb = None, getPositionCb = None, getOrientationCb = None,
                 getLocationCb = None, getAltitudeCb = None, getDepthCb = None,
                 navpose_update_rate = 10,
                 data_products =  [],
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

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
        
        ############################# 
        # Initialize Class Variables
        self.sensor_name = device_info["sensor_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.factory_device_name = device_info["sensor_name"] + "_" + device_info["identifier"]


        data_products_list = []
        for data_product in data_products:
            if data_product in SUPPORTED_DATA_PRODUCTS:
                data_products_list.append(data_product)
        self.data_products = data_products_list
        self.msg_if.pub_warn("Enabled data products: " + str(self.data_products))
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
        
        # Set up standard IDX parameters with ROS param and subscriptions
        # Defer actually setting these on the camera via the parent callbacks... the parent may need to do some 
        # additional setup/calculation first. Parent can then get these all applied by calling ApplyConfigUpdates()

        self.get_rtsp_url = get_rtsp_url


        ## Set None Capabilities Variables
        self.setControlsEnable = setControlsEnable

        self.setAutoAdjust = setAutoAdjust
        self.capabilities_report.auto_adjustment = True

        self.setBrightness = setBrightness
        self.capabilities_report.adjustable_brightness = True

        self.setContrast = setContrast
        self.capabilities_report.adjustable_contrast = True

        self.setThresholding = setThresholding       
        self.capabilities_report.adjustable_thresholding = True

        self.setResolutionRatio = setResolutionRatio
        self.capabilities_report.adjustable_resolution = True

        self.setFramerateRatio = setFramerateRatio
        self.getFramerate = getFramerate
        self.capabilities_report.adjustable_framerate = True

        self.setRange = setRange
        self.capabilities_report.adjustable_range = True


        self.status_msg = IDXStatus()

        ##################################################
        ### Node Class Setup

        self.msg_if.pub_debug("Starting Node IF Initialization", log_name_list = self.log_name_list)
        # Configs Config Dict ####################
        self.CFGS_DICT = {
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
                'factory_val': self.factory_device_name
            },
            'controls_enable': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["controls_enable"]
            },
            'auto_adjust': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["auto_adjust"]
            },
            'brightness': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["brightness_ratio"]
            },
            'contrast': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["contrast_ratio"]
            },
            'thresholding': {
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
            'range_window/start_range_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["start_range_ratio"]
            },
            'range_window/stop_range_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["stop_range_ratio"]
            },
            'range_limits/min_range_m': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["min_range_m"]
            },
            'range_limits/max_range_m': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict["max_range_m"]
            },
            'frame_3d_transform': {
                'namespace': self.node_namespace,
                'factory_val': self.ZERO_TRANSFORM
            },
            'frame_3d': {
                'namespace': self.node_namespace,
                'factory_val': "sensor_frame"
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
                'msg': IDXStatus,
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
            'set_controls_enable': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_controls_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setControlsEnableCb, 
                'callback_args': ()
            },
            'set_auto_adjust': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_auto_adjust',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setAutoAdjustCb, 
                'callback_args': ()
            },
            'set_brightness': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_brightness',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setBrightnessCb, 
                'callback_args': ()
            },
            'set_contrast': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_contrast',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setContrastCb, 
                'callback_args': ()
            },
            'set_thresholding': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_thresholding',
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
            'clear_frame_3d_transform': {
                'namespace': self.node_namespace,
                'topic': 'idx/clear_frame_3d_transform',
                'msg': Bool,
                'qsize': 1,
                'callback': self.clearFrame3dTransformCb, 
                'callback_args': ()
            },
            'set_frame_3d_transform': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_frame_3d_transform',
                'msg': Frame3DTransform,
                'qsize': 1,
                'callback': self.setFrame3dTransformCb, 
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
                'topic': 'idx/reset_controls',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetControlsCb, 
                'callback_args': ()
            },
            'update_device_name': {
                'namespace': self.node_namespace,
                'topic': 'idx/update_device_name',
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
            'set_zoom_ratio': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_zoom_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setZoomCb, 
                'callback_args': ()
            },
            'set_rotate_ratio': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_rotate_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setRotateCb, 
                'callback_args': ()
            },
            'set_tilt_ratio': {
                'namespace': self.node_namespace,
                'topic': 'idx/set_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setTiltCb, 
                'callback_args': ()
            }
        }


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )

        ready = self.node_if.wait_for_ready()



        # Setup Settings IF Class ####################
        self.msg_if.pub_debug("Starting Settings IF Initialization", log_name_list = self.log_name_list)
        settings_ns = nepi_ros.create_namespace(self.node_namespace,'idx')

        self.SETTINGS_DICT = {
                    'capSettings': capSettings, 
                    'factorySettings': factorySettings,
                    'setSettingFunction': settingUpdateFunction, 
                    'getSettingsFunction': getSettingsFunction, 
                    'namespace':  settings_ns
        }

        self.settings_if = SettingsIF(self.SETTINGS_DICT, 
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )

        # Create a NavPose Device IF
        self.capSettingsNavPose = capSettingsNavPose
        self.factorySettingsNavPose=factorySettingsNavPose
        self.settingUpdateFunctionNavPose=settingUpdateFunctionNavPose 
        self.getSettingsFunctionNavPose=getSettingsFunctionNavPose

        self.getHeadingCb = getHeadingCb  
        self.getPositionCb = getPositionCb
        self.getOrientationCb = getOrientationCb
        self.getLocationCb = getLocationCb
        self.getAltitudeCb = getAltitudeCb
        self.getDepthCb = getDepthCb
        self.navpose_update_rate = navpose_update_rate

        has_navpose = ( getHeadingCb is not None or \
        getPositionCb is not None or \
        getOrientationCb is not None or \
        getLocationCb is not None or \
        getAltitudeCb is not None or \
        getDepthCb is not None )
        
        if has_navpose == True:
            self.msg_if.pub_warn("Starting NPX Device IF Initialization", log_name_list = self.log_name_list)
            npx_if = NPXDeviceIF(device_info, 
                capSettings = self.capSettingsNavPose,
                factorySettings = self.factorySettingsNavPose,
                settingUpdateFunction = self.settingUpdateFunctionNavPose, 
                getSettingsFunction = self.getSettingsFunctionNavPose,
                getHeadingCb = self.getHeadingCb, 
                getPositionCb = self.getPositionCb, 
                getOrientationCb = self.getOrientationCb,
                getLocationCb = self.getLocationCb, 
                getAltitudeCb = self.getAltitudeCb, 
                getDepthCb = self.getDepthCb,
                navpose_update_rate = self.navpose_update_rate,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )


        #############################
        # Finish Initialization

        self.ctl_enabled = self.node_if.get_param('controls_enable')
        self.ctl_auto = self.node_if.get_param('auto_adjust')       
        self.ctl_brightness = self.node_if.get_param('brightness')
        self.ctl_contrast = self.node_if.get_param('contrast')        
        self.ctl_threshold = self.node_if.get_param('thresholding')
        self.ctl_res_ratio = self.node_if.get_param('resolution_ratio')  

        self.self.frame_transform = self.node_if.get_param('frame_3d_transform')  

        # Start the data producers
        if (getImage is not None and '2d_color_image' in self.data_products):
            self.getImage = getImage
            self.stopImageAcquisition = stopImageAcquisition
            data_product = '2d_color_image'
            data_type = '2d_color_image'

            start_data_function = self.getImage
            stop_data_function = self.stopImageAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,data_type,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread", log_name_list = self.log_name_list)
            self.image_thread = threading.Thread(target=self.runImageThread)
            self.image_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.capabilities_report.has_image = True
        else:
            self.capabilities_report.has_image = False
        

        if (getDepthMap is not None and 'depth_map' in self.data_products):
            self.getDepthMap = getDepthMap
            self.stopDepthMapAcquisition = stopDepthMapAcquisition
            data_product = 'depth_map'
            data_type = 'depth_map'
            
            start_data_function = self.getDepthMap
            stop_data_function = self.stopDepthMapAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,data_type,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread", log_name_list = self.log_name_list)
            self.depth_map_thread = threading.Thread(target=self.runDepthMapThread)
            self.depth_map_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.capabilities_report.has_depth_map = True
        else:
            self.capabilities_report.has_depth_map = False
          

        if (getPointcloud is not None and 'pointcloud' in self.data_products):
            self.getPointcloud = getPointcloud
            self.stopPointcloudAcquisition = stopPointcloudAcquisition
            data_product = 'pointcloud'
            data_type = 'pointcloud'

            start_data_function = self.getPointcloud
            stop_data_function = self.stopPointcloudAcquisition
            data_msg = PointCloud2
            data_status_msg = PointcloudStatus

            success = self.addDataProduct2Dict(data_product,data_type,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread", log_name_list = self.log_name_list)
            self.pointcloud_thread = threading.Thread(target=self.runPointcloudThread)
            self.pointcloud_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.capabilities_report.has_pointcloud = True
        else:
            self.capabilities_report.has_pointcloud = False

        '''
        if (getPointcloudImg is not None and 'pointcloud' in self.data_products):
            self.getPointcloudImg = getPointcloudImg
            self.stopPointcloudImgAcquisition = stopPointcloudImgAcquisition
            data_product = 'pointcloud_image'
            data_type = 'pointcloud'

            start_data_function = self.getPointcloudImg
            stop_data_function = self.stopPointcloudImgAcquisition
            data_msg = Image
            data_status_msg = ImageStatus

            success = self.addDataProduct2Dict(data_product,data_type,start_data_function,stop_data_function,data_msg,data_status_msg)
            self.msg_if.pub_warn("Starting " + data_product + " acquisition thread", log_name_list = self.log_name_list)
            self.pointcloud_img_thread = threading.Thread(target=self.runPointcloudImgThread)
            self.pointcloud_img_thread.daemon = True # Daemon threads are automatically killed on shutdown

            self.capabilities_report.has_pointcloud_image = True

        else:
            pass
            self.capabilities_report.has_pointcloud_image = False
        '''

        self.capabilities_report.data_products = str(self.data_products_list)
        self.capabilities_report.data_product_types = str(self.data_types_list)

        self.msg_if.pub_debug("Starting data products list: " + str(self.data_products_list))
        self.msg_if.pub_debug("Starting data types list: " + str(self.data_types_list))

        # Setup Save Data IF Class ####################
        self.msg_if.pub_debug("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
        factory_data_rates= {}
        for d in self.data_products_list:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if 'image' in self.data_products_list:
            factory_data_rates['image'] = [1.0, 0.0, 100.0] 
        

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_us': False,
            'suffix': "",
            'add_node_name': True
            }

        self.msg_if.pub_debug("Starting save_rate_dict: " + str(factory_data_rates))
        sd_namespace = nepi_ros.create_namespace(self.node_namespace,'idx')
        self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                                factory_rate_dict = factory_data_rates,
                                factory_filename_dict = factory_filename_dict,
                                namespace = sd_namespace,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )

        for data_product in self.data_products_list:
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


        ####################################
        self.initCb(do_updates = True)
        self.publishStatus()
        self.ready = True
        self.msg_if.pub_info("IDX IF Initialization Complete", log_name_list = self.log_name_list)


    ###############################
    # Class Methods

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready   




    def initCb(self,do_updates = False):
        if self.settings_if is not None:
            self.settings_if.initialize_settings(do_updates)
        self.init_zoom_ratio = self.zoom_ratio
        self.init_rotate_ratio = self.rotate_ratio
        self.init_tilt_ratio = self.rotate_ratio
        if do_updates == True:
            self.resetCb(do_updates)


    def resetCb(self,do_updates = True):
        self.ctl_enabled = self.node_if.get_param('controls_enable')
        self.ctl_auto = self.node_if.get_param('auto_adjust')       
        self.ctl_brightness = self.node_if.get_param('brightness')
        self.ctl_contrast = self.node_if.get_param('contrast')        
        self.ctl_threshold = self.node_if.get_param('thresholding')
        self.ctl_res_ratio = self.node_if.get_param('resolution_ratio')  

        self.zoom_ratio = self.init_zoom_ratio
        self.rotate_ratio = self.init_rotate_ratio
        self.tilt_ratio = self.init_rotate_ratio
        self.render_controls = [self.zoom_ratio,self.rotate_ratio,self.tilt_ratio]
        if do_updates:
            self.ApplyConfigUpdates()
        self.publishStatus()



    def factoryResetCb(self, do_updates = True):
        self.ctl_enabled = self.node_if.get_param('controls_enable')
        self.ctl_auto = self.node_if.get_param('auto_adjust')       
        self.ctl_brightness = self.node_if.get_param('brightness')
        self.ctl_contrast = self.node_if.get_param('contrast')        
        self.ctl_threshold = self.node_if.get_param('thresholding')
        self.ctl_res_ratio = self.node_if.get_param('resolution_ratio')  

        self.zoom_ratio = self.init_zoom_ratio
        self.rotate_ratio = self.init_rotate_ratio
        self.tilt_ratio = self.init_rotate_ratio
        self.render_controls = [self.zoom_ratio,self.rotate_ratio,self.tilt_ratio]
        if self.settings_if is not None:
            self.settings_if.factory_reset_settings(update_status = False, update_params = True)
        if do_updates:
            self.ApplyConfigUpdates()
        self.publishStatus()

    def ApplyConfigUpdates(self):
        if self.settings_if is not None:
            self.settings_if.reset_settings()
        param_dict = nepi_ros.get_param('~', dict())
        self.msg_if.pub_debug("Applying Config Updates from Params: " + str(param_dict))
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




    def addDataProduct2Dict(self,data_product,data_type, start_data_function,stop_data_function,data_msg,data_status_msg):
        success = False
        data_product = data_product
        namespace = os.path.join(self.base_namespace,self.node_name,'idx')
        dp_dict = dict()
        dp_dict['data_product'] = data_product
        dp_dict['data_type'] = data_type
        dp_dict['namespace'] = namespace

        dp_dict['get_data'] = start_data_function
        dp_dict['stop_data'] = stop_data_function

        self.data_products_list.append(data_product)
        self.data_types_list.append(data_type)
        if data_type == 'depth_map':
            self.data_products_list.append(data_product + '_image')
            self.data_types_list.append('image')
        if data_type == 'pointcloud':
            self.data_products_list.append(data_product + '_image')
            self.data_types_list.append('image')
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
            self.publishStatus(do_updates=False) # Updated inline here 
            self.node_if.set_param('device_name', new_device_name)



    def resetDeviceNameCb(self,msg):
        self.msg_if.pub_info("Recived update message: " + str(msg))
        self.resetDeviceName()

    def resetDeviceName(self):
        self.status_msg.device_name = self.factory_device_name
        self.publishStatus(do_updates=False) # Updated inline here
        self.node_if.set_param('device_name', self.factory_device_name)
         


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
        self.publishStatus(do_updates=False) # Updated inline here        
        self.node_if.set_param('controls_enable', new_controls_enable)

 

            
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
        self.status_msg.auto_adjust = new_auto_adjust
        self.publishStatus(do_updates=False) # Updated inline here
        self.node_if.set_param('auto_adjust', new_auto_adjust)




    def setBrightnessCb(self, msg):
        self.msg_if.pub_info("Recived Brightness update message: " + str(msg))
        new_brightness = msg.data
        if self.node_if.get_param('auto_adjust'):
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
        self.status_msg.brightness = new_brightness
        self.publishStatus(do_updates=False) # Updated inline here
        self.node_if.set_param('brightness', new_brightness)


    def setContrastCb(self, msg):
        self.msg_if.pub_info("Recived Contrast update message: " + str(msg))
        new_contrast = msg.data

        if (new_contrast < 0.0 and new_contrast != -1.0) or (new_contrast > 1.0):
            self.msg_if.pub_error("Contrast value out of bounds", log_name_list = self.log_name_list)
            self.publishStatus(do_updates=False) # No change
            return

        if self.node_if.get_param('auto_adjust'):
            self.msg_if.pub_info("Ignoring Set Contrast request. Auto Adjust enabled", log_name_list = self.log_name_list)
        else:
            if self.setContrast is not None:
                # Call the parent's method and update ROS param as necessary
                # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                status, err_str = self.setContrast(new_contrast)

        self.ctl_contrast = new_contrast     
        self.status_msg.contrast = new_contrast
        self.publishStatus(do_updates=False) # Updated inline here   
 
        self.node_if.set_param('contrast', new_contrast)
        


    def setThresholdingCb(self, msg):
        self.msg_if.pub_info("Received Threshold update message: " + str(msg))
        new_thresholding = msg.data

        if (new_thresholding < 0.0 or new_thresholding > 1.0):
            self.msg_if.pub_error("Thresholding value out of bounds", log_name_list = self.log_name_list)
            self.publishStatus(do_updates=False) # No change
            return

        if self.setThresholding is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setThresholding(new_thresholding)

        self.ctl_threshold = new_thresholding
        self.status_msg.thresholding = new_thresholding
        self.publishStatus(do_updates=False) # Updated inline here
        self.node_if.set_param('thresholding', new_thresholding)
        

    def setResolutionRatioCb(self, msg):
        self.msg_if.pub_info("Recived Resolution update message: " + str(msg))
        new_resolution = msg.data

        if (new_resolution < 0.0 or new_resolution > 1.0):
            self.msg_if.pub_error("Resolution value out of bounds", log_name_list = self.log_name_list)
            self.publishStatus(do_updates=False) # No change
            return

        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            if self.setResolutionRatio is not None:
                status, err_str = self.setResolutionRatio(new_resolution)
        self.ctl_res_ratio = new_resolution
        self.status_msg.resolution_ratio = new_resolution
        self.publishStatus(do_updates=False) # Updated inline here
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
        self.publishStatus(do_updates=False) # Updated inline here
        for data_product in self.data_products_list:
            self.fps_queue[data_product] = [0,0,0,0,0,0,0,0,0,0]
        self.node_if.set_param('framerate_ratio', new_framerate)


 
    def setRangeCb(self, msg):
        self.msg_if.pub_info("Recived Range update message: " + str(msg))
        self.msg_if.pub_info("Recived update message: " + str(msg))
        new_start_range_ratio = msg.start_range
        new_stop_range_ratio = msg.stop_range
        if (new_start_range_ratio < 0 or new_stop_range_ratio > 1 or new_stop_range_ratio < new_start_range_ratio):
            self.msg_if.pub_error("Range values out of bounds", log_name_list = self.log_name_list)
            self.publishStatus(do_updates=False) # No change
            return
        else:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            if self.setRange is not None:
                status, err_str = self.setRange(new_start_range_ratio,new_stop_range_ratio)

        self.status_msg.range_window.start_range = new_start_range_ratio
        self.status_msg.range_window.stop_range = new_stop_range_ratio
        self.publishStatus(do_updates=False) # Updated inline here  

        self.node_if.set_param('range_window/start_range_ratio', new_start_range_ratio)
        self.node_if.set_param('range_window/stop_range_ratio', new_stop_range_ratio)
     

    def setZoomCb(self, msg):
        self.msg_if.pub_info("Recived Zoom update message: " + str(msg))
        new_zoom = msg.data
        if (new_zoom < 0.0 and new_zoom != -1.0) or (new_zoom > 1.0):
            self.msg_if.pub_error("Zoom value out of bounds", log_name_list = self.log_name_list)
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
            self.msg_if.pub_error("rotate value out of bounds", log_name_list = self.log_name_list)
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
            self.msg_if.pub_error("tilt value out of bounds", log_name_list = self.log_name_list)
            self.publishStatus(do_updates=False) # No change
            return
        else:
            self.tilt_ratio = new_tilt
            self.status_msg.tilt = new_tilt
            self.render_controls[2] = new_tilt
        self.publishStatus(do_updates=False) # Updated inline here  

    def setFrame3dTransformCb(self, msg):
        self.msg_if.pub_info("Recived Frame Transform update message: " + str(msg))
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
        self.frame_transform = [x,y,z,roll,pitch,yaw,heading]
        self.status_msg.frame_3d_transform = transform_msg
        self.publishStatus(do_updates=False) # Updated inline here 

        self.node_if.set_param('frame_3d_transform',  self.frame_transform)


    def clearFrame3dTransformCb(self, msg):
        self.msg_if.pub_info("Recived Clear 3D Transform update message: " + str(msg))
        new_transform_msg = msg
        self.clearFrame3dTransform()

    def clearFrame3dTransform(self, transform_msg):
        self.frame_transform = self.ZERO_TRANSFORM
        self.status_msg.frame_3d_transform = transform_msg
        self.publishStatus(do_updates=False) # Updated inline here 
        self.init_frame_3d_transform = self.node_if.set_param('frame_3d_transform',  self.frame_transform)


    def setFrame3dCb(self, msg):
        self.msg_if.pub_info("Recived Set 3D Transform update message: " + str(msg))
        new_frame_3d = msg.data
        self.setFrame3d(new_frame_3d)

    def setFrame3d(self, new_frame_3d):
        self.status_msg.frame_3d = new_frame_3d
        self.publishStatus(do_updates=False) # Updated inline here 
        self.node_if.set_param('frame_3d', new_frame_3d)

   
    def initConfig(self):
      self.initCb(do_updates = True)

    def resetControlsCb(self, msg):
        self.msg_if.pub_info("Recived reset controls message: " + str(msg))
        self.node_if.reset_param('device_name')
        self.node_if.reset_param('controls_enable')
        self.node_if.reset_param('auto_adjust')       
        self.node_if.reset_param('brightness')
        self.node_if.reset_param('contrast')        
        self.node_if.reset_param('thresholding')
        self.node_if.reset_param('resolution_ratio')   
        self.node_if.reset_param('framerate_ratio')
        self.node_if.reset_param('range_window/start_range_ratio')
        self.node_if.reset_param('range_window/stop_range_ratio')
        self.node_if.reset_param('frame_3d_transform')
        self.node_if.reset_param('frame_3d')
        self.resetCb(do_updates = True)


    def factoryResetControlsCb(self, msg):
        self.msg_if.pub_info("Recived factory reset controls message: " + str(msg))
        self.node_if.factory_reset_param('device_name')
        self.node_if.factory_reset_param('controls_enable')
        self.node_if.factory_reset_param('auto_adjust')       
        self.node_if.factory_reset_param('brightness')
        self.node_if.factory_reset_param('contrast')        
        self.node_if.factory_reset_param('thresholding')
        self.node_if.factory_reset_param('resolution_ratio')   
        self.node_if.factory_reset_param('framerate_ratio')
        self.node_if.factory_reset_param('range_window/start_range_ratio')
        self.node_if.factory_reset_param('range_window/stop_range_ratio')
        self.node_if.factory_reset_param('frame_3d_transform')
        self.node_if.factory_reset_param('frame_3d')
        self.factoryResetCb(do_updates = True)




    def provide_capabilities(self, _):
        return self.capabilities_report
    
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
                self.publishStatus()
  
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

            pub_namespace = nepi_ros.create_namespace(dp_dict['namespace'],data_product)
            #img_pub = nepi_ros.create_publisher(pub_namespace, Image, queue_size = 10)

            dp_if = ImageIF(namespace = pub_namespace,log_name = data_product,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )
            
            # Get Data Product Dict and Data_IF
            
            self.msg_if.pub_debug("Accessing data_product dict: " + data_product + " " + str(dp_dict))

            while (not nepi_ros.is_shutdown()):
                dp_has_subs = dp_if.has_subscribers_check()
                dp_should_save = self.save_data_if.data_product_should_save(data_product)
                dp_should_save = dp_should_save or self.save_data_if.data_product_snapshot_enabled(data_product)

                # Get data if requried
                get_data = dp_has_subs or dp_should_save
                if get_data == True:
                    acquiring = True
                    if data_product != "pointcloud_image":
                        status, msg, cv2_img, timestamp, encoding = dp_get_data()
                    else:
                        status, msg, cv2_img, timestamp, encoding = dp_get_data(self.render_controls)
                    if (status is False or cv2_img is None):
                        self.msg_if.pub_debug("No Data Recieved: " + data_product)
                        pass
                    else:
                        self.msg_if.pub_debug("Got Data: " + data_product)
                        
                        # Get Image Info and Pub Status if Changed
                        cur_width = self.img_width
                        cur_height = self.img_height
                        cv2_shape = cv2_img.shape
                        self.img_width = cv2_shape[1] 
                        self.img_height = cv2_shape[0] 


                        
                        #############################
                        # Apply IDX Post Processing
                        
                        enabled = self.ctl_enabled
                        auto = self.ctl_auto      
                        brightness = self.ctl_brightness
                        contrast = self.ctl_contrast
                        threshold = self.ctl_threshold
                        res_ratio = self.ctl_res_ratio   
                        self.msg_if.pub_debug("Applying resolution ratio: " + data_product + " " + str(res_ratio))
                        
                        if enabled == True: 
                            if res_ratio < 0.9:
                                [cv2_img,new_res] = nepi_img.adjust_resolution_ratio(cv2_img, res_ratio)
                            if data_product != 'depth_map' and data_product != 'depth_image' and data_product != 'pointcloud_image':
                                if auto is False:
                                    cv2_img = nepi_img.adjust_brightness(cv2_img, brightness)
                                    cv2_img = nepi_img.adjust_contrast(cv2_img, contrast)
                                    cv2_img = nepi_img.adjust_sharpness(cv2_img, threshold)
                                else:
                                    cv2_img = nepi_img.adjust_auto(cv2_img,0.3)


                        if (dp_has_subs == True):
                            #Publish Ros Image
                            frame_id = self.node_if.get_param('frame_3d')
                            dp_if.publish_cv2_img(cv2_img, encoding = encoding, timestamp = timestamp, frame_id = frame_id)
                        if (dp_should_save == True):
                            self.save_data_if.save(data_product,cv2_img,timestamp = timestamp,save_check=False)


                        self.update_fps(data_product)

                        self.msg_if.pub_debug("Got cv2_img size: " + str(self.img_width) + ":" + str(self.img_height))
                        if cur_width != self.img_width or cur_height != self.img_height:
                            self.publishStatus()

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition", log_name_list = self.log_name_list)
                        dp_stop_data()
                    acquiring = False
                    self.current_fps[data_product] = 0.0
                    self.fps_queue[data_product] = [0,0,0,0,0,0,0,0,0,0]
                else: # No subscribers and already stopped
                    acquiring = False
                    nepi_ros.sleep(0.25)
                self.msg_if.pub_debug("Ending with avg fps: " + str(self.current_fps[data_product]))    
                nepi_ros.sleep(0.01) # Yield


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

            pub_namespace = nepi_ros.create_namespace(dp_dict['namespace'],data_product)
            #img_pub = nepi_ros.create_publisher(pub_namespace, Image, queue_size = 10)
            min_range_m = self.status_msg.range_window.start_range
            max_range_m = self.status_msg.range_window.stop_range
            dp_if = DepthMapIF(namespace = pub_namespace,
                        default_min_meters = min_range_m,
                        default_max_meters = max_range_m,
                        enable_image_pub = True,
                        max_image_pub_rate = 5,
                        init_overlay_list = [],
                        log_name = data_product,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )
            
            # Get Data Product Dict and Data_IF
            
            self.msg_if.pub_debug("Accessing data_product dict: " + data_product + " " + str(dp_dict))

            while (not nepi_ros.is_shutdown()):
                dp_has_subs = dp_if.has_subscribers_check()
                dp_should_save = self.save_data_if.data_product_should_save(data_product)
                dp_should_save = dp_should_save or self.save_data_if.data_product_snapshot_enabled(data_product)

                # Get data if requried
                get_data = dp_has_subs or dp_should_save
                if get_data == True:
                    acquiring = True

                    status, msg, cv2_img, timestamp, encoding = dp_get_data()
                    if (status is False or cv2_img is None):
                        self.msg_if.pub_debug("No Data Recieved: " + data_product)
                        pass
                    else:
                        self.msg_if.pub_debug("Got Data: " + data_product)
                        
                        # Get Image Info and Pub Status if Changed
                        cur_width = self.img_width
                        cur_height = self.img_height
                        cv2_shape = cv2_img.shape
                        self.img_width = cv2_shape[1] 
                        self.img_height = cv2_shape[0] 


                        if (dp_has_subs == True):
                            #Publish Ros Image
                            min_range_m = self.status_msg.range_window.start_range
                            max_range_m = self.status_msg.range_window.stop_range
                            frame_id = self.node_if.get_param('frame_3d')
                            dp_if.publish_cv2_depth_map(cv2_img,min_range_m = min_range_m, max_range_m = max_range_m, encoding = encoding, timestamp = timestamp, frame_id = frame_id)
                        if (dp_should_save == True):
                            self.save_data_if.save(data_product,cv2_img,timestamp = timestamp,save_check=False)


                        self.update_fps(data_product)

                        self.msg_if.pub_debug("Got cv2_img size: " + str(self.img_width) + ":" + str(self.img_height))
                        if cur_width != self.img_width or cur_height != self.img_height:
                            self.publishStatus()

                elif acquiring is True:
                    if dp_stop_data is not None:
                        self.msg_if.pub_info("Stopping " + data_product + " acquisition", log_name_list = self.log_name_list)
                        dp_stop_data()
                    acquiring = False
                    self.current_fps[data_product] = 0.0
                    self.fps_queue[data_product] = [0,0,0,0,0,0,0,0,0,0]
                else: # No subscribers and already stopped
                    acquiring = False
                    nepi_ros.sleep(0.25)
                self.msg_if.pub_debug("Ending with avg fps: " + str(self.current_fps[data_product]))    
                nepi_ros.sleep(0.01) # Yield


              

   
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

            pub_namespace = nepi_ros.create_namespace(dp_dict['namespace'],data_product)
            #img_pub = nepi_ros.create_publisher(pub_namespace, Image, queue_size = 10)

            dp_if = PointcloudIF(namespace =  pub_namespace,log_name = data_product,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )

            while (not nepi_ros.is_shutdown()):
 
                # Get Data Product Dict and Data_IF
                dp_has_subs = dp_if.has_subscribers_check()
                dp_should_save = self.save_data_if.data_product_should_save(data_product)
                dp_should_save = dp_should_save or self.save_data_if.data_product_snapshot_enabled(data_product)

                # Get data if requried
                get_data = dp_has_subs or dp_should_save
                if get_data == True:
                    acquiring = True
                    status, msg, o3d_pc, timestamp, frame_id = dp_get_data()
                    if o3d_pc is not None:


                        #********************
                        set_frame = self.status_msg.frame_3d
                        if set_frame == 'sensor_frame':
                            frame_id = set_frame # else pass through sensor frame
                        else:
                            frame_id = 'base_frame'

                        transform = self.frame_transform
                        zero_transform = True
                        for i in range(len(transform)):
                            if transform[i] != 0:
                                zero_transform = False
                        should_transform = (zero_transform == False) and (frame_id == 'nepi_center_frame')
                        if should_transform:   
                            o3d_pc = self.transformPointcloud(o3d_pc,transform)

                        #********************
                        if (dp_has_subs == True):
                            dp_if.publish_o3d_pc(o3d_pc, timestamp = timestamp, frame_id = frame_id )
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
                    nepi_ros.sleep(0.25)
                nepi_ros.sleep(0.01) # Yield
                




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
        self.image_thread_proccess('image')

    def runDepthMapThread(self):
        self.depth_map_thread_proccess('depth_map')

    def runPointcloudThread(self):
        self.pointcloud_thread_proccess('pointcloud')

 

    # Function to update and publish status message

    def publishStatus(self, do_updates = True):
        # TODO: Probably these should be queried from the parent (and through the driver) via explicit callbacks rather than via the param server
        if do_updates == True:
            param_dict = nepi_ros.get_param('~')

            self.status_msg.device_name = param_dict['device_name'] if 'device_name' in param_dict else self.init_device_name
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


            
            self.status_msg.resolution_ratio = param_dict['resolution_ratio'] if 'resolution_ratio' in param_dict else 0
            res_str = str(self.img_width) + ":" + str(self.img_height)
            self.status_msg.resolution_current = res_str

            self.status_msg.framerate_ratio = param_dict['framerate_ratio'] if 'framerate_ratio' in param_dict else 0
            self.status_msg.data_products = self.data_products_list
            self.status_msg.data_product_types = self.data_types_list
            framerates = []
            for dp in self.current_fps.keys():
                framerates.append(self.current_fps[dp])
            self.status_msg.framerates = framerates


            self.status_msg.controls_enable = param_dict['controls_enable'] if 'controls_enable' in param_dict else True
            self.status_msg.auto_adjust = param_dict['auto_adjust'] if 'auto_adjust' in param_dict else False
            self.status_msg.contrast = param_dict['contrast'] if 'contrast' in param_dict else 0
            self.status_msg.brightness = param_dict['brightness'] if 'brightness' in param_dict else 0
            self.status_msg.thresholding = param_dict['thresholding'] if 'thresholding' in param_dict else 0
            
            self.status_msg.range_window.start_range = self.node_if.get_param('range_window/start_range_ratio')
            self.status_msg.range_window.stop_range =  self.node_if.get_param('range_window/stop_range_ratio')
            self.status_msg.min_range_m = self.node_if.get_param('range_limits/min_range_m')
            self.status_msg.max_range_m = self.node_if.get_param('range_limits/max_range_m')
            # The transfer frame into which 3D data (pointclouds) are transformed for the pointcloud data topic
            transform = self.node_if.get_param('frame_3d_transform')
            transform_msg = Frame3DTransform()
            transform_msg.translate_vector.x = transform[0]
            transform_msg.translate_vector.y = transform[1]
            transform_msg.translate_vector.z = transform[2]
            transform_msg.rotate_vector.x = transform[3]
            transform_msg.rotate_vector.y = transform[4]
            transform_msg.rotate_vector.z = transform[5]
            transform_msg.heading_offset = transform[6]
            self.status_msg.frame_3d_transform = transform_msg
            
            self.status_msg.frame_3d = param_dict['frame_3d'] if 'frame_3d' in param_dict else "nepi_center_frame"
            self.status_msg.zoom = self.zoom_ratio
            self.status_msg.rotate = self.rotate_ratio
            self.status_msg.tilt = self.tilt_ratio
            self.msg_if.pub_debug("Got status msg: " + str(self.status_msg))
        self.node_if.publish_pub('status_pub',self.status_msg)
    
