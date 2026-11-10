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
import numpy as np
import copy
import threading
import math
import cv2



from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_controls

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped



from nepi_interfaces.msg import ColorBGR, ImageStatus, MgrSystemStatus
from nepi_interfaces.msg import DepthMapStatus
from nepi_interfaces.msg import IntensityMapStatus
from nepi_interfaces.msg import PointcloudStatus

from nepi_interfaces.msg import NavPoseTrack



from nepi_interfaces.msg import UpdateOrder, UpdateFloat, UpdateFloats, UpdateInt, UpdateInts, UpdateBool, UpdateBools, UpdateString, UpdateStringArray, UpdateTrigger
from nepi_interfaces.msg import StringArray, ImageWindow, RangeWindow, ImagePixel, ImageMouseEvent, ImageCrosshair, ImageTarget

from nepi_interfaces.msg import RangeWindow
from nepi_interfaces.msg import ImageSize

from sensor_msgs.msg import PointCloud2

from nepi_api.data_product_if import DataProductIF

from nepi_api.utils_if import ControlsIF

SYSTEM_ALL_TOPIC = 'all'




##################################################
# BaseImageIF

IMAGE_ALL_TOPIC = 'images'

ENCODING_OPTIONS = ["mono8",'rgb8','bgr8','32FC1','passthrough']

PERSPECTIVE_OPTIONS = ['pov','top']

EXAMPLE_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = False,
        has_flip_horz = False,
        has_flip_vert = False,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_zoom_3d = False,
        has_rotate_3d = False,
        has_tilt_3d = False,
        has_camera_3d = False
    )

EXAMPLE_FILTERS_DICT = dict(
    # Low_Light = {
    #     'enabled': False,
    #     'function': nepi_img.low_light_filter,
    #     'ratio': 0.5
    # }
)

EXAMPLE_CONTROLS_DICT = dict( 
    resolution_ratio = 1.0,
    auto_adjust_enabled = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.0,
    start_range_ratio = 0.0,
    stop_range_ratio = 1.0,
    window_ratios = [0,1,0,1],
    rotate_3d_ratio = 0.5,
    tilt_3d_ratio = 0.5       
    )

EXAMPLE_CALLBACK_DICT = dict(
    needs_update_callback = None,
    mouse_event_callback = None,
    click_pixel_callback = None,
    click_angle_callback = None,
    drag_callback = None,
    window_callback = None,
    scroll_callback = None,
    frame_updated_callback = None,
    options_updated_callback = None,
    get_source_status_callback = None,
    get_source_navpose_callback = None,
    pre_process_callback = None,
    post_process_callback = None
)

class BaseImageIF(DataProductIF):

    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = ImageStatus()
    DATA_MSG = Image()

    ID = 'image'
    DISPLAY_NAME = 'Base Image'
    DESCRIPTION = 'Base Image Data Product'

    DATA_PRODUCT = 'image'

    #########################
    ## Managed by NodeIF
    #########################

    # id = ''
    # name = ''
    # display_name = ''
    # namespace = '~'

    # status_msg = None
    # data_msg = None

    # msg_if = None
    # node_if = None
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
    # data_product = 'data_product'
    # data_source_description = 'sensor'
    # data_ref_description = 'sensor'

    # save_config_needed = False

    # needs_data = False
    # time_list = [0,0,0,0,0,0,0,0,0,0]
    # last_pub_time = None
    # publishing = False

    #########################
    ## CUSTOM
    #########################

    all_namespace = None

    image_topic = ""
    depth_map_topic = ""
    pointcloud_topic = ""

    options_controls_if = dict()

    ready = False

    DEFUALT_IMG_WIDTH_PX = 700
    DEFUALT_IMG_HEIGHT_PX = 400

    DEFAULT_WIDTH_DEG = 100
    DEFAULT_HEIGHT_DEG = 70

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = False,
        has_flip_horz = False,
        has_flip_vert = False,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_zoom_3d = False,
        has_rotate_3d = False,
        has_tilt_3d = False,
        has_camera_3d = False
        )

    DEFAULT_FILTERS_DICT = dict()

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        rotate_2d_deg = 0,
        rotate_2d_keep_size = False,
        rotate_2d_swap_box = False,
        flip_horz = False,
        flip_vert = False,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        zoom_3d_ratio = 0,
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5,
        cam_fov = 60,
        cam_view = [3, 0, 0],
        cam_pos = [-5, 0, 0],
        cam_rot = [0, 0, 1]
        )




    DEFAULT_CALLBACK_DICT = dict(
        needs_update_callback = None,
        mouse_event_callback = None,
        click_pixel_callback = None,
        click_angle_callback = None,
        drag_callback = None,
        window_callback = None,
        scroll_callback = None,
        frame_updated_callback = None,
        options_updated_callback = None,
        get_source_status_callback = None,
        get_source_navpose_callback = None,
        pre_process_callback = None,
        post_process_callback = None
    )

    BLANK_CROSSHAIR_DICT = dict(
        x_deg_offset = 0,
        y_deg_offset = 0,
        color_rgb = (0,255,0),
        msg_str = ''
    )

    BLANK_TARGET_DICT = dict(
        x_deg_offset = 0,
        y_deg_offset = 0,
        color_rgb = (0,255,0),
        msg_str = ''
    )

    DEFAULT_OVERLAYS_DICT = dict(
            overlay_text_enabled = False,
            
            overlay_text_size_ratio = 0.5,
            overlay_text_horz_ratio = 0.0,
            overlay_text_vert_ratio = 0.0,
            overlay_text_transparency_ratio = 0.0,
            overlay_text_color_rgb = (0,255,0),
            overlay_text_img_name = False,
            overlay_text_date_time = False,
            overlay_text_nav = False,
            overlay_text_pose = False, 
            init_overlay_text_list = [],
            add_overlay_text_list = [],

            crosshairs_enabled = False,
            crosshairs_size_ratio = 0.5,
            crosshairs_thickness_ratio = 0.5,
            crosshairs_text_ratio = 0.5,
            crosshairs_transparency_ratio = 0.0,
            crosshairs_color_rgb = (0,255,0),
            overlay_crosshair_names = False,
            overlay_crosshair_pixels = False,
            overlay_crosshair_degrees = False,
            overlay_crosshair_text = False,
            crosshairs_dict = dict(),
            
            targets_enabled = False,
            targets_size_ratio = 0.5,
            targets_thickness_ratio = 0.5,
            targets_text_ratio = 0.5,
            targets_transparency_ratio = 0.0,
            targets_color_rgb = (0,255,0),
            overlay_target_names = False,
            overlay_target_pixels = False,
            overlay_target_degrees = False,
            overlay_target_text = False,
            targets_dict = dict(),
    )

    controls_dict = copy.deepcopy(DEFAULT_CONTROLS_DICT)
    init_controls_dict = copy.deepcopy(DEFAULT_CONTROLS_DICT)

    # Enable state for 3D render controls (mouse drag/window interaction)
    render_3d_controls_enabled = False

    last_width = DEFUALT_IMG_WIDTH_PX
    last_height = DEFUALT_IMG_HEIGHT_PX

    min_range_m = 0
    max_range_m = 0

    perspective = 'pov'

    blank_img = nepi_img.create_cv2_blank_img(last_width, last_height, color_rgb = (0, 0, 0) )

    last_pub_time = None

    click_text_enabled = False
    click_crosshair_enabled = False
    click_target_enabled = False
    overlays_dict = copy.deepcopy(DEFAULT_OVERLAYS_DICT) 

    auto_adjust_controls = []
    filter_dict = dict()
    has_filters = False
    filter_options = []
    sel_filters = []

    pixel = None
    window = None

    drag_pixel = None
    drag_window = None

    x_offset = 0
    y_offset = 0
    x_scaler = 1
    y_scaler = 1

    height_org = 0
    width_org = 0

    height_proc = 0
    width_proc = 0

    width_start_deg = 0
    width_stop_deg = DEFAULT_WIDTH_DEG
    height_start_deg = 0
    height_stop_deg = DEFAULT_HEIGHT_DEG
    width_deg = DEFAULT_WIDTH_DEG
    height_deg = DEFAULT_HEIGHT_DEG

    zoom_ratio = 1
    x_ratio = 0.5
    y_ratio = 0.5
    window_ratios = [0,1,0,1]

    publishing = False

    add_pubs_dict = dict()
    pub_count = 0
    pubs_dict = dict()
    subs_dict = dict()

    aspect_adjustment_disabled = False
    aspect_adjust_enabled = False
    aspect_ratio_set = 1.78 # 16:9
    aspect_ratio = -1 # Not Known Yet
    aspect_x_offset = 0
    aspect_y_offset = 0

    live_adjustments_disabled = False
    live_adjust_enabled = True
    live_adjust_dict = dict(
        live_adjust_enabled = True,
        live_adjust_rotate_deg = 0,
        live_adjust_x_deg = 0,
        live_adjust_y_deg = 0,
    )

    stabilize_enabled = False
    stabilize_rotate_deg = 0

    stream_compression_enabled = False
    stream_compression_ratio = 0.5

    source_status_dict = None
    source_navpose_dict = None

    

    def __init__(self, 
                name,
                namespace , 
                data_product,
                data_source_description,
                data_ref_description,
                perspective,
                caps_dict,
                controls_dict, 
                filter_dict,
                options_dict,
                callback_dict,
                init_overlay_text_list,
                live_adjustments_disabled,
                aspect_adjustment_disabled,
                info_dict,
                msg_if,
                node_if,
                save_data_if,
                transform_if,
                navpose_if,
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.caps_dict = copy.deepcopy(self.caps_dict)
        self.overlays_dict = copy.deepcopy(self.overlays_dict)
        self.add_pubs_dict = copy.deepcopy(self.add_pubs_dict)
        self.live_adjust_dict = copy.deepcopy(self.live_adjust_dict)


        ##############################
        # Update Class Variables
        ##############################

        name = nepi_utils.get_clean_name(name)
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product.strip() != '':
                name = data_product



        ##############################  
        # Setup All Namespace

        # Collective controls publish on the shared images all_namespace, which fans a
        # single command out to every IDX image.
        all_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(), SYSTEM_ALL_TOPIC)
        all_namespace = nepi_sdk.create_namespace(all_namespace, IMAGE_ALL_TOPIC)
        self.all_namespace = all_namespace

        ##############################  
        # Update capabilities dictionary
        if caps_dict is not None:
            for cap in self.caps_dict.keys():
                if caps_dict.get(cap) != None:
                    self.caps_dict[cap] = caps_dict[cap]
          

        #############################
        # Create Image Options Controls         

        ##############################  
        # Update Controls Init Dict

        # Give this instance its own controls store before anything writes to it.
        # The class attributes controls_dict and init_controls_dict are both plain
        # references to BaseImageIF.DEFAULT_CONTROLS_DICT, so without this every
        # image IF in the process mutates one shared dict (color, depth map and
        # pointcloud controls bleed into each other), init_controls_dict tracks the
        # live values instead of the baseline (which makes every reset_* a no-op),
        # and the class-level defaults the pointcloud renderer falls back to get
        # overwritten as controls change. Base is the floor because the subclass
        # dicts omit keys Base code reads: PointcloudImageIF has no rotate_2d_deg
        # or flip_horz/flip_vert, so seeding from the subclass alone would KeyError.
        merged_controls_dict = copy.deepcopy(BaseImageIF.DEFAULT_CONTROLS_DICT)
        merged_controls_dict.update(copy.deepcopy(self.DEFAULT_CONTROLS_DICT))
        self.controls_dict = merged_controls_dict

        # Create and update controls dictionary
        if controls_dict is not None:
            for control in self.controls_dict.keys():
                if controls_dict.get(control) != None:
                    self.controls_dict[control] = controls_dict[control]
        self.init_controls_dict = copy.deepcopy(self.controls_dict)

 
        self.overlays_dict['init_overlay_text_list'] = init_overlay_text_list

        #########################
        # Update Class Data
        ##############################

        if perspective is not None:
            self.perspective = perspective

        # Setup filter dict
        if filter_dict is not None:
            self.filter_dict = filter_dict
            self.filter_options = list(self.filter_dict.keys())
            if len(self.filter_options) > 0:
                self.has_filters = True
        else:
            self.filter_dict = dict()


        self.live_adjustments_disabled = live_adjustments_disabled == True
        #self.msg_if.pub_debug(self.data_product + " Got Live Adjust Disabled: " + str(self.live_adjustments_disabled ) )
        self.aspect_adjustment_disabled = aspect_adjustment_disabled == True
        #self.msg_if.pub_debug(self.data_product + " Got Aspect Adjust Disabled: " + str(self.aspect_adjustment_disabled ) )

 

        ##############################  
        # Setup NodeIF Updates
        ##############################  

        ##################
        update_info_dict = info_dict
        ##################        
        update_caps_dict = copy.deepcopy(self.DEFAULT_CAPS_DICT)
        try:
            update_caps_dict.update(caps_dict)
        except:
            pass
        ##################
        update_stats_dict = dict()
        ##################
        update_rui_dict = dict()
        ##################       
        update_config_control_dict = dict()
        ##################       
        update_node_controls_dict = dict()

        update_callback_dict = copy.deepcopy(self.DEFAULT_CALLBACK_DICT)
        try:
            update_callback_dict.update(callback_dict)
        except:
            pass
        ################## 

        ###########
        # NODE IF SETUP

        # Call the parent class constructor
        super().__init__(
                name = name,
                namespace = namespace,
                data_product = data_product,
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
                save_data_if = save_data_if,
                transform_if = transform_if,
                navpose_if = navpose_if,
                )

        ready = self.wait_for_ready()


       #######################
        # Add Services
        #######################
       
        update_params_dict = {

            'resolution_ratio': {
                'name': 'resolution_ratio',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["resolution_ratio"]
            },
            'auto_adjust_enabled': {
                'name': 'auto_adjust_enabled',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["auto_adjust_enabled"]
            },
            'auto_adjust_ratio': {
                'name': 'auto_adjust_ratio',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["auto_adjust_ratio"]
            },
            'brightness_ratio': {
                'name': 'brightness_ratio',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["brightness_ratio"]
            },
            'contrast_ratio': {
                'name': 'contrast_ratio',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["contrast_ratio"]
            },
            'threshold_ratio': {
                'name': 'threshold_ratio',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["threshold_ratio"]
            },
            'rotate_2d_deg': {
                'name': 'rotate_2d_deg',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["rotate_2d_deg"]
            },
            'rotate_2d_swap_box': {
                'name': 'rotate_2d_swap_box',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["rotate_2d_swap_box"]
            },
            'flip_horz': {
                'name': 'flip_horz',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["flip_horz"]
            },
            'flip_vert': {
                'name': 'flip_vert',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["flip_vert"]
            },
            'start_range_ratio': {
                'name': 'start_range_ratio',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["start_range_ratio"]
            },
            'cam_fov': {
                'name': 'cam_fov',
                'namespace': self.namespace,
                'factory_val': self.controls_dict['cam_fov']
            },
            'cam_view': {
                'name': 'cam_view',
                'namespace': self.namespace,
                'factory_val': self.controls_dict['cam_view']
            },
            'cam_pos': {
                'name': 'cam_pos',
                'namespace': self.namespace,
                'factory_val': self.controls_dict['cam_pos']
            },
            'cam_rot': {
                'name': 'cam_rot',
                'namespace': self.namespace,
                'factory_val': self.controls_dict['cam_rot']
            },
             'stop_range_ratio': {
                'name': 'stop_range_ratio',
                'namespace': self.namespace,
                'factory_val': self.controls_dict["stop_range_ratio"]
            },
            # The three 3-D render ratios and the render background sit beside
            # cam_fov/cam_view/cam_pos/cam_rot in controls_dict and are resolved by
            # the same renderer, but had no keys here, so set_zoom_3d_ratio,
            # set_rotate_3d_ratio, set_tilt_3d_ratio and setWhiteBgCb all wrote into
            # a name node_if did not know and the writes were dropped silently.
            # use_wbg is only in PointcloudImageIF's DEFAULT_CONTROLS_DICT, hence the
            # get() default here.
            'zoom_3d_ratio': {
                'name': 'zoom_3d_ratio',
                'namespace': self.namespace,
                'factory_val': self.controls_dict['zoom_3d_ratio']
            },
            'rotate_3d_ratio': {
                'name': 'rotate_3d_ratio',
                'namespace': self.namespace,
                'factory_val': self.controls_dict['rotate_3d_ratio']
            },
            'tilt_3d_ratio': {
                'name': 'tilt_3d_ratio',
                'namespace': self.namespace,
                'factory_val': self.controls_dict['tilt_3d_ratio']
            },
            'use_wbg': {
                'name': 'use_wbg',
                'namespace': self.namespace,
                'factory_val': self.controls_dict.get('use_wbg', False)
            },
            'live_adjust_enabled': {
                'name': 'live_adjust_enabled',
                'namespace': self.namespace,
                'factory_val': self.live_adjust_enabled
            },
            'aspect_adjust_enabled': {
                'name': 'aspect_adjust_enabled',
                'namespace': self.namespace,
                'factory_val': self.aspect_adjust_enabled
            },
             'aspect_ratio_set': {
                'name': 'aspect_ratio_set',
                'namespace': self.namespace,
                'factory_val': self.aspect_ratio_set
            },
            'filter_dict': {
                'name': 'filter_dict',
                'namespace': self.namespace,
                'factory_val': self.filter_dict
            },
            'overlays_dict': {
                'name': 'overlays_dict',
                'namespace': self.namespace,
                'factory_val': self.overlays_dict
            },
            'stream_compression_enabled': {
                'name': 'stream_compression_enabled',
                'namespace': self.namespace,
                'factory_val': self.stream_compression_enabled
            },
            'stream_compression_ratio': {
                'name': 'stream_compression_ratio',
                'namespace': self.namespace,
                'factory_val': self.stream_compression_ratio
            },
            'stabilize_enabled': {
                'name': 'stabilize_enabled',
                'namespace': self.namespace,
                'factory_val': self.stabilize_enabled
            },   

        }

        self.add_params(update_params_dict)

        #######################
        # Add Services
        #######################
        # Services Config Dict ####################     
        update_services_dict = {}

        
        self.add_services(update_services_dict)

        #######################
        # Add Pubs 
        #######################
        # Pubs Config Dict ####################
        update_pubs_dict = {
            'data_pub': {
                'msg': Image,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            }
        }


        self.add_pubs(update_pubs_dict)

        #######################
        # Add Subs 
        #######################
        # Subs Config Dict ####################
        update_subs_dict = {
            'reset_all': {
                'namespace': self.namespace,
                'topic': 'reset',
                'msg': Empty,
                'qsize': 5,
                'callback': self._resetControlsCb, 
                'callback_args': ()
            },
            'reset_filters': {
                'namespace': self.namespace,
                'topic': 'reset_filters',
                'msg': Empty,
                'qsize': 5,
                'callback': self._resetFiltersCb, 
                'callback_args': ()
            },
            'reset_overlays': {
                'namespace': self.namespace,
                'topic': 'reset_overlays',
                'msg': Empty,
                'qsize': 5,
                'callback': self._resetOverlaysCb, 
                'callback_args': ()
            },
            'reset_settings': {
                'namespace': self.namespace,
                'topic': 'reset_settings',
                'msg': Empty,
                'qsize': 5,
                'callback': self._resetSettingsCb, 
                'callback_args': ()
            },
            'reset_renders': {
                'namespace': self.namespace,
                'topic': 'reset_renders',
                'msg': Empty,
                'qsize': 5,
                'callback': self._resetRendersCb, 
                'callback_args': ()
            },
            'mouse_event': {
                'namespace': self.namespace,
                'topic': 'mouse_event',
                'msg': ImageMouseEvent,
                'qsize': 5,
                'callback': self._mouseEventCb,
                'callback_args': ()
            },
        
            'render_3d_controls': {
                'namespace': self.namespace,
                'topic': 'render_3d_controls',
                'msg': Bool,
                'qsize': 10,
                'callback': self.render3dControlsCb,
                'callback_args': ()
            },
            'reset_render_3d_controls': {
                'namespace': self.namespace,
                'topic': 'reset_render_3d_controls',
                'msg': Empty,
                'qsize': 10,
                'callback': self.resetRender3dControlsCb,
                'callback_args': ()
            },
            'reset_render_3d_position': {
                'namespace': self.namespace,
                'topic': 'reset_render_3d_position',
                'msg': Empty,
                'qsize': 10,
                'callback': self.resetRender3dPositionCb,
                'callback_args': ()
            },

            ######################
            'overlay_text_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_enable',
                'qsize': 5,
                'callback': self._setrOverlayTextEnableCb
            },
            'click_text_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'click_text_enable',
                'qsize': 5,
                'callback': self._clickTextEnableCb
            },
            'overlay_text_size_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_size_ratio',
                'qsize': 5,
                'callback': self._setOverlaySizeCb
            },
            'overlay_text_vert_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_vert_ratio',
                'qsize': 5,
                'callback': self._setOverlayVertCb
            },
            'overlay_text_horz_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_horz_ratio',
                'qsize': 5,
                'callback': self._setOverlayHorzCb
            },
            'overlay_text_transparency_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_transparency_ratio',
                'qsize': 5,
                'callback': self._setOverlayTransparencyCb
            },
            'overlay_text_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_color_rgb',
                'qsize': 5,
                'callback': self._setOverlayColorRGBCb
            },
            'overlay_text_img_name': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_source_name',
                'qsize': 5,
                'callback': self._setOverlayImgNameCb
            },
            'overlay_text_date_time': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_date_time',
                'qsize': 5,
                'callback': self._setOverlayDateTimeCb
            },
            'overlay_text_nav': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_nav',
                'qsize': 5,
                'callback': self._setOverlayNavCb
            },
            'overlay_text_pose': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_pose',
                'qsize': 5,
                'callback': self._setOverlayPoseCb
            },
            'add_overlay_text': {
                'msg': String,
                'namespace': self.namespace,
                'topic': 'add_overlay_text',
                'qsize': 5,
                'callback': self._setOverlayTextCb
            },
            'set_overlay_text_list': {
                'msg': StringArray,
                'namespace': self.namespace,
                'topic': 'set_overlay_text_list',
                'qsize': 5,
                'callback': self._setOverlayListCb
            },
            'overlay_text_clear': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'clear_overlay_text_list',
                'qsize': 5,
                'callback': self._clearOverlayListCb
            },

            #######################################
            'crosshairs_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'crosshairs_enable',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairsCb
            },
            'crosshairs_size_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_crosshairs_size_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsSizeRatioCb
            },
            'crosshairs_thickness_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_crosshairs_thickness_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsThicknessRatioCb
            },
            'crosshairs_text_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_crosshairs_text_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsTextRatioCb
            },
            'crosshairs_transparency_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_crosshairs_transparency_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsTransparencyRatioCb
            },
            'crosshairs_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.namespace,
                'topic': 'set_crosshairs_color_rgb',
                'qsize': 5,
                'callback': self._setCrosshairsColorRGBCb
            },

            'overlay_crosshair_names': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_crosshair_names',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairNamesCb
            },
            'overlay_crosshair_pixels': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_crosshair_pixels',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairPixelsCb
            },
            'overlay_crosshair_degrees': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_crosshair_degrees',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairDegreesCb
            },
            'overlay_crosshair_text': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_crosshair_text',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairMessagesCb
            },
            'click_crosshair_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'click_crosshair_enable',
                'qsize': 5,
                'callback': self._clickCrosshairEnableCb
            },
            'add_crosshair_pixel': {
                'msg': ImageCrosshair,
                'namespace': self.namespace,
                'topic': 'add_crosshair_pixel',
                'qsize': 5,
                'callback': self._addCrosshairPixelCb
            },
            'add_crosshair_ratios': {
                'msg': ImageCrosshair,
                'namespace': self.namespace,
                'topic': 'add_crosshair_ratios',
                'qsize': 5,
                'callback': self._addCrosshairRatiosCb
            },
            'add_crosshair_degree_offsets': {
                'msg': ImageCrosshair,
                'namespace': self.namespace,
                'topic': 'add_crosshair_degree_offsets',
                'qsize': 5,
                'callback': self._addCrosshairDegreesCb
            },
            'remove_crosshair': {
                'msg': String,
                'namespace': self.namespace,
                'topic': 'remove_crosshair',
                'qsize': 5,
                'callback': self._removeCrosshairCb
            },
            'clear_crosshairs': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'clear_crosshairs',
                'qsize': 5,
                'callback': self._clearCrosshairsCb
            },
            'set_live_adjust_rotate_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_rotate_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustRotateRatioCb,
                'callback_args': ()
            },
            'set_live_adjust_rotate_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_rotate_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustRotateDegCb,
                'callback_args': ()
            },
            'set_live_adjust_x_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXRatioCb,
                'callback_args': ()
            },
            'set_live_adjust_x_pixel': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_pixel',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXPixelCb,
                'callback_args': ()
            },
            'set_live_adjust_x_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXDegCb,
                'callback_args': ()
            },
            'set_live_adjust_y_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYRatioCb,
                'callback_args': ()
            },
            'set_live_adjust_y_pixel': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_pixel',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYPixelCb,
                'callback_args': ()
            },
            'set_live_adjust_y_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYDegCb,
                'callback_args': ()
            },

            ############################

            'targets_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'targets_enable',
                'qsize': 5,
                'callback': self._setrOverlayTargetsCb
            },
            'targets_size_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_targets_size_ratio',
                'qsize': 5,
                'callback': self._setTargetsSizeRatioCb
            },
            'targets_thickness_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_targets_thickness_ratio',
                'qsize': 5,
                'callback': self._setTargetsThicknessRatioCb
            },
            'targets_text_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_targets_text_ratio',
                'qsize': 5,
                'callback': self._setTargetsTextRatioCb
            },
            'targets_transparency_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_targets_transparency_ratio',
                'qsize': 5,
                'callback': self._setTargetsTransparencyRatioCb
            },
            'targets_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.namespace,
                'topic': 'set_targets_color_rgb',
                'qsize': 5,
                'callback': self._setTargetsColorRGBCb
            },

            'overlay_target_names': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_target_names',
                'qsize': 5,
                'callback': self._setrOverlayTargetNamesCb
            },
            'overlay_target_pixels': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_target_pixels',
                'qsize': 5,
                'callback': self._setrOverlayTargetPixelsCb
            },
            'overlay_target_degrees': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_target_degrees',
                'qsize': 5,
                'callback': self._setrOverlayTargetDegreesCb
            },
            'overlay_target_text': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'overlay_target_text',
                'qsize': 5,
                'callback': self._setrOverlayTargetMessagesCb
            },
            'click_target_enable': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'click_target_enable',
                'qsize': 5,
                'callback': self._clickTargetEnableCb
            },
            'add_target_pixel': {
                'msg': ImageTarget,
                'namespace': self.namespace,
                'topic': 'add_target_pixel',
                'qsize': 5,
                'callback': self._addTargetPixelCb
            },
            'add_target_ratios': {
                'msg': ImageTarget,
                'namespace': self.namespace,
                'topic': 'add_target_ratios',
                'qsize': 5,
                'callback': self._addTargetRatiosCb
            },
            'add_target_degree_offsets': {
                'msg': ImageTarget,
                'namespace': self.namespace,
                'topic': 'add_target_degree_offsets',
                'qsize': 5,
                'callback': self._addTargetDegreesCb
            },
            'remove_target': {
                'msg': String,
                'namespace': self.namespace,
                'topic': 'remove_target',
                'qsize': 5,
                'callback': self._removeTargetCb
            },
            'clear_targets': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'clear_targets',
                'qsize': 5,
                'callback': self._clearTargetsCb
            },

            ####################################
            'set_aspect_adjust_enable': {
                'namespace': self.namespace,
                'topic': 'set_aspect_adjust_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setAspectAdjustEnableCb,
                'callback_args': ()
            },
              'set_aspect_adjust_ratio': {
                'namespace': self.namespace,
                'topic': 'set_aspect_adjust_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setAspectAdjustRatioCb,
                'callback_args': ()
            },
              'set_aspect_adjust_by_ratio': {
                'namespace': self.namespace,
                'topic': 'set_aspect_adjust_by_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setAspectAdjustByRatioCb,
                'callback_args': ()
            },
            ####################################
            'set_stream_compression_enable': {
                'namespace': self.namespace,
                'topic': 'set_stream_compression_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setStreamCompressionEnableCb,
                'callback_args': ()
            },
            'set_stream_compression_ratio': {
                'namespace': self.namespace,
                'topic': 'set_stream_compression_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setStreamCompressionRatioCb,
                'callback_args': ()
            },
            #########################
            'set_live_adjust_enable': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setLiveAdjustEnableCb,
                'callback_args': ()
            },
            'set_live_adjust_rotate_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_rotate_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustRotateRatioCb,
                'callback_args': ()
            },
            'set_live_adjust_rotate_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_rotate_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustRotateDegCb,
                'callback_args': ()
            },
            'set_live_adjust_x_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXRatioCb,
                'callback_args': ()
            },
            'set_live_adjust_x_pixel': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_pixel',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXPixelCb,
                'callback_args': ()
            },
            'set_live_adjust_x_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranXDegCb,
                'callback_args': ()
            },
            'set_live_adjust_y_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYRatioCb,
                'callback_args': ()
            },
            'set_live_adjust_y_pixel': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_pixel',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYPixelCb,
                'callback_args': ()
            },
            'set_live_adjust_y_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_deg',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setLiveAdjustTranYDegCb,
                'callback_args': ()
            },
            #########################
            'set_stabilize_enable': {
                'namespace': self.namespace,
                'topic': 'set_stabilize_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setStabilizeEnableCb,
                'callback_args': ()
            },
            ############################
            'all_overlay_text_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_enable',
                'qsize': 5,
                'callback': self._setrOverlayTextEnableCb
            },
            'all_click_text_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'click_text_enable',
                'qsize': 5,
                'callback': self._clickTextEnableCb
            },

            'all_overlay_text_size_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_size_ratio',
                'qsize': 5,
                'callback': self._setOverlaySizeCb
            },
            'all_overlay_text_vert_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_vert_ratio',
                'qsize': 5,
                'callback': self._setOverlayVertCb
            },
            'all_overlay_text_horz_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_horz_ratio',
                'qsize': 5,
                'callback': self._setOverlayHorzCb
            },
            'all_overlay_text_transparency_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_transparency_ratio',
                'qsize': 5,
                'callback': self._setOverlayTransparencyCb
            },
            'all_overlay_text_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_color_rgb',
                'qsize': 5,
                'callback': self._setOverlayColorRGBCb
            },
            'all_overlay_text_img_name': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_source_name',
                'qsize': 5,
                'callback': self._setOverlayImgNameCb
            },
            'all_overlay_text_date_time': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_date_time',
                'qsize': 5,
                'callback': self._setOverlayDateTimeCb
            },
            'all_overlay_text_nav': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_nav',
                'qsize': 5,
                'callback': self._setOverlayNavCb
            },
            'all_overlay_text_pose': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_pose',
                'qsize': 5,
                'callback': self._setOverlayPoseCb
            },
            'all_add_overlay_text': {
                'msg': String,
                'namespace': self.all_namespace,
                'topic': 'add_overlay_text',
                'qsize': 5,
                'callback': self._setOverlayTextCb
            },
            'all_set_overlay_text_list': {
                'msg': StringArray,
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_list',
                'qsize': 5,
                'callback': self._setOverlayListCb
            },
            'all_overlay_text_clear': {
                'msg': Empty,
                'namespace': self.all_namespace,
                'topic': 'clear_overlay_text_list',
                'qsize': 5,
                'callback': self._clearOverlayListCb
            },
            #############################
            'all_crosshairs_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'crosshairs_enable',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairsCb
            },
            'all_crosshairs_size_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_size_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsSizeRatioCb
            },
            'all_crosshairs_thickness_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_thickness_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsThicknessRatioCb
            },
            'all_crosshairs_text_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_text_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsTextRatioCb
            },
            'all_crosshairs_transparency_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_transparency_ratio',
                'qsize': 5,
                'callback': self._setCrosshairsTransparencyRatioCb
            },
            'all_crosshairs_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_color_rgb',
                'qsize': 5,
                'callback': self._setCrosshairsColorRGBCb
            },
            'all_overlay_crosshair_names': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_names',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairNamesCb
            },
            'all_overlay_crosshair_pixels': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_pixels',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairPixelsCb
            },
            'all_overlay_crosshair_degrees': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_degrees',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairDegreesCb
            },
            'all_overlay_crosshair_text': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_text',
                'qsize': 5,
                'callback': self._setrOverlayCrosshairMessagesCb
            },
            'all_click_crosshair_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'click_crosshair_enable',
                'qsize': 5,
                'callback': self._clickCrosshairEnableCb
            },
            'all_add_crosshair_pixel': {
                'msg': ImageCrosshair,
                'namespace': self.all_namespace,
                'topic': 'add_crosshair_pixel',
                'qsize': 5,
                'callback': self._addCrosshairPixelCb
            },
            'all_add_crosshair_ratios': {
                'msg': ImageCrosshair,
                'namespace': self.all_namespace,
                'topic': 'add_crosshair_ratios',
                'qsize': 5,
                'callback': self._addCrosshairRatiosCb
            },
            'all_add_crosshair_degree_offsets': {
                'msg': ImageCrosshair,
                'namespace': self.all_namespace,
                'topic': 'add_crosshair_degree_offsets',
                'qsize': 5,
                'callback': self._addCrosshairDegreesCb
            },
            'all_remove_crosshair': {
                'msg': String,
                'namespace': self.all_namespace,
                'topic': 'remove_crosshair',
                'qsize': 5,
                'callback': self._removeCrosshairCb
            },
            'all_clear_crosshairs': {
                'msg': Empty,
                'namespace': self.all_namespace,
                'topic': 'clear_crosshairs',
                'qsize': 5,
                'callback': self._clearCrosshairsCb
            },
            #############################
            'all_targets_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'targets_enable',
                'qsize': 5,
                'callback': self._setrOverlayTargetsCb
            },
            'all_targets_size_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_targets_size_ratio',
                'qsize': 5,
                'callback': self._setTargetsSizeRatioCb
            },
            'all_targets_thickness_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_targets_thickness_ratio',
                'qsize': 5,
                'callback': self._setTargetsThicknessRatioCb
            },
            'all_targets_text_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_targets_text_ratio',
                'qsize': 5,
                'callback': self._setTargetsTextRatioCb
            },
            'all_targets_transparency_ratio': {
                'msg': Float32,
                'namespace': self.all_namespace,
                'topic': 'set_targets_transparency_ratio',
                'qsize': 5,
                'callback': self._setTargetsTransparencyRatioCb
            },
            'all_targets_color_rgb': {
                'msg': ColorBGR,
                'namespace': self.all_namespace,
                'topic': 'set_targets_color_rgb',
                'qsize': 5,
                'callback': self._setTargetsColorRGBCb
            },
            'all_overlay_target_names': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_target_names',
                'qsize': 5,
                'callback': self._setrOverlayTargetNamesCb
            },
            'all_overlay_target_pixels': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_target_pixels',
                'qsize': 5,
                'callback': self._setrOverlayTargetPixelsCb
            },
            'all_overlay_target_degrees': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_target_degrees',
                'qsize': 5,
                'callback': self._setrOverlayTargetDegreesCb
            },
            'all_overlay_target_text': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'overlay_target_text',
                'qsize': 5,
                'callback': self._setrOverlayTargetMessagesCb
            },
            'all_click_target_enable': {
                'msg': Bool,
                'namespace': self.all_namespace,
                'topic': 'click_target_enable',
                'qsize': 5,
                'callback': self._clickTargetEnableCb
            },
            'all_add_target_pixel': {
                'msg': ImageTarget,
                'namespace': self.all_namespace,
                'topic': 'add_target_pixel',
                'qsize': 5,
                'callback': self._addTargetPixelCb
            },
            'all_add_target_ratios': {
                'msg': ImageTarget,
                'namespace': self.all_namespace,
                'topic': 'add_target_ratios',
                'qsize': 5,
                'callback': self._addTargetRatiosCb
            },
            'all_add_target_degree_offsets': {
                'msg': ImageTarget,
                'namespace': self.all_namespace,
                'topic': 'add_target_degree_offsets',
                'qsize': 5,
                'callback': self._addTargetDegreesCb
            },
            'all_remove_target': {
                'msg': String,
                'namespace': self.all_namespace,
                'topic': 'remove_target',
                'qsize': 5,
                'callback': self._removeTargetCb
            },
            'all_clear_targets': {
                'msg': Empty,
                'namespace': self.all_namespace,
                'topic': 'clear_targets',
                'qsize': 5,
                'callback': self._clearTargetsCb
            },

            ####################################
            'all_set_stream_compression_enable': {
                'namespace': self.all_namespace,
                'topic': 'set_stream_compression_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setStreamCompressionEnableCb,
                'callback_args': ()
            },
            'all_set_stream_compression_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_stream_compression_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setStreamCompressionRatioCb,
                'callback_args': ()
            },

    
        }



        if self.live_adjustments_disabled == False:
            update_subs_dict.update( {
                'all_set_live_adjust_enable': {
                    'namespace': self.all_namespace,
                    'topic': 'set_live_adjust_enable',
                    'msg': Bool,
                    'qsize': 5,
                    'callback': self._setLiveAdjustEnableCb,
                    'callback_args': ()
                },
                'all_set_live_adjust_rotate_ratio': {
                    'namespace': self.all_namespace,
                    'topic': 'set_live_adjust_rotate_ratio',
                    'msg': Float32,
                    'qsize': 5,
                    'callback': self._setLiveAdjustRotateRatioCb,
                    'callback_args': ()
                },
                'all_set_live_adjust_rotate_deg': {
                    'namespace': self.all_namespace,
                    'topic': 'set_live_adjust_rotate_deg',
                    'msg': Float32,
                    'qsize': 5,
                    'callback': self._setLiveAdjustRotateDegCb,
                    'callback_args': ()
                },
                'all_set_live_adjust_x_ratio': {
                    'namespace': self.all_namespace,
                    'topic': 'set_live_adjust_x_ratio',
                    'msg': Float32,
                    'qsize': 5,
                    'callback': self._setLiveAdjustTranXRatioCb,
                    'callback_args': ()
                },
                'all_set_live_adjust_x_pixel': {
                    'namespace': self.all_namespace,
                    'topic': 'set_live_adjust_x_pixel',
                    'msg': Int32,
                    'qsize': 5,
                    'callback': self._setLiveAdjustTranXPixelCb,
                    'callback_args': ()
                },
                'all_set_live_adjust_x_deg': {
                    'namespace': self.all_namespace,
                    'topic': 'set_live_adjust_x_deg',
                    'msg': Float32,
                    'qsize': 5,
                    'callback': self._setLiveAdjustTranXDegCb,
                    'callback_args': ()
                },
                'all_set_live_adjust_y_ratio': {
                    'namespace': self.all_namespace,
                    'topic': 'set_live_adjust_y_ratio',
                    'msg': Float32,
                    'qsize': 5,
                    'callback': self._setLiveAdjustTranYRatioCb,
                    'callback_args': ()
                },
                'all_set_live_adjust_y_pixel': {
                    'namespace': self.all_namespace,
                    'topic': 'set_live_adjust_y_pixel',
                    'msg': Int32,
                    'qsize': 5,
                    'callback': self._setLiveAdjustTranYPixelCb,
                    'callback_args': ()
                },
                'all_set_live_adjust_y_deg': {
                    'namespace': self.all_namespace,
                    'topic': 'set_live_adjust_y_deg',
                    'msg': Float32,
                    'qsize': 5,
                    'callback': self._setLiveAdjustTranYDegCb,
                    'callback_args': ()
                }


            })

        #########################
        update_subs_dict.update( {
            'set_stabilize_enable': {
            'namespace': self.namespace,
            'topic': 'set_stabilize_enable',
            'msg': Bool,
            'qsize': 5,
            'callback': self._setStabilizeEnableCb,
            'callback_args': ()
            }
        } )

        if self.aspect_adjustment_disabled == False:
            update_subs_dict.update( {
                'all_set_aspect_adjust_enable': {
                    'namespace': self.all_namespace,
                    'topic': 'set_aspect_adjust_enable',
                    'msg': Bool,
                    'qsize': 5,
                    'callback': self._setAspectAdjustEnableCb,
                    'callback_args': ()
                },
                'all_set_aspect_adjust_ratio': {
                    'namespace': self.all_namespace,
                    'topic': 'set_aspect_adjust_ratio',
                    'msg': Float32,
                    'qsize': 5,
                    'callback': self._setAspectAdjustRatioCb,
                    'callback_args': ()
                },
                'all_set_aspect_adjust_by_ratio': {
                    'namespace': self.all_namespace,
                    'topic': 'set_aspect_adjust_by_ratio',
                    'msg': Float32,
                    'qsize': 5,
                    'callback': self._setAspectAdjustByRatioCb,
                    'callback_args': ()
                },

            })

        # Create subs if required
        if caps_dict['has_resolution'] == True:
            update_subs_dict['set_resolution'] = {
                'namespace': self.namespace,
                'topic': 'set_resolution_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setResolutionRatioCb, 
                'callback_args': ()
            }

        if caps_dict['has_auto_adjust'] == True:
            update_subs_dict['set_auto_adjust'] = {
                'namespace': self.namespace,
                'topic': 'set_auto_adjust_enable',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setAutoAdjustCb, 
                'callback_args': ()
            }
            update_subs_dict['set_auto_adjust_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_auto_adjust_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setAutoAdjustRatioCb, 
                'callback_args': ()
            }
        if caps_dict['has_brightness'] == True:
            update_subs_dict['set_brightness'] = {
                'namespace': self.namespace,
                'topic': 'set_brightness_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setBrightnessCb, 
                'callback_args': ()
            }
        if caps_dict['has_contrast'] == True:
            update_subs_dict['set_contrast'] = {
                'namespace': self.namespace,
                'topic': 'set_contrast_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setContrastCb, 
                'callback_args': ()
            }
        if caps_dict['has_threshold'] == True:
            update_subs_dict['set_thresholding'] = {
                'namespace': self.namespace,
                'topic': 'set_threshold_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setThresholdingCb, 
                'callback_args': ()
            }
        if caps_dict['has_rotate_2d'] == True:
            update_subs_dict['rotate_2d'] = {
                'namespace': self.namespace,
                'topic': 'rotate_2d',
                'msg': Empty,
                'qsize': 5,
                'callback': self._setRotate2dCb,
                'callback_args': ()
            }
            update_subs_dict['set_rotate_2d_deg'] = {
                'namespace': self.namespace,
                'topic': 'set_rotate_2d_deg',
                'msg': Int32,
                'qsize': 5,
                'callback': self._setRotate2dDegCb,
                'callback_args': ()
            }
            update_subs_dict['set_rotate_2d_swap_box'] = {
                'namespace': self.namespace,
                'topic': 'set_rotate_2d_swap_box',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setRotate2dSwapBoxCb,
                'callback_args': ()
            }
        if caps_dict['has_flip_horz'] == True:
            update_subs_dict['set_flip_horz'] = {
                'namespace': self.namespace,
                'topic': 'set_flip_horz',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setFlipHorzCb, 
                'callback_args': ()
            }
        if caps_dict['has_flip_vert'] == True:
            update_subs_dict['set_flip_vert'] = {
                'namespace': self.namespace,
                'topic': 'set_flip_vert',
                'msg': Bool,
                'qsize': 5,
                'callback': self._setFlipVertCb, 
                'callback_args': ()
            }
        if caps_dict['has_range'] == True:
            update_subs_dict['set_range'] = {
                'namespace': self.namespace,
                'topic': 'set_range_ratios',
                'msg': RangeWindow,
                'qsize': 5,
                'callback': self._setRangeCb, 
                'callback_args': ()
            }
            # update_subs_dict['set_range_ratios'] = {
            #     'namespace': self.node_namespace,
            #     'topic': 'set_range_ratios',
            #     'msg': RangeWindow,
            #     'qsize': 10,
            #     'callback': self.setRangeRatiosCb,
            #     'callback_args': ()setRangeRatiosCb
            # },
        if caps_dict['has_zoom'] == True:
            update_subs_dict['set_zoom'] = {
                'namespace': self.namespace,
                'topic': 'set_zoom_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setZoomCb, 
                'callback_args': ()
            }
        if caps_dict['has_pan'] == True:
            update_subs_dict['set_pan_x_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_pan_x_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setPanXCb, 
                'callback_args': ()
            }
            update_subs_dict['set_pan_y_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_pan_y_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setPanYCb, 
                'callback_args': ()
            }

        if caps_dict['has_zoom_3d'] == True:
            update_subs_dict['set_zoom_3d'] = {
                'namespace': self.namespace,
                'topic': 'set_zoom_3d_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setZoom3DCb, 
                'callback_args': ()
            }

        if caps_dict['has_rotate_3d'] == True:
            update_subs_dict['set_rotate_3d'] = {
                'namespace': self.namespace,
                'topic': 'set_rotate_3d_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setRotate3DCb, 
                'callback_args': ()
            }

        if caps_dict['has_tilt_3d'] == True:
            update_subs_dict['set_tilt_3d'] = {
                'namespace': self.namespace,
                'topic': 'set_tilt_3d_ratio',
                'msg': Float32,
                'qsize': 5,
                'callback': self._setTilt3DCb, 
                'callback_args': ()
            }




            if caps_dict['has_tilt_3d'] == True:
                update_subs_dict['set_camera_fov'] = {
                    'namespace': self.namespace,
                    'topic': 'set_camera_fov',
                    'msg': Int32,
                    'qsize': 10,
                    'callback': self.setCamFovCb,
                    'callback_args': ()
                }
                update_subs_dict['set_camera_view'] = {
                    'namespace': self.namespace,
                    'topic': 'set_camera_view',
                    'msg': Vector3,
                    'qsize': 10,
                    'callback': self.setCamViewCb,
                    'callback_args': ()
                }
                update_subs_dict['set_camera_position'] = {
                    'namespace': self.namespace,
                    'topic': 'set_camera_position',
                    'msg': Vector3,
                    'qsize': 10,
                    'callback': self.setCamPositionCb,
                    'callback_args': ()
                }
                update_subs_dict['set_camera_rotation'] = {
                    'namespace': self.namespace,
                    'topic': 'set_camera_rotation',
                    'msg': Vector3,
                    'qsize': 10,
                    'callback': self.setCamRotationCb,
                    'callback_args': ()
                }
                update_subs_dict['set_white_bg_enable'] = {
                    'namespace': self.namespace,
                    'topic': 'set_white_bg_enable',
                    'msg': Bool,
                    'qsize': 10,
                    'callback': self.setWhiteBgCb,
                    'callback_args': ()
                }

        if self.has_filters == True:
            update_subs_dict['set_filter_enable'] = {
                'namespace': self.namespace,
                'topic': 'set_filter_enable',
                'msg': UpdateBool,
                'qsize': 5,
                'callback': self._setFilterEnableCb, 
                'callback_args': ()
            }
            update_subs_dict['set_filter_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_filter_ratio',
                'msg': UpdateFloat,
                'qsize': 5,
                'callback': self._setFilterRatioCb, 
                'callback_args': ()
            }



        self.add_subs(update_subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################





        ##############################
        # Initialize Status Msg Values
        ##############################

        self.status_msg.node_name = self.node_name

        # Published Data Info
        self.status_msg.source_description = self.data_source_description
        self.status_msg.end_description = self.data_ref_description
        self.status_msg.encoding = 'bgr8'
        self.status_msg.width_px = 0
        self.status_msg.height_px = 0

        self.status_msg.width_start_deg = 0
        self.status_msg.width_stop_deg = 0
        self.status_msg.height_start_deg = 0
        self.status_msg.height_stop_deg = 0
        self.status_msg.width_deg = 0
        self.status_msg.height_deg = 0


        self.status_msg.perspective = self.perspective
        self.status_msg.auto_adjust_controls = self.auto_adjust_controls
        self.status_msg.get_latency_time = 0
        self.status_msg.pub_latency_time = 0
        self.status_msg.process_time = 0        



        ##############################
        # Create Options IF if needed
        # Only when there are options, and under this image IF's own namespace, so
        # image IFs in one node do not all claim <node>/options
        if options_dict is None: 
            options_dict = dict()
        self.options_controls_if = ControlsIF( controls_name = 'options',
            namespace = self.namespace,
            controls_display_name = 'Options',
            controls_description = 'Custom Options',
            controls_init_dict = options_dict,
            controls_updated_callback = self.callback_dict['options_updated_callback'], 
            save_params = True,
            msg_if = self.msg_if,
            node_if = self.node_if
        )
        self.status_msg.controls_options_topic = self.options_controls_if.get_namespace()


        if self.save_data_if:
            data_products = self.save_data_if.get_data_products()
            if self.data_product not in data_products:
                self.save_data_if.register_data_product(self.data_product)
                if self.data_product == 'color_image':
                    self.save_data_if.set_save_rate(self.data_product,1)
            
     


        ##############################
        # Start Node Processes

        self.msg_if.pub_debug("Staring updater process" )
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)



        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################

    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):


        source_status_dict = self.source_status_dict
        if self.callback_dict['get_source_status_callback'] is not None and  source_status_dict is None:
                try:
                    source_status_dict = self.callback_dict['get_source_status_callback']()
                except Exception as e:
                    self.msg_if.pub_debug("Failed to get_source_status_callback: " + str(e) , throttle_s = 5)
        if source_status_dict is not None:
            self.status_msg.source_description = source_status_dict.get('end_description', self.data_source_description)
            self.status_msg.source_ref_description = source_status_dict.get('end_ref_description', self.data_source_description)

        if source_status_dict is not None:
            is_stabilized = source_status_dict.get('stabilize_enabled', False)
            if is_stabilize == True:
                self.status_msg.stabilize_disabled = True
                self.status_msg.stabilize_enabled = True
                self.stabilize_rotate_deg = source_status_dict.get('stabilize_rotate_deg', 0)
            else:       
                self.status_msg.stabilize_disabled = False
                self.status_msg.stabilize_enabled = self.stabilize_enabled   
        else:       
            self.status_msg.stabilize_disabled = False
            self.status_msg.stabilize_enabled = self.stabilize_enabled
        self.status_msg.stabilize_rotate_deg = self.stabilize_rotate_deg  

        source_navpose_dict = self.source_navpose_dict
        if self.callback_dict['get_source_navpose_callback'] is not None and source_navpose_dict is None:
                try:
                    source_navpose_dict = self.callback_dict['get_source_navpose_callback']()
                except Exception as e:
                    self.msg_if.pub_debug("Failed to get_source_navpose_callback: " + str(e) , throttle_s = 5)
        if source_navpose_dict is not None:
            navpose_dict = source_navpose_dict
            status_msg.navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
        else:
            status_msg.navpose_msg = self.get_navpose_msg()


        status_msg.auto_adjust_enabled = self.controls_dict['auto_adjust_enabled']
        status_msg.auto_adjust_ratio = self.controls_dict['auto_adjust_ratio']
        status_msg.contrast_ratio = self.controls_dict['contrast_ratio']
        status_msg.brightness_ratio = self.controls_dict['brightness_ratio']
        status_msg.threshold_ratio = self.controls_dict['threshold_ratio']
        filter_options = []
        filter_states = []
        filter_ratios = []
        if self.filter_dict is not None:
            for name in self.filter_dict.keys():
                filter_dict = self.filter_dict[name]
                filter_options.append(name)
                filter_states.append(filter_dict['enabled'])
                filter_ratios.append(filter_dict['ratio'])
        status_msg.filter_options = filter_options
        status_msg.filter_states = filter_states
        status_msg.filter_ratios = filter_ratios

        status_msg.aspect_adjustment_disabled = self.aspect_adjustment_disabled
        #self.msg_if.pub_debug(self.data_product + " Publishing Aspect Adjust Disabled: " + str(self.aspect_adjustment_disabled ) )
        status_msg.aspect_adjust_enabled = self.aspect_adjust_enabled and self.aspect_adjustment_disabled == False
        status_msg.aspect_ratio_set = self.aspect_ratio_set

        status_msg.aspect_ratio = self.aspect_ratio
        aspect_ratio_str = nepi_img.get_aspect_ratio_str(self.aspect_ratio)
        status_msg.aspect_ratio_str = aspect_ratio_str

        status_msg.resolution_ratio = self.controls_dict['resolution_ratio']
        status_msg.rotate_2d_deg = self.controls_dict['rotate_2d_deg']
        status_msg.flip_horz = self.controls_dict['flip_horz'] 
        status_msg.flip_vert = self.controls_dict['flip_vert'] 

        status_msg.range_ratios.start_range = self.controls_dict['start_range_ratio']
        status_msg.range_ratios.stop_range = self.controls_dict['stop_range_ratio']


        status_msg.zoom_ratio = self.zoom_ratio
        if self.zoom_ratio > 0.01:
            self.click_crosshair_enabled = False
            self.click_target_enabled = False
        status_msg.pan_x_ratio = self.x_ratio
        status_msg.pan_y_ratio = self.y_ratio
        status_msg.window_x_ratios.start_range = self.controls_dict['window_ratios'][0]
        status_msg.window_x_ratios.stop_range = self.controls_dict['window_ratios'][1]
        status_msg.window_y_ratios.start_range = self.controls_dict['window_ratios'][2]
        status_msg.window_y_ratios.stop_range = self.controls_dict['window_ratios'][3]
        status_msg.zoom_3d_ratio = self.controls_dict['zoom_3d_ratio']
        status_msg.rotate_3d_ratio = self.controls_dict['rotate_3d_ratio']
        status_msg.tilt_3d_ratio = self.controls_dict['tilt_3d_ratio']
        status_msg.render_3d_controls_enabled = self.render_3d_controls_enabled

        try:
            status_msg.camera_fov = self.controls_dict['cam_fov']
        except:
            pass

        try:
            view = self.controls_dict['cam_view']
            cam_view = Vector3()
            cam_view.x = view[0]
            cam_view.y = view[1]
            cam_view.z = view[2]
            status_msg.camera_view = cam_view
        except:
            pass

        try:
            pos = self.controls_dict['cam_pos']
            cam_pos = Vector3()
            cam_pos.x = pos[0]
            cam_pos.y = pos[1]
            cam_pos.z = pos[2]
            status_msg.camera_position = cam_pos
        except:
            pass

        try:
            rot = self.controls_dict['cam_rot']
            cam_rot = Vector3()
            cam_rot.x = rot[0]
            cam_rot.y = rot[1]
            cam_rot.z = rot[2]
            status_msg.camera_rotation = cam_rot
        except:
            pass

        live_adjust_dict = copy.deepcopy(self.live_adjust_dict)
        live_adjust_enabled = live_adjust_dict['live_adjust_enabled']
        live_adjust_rotate_deg = live_adjust_dict['live_adjust_rotate_deg']
        #  deg = 2 * (ratio - 0.5) * 180
        live_adjust_rotate_ratio = ((live_adjust_rotate_deg / 2 / 180)  + 0.5)
        

        live_adjust_x_degs = round(live_adjust_dict['live_adjust_x_deg'],1)
        live_adjust_x_ratio = 0.5 - (live_adjust_x_degs / self.width_deg) / 2
        shift_x_scaler = (live_adjust_x_ratio - 0.5) * 2
        live_adjust_x_pixels = math.floor((shift_x_scaler * self.width_org))
        
        live_adjust_y_degs = round(live_adjust_dict['live_adjust_y_deg'],1)
        live_adjust_y_ratio = 0.5 + (live_adjust_y_degs / self.height_deg) / 2
        shift_y_scaler = (live_adjust_y_ratio - 0.5) * 2
        live_adjust_y_pixels = math.floor((shift_y_scaler * self.height_deg))

        status_msg.live_adjustments_disabled = self.live_adjustments_disabled
        #self.msg_if.pub_debug(self.data_product + " Publishing Live Adjust Disabled: " + str(self.live_adjustments_disabled ) , throttle_s = 5)
        status_msg.live_adjust_enabled = live_adjust_enabled
        status_msg.live_adjust_rotate_ratio = live_adjust_rotate_ratio
        status_msg.live_adjust_rotate_deg = live_adjust_rotate_deg
        status_msg.live_adjust_x_ratio = live_adjust_x_ratio
        status_msg.live_adjust_x_pixels = live_adjust_x_pixels
        status_msg.live_adjust_x_degs = live_adjust_x_degs
        status_msg.live_adjust_y_ratio = live_adjust_y_ratio
        status_msg.live_adjust_y_pixels = live_adjust_y_pixels
        status_msg.live_adjust_y_degs = live_adjust_y_degs


        status_msg.overlay_text_enabled = self.overlays_dict['overlay_text_enabled']
        status_msg.click_text_enabled = self.click_text_enabled
        status_msg.overlay_text_size_ratio = self.overlays_dict['overlay_text_size_ratio']
        status_msg.overlay_text_vert_ratio = self.overlays_dict['overlay_text_vert_ratio']
        status_msg.overlay_text_horz_ratio = self.overlays_dict['overlay_text_horz_ratio']
        status_msg.overlay_text_transparency_ratio = self.overlays_dict['overlay_text_transparency_ratio']
        overlay_text_color_rgb = self.overlays_dict['overlay_text_color_rgb']
        status_msg.overlay_text_color_r = overlay_text_color_rgb[0]
        status_msg.overlay_text_color_g = overlay_text_color_rgb[1]
        status_msg.overlay_text_color_b = overlay_text_color_rgb[2]
        status_msg.overlay_text_source_name = self.overlays_dict['overlay_text_img_name']
        status_msg.overlay_text_date_time =  self.overlays_dict['overlay_text_date_time']
        status_msg.overlay_text_nav = self.overlays_dict['overlay_text_nav']
        status_msg.overlay_text_pose = self.overlays_dict['overlay_text_pose']
        status_msg.base_overlay_text_list = self.overlays_dict['init_overlay_text_list']
        status_msg.add_overlay_text_list = self.overlays_dict['add_overlay_text_list']


        ################
        crosshairs_dict = self.overlays_dict['crosshairs_dict']
        #self.msg_if.pub_debug("Publishing crosshairs_dict: " + str(crosshairs_dict) )
        crosshairs_msg_list = []
        for crosshair_name in crosshairs_dict.keys():
            crosshair_msg = ImageCrosshair()
            crosshair_msg.name = crosshair_name
            crosshair_dict = crosshairs_dict[crosshair_name]
            try:

                x_deg_offset = crosshair_dict['x_deg_offset'] #round( -1 * ((x_ratio - 0.5) * self.width_deg),1)
                x_ratio = ((self.width_deg/2) - x_deg_offset)/self.width_deg
                x_offset_ratio = (0.5 - x_ratio)
                x_scale = (self.width_proc/self.width_org)
                x_offset_pixel = int((x_offset_ratio * self.width_org))
                x_pixel = int((self.width_proc/2) + x_offset_pixel)
                #self.msg_if.pub_debug("Rendering target x: " + str([x_ratio,x_offset_ratio,x_scale,x_offset_pixel,x_pixel,x_deg_offset])  , throttle_s = 5)


                y_deg_offset = crosshair_dict['y_deg_offset'] #round( -1 * ((y_ratio - 0.5) * self.height_deg),1)
                y_ratio = (self.height_deg/2) + y_deg_offset
                y_offset_ratio = -1 * (0.5 - y_ratio)
                y_scale = (self.height_proc/self.height_org)
                y_offset_pixel = int((y_offset_ratio * self.height_org))
                y_pixel = int((self.height_proc/2) + y_offset_pixel)
                #self.msg_if.pub_debug("Rendering target y: " + str([y_ratio,y_offset_ratio,y_scale,y_offset_pixel,y_pixel,y_deg_offset])  , throttle_s = 5)

                crosshair_msg.x_ratio = x_ratio
                crosshair_msg.x_offset_deg = x_deg_offset
                crosshair_msg.x_offset_pixel = x_offset_pixel
                crosshair_msg.x_pixel = x_pixel

                crosshair_msg.y_ratio = y_ratio
                crosshair_msg.y_offset_deg = y_deg_offset
                crosshair_msg.y_offset_pixel = y_offset_pixel
                crosshair_msg.y_pixel = y_pixel

                crosshair_msg.r = crosshair_dict['color_rgb'][0]
                crosshair_msg.g = crosshair_dict['color_rgb'][1]
                crosshair_msg.b = crosshair_dict['color_rgb'][2]


                crosshair_msg.msg_str = str(crosshair_dict['msg_str'])
                crosshairs_msg_list.append(crosshair_msg)
            except:
                pass
                

        status_msg.crosshairs_enabled = self.overlays_dict['crosshairs_enabled']
        status_msg.click_crosshair_enabled = self.click_crosshair_enabled
        status_msg.crosshairs_size_ratio = self.overlays_dict['crosshairs_size_ratio']
        status_msg.crosshairs_thickness_ratio = self.overlays_dict['crosshairs_thickness_ratio']
        status_msg.crosshairs_text_ratio = self.overlays_dict['crosshairs_text_ratio']
        status_msg.crosshairs_transparency_ratio = self.overlays_dict['crosshairs_transparency_ratio']
        crosshairs_color_rgb = self.overlays_dict['crosshairs_color_rgb']
        status_msg.crosshairs_color_r = crosshairs_color_rgb[0]
        status_msg.crosshairs_color_g = crosshairs_color_rgb[1]
        status_msg.crosshairs_color_b = crosshairs_color_rgb[2]
        status_msg.overlay_crosshair_names = self.overlays_dict['overlay_crosshair_names']
        status_msg.overlay_crosshair_pixels = self.overlays_dict['overlay_crosshair_pixels']
        status_msg.overlay_crosshair_degrees = self.overlays_dict['overlay_crosshair_degrees']
        status_msg.overlay_crosshair_text = self.overlays_dict['overlay_crosshair_text']
        status_msg.num_crosshairs = len(list(crosshairs_dict.keys()))
        status_msg.crosshairs = crosshairs_msg_list


        ################
        targets_dict = self.overlays_dict['targets_dict']
        #self.msg_if.pub_debug("Publishing targets_dict: " + str(targets_dict) )
        targets_msg_list = []
        for target_name in targets_dict.keys():
            target_msg = ImageTarget()
            target_msg.name = target_name
            target_dict = targets_dict[target_name]
            try:

                x_deg_offset = target_dict['x_deg_offset'] #round( -1 * ((x_ratio - 0.5) * self.width_deg),1)
                x_ratio = ((self.width_deg/2) - x_deg_offset)/self.width_deg
                x_offset_ratio = (0.5 - x_ratio)
                x_scale = (self.width_proc/self.width_org)
                x_offset_pixel = int((x_offset_ratio * self.width_org))
                x_pixel = int((self.width_proc/2) + x_offset_pixel)
                #self.msg_if.pub_debug("Rendering target x: " + str([x_ratio,x_offset_ratio,x_scale,x_offset_pixel,x_pixel,x_deg_offset])  , throttle_s = 5)


                y_deg_offset = target_dict['y_deg_offset'] #round( -1 * ((y_ratio - 0.5) * self.height_deg),1)
                y_ratio = ((self.height_deg/2) + y_deg_offset)/self.height_deg
                y_offset_ratio = -1 * (0.5 - y_ratio)
                y_scale = (self.height_proc/self.height_org)
                y_offset_pixel = int((y_offset_ratio * self.height_org))
                y_pixel = int((self.height_proc/2) + y_offset_pixel)
                #self.msg_if.pub_debug("Rendering target y: " + str([y_ratio,y_offset_ratio,y_scale,y_offset_pixel,y_pixel,y_deg_offset])  , throttle_s = 5)

                target_msg.x_ratio = x_ratio
                target_msg.x_offset_deg = x_deg_offset
                target_msg.x_offset_pixel = x_offset_pixel
                target_msg.x_pixel = x_pixel

                target_msg.y_ratio = y_ratio
                target_msg.y_offset_deg = y_deg_offset
                target_msg.y_offset_pixel = y_offset_pixel
                target_msg.y_pixel = y_pixel

                target_msg.r = target_dict['color_rgb'][0]
                target_msg.g = target_dict['color_rgb'][1]
                target_msg.b = target_dict['color_rgb'][2]


                target_msg.msg_str = str(target_dict['msg_str'])
                targets_msg_list.append(target_msg)
            except:
                pass
                

        status_msg.targets_enabled = self.overlays_dict['targets_enabled']
        status_msg.click_target_enabled = self.click_target_enabled
        status_msg.targets_size_ratio = self.overlays_dict['targets_size_ratio']
        status_msg.targets_thickness_ratio = self.overlays_dict['targets_thickness_ratio']
        status_msg.targets_text_ratio = self.overlays_dict['targets_text_ratio']
        status_msg.targets_transparency_ratio = self.overlays_dict['targets_transparency_ratio']
        targets_color_rgb = self.overlays_dict['targets_color_rgb']
        status_msg.targets_color_r = targets_color_rgb[0]
        status_msg.targets_color_g = targets_color_rgb[1]
        status_msg.targets_color_b = targets_color_rgb[2]
        status_msg.overlay_target_names = self.overlays_dict['overlay_target_names']
        status_msg.overlay_target_pixels = self.overlays_dict['overlay_target_pixels']
        status_msg.overlay_target_degrees = self.overlays_dict['overlay_target_degrees']
        status_msg.overlay_target_text = self.overlays_dict['overlay_target_text']
        status_msg.num_targets = len(list(targets_dict.keys()))
        status_msg.targets = targets_msg_list


        status_msg.stream_compression_enabled  = self.stream_compression_enabled
        stream_compression_ratio = 0
        if self.stream_compression_enabled == True:
            stream_compression_ratio = self.stream_compression_ratio
        status_msg.stream_compression_ratio = stream_compression_ratio

        status_msg.publishing = self.needs_data

        avg_rate = 0
        if len(self.time_list) > 0:
            avg_time = sum(self.time_list) / len(self.time_list)
            if avg_time > .01:
                avg_rate = float(1) / avg_time
        status_msg.avg_pub_rate = avg_rate


        return status_msg




    def init(self, do_updates = False):
        """Initialize or re-initialize controls from the parameter server and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            self.controls_dict['resolution_ratio'] = self.node_if.get_param(self.node_if_prefix +  'resolution_ratio')
            self.controls_dict['auto_adjust_enabled'] = self.node_if.get_param(self.node_if_prefix +  'auto_adjust_enabled')
            self.controls_dict['auto_adjust_ratio'] = self.node_if.get_param(self.node_if_prefix +  'auto_adjust_ratio')
            self.controls_dict['brightness_ratio'] = self.node_if.get_param(self.node_if_prefix +  'brightness_ratio')
            self.controls_dict['contrast_ratio'] = self.node_if.get_param(self.node_if_prefix +  'contrast_ratio')
            self.controls_dict['threshold_ratio'] = self.node_if.get_param(self.node_if_prefix +  'threshold_ratio')

            self.controls_dict['rotate_2d_deg'] = self.node_if.get_param(self.node_if_prefix +  'rotate_2d_deg')
            self.controls_dict['rotate_2d_swap_box'] = self.node_if.get_param(self.node_if_prefix +  'rotate_2d_swap_box')
            self.controls_dict['flip_horz'] = self.node_if.get_param(self.node_if_prefix +  'flip_horz')
            self.controls_dict['flip_vert'] = self.node_if.get_param(self.node_if_prefix +  'flip_vert')


            self.controls_dict['window_ratios'] = [0,1,0,1]
            # Read the range window back rather than hardcoding 0/1. Both keys are
            # registered and both write paths persist them, so hardcoding here threw
            # away a correctly saved range window on every restart.
            self.controls_dict['start_range_ratio'] = self.node_if.get_param(self.node_if_prefix +  'start_range_ratio')
            self.controls_dict['stop_range_ratio'] = self.node_if.get_param(self.node_if_prefix +  'stop_range_ratio')


            self.controls_dict['cam_fov'] = self.node_if.get_param(self.node_if_prefix +  'cam_fov')


            self.controls_dict['cam_view'] =  self.node_if.get_param(self.node_if_prefix +  'cam_view')
            self.controls_dict['cam_pos'] =  self.node_if.get_param(self.node_if_prefix +  'cam_pos')
            self.controls_dict['cam_rot'] =  self.node_if.get_param(self.node_if_prefix +  'cam_rot')

            self.controls_dict['zoom_3d_ratio'] = self.node_if.get_param(self.node_if_prefix +  'zoom_3d_ratio')
            self.controls_dict['rotate_3d_ratio'] = self.node_if.get_param(self.node_if_prefix +  'rotate_3d_ratio')
            self.controls_dict['tilt_3d_ratio'] = self.node_if.get_param(self.node_if_prefix +  'tilt_3d_ratio')
            self.controls_dict['use_wbg'] = self.node_if.get_param(self.node_if_prefix +  'use_wbg')




            self.live_adjust_enabled  = self.node_if.get_param(self.node_if_prefix +  'live_adjust_enabled')
            self.live_adjust_dict['live_adjust_enabled'] = self.live_adjust_enabled

            aspect_adjust_enabled  = self.node_if.get_param(self.node_if_prefix +  'aspect_adjust_enabled')
            self.set_aspect_adjust_enable(aspect_adjust_enabled)
            aspect_ratio_set = self.node_if.get_param(self.node_if_prefix +  'aspect_ratio_set')
            self.set_aspect_adjust_ratio(aspect_ratio_set)

            self.stream_compression_enabled  = self.node_if.get_param(self.node_if_prefix +  'stream_compression_enabled')
            self.stream_compression_ratio = self.node_if.get_param(self.node_if_prefix +  'stream_compression_ratio')


            filter_dict = self.node_if.get_param(self.node_if_prefix +  'filter_dict')
            if filter_dict is not None:
                self.filter_dict = filter_dict
            else:
                self.filter_dict = dict()

            overlays_dict = self.node_if.get_param(self.node_if_prefix +  'overlays_dict')
            if overlays_dict is not None:

                crosshairs_dict = dict()
                if 'crosshairs_dict' in overlays_dict.keys():
                    for name in overlays_dict['crosshairs_dict'].keys():
                        if name != 'click':
                            try:
                                crosshair_dict = overlays_dict['crosshairs_dict'][name]
                                for key in self.BLANK_CROSSHAIR_DICT.keys():
                                    if key not in crosshair_dict.keys():
                                        crosshair_dict[key] = self.BLANK_CROSSHAIR_DICT[key]
                                crosshairs_dict[name] = crosshair_dict
                            except:
                                pass
                overlays_dict['crosshairs_dict'] = crosshairs_dict

                targets_dict = dict()
                if 'targets_dict' in overlays_dict.keys():
                    for name in overlays_dict['targets_dict'].keys():
                        if name != 'click':
                            try:
                                target_dict = overlays_dict['targets_dict'][name]
                                for key in self.BLANK_TARGET_DICT.keys():
                                    if key not in target_dict.keys():
                                        target_dict[key] = self.BLANK_TARGET_DICT[key]
                                targets_dict[name] = target_dict
                            except:
                                pass
                overlays_dict['targets_dict'] = targets_dict

                if overlays_dict is not None:
                    for key in self.overlays_dict.keys():
                        if key in overlays_dict.keys():
                            self.overlays_dict[key] = overlays_dict[key]

        if do_updates == True:
            pass
        self.zoom_ratio = 0
        self.x_ratio = 0.5
        self.y_ratio = 0.5
        self.x_offset = 0
        self.y_offset = 0
        self.x_scaler = 1
        self.y_scaler = 1

        self.publish_status()

    # def reset(self):
    #     """Reset the image interface to its initialized state."""
    #     if self.node_if is not None and self.node_if_shared == False:
    #         self.msg_if.pub_debug("Reseting params" )
    #         self.node_if.reset_params()
    #         nepi_sdk.sleep(1)     
    #     self.init(do_updates = True)

    # def factory_reset(self):
    #     """Reset the image interface to factory defaults."""
    #     if self.node_if is not None and self.node_if_shared == False:
    #         self.msg_if.pub_debug("Factory resetting params" )
    #         self.node_if.factory_reset_params()
    #         nepi_sdk.sleep(1) 
    #     self.init(do_updates = True)





    ###############################
    # Class Public Methods
    ###############################


    def add_pub_namespace(self,namespace):
        """Register an additional image publisher under the given namespace.

        Creates a new image publisher and a companion status publisher under
        a sub-namespace derived from the supplied namespace and the data product
        name. No-ops if the namespace is already registered or equals the
        primary namespace.

        Args:
            namespace (str): The base ROS namespace to add a publisher for.

        Returns:
            bool: True if the publisher was successfully added, False otherwise.
        """
        success = False
        if namespace is None:
            return success
        namespace = nepi_sdk.get_full_namespace(namespace)
        pub_namespace = nepi_sdk.create_namespace(namespace,self.data_product)
        if pub_namespace == self.namespace or namespace in self.add_pubs_dict.keys():
            self.msg_if.pub_debug("Image pub namespace allready registered: " + str(namespace) )
            return success
        else:
            self.msg_if.pub_debug("Adding image pub namespace: " + str(self.namespace) )
            img_pub_dict ={
                    'msg': Image,
                    'namespace': pub_namespace,
                    'topic': '',
                    'qsize': 1,
                    'latch': False
                }
            self.node_if.register_pub(pub_namespace,img_pub_dict)

            status_ns = nepi_sdk.create_namespace(pub_namespace,'status')
            status_pub_dict ={
                    'msg': ImageStatus,
                    'namespace': pub_namespace,
                    'topic': 'status',
                    'qsize': 1,
                    'latch': False
                }
            self.node_if.register_pub(status_ns,status_pub_dict)

        return success

    def remove_pub_namespace(self,namespace):
        """Unregister an additional image publisher that was previously added.

        Removes the image, status, and nav publishers associated with the given
        namespace. Cannot remove the primary namespace.

        Args:
            namespace (str): The base ROS namespace to remove.

        Returns:
            bool: True if the publisher was successfully removed, False otherwise.
        """
        success = False
        if namespace is None:
            return success
        namespace = nepi_sdk.get_full_namespace(namespace)
        pub_namespace = nepi_sdk.create_namespace(namespace,self.data_product)
        if pub_namespace == self.namespace:
            self.msg_if.pub_debug("Can't remove base namespace: " + str(namespace) )
            return success
        elif namespace in self.add_pubs_dict.keys():
            self.msg_if.pub_debug("Removing image pub namespace: " + str(self.namespace) )
            [pub_namespace,status_ns,nav_ns] =  self.add_pubs_dict[namespace]
            del self.add_pubs_dict[namespace]
            nepi_sdk.sleep(1)
            self.node_if.unregister_pub(pub_namespace)
            self.node_if.unregister_pub(status_ns)
            self.node_if.unregister_pub(nav_ns)
            success = True
        return success


    def get_blank_navpose_dict(self):
        """Return a deep copy of the blank nav pose dictionary template.

        Returns:
            dict: A blank nav pose dictionary with all fields at default values.
        """
        blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return blank_navpose_dict

    def get_image_callback_options(self):
        """Return the list of supported image callback names.

        Returns:
            list: Callback name strings registered in the callback dictionary.
        """
        return list(self.callback_dict.keys())

    def set_image_callback(self,name,function):
        """Register a callable for the named image callback slot.

        Args:
            name (str): Name of the callback slot (must be in the callback dict).
            function (callable): Function to call when the event fires.
        """
        self.msg_if.pub_debug("Got set callback for: " + str(name) )
        if name in self.callback_dict.keys():
            self.msg_if.pub_debug("Callback set for: " + str(name) )
            self.callback_dict[name] = function
        #self.msg_if.pub_debug("Updated callback dict: " + str(self.callback_dict) )


    def get_options_dict(self):
        """Return the current custom option values.

        Returns:
            dict: Option name to current value, empty when there are no options.
        """
        options = dict()
        if self.options_controls_if is not None:
            options = nepi_controls.get_values_dict(self.options_controls_if.get_controls_dict())

        return options

    def set_option_value(self,name,value):
        """Set one custom option value.

        Args:
            name (str): Option name (must be one of the options_dict controls).
            value: New value for the option.
        """
        if self.options_controls_if is not None:
            self.options_controls_if.set_control_value(name,value)

    def set_option_values(self,options_dict):
        """Set several custom option values.

        Args:
            options_dict (dict): Option name to new value.
        """
        if self.options_controls_if is not None:
            for name, value in options_dict.items():
                self.options_controls_if.set_control_value(name,value)


    def process_cv2_img(self, cv2_img):
        """Apply any image processing pipeline to a org OpenCV image.

        Base implementation is a pass-through. Subclasses override this to apply
        resolution, orientation, filter, and adjustment controls.

        Args:
            cv2_img (numpy.ndarray): Input OpenCV image array.

        Returns:
            numpy.ndarray: The processed image (same object in the base class).
        """
        return cv2_img

    def publish_cv2_img(self, cv2_img_org,
                        encoding = "bgr8",
                        timestamp = None,
                        width_deg = 100,
                        height_deg = 70,
                        min_range_m = 0,
                        max_range_m = 0,
                        add_overlay_text_list = [],
                        process_data = True,
                        pub_twice = False,
                        add_pubs = [],
                        source_status_dict = None,
                        source_navpose_dict = None,
                        ):
        """Process and publish an OpenCV image to all configured ROS topics.

        Applies the image processing pipeline (if process_data is True), renders
        configured text overlays, publishes the result as a ROS Image message,
        optionally publishes to additional namespaces, saves to disk if a
        SaveDataIF is registered, and updates publication statistics.

        Args:
            cv2_img (numpy.ndarray): OpenCV image to publish.
            encoding (str, optional): ROS image encoding string. Defaults to 'bgr8'.
            timestamp (float or rospy.Time, optional): Acquisition timestamp.
                Defaults to current time if None.
            width_deg (float, optional): Horizontal field of view in degrees.
                Defaults to 100.
            height_deg (float, optional): Vertical field of view in degrees.
                Defaults to 70.
            min_range_m (float, optional): Minimum sensor range in meters.
                Defaults to 0.
            max_range_m (float, optional): Maximum sensor range in meters.
                Defaults to 0.
            add_overlay_text_list (list, optional): Additional text strings to overlay
                on the image. Defaults to [].
            process_data (bool, optional): Whether to run process_cv2_img before
                publishing. Defaults to True.
            pub_twice (bool, optional): Publish the image a second time after a
                brief sleep to work around subscriber latching issues.
                Defaults to False.
            add_pubs (list, optional): Additional namespace strings to publish to.
                Defaults to [].

        Returns:
            numpy.ndarray: The processed image, or the original image if processing
            was skipped or an error occurred.
        """
        
        cv2_img = copy.deepcopy(cv2_img_org)


        self.source_status_dict = source_status_dict
        if self.callback_dict['get_source_status_callback'] is not None and  source_status_dict is None:
                try:
                    source_status_dict = self.callback_dict['get_source_status_callback']()
                except Exception as e:
                    self.msg_if.pub_debug("Failed to get_source_status_callback: " + str(e) , throttle_s = 5)
        if source_status_dict is None:
            source_status_dict = dict()

        self.source_navpose_dict = source_navpose_dict
        if self.callback_dict['get_source_navpose_callback'] is not None and  source_navpose_dict is None:
                try:
                    source_navpose_dict = self.callback_dict['get_source_navpose_callback']()
                except Exception as e:
                    self.msg_if.pub_debug("Failed to get_source_navpose_callback: " + str(e) , throttle_s = 5)
        if source_navpose_dict is not None:
            navpose_dict = source_status_dict
        else:
            navpose_dict = self.get_navpose_dict()




        if self.publishing == False:
            self.publishing = True


            #self.msg_if.pub_debug("Got Image to Publish" , throttle_s = 5.0)
            success = False
            if cv2_img is None and self.status_msg is not None:
                self.msg_if.pub_debug("Can't publish None image" )
                # Clear the re-entrancy latch before the early return, or this IF
                # never publishes another frame.
                self.publishing = False
                return cv2_img

            # Process
            try: # Catch for lost camera in middle of send
                
        

                self.status_msg.encoding = encoding
                #self.msg_if.pub_debug("Got timestamp: " + str(timestamp) )
                if timestamp == None:
                    timestamp = source_status_dict.get('timestamp',nepi_utils.get_time())
                else:
                    timestamp = nepi_sdk.sec_from_timestamp(timestamp)
                #self.msg_if.pub_debug("Using timestamp: " + str(timestamp) )


                current_time = nepi_utils.get_time()
                latency = (current_time - timestamp)
                self.status_msg.get_latency_time = latency
                #self.msg_if.pub_debug("Get Data Latency: {:.2f}".format(latency) , throttle_s = 30)

                # Start Img Pub Process
                start_time = nepi_utils.get_time()   


                self.width_deg = source_status_dict.get('width_deg', width_deg)
                self.height_deg = source_status_dict.get('height_deg', height_deg)
                self.status_msg.width_deg = width_deg * (1 - self.zoom_ratio)
                self.status_msg.height_deg = height_deg * (1 - self.zoom_ratio)

                if (min_range_m is not None and max_range_m is not None):
                    self._updateRangesM(min_range_m,max_range_m)
                    self.status_msg.min_range_m = self.min_range_m
                    self.status_msg.max_range_m = self.max_range_m
                else:
                    self.status_msg.min_range_m = 0
                    self.status_msg.max_range_m = 1

                start_range_ratio = self.controls_dict['start_range_ratio']
                stop_range_ratio = self.controls_dict['stop_range_ratio']

                delta_range = self.max_range_m - self.min_range_m
                self.status_msg.min_range_m_adj = self.min_range_m + delta_range * start_range_ratio
                self.status_msg.max_range_m_adj = self.min_range_m + delta_range * stop_range_ratio

                [height,width] = cv2_img.shape[0:2]
                [self.height_org,self.width_org] = [height,width]
                #self.msg_if.pub_debug("Got Image size: " + str([height,width]) )



                if self.callback_dict['pre_process_callback'] is not None and  cv2_img is not None:
                        try:
                            cv2_img_status = nepi_sdk.convert_msg2dict(self.status_msg)
                            cv2_img = self.callback_dict['pre_process_callback'](cv2_img, cv2_img_status)
                        except Exception as e:
                            self.msg_if.pub_debug("Failed to pre_process_callback: " + str(e) , throttle_s = 5)

                if process_data == True and  cv2_img is not None:
                    cv2_img = self.process_cv2_img(cv2_img)

                if cv2_img is not None:
                    cv2_img = self._liveAdjust(cv2_img) 

                if cv2_img is not None:
                    is_stabilized = source_status_dict.get('stabilize_enabled', False)
                    if is_stabilize == True:
                        self.status_msg.stabilize_disabled = True
                        self.status_msg.stabilize_enabled = True
                        self.stabilize_rotate_deg = source_status_dict.get('stabilize_rotate_deg', 0)
                    else:
                        cv2_img = self._stabilizeAdjust(cv2_img, navpose_dict)          
                        self.status_msg.stabilize_disabled = False
                        self.status_msg.stabilize_enabled = self.stabilize_enabled
                    self.status_msg.stabilize_rotate_deg = self.stabilize_rotate_deg                               

                if self.callback_dict['post_process_callback'] is not None and  cv2_img is not None:
                        try:
                            cv2_img_status = nepi_sdk.convert_msg2dict(self.status_msg)
                            cv2_img = self.callback_dict['post_process_callback'](cv2_img, cv2_img_status)
                        except Exception as e:
                            self.msg_if.pub_debug("Failed to post_process_callback: " + str(e) , throttle_s = 5)

                if cv2_img is not None:
                    [height,width] = cv2_img.shape[0:2]
                    [self.height_proc,self.width_proc] = [height,width]


                    last_width = self.status_msg.width_px
                    last_height = self.status_msg.height_px
                    self.status_msg.width_px = width
                    self.status_msg.height_px = height
                    res_str = str(width) + ":" + str(height)
                    self.status_msg.resolution_current = res_str

                    if height > 5 and width > 5:

                        ######################             
                        crosshairs_dict = self.overlays_dict['crosshairs_dict']
                        crosshair_len = len(list(crosshairs_dict.keys()))
                        crosshairs_enabled = self.overlays_dict['crosshairs_enabled']
                        overlay_crosshair_names = self.overlays_dict['overlay_crosshair_names']
                        overlay_crosshair_pixels = self.overlays_dict['overlay_crosshair_pixels']
                        overlay_crosshair_degrees = self.overlays_dict['overlay_crosshair_degrees']
                        overlay_crosshair_text = self.overlays_dict['overlay_crosshair_text']
                        if crosshair_len > 0 and crosshairs_enabled == True:
                            #self.msg_if.pub_debug("Rendering image overlays: " + str(self.overlays_dict)  , throttle_s = 5)
                            for crosshair_name in crosshairs_dict.keys():
                                crosshair_dict = crosshairs_dict[crosshair_name]
                                #self.msg_if.pub_debug("Rendering image with crosshair_dict: " + str(crosshair_dict)  )
        


                                x_deg_offset = round(crosshair_dict['x_deg_offset'] + 0.02, 1)
                                x_ratio = ((self.width_deg/2) - x_deg_offset)/self.width_deg
                                x_offset_ratio = (x_ratio - 0.5)
                                x_scale = (self.width_proc/self.width_org)
                                x_offset_pixel = int((x_offset_ratio * self.width_org))
                                x_pixel = int((self.width_proc/2) + x_offset_pixel)
                                if x_pixel < 0:
                                    x_pixel = 0
                                if x_pixel > self.width_proc:
                                    x_pixel = self.width_proc
                                #self.msg_if.pub_debug("Rendering target x: " + str([x_deg_offset,x_ratio,x_offset_ratio,x_scale,x_offset_pixel,x_pixel])  , throttle_s = 5)


                                y_deg_offset = round(crosshair_dict['y_deg_offset'] + 0.02, 1)
                                y_ratio = ((self.height_deg/2) + y_deg_offset)/self.height_deg
                                y_offset_ratio = -1 * (0.5 - y_ratio)
                                y_scale = (self.height_proc/self.height_org)
                                y_offset_pixel = int((y_offset_ratio * self.height_org))
                                y_pixel = int((self.height_proc/2) + y_offset_pixel)
                                if y_pixel < 0:
                                    y_pixel = 0
                                if y_pixel > self.height_proc:
                                    y_pixel = self.height_proc
                               
                                pixel = [x_pixel,y_pixel]
                                pixel_adj = copy.deepcopy(pixel)
                                x_pixel_offset_adj = pixel[0]
                                y_pixel_offset_adj = pixel[1]
                                x_ratio_adj = copy.deepcopy(x_ratio)
                                y_ratio_adj = copy.deepcopy(y_ratio)

                                r_deg = 0
                                x_deg = 0
                                y_deg = 0
                                if self.live_adjust_dict['live_adjust_enabled'] == True:
                                    r_deg = self.live_adjust_dict['live_adjust_rotate_deg']
                                    x_deg = self.live_adjust_dict['live_adjust_x_deg']
                                    y_deg = self.live_adjust_dict['live_adjust_y_deg']
                                if self.stabilize_enabled == True:
                                    r_deg = r_deg + self.stabilize_rotate_deg
                                if r_deg != 0 or x_deg != 0 or y_deg != 0:
                                    x_ratio_adj = x_ratio_adj + (width / 2) * ( x_deg / (self.width_deg / 2)) / width
                                    y_ratio_adj = y_ratio_adj + (height / 2) * (y_deg / (self.height_deg / 2)) / height
                                    deg_adj =  r_deg
                                    x_pixel_offset_adj =  (x_ratio_adj * width) - (width / 2) 
                                    y_pixel_offset_adj =  (y_ratio_adj * height) - (height / 2)
                                    [x_pixel_offset_adj,y_pixel_offset_adj] = nepi_utils.rotate_point(x_pixel_offset_adj,y_pixel_offset_adj,deg_adj)
                                    x_pixel_offset_adj = int(x_pixel_offset_adj)
                                    y_pixel_offset_adj = int(y_pixel_offset_adj)
                                    x_ratio_adj = (x_pixel_offset_adj + (width / 2)) / width
                                    y_ratio_adj= (y_pixel_offset_adj + (height / 2)) / height
                                    pixel_adj = [int(x_ratio_adj * width), int(y_ratio_adj * height)]
                                [x_pixel,y_pixel] = pixel_adj   

                                #self.msg_if.pub_debug("Rendering crosshair y: " + str([y_deg_offset,y_ratio,y_offset_ratio,y_scale,y_offset_pixel,y_pixel])  , throttle_s = 5)
                                #self.msg_if.pub_debug("Rendering crosshair pixel,pixel_adj: " + str([pixel,pixel_adj])  , throttle_s = 5)


                                crosshair_rbg = crosshair_dict['color_rgb']
                                crosshair_msg = crosshair_dict['msg_str']

                                overlay_text = []
                                if overlay_crosshair_names == True:
                                    overlay_text.append(crosshair_name)
                                if overlay_crosshair_pixels == True:
                                    overlay_text.append(str(x_pixel) + ',' + str(y_pixel))
                                if overlay_crosshair_degrees == True:
                                    overlay_text.append(str(round(x_deg_offset,0)) + ',' + str(round(y_deg_offset,0)))
                                if overlay_crosshair_text == True and len(crosshair_msg) > 0:
                                    overlay_text.append(str(crosshair_msg))

                                crosshairs_size_ratio = self.overlays_dict['crosshairs_size_ratio']
                                crosshairs_thickness_ratio = self.overlays_dict['crosshairs_thickness_ratio']
                                crosshairs_text_ratio = self.overlays_dict['crosshairs_text_ratio']
                                #self.msg_if.pub_debug("Rendering image crosshair: " + str([x_pixel,y_pixel])  , throttle_s = 5)
                                cv2_img = nepi_img.overlay_crosshair(cv2_img, 
                                                        x_px = x_pixel , y_px = y_pixel, 
                                                        color_rgb = crosshair_rbg, 
                                                        size_ratio =  crosshairs_size_ratio,
                                                        thickness_ratio = crosshairs_thickness_ratio,
                                                        text_list = overlay_text,
                                                        text_ratio = crosshairs_text_ratio)


                        ######################             
                        targets_dict = self.overlays_dict['targets_dict']
                        target_len = len(list(targets_dict.keys()))
                        targets_enabled = self.overlays_dict['targets_enabled']
                        overlay_target_names = self.overlays_dict['overlay_target_names']
                        overlay_target_pixels = self.overlays_dict['overlay_target_pixels']
                        overlay_target_degrees = self.overlays_dict['overlay_target_degrees']
                        overlay_target_text = self.overlays_dict['overlay_target_text']
                        if target_len > 0 and targets_enabled == True:
                            
                            for target_name in targets_dict.keys():
                                target_dict = targets_dict[target_name]
                                #self.msg_if.pub_debug("Rendering image with target_dict: " + str(target_dict)  )
        

                                x_deg_offset = round(target_dict['x_deg_offset'] + 0.02, 1) 
                                x_ratio = ((self.width_deg/2) - x_deg_offset)/self.width_deg
                                x_offset_ratio = (x_ratio - 0.5)
                                x_scale = (self.width_proc/self.width_org)
                                x_offset_pixel = int((x_offset_ratio * self.width_org))
                                x_pixel = int((self.width_proc/2) + x_offset_pixel)
                                #self.msg_if.pub_debug("Rendering target x: " + str([x_deg_offset,x_ratio,x_offset_ratio,x_scale,x_offset_pixel,x_pixel])  , throttle_s = 5)


                                y_deg_offset = round(target_dict['y_deg_offset'] + 0.02, 1)
                                y_ratio = ((self.height_deg/2) + y_deg_offset)/self.height_deg
                                y_offset_ratio = -1 * (0.5 - y_ratio)
                                y_scale = (self.height_proc/self.height_org)
                                y_offset_pixel = int((y_offset_ratio * self.height_org))
                                y_pixel = int((self.height_proc/2) + y_offset_pixel)


                                pixel = [x_pixel,y_pixel]
                                pixel_adj = copy.deepcopy(pixel)
                                x_pixel_offset_adj = pixel[0]
                                y_pixel_offset_adj = pixel[1]
                                x_ratio_adj = copy.deepcopy(x_ratio)
                                y_ratio_adj = copy.deepcopy(y_ratio)


                                r_deg = 0
                                x_deg = 0
                                y_deg = 0
                                if self.live_adjust_dict['live_adjust_enabled'] == True:
                                    r_deg = self.live_adjust_dict['live_adjust_rotate_deg']
                                    x_deg = self.live_adjust_dict['live_adjust_x_deg']
                                    y_deg = self.live_adjust_dict['live_adjust_y_deg']
                                if self.stabilize_enabled == True:
                                    r_deg = r_deg + self.stabilize_rotate_deg
                                if r_deg != 0 or x_deg != 0 or y_deg != 0:
                                    x_ratio_adj = x_ratio_adj + (width / 2) * ( x_deg / (self.width_deg / 2)) / width
                                    y_ratio_adj = y_ratio_adj + (height / 2) * (y_deg / (self.height_deg / 2)) / height
                                    deg_adj =  r_deg
                                    x_pixel_offset_adj =  (x_ratio_adj * width) - (width / 2) 
                                    y_pixel_offset_adj =  (y_ratio_adj * height) - (height / 2)
                                    [x_pixel_offset_adj,y_pixel_offset_adj] = nepi_utils.rotate_point(x_pixel_offset_adj,y_pixel_offset_adj,deg_adj)
                                    x_pixel_offset_adj = int(x_pixel_offset_adj)
                                    y_pixel_offset_adj = int(y_pixel_offset_adj)
                                    x_ratio_adj = (x_pixel_offset_adj + (width / 2)) / width
                                    y_ratio_adj= (y_pixel_offset_adj + (height / 2)) / height
                                    pixel_adj = [int(x_ratio_adj * width), int(y_ratio_adj * height)]
                                [x_pixel,y_pixel] = pixel_adj   

                                #self.msg_if.pub_debug("Rendering target y: " + str([y_deg_offset,y_ratio,y_offset_ratio,y_scale,y_offset_pixel,y_pixel])  , throttle_s = 5)


                                target_rbg = target_dict['color_rgb']
                                target_msg = target_dict['msg_str']

                                overlay_text = []
                                if overlay_target_names == True:
                                    overlay_text.append(target_name)
                                if overlay_target_pixels == True:
                                    overlay_text.append(str(x_pixel) + ',' + str(y_pixel))
                                if overlay_target_degrees == True:
                                    overlay_text.append(str(round(x_deg_offset,0)) + ',' + str(round(y_deg_offset,0)))
                                if overlay_target_text == True and len(target_msg) > 0:
                                    overlay_text.append(str(target_msg))

                                targets_size_ratio = self.overlays_dict['targets_size_ratio']
                                targets_thickness_ratio = self.overlays_dict['targets_thickness_ratio']
                                targets_text_ratio = self.overlays_dict['targets_text_ratio']
                                #self.msg_if.pub_debug("Rendering image target: " + str([x_pixel,y_pixel])  )
                                cv2_img = nepi_img.overlay_target(cv2_img, 
                                                        x_px = x_pixel , y_px = y_pixel, 
                                                        color_rgb = target_rbg, 
                                                        size_ratio =  targets_size_ratio,
                                                        thickness_ratio = targets_thickness_ratio,
                                                        text_list = overlay_text,
                                                        text_ratio = targets_text_ratio)
                                

                        if cv2_img is not None:
                             cv2_img = self._aspectAdjust(cv2_img)

                        if cv2_img is not None:
                             cv2_img = self._zoomAdjust(cv2_img)

     

                        if cv2_img is not None:
                            [height,width] = cv2_img.shape[0:2]
                            if height > 5 and width > 5:
                                #self.msg_if.pub_debug("Got Processed size: " + str([height,width]) )



                                
                                # Apply Text Overlays
                                overlay_text_list = []
                                overlay_text_enabled = self.overlays_dict['overlay_text_enabled']
                                overlay_text_size_ratio = self.overlays_dict['overlay_text_size_ratio']
                                overlay_text_vert_ratio = self.overlays_dict['overlay_text_vert_ratio']
                                overlay_text_horz_ratio = self.overlays_dict['overlay_text_horz_ratio']
                                overlay_text_color_rgb = self.overlays_dict['overlay_text_color_rgb']
                                overlay_text_img_name = self.overlays_dict['overlay_text_img_name']
                                overlay_text_date_time = self.overlays_dict['overlay_text_date_time']
                                overlay_text_nav = self.overlays_dict['overlay_text_nav']
                                overlay_text_pose = self.overlays_dict['overlay_text_pose']
                                if overlay_text_enabled == True:
                                    if overlay_text_img_name == True:
                                        overlay = nepi_img.getImgShortName(self.namespace)
                                        overlay_text_list.append(overlay)
                                    
                                    if overlay_text_date_time == True:
                                        overlay = nepi_utils.get_datetime_str_from_timestamp(timestamp)
                                        overlay = overlay.replace('D','')
                                        overlay = overlay.replace('T',' T: ')
                                        overlay_text_list.append(overlay)

                                    if overlay_text_nav == True or overlay_text_pose == True:
                                        if navpose_dict is not None:
                                            if overlay_text_nav == True and navpose_dict is not None:
                                                overlay = 'Lat: ' +  str(round(navpose_dict['latitude'],6)) + ' Long: ' +  str(round(navpose_dict['longitude'],6)) + ' Head: ' +  str(round(navpose_dict['heading_deg'],0))
                                                overlay_text_list.append(overlay)

                                            if overlay_text_pose == True and navpose_dict is not None:
                                                overlay = 'Roll: ' +  str(round(navpose_dict['roll_deg'],0)) + ' Pitch: ' +  str(round(navpose_dict['pitch_deg'],0)) + ' Yaw: ' +  str(round(navpose_dict['yaw_deg'],0))
                                                overlay_text_list.append(overlay)

                                    overlay_text_list = overlay_text_list + self.overlays_dict['init_overlay_text_list'] + self.overlays_dict['add_overlay_text_list'] + add_overlay_text_list

                                    if len(overlay_text_list) > 0:
                                        start_x = (width * 0.01) + ((width * 0.99) * overlay_text_horz_ratio)
                                        start_y = (height * 0.01) + ((height * 0.90) * overlay_text_vert_ratio)
                                        cv2_img = nepi_img.overlay_text_list(cv2_img, 
                                                                text_list = overlay_text_list, 
                                                                x_px = start_x , y_px = start_y, 
                                                                color_rgb = overlay_text_color_rgb, 
                                                                apply_shadow = True, 
                                                                size_ratio = overlay_text_size_ratio )



                        if self.node_if is not None and self.needs_data == True and cv2_img is not None:
                            #self.msg_if.pub_debug("Publishing once")
                            #Convert to ros Image message
                            ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding=encoding)
                            sec = nepi_sdk.sec_from_timestamp(timestamp)
                            header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = navpose_dict['navpose_frame'])
                            #self.msg_if.pub_debug("Publishing image with header: " + str(header))
                            ros_img.header = header
                            self.publish_status()
                            self.node_if.publish_pub(self.node_if_prefix + 'data_pub', ros_img)



                            for namespace in add_pubs:
                                if namespace in self.add_pubs_dict.keys():
                                    [pub_namespace,status_ns,nav_ns] =  self.add_pubs_dict[namespace]
                                    #self.msg_if.pub_debug("Publishing Add Image on namespace: " + str(pub_namespace) , throttle_s = 5.0)
                                    self.node_if.publish_pub(status_ns, self.status_msg)
                                    self.node_if.publish_pub(pub_namespace, ros_img)
                                    
                            if pub_twice == True:
                                #self.msg_if.pub_debug("Publishing twice: " + str(pub_twice))
                                nepi_sdk.sleep(0.01)
                                self.node_if.publish_pub(self.node_if_prefix + 'data_pub', ros_img)
                                for namespace in add_pubs:
                                    if namespace in self.add_pubs_dict.keys():
                                        [pub_namespace,status_ns,nav_ns] =  self.add_pubs_dict[namespace]
                                        self.node_if.publish_pub(status_ns, self.status_msg)
                                        self.node_if.publish_pub(pub_namespace, ros_img)

                        # Save Data
                        if self.save_data_if is not None:
                            self.save_data_if.save(self.data_product,cv2_img,timestamp)
                        
                        # Update stats
                        process_time = round( (nepi_utils.get_time() - start_time) , 3)
                        self.status_msg.process_time = process_time
                        latency = (current_time - timestamp)
                        self.status_msg.pub_latency_time = latency
                        

                        if self.last_pub_time is None:
                            self.last_pub_time = nepi_utils.get_time()
                        else:
                            cur_time = nepi_utils.get_time()
                            pub_time_sec = cur_time - self.last_pub_time
                            self.last_pub_time = cur_time
                            self.status_msg.last_pub_sec = pub_time_sec

                            self.time_list.pop(0)
                            self.time_list.append(pub_time_sec)

                        # Update blank image if needed
                        if last_width != self.status_msg.width_px or last_height != self.status_msg.height_px:
                            self.blank_img = nepi_img.create_cv2_blank_img(width, height, color_rgb = (0, 0, 0) )


   
            except Exception as e:
                self.msg_if.pub_debug("Failed to publish image: " + str(e) , throttle_s = 5)
        self.publishing = False
        return cv2_img
        

    def publish_msg_img(self, msg_text, timestamp = None, navpose_frame = 'None'):
        """Publish a text message rendered onto the blank image template.

        Renders the message string over the cached blank image and publishes it
        as a ROS Image message on the primary data publisher.

        Args:
            msg_text (str): Text to render on the image.
            timestamp (float, optional): Timestamp in seconds. Defaults to current time.
            navpose_frame (str, optional): TF frame ID for the image header.
                Defaults to 'None'.
        """
        # overlay_text autoscales by default and draws in place, so draw on a copy of the cached blank
        cv2_img = nepi_img.overlay_text(copy.deepcopy(self.blank_img), msg_text)

        if timestamp == None:
            timestamp = nepi_utils.get_time()

        #Convert to ros Image message
        ros_img = nepi_img.cv2img_to_rosimg(cv2_img)
        sec = nepi_sdk.sec_from_timestamp(timestamp)
        ros_img.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = navpose_frame)
        if self.node_if is not None:
            self.node_if.publish_pub(self.node_if_prefix + 'data_pub', ros_img)

    

    ########################
    # Filter Functions

    def set_auto_adjust_enable(self, enabled):
        """Enable or disable automatic image adjustment.

        Args:
            enabled (bool): True to enable auto-adjust, False to disable.
        """
        if enabled:
            self.msg_if.pub_debug("Enabling Auto Adjust" )
        else:
            self.msg_if.pub_debug("Disabling Auto Adjust" )
        self.controls_dict['auto_adjust_enabled'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'auto_adjust_enabled', enabled)

    def set_auto_adjust_ratio(self, ratio):
        """Set the auto-adjust strength ratio, clamped to [0.0, 1.0].

        Args:
            ratio (float): Auto-adjust strength where 0.0 is no adjustment and
                1.0 is maximum adjustment.
        """
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['auto_adjust_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'auto_adjust_ratio', ratio)

    def set_brightness_ratio(self, ratio):
        """Set the image brightness ratio, clamped to [0.0, 1.0].

        Args:
            ratio (float): Brightness level where 0.0 is darkest and 1.0 is brightest.
        """
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['brightness_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'brightness_ratio', ratio)

    def set_contrast_ratio(self, ratio):
        """Set the image contrast ratio, clamped to [0.0, 1.0].

        Args:
            ratio (float): Contrast level where 0.0 is minimum and 1.0 is maximum.
        """
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['contrast_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'contrast_ratio', ratio)

    def set_threshold_ratio(self, ratio):
        """Set the image threshold (sharpness) ratio, clamped to [0.0, 1.0].

        Args:
            ratio (float): Sharpness level where 0.0 is no sharpening and
                1.0 is maximum sharpening.
        """
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['threshold_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'threshold_ratio', ratio)

    def set_filter_enable(self,name, enabled):
        """Enable or disable a named image filter.

        Args:
            name (str): Name of the filter as registered in the filter dictionary.
            enabled (bool): True to enable the filter, False to disable it.
        """
        if self.filter_dict is not None:
            if name in self.filter_dict.keys():
                was_enabled = self.filter_dict[name]['enabled']
                if was_enabled != enabled:
                    if enabled == True:
                        self.msg_if.pub_debug("Enabling Filter: " + name )
                    else:
                        self.msg_if.pub_debug("Disabling Filter: " + name )
                    self.filter_dict[name]['enabled'] = enabled
                    self.publish_status()
                    self.needs_update()
                    if self.node_if is not None:
                        self.node_if.set_param(self.node_if_prefix +  'filter_dict', self.filter_dict)

    def set_filter_ratio(self,name, ratio):
        """Set the intensity ratio for a named image filter, clamped to [0.0, 1.0].

        Args:
            name (str): Name of the filter as registered in the filter dictionary.
            ratio (float): Filter intensity where 0.0 is no effect and 1.0 is
                maximum effect.
        """
        if self.filter_dict is not None:
            if ratio < 0:
                ratio = 0
            if ratio > 1.0:
                ratio = 1.0
            if name in self.filter_dict.keys():
                self.msg_if.pub_debug("Setting Filter Ratio: " + name + " : " + str(ratio) )
                self.filter_dict[name]['ratio'] = ratio
                self.publish_status()
                self.needs_update()
                self.node_if.set_param(self.node_if_prefix +  'filter_dict', self.filter_dict)

    ########################
    # Res and Orientation Functions

    def set_resolution_ratio(self, ratio):
        """Set the output image resolution ratio, clamped to [0.2, 1.0].

        Args:
            ratio (float): Fraction of full resolution, where 1.0 is full resolution
                and 0.2 is the minimum allowed.
        """
        if (ratio < 0.2):
            ratio = 0.2
        if (ratio > 1.0):
            ratio = 1.0
        self.controls_dict['resolution_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'resolution_ratio', ratio)

    def set_rotate_2d_deg(self, deg):
        """Set the 2-D rotation angle, rounded to the nearest integer degree.

        Args:
            deg (float): Rotation angle in degrees.
        """
        deg_int = int(round(deg,0))
        self.controls_dict['rotate_2d_deg'] = deg_int
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'rotate_2d_deg', deg_int)

    def set_rotate_2d_swap_box(self, enabled):
        """Enable or disable swapping the Free Cam output box dimensions (W/H).

        Swaps only the output box aspect (e.g. landscape <-> portrait); the image
        content orientation is unaffected and stays rotated by the current angle.

        Args:
            enabled (bool): True to swap the output box aspect, False for the original box.
        """
        self.controls_dict['rotate_2d_swap_box'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'rotate_2d_swap_box', enabled)

    def set_flip_horz(self, enabled):
        """Enable or disable horizontal image flip.

        Args:
            enabled (bool): True to flip horizontally, False to disable.
        """
        self.controls_dict['flip_horz'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'flip_horz', enabled)

    def set_flip_vert(self, enabled):
        """Enable or disable vertical image flip.

        Args:
            enabled (bool): True to flip vertically, False to disable.
        """
        self.controls_dict['flip_vert'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'flip_vert', enabled)

    ########################
    # Render Functions

    def set_range_ratios(self, start_ratio, stop_ratio):
        """Set the normalized depth range window for image rendering.

        Both ratios must be in [0, 1] and start_ratio must be less than or
        equal to stop_ratio.

        Args:
            start_ratio (float): Start of the displayed depth range as a fraction
                of the total sensor range.
            stop_ratio (float): End of the displayed depth range as a fraction of
                the total sensor range.
        """
        if (start_ratio < 0 or stop_ratio > 1 or stop_ratio < start_ratio):
            self.msg_if.pub_error("Range values out of bounds" )
            self.publish_status() # No change
            return

        self.controls_dict['start_range_ratio'] = start_ratio
        self.controls_dict['stop_range_ratio'] = stop_ratio

        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'start_range_ratio', start_ratio)
            self.node_if.set_param(self.node_if_prefix +  'stop_range_ratio', stop_ratio)


    def set_zoom_ratio(self, ratio):
        """Set the zoom level, recalculating the image crop window accordingly.

        A zoom of 0.0 shows the full image and 1.0 crops to the maximum zoom
        centered on the current pan position.

        Args:
            ratio (float): Zoom level in [0.0, 1.0].
        """
        self.drag_pixel = None
        self.drag_window = None

        # Update Ratios

        wrs = copy.deepcopy(self.controls_dict['window_ratios'])
        xr_ratio = wrs[0] + (wrs[1] - wrs[0]) / 2
        yr_ratio = wrs[2] + (wrs[3] - wrs[2]) / 2

        xlen_max_r = (1 - abs(xr_ratio - 0.)) * 2
        ylen_max_r = (1 - abs(yr_ratio - 0.5)) * 2
        len_min_r = 0.05
        len_max_r = min(xlen_max_r, ylen_max_r)

        len_zoom = 1 - ratio * len_max_r
        if len_zoom < len_min_r:
            len_zoom = len_min_r

        xr_min = xr_ratio - len_zoom / 2
        xr_max = xr_ratio + len_zoom / 2
        if xr_min < 0:
            xr_min = 0
            xr_max = len_zoom
        if xr_max > 1:
            xr_min = 1 - len_zoom
            xr_max = 1
    
        yr_min = yr_ratio - len_zoom / 2
        yr_max = yr_ratio + len_zoom / 2
        if yr_min < 0:
            yr_min = 0
            yr_max = len_zoom
        if yr_max > 1:
            yr_min = 1 - len_zoom
            yr_max = 1

        self.msg_if.pub_debug("Zoom Image Window: " + str([xr_min, xr_max, yr_min, yr_max]) )
        self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
        self.x_ratio = xr_min + (xr_max - xr_min) / 2
        self.y_ratio = yr_min + (yr_max - yr_min) / 2
        self.zoom_ratio = ratio


        self.publish_status() 
        self.needs_update()


    def set_pixel(self, pixel, color_bgr = [0,0,0,0]):
        """Center the image crop window on a selected pixel coordinate.

        Recenters the current zoom window so that the given pixel becomes the
        center of the view, preserving the existing window size.

        Args:
            pixel (list): [x, y] pixel coordinates in the org image space.
            color_bgr (list, optional): BGRA color of the selected pixel.
                Defaults to [0, 0, 0, 0].
        """
        self.drag_pixel = None
        self.drag_window = None

        if self.width_org > 10 and self.height_org > 10:
            # Update Ratios
            xr_ratio = pixel[0] / self.width_org
            yr_ratio = pixel[1] / self.height_org
            wrs = copy.deepcopy(self.controls_dict['window_ratios'])

            xr_len = wrs[1] - wrs[0]
            xr_min = xr_ratio - xr_len / 2
            xr_max = xr_ratio + xr_len / 2
            if xr_min < 0:
                xr_min = 0
                xr_max = xr_len
            if xr_max > 1:
                xr_min = 1 - xr_len
                xr_max = 1
        
            yr_len = wrs[3] - wrs[2]
            yr_min = yr_ratio - yr_len / 2
            yr_max = yr_ratio + yr_len / 2
            if yr_min < 0:
                yr_min = 0
                yr_max = yr_len
            if yr_max > 1:
                yr_min = 1 - yr_len
                yr_max = 1

            self.msg_if.pub_debug("Pixel Image Window: " + str([xr_min, xr_max, yr_min, yr_max]) )
            self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
            self.x_ratio = xr_min + (xr_max - xr_min) / 2
            self.y_ratio = yr_min + (yr_max - yr_min) / 2
            self.zoom_ratio = 1 - max(xr_len, yr_len)

            self.publish_status()  
            self.needs_update()    



    def set_x_ratio(self, ratio):
        """Pan the image crop window horizontally.

        Moves the horizontal center of the crop window to the position
        corresponding to the given ratio within the pannable range.

        Args:
            ratio (float): Horizontal pan position in [0.0, 1.0], where 0.0 is
                full left and 1.0 is full right.
        """
        self.drag_pixel = None
        self.drag_window = None


        wrs = copy.deepcopy(self.controls_dict['window_ratios'])
        xr_len = wrs[1] - wrs[0]
        yr_len = wrs[3] - wrs[2]

        r_min = xr_len / 2
        r_max = 1 - xr_len / 2
        xr_r = r_min + (ratio * (r_max - r_min))

        xr_min = xr_r - xr_len / 2
        xr_max = xr_r + xr_len / 2
        if xr_min < 0:
            xr_min = 0
            xr_max = xr_len
        if xr_max > 1:
            xr_min = 1 - xr_len
            xr_max = 1
    
        yr_r = wrs[2] + yr_len / 2
        yr_min = yr_r - xr_len / 2
        yr_max = yr_r + xr_len / 2
        if yr_min < 0:
            yr_min = 0
            yr_max = xr_len
        if yr_max > 1:
            yr_min = 1 - xr_len
            yr_max = 1

        self.msg_if.pub_debug("X Ratio Image Window: " + str([xr_min, xr_max, yr_min, yr_max]) )
        self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
        self.x_ratio = xr_min + (xr_max - xr_min) / 2
        self.y_ratio = yr_min + (yr_max - yr_min) / 2
        self.zoom_ratio = 1 - xr_len

        self.publish_status() 
        self.needs_update()

    def set_y_ratio(self, ratio):
        """Pan the image crop window vertically.

        Moves the vertical center of the crop window to the position
        corresponding to the given ratio within the pannable range.

        Args:
            ratio (float): Vertical pan position in [0.0, 1.0], where 0.0 is
                top and 1.0 is bottom.
        """
        self.drag_pixel = None
        self.drag_window = None


        wrs = copy.deepcopy(self.controls_dict['window_ratios'])
        xr_len = wrs[1] - wrs[0]
        yr_len = wrs[3] - wrs[2]

        r_min = yr_len / 2
        r_max = 1 - yr_len / 2
        yr_r = r_min + (ratio * (r_max - r_min))

        yr_min = yr_r - yr_len / 2
        yr_max = yr_r + yr_len / 2
        if yr_min < 0:
            yr_min = 0
            yr_max = yr_len
        if yr_max > 1:
            yr_min = 1 - yr_len
            yr_max = 1
    
        xr_r = wrs[0] + xr_len / 2
        xr_min = xr_r - yr_len / 2
        xr_max = xr_r + yr_len / 2
        if xr_min < 0:
            xr_min = 0
            xr_max = yr_len
        if xr_max > 1:
            xr_min = 1 - yr_len
            xr_max = 1

        self.msg_if.pub_debug("Y Ratio Image Window: " + str([xr_min, xr_max, yr_min, yr_max]) )
        self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
        self.x_ratio = xr_min + (xr_max - xr_min) / 2
        self.y_ratio = yr_min + (yr_max - yr_min) / 2
        self.zoom_ratio = 1 - yr_len

        self.publish_status() 
        self.needs_update()


    def set_window(self, window):
        """Set the image crop window from absolute pixel coordinates.

        Converts the pixel-space window into normalized ratio coordinates and
        updates the crop window. Requires the org image dimensions to be known.

        Args:
            window (list): [x_min, x_max, y_min, y_max] in org image pixel space.
        """
        self.drag_pixel = None
        self.drag_window = None
 
        if self.width_org > 10 and self.height_org > 10:
            # Update Ratios
            xr_len = (window[1] - window[0]) / self.width_org
            yr_len = (window[3] - window[2]) / self.height_org
            xr_ratio = window[0] / self.width_org + (xr_len / 2) 
            yr_ratio = window[2] / self.height_org + (yr_len / 2)

            r_len_max = max(xr_len, yr_len)

            xr_min = xr_ratio - r_len_max / 2
            xr_max = xr_ratio + r_len_max / 2
            if xr_min < 0:
                xr_min = 0
                xr_max = r_len_max
            if xr_max > 1:
                xr_min = 1 - r_len_max
                xr_max = 1
        

            yr_min = yr_ratio - r_len_max / 2
            yr_max = yr_ratio + r_len_max / 2
            if yr_min < 0:
                yr_min = 0
                yr_max = r_len_max
            if yr_max > 1:
                yr_min = 1 - r_len_max
                yr_max = 1

            self.msg_if.pub_debug("Window Image Window set to: " + str([xr_min, xr_max, yr_min, yr_max]) )
            self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
            self.x_ratio = xr_min + (xr_max - xr_min) / 2
            self.y_ratio = yr_min + (yr_max - yr_min) / 2
            self.zoom_ratio = 1 - r_len_max 

            self.publish_status()  
            self.needs_update()    
            self.publish_status()  
            self.needs_update()



    def update_window_ratios(self):
        """Republish status and request a new frame after a window ratio change."""
        self.window_ratios = copy.deepcopy(self.controls_dict['window_ratios'])
        self.publish_status()
        self.needs_update()

    def set_zoom_3d_ratio(self, ratio):
        """Set the 3-D rotation ratio, clamped to a valid ratio range.

        Args:
            ratio (float): Rotation position in [0.0, 1.0].
        """
        self.controls_dict['zoom_3d_ratio'] = nepi_utils.check_ratio(ratio)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'zoom_3d_ratio', self.controls_dict['zoom_3d_ratio'])

    def set_rotate_3d_ratio(self, ratio):
        """Set the 3-D rotation ratio, clamped to a valid ratio range.

        Args:
            ratio (float): Rotation position in [0.0, 1.0].
        """
        self.controls_dict['rotate_3d_ratio'] = nepi_utils.check_ratio(ratio)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'rotate_3d_ratio', self.controls_dict['rotate_3d_ratio'])

    def set_tilt_3d_ratio(self, ratio):
        """Set the 3-D tilt ratio, clamped to a valid ratio range.

        Args:
            ratio (float): Tilt position in [0.0, 1.0].
        """
        self.controls_dict['tilt_3d_ratio'] = nepi_utils.check_ratio(ratio)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'tilt_3d_ratio', self.controls_dict['tilt_3d_ratio'])

    ########################

    def set_aspect_adjust_enable(self,enabled):
        self.aspect_adjust_enabled = enabled and self.aspect_adjustment_disabled == False
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'aspect_adjust_enabled', self.aspect_adjust_enabled)

    def set_aspect_adjust_ratio(self,aspect_ratio):
        if aspect_ratio is not None:
            if aspect_ratio >= 0.5 and aspect_ratio <= 2.5:
                self.aspect_ratio_set = aspect_ratio #nepi_img.get_aspect_ratio_clean(aspect_ratio)
                if self.node_if is not None:
                    self.node_if.set_param(self.node_if_prefix +  'aspect_ratio_set', self.aspect_ratio_set)

    def set_aspect_adjust_by_ratio(self,ratio):
        ratio = nepi_utils.check_ratio(ratio)
        aspect_ratio = 1 + ratio
        self.set_aspect_adjust_ratio(aspect_ratio)
           

        

    def set_stream_compression_enable(self,enabled):
        self.stream_compression_enabled = enabled
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'stream_compression_enabled', self.stream_compression_enabled)

    def set_stream_compression_ratio(self,ratio):
        self.stream_compression_ratio = nepi_utils.check_ratio(ratio)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'stream_compression_ratio', self.stream_compression_ratio)



    ########################
    # Overlay Functions
    def set_overlay_text_enable(self,enabled):
        """Enable or disable the text overlays.

        Args:
            enabled (bool): True to show text data, False to hide it.
        """
        self.overlays_dict['overlay_text_enabled'] = enabled
        if enabled == False:
            self.click_text_enabled = False
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)



    def set_click_text(self,enabled):
        """Enable or disable the click text enable.

        Args:
            enabled (bool): True to show text data, False to hide it.
        """
        self.click_text_enabled = enabled
        self.click_crosshair_enabled = False
        self.click_target_enabled = False
        self.publish_status()

    def set_overlay_text_size_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['overlay_text_size_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_vert_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['overlay_text_vert_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_horz_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['overlay_text_horz_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_transparency_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['overlay_text_transparency_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_color_rgb(self, r = 0, g = 255, b = 0):
        self.overlays_dict['overlay_text_color_rgb'] = (r,g,b)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_image_name(self,enabled):
        """Enable or disable the image name text overlay.

        Args:
            enabled (bool): True to show the image source name, False to hide it.
        """
        self.overlays_dict['overlay_text_img_name'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_date_time(self,enabled):
        """Enable or disable the date/time text overlay.

        Args:
            enabled (bool): True to show the timestamp, False to hide it.
        """
        self.overlays_dict['overlay_text_date_time'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_nav(self,enabled):
        """Enable or disable the GPS navigation (lat/lon/heading) text overlay.

        Args:
            enabled (bool): True to show navigation data, False to hide it.
        """
        self.overlays_dict['overlay_text_nav'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_pose(self,enabled):
        """Enable or disable the roll/pitch/yaw pose text overlay.

        Args:
            enabled (bool): True to show pose data, False to hide it.
        """
        self.overlays_dict['overlay_text_pose'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text_list(self,overlay_text_list):
        """Replace the additional text overlay list with the provided list.

        Args:
            overlay_text_list (list): List of text strings to display as overlays.
        """
        self.overlays_dict['add_overlay_text_list'] = overlay_text_list
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_text(self,overlay_text):
        """Append a text string to the additional overlay list.

        Args:
            overlay_text_text (str): Text string to add to the overlay list.
        """
        overlay_text_list = self.overlays_dict['add_overlay_text_list']
        overlay_text_list.append(overlay_text)
        self.overlays_dict['add_overlay_text_list'] = overlay_text_list
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def clear_overlay_text_list(self):
        """Clear all entries from the additional text overlay list."""
        self.overlays_dict['add_overlay_text_list'] = []
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    ###################################
    def set_crosshairs_enable(self,enabled):
        """Enable or disable the crosshair overlays.

        Args:
            enabled (bool): True to show crosshairs data, False to hide it.
        """
        was_enabled = copy.deepcopy(self.overlays_dict['crosshairs_enabled'])
        self.overlays_dict['crosshairs_enabled'] = enabled
        if enabled == False:
            self.click_crosshair_enabled = False
        if was_enabled != enabled:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_click_crosshair(self,enabled):
        """Enable or disable the click crosshair enable.

        Args:
            enabled (bool): True to show crosshairs data, False to hide it.
        """
        self.msg_if.pub_debug("Setting Click Crosshair Enable" + str(enabled) )
        self.click_text_enabled = False
        self.click_crosshair_enabled = enabled
        self.click_target_enabled = False
        self.publish_status()

    def set_crosshairs_size_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['crosshairs_size_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['crosshairs_size_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_crosshairs_thickness_ratio(self, ratio):
        """Set the relative thickness of crosshairs overlays on the image.

        Args:
            ratio (float): Text thickness ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['crosshairs_thickness_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['crosshairs_thickness_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_crosshairs_text_ratio(self, ratio):
        """Set the relative text of crosshairs overlays on the image.

        Args:
            ratio (float): Text text ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['crosshairs_text_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['crosshairs_text_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_crosshairs_transparency_ratio(self, ratio):
        """Set the relative size of crosshairs overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['targets_thickness_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['crosshairs_transparency_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    def set_crosshairs_color_rgb(self, r = 0, g = 255, b = 0):
        self.overlays_dict['crosshairs_color_rgb'] = (r,g,b)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)
      



    def set_overlay_crosshair_names(self,enabled):
        """Enable or disable the crosshair names overlays.

        Args:
            enabled (bool): True to show crosshairs name data, False to hide it.
        """
        self.overlays_dict['overlay_crosshair_names'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_crosshair_pixels(self,enabled):
        """Enable or disable the crosshair pixels overlays.

        Args:
            enabled (bool): True to show crosshairs pixel data, False to hide it.
        """
        self.overlays_dict['overlay_crosshair_pixels'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_crosshair_degrees(self,enabled):
        """Enable or disable the crosshair degrees overlays.

        Args:
            enabled (bool): True to show crosshairs degrees data, False to hide it.
        """
        self.overlays_dict['overlay_crosshair_degrees'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_crosshair_text(self,enabled):
        """Enable or disable the crosshair messages overlays.

        Args:
            enabled (bool): True to show crosshairs messages, False to hide it.
        """
        self.overlays_dict['overlay_crosshair_text'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    def add_crosshair_pixels(self, x_pixel, y_pixel, name = None, color_rgb = None, msg_str = ''):
        """Append a crosshair overlay at pixel location.

        Args:
            x pixel, y pixel, crosshair name str.
        """
        x_ratio = float(x_pixel / self.width_org) 
        y_ratio = float(y_pixel / self.height_org)
        x_deg =  -1 * round(  ((x_ratio - 0.5) * self.width_deg),1)
        y_deg = round(  ((y_ratio - 0.5) * self.height_deg),1)  
        self.add_crosshair_degs(x_deg, y_deg, name, color_rgb, msg_str)


    def add_crosshair_ratios(self, x_ratio, y_ratio, name = None, color_rgb = None, msg_str = ''):
        """Append a crosshair overlay at pixel location.

        Args:
            x pixel, y pixel, crosshair name str.
        """
        x_ratio = nepi_utils.check_ratio(x_ratio)
        y_ratio = nepi_utils.check_ratio(y_ratio)
        x_deg = -1 * round(  ((x_ratio - 0.5) * self.width_deg),1)
        y_deg = round(  ((y_ratio - 0.5) * self.height_deg),1)  
        self.msg_if.pub_debug("Adding Crosshair ratio to degs: " + str([x_ratio,y_ratio,x_deg,y_deg]) , throttle_s = 5)
        self.add_crosshair_degs(x_deg, y_deg, name, color_rgb, msg_str)


    def add_crosshair_degs(self, x_deg, y_deg, name = None, color_rgb = None, msg_str = ''):
        """Append a crosshair overlay at degs offset location.

        Args:
            x deg, y deg, crosshair name str.
        """
        crosshairs_dict = self.overlays_dict['crosshairs_dict']
        crosshair_names = list(crosshairs_dict.keys())
        num_crosshairs = len(crosshair_names)
        ch_name = str(num_crosshairs + 1)
        if name is not None:
            if name != '':
                ch_name = name
        if color_rgb is None:
            color_rgb = self.overlays_dict['crosshairs_color_rgb']
        crosshair_dict = copy.deepcopy(self.BLANK_CROSSHAIR_DICT)
        crosshair_dict['x_deg_offset'] = round(x_deg, 2)
        crosshair_dict['y_deg_offset'] = round(y_deg, 2)
        crosshair_dict['color_rgb'] = color_rgb
        crosshair_dict['msg_str'] = msg_str
        #self.msg_if.pub_debug("Adding Crosshair: " + str(crosshair_dict) )
        needs_update = True
        if ch_name in self.overlays_dict['crosshairs_dict'].keys():
            if self.overlays_dict['crosshairs_dict'][ch_name] == crosshair_dict:
                needs_update = False
        if needs_update == True:
            self.overlays_dict['crosshairs_dict'][ch_name] = crosshair_dict
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def remove_crosshair(self, name):
        """Remove entry from crosshairs overlay dict."""
        try:
            del self.overlays_dict['crosshairs_dict'][name]
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)
        except:
            pass
        
        

    def clear_crosshairs(self):
        """Clear all entries from crosshairs overlay dict."""
        if len(list(self.overlays_dict['crosshairs_dict'].keys())) > 0:
            self.overlays_dict['crosshairs_dict'] = dict()
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    ###################################
    def set_targets_enable(self,enabled):
        """Enable or disable the target overlays.

        Args:
            enabled (bool): True to show targets data, False to hide it.
        """
        was_enabled = copy.deepcopy(self.overlays_dict['targets_enabled'])
        self.overlays_dict['targets_enabled'] = enabled
        if enabled == False:
            self.click_target_enabled = False
        if was_enabled != enabled:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_click_target(self,enabled):
        """Enable or disable the click target enable.

        Args:
            enabled (bool): True to show targets data, False to hide it.
        """
        self.click_text_enabled = False
        self.click_crosshair_enabled = False
        self.click_target_enabled = enabled
        self.publish_status()

    def set_targets_size_ratio(self, ratio):
        """Set the relative size of text overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['targets_size_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['targets_size_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_targets_thickness_ratio(self, ratio):
        """Set the relative thickness of targets overlays on the image.

        Args:
            ratio (float): Text thickness ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['targets_thickness_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['targets_thickness_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_targets_text_ratio(self, ratio):
        """Set the relative text of targets overlays on the image.

        Args:
            ratio (float): Text text ratio in [0.0, 1.0].
        """
        cur_ratio = copy.deepcopy(self.overlays_dict['targets_text_ratio'])
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['targets_text_ratio'] = ratio
        if cur_ratio != ratio:
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_targets_transparency_ratio(self, ratio):
        """Set the relative size of targets overlays on the image.

        Args:
            ratio (float): Text size ratio in [0.0, 1.0].
        """
        ratio = nepi_utils.check_ratio(ratio)
        self.overlays_dict['targets_transparency_ratio'] = ratio
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    def set_targets_color_rgb(self, r = 0, g = 255, b = 0):
        self.overlays_dict['targets_color_rgb'] = (r,g,b)
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)
      



    def set_overlay_target_names(self,enabled):
        """Enable or disable the target names overlays.

        Args:
            enabled (bool): True to show targets name data, False to hide it.
        """
        self.overlays_dict['overlay_target_names'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_target_pixels(self,enabled):
        """Enable or disable the target pixels overlays.

        Args:
            enabled (bool): True to show targets pixel data, False to hide it.
        """
        self.overlays_dict['overlay_target_pixels'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_target_degrees(self,enabled):
        """Enable or disable the target degrees overlays.

        Args:
            enabled (bool): True to show targets degrees data, False to hide it.
        """
        self.overlays_dict['overlay_target_degrees'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def set_overlay_target_text(self,enabled):
        """Enable or disable the target messages overlays.

        Args:
            enabled (bool): True to show targets messages, False to hide it.
        """
        self.overlays_dict['overlay_target_text'] = enabled
        self.publish_status()
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)


    def add_target_pixels(self, x_pixel, y_pixel, name = None, color_rgb = None, msg_str = ''):
        """Append a target overlay at pixel location.

        Args:
            x pixel, y pixel, target name str.
        """
        x_ratio = float(x_pixel / self.width_org) 
        y_ratio = float(y_pixel / self.height_org)
        x_deg =  -1 * round(  ((x_ratio - 0.5) * self.width_deg),1)
        y_deg = round(  ((y_ratio - 0.5) * self.height_deg),1)  
        self.add_target_degs(x_deg, y_deg, name, color_rgb, msg_str)


    def add_target_ratios(self, x_ratio, y_ratio, name = None, color_rgb = None, msg_str = ''):
        """Append a target overlay at pixel location.

        Args:
            x pixel, y pixel, target name str.
        """
        x_ratio = nepi_utils.check_ratio(x_ratio)
        y_ratio = nepi_utils.check_ratio(y_ratio)
        x_deg =  -1 * round(  ((x_ratio - 0.5) * self.width_deg),1)
        y_deg = round(  ((y_ratio - 0.5) * self.height_deg),1)  
        self.add_target_degs(x_deg, y_deg, name, color_rgb, msg_str)


    def add_target_degs(self, x_deg, y_deg, name = None, color_rgb = None, msg_str = ''):
        """Append a target overlay at degs offset location.

        Args:
            x deg, y deg, target name str.
        """
        targets_dict = self.overlays_dict['targets_dict']
        target_names = list(targets_dict.keys())
        num_targets = len(target_names)
        ch_name = str(num_targets + 1)
        if name is not None:
            if name != '':
                ch_name = name
        if color_rgb is None:
            color_rgb = self.overlays_dict['targets_color_rgb']
        target_dict = copy.deepcopy(self.BLANK_CROSSHAIR_DICT)
        target_dict['x_deg_offset'] = round(x_deg)
        target_dict['y_deg_offset'] = round(y_deg)
        target_dict['color_rgb'] = color_rgb
        target_dict['msg_str'] = msg_str
        #self.msg_if.pub_debug("Adding Target: " + str(target_dict) )
        needs_update = True
        if ch_name in self.overlays_dict['targets_dict'].keys():
            if self.overlays_dict['targets_dict'][ch_name] == target_dict:
                needs_update = False
        if needs_update == True:
            self.overlays_dict['targets_dict'][ch_name] = target_dict
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)

    def remove_target(self, name):
        """Remove entry from targets overlay dict."""
        try:
            del self.overlays_dict['targets_dict'][name]
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)
        except:
            pass
        


    def clear_targets(self):
        """Clear all entries from targets overlay dict."""
        if len(list(self.overlays_dict['targets_dict'].keys())) > 0:
            self.overlays_dict['targets_dict'] = dict()
            self.publish_status()
            self.needs_update()
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix +  'overlays_dict', self.overlays_dict)




    #############################

    def set_live_adjust_enable(self,enabled):
        self.live_adjust_dict['live_adjust_enabled'] = enabled
        self.live_adjust_enabled = self.live_adjust_dict['live_adjust_enabled']
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'live_adjust_enabled', self.live_adjust_enabled)

    def set_live_adjust_rotate_ratio(self,ratio):
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            ratio = nepi_utils.check_ratio(ratio)
            deg = 2 * (ratio - 0.5) * 180
            self.live_adjust_dict['live_adjust_rotate_deg'] = deg
            

    def set_live_adjust_rotate_deg(self,deg):
        #self.msg_if.pub_debug("Received Live Adjust Rotate Deg: " + str(deg) )
        if abs(deg) > 180:
                deg = np.sign(deg) * 180
        #self.msg_if.pub_debug("Updated Live Adjust Rotate Deg: " + str(deg) )
        ratio = nepi_utils.check_ratio(0.5 + (deg / 180)/2)
        #self.msg_if.pub_debug("Received Live Adjust Rotate Ratio: " + str(ratio) )
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            self.live_adjust_dict['live_adjust_rotate_deg'] = deg

    def set_live_adjust_x_ratio(self,ratio):
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            ratio = nepi_utils.check_ratio(ratio)
            self.live_adjust_dict['live_adjust_x_deg'] = ((ratio - 0.5) * -2) * self.width_deg

    def set_live_adjust_x_pixel(self,pixel):
        if abs(pixel) > self.width_org:
            pixel = np.sign(pixel) * self.width_org
        ratio = round(0.5 + (pixel / self.width_org)/2,4)
        ratio = nepi_utils.check_ratio(ratio)
        self.set_live_adjust_x_ratio(ratio)

    def set_live_adjust_x_deg(self,deg):
        if abs(deg) > self.width_deg:
            deg = np.sign(deg) * self.width_deg
        self.live_adjust_dict['live_adjust_x_deg'] = deg
            

    def set_live_adjust_y_ratio(self,ratio):
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            ratio = nepi_utils.check_ratio(ratio)
            self.live_adjust_dict['live_adjust_y_deg'] = ((ratio - 0.5) * 2) * self.height_deg

    def set_live_adjust_y_pixel(self,pixel):
        if abs(pixel) > self.height_org:
            pixel = np.sign(pixel) * self.height_org
        ratio = round(0.5 + (pixel / self.height_org)/2,4)
        self.set_live_adjust_y_ratio(ratio)
        
    def set_live_adjust_y_deg(self,deg):
        if abs(deg) > self.height_deg:
            deg = np.sign(deg) * self.height_deg
        self.live_adjust_dict['live_adjust_y_deg'] = deg

    #############################

    def set_stabilize_enable(self,enabled):
        self.stabilize_enabled = enabled
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix +  'stabilize_enabled', self.stabilize_enabled)

    #############################
    def reset_filters(self):
        """Reset all filter and adjustment controls to factory defaults."""
        # First reset controls to init dict to capture non param managed settings
        self.controls_dict = copy.deepcopy(self.init_controls_dict)

        self.node_if.factory_reset_param('auto_adjust_enabled')
        self.node_if.factory_reset_param('auto_adjust_ratio')
        self.node_if.factory_reset_param('brightness_ratio')
        self.node_if.factory_reset_param('contrast_ratio')
        self.node_if.factory_reset_param('threshold_ratio')
        self.node_if.factory_reset_param('filter_dict')

        if self.node_if is not None:
            self.controls_dict['auto_adjust_enabled'] = self.node_if.get_param(self.node_if_prefix +  'auto_adjust_enabled')
            self.controls_dict['auto_adjust_ratio'] = self.node_if.get_param(self.node_if_prefix +  'auto_adjust_ratio')
            self.controls_dict['brightness_ratio'] = self.node_if.get_param(self.node_if_prefix +  'brightness_ratio')
            self.controls_dict['contrast_ratio'] = self.node_if.get_param(self.node_if_prefix +  'contrast_ratio')
            self.controls_dict['threshold_ratio'] = self.node_if.get_param(self.node_if_prefix +  'threshold_ratio')
            filter_dict = self.node_if.get_param(self.node_if_prefix +  'filter_dict')
            if filter_dict is not None:
                self.filter_dict = filter_dict

        self.publish_status()  
        self.needs_update()


    def reset_overlays(self):
        """Reset all overlay settings to factory defaults."""
        if self.node_if is not None:
            self.node_if.factory_reset_param('overlays_dict')
            self.overlays_dict = self.node_if.get_param(self.node_if_prefix +  'overlays_dict')
       
        
        self.publish_status()  
        self.needs_update()


    def reset_settings(self):
        """Reset resolution, rotation, and flip settings to factory defaults."""
        # First reset controls to init dict to capture non param managed settings
        self.controls_dict = copy.deepcopy(self.init_controls_dict)
        if self.node_if is not None:
            self.node_if.factory_reset_param('resolution_ratio')
            self.node_if.factory_reset_param('rotate_2d_deg')
            self.node_if.factory_reset_param('rotate_2d_swap_box')
            self.node_if.factory_reset_param('flip_horz')
            self.node_if.factory_reset_param('flip_vert')


            self.controls_dict['resolution_ratio'] = self.node_if.get_param(self.node_if_prefix +  'resolution_ratio')
            self.controls_dict['rotate_2d_deg'] = self.node_if.get_param(self.node_if_prefix +  'rotate_2d_deg')
            self.controls_dict['rotate_2d_swap_box'] = self.node_if.get_param(self.node_if_prefix +  'rotate_2d_swap_box')
            self.controls_dict['flip_horz'] = self.node_if.get_param(self.node_if_prefix +  'flip_horz')
            self.controls_dict['flip_vert'] = self.node_if.get_param(self.node_if_prefix +  'flip_vert')

            filter_dict = self.node_if.get_param(self.node_if_prefix +  'filter_dict')
            if filter_dict is not None:
                self.filter_dict = filter_dict

        self.publish_status()  
        self.needs_update()

    def reset_renders(self):
        """Reset all render controls (zoom, pan, window, range, 3-D rotation) to defaults."""
        self.msg_if.pub_debug("Reseting render values" )
        self.drag_pixel = None
        self.drag_window = None
        self.zoom_ratio = 0
        self.x_ratio = 0.5
        self.y_ratio = 0.5
        self.x_offset = 0
        self.y_offset = 0
        self.x_scaler = 1
        self.y_scaler = 1
        self.controls_dict = copy.deepcopy(self.init_controls_dict)


        # self.controls_dict['start_range_ratio'] = 0
        # self.controls_dict['stop_range_ratio'] = 1
        # self.controls_dict['window_ratios'] = [0,1,0,1]

        # self.window_ratios = [0,1,0,1]

        # self.controls_dict['rotate_2d_ratio'] = 0.5
        # self.controls_dict['rotate_3d_ratio'] = 0.5
        # self.controls_dict['tilt_3d_ratio'] = 0.5

        if self.node_if is not None:
            pass
            # self.node_if.factory_reset_param('start_range_ratio')
            # self.node_if.factory_reset_param('stop_range_ratio')

            # self.controls_dict['start_range_ratio'] = self.node_if.get_param(self.node_if_prefix +  'start_range_ratio')
            # self.controls_dict['stop_range_ratio'] = self.node_if.get_param(self.node_if_prefix +  'stop_range_ratio')

        self.live_adjust_dict['live_adjust_rotate_deg'] = 0
        self.live_adjust_dict['live_adjust_x_deg'] = 0
        self.live_adjust_dict['live_adjust_y_deg'] = 0


        self.publish_status()  
        self.needs_update()





    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self):
        self.msg_if.pub_debug("Got Reset params callback " )
        self.reset()

    def _factoryResetCb(self):
        self.msg_if.pub_debug("Got Factory Reset params callback" )
        self.factory_reset()

    def _updaterCb(self, timer):

        # Check for other topics
        image_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),self.data_product)
        depth_map_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'depth_map')
        pointcloud_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'pointcloud')
        found_topics = self.active_topics
        for topic in found_topics:
            if image_ns == topic:
                self.status_msg.image_topic = image_ns
                #self.msg_if.pub_debug("Found depth map topic: " + str(topic) , throttle_s = 5)
            if depth_map_ns == topic:
                self.status_msg.depth_map_topic = depth_map_ns
            if pointcloud_ns == topic:
                self.status_msg.pointcloud_topic = pointcloud_ns

        # Same once-a-second config save check as DepthMapIF._updaterCb
        if self.save_config_needed == True and self.node_if is not None:
            self.save_config_needed = False
            self.node_if.save_config()
       
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)
        




    def _updateRangesM(self, min_m, max_m):
        if min_m < 0:
            min_m = 0
        if min_m <= max_m:
          self.min_range_m = min_m
          self.max_range_m = max_m
        else:
          self.msg_if.pub_debug("Invalid ranges supplied: " + str([min_m,max_m]) )



    def _aspectAdjust(self, cv2_img):
       
        ##########
        # Apply Aspect Controls
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1]
        img_height = cv2_shape[0]

        aspect_ratio_set = self.aspect_ratio_set
        if self.aspect_adjust_enabled == True and self.aspect_adjustment_disabled == False:
            try:
                cv2_img = nepi_img.adjust_aspect_ratio(cv2_img, aspect_ratio_set)
            except:
                pass
        cv2_shape_ar = cv2_img.shape
        img_width_ar = cv2_shape_ar[1]
        img_height_ar = cv2_shape_ar[0]
        self.aspect_ratio = (img_width_ar / img_height_ar)
        self.aspect_x_offset = int((img_width - img_width_ar)/2)
        self.aspect_y_offset = int((img_height - img_height_ar)/2)
        
        return cv2_img
        

    def _zoomAdjust(self,cv2_img):
        #####################
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1]
        img_height = cv2_shape[0]
        ratio = img_width / img_height

        #####################
        # Apply render controls 
        [xr_min,xr_max,yr_min,yr_max] = copy.deepcopy(self.controls_dict['window_ratios'])
        x_min = int(max(0, img_width * xr_min )) 
        x_max = int(min(img_width, img_width * xr_max))
        y_min = int(max(0, img_height * yr_min))
        y_max = int(min(img_height, img_height * yr_max))

        #self.msg_if.pub_debug("Got Image Window: " + str([x_min,x_max,y_min,y_max]) )
        cv2_img = cv2_img[y_min:y_max, x_min:x_max]

        self.x_offset = x_min 
        self.y_offset = y_min
        #self.msg_if.pub_debug("Image Render: " + str(cv2_img.shape) )



        ##########
        # Show Drag Box if Needed
        drag_window = copy.deepcopy(self.drag_window)

        #self.msg_if.pub_debug("Processing drag_window" + str(drag_window) )
        if drag_window is not None:
            #self.msg_if.pub_debug("Processing drag_window" + str(drag_window) )
            # Define the rectangle parameters
            x1 = min(drag_window[0], drag_window[1])
            x2 = max(drag_window[0], drag_window[1])
            y1 = min(drag_window[2], drag_window[3])
            y2 = max(drag_window[2], drag_window[3])


            color = (0, 200, 0) # Green color in BGR
            alpha = 0.4 # Transparency factor (0.0 for fully transparent, 1.0 for fully opaque)

            # Dorg a filled rectangle on the overlay copy
            cv2_img = nepi_img.overlay_rectangle(cv2_img, (x1, y1), (x2, y2), color = color, alpha = alpha)
        
        return cv2_img


    def _liveAdjust(self, cv2_img):
        """
        Translates an OpenCV image by a given number of ratio in x and y directions.
        
        Positive shift_x_ratio: moves image right
        Negative shift_x_ratio: moves image left
        Positive shift_y_ratio: moves image down
        Negative shift_y_ratio: moves image up
        """
        # Get adjustment in pixels)
        height, width = cv2_img.shape[:2]


        ########### Live Adjust
        ###### Rotation Update
        live_adjust_dict = copy.deepcopy(self.live_adjust_dict)
        live_adjust_enabled = live_adjust_dict['live_adjust_enabled']
        if live_adjust_enabled:
            live_adjust_rotate_deg = live_adjust_dict['live_adjust_rotate_deg']
            live_adjust_x_deg = live_adjust_dict['live_adjust_x_deg']
            live_adjust_y_deg = live_adjust_dict['live_adjust_y_deg']

            
            if abs(live_adjust_rotate_deg) > 0.01:
                #self.msg_if.pub_debug("Live Adjusting Rotate with degrees: " + str([live_adjust_rotate_deg]) , throttle_s = 5)
                cv2_img = nepi_img.rotate_degrees(cv2_img,live_adjust_rotate_deg)
        
            ###### Translation Update
            
            shift_x_pixels = -1 * math.floor((width * live_adjust_x_deg/self.width_deg))
            shift_y_pixels = -1 * math.floor((height * live_adjust_y_deg/self.height_deg))



            if abs(shift_x_pixels) > 0 or abs(shift_y_pixels) > 0:
                cv2_img = nepi_img.translate_pixels(cv2_img,shift_x_pixels,shift_y_pixels)
 
        
        return cv2_img


    def _stabilizeAdjust(self, cv2_img, navpose_dict):
        """
        Rotate an OpenCV image by a based on navpose data.
        """
        # Get adjustment in pixels)
        height, width = cv2_img.shape[:2]


        ########### Rotate Adjust
        stabilize_enabled = self.stabilize_enabled
        if stabilize_enabled == False:
            self.stabilize_rotate_deg = 0
        else:
            if navpose_dict.get('has_pan_tilt', False):
                stab_adjust_rotate_deg = -1 * navpose_dict.get('pan_tilt_roll_deg',0)
            else:
                stab_adjust_rotate_deg = -1 * navpose_dict.get('roll_deg',0)
            self.stabilize_rotate_deg = stab_adjust_rotate_deg
            if abs(stab_adjust_rotate_deg) > 0.01:
                #self.msg_if.pub_debug("Stabilize Adjusting Rotate with degrees: " + str([stab_adjust_rotate_deg]) , throttle_s = 5)
                cv2_img = nepi_img.rotate_degrees(cv2_img,stab_adjust_rotate_deg) 
        
        return cv2_img


    def _mouseEventCb(self,msg):
        #self.msg_if.pub_debug("Received mouse event message: " + str(msg) )

        width = self.status_msg.width_px
        height = self.status_msg.height_px


        pixel = [int(msg.click.x   + self.x_offset + self.aspect_x_offset), int(msg.click.y   + self.y_offset + self.aspect_y_offset)]
        color_bgr = (msg.click.b,msg.click.g,msg.click.r,msg.click.a)
        click_count = msg.click_count


        x_ratio = float(pixel[0] / max(1,self.width_org)) 
        y_ratio = float(pixel[1] / max(1,self.height_org))

        x_pixel_offset_adj = pixel[0]
        y_pixel_offset_adj = pixel[1]

        x_ratio_adj = copy.deepcopy(x_ratio)
        y_ratio_adj = copy.deepcopy(y_ratio)

        pixel_adj = copy.deepcopy(pixel)

        r_deg = 0
        x_deg = 0
        y_deg = 0
        if self.live_adjust_dict['live_adjust_enabled'] == True:
            r_deg = self.live_adjust_dict['live_adjust_rotate_deg']
            x_deg = self.live_adjust_dict['live_adjust_x_deg']
            y_deg = self.live_adjust_dict['live_adjust_y_deg']
        if self.stabilize_enabled == True:
            r_deg = r_deg + self.stabilize_rotate_deg
        if r_deg != 0 or x_deg != 0 or y_deg != 0:
            x_ratio_adj = x_ratio_adj + (width / 2) * ( x_deg / (self.width_deg / 2)) / width
            y_ratio_adj = y_ratio_adj + (height / 2) * (y_deg / (self.height_deg / 2)) / height
            deg_adj =  -1 * r_deg
            x_pixel_offset_adj =  (x_ratio_adj * width) - (width / 2) 
            y_pixel_offset_adj =  (y_ratio_adj * height) - (height / 2)
            [x_pixel_offset_adj,y_pixel_offset_adj] = nepi_utils.rotate_point(x_pixel_offset_adj,y_pixel_offset_adj,deg_adj)
            x_pixel_offset_adj = int(x_pixel_offset_adj)
            y_pixel_offset_adj = int(y_pixel_offset_adj)
            x_ratio_adj = (x_pixel_offset_adj + (width / 2)) / width
            y_ratio_adj= (y_pixel_offset_adj + (height / 2)) / height
            pixel_adj = [int(x_ratio_adj * width), int(y_ratio_adj * height)]
        pixel = pixel_adj        

        
        image_fov_horz = self.status_msg.width_deg
        image_fov_vert = self.status_msg.height_deg
        pixel_vert_angle_deg = 0
        pixel_horz_angle_deg = 0
        object_loc_x_ratio_from_center = 0
        object_loc_y_ratio_from_center = 0
        if width > 10 and height > 10 and image_fov_horz > 10 and image_fov_vert > 10:
            object_loc_x_ratio_from_center = float(pixel[0] - width/2) / float(width/2)
            object_loc_y_ratio_from_center = float(pixel[0] - height/2) / float(height/2)
            pixel_vert_angle_deg = (object_loc_y_ratio_from_center * float(image_fov_vert/2))
            pixel_horz_angle_deg = - (object_loc_x_ratio_from_center * float(image_fov_horz/2))
        angles = [pixel_horz_angle_deg,pixel_vert_angle_deg]

        
        # self.msg_if.pub_debug("Got mouse click pixel" )
        # self.msg_if.pub_debug("Click Pixels: " + str([msg.click.x,msg.click.y]) )
        # self.msg_if.pub_debug("Pixel Offsets: " + str([self.x_offset,self.y_offset]) )
        
        # self.msg_if.pub_debug("Orig H/W: " + str([self.width_org,self.height_org]) )
        # self.msg_if.pub_debug("Proc H/W: " + str([self.width_proc,self.height_proc]) )
        # self.msg_if.pub_debug("Pixel Ratios: " + str([x_ratio,y_ratio]) )
        # self.msg_if.pub_debug("Pixels Offset_Adj: " + str([x_pixel_offset_adj,y_pixel_offset_adj]) )
        # self.msg_if.pub_debug("Pixel Ratios Adj: " + str([x_ratio_adj,y_ratio_adj])




        if self.callback_dict['mouse_event_callback'] is not None:
            try:
                self.callback_dict['mouse_event_callback'](msg)
            except Exception as e:
                self.msg_if.pub_debug("Failed to call mouse mouse_event_callback: " + str(e) )

        if msg.click_event == True:
            
            if self.callback_dict['click_pixel_callback'] is not None:
                    try:
                        self.callback_dict['click_pixel_callback'](pixel,color_bgr,click_count,angles)
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to call mouse click_pixel_callback: " + str(e) )
            elif self.click_text_enabled == True and click_count == 1:
                            self.set_overlay_text_horz_ratio(x_ratio)
                            self.set_overlay_text_vert_ratio(y_ratio)
            elif self.click_crosshair_enabled == True and click_count == 1:
                            if self.zoom_ratio < 0.01:
                                click_color_rgb = self.overlays_dict['crosshairs_color_rgb']
                                click_name = 'click'
                                self.add_crosshair_ratios(x_ratio_adj,y_ratio_adj, color_rgb = click_color_rgb, name = click_name)
            elif self.click_target_enabled == True and click_count == 1:
                            if self.zoom_ratio < 0.01:
                                click_color_rgb = self.overlays_dict['targets_color_rgb']
                                click_name = 'click'
                                self.add_target_ratios(x_ratio_adj,y_ratio_adj, color_rgb = click_color_rgb, name = click_name)
            else:
                    if click_count == 1:
                        #self.msg_if.pub_debug("Single Click setting pixel value: " + str(pixel) )
                        self.set_pixel(pixel,color_bgr)
                    else:
                        #self.msg_if.pub_debug("Double Click resetting render controls" )
                        self.reset_renders()

        if msg.drag_event == True:
            self.last_click_time = None
            start_pixel = [int(msg.drag_start.x), int(msg.drag_start.y)]
            start_color_bgr = (msg.drag_start.b,msg.drag_start.g,msg.drag_start.r,msg.drag_start.a)
            stop_pixel = [int(msg.drag_stop.x), int(msg.drag_stop.y)]
            stop_color_bgr = (msg.drag_stop.b,msg.drag_stop.g,msg.drag_stop.r,msg.drag_stop.a)
            #self.msg_if.pub_debug("Using drag pixel: " + str(pixel) )
            if self.callback_dict['drag_callback'] is not None:
                try:
                    self.callback_dict['drag_callback'](start_pixel, start_color_bgr, stop_pixel, stop_color_bgr )
                except Exception as e:
                    self.msg_if.pub_debug("Failed to call mouse drag_callback: " + str(e) )
            else: #if self.zoom_ratio < 0.01:
                    self.drag_window = [start_pixel[0], stop_pixel[0],start_pixel[1], stop_pixel[1]]
                    self.msg_if.pub_debug("Drag Window Updated: " + str(self.drag_window) , throttle_s = 1)
                    self.needs_update()




        if msg.window_event == True:
            window = [int(msg.window.x_min   + self.x_offset) , 
                    int(msg.window.x_max   + self.x_offset), 
                    int(msg.window.y_min   + self.y_offset), 
                    int(msg.window.y_max  + self.y_offset)]
            if msg.window.x_min > msg.window.x_max:
                window[0] = msg.window.x_max  + self.x_offset
                window[1] = msg.window.x_min  + self.x_offset
            if msg.window.y_min > msg.window.y_max:
                window[2] = msg.window.y_max   + self.y_offset
                window[3] = msg.window.y_min   + self.y_offset

            if self.callback_dict['window_callback'] is not None:
                try:
                    self.callback_dict['window_callback'](window)
                except Exception as e:
                    self.msg_if.pub_debug("Failed to call mouse window_callback: " + str(e) )
            else:
                self.set_window(window)
                self.needs_update()
            self.drag_window = None

        if msg.scroll_event == True:
            scroll_pixel = [int(msg.scroll.x), int(msg.scroll.y)]
            scroll_color_bgr = (msg.scroll.b, msg.scroll.g, msg.scroll.r, msg.scroll.a)
            scroll_amount = msg.scroll_amount
            if self.callback_dict['scroll_callback'] is not None:
                try:
                    self.callback_dict['scroll_callback'](scroll_pixel, scroll_color_bgr, scroll_amount)
                except Exception as e:
                    self.msg_if.pub_debug("Failed to call mouse scroll_callback: " + str(e) )





    ########################
    # Filter Callbacks

    def _setFilterEnableCb(self, msg):
        self.msg_if.pub_debug("Received Enable Enhacement message: " + str(msg) )
        name = msg.name
        enabled = msg.value
        self.set_filter_enable(name,enabled) 

    def _setFilterRatioCb(self, msg):
        self.msg_if.pub_debug("Received Ehnacement Ratio update message: " + str(msg) )
        name = msg.name
        ratio = msg.value
        self.set_filter_ratio(name,ratio) 


    def _setAutoAdjustCb(self, msg):
        self.msg_if.pub_debug("Received Auto Adjust Enable update message: " + str(msg) )
        enabled = msg.data
        self.set_auto_adjust_enable(enabled)

    def _setAutoAdjustRatioCb(self, msg):
        self.msg_if.pub_debug("Received Auto Adjust Ratio update message: " + str(msg) )
        ratio = msg.data
        self.set_auto_adjust_ratio(ratio)

    def _setBrightnessCb(self, msg):
        self.msg_if.pub_debug("Received Brightness update message: " + str(msg) )
        ratio = msg.data
        self.set_brightness_ratio(ratio)


    def _setContrastCb(self, msg):
        self.msg_if.pub_debug("Received Contrast update message: " + str(msg) )
        ratio = msg.data
        self.set_contrast_ratio(ratio)
        


    def _setThresholdingCb(self, msg):
        self.msg_if.pub_debug("Received Threshold update message: " + str(msg) )
        ratio = msg.data
        self.set_threshold_ratio(ratio)



    ########################
    # Res and Orientation Callbacks

    def _setResolutionRatioCb(self, msg):
        self.msg_if.pub_debug("Received Resolution update message: " + str(msg) )
        ratio = msg.data
        self.set_resolution_ratio(ratio)


    def _setRotate2dCb(self, msg):
        self.msg_if.pub_debug("Received Rotate 2d Deg update message: " + str(msg) )
        # 90-deg button path: resize-box mode (output box grows to fit).
        self.controls_dict['rotate_2d_keep_size'] = False
        cur_angle = self.controls_dict['rotate_2d_deg']
        new_angle = cur_angle + 90
        if new_angle >= 360:
            new_angle = 0
        new_angle = int(round(new_angle/90.0,0) * 90)
        self.set_rotate_2d_deg(new_angle)

    def _setRotate2dDegCb(self, msg):
        self.msg_if.pub_debug("Received Set Rotate 2d Deg message: " + str(msg) )
        # Free Cam path: fixed-box mode (box holds shape, black fill).
        self.controls_dict['rotate_2d_keep_size'] = True
        new_angle = int(msg.data) % 360
        self.set_rotate_2d_deg(new_angle)

    def _setRotate2dSwapBoxCb(self, msg):
        self.msg_if.pub_debug("Received Set Rotate 2d Swap Box message: " + str(msg) )
        enable = msg.data
        self.set_rotate_2d_swap_box(enable)

    def _setFlipHorzCb(self, msg):
        self.msg_if.pub_debug("Received Flip Horz update message: " + str(msg) )
        enable = msg.data
        self.set_flip_horz(enable)

    def _setFlipVertCb(self, msg):
        self.msg_if.pub_debug("Received Flip Vert update message: " + str(msg) )
        enable = msg.data
        self.set_flip_vert(enable)

    ########################
    # Render Callbacks

    def _setRangeCb(self, msg):
        self.msg_if.pub_debug("Received Range update message: " + str(msg) )
        start_ratio = msg.start_range
        stop_ratio = msg.stop_range
        self.set_range_ratios(start_ratio,stop_ratio)
      

    def _setZoomCb(self, msg):
        self.msg_if.pub_debug("Received Zoom update message: " + str(msg) )
        ratio = msg.data
        self.set_zoom_ratio(ratio)

    def _setPanXCb(self, msg):
        self.msg_if.pub_debug("Received Pan Left Right update message: " + str(msg) )
        ratio = msg.data
        self.set_x_ratio(ratio)


    def _setPanYCb(self, msg):
        self.msg_if.pub_debug("Received Pan Up Down update message: " + str(msg) )
        ratio = msg.data
        self.set_y_ratio(ratio)


    def _setZoom3DCb(self, msg):
        self.msg_if.pub_debug("Received Zoom 3D update message: " + str(msg) )
        ratio = msg.data
        self.set_zoom_3d_ratio(ratio) 

    def _setRotate3DCb(self, msg):
        self.msg_if.pub_debug("Received Rotate 3D update message: " + str(msg) )
        ratio = msg.data
        self.set_rotate_3d_ratio(ratio) 

    def _setTilt3DCb(self, msg):
        ratio = msg.data
        self.set_tilt_3d_ratio(ratio)

    def render3dControlsCb(self, msg):
        self.msg_if.pub_debug("Received Render 3D Controls Enable message: " + str(msg) )
        enabled = msg.data
        self.render_3d_controls_enabled = enabled

        if enabled == True:
            self.msg_if.pub_debug("Enabling 3D render controls (mouse drag/window)" )
            if hasattr(self, 'render3dDragHandler'):
                self.set_image_callback('drag_callback', self.render3dDragHandler)
            if hasattr(self, 'render3dWindowHandler'):
                self.set_image_callback('window_callback', self.render3dWindowHandler)
            if hasattr(self, 'render3dScrollHandler'):
                self.set_image_callback('scroll_callback', self.render3dScrollHandler)
            if hasattr(self, 'render3dClickHandler'):
                self.set_image_callback('click_pixel_callback', self.render3dClickHandler)
        else:
            self.msg_if.pub_debug("Disabling 3D render controls (mouse drag/window)" )
            self.set_image_callback('drag_callback', None)
            self.set_image_callback('window_callback', None)
            self.set_image_callback('scroll_callback', None)
            self.set_image_callback('click_pixel_callback', None)
        self.publish_status()



    def setCamFovCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        new_val = msg.data
        if new_val > 100:
            new_val = 100
        if new_val < 30:
            new_val = 30
        self.controls_dict['cam_fov'] = new_val
        self.publish_status()
        self.node_if.set_param(self.node_if_prefix +  'cam_fov',new_val)

    def setCamViewCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        new_array = []
        new_array.append(msg.x)
        new_array.append(msg.y)
        new_array.append(msg.z)
        self.controls_dict['cam_view'] = new_array
        self.publish_status()
        self.node_if.set_param(self.node_if_prefix +  'cam_view',new_array)

    def setCamPositionCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        new_array = []
        new_array.append(msg.x)
        new_array.append(msg.y)
        new_array.append(msg.z)
        self.controls_dict['cam_pos'] = new_array
        self.publish_status()
        self.node_if.set_param(self.node_if_prefix +  'cam_pos',new_array)


    def setCamRotationCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        new_array = []
        new_array.append(msg.x)
        new_array.append(msg.y)
        new_array.append(msg.z)
        self.controls_dict['cam_rot'] = new_array
        self.publish_status()
        self.node_if.set_param(self.node_if_prefix +  'cam_rot',new_array)

    def setRangeRatiosCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        min_ratio = msg.start_range
        max_ratio = msg.stop_range
        if min_ratio < max_ratio and min_ratio >= 0 and max_ratio <= 1:
            # controls_dict is what publish_status reports and what the renderer
            # resolves from; a param write alone left the slider snapping back to the
            # old value in the status published on the next line, and the image never
            # re-ranged. Same shape as the setWhiteBgCb note below.
            self.controls_dict['start_range_ratio'] = min_ratio
            self.controls_dict['stop_range_ratio'] = max_ratio
            self.node_if.set_param(self.node_if_prefix +  'start_range_ratio', min_ratio)
            self.node_if.set_param(self.node_if_prefix +  'stop_range_ratio', max_ratio)
            self.publish_status()
            self.needs_update()

    def setWhiteBgCb(self,msg):
        enable = msg.data
        # controls_dict is what the renderer resolves from; a param write alone
        # left this toggle inert until the next node restart.
        self.controls_dict['use_wbg'] = enable
        self.node_if.set_param(self.node_if_prefix +  'use_wbg', enable)
        self.publish_status()
        self.needs_update()

    def resetRender3dControlsCb(self, msg):
        self.msg_if.pub_debug("Got reset 3D render orientation msg" )
        self.live_adjust_dict['live_adjust_rotate_deg'] = 0
        if hasattr(self, 'reset_render_3d_orientation'):
            self.reset_render_3d_orientation()

    def resetRender3dPositionCb(self, msg):
        self.msg_if.pub_debug("Got reset 3D render position msg" )
        self.live_adjust_dict['live_adjust_x_deg'] = 0
        self.live_adjust_dict['live_adjust_y_deg'] = 0
        if hasattr(self, 'reset_render_3d_position'):
            self.reset_render_3d_position()



    ########################
    # Overlay Callbacks
    def _setrOverlayTextEnableCb(self,msg):
        enabled = msg.data
        self.set_overlay_text_enable(enabled)

    def _setOverlaySizeCb(self,msg):
        ratio = msg.data
        self.set_overlay_text_size_ratio(ratio)


    def _setOverlayVertCb(self,msg):
        ratio = msg.data
        self.set_overlay_text_vert_ratio(ratio)


    def _setOverlayHorzCb(self,msg):
        ratio = msg.data
        self.set_overlay_text_horz_ratio(ratio)

    def _setOverlayTransparencyCb(self,msg):
        ratio = msg.data
        self.set_overlay_text_transparency_ratio(ratio)


    def _setOverlayColorRGBCb(self,msg):
        r = msg.r
        g = msg.g
        b = msg.b
        self.set_overlay_text_color_rgb(r,g,b)

    def _setOverlayImgNameCb(self,msg):
        enabled = msg.data
        self.set_overlay_text_image_name(enabled)

    def _setOverlayDateTimeCb(self,msg):
        enabled = msg.data
        self.set_overlay_text_date_time(enabled)

    def _setOverlayNavCb(self,msg):
        enabled = msg.data
        self.set_overlay_text_nav(enabled)

    def _setOverlayPoseCb(self,msg):
        enabled = msg.data
        self.set_overlay_text_pose(enabled)

    def _setOverlayListCb(self,msg):
        overlay_text_list = msg.array
        self.set_overlay_text_list(overlay_text_list)


    def _setOverlayTextCb(self,msg):
        overlay_text = msg.data
        self.set_overlay_text(overlay_text)


    def _clearOverlayListCb(self,msg):
        self.clear_overlay_text_list()

    ##############################

    def _clickTextEnableCb(self,msg):
        enabled = msg.data
        self.set_click_text(enabled)

    def _clickCrosshairEnableCb(self,msg):
        self.msg_if.pub_debug("Got Click Crosshair Enable msg" )
        enbled = msg.data
        self.set_click_crosshair(enbled)


    def _setCrosshairsSizeRatioCb(self,msg):
        ratio = msg.data
        self.set_crosshairs_size_ratio(ratio)

    def _setCrosshairsThicknessRatioCb(self,msg):
        ratio = msg.data
        self.set_crosshairs_thickness_ratio(ratio)

    def _setCrosshairsTextRatioCb(self,msg):
        ratio = msg.data
        self.set_crosshairs_text_ratio(ratio)

    def _setCrosshairsTransparencyRatioCb(self,msg):
        ratio = msg.data
        self.set_crosshairs_transparency_ratio(ratio)

    def _setCrosshairsColorRGBCb(self,msg):
        r = msg.r
        g = msg.g
        b = msg.b
        self.set_crosshairs_color_rgb(r,g,b)

    def _setrOverlayCrosshairsCb(self,msg):
        enabled = msg.data
        self.set_crosshairs_enable(enabled)


    def _setrOverlayCrosshairNamesCb(self,msg):
        enabled = msg.data
        self.set_overlay_crosshair_names(enabled)

    def _setrOverlayCrosshairPixelsCb(self,msg):
        enabled = msg.data
        self.set_overlay_crosshair_pixels(enabled)
        

    def _setrOverlayCrosshairDegreesCb(self,msg):
        enabled = msg.data
        self.set_overlay_crosshair_degrees(enabled)

    def _setrOverlayCrosshairMessagesCb(self,msg):
        enabled = msg.data
        self.set_overlay_crosshair_text(enabled)


    def _addCrosshairPixelCb(self,msg):
        name = msg.name
        if name != '':
            name = 'Crosshair'
        x_px = msg.x_pixel
        x_ratio = nepi_utils.check_ratio(x_px/self.width_org)
        y_px = msg.y_pixel
        y_ratio = nepi_utils.check_ratio(y_px/self.height_org)
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_crosshair_enabled = False
        self.add_crosshair_ratios(x_ratio, y_ratio, name = name, color_rgb = (r,g,b), msg_str = msg_str)

    def _addCrosshairRatiosCb(self,msg):
        name = msg.name
        if name != '':
            name = 'Crosshair'
        x_ratio = msg.x_ratio
        y_ratio = msg.y_ratio
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_crosshair_enabled = False
        self.add_crosshair_ratios(x_ratio, y_ratio, name = name, color_rgb = (r,g,b), msg_str = msg_str)


    def _addCrosshairDegreesCb(self,msg):
        self.msg_if.pub_debug("Got Add Crosshair Msg: " + str(msg) )
        name = msg.name
        if name != '':
            name = 'Crosshair'
        x_deg = msg.x_offset_deg
        y_deg = msg.y_offset_deg
       
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_crosshair_enabled = False
        self.add_crosshair_degs(x_deg, y_deg, name = name, color_rgb = (r,g,b), msg_str = msg_str)

    def _removeCrosshairCb(self,msg):
        name = msg.data
        #self.click_crosshair_enabled = False
        self.remove_crosshair(name)


    def _clearCrosshairsCb(self,msg):
        self.click_crosshair_enabled = False
        self.clear_crosshairs()

    ########################################


    def _clickTargetEnableCb(self,msg):
        enbled = msg.data
        self.set_click_target(enbled)


    def _setTargetsSizeRatioCb(self,msg):
        ratio = msg.data
        self.set_targets_size_ratio(ratio)

    def _setTargetsThicknessRatioCb(self,msg):
        ratio = msg.data
        self.set_targets_thickness_ratio(ratio)

    def _setTargetsTextRatioCb(self,msg):
        ratio = msg.data
        self.set_targets_text_ratio(ratio)

    def _setTargetsTransparencyRatioCb(self,msg):
        ratio = msg.data
        self.set_targets_transparency_ratio(ratio)

    def _setTargetsColorRGBCb(self,msg):
        r = msg.r
        g = msg.g
        b = msg.b
        self.set_targets_color_rgb(r,g,b)

    def _setrOverlayTargetsCb(self,msg):
        enabled = msg.data
        self.set_targets_enable(enabled)


    def _setrOverlayTargetNamesCb(self,msg):
        enabled = msg.data
        self.set_overlay_target_names(enabled)

    def _setrOverlayTargetPixelsCb(self,msg):
        enabled = msg.data
        self.set_overlay_target_pixels(enabled)
        

    def _setrOverlayTargetDegreesCb(self,msg):
        enabled = msg.data
        self.set_overlay_target_degrees(enabled)

    def _setrOverlayTargetMessagesCb(self,msg):
        enabled = msg.data
        self.set_overlay_target_text(enabled)


    def _addTargetPixelCb(self,msg):
        name = msg.name
        if name != '':
            name = 'Target'
        x_px = msg.x_pixel
        x_ratio = nepi_utils.check_ratio(x_px/self.width_org)
        y_px = msg.y_pixel
        y_ratio = nepi_utils.check_ratio(y_px/self.height_org)
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_target_enabled = False
        self.add_target_ratios(x_ratio, y_ratio, name = name, color_rgb = (r,g,b), msg_str = msg_str)

    def _addTargetRatiosCb(self,msg):
        name = msg.name
        if name != '':
            name = 'Target'
        x_ratio = msg.x_ratio
        y_ratio = msg.y_ratio
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_target_enabled = False
        self.add_target_ratios(x_ratio, y_ratio, name = name, color_rgb = (r,g,b), msg_str = msg_str)


    def _addTargetDegreesCb(self,msg):
        self.msg_if.pub_debug("Got Add Target Msg: " + str(msg) )
        name = msg.name
        if name != '':
            name = 'Target'
        x_deg = msg.x_offset_deg
        y_deg = msg.y_offset_deg
       
        r = msg.r
        g = msg.g
        b = msg.b
        msg_str = msg.msg_str
        #self.click_target_enabled = False
        self.add_target_degs(x_deg, y_deg, name = name, color_rgb = (r,g,b), msg_str = msg_str)

    def _removeTargetCb(self,msg):
        name = msg.data
        #self.click_target_enabled = False
        self.remove_target(name)


    def _clearTargetsCb(self,msg):
        self.click_target_enabled = False
        self.clear_targets()

    ########################################
    def _setAspectAdjustEnableCb(self,msg):
        enabled = msg.data
        self.set_aspect_adjust_enable(enabled)

    def _setAspectAdjustRatioCb(self,msg):
        ratio = msg.data
        self.set_aspect_adjust_ratio(ratio)

    def _setAspectAdjustByRatioCb(self,msg):
        ratio = msg.data
        self.set_aspect_adjust_by_ratio(ratio)


    ########################################
    def _setStreamCompressionEnableCb(self,msg):
        enabled = msg.data
        self.set_stream_compression_enable(enabled)

    def _setStreamCompressionRatioCb(self,msg):
        ratio = msg.data
        self.set_stream_compression_ratio(ratio)


    ########################################
    def _setLiveAdjustEnableCb(self,msg):
        enabled = msg.data
        self.set_live_adjust_enable(enabled)

    def _setLiveAdjustRotateRatioCb(self,msg):
        ratio = msg.data
        self.set_live_adjust_rotate_ratio(ratio)

    def _setLiveAdjustRotateDegCb(self,msg):
        deg = msg.data
        self.set_live_adjust_rotate_deg(deg)

    def _setLiveAdjustTranXRatioCb(self,msg):
        ratio = msg.data
        self.set_live_adjust_x_ratio(ratio)

    def _setLiveAdjustTranXPixelCb(self,msg):
        pixel = msg.data
        self.set_live_adjust_x_pixel(pixel)

    def _setLiveAdjustTranXDegCb(self,msg):
        deg = msg.data
        self.set_live_adjust_x_deg(deg)

    def _setLiveAdjustTranYRatioCb(self,msg):
        ratio = msg.data
        self.set_live_adjust_y_ratio(ratio)

    def _setLiveAdjustTranYPixelCb(self,msg):
        pixel = msg.data
        self.set_live_adjust_y_pixel(pixel)

    def _setLiveAdjustTranYDegCb(self,msg):
        deg = msg.data
        self.set_live_adjust_y_deg(deg)

    def _resetControlsCb(self,msg):
        self.reset_filters()
        self.reset_overlays()
        self.reset_settings()
        self.reset_renders()

    def _resetFiltersCb(self,msg):
        self.reset_filters()

    def _resetOverlaysCb(self,msg):
        self.reset_overlays()

    def _resetSettingsCb(self,msg):
        self.reset_settings()
    
    def _resetRendersCb(self,msg):
        self.msg_if.pub_debug("Received reset renders message" )
        self.reset_renders()


    ########################################
    def _setStabilizeEnableCb(self,msg):
        enabled = msg.data
        self.set_stabilize_enable(enabled)

##################################################
# ColorImageIF

class ColorImageIF(BaseImageIF):

    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = ImageStatus()
    DATA_MSG = Image()

    ID = 'color_image'
    DISPLAY_NAME = 'Color Image'
    DESCRIPTION = 'Color Image Data Product'

    DATA_PRODUCT = 'color_image'

    #########################
    ## CUSTOM
    #########################

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = True,
        has_auto_adjust = True,
        has_contrast = True,
        has_brightness = True,
        has_threshold = True,
        has_rotate_2d = True,
        has_flip_horz = True,
        has_flip_vert = True,
        has_range = False,
        has_zoom = True,
        has_pan = True,
        has_window = True,
        has_zoom_3d = False,
        has_rotate_3d = False,
        has_tilt_3d = False,
        has_camera_3d = False
        )

    DEFAULT_FILTERS_DICT = dict(
        # Low_Light = {
        #     'enabled': False,
        #     'function': nepi_img.low_light_filter,
        #     'ratio': 0.5
        # }
    )

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5,
        )


    auto_adjust_controls = ['brightness','contrast','threshold']


    
    def __init__(self, 
                name = 'color_image',
                namespace = None ,
                data_product = None,
                data_source_description = None,
                data_ref_description = None,
                perspective = 'pov',
                init_overlay_text_list = [],
                options_dict = dict(),
                callback_dict = dict(),
                live_adjustments_disabled = False,
                aspect_adjustment_disabled = False,
                info_dict = dict(),
                msg_if = None,
                node_if = None,
                save_data_if = None,
                transform_if = None,
                navpose_if = None,
                ):


        # Call the parent class constructor
        super().__init__(
                name,
                namespace ,
                data_product,
                data_source_description,
                data_ref_description,
                perspective,
                self.DEFAULT_CAPS_DICT,
                self.DEFAULT_CONTROLS_DICT,
                self.DEFAULT_FILTERS_DICT,
                options_dict,
                callback_dict,
                init_overlay_text_list,
                live_adjustments_disabled,
                aspect_adjustment_disabled,
                info_dict,
                msg_if,
                node_if,
                save_data_if,
                transform_if,
                navpose_if,
                )

        ###############################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is ColorImageIF:
            self.start()

    ###############################
    # Class Public Methods
    ###############################


    def process_cv2_img(self, cv2_img):
        """Apply the full color image processing pipeline to an OpenCV image.

        Applies, in order: resolution scaling, 2-D rotation and flips, crop
        window, drag-selection overlay, user-defined filters, and then either
        manual brightness/contrast/sharpness adjustments or auto-adjustment.

        Args:
            cv2_img (numpy.ndarray): Input BGR OpenCV image array.

        Returns:
            numpy.ndarray: The fully-processed BGR image.
        """


        # if res_ratio < 0.9:
        #     [cv2_img,new_res] = nepi_img.adjust_resolution_ratio(cv2_img, res_ratio)


        ##########
        # Apply Resolution Controls
        res_ratio = self.controls_dict['resolution_ratio']
        # cv2_shape = cv2_img.shape
        # img_width1 = cv2_shape[1]
        # img_height1 = cv2_shape[0]
        if res_ratio < 0.9:
            [cv2_img,new_res] = nepi_img.adjust_resolution_ratio(cv2_img, res_ratio)



        ##########
        # Apply Oreantation Controls
        degrees = self.controls_dict['rotate_2d_deg']
        keep_size = self.controls_dict['rotate_2d_keep_size']
        swap_box = self.controls_dict['rotate_2d_swap_box']
        fliph = self.controls_dict['flip_horz']
        flipv = self.controls_dict['flip_vert']

        # Also rotate at angle 0 when the Free Cam box is swapped, so the upright
        # image is re-canvased (centered, black-filled) into the swapped box.
        if degrees != 0 or (keep_size == True and swap_box == True):
           cv2_img = nepi_img.rotate_degrees(cv2_img, degrees) #, keep_size=keep_size, swap_dims=swap_box)

        if fliph == True:
            cv2_img = nepi_img.flip_horz(cv2_img)

        if flipv == True:
            cv2_img = nepi_img.flip_vert(cv2_img)






        ###################
        # Apply Filters
        if self.filter_dict is not None:
            for filter_name in self.filter_dict.keys():
                enabled = self.filter_dict[filter_name]['enabled']
                if enabled == True:
                    ratio = self.filter_dict[filter_name]['ratio']
                    if ratio > 0.05:
                        function = self.filter_dict[filter_name]['function']
                        cv2_img = function(cv2_img,ratio)


        ##########
        # Apply Adjustment Controls
        auto = self.controls_dict['auto_adjust_enabled']
        auto_ratio = self.controls_dict['auto_adjust_ratio']
        brightness = self.controls_dict['contrast_ratio']
        contrast = self.controls_dict['brightness_ratio']
        threshold = self.controls_dict['threshold_ratio']

        if auto is False:
            cv2_img = nepi_img.adjust_brightness(cv2_img, brightness)
            cv2_img = nepi_img.adjust_contrast(cv2_img, contrast)
            cv2_img = nepi_img.adjust_sharpness(cv2_img, threshold)
        else:
            cv2_img = nepi_img.adjust_auto(cv2_img,auto_ratio)

        #self.msg_if.pub_debug("Image Filter: " + str(cv2_img.shape) )


        return cv2_img
        




    ###############################
    # Class Private Methods
    ###############################


