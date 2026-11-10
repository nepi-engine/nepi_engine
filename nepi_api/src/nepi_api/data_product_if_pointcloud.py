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
from nepi_api.data_product_if_image import BaseImageIF

from nepi_api.utils_if import ControlsIF


SYSTEM_ALL_TOPIC = 'all'





##################################################
# PointcloudIF
##################################################


class PointcloudIF(DataProductIF):


    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = PointcloudStatus()
    DATA_MSG = PointCloud2()

    ID = 'pointcloud'
    DISPLAY_NAME = 'Pointcloud'
    DESCRIPTION = 'Pointcloud Data Product'

    DATA_PRODUCT = 'pointcloud'

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
    # options_if = dict()

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
    #caps_report = ImageCapabilitiesQueryResponse()

    image_topic = ""
    depth_map_topic = ""
    pointcloud_topic = ""

    ready = False

    DEFAULT_WIDTH_DEG = 100
    DEFAULT_HEIGHT_DEG = 70

    Factory_Image_Width = 955
    Factory_Image_Height = 600
    Factory_Zoom_Ratio = .5
    Factory_Rotate_3D_Ratio = .5
    Factory_Tilt_3D_Ratio = .5
    Factory_Cam_FOV = 60
    Factory_Cam_View = [3, 0, 0]
    Factory_Cam_Pos = [-5, 0, 0]
    Factory_Cam_Rot = [0, 0, 1]

    #Default Control Values (single merged dict holding all process + render controls)
    DEFAULT_CONTROLS_DICT = dict(
        clip_enabled = True,
        clip_selection = 'Range',
        clip_min_range_m = -20,
        clip_max_range_m = 20,
        voxel_downsample_size = 0.0, # Zero value skips process
        uniform_downsample_k_points = 0, # Zero value skips process
        outlier_removal_num_neighbors = 0, # Zero value skips process
        outlier_removal_std_ratio = 1.0 # Only applied when outlier_removal_num_neighbors > 0
        )

    DEFAULT_CALLBACK_DICT = dict(
        options_updated_callback = None
    )

    api_lib_folder = '/opt/nepi/nepi_engine/lib/nepi_api'

    bounding_box3d_topic = "NONE"
    bounding_box3d_sub = None

    launch_node_process = None
    pub_img_node_name = ""
    pub_img_namepace = ""

    needs_data_last_logged = None

    min_range_m = 0
    max_range_m = 100

    perspective = 'pov'

    voxel = [0,0,0]


    def __init__(self, 
                name = 'pointcloud',
                namespace = None,
                data_product = None,
                data_source_description = None,
                data_ref_description = None,
                perspective = 'pov',
                pub_image = True,
                init_overlay_text_list = [],
                options_dict = dict(),
                callback_dict = dict(),
                info_dict = dict(),
                msg_if = None,
                node_if = None,
                save_data_if = None,
                transform_if = None,
                navpose_if = None,
                ):




   
        ##############################
        # Update Class Variables
        ##############################

        name = nepi_utils.get_clean_name(name)
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product.strip() != '':
                name = data_product


    
        self.init_overlay_text_list = init_overlay_text_list
        self.perspective = perspective

        self.clip_options = ['Range', 'BoundingBox']
        self.frame3d_list = nepi_nav.NAVPOSE_3D_FRAME_OPTIONS

        ##############################  
        # Setup NodeIF Updates
        ##############################  

        ##################
        update_info_dict = info_dict
        ##################        
        update_caps_dict = dict()
        # try:
        #     update_caps_dict.udpate(caps_dict)
        # except:
        #     pass
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
       
        # Params Config Dict ####################
        update_params_dict = {

            'render_enable': {
                'name': 'render_enable',
                'namespace': self.namespace,
                'factory_val': True
            },
            'use_wbg': {
                'name': 'use_wbg',
                'namespace': self.namespace,
                'factory_val': False
            },
            # setRotateRatioCb/setTiltRatioCb are wired to the live set_rotate_ratio
            # and set_tilt_ratio topics below and PointcloudStatus declares matching
            # fields, but neither key was registered: the writes were dropped, the
            # factory_reset_param calls in resetRenderControls were no-ops, and the
            # status fields never left 0.0.
            'rotate_ratio': {
                'name': 'rotate_ratio',
                'namespace': self.namespace,
                'factory_val': 0.5
            },
            'tilt_ratio': {
                'name': 'tilt_ratio',
                'namespace': self.namespace,
                'factory_val': 0.5
            },
            'frame_3d': {
                'name': 'frame_3d',
                'namespace': self.namespace,
                'factory_val': self.frame3d_list[0]
            },
            'clip_enabled': {
                'name': 'clip_enabled',
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['clip_enabled']
            },
            'clip_selection': {
                'name': 'clip_selection',
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['clip_selection']
            },
            'range_min_m': {
                'name': 'range_min_m',
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['clip_min_range_m']
            },
            'range_max_m': {
                'name': 'range_max_m',
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['clip_max_range_m']
            },
            'voxel_downsample_size': {
                'name': 'voxel_downsample_size',
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['voxel_downsample_size']
            },
            'uniform_downsample_k_points': {
                'name': 'uniform_downsample_k_points',
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['uniform_downsample_k_points']
            },
            'outlier_removal_num_neighbors': {
                'name': 'outlier_removal_num_neighbors',
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['outlier_removal_num_neighbors']
            },
            'outlier_removal_std_ratio': {
                'name': 'outlier_removal_std_ratio',
                'namespace': self.namespace,
                'factory_val': self.DEFAULT_CONTROLS_DICT['outlier_removal_std_ratio']
            }
        }

        self.add_params(update_params_dict)

        #######################
        # Add Services
        #######################
        services_dict = {}
        self.add_services(services_dict)

        #######################
        # Add Pubs 
        #######################
        # Pubs Config Dict ####################
        update_pubs_dict = {
            'data_pub': {
                'msg': PointCloud2,
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
        # Subs Config Dict ###########
        update_subs_dict = {
            'reset_controls': {
                'namespace': self.namespace,
                'topic': 'reset_controls',
                'msg': Empty,
                'qsize': 10,
                'callback': self.resetProcessControlsCb,
                'callback_args': ()
            },
            'set_clip_enable': {
                'namespace': self.namespace,
                'topic': 'set_clip_enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.clipEnableCb,
                'callback_args': ()
            },
            'clip_selection': {
                'namespace': self.namespace,
                'topic': 'set_clip_selection',
                'msg': String,
                'qsize': 10,
                'callback': self.setClipSelectionCb,
                'callback_args': ()
            },
            'range_clip_m': {
                'namespace': self.namespace,
                'topic': 'set_range_clip_m',
                'msg': RangeWindow,
                'qsize': 10,
                'callback': self.setRangeMetersCb,
                'callback_args': ()
            },
            'clip_bounding_box3d_topic': {
                'namespace': self.namespace,
                'topic': 'set_clip_bounding_box3d_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.setClipBoxTopicCb,
                'callback_args': ()
            },
            'voxel_downsample_size': {
                'namespace': self.namespace,
                'topic': 'set_voxel_downsample_size',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setVoxelSizeCb,
                'callback_args': ()
            },
            'downsample_k_points': {
                'namespace': self.namespace,
                'topic': 'uniform_downsample_k_points',
                'msg': Int32,
                'qsize': 10,
                'callback': self.setUniformPointsCb,
                'callback_args': ()
            },
            'outlier_removal': {
                'namespace': self.namespace,
                'topic': 'outlier_removal_num_neighbors',
                'msg': Int32,
                'qsize': 10,
                'callback': self.setOutlierNumCb,
                'callback_args': ()
            },
            'set_rotate_ratio': {
                'namespace': self.namespace,
                'topic': 'set_rotate_ratio',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setRotateRatioCb,
                'callback_args': ()
            },
            'set_tilt_ratio': {
                'namespace': self.namespace,
                'topic': 'set_tilt_ratio',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setTiltRatioCb,
                'callback_args': ()
            },

            'set_render_enable': {
                'namespace': self.namespace,
                'topic': 'set_render_enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setRenderEnableCb,
                'callback_args': ()
            },
        }


        self.add_subs(update_subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################


        #######################
        # Update Class Variables
        #######################


        ########################
        # Update Status Msg
        ########################


        self.status_msg.source_description = self.data_source_description
        self.status_msg.end_description = self.data_ref_description


        self.status_msg.publishing = False
        self.status_msg.has_rgb = False
        self.status_msg.has_intensity = False
        self.status_msg.point_count = 0
        self.status_msg.get_latency_time = 0.0
        self.status_msg.pub_latency_time = 0.0
        self.status_msg.process_time = 0.0
        self.status_msg.img_pub_enabled = pub_image

        ##############################
        # Start Node Processes
        ##############################



        ##############################
        # Create Options IF if needed
        if options_dict is not None:
            # Under this IF's own namespace, as BaseImageIF does: with none, every
            # data product in the node claims <node>/options on the shared node_if.
            self.options_if = ControlsIF( controls_name = 'options',
                namespace = self.namespace,
                controls_display_name = 'Options',
                controls_description = 'Custom Options',
                controls_init_dict = options_dict,
                controls_updated_callback = self.callback_dict['options_updated_callback'], 
                save_params = True,
                msg_if = self.msg_if,
                node_if = self.node_if
            )





        ####################
        # Setup Save Data IF Class 
        if save_data_if is not None and save_data_if != 'None':
            self.save_data_if = save_data_if
            data_products = self.save_data_if.get_data_products()
            if self.data_product not in data_products:
                self.save_data_if.register_data_product(self.data_product)
            if 'pointcloud_image' not in data_products and pub_image == True:
                self.save_data_if.register_data_product('pointcloud_image')



        ##############################
        # Launch Image Pub Node
      

        self.pub_image = pub_image
        if pub_image == True:
            self.msg_if.pub_debug("Launching Image Pub Node")
            self.launch_image_pub_node()

        ##############################
        # Start Node Processes

        self.msg_if.pub_debug("Staring updater process" )
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is PointcloudIF:
            self.start()


    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):

        status_msg.navpose_msg = self.get_navpose_msg()
        status_msg.clip_enabled = self.node_if.get_param(self.node_if_prefix +  'clip_enabled')
        status_msg.clip_options = self.clip_options
        status_msg.clip_selection = self.node_if.get_param(self.node_if_prefix +  'clip_selection')

        clip_meters = RangeWindow()
        clip_meters.start_range =   float(self.node_if.get_param(self.node_if_prefix +  'range_min_m'))
        clip_meters.stop_range =   float(self.node_if.get_param(self.node_if_prefix +  'range_max_m'))
        status_msg.clip_meters = clip_meters

        status_msg.clip_target_topic = self.bounding_box3d_topic


        status_msg.voxel_downsample_size_m = self.node_if.get_param(self.node_if_prefix +  'voxel_downsample_size')
        status_msg.uniform_downsample_points = self.node_if.get_param(self.node_if_prefix +  'uniform_downsample_k_points')
        status_msg.outlier_k_points = self.node_if.get_param(self.node_if_prefix +  'outlier_removal_num_neighbors')

        status_msg.render_enable = self.node_if.get_param(self.node_if_prefix +  'render_enable')

        # Degrees use the same conversion the renderer applies to its own
        # rotate_3d_ratio/tilt_3d_ratio, so the two agree on what 0.5 means.
        rotate_ratio = self.node_if.get_param(self.node_if_prefix +  'rotate_ratio')
        status_msg.rotate_ratio = rotate_ratio
        status_msg.rotate_deg = (0.5 - rotate_ratio) * 2 * 180
        tilt_ratio = self.node_if.get_param(self.node_if_prefix +  'tilt_ratio')
        status_msg.tilt_ratio = tilt_ratio
        status_msg.tilt_deg = (0.5 - tilt_ratio) * 2 * 180

        status_msg.range_min_max_m.start_range = self.min_range_m
        status_msg.range_min_max_m.stop_range = self.max_range_m

        avg_rate = 0
        if len(self.time_list) > 0:
            avg_time = sum(self.time_list) / len(self.time_list)
            if avg_time > .01:
                avg_rate = float(1) / avg_time
        status_msg.avg_pub_rate = avg_rate

        return status_msg


    
    ###############################
    # Class Setup Methods
    ###############################

    def launch_image_pub_node(self):
        """Launch the pointcloud image publisher node as a subprocess.

        Resolves the image publisher node file path and starts the node with
        the correct namespace and parameters if the file exists and image
        publishing is enabled. Does nothing if the node is already running or
        the node file cannot be found.
        """

        self.msg_if.pub_debug("Waiting for system folders")
        system_folders = nepi_system.get_system_folders()
        while system_folders is None and nepi_sdk.is_shutdown() == False:
            nepi_sdk.sleep(1)

        self.msg_if.pub_debug("Got system folders: " + str(system_folders))

        if system_folders is not None:
            self.api_lib_folder = system_folders['api_lib']
        self.msg_if.pub_debug("Using SDK Share Folder: " + str(self.api_lib_folder))


        #######################################
        node_name = self.node_name + "_img_pub"
        launch_namespace = os.path.dirname(self.node_namespace)
        node_namespace = self.node_namespace + "_img_pub"
        pkg_name = 'nepi_api'
        node_file_folder = self.api_lib_folder
        node_file_name = 'nepi_pointcloud_img_pub_node.py'

        ###############################
        # Launch Node
        node_file_path = os.path.join(node_file_folder, node_file_name)
        if self.launch_node_process is not None:
            self.msg_if.pub_debug("Node Already Launched: " + node_name)
        elif os.path.exists(node_file_path) == False or self.pub_image == False:
            self.msg_if.pub_debug("Could not find Node File at: " + node_file_path)
        else:
            #Try and launch node
            self.msg_if.pub_debug("Launching Pointcloud Img Node with settings " + str([pkg_name, node_file_name, node_name]))
            self.msg_if.pub_warn("Launching Node: " + node_name)

            param_ns = nepi_sdk.create_namespace(node_namespace, 'data_products')
            nepi_sdk.set_param(param_ns, [self.data_product])

            param_ns = nepi_sdk.create_namespace(node_namespace, 'pointcloud_namespace')
            nepi_sdk.set_param(param_ns, self.namespace)

            # From the IFs themselves: the status msg topic fields are only filled
            # once status starts publishing, which is after this launch
            transform_topic = ''
            if self.transform_if is not None:
                transform_topic = self.transform_if.get_namespace()
            param_ns = nepi_sdk.create_namespace(node_namespace, 'transform_namespace')
            nepi_sdk.set_param(param_ns, transform_topic)

            navpose_topic = ''
            if self.navpose_if is not None:
                navpose_topic = self.navpose_if.get_namespace()
            param_ns = nepi_sdk.create_namespace(node_namespace, 'navpose_namespace')
            nepi_sdk.set_param(param_ns, navpose_topic)

            [success, msg, sub_process] = nepi_sdk.launch_node(pkg_name, node_file_name, node_name, namespace=launch_namespace)
            if success == True:
                self.launch_node_process = sub_process
                self.pub_img_node_name = node_name
                self.pub_img_namepace = node_namespace
            self.msg_if.pub_debug("Node launch return msg: " + str(msg))

    def kill_image_pub_node(self):
        """Terminate the running pointcloud image publisher node.

        Sends a kill signal to the subprocess started by
        ``launch_image_pub_node`` and clears the process handle and node name
        on success. Logs a warning if the node is not currently running.
        """
        if self.launch_node_process is None:
            self.msg_if.pub_debug("Node Not Running")
        else:
            self.msg_if.pub_warn("Killing Node")
            success = nepi_sdk.kill_node_process(self.pub_img_node_name, self.launch_node_process)
            if success == True:
                self.launch_node_process = None
                self.pub_img_node_name = ""
                self.pub_img_namepace = ""
                self.msg_if.pub_debug("Node Killed")
            else:
                self.msg_if.pub_warn("Failed to Kill Node")


    ###############################
    # Class Public Methods
    ###############################


    def _setVoxelCb(self,msg):
        self.msg_if.pub_debug("Received set voxel message: " + str(msg) )
        self.voxel = [msg.x,msg.y,msg.z]
        if self.callback_dict['voxel_callback'] is not None:
            self.callback_dict['voxel_callback'](self.voxel)

    def publish_o3d_pc(self,o3d_pc,
                        timestamp = None,
                        width_deg = 100,
                        height_deg = 70,
                        min_range_m = None,
                        max_range_m = None,
                        process_data = True,
                        pub_twice = False,
                        frame_id = 'sensor',
                        add_pubs = []):
        """Publish an Open3D pointcloud as a ROS PointCloud2 message.

        Also forwards the pointcloud to the PointcloudImageIF for image generatio
        (if pub_image was True at construction) and saves to disk via SaveDataIF
        if registered.

        Args:
            o3d_pc (open3d.geometry.PointCloud): The pointcloud to publish.
            timestamp (float or rospy.Time, optional): Acquisition timestamp.
                Defaults to current time if None.
            width_deg (float, optional): Horizontal field of view in degrees.
                Defaults to 100.
            height_deg (float, optional): Vertical field of view in degrees.
                Defaults to 70.
            min_range_m (float, optional): Minimum valid range in meters. When
                None, range status is set to 0/1 defaults. Defaults to None.
            max_range_m (float, optional): Maximum valid range in meters.
                Defaults to None.
            process_data (bool, optional): Reserved for future use. Defaults to True.
            pub_twice (bool, optional): Publish the pointcloud twice with a brief
                delay. Defaults to False.
            frame_id (str, optional): TF frame ID for the PointCloud2 header.
                Defaults to 'sensor'.
            add_pubs (list, optional): Additional namespace strings to publish to.
                Defaults to [].

        Returns:
            open3d.geometry.PointCloud: The original pointcloud unchanged.
        """
    


        navpose_dict = self.get_navpose_dict()


        if self.publishing == False:
            self.publishing = True

            if o3d_pc is None:
                self.msg_if.pub_debug("Can't publish None image" )
                return False

            if timestamp == None:
                timestamp = nepi_utils.get_time()
            else:
                timestamp = nepi_sdk.sec_from_timestamp(timestamp)

            self.status_msg.has_rgb = o3d_pc.has_colors()
            self.status_msg.has_intensity = False # Need to add
            #self.status_msg.point_count = o3d_pc.point["colors"].shape[0]

            self.status_msg.width_deg = width_deg
            self.status_msg.height_deg = height_deg

            if (min_range_m is not None and max_range_m is not None):
                self._updateRangesM(min_range_m,max_range_m)
                self.status_msg.range_min_max_m.start_range = self.min_range_m
                self.status_msg.range_min_max_m.stop_range = self.max_range_m
                o3d_pc = nepi_pc.range_clip_spherical(o3d_pc, self.min_range_m, self.max_range_m)

            else:
                self.status_msg.range_min_max_m.start_range = 0
                self.status_msg.range_min_max_m.stop_range = 1



            current_time = nepi_utils.get_time()
            latency = (current_time - timestamp)
            self.status_msg.get_latency_time = latency
            #self.msg_if.pub_debug("Get Data Latency: {:.2f}".format(latency) , throttle_s = 5.0)

            # Start Pub Process. Timed from here so process_time covers the process controls
            # below as well as the message conversion and publish.
            start_time = nepi_utils.get_time()

            ##########
            # Apply Process Controls
            # Order is voxel, then uniform, then outlier removal. Outlier removal runs last on
            # the already-reduced cloud even though its neighbor statistics are more meaningful
            # at native density: it builds a KD-tree and runs a per-point k-nearest-neighbor
            # query, by far the most expensive of the three, and is not viable at frame rate on
            # a full-resolution stereo cloud of ~2M points. Decimate first, then filter.
            # Each stage is a no-op at its factory value, so a device that never sets these
            # controls sees the pointcloud it saw before.
            pre_process_pc = o3d_pc

            # Read the controls once per publish call, never per point
            voxel_size_m = None
            uniform_k_points = None
            outlier_num_neighbors = None
            outlier_std_ratio = None
            if self.node_if is not None:
                voxel_size_m = self.node_if.get_param(self.node_if_prefix +  'voxel_downsample_size')
                uniform_k_points = self.node_if.get_param(self.node_if_prefix +  'uniform_downsample_k_points')
                outlier_num_neighbors = self.node_if.get_param(self.node_if_prefix +  'outlier_removal_num_neighbors')
                outlier_std_ratio = self.node_if.get_param(self.node_if_prefix +  'outlier_removal_std_ratio')

            if voxel_size_m is not None and voxel_size_m > 0:
                try:
                    o3d_pc = nepi_pc.voxel_down_sampling(o3d_pc, voxel_size_m)
                except Exception as e:
                    self.msg_if.pub_debug("Failed to voxel downsample pointcloud with size: " + str(voxel_size_m) + " " + str(e) , throttle_s = 5.0)

            # k of zero is invalid input to Open3D and k of one returns every point
            if uniform_k_points is not None and uniform_k_points > 1:
                try:
                    o3d_pc = nepi_pc.uniform_down_sampling(o3d_pc, uniform_k_points)
                except Exception as e:
                    self.msg_if.pub_debug("Failed to uniform downsample pointcloud with k points: " + str(uniform_k_points) + " " + str(e) , throttle_s = 5.0)

            if outlier_num_neighbors is not None and outlier_num_neighbors > 0:
                if outlier_std_ratio is None:
                    outlier_std_ratio = self.DEFAULT_CONTROLS_DICT['outlier_removal_std_ratio']
                try:
                    o3d_pc = nepi_pc.statistical_outlier_removal(o3d_pc, outlier_num_neighbors, outlier_std_ratio)
                except Exception as e:
                    self.msg_if.pub_debug("Failed to remove pointcloud outliers with neighbors and std ratio: " + str([outlier_num_neighbors,outlier_std_ratio]) + " " + str(e) , throttle_s = 5.0)

            # Don't let an over-aggressive control setting blank the data product silently
            if o3d_pc is None or len(o3d_pc.points) == 0:
                #self.msg_if.pub_debug("Process controls returned an empty pointcloud, publishing unprocessed data. Controls: " + str([voxel_size_m,uniform_k_points,outlier_num_neighbors]) , throttle_s = 5.0)
                o3d_pc = pre_process_pc

            self.status_msg.point_count = len(o3d_pc.points)


            if self.node_if is not None and self.needs_data == True:
                #Convert to ros Image message
                ros_pc = nepi_pc.o3dpc_to_rospc(o3d_pc)
                sec = nepi_sdk.sec_from_timestamp(timestamp)
                ros_pc.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = 'sensor')
                #self.msg_if.pub_debug("Publishing Pointcloud with header: " + str(ros_pc.header) , throttle_s = 5.0)
                self.publish_status()
                self.node_if.publish_pub(self.node_if_prefix + 'data_pub', ros_pc)

            process_time = round( (nepi_utils.get_time() - start_time) , 3)
            self.status_msg.process_time = process_time
            latency = (current_time - timestamp)
            self.status_msg.pub_latency_time = latency

            # Save Data
            if self.save_data_if is not None:
                self.save_data_if.save(self.data_product,o3d_pc,timestamp)


            if self.last_pub_time is None:
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time
                self.status_msg.last_pub_sec = pub_time_sec

                self.time_list.pop(0)
                self.time_list.append(pub_time_sec)

            self.publishing = False
            
        return o3d_pc
    

    ###############################
    # Class Private Methods
    ###############################
  
    def _updaterCb(self, timer):

        # Check for other topics
        image_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'color_image')
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





    ###################
    ## Process Control Public API
    #
    # These exist so a host node (e.g. a driver exposing these as SettingsIF
    # cap_settings entries) can drive the process pipeline directly instead of
    # constructing a ROS message to hand to the subscriber callbacks below.
    # Each accepts a value that str()s to a number, so a SettingsIF string value
    # can be passed through unparsed, and returns whether the value was applied.

    def set_voxel_downsample_size(self, size_m):
        """Set the voxel downsample size for the pointcloud process pipeline.

        Args:
            size_m (float): Voxel edge length in meters. Zero disables the voxel
                downsample stage. Negative values are rejected.

        Returns:
            bool: True if the value was applied, False if it was rejected.
        """
        success = False
        try:
            val = float(size_m)
        except (TypeError, ValueError):
            val = None
        if val is not None and val >= 0:
            self.node_if.set_param(self.node_if_prefix +  'voxel_downsample_size', val)
            success = True
        self.publish_status()
        return success

    def set_uniform_downsample_k_points(self, k_points):
        """Set the uniform downsample sampling rate for the process pipeline.

        Args:
            k_points (int): Keep every k-th point. Zero or one disables the
                uniform downsample stage. Negative values are rejected.

        Returns:
            bool: True if the value was applied, False if it was rejected.
        """
        success = False
        try:
            val = int(k_points)
        except (TypeError, ValueError):
            val = None
        if val is not None and val >= 0:
            self.node_if.set_param(self.node_if_prefix +  'uniform_downsample_k_points', val)
            success = True
        self.publish_status()
        return success

    def set_outlier_removal_num_neighbors(self, num_neighbors):
        """Set the neighbor count for the statistical outlier removal stage.

        Args:
            num_neighbors (int): Neighbors considered per point. Zero disables
                the outlier removal stage. Negative values are rejected.

        Returns:
            bool: True if the value was applied, False if it was rejected.
        """
        success = False
        try:
            val = int(num_neighbors)
        except (TypeError, ValueError):
            val = None
        if val is not None and val >= 0:
            self.node_if.set_param(self.node_if_prefix +  'outlier_removal_num_neighbors', val)
            success = True
        self.publish_status()
        return success

    def set_outlier_removal_std_ratio(self, std_ratio):
        """Set the standard deviation ratio for statistical outlier removal.

        Lower values cull more aggressively. Has no effect unless the outlier
        removal neighbor count is greater than zero. 

        Args:
            std_ratio (float): Standard deviation ratio. Must be greater than
                zero; zero and negative values are rejected.

        Returns:
            bool: True if the value was applied, False if it was rejected.
        """
        success = False
        try:
            val = float(std_ratio)
        except (TypeError, ValueError):
            val = None
        if val is not None and val > 0:
            self.node_if.set_param(self.node_if_prefix +  'outlier_removal_std_ratio', val)
            success = True
        self.publish_status()
        return success

    ###################
    ## Process Callbacks
    def resetProcessControlsCb(self,msg):
        self.resetProcessControls()

    def resetProcessControls(self,do_updates = True):
        self.node_if.factory_reset_param('clip_enabled')
        self.node_if.factory_reset_param('clip_selection')
        self.node_if.factory_reset_param('range_min_m')
        self.node_if.factory_reset_param('range_max_m')
        self.bounding_box3d_topic = "NONE"
        self.node_if.factory_reset_param('voxel_downsample_size')
        self.node_if.factory_reset_param('uniform_downsample_k_points')
        self.node_if.factory_reset_param('outlier_removal_num_neighbors')
        self.node_if.factory_reset_param('outlier_removal_std_ratio')

        if do_updates:
            self.publish_status()

    def clipEnableCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        new_enable = msg.data
        self.node_if.set_param(self.node_if_prefix +  'clip_enabled', new_enable)
        self.publish_status()

    def setClipSelectionCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        sel = msg.data
        if sel in self.clip_options:
            self.node_if.set_param(self.node_if_prefix +  'clip_selection', sel )
        self.publish_status()

    def setClipBoxTopicCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        self.bounding_box3d_topic = msg.data
        self.publish_status()

    def setRangeMetersCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        range_min_m = msg.start_range
        range_max_m = msg.stop_range
        if range_min_m < range_max_m:
            self.node_if.set_param(self.node_if_prefix +  'range_min_m', range_min_m)
            self.node_if.set_param(self.node_if_prefix +  'range_max_m', range_max_m)
        self.publish_status()

    def setVoxelSizeCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        self.set_voxel_downsample_size(msg.data)

    def setUniformPointsCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        self.set_uniform_downsample_k_points(msg.data)

    def setOutlierNumCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        self.set_outlier_removal_num_neighbors(msg.data)

    def setFrame3dCb(self, msg):
        #self.msg_if.pub_debug(str(msg))
        frame_3d = msg.data
        frame3d_list = self.frame3d_list
        if frame_3d in frame3d_list:
            self.node_if.set_param(self.node_if_prefix +  'frame_3d',frame_3d)
        self.publish_status()

    ###################
    ## Render Callbacks
    def resetRenderControlsCb(self,msg):
        self.resetRenderControls()

    def resetRenderControls(self,do_updates = True):
        self.node_if.factory_reset_param('rotate_ratio')
        self.node_if.factory_reset_param('tilt_ratio')
        self.node_if.factory_reset_param('render_enable')

        if do_updates:
            self.publish_status()

    def setImageSizeCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        width = msg.image_width
        height = msg.image_height
        if width > 100 and width < 5000 and height > 100 and height < 5000:
            self.node_if.set_param(self.node_if_prefix +  'image_width',  width)
            self.node_if.set_param(self.node_if_prefix +  'image_height', height)
            self.publish_status()

    def setImageSizeIndCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        index = msg.data
        if index < len(nepi_img.STANDARD_IMAGE_SIZES):
            size_str = nepi_img.STANDARD_IMAGE_SIZES[index]
            size__split = size_str.split(" ")
            width = float(size__split[0])
            height = float(size__split[2])
            if width > 100 and width < 5000 and height > 100 and height < 5000:
                self.node_if.set_param(self.node_if_prefix +  'image_width',  width)
                self.node_if.set_param(self.node_if_prefix +  'image_height', height)
                self.publish_status()

    def setZoomRatioCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        new_val = msg.data
        if new_val >= 0 and new_val <= 1 :
            self.node_if.set_param(self.node_if_prefix +  'zoom_ratio',new_val)
            self.publish_status()

    def setRotateRatioCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        new_val = msg.data
        if new_val >= 0 and new_val <= 1 :
            self.node_if.set_param(self.node_if_prefix +  'rotate_ratio',new_val)
            self.publish_status()

    def setTiltRatioCb(self,msg):
        #self.msg_if.pub_debug(str(msg))
        new_val = msg.data
        if new_val >= 0 and new_val <= 1 :
            self.node_if.set_param(self.node_if_prefix +  'tilt_ratio',new_val)
            self.publish_status()


    def setRenderEnableCb(self,msg):
        render_enable = msg.data
        self.node_if.set_param(self.node_if_prefix +  'render_enable', render_enable)
        self.publish_status()







##################################################
# PointcloudImageIF

class PointcloudImageIF(BaseImageIF):

    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = ImageStatus()
    DATA_MSG = Image()

    ID = 'pointcloud_image'
    DISPLAY_NAME = 'Pointcloud Image'
    DESCRIPTION = 'Pointcloud Image Data Product'

    DATA_PRODUCT = 'pointcloud_image'

    #########################
    ## CUSTOM
    #########################

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = True,
        has_flip_horz = True,
        has_flip_vert = True,
        has_range = True,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_zoom_3d = True,
        has_rotate_3d = True,
        has_tilt_3d = True,
        has_camera_3d = True
        )

    DEFAULT_FILTERS_DICT = dict()

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 

        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0,
        start_range_ratio = 0,
        # 1.0 = no range clip. A 0 here makes the renderer clip to
        # [min_range_m, min_range_m] and hand back an empty cloud, so every
        # rendered frame comes out blank. BaseImageIF uses 1.0 for the same key.
        stop_range_ratio = 1.0,
        # [x_min, x_max, y_min, y_max] as ratios: [0,1,0,1] is the full frame. A
        # [0,0,0,0] here is a zero-area window, and the crop step then divides by
        # (x_max - x_min) == 0. init() force-corrects it at startup, so the bad value
        # only ever surfaced through reset_renders, which restores the constructor
        # baseline captured before init() ran. Every other image IF uses [0,1,0,1].
        window_ratios = [0,1,0,1],
        # Camera distance, and the mapping is inverted: for a negative cam_pos x the
        # renderer uses (1 - zoom_3d_ratio) as the multiplier on cam_pos, so 0 is the
        # FARTHEST view (full -5 m) and larger values move the camera in. A 0 here
        # started every pointcloud pinned to the far end of its range, which reads as
        # "zoomed way out". 0.5 is the neutral midpoint, matching how every other
        # ratio control in this file defaults, and puts the camera at -2.5 m with
        # equal headroom to scroll in or out.
        zoom_3d_ratio = 0.5,
        # 0.5 = no rotation. The renderer maps ratio to angle as
        # (0.5 - ratio) * 360, so a 0 here is a 180 deg flip: rotate about Z and
        # tilt about Y both invert, and the cloud renders upside down. Every other
        # image IF uses 0.5 for these two keys.
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5,
        cam_fov = 60,
        cam_view = [3, 0, 0],
        cam_pos = [-5, 0, 0],
        cam_rot = [0, 0, 1],
        # Render background. Present here so the renderer resolves it from
        use_wbg = False
        )


    # #Default Control Values 
    # DEFAULT_OFFSETS_DICT = dict( 

    #     resolution_ratio = 0,
    #     auto_adjust_enabled = False,
    #     auto_adjust_ratio = 0,
    #     brightness_ratio = 0,
    #     contrast_ratio =  0,
    #     threshold_ratio =  0,
    #     start_range_ratio = 0,
    #     stop_range_ratio = 0,
    #     zoom_3d_ratio = 0,
    #     window_ratios = [0,0,0,0],
    #     rotate_3d_ratio = 0,
    #     tilt_3d_ratio = 0,
    #     cam_fov = 0,
    #     cam_view = [0, 0, 0],
    #     cam_pos = [0, 0, 0],
    #     cam_rot = [0, 0, 0]
    #     )

    # # Working copy of the render offsets; drag handlers mutate this, the
    # # reset callback restores it to DEFAULT_OFFSETS_DICT (start orientation).
    #offsets_dict = copy.deepcopy(DEFAULT_OFFSETS_DICT)

    # Anchor state for interactive 3D drags (start pixel is the fixed drag anchor)
    render_3d_drag_anchor = None
    render_3d_drag_last = None


    auto_adjust_controls = []

    # Pointcloud renderer state (lazily created on first render)
    img_renderer = None
    img_renderer_mtl = None
    last_img_width = None
    last_img_height = None
    last_fov = None
    last_bg_white = None

    is_processing = False
    def __init__(self, 
                name = 'pointcloud_image',
                namespace = None ,
                data_product = None,
                data_source_description = None,
                data_ref_description = None,
                perspective = 'pov',
                init_overlay_text_list = [],
                options_dict = None,
                callback_dict = None,
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
        if type(self) is PointcloudImageIF:
            self.start()



    ###############################
    # Class Public Methods
    ###############################

    def publish_pointcloud_img(self,o3d_pc,
                            width_deg = 100,
                            height_deg = 70,
                            render_dict = None,
                            min_range_m = None,
                            max_range_m = None,
                            img_width = None,
                            img_height = None,
                            timestamp = None,
                            frame_id = 'sensor',
                            pub_twice = False
                            ):
        """Render an Open3D pointcloud to an image and publish it.

        Applies the supplied render controls (image size, range clip, zoom,
        rotate, tilt, and camera pose), renders the pointcloud through an
        Open3D offscreen renderer, and publishes the resulting image via the
        BaseImageIF publish pipeline.

        Args:
            o3d_pc (open3d.geometry.PointCloud): The pointcloud to visualize.
            width_deg (float, optional): Horizontal field of view in degrees.
                Defaults to 100.
            height_deg (float, optional): Vertical field of view in degrees.
                Defaults to 70.
            render_dict (dict, optional): Render control values (image_width,
                image_height, start_range_ratio, stop_range_ratio, zoom_3d_ratio,
                rotate_ratio, tilt_ratio, cam_fov, cam_view, cam_pos, cam_rot,
                use_wbg). Falls back to interface defaults when None.
            timestamp (float or rospy.Time, optional): Acquisition timestamp.
                Defaults to current time if None.
            frame_id (str, optional): TF frame ID for the published image.
                Defaults to 'sensor'.
            pub_twice (bool, optional): Publish the resulting image twice.
                Defaults to False.

        Returns:
            tuple: (o3d_pc, cv2_img) where cv2_img is the rendered image, or
            None if rendering produced no image.
        """
        cv2_img = None
        if self.is_processing == True:
            self.is_processing = False
            return o3d_pc, cv2_img
        self.is_processing = True

        
        if o3d_pc is None:
            self.is_processing = False
            return o3d_pc, cv2_img

        if timestamp is None:
            timestamp = nepi_utils.get_time()
        else:
            timestamp = nepi_sdk.sec_from_timestamp(timestamp)

        # Resolve render controls from self.controls_dict. That is the single store
        # every render control writes: the ROS/RUI setters (set_zoom_3d_ratio,
        # set_rotate_3d_ratio, set_tilt_3d_ratio, setCamFovCb, setCamViewCb,
        # setCamPositionCb, setCamRotationCb, setWhiteBgCb) and the interactive
        # drag/scroll/click handlers all update it, and publish_status reports it.
        # Rendering from anything else is what made the ROS and RUI controls inert.
        # A caller-supplied render_dict overrides for this frame only and is not
        # persisted, so it cannot become a competing source of truth.
        resolved = copy.deepcopy(self.controls_dict)
        if render_dict is not None:
            resolved.update(render_dict)

        # Explicit img_width/img_height args override the status-driven size
        if img_width is None:
            img_width = resolved.get('image_width', 955)
        if img_height is None:
            img_height = resolved.get('image_height', 600)
        img_width = int(img_width)
        img_height = int(img_height)
        start_range_ratio = resolved.get('start_range_ratio', 0.0)
        stop_range_ratio = resolved.get('stop_range_ratio', 1.0)
        zoom_ratio = nepi_utils.check_ratio(resolved.get('zoom_3d_ratio', 0))
        rotate_ratio = resolved.get('rotate_3d_ratio', 0.5)
        tilt_ratio = resolved.get('tilt_3d_ratio', 0.5)
        cam_fov = resolved.get('cam_fov', self.DEFAULT_CONTROLS_DICT['cam_fov'])
        # Read each camera vector once. These previously read the same key twice as
        # a "base" plus an "offset" and summed them, which doubled every value
        # (cam_pos [-5,0,0] rendered as [-10,0,0]) and made a camera position set
        # over ROS arrive at twice what was asked for. The offsets_dict those three
        # lines were written against does not exist.
        cam_view = list(resolved.get('cam_view', self.DEFAULT_CONTROLS_DICT['cam_view']))
        cam_pos = list(resolved.get('cam_pos', self.DEFAULT_CONTROLS_DICT['cam_pos']))
        cam_rot = list(resolved.get('cam_rot', self.DEFAULT_CONTROLS_DICT['cam_rot']))
        use_wbg = resolved.get('use_wbg', False)

        # Range clip
        if min_range_m is None:
            min_range_m = nepi_pc.get_min_range(o3d_pc)
        if max_range_m is None:
            max_range_m = nepi_pc.get_max_range(o3d_pc)
        delta_range_m = max_range_m - min_range_m
        clip_min_range_m = min_range_m + start_range_ratio * delta_range_m
        clip_max_range_m = min_range_m + stop_range_ratio * delta_range_m
        if start_range_ratio > 0 or stop_range_ratio < 1:
            o3d_pc = nepi_pc.range_clip_spherical(o3d_pc, clip_min_range_m, clip_max_range_m)

        # Apply zoom via camera position
        if cam_pos[0] < 0:
            zoom_ratio = 1 - zoom_ratio
        cam_pos[0] = cam_pos[0] * zoom_ratio

        # Apply rotate and tilt
        rotate_angle = (0.5 - rotate_ratio) * 2 * 180
        o3d_pc = nepi_pc.rotate_pc(o3d_pc, [0, 0, rotate_angle])
        tilt_angle = (0.5 - tilt_ratio) * 2 * 180
        o3d_pc = nepi_pc.rotate_pc(o3d_pc, [0, tilt_angle, 0])

        bg_color = [1, 1, 1, 1] if use_wbg else [0, 0, 0, 0]

        # (Re)create the renderer when geometry-independent params change
        update_renderer = (self.img_renderer is None or self.img_renderer_mtl is None
            or self.last_img_width != img_width or self.last_img_height != img_height
            or self.last_fov != cam_fov or self.last_bg_white != use_wbg)
        if update_renderer:
            self.img_renderer = None
            self.msg_if.pub_warn("Creating new pointcloud renderer" )
            self.img_renderer = nepi_pc.create_img_renderer(img_width=img_width, img_height=img_height, fov=cam_fov, background=bg_color)
            self.img_renderer_mtl = nepi_pc.create_img_renderer_mtl(shader="defaultUnlit")
            self.last_img_width = img_width
            self.last_img_height = img_height
            self.last_fov = cam_fov
            self.last_bg_white = use_wbg

        # Render the frame
        self.img_renderer = nepi_pc.remove_img_renderer_geometry(self.img_renderer)
        self.img_renderer = nepi_pc.add_img_renderer_geometry(o3d_pc, self.img_renderer, self.img_renderer_mtl)
        o3d_img = nepi_pc.render_img(self.img_renderer, cam_view, cam_pos, cam_rot)
        self.img_renderer = nepi_pc.remove_img_renderer_geometry(self.img_renderer)

        if o3d_img is not None:
            cv2_img = nepi_pc.o3dimg_to_cv2img(o3d_img)

        # Publish and Save Pointcloud Image Data
        if cv2_img is not None:
            self.publish_cv2_img(cv2_img,
                                encoding = 'rgb8',
                                width_deg = width_deg,
                                height_deg = height_deg,
                                min_range_m = min_range_m,
                                max_range_m = max_range_m,
                                timestamp = timestamp,
                                pub_twice = pub_twice
                                )
        self.is_processing = False
        return o3d_pc, cv2_img


    ###############################
    # Class Private Methods
    ###############################

    def render3dDragHandler(self, start_pixel, start_color_bgr, stop_pixel, stop_color_bgr):
        #self.msg_if.pub_debug("3D render drag handler fired - start: " + str(start_pixel) + " stop: " + str(stop_pixel) )
        img_width = self.status_msg.width_px
        img_height = self.status_msg.height_px
        if img_width <= 0 or img_height <= 0:
            return
        # A drag streams many events sharing the same fixed anchor (start) pixel and a
        # moving stop pixel; accumulate the offset from the previous streamed position
        # so a continuous drag maps 1:1 and successive drags keep adding.
        if start_pixel != self.render_3d_drag_anchor:
            self.render_3d_drag_anchor = list(start_pixel)
            self.render_3d_drag_last = list(start_pixel)
        rotate_change = float(stop_pixel[0] - self.render_3d_drag_last[0]) / float(img_width)
        tilt_change = float(stop_pixel[1] - self.render_3d_drag_last[1]) / float(img_height)
        self.render_3d_drag_last = list(stop_pixel)
        rotate_offset = min(1.0, max(-1.0, self.controls_dict['rotate_3d_ratio'] - rotate_change))
        tilt_offset = min(1.0, max(-1.0, self.controls_dict['tilt_3d_ratio'] + tilt_change))
        self.controls_dict['rotate_3d_ratio'] = rotate_offset
        self.controls_dict['tilt_3d_ratio'] = tilt_offset
        self.publish_status()
        self.needs_update()

    def render3dWindowHandler(self, window = None):
        self.msg_if.pub_debug("3D render window handler fired - window: " + str(window) )
        self.publish_status()

    def render3dScrollHandler(self, scroll_pixel, scroll_color_bgr, scroll_amount):
        self.msg_if.pub_debug("3D render scroll handler fired - amount: " + str(scroll_amount) )
        # scroll_amount is a normalized wheel step (+ = zoom in, - = zoom out); accumulate
        # into the zoom offset applied on top of the status-driven zoom in publish.
        zoom_step = float(scroll_amount) * 0.05
        zoom_offset = min(1.0, max(-1.0, self.controls_dict['zoom_3d_ratio'] + zoom_step))
        self.controls_dict['zoom_3d_ratio'] = zoom_offset
        self.publish_status()
        self.needs_update()

    def render3dClickHandler(self, pixel, color_bgr, click_count, angles):
        self.msg_if.pub_debug("3D render click handler fired - pixel: " + str(pixel) )
        img_width = self.status_msg.width_px
        img_height = self.status_msg.height_px
        if img_width <= 0 or img_height <= 0:
            return
        # Click position as a ratio from image center, [-1, 1]
        x_ratio = float(pixel[0] - img_width / 2.0) / float(img_width / 2.0)
        y_ratio = float(pixel[1] - img_height / 2.0) / float(img_height / 2.0)
        # Pan the view by shifting both the camera eye (cam_pos) and look-at target
        # (cam_view) by the same vector so the view direction stays fixed: horizontal
        # click pans sideways (camera y), vertical click pans up/down (camera z).
        # pixel y increases downward, so invert it for up/down.
        pan_scale = 1.0
        pan_y = x_ratio * pan_scale
        pan_z = -y_ratio * pan_scale
        cam_view = list(self.controls_dict['cam_view'])
        cam_pos = list(self.controls_dict['cam_pos'])
        cam_view[1] = cam_view[1] + pan_y
        cam_pos[1] = cam_pos[1] + pan_y
        cam_view[2] = cam_view[2] + pan_z
        cam_pos[2] = cam_pos[2] + pan_z
        self.controls_dict['cam_view'] = cam_view
        self.controls_dict['cam_pos'] = cam_pos
        self.publish_status()
        self.needs_update()

    def reset_render_3d_orientation(self):
        """Reset the interactive 3D orientation offsets (rotate, tilt, zoom) to start."""
        defaults = copy.deepcopy(self.DEFAULT_CONTROLS_DICT)
        
        self.controls_dict['rotate_3d_ratio'] = defaults['rotate_3d_ratio']
        self.controls_dict['tilt_3d_ratio'] = defaults['tilt_3d_ratio']
        self.controls_dict['zoom_3d_ratio'] = defaults['zoom_3d_ratio']
        self.render_3d_drag_anchor = None
        self.render_3d_drag_last = None
        self.publish_status()
        self.needs_update()

    def reset_render_3d_position(self):
        """Reset the interactive 3D position offsets (pan / camera translation) to start."""
        defaults = copy.deepcopy(self.DEFAULT_CONTROLS_DICT)

        self.controls_dict['cam_view'] = defaults['cam_view']
        self.controls_dict['cam_pos'] = defaults['cam_pos']
        self.publish_status()
        self.needs_update()



