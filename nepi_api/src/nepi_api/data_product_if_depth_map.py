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
# DepthMapIF

class DepthMapIF(DataProductIF):


    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = DepthMapStatus()
    DATA_MSG = Image()

    ID = 'depth_map'
    DISPLAY_NAME = 'Depth Map'
    DESCRIPTION = 'Depth Map Data Product'

    DATA_PRODUCT = 'depth_map'

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

    image_if = None

    ready = False

    DEFUALT_IMG_WIDTH_PX = 700
    DEFUALT_IMG_HEIGHT_PX = 400

    DEFAULT_WIDTH_DEG = 100
    DEFAULT_HEIGHT_DEG = 70

    DEFAULT_CALLBACK_DICT = dict(
        needs_update_callback = None,
        mouse_event_callback = None,
        click_pixel_callback = None,
        click_angle_callback = None,
        drag_callback = None,
        window_callback = None,
        frame_updated_callback = None,
        options_updated_callback = None
    )

    last_width = DEFUALT_IMG_WIDTH_PX
    last_height = DEFUALT_IMG_HEIGHT_PX

    blank_img = nepi_img.create_cv2_blank_img(DEFUALT_IMG_WIDTH_PX, DEFUALT_IMG_HEIGHT_PX, color_rgb = (0, 0, 0) )

    min_range_m = 0.0
    max_range_m = 1.0

    perspective = 'pov'

    live_adjustments_disabled = False
    aspect_adjustment_disabled = False

    def __init__(self, 
                name = 'depth_map',
                namespace = None,
                data_product = None,
                data_source_description = 'depth_map_sensor',
                data_ref_description = 'sensor',
                perspective = 'pov',
                pub_image = True,
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
 
  

        ##############################
        # Update Class Variables
        ##############################

        name = nepi_utils.get_clean_name(name)
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product.strip() != '':
                name = data_product


        self.perspective = perspective

        self.live_adjustments_disabled = live_adjustments_disabled
        self.aspect_adjustment_disabled = aspect_adjustment_disabled

 

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

        # NodeIF's own ready flag. Not self.wait_for_ready(): DepthMapIF overrides it to
        # wait on self.ready, which is set only at the end of this __init__.
        ready = super().get_ready_state()


       #######################
        # Add Services
        #######################
       
        params_dict = {}
        self.add_params(params_dict)

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
        update_subs_dict = {}
        self.add_subs(update_subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################


        #######################
        # Update Class Variables
        #######################


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
            self.status_msg.controls_options_topic = self.options_if.get_namespace()





        #########################
        # Update Status Msg
        ##############################
        self.status_msg.source_description = self.data_source_description
        self.status_msg.end_description = self.data_ref_description

        self.status_msg.publishing = False
        self.status_msg.encoding = '32FC1'
        self.status_msg.width_px = 0
        self.status_msg.height_px = 0
        self.status_msg.get_latency_time = 0
        self.status_msg.pub_latency_time = 0
        self.status_msg.process_time = 0
        self.status_msg.img_pub_enabled = pub_image

        ##############################
        # Start Node Processes
        ##############################



        ####################
        # Setup Save Data IF Class 
        self.msg_if.pub_debug("####################" )
        self.msg_if.pub_debug("Got Save Data IF is None: " + str(save_data_if is None) )
        if save_data_if is not None and save_data_if != 'None':
            self.save_data_if = save_data_if
            data_products = self.save_data_if.get_data_products()
            if self.data_product not in data_products:
                self.save_data_if.register_data_product(self.data_product)
    



        ################
        if pub_image == True:
            self.image_if = DepthMapImageIF(namespace = self.namespace, 
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        perspective = self.perspective,
                        init_overlay_text_list = init_overlay_text_list,
                        options_dict = options_dict,
                        callback_dict = callback_dict,
                        live_adjustments_disabled = self.live_adjustments_disabled,
                        aspect_adjustment_disabled = self.aspect_adjustment_disabled,
                        msg_if = self.msg_if,
                        node_if = self.node_if,
                        save_data_if = self.save_data_if,
                        transform_if = self.transform_if,
                        navpose_if = self.navpose_if,
                        )

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
        if type(self) is DepthMapIF:
            self.start()

    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):

        status_msg.navpose_msg = self.get_navpose_msg()
           
        status_msg.min_range_m = self.min_range_m
        status_msg.max_range_m = self.max_range_m
        avg_rate = 0
        if len(self.time_list) > 0:
            avg_time = sum(self.time_list) / len(self.time_list)
            if avg_time > .01:
                avg_rate = float(1) / avg_time
        status_msg.avg_pub_rate = avg_rate

        return status_msg


    
    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        """Return the current ready state of the depth map interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or a timeout elapses.

        Args:
            timeout (float, optional): Maximum seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready within the timeout, False otherwise.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_debug("Waiting for connection" )
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_debug("Failed to Connect" )
            else:
                self.msg_if.pub_debug("Connected" )
        return self.ready

    def get_namespace(self):
        """Return the ROS namespace for this depth map interface.

        Returns:
            str: The fully-qualified ROS namespace.
        """
        return self.namespace


    def get_depth_map_callback_options(self):
        """Return the list of supported depth map callback names.

        Returns:
            list: Callback name strings registered in the callback dictionary.
        """
        return list(self.callback_dict.keys())

    def set_depth_map_callback(self,name,function):
        """Register a callable for the named depth map callback slot.

        Args:
            name (str): Name of the callback slot (must be in the callback dict).
            function (callable): Function to call when the event fires.
        """
        self.msg_if.pub_debug("Got set callback for: " + str(name) )
        if name in self.callback_dict.keys():
            self.msg_if.pub_debug("Callback set for: " + str(name) )
            self.callback_dict[name] = function
        #self.msg_if.pub_debug("Updated callback dict: " + str(self.callback_dict) )

    def clear_depth_map_callback(self,name):
        """Clear (un-register) the callable for the named depth map callback slot.

        Args:
            name (str): Name of the callback slot to clear.
        """
        self.msg_if.pub_debug("Got clear callback for: " + str(name) )
        if name in self.callback_dict.keys():
            self.callback_dict[name] = None

    def get_image_callback_options(self):
        """Return the list of supported image callback names from the image sub-interface.

        Returns:
            list: Callback name strings from the DepthMapImageIF, or an empty list
            if no image interface is attached.
        """
        if self.image_if is not None:
            return self.image_if.get_image_callback_options()
        else:
            return []

    def set_image_callback(self,name,function):
        """Register a callable for a named callback slot in the image sub-interface.

        Args:
            name (str): Name of the image callback slot.
            function (callable): Function to call when the event fires.
        """
        if self.image_if is not None:
            self.msg_if.pub_debug("Got set image callback for: " + str(name) )
            if name in self.image_if.get_image_callback_options():
                self.msg_if.pub_debug("Image Callback set for: " + str(name) )
                self.image_if.set_callback(name,function)
        #self.msg_if.pub_debug("Updated callback dict: " + str(self.callback_dict) )

    def clear_image_callback(self,name):
        """Clear the callable for a named callback slot in the image sub-interface.

        Args:
            name (str): Name of the image callback slot to clear.
        """
        if self.image_if is not None:
            self.msg_if.pub_debug("Got clear image callback for: " + str(name) )
            if name in self.image_if.get_image_callback_options():
               self.image_if.callback_dict[name] = None

    def set_image_pub_enabled(self, enabled = True):
        """Declare whether a depth map image is being published for this depth map.

        For producers that were constructed with pub_image = False because they
        publish their own depth map image rather than letting this interface
        colorize the raw array -- a file-backed publisher serving a pre-rendered
        image, for example. Such a producer still owes consumers the standard
        contract: the image published at <namespace>/depth_map_image and
        DepthMapStatus.img_pub_enabled reporting True, which is the flag every
        consumer gates its depth map image topic on.

        This only sets the reported flag. It does not create, destroy or
        reconfigure the internal DepthMapImageIF, and it is not needed by
        producers constructed with pub_image = True, which already report True.
        A producer calling this is responsible for publishing the image at
        <namespace>/depth_map_image itself.

        Note that status_msg is declared at class scope, so co-resident
        DepthMapIF instances in one process share it -- as they already do for
        every other status field this class writes.

        Args:
            enabled (bool, optional): True while a depth map image is being
                published for this depth map, False when none is. Defaults to True.
        """
        self.status_msg.img_pub_enabled = enabled

    def publish_np_depth_map(self, np_depth_map,
                            encoding = '32FC1',
                            width_deg = 100,
                            height_deg = 70,
                             min_range_m = 0,
                             max_range_m = 100,
                             timestamp = None,
                             pub_twice = False
                            ):
        """Publish a NumPy depth map array as a ROS Image message.

        Also forwards the depth map to the DepthMapImageIF for colorized image
        generatio (if pub_image was True at construction) and saves to disk via
        SaveDataIF if registered.

        Args:
            np_depth_map (numpy.ndarray): 2-D float32 depth map array in meters.
            encoding (str, optional): ROS image encoding. Defaults to '32FC1'.
            width_deg (float, optional): Horizontal field of view in degrees.
                Defaults to 100.
            height_deg (float, optional): Vertical field of view in degrees.
                Defaults to 70.
            min_range_m (float, optional): Minimum valid depth in meters.
                Defaults to 0.
            max_range_m (float, optional): Maximum valid depth in meters.
                Defaults to 100.
            timestamp (float or rospy.Time, optional): Acquisition timestamp.
                Defaults to current time if None.
            pub_twice (bool, optional): Publish the image message twice with a
                brief delay to work around subscriber latching issues.
                Defaults to False.

        Returns:
            numpy.ndarray: The original depth map array unchanged.
        """
        
        navpose_dict = self.get_navpose_dict()

        #print('Pub Min Max Depths: ' + str([np.nanmin(np_depth_map),np.nanmax(np_depth_map)]) )
        success = False

        if self.publishing == False:
            self.publishing = True

            if np_depth_map is None and self.status_msg is not None:
                self.msg_if.pub_debug("Can't publish None image" )
                return np_depth_map


            self.status_msg.encoding = encoding

            if timestamp == None:
                timestamp = nepi_utils.get_time()
            else:
                timestamp = nepi_sdk.sec_from_timestamp(timestamp)


            current_time = nepi_utils.get_time()
            latency = (current_time - timestamp)
            self.status_msg.get_latency_time = latency
            #self.msg_if.pub_debug("Get Data Latency: {:.2f}".format(latency) , throttle_s  = 30)

            # Start Img Pub Process
            start_time = nepi_utils.get_time()   

            # Publish and Save org Image Data if Required  
            [height,width] = np_depth_map.shape[0:2]
            last_width = self.status_msg.width_px
            last_height = self.status_msg.height_px
            self.status_msg.width_px = width
            self.status_msg.height_px = height

            self.status_msg.width_deg = width_deg
            self.status_msg.height_deg = height_deg
            #self.msg_if.pub_debug('Pub Depth Got Min Max Depths: ' + str([min_range_m, max_range_m]) )
            if (min_range_m is not None and max_range_m is not None):
                self._updateRangesM(min_range_m,max_range_m)
            else:
                self._updateRangesM(0,1)

            #self.msg_if.pub_debug('Pub Depth Adj Min Max Depths: ' + str([self.min_range_m, self.max_range_m]) )

            self.status_msg.publishing = True

            if self.node_if is not None and self.needs_data == True:
                #Convert to ros Image message
                try:
                    ros_img = nepi_img.cv2img_to_rosimg(np_depth_map, encoding=encoding)
                    sec = nepi_sdk.sec_from_timestamp(timestamp)
                    ros_img.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = 'None')
                    #self.msg_if.pub_debug("Publishing Image with header: " + str(ros_img.header) , throttle_s = 5.0)
                    self.publish_status()
                    self.node_if.publish_pub(self.node_if_prefix + 'data_pub', ros_img)
                except Exception as e:
                    self.msg_if.pub_debug("Failed to publish Depth Map: " + str(e) , throttle = 5)

            process_time = round( (nepi_utils.get_time() - start_time) , 3)
            self.status_msg.process_time = process_time
            latency = (current_time - timestamp)
            self.status_msg.pub_latency_time = latency

            
            
            if self.image_if is not None:
                needs_img = self.image_if.needs_data_check()
                if needs_img == True:
                    self.image_if.publish_depth_map_img(np_depth_map,
                                        min_range_m = self.min_range_m, 
                                        max_range_m = self.max_range_m,
                                        width_deg = width_deg,
                                        height_deg = height_deg,
                                        timestamp = timestamp,
                                        pub_twice = pub_twice
                                    )

            # Save Data
            if self.save_data_if is not None:
                self.save_data_if.save(self.data_product,np_depth_map,timestamp)

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

        return np_depth_map

    def unregister_pubs(self):
        """Unregister all ROS publishers managed by this depth map interface."""          
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_pubs()
            else:
                if self.node_if is not None:
                    if self.node_pubs_dict is not None:
                        for pub_name in self.node_pubs_dict.keys():
                            self.node_if.unregister_pub(pub_name)

    def register_pubs(self):
        """Re-register all ROS publishers managed by this depth map interface."""
        if self.node_if is not None:
           self.node_if.register_pubs(self.node_pubs_dict)

    def unregister(self):
        """Shut down this pointcloud interface and release all ROS resources."""
        self.ready = False
        if self.node_if is not None:
            if self.node_if_shared == False:
                self.node_if.unregister_class()
                nepi_sdk.wait()
                self.node_if = None
            else:
                if self.node_srvs_dict is not None:
                        for service_name in self.node_srvs_dict.keys():
                            self.node_if.unregister_service(service_name)
                self.service_name = None

                if self.node_subs_dict is not None:
                        for sub_name in self.node_subs_dict.keys():
                            self.node_if.unregister_sub(sub_name)
                self.node_subs_dict = None

                if self.node_if is not None:
                    if self.node_pubs_dict is not None:
                        for pub_name in self.node_pubs_dict.keys():
                            self.node_if.unregister_pub(pub_name)



    def init(self, do_updates = False):
        """Initialize or re-initialize depth map interface state and publish status.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.
        """
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        """Reset the depth map interface to its initialized state."""
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_debug("Reseting params" )
            self.node_if.reset_params()
            nepi_sdk.sleep(1)     
        self.init(do_updates = True)


    def factory_reset(self):
        """Reset the depth map interface to factory defaults."""
        if self.node_if is not None and self.node_if_shared == False:
            self.msg_if.pub_debug("Factory resetting params" )
            self.node_if.factory_reset_params()
            nepi_sdk.sleep(1) 
        self.init(do_updates = True)


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

        if self.save_config_needed == True and self.node_if is not None:
            self.save_config_needed = False
            self.node_if.save_config()

        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)


    def _needsDataCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers(self.node_if_prefix + 'data_pub')
        needs_save = False
        needs_snapshot = False
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
        needs_data = has_subs or needs_save or needs_snapshot
        img_needs_data = None
        if self.image_if is not None:
            img_needs_data = self.image_if.needs_data_check()
            needs_data = needs_data or img_needs_data
        if needs_data == False and self.status_msg is not None:
            self.status_msg.publishing = False
        self.needs_data = needs_data
        #self.msg_if.pub_debug("Needs Data Check End: " + self.namespace + " : " + str([has_subs,needs_save, needs_snapshot,img_needs_data]) )
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)


    def _publishStatusCb(self,timer):
        self.publish_status()

    def needs_update(self):
        """Signal that a parameter change requires a new depth map frame.

        Sets the save-config flag and fires the registered needs_update_callback
        if one has been set.
        """
        self.save_config_needed = True
        if self.callback_dict['needs_update_callback'] is not None:
            self.callback_dict['needs_update_callback']()

    def _updateRangesM(self, min_m, max_m):
        if min_m < 0:
            min_m = 0
        if min_m <= max_m:
          self.min_range_m = min_m
          self.max_range_m = max_m
        else:
          self.msg_if.pub_debug("Invalid ranges supplied: " + str([min_m,max_m]) )


##################################################
# DepthMapImageIF

class DepthMapImageIF(BaseImageIF):

    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = ImageStatus()
    DATA_MSG = Image()

    ID = 'depth_map_image'
    DISPLAY_NAME = 'Depth Image'
    DESCRIPTION = 'Depth Image Data Product'

    DATA_PRODUCT = 'depth_map_image'

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
        has_zoom = True,
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
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5,
        )


    auto_adjust_controls = []



    def __init__(self, 
                name = 'depth_map_image',
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
        if type(self) is DepthMapImageIF:
            self.start()



    ###############################
    # Class Public Methods
    ###############################

    def publish_depth_map_img(self,np_depth_map,
                            min_range_m = 0,
                            max_range_m = 100,
                            width_deg = 100,
                            height_deg = 70,
                            timestamp = None,
                            pub_twice = False
                            ):
        """Convert a depth map to a colorized image and publish it.

        Applies the configured range ratios to clip the depth map, converts it
        to a BGR colorized image using a jet colormap, then calls publish_cv2_img
        to apply orientation controls and publish the result.

        Args:
            np_depth_map (numpy.ndarray): 2-D float32 depth map in meters.
            min_range_m (float, optional): Minimum depth value for colorization.
                Defaults to 0.
            max_range_m (float, optional): Maximum depth value for colorization.
                Defaults to 100.
            width_deg (float, optional): Horizontal field of view in degrees.
                Defaults to 100.
            height_deg (float, optional): Vertical field of view in degrees.
                Defaults to 70.
            timestamp (float or rospy.Time, optional): Acquisition timestamp.
                Defaults to current time if None.
            pub_twice (bool, optional): Publish the image twice to work around
                subscriber latching issues. Defaults to False.

        Returns:
            numpy.ndarray: The colorized BGR image, or None if input was None.
        """

        cv2_img = None
        if np_depth_map is not None:
            start_range_ratio = self.controls_dict['start_range_ratio']
            stop_range_ratio = self.controls_dict['stop_range_ratio']

            #self.msg_if.pub_debug('Pub Img Got Min Max Depths: ' + str([min_range_m, max_range_m]) )
            cv2_img = nepi_img.npDepthMap_to_cv2ColorImg(np_depth_map, min_range_m = min_range_m, max_range_m = max_range_m,min_ratio = start_range_ratio, max_ratio = stop_range_ratio)      
            if cv2_img is not None:
                self.publish_cv2_img(cv2_img,
                                    encoding = 'bgr8',
                                    width_deg = width_deg,
                                    height_deg = height_deg,
                                    min_range_m = min_range_m, 
                                    max_range_m = max_range_m,
                                    timestamp = timestamp,
                                    pub_twice = pub_twice
                                 )
            
            return cv2_img


    ###############################
    # Class Private Methods
    ###############################



    def process_cv2_img(self, cv2_img):
        """Apply the depth map image processing pipeline to a colorized depth image.

        Applies, in order: resolution scaling, 2-D rotation and flips, crop
        window, drag-selection overlay, user-defined filters, and then either
        manual brightness/contrast/sharpness adjustments or auto-adjustment.

        Args:
            cv2_img (numpy.ndarray): Input BGR colorized depth image array.

        Returns:
            numpy.ndarray: The fully-processed BGR image.
        """

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


