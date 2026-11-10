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
import numpy as np
import math
import time 
import copy
import sys
import yaml

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from geographic_msgs.msg import GeoPoint


from geometry_msgs.msg import Vector3

from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryResponse, DeviceInfoQueryRequest

from nepi_interfaces.msg import DeviceNPXStatus
from nepi_interfaces.srv import  DeviceNPXStatusQuery, DeviceNPXStatusQueryRequest, DeviceNPXStatusQueryResponse


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_nav

from nepi_api.messages_if import MsgIF

from nepi_api.device_if import DeviceIF

from nepi_api.system_if import NavPoseIF
#########################################
# Node Class
#########################################


class NPXDeviceIF(DeviceIF):

    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = DeviceNPXStatus()
    DATA_MSG = None

    ID = 'npx'
    DESCRIPTION = 'NavPose Sensor Device'

    DATA_SOURCE_DESCRIPTION = 'Device'
    DATA_REF_DESCRIPTION = 'Device Center'

    #Factory Control Values 
    FACTORY_CONTROLS_DICT = dict()

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

    caps_dict = dict()
    caps_dict['has_location'] = False
    caps_dict['has_heading'] = False
    caps_dict['has_orientation'] = False
    caps_dict['has_position'] = False
    caps_dict['has_altitude'] = False
    caps_dict['has_depth'] = False
    caps_dict['has_pan_tilt'] = False


    MIN_PUB_RATE = 1.0
    NAVPOSE_FRAME_ID_OPTIONS = ['sensor_frame','custom_frame']
    NAVPOSE_NAV_FRAME_OPTIONS = ['ENU','NED','UKNOWN']
    NAVPOSE_ALT_FRAME_OPTIONS = ['WGS84','AMSL','AGL','MSL','HAE','BAROMETER','UKNOWN']
    NAVPOSE_DEPTH_FRAME_OPTIONS = ['MSL','TOC','DF','KB','DEPTH','UKNOWN']

    DEFAULT_UPDATE_RATE = 30
    # [min, max] Hz clamp range for the update rate. Mirrors the caps report range
    # [MIN_PUB_RATE, DEFAULT_UPDATE_RATE]; used by _setUpdateMaxRateCb.
    NAVPOSE_UPDATE_RATE_OPTIONS = [MIN_PUB_RATE, DEFAULT_UPDATE_RATE]
    DEFAULT_3D_FRAME = 'sensor_frame'
    DEFAULT_NAV_FRAME = 'ENU'
    DEFAULT_ALT_FRAME = 'WGS84'
    DEFAULT_DEPTH_FRAME = 'DEPTH'


    data_products_list = ['navpose']
    
    update_rate = DEFAULT_UPDATE_RATE
    navpose_frame = DEFAULT_3D_FRAME
    frame_nav = DEFAULT_NAV_FRAME
    frame_altitude = DEFAULT_ALT_FRAME
    frame_depth = DEFAULT_DEPTH_FRAME


    has_location = False  
    has_heading = False
    has_orientation = False
    has_position = False
    has_altitude = False
    has_depth = False
    has_pan_tilt = False
    supports_updates = True

    set_location_source = False
    set_heading_source = False
    set_orientation_source = False  
    set_position_source = False
    set_altitude_source = False
    set_depth_source = False
    set_pan_tilt_source = False

    was_location_source = False
    was_heading_source = False
    was_orientation_source = False  
    was_position_source = False
    was_altitude_source = False
    was_depth_source = False
    was_pan_tilt_source = False

    navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)


    has_updated = False

    pub_subs = False

    getNavPoseCb = None
    supports_updates = True

    # True once the has_* flags have been derived from the telemetry source (or once
    # it is known there is no source to derive them from). The NavPose IF is created
    # at the moment this becomes True, so its publisher set and the reported flags are
    # always decided together, from the same data.
    navpose_caps_known = False
    # Set once the "Waiting on NavPose source" warning has been logged. A source can
    # be absent indefinitely (an app whose NavPose selection is None), so the wait
    # is announced once at warn and only repeats at debug.
    navpose_wait_warned = False

    #######################
    ### IF Initialization
    def __init__(self, 
                    device_info,
                    factory_controls = dict(),
                    getSettingsFunction=None, setSettingFunction=None, 
                    data_source_description = None,
                    data_ref_description = None,
                    navpose_frame = 'sensor_frame', frame_nav = 'ENU',
                    frame_altitude = 'WGS84', frame_depth = 'DEPTH',
                    getNavPoseCb = None,
                    max_navpose_update_rate = DEFAULT_UPDATE_RATE,
                    msg_if = None,
                    node_if = None,
                    settings_if = None,
                    save_data_if = None,
                    connect_navpose_if = None,
                    ):

            ##############################
            # Instance Copies of Class State
            ##############################
            self.navpose_dict = copy.deepcopy(self.navpose_dict)


            self.start_time = nepi_utils.get_time()
            ##############################  
            # Create Msg Class
            if msg_if is None:
                msg_if = MsgIF()
            self.msg_if = msg_if
           

            ##############################
            # Initialize Class   

            if navpose_frame is not None:
                self.navpose_frame = navpose_frame

            if frame_nav is not None:
                self.frame_nav = frame_nav

            if frame_altitude is not None:
                self.frame_altitude = frame_altitude

            if frame_depth is not None:
                self.frame_depth = frame_depth
        

            # Probe the telemetry source once for its capability flags. The callback is
            # retained either way: a source that is not connected yet is not a broken
            # source, and disabling it here is what made a device whose bridge or link
            # comes up later report has_position = False for the life of the node.
            callback_dict = dict()
            self.getNavPoseCb = getNavPoseCb
            if getNavPoseCb is not None:
                navpose_dict = None
                try:
                    navpose_dict = getNavPoseCb()
                except Exception as e:
                    self.msg_if.pub_warn("getNavPoseCb failed during initialization: " + str(e) )

                if navpose_dict is None:
                    # Source not up yet. Defer the NavPose IF to _updateNavPoseDictCb so
                    # its capability flags get derived from real telemetry rather than
                    # frozen at all-False here. See createNavPosePubIf below.
                    self.msg_if.pub_info("NavPose source not ready during initialization, deferring NavPose IF until first telemetry" )
                else:
                    callback_dict['get_navpose'] = self.getNavPoseCb
            else:
                # No telemetry callback at all. Nothing to learn later, so the flags are
                # already final and the NavPose IF is created inline as it always was.
                self.navpose_caps_known = True



            ##############################
            # Update Class Variables
            ##############################


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
            update_callback_dict = callback_dict

            ###########
            # NODE IF SETUP

            # Call the parent class constructor
            super().__init__(                
                    device_info = device_info, 
                    getSettingsFunction=getSettingsFunction,
                    setSettingFunction=setSettingFunction, 
                    factoryControls = factory_controls, 
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

            if getNavPoseCb is not None and navpose_dict is not None:
                self.deriveNavPoseCaps(navpose_dict)

            #######################
            # Update NodeIF NodeClass
            #######################

            alt_namespace = None
            if self.device_name != self.node_name:
                alt_namespace = self.node_namespace.replace(self.node_name,self.device_name)



            # Params Config Dict ####################
            params_dict = {
                'update_rate': {
                    'namespace': self.namespace,
                    'factory_val': self.update_rate
                },
                'navpose_frame': {
                    'namespace': self.namespace,
                    'factory_val': self.navpose_frame
                }

            }
        
            # Services Config Dict ####################
            services_dict = {

                'status_query': {
                    'namespace': self.namespace,
                    'topic': 'status_query',
                    'srv': DeviceNPXStatusQuery,
                    'req': DeviceNPXStatusQueryRequest(),
                    'resp': DeviceNPXStatusQueryResponse(),
                    'callback': self.status_query_callback
                }
            }



            # Publishers Config Dict ####################
            pubs_dict = {}
                                

            # Subscribers Config Dict ####################
            subs_dict = {
                'set_max_update_rate': {
                    'namespace': self.namespace,
                    'topic': 'set_max_update_rate',
                    'msg': Float32,
                    'qsize': 1,
                    'callback': self._setUpdateMaxRateCb,
                    'callback_args': ()
                },
                # 'set_navpose_frame': {
                #     'namespace': self.namespace,
                #     'topic': 'set_navpose_frame',
                #     'msg': String,
                #     'qsize': 1,
                #     'callback': self._setNavPoseFrameCb,
                #     'callback_args': ()
                # },
            }



            #######################
            self.add_params(params_dict)
            self.add_services(services_dict)
            self.add_pubs(pubs_dict)
            self.add_subs(subs_dict)
            nepi_sdk.sleep(0.1)

            ######################################################


            #########################
            # Update Status Message
            ##############################
            # Fill in capabilities, which status_query returns with the status

            self.status_msg.device_name = self.device_name
            self.status_msg.device_path = self.path
            self.status_msg.device_node_name = self.node_name
            self.status_msg.serial_num = self.serial_num
            self.status_msg.hw_version = self.hw_version
            self.status_msg.sw_version = self.sw_version


            self.status_msg.has_heading = self.has_heading
            self.status_msg.has_position = self.has_position
            self.status_msg.has_orientation = self.has_orientation
            self.status_msg.has_location = self.has_location
            self.status_msg.has_altitude = self.has_altitude
            self.status_msg.has_depth =  self.has_depth
            self.status_msg.has_pan_tilt = self.has_pan_tilt


            self.status_msg.supports_updates = self.supports_updates

            self.status_msg.pub_rate_min_max = [self.MIN_PUB_RATE,max_navpose_update_rate]

            self.status_msg.data_products =  self.data_products_list


            # Initialize navpose_dict
            self.navpose_dict['navpose_frame'] = self.navpose_frame
            self.navpose_dict['frame_nav'] = self.frame_nav
            self.navpose_dict['frame_altitude'] = self.frame_altitude
            self.navpose_dict['frame_depth'] = self.frame_depth

            self.status_msg.source_description = self.data_source_description
            self.status_msg.source_ref_description = self.data_ref_description


            self.status_msg.has_location = self.has_location
            self.status_msg.has_heading = self.has_heading
            self.status_msg.has_orientation = self.has_orientation
            self.status_msg.has_position = self.has_position
            self.status_msg.has_altitude = self.has_altitude
            self.status_msg.has_depth = self.has_depth
            self.status_msg.has_pan_tilt = self.has_pan_tilt

            self.status_msg.navpose_frame = self.navpose_frame

            self.status_msg.source_frame_nav = self.frame_nav
            self.status_msg.source_frame_altitude = self.frame_altitude
            self.status_msg.source_frame_depth = self.frame_depth

            ##############################
            # Start Node Processes
            ##############################




            ####################
            # Create a NavPose IF, but only if the capability flags are actually known.
            # NavPoseIF decides its publisher set from these flags once, at construction,
            # and has no way to add a component later, so creating it before the telemetry
            # source has reported would lock in an all-False publisher set permanently.
            # When the source is not up yet, _updateNavPoseDictCb creates it on the first
            # successful read instead.
            if self.navpose_caps_known == True:
                self.createNavPosePubIf()

            nepi_sdk.start_timer_process(0.1, self._updateNavPoseDictCb, oneshot = True)
            
            ###############################
            # Finish Initialization

            self.ready = True
            self.msg_if.pub_info("Initialization Complete", color = 'GREEN' )

            ##############################
            # Start NodeIF Processes
            if type(self) is NPXDeviceIF:
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

        """Builds and publishes the DeviceNPXStatus message to the status topic.

        Updates the device name and current update rate on the status message, then
        publishes it on the latched status publisher if the node interface is available.
        """
        status_msg.update_rate = self.update_rate

        status_msg.data_products = self.data_products_list

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
        #self.msg_if.pub_info("Setting init values to param values" )
        if self.node_if is not None:
            update_rate = self.node_if.get_param(self.node_if_prefix + 'update_rate')
            if update_rate is not None:
                self.set_update_max_rate(update_rate)
            navpose_frame = self.node_if.get_param(self.node_if_prefix + 'navpose_frame')
            if navpose_frame is not None:
                self.navpose_frame = str(navpose_frame)
        if do_updates == True:
            pass
        self.publish_status()


    ###############################
    # Class Methods
    ###############################    

    def set_update_max_rate(self,rate):
        min = self.NAVPOSE_UPDATE_RATE_OPTIONS[0]
        max = self.NAVPOSE_UPDATE_RATE_OPTIONS[1]
        if rate < min:
            rate = min
        if rate > max:
            rate = max
        self.update_rate = rate
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'update_rate',rate)





    def deriveNavPoseCaps(self,navpose_dict):
        # Derive the has_* capability flags from a telemetry dict and mirror them into
        # the status message, which status_query also returns. Called exactly once per node life --
        # either from __init__ when the source answers immediately, or from
        # _updateNavPoseDictCb on the first successful read when it does not. The flags
        # cannot move after that, because NavPoseIF is created from them in the same
        # breath and its publisher set is fixed at its construction.

        caps_dict = dict()
        caps_dict['has_location'] = navpose_dict['has_location']
        caps_dict['has_heading'] = navpose_dict['has_heading']
        caps_dict['has_orientation'] = navpose_dict['has_orientation']
        caps_dict['has_position'] = navpose_dict['has_position']
        caps_dict['has_altitude'] = navpose_dict['has_altitude']
        caps_dict['has_depth'] = navpose_dict['has_depth']
        caps_dict['has_pan_tilt'] = navpose_dict['has_pan_tilt']
        self.caps_dict = caps_dict


        self.has_location = navpose_dict['has_location']
        self.has_heading = navpose_dict['has_heading']
        self.has_orientation = navpose_dict['has_orientation']
        self.has_position = navpose_dict['has_position']
        self.has_altitude = navpose_dict['has_altitude']
        self.has_depth = navpose_dict['has_depth']
        self.has_pan_tilt = navpose_dict['has_pan_tilt']

        self.status_msg.has_location = self.has_location
        self.status_msg.has_heading = self.has_heading
        self.status_msg.has_orientation = self.has_orientation
        self.status_msg.has_position = self.has_position
        self.status_msg.has_altitude = self.has_altitude
        self.status_msg.has_depth = self.has_depth
        self.status_msg.has_pan_tilt = self.has_pan_tilt

        self.navpose_caps_known = True



    def createNavPosePubIf(self):
        # Create the NavPose IF from the current has_* flags. Split out of __init__ so
        # the deferred path uses the same construction, with the same namespace and the
        # same arguments, as the immediate one.
        np_namespace = self.namespace #+ '/pub'
        self.navpose_pub_if = NavPoseIF(namespace = np_namespace,
                            data_source_description = self.data_source_description,
                            data_ref_description = self.data_ref_description,
                            pub_navpose = True,
                            pub_location = self.has_location,
                            pub_heading = self.has_heading,
                            pub_orientation =  self.has_orientation,
                            pub_position = self.has_position,
                            pub_altitude = self.has_altitude,
                            pub_depth = self.has_depth,
                            pub_pan_tilt = self.has_pan_tilt,
                            msg_if = self.msg_if,
                            node_if = self.node_if
                            )
        # self.status_msg.navpose_topic = self.navpose_pub_if.get_namespace()


    def _setUpdateMaxRateCb(self,msg):
        rate = msg.data
        self.set_update_max_rate(rate)



    # def _setNavPoseFrameCb(self,msg):
    #     frame = msg.data
    #     if frame in nepi_nav.NAVPOSE_3D_FRAME_OPTIONS:
    #         self.navpose_frame = frame
    #     if self.node_if is not None:
    #         self.node_if.set_param(self.node_if_prefix + 'navpose_frame',frame)
    #         self.publish_status()
    #     else:
    #         self.msg_if.pub_warn("Ignoring invalid navpose_frame: " + str(frame) + " not in " + str(nepi_nav.NAVPOSE_3D_FRAME_OPTIONS) )



    def status_query_callback(self, _):
        """Service handler that returns the NPX capabilities report.

        Returns:
            DeviceNPXStatusQueryResponse: The current DeviceNPXStatus in device_status,
                describing which navpose data types this device provides, including
                location, heading, orientation, position, altitude, depth, and pan/tilt,
                as well as supported update rate range and available data products.
        """
        return DeviceNPXStatusQueryResponse(device_status = self.status_msg)


    def _updateNavPoseDictCb(self,timer):
        navpose_dict = None
        if self.getNavPoseCb is not None:
            try:
                navpose_dict = self.getNavPoseCb()
            except Exception as e:
                self.msg_if.pub_warn("getNavPoseCb failed: " + str(e), throttle_s = 5.0 )

        if self.navpose_caps_known == False:
            # The telemetry source was not up when this IF was constructed, so the
            # capability flags were never derived and the NavPose IF was not created.
            # Do both now, off the first read that returns something.
            if navpose_dict is None:
                if self.navpose_wait_warned == False:
                    self.navpose_wait_warned = True
                    self.msg_if.pub_warn("Waiting on NavPose source before creating NavPose IF" )
                else:
                    self.msg_if.pub_debug("Waiting on NavPose source before creating NavPose IF", throttle_s = -1 )
                nepi_sdk.start_timer_process(1.0, self._updateNavPoseDictCb, oneshot = True)
                return
            try:
                self.deriveNavPoseCaps(navpose_dict)
                self.createNavPosePubIf()
            except Exception as e:
                # A source mid-handshake can return a partial dict. Stay unresolved and
                # retry on the next read rather than locking in half-derived flags or
                # letting the exception kill this timer loop for good.
                self.navpose_caps_known = False
                self.msg_if.pub_warn("Failed to create NavPose IF from telemetry, will retry: " + str(e), throttle_s = 10.0 )
                nepi_sdk.start_timer_process(1.0, self._updateNavPoseDictCb, oneshot = True)
                return
            self.msg_if.pub_info("Created NavPose IF from first telemetry at " + str(self.status_msg.navpose_topic) + \
                " with has_location: " + str(self.has_location) + " has_heading: " + str(self.has_heading) + \
                " has_orientation: " + str(self.has_orientation) + " has_position: " + str(self.has_position) + \
                " has_altitude: " + str(self.has_altitude) + " has_depth: " + str(self.has_depth) + \
                " has_pan_tilt: " + str(self.has_pan_tilt) )
            self.publish_status()

        if navpose_dict is None:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        # publish navpose data

        if self.navpose_pub_if is not None:
            # Stamp the operator-selected 3D frame, then apply the per-device mount transform
            navpose_dict['navpose_frame'] = self.navpose_frame
            self.navpose_dict = self.navpose_pub_if.publish_navpose(navpose_dict)

        # # save data if needed
        # timestamp = nepi_utils.get_time()
        # self.save_data_if.save('navpose',self.navpose_dict,timestamp)

        # set up next loop
        delay = float(1) / self.update_rate
        nepi_sdk.start_timer_process(delay, self._updateNavPoseDictCb, oneshot = True)


    def cleanup_actions(self):
        """Performs cleanup actions when the ROS node is shutting down.

        Logs a shutdown message via the message interface. Called by the node shutdown
        hook to allow orderly teardown of the device interface.
        """
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions" )



