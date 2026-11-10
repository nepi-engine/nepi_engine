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
import importlib
import math

os.environ['EGL_PLATFORM'] = 'surfaceless'   # Ubuntu 20.04+
import open3d as o3d

import cv2


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_states
from nepi_sdk import nepi_triggers
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64


from nepi_interfaces.msg import Control, ControlsStatus, SettingsStatus, UpdateControl, MgrSystemStatus
from nepi_interfaces.msg import Datum, DataStatus

from nepi_interfaces.msg import NavPose, NavPoses, NavPoseStatus, NavPosesStatus
from nepi_interfaces.msg import NavPoseLocation, NavPoseHeading
from nepi_interfaces.msg import NavPoseOrientation, NavPosePosition
from nepi_interfaces.msg import NavPoseAltitude, NavPoseDepth
from nepi_interfaces.msg import NavPosePanTilt



from nepi_interfaces.msg import ProcessStatus

from nepi_interfaces.msg import SaveDataRate, SaveDataStatus, FilenameConfig
from nepi_interfaces.srv import SaveDataCapabilitiesQuery, SaveDataCapabilitiesQueryRequest, SaveDataCapabilitiesQueryResponse


from nepi_interfaces.msg import Transform, TransformStatus

from nepi_interfaces.msg import SystemState, SystemStates, SystemStatesStatus
from nepi_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_interfaces.msg import SystemTrigger, SystemTriggers, SystemTriggersStatus

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import  NodeIF
from nepi_api.utils_if import ReadWriteIF


SYSTEM_IF_KEYS = [
    'msg_if',
    'node_if',
    'settings_if',
    'save_data_if',
    'transform_if',
    'navpose_if',
    'states_if',
    'triggers_if'
]



#######################################
# NavPoseIF
#######################################
NAVPOSE_NAV_FRAME_OPTIONS = ['ENU','NED','UKNOWN']
NAVPOSE_ALT_FRAME_OPTIONS = ['WGS84','AMSL','UKNOWN'] # ['WGS84','AMSL','AGL','MSL','HAE','BAROMETER','UKNOWN']
NAVPOSE_DEPTH_FRAME_OPTIONS = ['DEPTH','UKNOWN'] # ['MSL','TOC','DF','KB','DEPTH','UKNOWN']


class NavPoseIF(NodeIF):

    #########################
    ## NODE IF Required
    ## Class Variables Updated by NodeIF Init Class
    #########################

    STATUS_MSG = NavPoseStatus()
    DATA_MSG = NavPose()

    ID = 'navpose'
    DISPLAY_NAME = 'NavPose'
    DESCRIPTION = 'Navigation and Pose Data'

    IF_DICT = dict()

    #########################
    ## CUSTOM
    #########################
    frame_name = None
    data_product = 'navpose'

    msg_if = None
    node_if = None
    save_data_if = None
    transform_if = None

    ready = False

    needs_data = False

    last_pub_time = None
    time_list = [0,0,0,0,0,0,0,0,0,0]

    frame_nav = 'ENU'
    frame_altitude = 'WGS84'
    frame_depth = 'MSL'
    
    data_source_description = 'sensor'
    data_ref_description = 'sensor'

    navpose_msg = NavPose()
    navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    navpose_settings_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_INFO_DICT)
    navpose_frame = 'None'

    data_lock = threading.Lock()
    navpose_dict_lock = threading.Lock()

    pub_msg_dict = dict()
    pub_msg_dict['navpose_pub'] = NavPose
    pub_msg_dict['location_pub'] = NavPoseLocation
    pub_msg_dict['orientation_pub'] = NavPoseOrientation
    pub_msg_dict['position_pub'] = NavPosePosition
    pub_msg_dict['heading_pub'] = NavPoseHeading
    pub_msg_dict['altitude_pub'] = NavPoseAltitude
    pub_msg_dict['depth_pub'] = NavPoseDepth
    pub_msg_dict['pan_tilt_pub'] = NavPosePanTilt
              
    def __init__(self,
                name = 'navpose',
                namespace = None,
                frame_name = None,
                data_product = None,
                data_source_description = 'sensor',
                data_ref_description = 'sensor',
                pub_navpose = True,
                pub_location = False, pub_heading = False,
                pub_orientation = False, pub_position = False,
                pub_altitude = False, pub_depth = False,
                pub_pan_tilt = False,
                info_dict = dict(),
                msg_if = None,
                node_if = None,
                save_data_if = None,
                transform_if = None,
                
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.IF_DICT = dict(self.IF_DICT)
        self.navpose_dict = copy.deepcopy(self.navpose_dict)
        self.navpose_settings_dict = copy.deepcopy(self.navpose_settings_dict)
        self.pub_msg_dict = dict(self.pub_msg_dict)
        self.time_list = copy.deepcopy(self.time_list)
        self.navpose_msg = copy.deepcopy(self.navpose_msg)
        self.data_lock = threading.Lock()
        self.navpose_dict_lock = threading.Lock()

               
        ##############################
        # Update Class Variables
        ##############################

        name = nepi_utils.get_clean_name(name)
        if data_product is not None:
            data_product = nepi_utils.get_clean_name(data_product)
            if data_product.strip() != '':
                self.data_product = data_product


        if frame_name is None:
            frame_name = name
        self.frame_name = frame_name

        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description

        self.pub_navpose = pub_navpose
        self.pub_location = pub_location
        self.pub_heading = pub_heading
        self.pub_orientation = pub_orientation
        self.pub_position = pub_position
        self.pub_altitude = pub_altitude
        self.pub_depth = pub_depth
        self.pub_pan_tilt = pub_pan_tilt


        ##############################  
        # Setup NodeIF Updates
        ##############################  

        #######################
        # Sync System IF inputs to if_dict

        self.msg_if = msg_if
        self.IF_DICT['msg_if'] = msg_if
        self.node_if = node_if
        self.IF_DICT['node_if'] = node_if
        self.save_data_if = save_data_if
        self.IF_DICT['save_data_if'] = save_data_if
        self.transform_if = transform_if
        self.IF_DICT['transform_if'] = transform_if

        ##################
        if_info_dict = dict(
            description = self.DESCRIPTION,
            display_name = self.DISPLAY_NAME,
            type = 'system',
            sub_type = self.ID,
            source_description = data_source_description,
            source_ref_description = data_source_description,
            end_description = data_ref_description,
            end_ref_description = data_ref_description,

            # Custom
            frame_name = frame_name,
        )
        update_info_dict = copy.deepcopy(info_dict)
        update_info_dict.update(if_info_dict)
        
        ##################        
        update_caps_dict = dict(
            has_navpose_pub = self.pub_navpose,
            has_location_pub = self.pub_location,
            has_heading_pub = self.pub_heading,
            has_position_pub = self.pub_position,
            has_orientation_pub = self.pub_orientation,
            has_depth_pub = self.pub_depth,
            has_pan_tilt = self.pub_pan_tilt,
        )
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
        super().__init__(id = self.ID,
                name = name,
                namespace = namespace,
                status_msg = copy.deepcopy(self.STATUS_MSG),
                data_msg = copy.deepcopy(self.DATA_MSG),
                info_dict = update_info_dict,
                caps_dict = update_caps_dict, 
                stats_dict = update_stats_dict, 
                rui_dict = update_rui_dict, 
                config_control_dict = update_config_control_dict, 
                node_controls_dict = update_node_controls_dict, 
                callback_dict = update_callback_dict, 
                if_dict = dict(self.IF_DICT)
                )

        ready = self.wait_for_ready()


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
        pubs_dict = {}
        if self.pub_navpose == True:
                pubs_dict['navpose_pub'] = {
                    'msg': NavPose,
                    'namespace': self.namespace,
                    'topic': '',
                    'qsize': 1,
                    'latch': False
                }
    
        if self.pub_location == True:
            pubs_dict['location_pub'] = {
                'msg': NavPoseLocation,
                'namespace': self.namespace,
                'topic': 'location',
                'qsize': 1,
                'latch': False
            }
            
        if self.pub_orientation == True:
            pubs_dict['orientation_pub'] = {
                'msg': NavPoseOrientation,
                'namespace': self.namespace,
                'topic': 'orientation',
                'qsize': 1,
                'latch': False
            }

        if self.pub_position == True:
            pubs_dict['position_pub'] = {
                'msg': NavPosePosition,
                'namespace': self.namespace,
                'topic': 'position',
                'qsize': 1,
                'latch': False
            }

        if self.pub_heading == True:
            pubs_dict['heading_pub'] = {
                'msg': NavPoseHeading,
                'namespace': self.namespace,
                'topic': 'heading',
                'qsize': 1,
                'latch': False
            }

        if self.pub_altitude == True:
            pubs_dict['altitude_pub'] = {
                'msg': NavPoseAltitude,
                'namespace': self.namespace,
                'topic': 'altitude',
                'qsize': 1,
                'latch': False
            }

        if self.pub_depth == True:
            pubs_dict['depth_pub'] = {
                'msg': NavPoseDepth,
                'namespace': self.namespace,
                'topic': 'depth',
                'qsize': 1,
                'latch': False
            }

        if self.pub_pan_tilt == True:
            pubs_dict['pan_tilt_pub'] = {
                'msg': NavPosePanTilt,
                'namespace': self.namespace,
                'topic': 'pan_tilt',
                'qsize': 1,
                'latch': False
            }


        self.add_pubs(pubs_dict)

        #######################
        # Add Subs 
        #######################
        subs_dict =  {}
        self.add_subs(subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################

        ####################
        # Setup Save Data IF Class 
        ####################
        if self.save_data_if is not None:
            data_products = self.save_data_if.get_data_products()
            if self.data_product not in data_products:
                self.save_data_if.register_data_product(self.data_product)



        #########################
        # Update status message for values not in NodeIF Dictionaries
        ##############################
        self.status_msg.frame_name = self.frame_name
        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description


        ##############################
        # Start Node Processes
        ##############################
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete" )
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is NavPoseIF:
            self.start()

    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):

        avg_rate = 0
        if len(self.time_list) > 0:
            avg_time = sum(self.time_list) / len(self.time_list)
            if avg_time > .01:
                avg_rate = float(1) / avg_time
        status_msg.avg_pub_rate = avg_rate

        return status_msg


    
    def get_navpose_msg(self):
        """Return a deep copy of the most recently received nav pose dictionary.

        Mirrors data_if.NavPoseIF.get_navpose_dict, which is the contract the
        data_if image, depth map and pointcloud classes call against whichever
        navpose interface they hold -- a server-side NavPoseIF when one is passed
        in, or a ConnectNavPoseIF built from a namespace when one is not.

        Returns:
            dict: The last nav pose data dictionary, or None if no NavPose message
                has arrived from the connected source yet.
        """
        if self.navpose_msg is None:
            return None
        self.data_lock.acquire()
        navpose_msg = copy.deepcopy(self.navpose_msg)
        self.data_lock.release()
        if navpose_msg is None:
            navpose_msg = NavPose()
  
        return navpose_msg

    def get_navpose_dict(self):
        """Return a deep copy of the most recently published nav pose dictionary.

        Returns:
            dict: The last nav pose data dictionary.
        """
        self.navpose_dict_lock.acquire()
        navpose_dict = copy.deepcopy(self.navpose_dict)
        self.navpose_dict_lock.release()
        if navpose_dict is None:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return navpose_dict



    ###############################
    # Class Public Methods
    ###############################

    def get_frame_nav_options(self):
        """Return the list of supported navigation frame identifiers.

        Returns:
            list: Supported nav frame strings (e.g. 'ENU', 'NED').
        """
        return NAVPOSE_NAV_FRAME_OPTIONS

    def get_frame_altitude_options(self):
        """Return the list of supported altitude frame identifiers.

        Returns:
            list: Supported altitude frame strings (e.g. 'WGS84', 'AMSL').
        """
        return NAVPOSE_ALT_FRAME_OPTIONS

    def get_frame_depth_options(self):
        """Return the list of supported depth frame identifiers.

        Returns:
            list: Supported depth frame strings (e.g. 'DEPTH').
        """
        return NAVPOSE_DEPTH_FRAME_OPTIONS



    def get_blank_navpose_dict(self):
        """Return a deep copy of the blank nav pose dictionary template.

        Returns:
            dict: A blank nav pose dictionary with all fields at default values.
        """
        blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return blank_navpose_dict


    def needs_data_check(self):
        """Return whether downstream consumers currently need nav pose data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        needs_data = copy.deepcopy(self.needs_data)
        # self.msg_if.pub_debug("Returning: " + self.namespace + " " "needs data: " + str(needs_data) , throttle_s = 5.0)
        return needs_data

    # Update System Status
    def publish_navpose(self,navpose_dict,
                        timestamp = None,
                        ):
        """Publish a nav pose data dictionary to all configured ROS topics.

        Converts the input dictionary to NEPI standard frames (ENU / WGS84),
        publishes individual component messages (location, heading, orientation,
        position, altitude, depth, pan/tilt) if enabled, publishes the combined
        NavPose message, and triggers data
        saving if a SaveDataIF is registered.

        Args:
            navpose_dict (dict): Nav pose data following the NEPI navpose dict schema.
            timestamp (float or rospy.Time, optional): Acquisition timestamp in seconds
                or as a ROS Time object. Defaults to current time if None.
          

        Returns:
            dict: The processed nav pose dictionary in NEPI standard frames.
        """
        np_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        if navpose_dict is None and self.status_msg is not None:
            return np_dict
        else:
            # Initialize np_dict here so it's available in both branches
            for key in np_dict.keys():
                if key in navpose_dict.keys():
                    np_dict[key] = navpose_dict[key]


            #self.msg_if.pub_debug("Start Navpose data dict: " + str(np_dict) , throttle_s = 30)

            if timestamp == None:
                timestamp = nepi_utils.get_time()
            else:
                timestamp = nepi_sdk.sec_from_timestamp(timestamp)

            current_time = nepi_utils.get_time()
            get_latency = (current_time - timestamp)
            #self.msg_if.pub_debug("Get Data Latency: {:.2f}".format(get_latency) , throttle_s = 30)

            # Start Img Pub Process
            start_time = nepi_utils.get_time()   

            ###################################
            # # Transform navpose data frames to nepi standard frames
            # if np_dict['frame_nav'] != 'ENU':
            #     if np_dict['frame_nav'] == 'NED':
            #         nepi_nav.convert_navpose_ned2enu(np_dict)
            # if np_dict['frame_altitude'] != 'WGS84':
            #     if np_dict['frame_altitude'] == 'AMSL':
            #         nepi_nav.convert_navpose_amsl2wgs84(np_dict)
            # if np_dict['frame_depth'] != 'MSL':
            #     if np_dict['frame_depth'] == 'DEPTH':
            #         pass # need to add conversions     

            ###################################
            # Transform navpose data frames to system set frames
            frame_nav = self.navpose_settings_dict['frame_nav']
            frame_alt = self.navpose_settings_dict['frame_alt']
            frame_depth = self.navpose_settings_dict['frame_depth']
            
            if np_dict['frame_nav'] != frame_nav:
                if np_dict['frame_nav'] == 'NED' and frame_nav == 'ENU':
                    nepi_nav.convert_navpose_ned2enu(np_dict)
                elif np_dict['frame_nav'] == 'ENU' and frame_nav == 'NED':
                    nepi_nav.convert_navpose_enu2ned(np_dict)
            if np_dict['frame_altitude'] != frame_alt:
                if np_dict['frame_altitude'] == 'AMSL' and frame_alt ==  'WGS84':
                    nepi_nav.convert_navpose_amsl2wgs84(np_dict)
                elif np_dict['frame_altitude'] == 'WGS84' and frame_alt ==  'AMSL':
                    nepi_nav.convert_navpose_wgs842amsl(np_dict)
            #if np_dict['frame_depth'] != 'MSL':
            #    if np_dict['frame_depth'] == 'DEPTH':
            #        pass # need to add conversions


            ###################################
            # Transform in Frame
            tnp_dict = np_dict
            if self.transform_if is not None:
                transform_dict = self.transform_if.get_transform_dict()
                if transform_dict is not None:
                    tnp_dict = nepi_nav.transform_navpose_dict(np_dict, transform_dict)
            if tnp_dict is None:
                self.msg_if.pub_debug("Failed to tranform navpose dict" , throttle_s = 30)
            else:
                np_dict = tnp_dict



            ###################################
            # Update Class Data and Status Msg     
            ###################################          
            self.navpose_dict  = np_dict

            self.status_msg.frame_nav = np_dict['frame_nav']
            self.status_msg.frame_altitude = np_dict['frame_altitude']
            self.status_msg.frame_depth = np_dict['frame_depth']

            self.status_msg.frame_name = self.frame_name #np_dict['navpose_frame']
            self.status_msg.frame_nav = np_dict['frame_nav']
            self.status_msg.frame_altitude = np_dict['frame_altitude']
            self.status_msg.frame_depth = np_dict['frame_depth']


            ###################################
            # Pub NavPose Sub Topics               
            ####################################
            
            # Publish nav pose subs
            if self.pub_location == True:
                pub_name = 'location_pub'
                msg = self.pub_msg_dict[pub_name]()
                # gps_fix pub
                msg.timestamp = np_dict['time_location']
                msg.latitude = np_dict['latitude']
                msg.longitude = np_dict['longitude']
                msg.location_m_per_sec = np_dict['location_m_per_sec']
                self.publish_msg(pub_name,msg)

            if self.pub_heading == True:
                pub_name = 'heading_pub'
                msg = self.pub_msg_dict[pub_name]()
                # gps_fix pub
                msg.timestamp = np_dict['time_heading']
                msg.heading_deg = np_dict['heading_deg']
                msg.heading_m_per_sec = np_dict['heading_m_per_sec']
                self.publish_msg(pub_name,msg)

            if self.pub_orientation == True:
                pub_name = 'orientation_pub'
                msg = self.pub_msg_dict[pub_name]()
                # gps_fix pub
                msg.timestamp = np_dict['time_orientation']
                msg.roll_deg = np_dict['roll_deg']
                msg.pitch_deg = np_dict['pitch_deg']
                msg.yaw_deg = np_dict['yaw_deg']
                msg.roll_deg_per_sec = np_dict['roll_deg_per_sec']
                msg.pitch_deg_per_sec = np_dict['pitch_deg_per_sec']
                msg.yaw_deg_per_sec = np_dict['yaw_deg_per_sec']
                self.publish_msg(pub_name,msg)

            if self.pub_position == True:
                pub_name = 'position_pub'
                msg = self.pub_msg_dict[pub_name]()
                # gps_fix pub
                msg.timestamp = np_dict['time_position']
                msg.x_m = np_dict['x_m']
                msg.y_m = np_dict['y_m']
                msg.z_m = np_dict['z_m']
                # The component msg has no velocity flag, so -999 is the flag:
                # nepi_nav.update_navpose_dict_from_msg reads it back that way.
                msg.x_m_per_sec = np_dict['x_m_per_sec'] if np_dict['has_velocity'] else -999
                msg.y_m_per_sec = np_dict['y_m_per_sec'] if np_dict['has_velocity'] else -999
                msg.z_m_per_sec = np_dict['z_m_per_sec'] if np_dict['has_velocity'] else -999
                self.publish_msg(pub_name,msg)

            if self.pub_altitude == True:
                pub_name = 'altitude_pub'
                msg = self.pub_msg_dict[pub_name]()
                # gps_fix pub
                msg.timestamp = np_dict['time_altitude']
                msg.altitude_m = np_dict['altitude_m']
                msg.altitude_m_per_sec = np_dict['altitude_m_per_sec']
                self.publish_msg(pub_name,msg)

            if self.pub_depth == True:
                pub_name = 'depth_pub'
                msg = self.pub_msg_dict[pub_name]()
                # gps_fix pub
                msg.timestamp = np_dict['time_depth']
                msg.depth_m = np_dict['depth_m']
                msg.depth_m_per_sec = np_dict['depth_m_per_sec']
                self.publish_msg(pub_name,msg)

            if self.pub_pan_tilt == True:
                pub_name = 'pan_tilt_pub'
                msg = self.pub_msg_dict[pub_name]()
                # gps_fix pub
                msg.timestamp = np_dict['time_depth']
                msg.pan_deg = np_dict['pan_deg']
                msg.tilt_deg = np_dict['tilt_deg']
                self.publish_msg(pub_name,msg)

            ###################################
            # Pub NavPose Topic            
            ####################################

            if self.pub_navpose == True:
                navpose_msg = None
                try:
                    navpose_msg = nepi_nav.convert_navpose_dict2msg(np_dict)
                except Exception as e:
                    self.msg_if.pub_debug("Failed to convert navpose data to msg: " + str(e) , throttle_s = 5.0)
                    success = False

                if navpose_msg is not None:
                    try:
                        self.navpose_msg = navpose_msg
                        self.publish_msg('navpose_pub', navpose_msg)
                    except Exception as e:
                        self.msg_if.pub_debug("Failed to publish navpose data msg: " + str(e) , throttle_s = 5.0)
                        success = False


            ###################################
            # Get Stats
            ####################################

            current_time = nepi_utils.get_time()
            pub_latency = (current_time - timestamp)
            process_time = (current_time - start_time)
            #self.msg_if.pub_debug("Get Data Latency: {:.2f}".format(pub_latency) , throttle_s = 30)

            # Update Pub Stats
            if self.last_pub_time is None:
                pub_time_sec = 1.0
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time


            self.time_list.pop(0)
            self.time_list.append(pub_time_sec)


            ###################################
            # Save Data    
            ####################################

            if self.save_data_if is not None:
                save_enabled = self.save_data_if.data_product_save_enabled('navpose') == True
                should_save = self.save_data_if.data_product_should_save('navpose') == True
                snapshot_enabled = self.save_data_if.data_product_snapshot_enabled('navpose') == True
                save_navpose = should_save or snapshot_enabled
                time_ns = nepi_utils.get_time()
                key_name = int(math.floor(time_ns * 1000))
                navposes_save_dict = {key_name: np_dict}
                if self.save_data_if is not None and len(list(navposes_save_dict.keys())) > 0 and save_navpose == True:
                    filename = self.save_data_if.save('navposes', navposes_save_dict, timestamp = time_ns, filename = self.save_filename, key_name = key_name)
                    if save_enabled == False:
                        filename = None
                    self.save_filename = filename

        return np_dict




    ###############################
    # Class Private Methods
    ###############################


    def _needsDataCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers(self.node_if_prefix + 'navpose_pub')
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
            needs_data = has_subs or needs_save or needs_snapshot
        else:
            needs_data = has_subs
        self.needs_data = needs_data
        #self.msg_if.pub_debug("Needs Data Check End: " + self.namespace + " : " + str([has_subs,needs_save, needs_snapshot]) )
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)








#######################################
# SettingsIF
#######################################

# The device-side settings contract, unchanged by the move onto nepi_controls.
# A driver (or system_mgr, for the system config) describes its settings as
# string-valued dicts and hands SettingsIF two functions to read and write them.
# SettingsIF converts that form to and from a nepi_controls controls dict, which
# is what it publishes -- so the capability report that used to require a
# SettingsCapabilitiesQuery round trip now rides the status message.


def SET_NONE_SETTINGS_FUNCTION(setting):
    return False, "No settings update function available", dict()

def GET_NONE_SETTINGS_FUNCTION():
    return dict()


class SettingsIF(NodeIF):
    """Publishes a node's settings as a nepi_settings settings set.

    A device's settings are settings: named, typed, bounded values with a
    factory, default and set tier. This interface keeps the string-valued
    settings dict contract that drivers implement, holds the live state as a
    nepi_settings settings dict, and publishes it as a SettingsStatus message
    on '<namespace>/settings/status'. The capability information that the
    retired SettingsCapabilitiesQuery service used to return (type, options,
    bounds, default) is carried in that status message.

    ROS interface, all under '<namespace>/<settings_name>':
        status              (SettingsStatus, latched)  the settings and their capabilities
        update_setting_value      (UpdateSetting)            change one setting
        reset_settings      (Empty)                    restore last-saved values
    """


    #########################
    ## NODE IF Required
    ## Class Variables Updated by NodeIF Init Class
    #########################

    STATUS_MSG = SettingsStatus()
    DATA_MSG = None

    ID = 'settings'
    DISPLAY_NAME = 'Settings'
    DESCRIPTION = 'Settings'

    IF_DICT = dict()

    #########################
    ## CUSTOM
    #########################
    msg_if = None
    node_if = None

    ready = False
 
    settings_dict_values = None
    getCapSettingsFunction = None
    getSettingsFunction = None
    setSettingFunction = None
    callback_arg = None

    settings_dict = dict()
    settings_status_msg = None

    settings_name = 'settings'


    save_params = True

    #######################
    ### IF Initialization
    def __init__(self, 
                name = 'settings',
                namespace = None,
                settings_name = None,
                getSettingsFunction=None, 
                setSettingFunction=None, 
                callback_arg = None,
                save_params = True,
                info_dict = dict(),
                msg_if = None,
                node_if = None,
                
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.IF_DICT = dict(self.IF_DICT)

        ##############################
        # Update Class Variables
        ##############################

        if settings_name is not None:
            name = settings_name

        self.callback_arg = callback_arg
        self.save_params = save_params

        if getSettingsFunction is None:
            self.getSettingsFunction = GET_NONE_SETTINGS_FUNCTION
        else:
            self.getSettingsFunction = getSettingsFunction

        if setSettingFunction is None:
            self.setSettingFunction = SET_NONE_SETTINGS_FUNCTION
        else:
            self.setSettingFunction = setSettingFunction
        
        # Build the settings dict. The factory and default tiers come from the
        # device's factory settings (or a cap setting's own declared default);
        # the set tier is seeded from whatever the device currently reports.

        if self.callback_arg is None:
            settings_dict = self.getSettingsFunction()
        else:
            settings_dict = self.getSettingsFunction(self.callback_arg)

        if settings_dict is not None:
            self.settings_dict = settings_dict
            #self.msg_if.pub_debug("Get Settings function returned Settings Dict: " + str(settings_dict) )


        ##############################  
        # Setup NodeIF Updates
        ##############################  

        #######################
        # Sync System IF inputs to if_dict

        self.msg_if = msg_if
        self.IF_DICT['msg_if'] = msg_if
        self.node_if = node_if
        self.IF_DICT['node_if'] = node_if


        ##################
        if_info_dict = dict(
            description = self.DESCRIPTION,
            display_name = self.DISPLAY_NAME,
            type = 'system'
        )
        update_info_dict = copy.deepcopy(info_dict)
        update_info_dict.update(if_info_dict)
        ##################        
        update_caps_dict = dict()
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
        super().__init__(id = self.ID,
                name = name,
                namespace = namespace,
                status_msg = copy.deepcopy(self.STATUS_MSG),
                data_msg = copy.deepcopy(self.DATA_MSG),
                info_dict = update_info_dict,
                caps_dict = update_caps_dict, 
                stats_dict = update_stats_dict, 
                rui_dict = update_rui_dict, 
                config_control_dict = update_config_control_dict, 
                node_controls_dict = update_node_controls_dict, 
                callback_dict = update_callback_dict, 
                if_dict = dict(self.IF_DICT)
                )

        ready = self.wait_for_ready()


       #######################
        # Add Services
        #######################
        if self.save_params == True:
            # Empty, not None: add_params skips a param whose factory value is None, so
            # saved settings would never register, persist or restore
            params_dict = {
                'settings': {
                    'name': 'settings',
                    'namespace': self.namespace,
                    'factory_val': dict()
                }
            }
        else:
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
        pubs_dict = {}

        self.add_pubs(pubs_dict)

        #######################
        # Add Subs 
        #######################
        subs_dict =  {
            'update_setting': {
                'msg': UpdateControl,
                'namespace': self.namespace,
                'topic': 'update_setting',
                'qsize': 5,
                'callback': self._updateSettingCb
            },
            'reset_settings': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset_settings',
                'qsize': 5,
                'callback': self._resetSettingsCb,
                'callback_args': None
            }
        }
        self.add_subs(subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################

  
        #########################
        # Update status message for values not in NodeIF Dictionaries
        ##############################
        self.status_msg = nepi_settings.create_status_msg(
                                            self.settings_name, 'Settings', 'Device Settings')


        ##############################
        # Start Node Processes
        ##############################


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is SettingsIF:
            self.start()



    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):

        """Build and publish the settings SettingsStatus message.

        Refreshes the settings dict from the device (its live values, and its
        capability report if it provides a getCapSettingsFunction), then
        publishes. No-ops if node_if is None.
        """

        settings_dict = self.getSettingsDict()
        status_msg = nepi_settings.update_status_msg(status_msg, settings_dict)

        return status_msg

    def init(self, do_updates = True):
        """Load settings from the ROS param server and optionally apply them.

        Reads the persisted string-valued settings dict from the param server,
        seeds the settings dict default and set tiers from it, and if
        do_updates is True pushes each stored value down to the device before a
        final status publish.

        Args:
            do_updates (bool, optional): If True, apply all stored settings to the
                hardware after loading. Defaults to True.
        """
        settings_params_dict = nepi_controls.get_params_dict(self.settings_dict)
        if self.node_if is not None and self.save_params == True:
            settings_params_dict = self.get_param('settings')            
        #self.msg_if.pub_debug("Init start settings params: " + str(settings_params_dict) )

        if self.callback_arg is None:
            init_settings_dict = self.getSettingsFunction()
        else:
            init_settings_dict = self.getSettingsFunction(self.callback_arg)

        #self.msg_if.pub_debug("Init start settings dict: " + str(init_settings_dict) )
        if type(init_settings_dict) == dict:
            if type(settings_params_dict) == dict:
                for setting_name in settings_params_dict.keys():
                    if setting_name in init_settings_dict.keys():
                        setting_value = settings_params_dict[setting_name]
                        # set_value's fourth parameter is `index`, not a second
                        # copy of the value: passing setting_value twice used the
                        # value as an array index into itself.
                        init_settings_dict = nepi_controls.set_value(init_settings_dict, setting_name, setting_value)
        else:
            init_settings_dict = dict()

        self.settings_dict = init_settings_dict


        #self.msg_if.pub_debug("Init update settings dict: " + str(init_settings_dict) )

        if do_updates == True:
            settings_dict = self.getSettingsDict()
            for setting_name in settings_dict.keys():
                setting_value = nepi_controls.get_value(settings_dict, setting_name)
                try:
                    if self.callback_arg is None:
                        [success, msg, settings_dict] = self.setSettingFunction(setting_name, setting_value)
                    else:
                        [success, msg, settings_dict] = self.setSettingFunction(setting_name, setting_value, self.callback_arg)
                    if settings_dict is not None:
                        self.settings_dict = settings_dict
                        #self.msg_if.pub_debug("Init Got Settings update: " + str(settings_dict[setting_name]) )
                    else:
                        self.msg_if.pub_debug("Init Settings update function returned None Settings Dict" )

                except Exception as e:
                    self.msg_if.pub_debug("Init setSettingFunction callback failed: " + str(e) )
                    success = False
                    msg = str(e)
            self.save_params_dict()
            init_settings_values_dict = nepi_controls.get_values_dict(self.settings_dict)
            self.msg_if.pub_debug("Init Settings Complete: " + str(init_settings_values_dict) )
        self.publish_status()

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def resetCb(self, do_updates = True):
        self.settings_dict = nepi_controls.reset_values(self.settings_dict)

    def factoryResetCb(self, do_updates = True):
        self.settings_dict = nepi_controls.reset_values(self.settings_dict)

    ###############################
    # Class Public Methods
    ###############################

    def get_settings_dict(self):
        """Return the current settings in the string-valued device settings form.

        Returns:
            dict: setting name -> {'name', 'type', 'value'}.
        """
        return nepi_controls.get_values_dict(self.getSettingsDict())

    def get_status_pub_msg(self):
        return self.get_status_msg()



    def update_setting_value(self,setting_name, setting_value, index = None):
        """Apply a single setting update using the registered setSettingFunction.

        Compares the incoming value against the value currently held in the
        settings dict and only calls down to the device if it has changed.
        Optionally persists the settings dict to the ROS param server and
        publishes an updated status message.

        Args:
            setting (dict): Setting dictionary with 'name', 'type', and 'value' keys.
            do_updates (bool, optional): If True, publish status after a successful
                update. Defaults to True.
            update_param (bool, optional): If True, write the updated settings to the
                ROS param server. Defaults to True.

        Returns:
            bool: True if the setting was successfully applied, False otherwise.
        """
        success = False
        
        if self.setSettingFunction is None:
            self.msg_if.pub_debug("Settings updates ignored. No settings update function defined " )
            return success

        settings_dict = self.getSettingsDict()
        if setting_name not in settings_dict.keys():
            self.msg_if.pub_debug("Ignoring update for unknown setting: " + str(setting_name) )
            return

        if nepi_controls.get_clean_value(settings_dict, setting_name, setting_value) is None:
            self.msg_if.pub_debug("Setting update rejected as invalid: " + str([setting_name, setting_value, index]) )

        current_value = nepi_controls.get_value(self.settings_dict, setting_name)
        if current_value == setting_value:
            #self.msg_if.pub_debug("Setting allready set: " + str([current_value, setting_value]) )
            return True

        self.msg_if.pub_debug("Updating setting : " + str([setting_name,setting_value]) )
        try:
            if self.callback_arg is None:
                [success, msg, settings_dict] = self.setSettingFunction(setting_name, setting_value)
            else:
                [success, msg, settings_dict] = self.setSettingFunction(setting_name, setting_value, self.callback_arg)
        except Exception as e:
            self.msg_if.pub_debug("setSettingFunction callback failed: " + str(e) )
            success = False
            msg = str(e)
        if settings_dict is not None:
            self.settings_dict = settings_dict
        else:
            self.msg_if.pub_debug("Setting update function returned None Settings Dict" )
        #self.msg_if.pub_debug("setSettingFunction returned: " + str(self.settings_dict) )
 
        self.publish_status()
        if success == True:
            self.msg_if.pub_debug("Setting Updated: " + str([setting_name, setting_value]) )
            self.save_params_dict()
        else:
            self.msg_if.pub_debug("Setting update failed: " + str([setting_name, setting_value]) + " : " + str(msg) )
        return success





    ###############################
    # Class Private Methods
    ###############################




    def getSettingsDict(self):
        return copy.deepcopy(self.settings_dict)


    def save_params_dict(self):
        settings_dict = copy.deepcopy(self.settings_dict)
        params_dict = nepi_controls.get_params_dict(settings_dict)
        if self.node_if is not None and self.save_params == True and settings_dict is not None:
            self.set_value('settings', params_dict)


    def _resetSettingsCb(self, msg):
        self.reset()

    def _updateSettingCb(self,msg):
            #self.msg_if.pub_debug("Received setting update msg: " + str(msg) )
            setting_name = msg.name
            settings_dict = copy.deepcopy(self.settings_dict)
            cur_value = nepi_controls.get_value(settings_dict, setting_name )
            settings_dict = nepi_controls.apply_update_msg(settings_dict, msg)
            setting_value = nepi_controls.get_value(settings_dict, setting_name )
            #self.msg_if.pub_debug("Sending Updated Val from/to: " + str([cur_value, setting_value]) )
            self.update_setting_value(setting_name, setting_value)







################################################
## SaveDataIF

FALLBACK_DATA_FOLDER = '/mnt/nepi_storage/data'

SUPPORTED_DICT_FILE_TYPES = ['yaml']
SUPPORTED_IMG_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']  
SUPPORTED_PC_FILE_TYPES = ['pcd']
SUPPORTED_VID_FILE_TYPES = ['avi','AVI']

SUPPORTED_DATA_TYPES = ['dict','cv2_image','o3d_pointcloud']

'''
EXAMPLE_RATE_DICT = {
    'data_product_1' : [save_rate_hz, last_time, max_rate],
    'data_product_2' : [save_rate_hz, last_time, max_rate]
}
'''

EXAMPLE_FILENAME_DICT = {
    'prefix': "", 
    'subfolder': "",
    'add_timestamp': True, 
    'use_utc_tz': True,
    'add_ms': True,
    'add_us': False,
    'add_tz': True,
    'add_node_name': False
    }

class SaveDataIF(NodeIF):


    #########################
    ## NODE IF Required
    ## Class Variables Updated by NodeIF Init Class
    #########################

    STATUS_MSG = SaveDataStatus()
    DATA_MSG = None

    ID = 'save_data'
    DISPLAY_NAME = 'Save Data'
    DESCRIPTION = 'Save Data'

    IF_DICT = dict()

    #########################
    ## CUSTOM
    #########################
    msg_if = None
    node_if = None

    ready = False

    DEFAULT_TIMEZONE = 'UTC'

    DATA_UID = 'nepi'
    DATA_GID = 'nepi'

    read_write_if = None
 
    snapshot_dict = dict()

    sys_mgr_if = None
    read_write_if = None
    all_save_namespace = None

    save_data_root_directory = FALLBACK_DATA_FOLDER

    filename_dict = {
        'prefix': "", 
        'subfolder': "", 
        'add_timestamp': True, 
        'use_utc_tz': True,
        'add_ms': True,
        'add_us': False,
        'add_tz': True,
        'add_node_name': True
        }

    save_rate_dict = dict()
    save_data = False
    use_utc_tz = False

    disabled = False
    save_all_enabled = False
    save_all_rate = 0.0

    space_available = False


    file_prefix = ""
    subfolder = ""

    was_saving = False


    ### IF Initialization
    def __init__(self, 
                name = 'save_data',
                namespace = None,
                save_data_name = None,
                data_products = [], 
                factory_rate_dict = None, 
                factory_filename_dict = None, 
                ignore_global_rate_updates = False,
                info_dict = dict(),
                msg_if = None,
                node_if = None,
                
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.IF_DICT = dict(self.IF_DICT)
        self.snapshot_dict = copy.deepcopy(self.snapshot_dict)
        self.filename_dict = copy.deepcopy(self.filename_dict)
        self.save_rate_dict = copy.deepcopy(self.save_rate_dict)
    
        ##############################
        # Update Class Variables
        ##############################

        name = nepi_utils.get_clean_name(name)
        if save_data_name is not None:
            save_data_name = nepi_utils.get_clean_name(save_data_name)
            if save_data_name.strip() != '':
                name = save_data_name


        tzd = nepi_utils.get_timezone_description(self.DEFAULT_TIMEZONE)
        self.timezone = tzd

    


        ##############################  
        # Setup NodeIF Updates
        ##############################  

        #######################
        # Sync System IF inputs to if_dict

        self.msg_if = msg_if
        self.IF_DICT['msg_if'] = msg_if
        self.node_if = node_if
        self.IF_DICT['node_if'] = node_if


        ##################
        if_info_dict = dict(
            description = self.DESCRIPTION,
            display_name = self.DISPLAY_NAME,
            type = 'system'
        )
        update_info_dict = copy.deepcopy(info_dict)
        update_info_dict.update(if_info_dict)
        ##################        
        update_caps_dict = dict()
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
        super().__init__(id = self.ID,
                name = name,
                namespace = namespace,
                status_msg = copy.deepcopy(self.STATUS_MSG),
                data_msg = copy.deepcopy(self.DATA_MSG),
                info_dict = update_info_dict,
                caps_dict = update_caps_dict, 
                stats_dict = update_stats_dict, 
                rui_dict = update_rui_dict, 
                config_control_dict = update_config_control_dict, 
                node_controls_dict = update_node_controls_dict, 
                callback_dict = update_callback_dict, 
                if_dict = dict(self.IF_DICT)
                )

        ready = self.wait_for_ready()

        all_save_namespace = nepi_sdk.create_namespace(self.base_namespace,'/save_data')
        if all_save_namespace != self.namespace:
            self.all_save_namespace = all_save_namespace


        # Config initial data products dict. Built before 'save_rate_dict' is
        # registered below, so its factory value holds every product's default rate.
        self.msg_if.pub_debug("^^^^^^^^^^^^^^^^^^^^^^" )
        self.msg_if.pub_debug("Starting Save_Data_IF with data products: " + str(data_products) )
        self.msg_if.pub_debug("Starting Save_Data_IF with rate dict: " + str(factory_rate_dict) )
        save_rate_dict = dict()
        save_rate = 0.0
        last_time = 0.0
        max_rate = 100
        for data_product in data_products:
            save_rate = 0.0
            last_time = 0.0
            max_rate = 100
            save_rate_entry = [save_rate, last_time, max_rate]
            if factory_rate_dict is not None:
                if data_product in factory_rate_dict.keys():
                    save_rate = factory_rate_dict[data_product][0]
            save_rate_entry[0] = save_rate
            save_rate_dict[data_product] = save_rate_entry
            self.snapshot_dict[data_product] = False
        self.save_rate_dict = save_rate_dict
        self.msg_if.pub_debug("Got defualt data rate dict: " + str(self.save_rate_dict) )

       #######################
        # Add Services
        #######################
       
        params_dict = {            
            'save_rate_dict': {
                'name': 'save_rate_dict',
                'namespace': self.namespace,
                'factory_val': self.save_rate_dict
            },
            'filename_dict': {
                'name': 'filename_dict',
                'namespace': self.namespace,
                'factory_val': self.filename_dict
            }
        }
        self.add_params(params_dict)

        #######################
        # Add Services
        #######################
        services_dict = {
            'save_data_capabilities_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': SaveDataCapabilitiesQuery,
                'req': SaveDataCapabilitiesQueryRequest(),
                'resp': SaveDataCapabilitiesQueryResponse(),
                'callback': self._capabilitiesHandler
            }
        }
        self.add_services(services_dict)

        #######################
        # Add Pubs 
        #######################
        pubs_dict = {}
        self.add_pubs(pubs_dict)

        #######################
        # Add Subs 
        #######################
        subs_dict = {
            'save_data_disable': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'disable',
                'qsize': 5,
                'callback': self._disableCb, 
                'callback_args': ()
            }, 
            'save': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'save_data_enable',
                'qsize': 5,
                'callback': self._saveEnableCb, 
                'callback_args': ()
            },  
            'prefix': {
                'namespace': self.namespace,
                'msg': String,
                'topic': 'save_data_prefix',
                'qsize': 5,
                'callback': self._setPrefixCb, 
                'callback_args': ()
            },
            'subfolder': {
                'namespace': self.namespace,
                'msg': String,
                'topic': 'save_data_subfolder',
                'qsize': 5,
                'callback': self._setSubfolderCb, 
                'callback_args': ()
            },
            'save_data_utc': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'save_data_utc',
                'qsize': 5,
                'callback': self._setLocalTzCb, 
                'callback_args': ()
            },
            'filename': {
                'namespace': self.namespace,
                'msg': FilenameConfig,
                'topic': 'filename_config',
                'qsize': 5,
                'callback': self._setFilenameCb, 
                'callback_args': ()
            },
            'rate': {
                'namespace': self.namespace,
                'msg': SaveDataRate,
                'topic': 'save_data_rate',
                'qsize': 5,
                'callback': self._saveRateCb, 
                'callback_args': ()
            },          
            'snapshot': {
                'namespace': self.namespace,
                'msg': Empty,
                'topic': 'snapshot_trigger',
                'qsize': 5,
                'callback': self._snapshotCb,  
                'callback_args': ()
            },
            'save_data_reset': {
                'namespace': self.namespace,
                'msg': Empty,
                'topic': 'reset_save_data',
                'qsize': 5,
                'callback': self._resetSaveDataCb,  
                'callback_args': ()
            },

        }

        if self.all_save_namespace is not None:
            ALL_SUBS_DICT =  {
                'disable_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Bool,
                    'topic': 'saving_disable',
                    'qsize': 5,
                    'callback': self._disableCb, 
                    'callback_args': ()
                },
                'save_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Bool,
                    'topic': 'save_data_enable',
                    'qsize': 5,
                    'callback': self._saveEnableCb, 
                    'callback_args': ()
                },
                'prefix_all': {
                    'namespace': self.all_save_namespace,
                    'msg': String,
                    'topic': 'save_data_prefix',
                    'qsize': 5,
                    'callback': self._setPrefixCb, 
                    'callback_args': ()
                },
                'subfolder_all': {
                    'namespace': self.all_save_namespace,
                    'msg': String,
                    'topic': 'save_data_subfolder',
                    'qsize': 5,
                    'callback': self._setSubfolderCb, 
                    'callback_args': ()
                },
                'use_local_tz_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Bool,
                    'topic': 'save_data_utc',
                    'qsize': 5,
                    'callback': self._setLocalTzCb, 
                    'callback_args': ()
                },
                'filename_all': {
                    'namespace': self.all_save_namespace,
                    'msg': FilenameConfig,
                    'topic': 'filename_config',
                    'qsize': 5,
                    'callback': self._setFilenameCb,
                    'callback_args': ()
                },
                'snapshot_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Empty,
                    'topic': 'snapshot_trigger',
                    'qsize': 5,
                    'callback': self._snapshotCb, 
                    'callback_args': ()
                },
                'save_all_config': {
                    'namespace': self.all_save_namespace,
                    'msg': Empty,
                    'topic': 'save_config',
                    'qsize': 5,
                    'callback': self._saveConfigCb, 
                    'callback_args': ()
                },
                'all_save_sub': {
                    'namespace': self.all_save_namespace,
                    'msg': SaveDataStatus,
                    'topic': 'status',
                    'qsize': 5,
                    'callback': self._saveAllStatusCb, 
                    'callback_args': ()
                }
            }
            if ignore_global_rate_updates == False:
                ALL_SUBS_DICT['rate_all'] = {
                        'namespace': self.all_save_namespace,
                        'msg': SaveDataRate,
                        'topic': 'save_data_rate',
                        'qsize': 5,
                        'callback': self._saveRateCb, 
                        'callback_args': ()
                    } 

            subs_dict.update(ALL_SUBS_DICT)
        
        self.add_subs(subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################
       

        ##############################
        # Get for System Folders and Update
        ##############################
        self.msg_if.pub_debug("Waiting for user folders" )
        user_folders = nepi_system.get_user_folders()
        if user_folders is not None and 'data' in user_folders.keys():
            self.save_data_root_directory = user_folders['data']
        self.msg_if.pub_debug("Using SDK Share Folder: " + str(self.save_data_root_directory) )

        # Ensure the data folder exists with proper ownership
        if not os.path.exists(self.save_data_root_directory):
            self.msg_if.pub_debug("Reported data folder does not exist... data saving is disabled" )
            self.save_data_root_directory = None # Flag it as non-existent
            return # Don't enable any of the ROS interface stuff
        self.save_path = self.save_data_root_directory
        # And figure out user/group so that we know what ownership to create subfolders with
        stat_info = os.stat(self.save_data_root_directory)
        self.DATA_UID = stat_info.st_uid
        self.DATA_GID = stat_info.st_gid


        nepi_sdk.sleep(1)
        self.msg_if.pub_debug("Got starting filename dict: " + str(self.filename_dict) )
        if factory_filename_dict is not None:
            self.update_filename_dict(factory_filename_dict)
        self.msg_if.pub_debug("Got Updated filename dict: " + str(self.filename_dict) )

            


        #########################
        # Setup System IF Classes
        #########################
        # Initialize with empty dict, then call update function

        self.read_write_if = ReadWriteIF(
                            filename_dict = dict(),
                            node_name = self.node_name,
                            msg_if = self.msg_if
                            )

        #########################
        # Update status message for values not in NodeIF Dictionaries
        ##############################


        ##############################
        # Start Node Processes
        ##############################
        self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is SaveDataIF:
            self.start()


    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):
        """Build and publish a SaveDataStatus message on the status topic.

        Compiles current save rates, filename config, data directory, timezone,
        and an example filename into the status message and latches it to the
        'status' publisher. No-ops if pub_status is False or node_if is None.
        """

        save_rates_msg = []
        save_rate_dict = self.save_rate_dict

        #self.msg_if.pub_debug("Status got save_rate_dict", data = save_rate_dict , throttle_s = 5)
        for name in save_rate_dict.keys():
            save_rate_msg = SaveDataRate()
            save_rate_msg.data_product = name
            save_rate_msg.save_rate_hz = 0
            if self.disabled == False:
                save_rate_msg.save_rate_hz = save_rate_dict[name][0]
            save_rates_msg.append(save_rate_msg)
            #self.msg_if.pub_debug("data_rates_msg " + str(save_rates_msg) , throttle_s = 5)
        
        status_msg.node_name = self.node_name
        status_msg.save_data_topic = self.namespace
        status_msg.filename_config = self.create_filename_msg()
        status_msg.data_dir = self.save_path
        if self.filename_dict is not None:
            filename_dict = copy.deepcopy(self.filename_dict)
            try:
                status_msg.filename_prefix = filename_dict['prefix']
                status_msg.save_subfolder = filename_dict['subfolder']
                status_msg.save_data_utc = filename_dict['use_utc_tz']
            except Exception as e:
                self.msg_if.pub_debug("Failed to Publish filename dict: " + str(e) , throttle_s = 5)
            #self.msg_if.pub_debug("Publishing filename dict: " + str(filename_dict) , throttle_s = 5)

        status_msg.timezone = self.timezone
        status_msg.data_products = list(save_rate_dict.keys())
        status_msg.save_data_rates = save_rates_msg
        status_msg.save_data_enabled = self.save_data
        status_msg.disabled = self.disabled

        if self.all_save_namespace is None:
            status_msg.save_all_enabled = self.save_data
            # Guarded: publish_status runs on a 1 Hz timer, so a KeyError here would
            # silence the global status topic entirely.
            status_msg.save_all_rate = save_rate_dict.get('All',[0.0])[0]
        else:
            status_msg.save_all_enabled = self.save_all_enabled
            status_msg.save_all_rate = self.save_all_rate   


        if self.save_data_root_directory is not None:
            status_msg.data_dir = self.save_data_root_directory

        timezone = 'UTC'
        if self.filename_dict is not None:
            if self.filename_dict['use_utc_tz'] == False:
                timezone = self.timezone

        #self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone)  , throttle_s = 5)
        exp_filename = self.read_write_if.get_example_filename(timezone = timezone)
        status_msg.example_filename = exp_filename


        return status_msg

    def init(self, do_updates = False):
        """Load save rate and filename parameters from the ROS param server and publish status.

        Reads the persisted save_rate_dict from the param server, merges it into the
        current dict (preserving last-save timers at zero), writes the merged dict back,
        and then publishes the current status.

        Args:
            do_updates (bool, optional): Reserved for future use; has no effect currently.
                Defaults to False.
        """
        #self.msg_if.pub_debug("Param updated save rate dict: " + str(self.save_rate_dict) )
        if self.node_if is not None:
            # Prefixed keys, matching how the params are registered in PARAMS_DICT
            # and how set_save_rate()/update_filename_dict()/register_data_product()
            # write them back. get_param() returns None for a name it does not know,
            # so the unprefixed name wiped filename_dict on every config init, reset
            # and factory reset -- and save() no-ops entirely when filename_dict is
            # None, so nothing was ever written to disk.
            save_rate_dict = self.get_param('save_rate_dict')
            if save_rate_dict is not None:
                for data_product in self.save_rate_dict.keys():
                    if data_product in save_rate_dict.keys():
                        self.save_rate_dict[data_product][0] = save_rate_dict[data_product][0]
                    self.save_rate_dict[data_product][1] = 0.0 # Reset timer
            self.set_value('save_rate_dict',self.save_rate_dict)
            filename_dict = self.get_param('filename_dict')
            if filename_dict is not None:
                self.filename_dict = filename_dict
            disabled = self.get_param('disabled')
            if disabled is not None:
                self.disabled = disabled
        self.publish_status()

    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    

    ###############################
    # Class Public Methods
    ###############################

    def get_data_products(self):
        """Return the list of registered data product names.

        Returns:
            list: List of data product name strings currently tracked by this interface.
        """
        return list(self.save_rate_dict.keys())

    def register_data_product(self, data_product,factory_rate = 0):
        """Register a new data product with an optional factory save rate.

        If the data product is not already registered it is added to the save rate
        dictionary, the snapshot dictionary, and the ROS parameter server.

        Args:
            data_product (str): Unique name for the data product.
            factory_rate (float, optional): Default save rate in Hz. Defaults to 0
                (disabled).
        """
        save_rate_dict = self.save_rate_dict
        if data_product not in save_rate_dict.keys():
            save_rate_dict[data_product] =  [factory_rate, 0.0, 100] # Default to 1Hz save rate, max rate = 100Hz
            self.snapshot_dict[data_product] = False
            self.set_value('save_rate_dict',save_rate_dict)

    def unregister_data_product(self, data_product):
        """Remove a previously registered data product from the save rate tracking dict.

        Args:
            data_product (str): Name of the data product to remove.
        """
        purge = False
        if data_product in self.save_rate_dict.keys():
            purge = True
        if purge == True:
            del self.save_rate_dict[data_product]


    def update_filename_dict(self,filename_dict):
        """Apply updates from a filename configuration dictionary.

        Handles prefix sanitization, subfolder creation with proper ownership, and
        updates the underlying ReadWriteIF and ROS parameter server. Only applies
        if the new dict differs from the current one.

        Args:
            filename_dict (dict): Dictionary containing one or more filename config
                keys to update (prefix, subfolder, add_timestamp, use_utc_tz,
                add_ms, add_us, add_tz, add_node_name).
        """

        if self.filename_dict != filename_dict and filename_dict is not None:
            if 'prefix' in filename_dict.keys():
               new_prefix = filename_dict['prefix']
               self.filename_dict['prefix'] = nepi_utils.get_clean_name(new_prefix)

            if 'subfolder' in filename_dict.keys():
                new_subfolder= nepi_utils.get_clean_name(filename_dict['subfolder'])
                 

                if new_subfolder != "" and self.save_data_root_directory != None:
                    full_path = os.path.join(self.save_data_root_directory, new_subfolder)
                elif self.save_data_root_directory != None:
                    full_path = self.save_data_root_directory
                else:
                    full_path = ""

                if not os.path.exists(full_path):
                    self.msg_if.pub_debug("Creating new data subdirectory " + full_path )
                    try:
                        os.makedirs(full_path)
                    except Exception as e:
                        self.msg_if.pub_debug("Could not create save folder " + new_subfolder + str(e)  )
                if os.path.exists(full_path):
                    try:
                        os.chown(full_path, self.DATA_UID, self.DATA_GID)
                    except Exception as e:
                        self.msg_if.pub_debug("Could not chmod on save folder " + full_path + str(e)  )
                    self.save_path = full_path
                    filename_dict['subfolder'] = new_subfolder

            # Apply Updates
            for key in self.filename_dict.keys():
                if key in filename_dict.keys():
                    self.filename_dict[key] = filename_dict[key]  

            self.set_value('filename_dict',self.filename_dict)
            if self.read_write_if is not None and self.filename_dict is not None:
                self.read_write_if.set_filename_dict(filename_dict)
            

    def set_save_rate(self,data_product,save_rate_hz=0):
        """Set the save rate for one or more data products.

        Supports special sentinel values for data_product to target all products
        (ALL_DATA_PRODUCTS), none (NONE_DATA_PRODUCTS), or only currently-active
        ones (ACTIVE_DATA_PRODUCTS). Per-product rates are clamped to the
        configured max rate for that product.

        Args:
            data_product (str): Name of the data product, or a sentinel constant
                from SaveDataRate (ALL_DATA_PRODUCTS, NONE_DATA_PRODUCTS,
                ACTIVE_DATA_PRODUCTS).
            save_rate_hz (float, optional): Target save rate in Hz. Defaults to 0
                (disabled).
        """
        save_all = SaveDataRate().ALL_DATA_PRODUCTS
        save_none = SaveDataRate().NONE_DATA_PRODUCTS
        save_active = SaveDataRate().ACTIVE_DATA_PRODUCTS
        save_rate_dict = self.save_rate_dict
        if (data_product == save_active):
            for d in save_rate_dict.keys():
                # Respect the max save rate
                if save_rate_dict[d][0] > 0:
                    save_rate_dict[d][0] = save_rate_hz if save_rate_hz <= save_rate_dict[d][2] else save_rate_dict[d][2]
        elif (data_product == save_all):
            for d in save_rate_dict.keys():
                save_rate_dict[d][0] = save_rate_hz
        elif (data_product == save_none):
            for d in save_rate_dict.keys():
                save_rate_dict[d][0] = 0.0
        elif (data_product in save_rate_dict.keys()):
            save_rate_dict[data_product][0] = save_rate_hz if save_rate_hz <= save_rate_dict[data_product][2] else save_rate_dict[data_product][2]
        else:
            self.msg_if.pub_debug("Requested unknown data product: " + data_product )    
        self.set_value('save_rate_dict',save_rate_dict)
        
    def disable(self, enabled):
        """Enable or disable data saving for all registered data products.

        When enabling after a disabled period, all last-save timestamps are reset
        so that each product triggers a save on the next call to data_product_should_save.

        Args:
            enabled (bool): True to enable saving, False to disable.
        """
        #self.msg_if.pub_debug("Setting Saving Disabled to: " + str(enabled) )
        self.disabled = enabled
        self.publish_status()    

    def save_data_enable(self, enabled):
        """Enable or disable data save state for all registered data products.

        When enabling after a disabled period, all last-save timestamps are reset
        so that each product triggers a save on the next call to data_product_should_save.

        Args:
            enabled (bool): True to enable save, False to disable.
        """
        if enabled == True and self.was_saving == False:
            for d in self.save_rate_dict.keys():
                self.save_rate_dict[d][1] = 0.0
            self.was_saving = True
        else:
            self.was_saving = False
        self.msg_if.pub_debug("Setting Save Enable to: " + str(enabled) ) 
        self.save_data = enabled
        self.publish_status()    


    def get_saving_enabled(self):
        """Return whether data saving is currently enabled.

        Returns:
            bool: True if saving is enabled, False if disabled.
        """
        return self.save_data and self.disabled == False

    def data_product_save_enabled(self, data_product):
        """Return whether a specific data product is currently configured to save.

        Returns False if global saving is disabled or if the product's save rate
        is zero.

        Args:
            data_product (str): Name of the data product to check.

        Returns:
            bool: True if saving is enabled and the product has a non-zero rate,
                False otherwise.
        """
        # If saving is disabled for this node, then no data products are saving
        try:
            save_rate_dict = self.save_rate_dict
            if self.save_data == False:
                return False

            if data_product not in save_rate_dict:
                self.msg_if.pub_debug("Unknown data product " + data_product )
                return False

            save_rate = save_rate_dict[data_product][0]
            return (save_rate > 0.0) and self.disabled == False
        except:
            return False
        
    def data_product_save_rate(self, data_product):
        """Return the configured save rate for a specific data product.

        Returns False if global saving is disabled or the product is unknown.

        Args:
            data_product (str): Name of the data product.

        Returns:
            float: Save rate in Hz, or False if saving is disabled or the product
                is unrecognized. Returns 0.0 on unexpected errors.
        """
        # If saving is disabled for this node, then no data products are saving
        try:
            save_rate_dict = self.save_rate_dict
            if self.save_data == False:
                return False

            if data_product not in save_rate_dict:
                self.msg_if.pub_debug("Unknown data product " + data_product )
                return False

            save_rate = save_rate_dict[data_product][0]
            return save_rate
        except:
            return 0.0



    def data_product_should_save(self, data_product):
        """Determine whether it is time to save a data product based on its rate and elapsed time.

        Checks the configured save rate and the time since the last save. Also
        returns True if a snapshot has been triggered for the product. Returns
        False if global saving is disabled, the product is unknown, or the rate
        is zero.

        Args:
            data_product (str): Name of the data product to check.

        Returns:
            bool: True if the product should be saved now, False otherwise.
        """
        # If saving is disabled for this node, then it is not time to save this data product!
        save_rate_dict = self.save_rate_dict
        #self.msg_if.pub_debug("Checking should save for save rate dict: " + str(save_rate_dict) , throttle_s = 5)
        
        if self.save_data == False or  self.disabled == True:
            return False

        if data_product not in save_rate_dict.keys():
            self.msg_if.pub_debug("Unknown data product " + data_product , throttle_s = 5)
            return False

        save_rate = save_rate_dict[data_product][0]
        if save_rate == 0.0:
            return False

        save_period = float(1) / float(save_rate)
        now = nepi_utils.get_time()
        elapsed = now - save_rate_dict[data_product][1]
        #self.msg_if.pub_debug("Checking should save: " + str([save_period,elapsed]) , throttle_s = 5)
        snapshot = self.snapshot_dict[data_product]
        if (elapsed >= save_period or snapshot):
            #self.msg_if.pub_debug("Should save: " + data_product + " : " + str([save_period,elapsed]) , throttle_s = 5)
            self.save_rate_dict = save_rate_dict
            return True 
        return False



    def data_product_snapshot_enabled(self, data_product):
        """Return whether a one-shot snapshot is pending for a data product.

        Args:
            data_product (str): Name of the data product to check.

        Returns:
            bool: True if a snapshot is pending, False if not or if the product
                is unrecognized.
        """
        try:
            enabled = self.snapshot_dict[data_product]  and self.disabled == False
            return enabled
        except:
            self.msg_if.pub_debug("Unknown snapshot data product " + data_product , throttle_s = 5)
            return False
        return False

    def data_product_snapshot_reset(self, data_product):
        """Clear the snapshot-pending flag for a data product after it has been saved.

        Args:
            data_product (str): Name of the data product to reset.

        Returns:
            bool: True if the flag was cleared, False if the product is unrecognized.
        """
        try:
            self.snapshot_dict[data_product] = False
            return True
        except:
            self.msg_if.pub_debug("Unknown snapshot data product " + data_product , throttle_s = 5)
            return False
        return False


    def data_products_should_save_dict(self):
        """Return a dictionary mapping each data product to whether it should save now.

        Combines the snapshot flag and the rate-based save logic for every
        registered data product.

        Returns:
            dict: Mapping of data product name to bool (True = should save now).
        """
        dps_dict=copy.deepcopy(self.snapshot_dict)
        for dp in dps_dict.keys():
            ss =  dps_dict[dp]
            sr = self.save_rate_dict[dp][0] > 0
            dps_dict[dp] = (ss or (self.save_data and sr)) and self.disabled == False
        return dps_dict


    def get_timestamp_string(self):
        """Return the current time as a formatted datetime string with milliseconds.

        Returns:
            str: Current datetime string in NEPI format including milliseconds.
        """
        return nepi_utils.get_datetime_str_now(add_ms = True, add_us = False)

    def get_filename_path_and_prefix(self):
        """Return the full save path joined with the current filename prefix.

        Returns:
            str: Combined path string, or empty string if the save path is None.
        """
        if self.save_path is None:
            return ""
        return os.path.join(self.save_path, self.read_write_if.get_filename_prefix())




    #***************************
    # NEPI data saving utility functions
    def save(self,data_product,data,timestamp = None, save_check=True, filename = None, key_name = None):
        """Save a data object for a named data product if conditions are met.

        Checks whether disk space is available, whether the product should save
        (rate-based or snapshot), and then delegates to ReadWriteIF.write_data_file.
        Resets the snapshot flag and updates the last-save timestamp after writing.

        Args:
            data_product (str): Name of the data product being saved.
            data: Data object to write. Type is inferred automatically.
            timestamp (float, optional): Unix timestamp for the filename. Defaults
                to current time if None.
            save_check (bool, optional): When True, the rate/snapshot logic gates
                the write. When False, the data is written unconditionally (if
                space is available). Defaults to True.
        """
        if self.space_available == False:
            self.data_product_snapshot_reset(data_product)
            return ''
        else:
            if self.disabled == False and self.filename_dict is not None:
                should_save = self.data_product_should_save(data_product)
                snapshot_enabled = self.data_product_snapshot_enabled(data_product)
                # Save data if enabled
                self.msg_if.pub_debug("******" , throttle_s = -1)
                save_check = [should_save, snapshot_enabled, save_check]
                self.msg_if.pub_debug("Checking save checks: " + data_product + " " + str(save_check)  , throttle_s = -1)
                if (should_save or snapshot_enabled or save_check == False):
                    if self.filename_dict['use_utc_tz'] == False:
                        timezone = self.timezone
                    else:
                        timezone = 'UTC'
                    self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone)  , throttle_s = -1)
                    if should_save == False:
                        filename = None
                    filename = self.read_write_if.write_data_file(self.save_path, data, data_product, timezone = timezone, timestamp = timestamp, filename = filename, key_name = key_name)
                    self.data_product_snapshot_reset(data_product)
                    self.save_rate_dict[data_product][1] = nepi_utils.get_time()
                self.msg_if.pub_debug("Finished Checking save data: " + data_product  , throttle_s = -1)
                self.msg_if.pub_debug("******" , throttle_s = -1)
                return filename


    def create_filename_msg(self):
        """Build and return a FilenameConfig ROS message from the current filename dict.

        Returns:
            nepi_interfaces.msg.FilenameConfig: Populated FilenameConfig message
                reflecting the current naming settings.
        """
        fn_msg = FilenameConfig()
        try:
            fn_dict = self.filename_dict
            fn_msg.save_prefix = fn_dict['prefix']
            fn_msg.save_subfolder = fn_dict['subfolder']
            fn_msg.add_timestamp = fn_dict['add_timestamp']
            fn_msg.use_utc_tz = fn_dict['use_utc_tz']
            fn_msg.add_ms = fn_dict['add_ms']
            fn_msg.add_us = fn_dict['add_us']
            fn_msg.add_tz = fn_dict['add_tz']
        except:
            pass
        return fn_msg


    def get_status_pub_msg(self):
        return self.get_status_msg()


    ###############################
    # Class Private Methods
    ###############################


 
    def updaterCb(self,timer):
        tzd = nepi_system.get_timezone()
        last_tz = copy.deepcopy(self.timezone)
        self.timezone = tzd
        self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)

        self.space_available = nepi_system.get_space_available()


    def _capabilitiesHandler(self, req):
        return_list = []
        save_rate_dict = self.save_rate_dict
        for d in save_rate_dict:
            return_list.append(SaveDataRate(data_product = d, save_rate_hz = save_rate_dict[d][0]))
        return SaveDataCapabilitiesQueryResponse(return_list)





    def _logNavPoseEnableCb(self, msg):
        self.msg_if.pub_debug("Recieved Log NavPose Enable Update: " + str(msg) )
        enabled = msg.data



    def _saveAllStatusCb(self,msg):
        #self.msg_if.pub_debug("Recieved save All status msg: " + str(msg) )
        data_product_list = msg.data_products
        if 'All' in data_product_list:
            self.save_all_enabled = msg.save_data_enabled
            index = data_product_list.index('All')
            self.save_all_rate = msg.save_data_rates[index].save_rate_hz

    def _disableCb(self, msg):
        #self.msg_if.pub_debug("Recieved Disable Update: " + str(msg) )
        enabled = msg.data
        self.disable(enabled)

    def _saveEnableCb(self, msg):
        self.msg_if.pub_debug("Recieved Enable Update: " + str(msg) )
        enabled = msg.data
        self.save_data_enable(enabled)
        

    def _saveRateCb(self, msg):
        self.msg_if.pub_debug("Recieved Rate Update: " + str(msg) )
        data_product = msg.data_product
        save_rate_hz = msg.save_rate_hz
        self.set_save_rate(data_product,save_rate_hz)
    
        
    def _setPrefixCb(self, msg):
        prefix = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        if filename_dict is not None:
            filename_dict['prefix'] = prefix
            self.update_filename_dict(filename_dict)


    def _setSubfolderCb(self, msg):
        subfolder = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        if filename_dict is not None:
            filename_dict['subfolder'] = subfolder
            self.update_filename_dict(filename_dict)


    def _setLocalTzCb(self, msg):
        use_utc = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        if filename_dict is not None:
            filename_dict['use_utc_tz'] = use_utc
            self.update_filename_dict(filename_dict)


    def _setFilenameCb(self, msg):
        filename_dict = nepi_sdk.convert_msg2dict(msg)
        if self.filename_dict is not None:
            self.update_filename_dict(filename_dict)



    def _snapshotCb(self,msg):
        self.msg_if.pub_debug("Recieved Snapshot Trigger" )
        save_rate_dict = self.save_rate_dict
        if self.disabled == False:
            for data_product in save_rate_dict.keys():
                save_rate = save_rate_dict[data_product][0]
                enabled = (save_rate > 0.0)
                if enabled:
                    self.snapshot_dict[data_product] = True

    def _saveConfigCb(self,msg):
        if self.node_if is not None:
            self.node_if.save_config()

    def _resetSaveDataCb(self,msg):
        self.reset()

    def _publishStatusCb(self, timer):
        self.publish_status()

#######################################
# TransformIF

# Transform_List = [x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, heading_deg]



class TransformIF(NodeIF):

    #########################
    ## NODE IF Required
    ## Class Variables Updated by NodeIF Init Class
    #########################

    STATUS_MSG = TransformStatus()
    DATA_MSG = None

    ID = 'transform'
    DISPLAY_NAME = 'Transform'
    DESCRIPTION = 'Transform'

    IF_DICT = dict()

    CAPS_DICT = dict(
            has_x = True,
            has_y = True,
            has_z = True,
            has_roll = True,
            has_pitch = True,
            has_yaw = True,
            has_heading = True,
            has_invert = True,
            has_offsets = True,
            supports_updates = True,
    )

    #########################
    ## CUSTOM
    #########################
    msg_if = None
    node_if = None

    ready = False
    
    ####
    # Capabilities
    has_x = True
    has_y = True
    has_z = True
    has_roll = True
    has_pitch = True
    has_yaw = True
    has_heading = True
    has_invert = True
    has_offsets = True
    supports_updates = True


    ######
    transform_dict = copy.deepcopy(nepi_nav.BLANK_TRANSFORM_DICT)
    source = ''
    end = ''
    has_transform = True



    
    #######################
    ### IF Initialization
    def __init__(self, 
                name = 'transform',
                namespace = None,
                transform_name = None,
                source_ref_description = '',
                end_ref_description = '',
                get_transform_function = None,
                info_dict = dict(),
                has_x = True,
                has_y = True,
                has_z = True,
                has_roll = True,
                has_pitch = True,
                has_yaw = True,
                has_heading = True,
                has_invert = True,
                has_offsets = True,
                msg_if = None,
                node_if = None,
                    
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.IF_DICT = dict(self.IF_DICT)
        self.transform_dict = copy.deepcopy(self.transform_dict)

        #############################
        # Initialize Class Variables
      


        ##############################
        # Update Class Variables
        ##############################

        name = nepi_utils.get_clean_name(name)
        if transform_name is not None:
            transform_name = nepi_utils.get_clean_name(transform_name)
            if transform_name.strip() != '':
                name = transform_name



        self.source = source_ref_description
        self.end = end_ref_description


        self.has_x = has_x
        self.has_y = has_y
        self.has_z = has_z
        self.has_roll = has_roll
        self.has_pitch = has_pitch
        self.has_yaw = has_yaw
        self.has_heading = has_heading
        self.has_invert = has_invert
        self.has_offsets = has_offsets


        self.get_transform_function = get_transform_function
        if self.get_transform_function is not None:
            self.supports_updates = False

        ####################
        # Update Caps Dict
        # Node IF adds/updtes these values to its self.caps_dict
        self.CAPS_DICT = dict(
                has_x = self.has_x,
                has_y = self.has_y,
                has_z = self.has_z,
                has_roll = self.has_roll,
                has_pitch = self.has_pitch,
                has_yaw = self.has_yaw,
                has_heading = self.has_heading,
                has_invert = self.has_invert,
                has_offsets = self.has_offsets,
                supports_updates = self.supports_updates,
        )

        ##############################  
        # Setup NodeIF Updates
        ##############################  

        #######################
        # Sync System IF inputs to if_dict

        self.msg_if = msg_if
        self.IF_DICT['msg_if'] = msg_if
        self.node_if = node_if
        self.IF_DICT['node_if'] = node_if

        ##################
        if_info_dict = dict(
            description = self.DESCRIPTION,
            display_name = self.DISPLAY_NAME,
            type = 'system',
            source_description = source_ref_description,
            source_ref_description = source_ref_description,
            end_description = end_ref_description,
            end_ref_description = end_ref_description,
        )
        update_info_dict = copy.deepcopy(info_dict)
        update_info_dict.update(if_info_dict)
        ##################        
        update_caps_dict = self.CAPS_DICT
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
        super().__init__(id = self.ID,
                name = name,
                namespace = namespace,
                status_msg = copy.deepcopy(self.STATUS_MSG),
                data_msg = copy.deepcopy(self.DATA_MSG),
                info_dict = update_info_dict,
                caps_dict = update_caps_dict, 
                stats_dict = update_stats_dict, 
                rui_dict = update_rui_dict, 
                config_control_dict = update_config_control_dict, 
                node_controls_dict = update_node_controls_dict, 
                callback_dict = update_callback_dict, 
                if_dict = dict(self.IF_DICT)
                )

        ready = self.wait_for_ready()


       #######################
        # Add Services
        #######################
       
        params_dict = {
            'transform': {
                'name': 'transform',
                'namespace': self.namespace,
                'factory_val': self.transform_dict
            },
        }
        self.add_params(params_dict)

        #######################
        # Add Services
        #######################
        services_dict = {}
        self.add_services(services_dict)

        #######################
        # Add Pubs 
        #######################
        pubs_dict = {
            'transform_pub': {
                'namespace': self.namespace,
                'msg': Transform,
                'topic': '',
                'qsize': 1,
                'latch': True
            }
        }
        self.add_pubs(pubs_dict)

        #######################
        # Add Subs 
        #######################
        if self.supports_updates == False:
            subs_dict = None
        else:
            subs_dict = {
                'clear_navpose_frame_transform': {
                    'namespace': self.namespace,
                    'topic': 'clear_transform',
                    'msg': Empty,
                    'qsize': 5,
                    'callback': self._clearFrame3dTransformCb, 
                    'callback_args': ()
                },
                'set_navpose_frame_transform': {
                    'namespace': self.namespace,
                    'topic': 'set_transform',
                    'msg': Transform,
                    'qsize': 5,
                    'callback': self._setFrame3dTransformCb,
                    'callback_args': ()
                },
                'set_source_ref_description': {
                    'namespace': self.namespace,
                    'topic': 'set_source_ref',
                    'msg': String,
                    'qsize': 5,
                    'callback': self._setSourceRefCb,
                    'callback_args': ()
                }
        }
        self.add_subs(subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################


        #########################
        # Update Status Message
        ##############################



        self.status_msg.has_x = self.has_x
        self.status_msg.has_y = self.has_y
        self.status_msg.has_z = self.has_z
        self.status_msg.has_roll = self.has_roll
        self.status_msg.has_pitch = self.has_pitch
        self.status_msg.has_yaw = self.has_yaw
        self.status_msg.has_heading = self.has_heading
        self.status_msg.has_invert = self.has_invert
        self.status_msg.has_offsets = self.has_offsets
        self.status_msg.supports_updates = self.supports_updates


        ##############################
        # Start Node Processes
        ##############################
        nepi_sdk.start_timer_process(1.0, self._publishTransformCb)

        
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is TransformIF:
            self.start()


    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):
        return status_msg

    ###############################
    # Class Public Methods
    ###############################
    def get_transform_dict(self):
        """Return the current 3D transform as a nepi_nav transform dict.

        Suitable for passing to nepi_nav.transform_navpose_dict() to apply the
        transform to a navpose dict.

        Returns:
            dict: BLANK_TRANSFORM_DICT-shaped dict populated from the current transform.
        """

        transform_dict = copy.deepcopy(self.transform_dict)
        return transform_dict
    

    def set_transform_value(self, name, value):
        """Return the current 3D transform as a nepi_nav transform dict.

        Suitable for passing to nepi_nav.transform_navpose_dict() to apply the
        transform to a navpose dict.

        Returns:
            dict: BLANK_TRANSFORM_DICT-shaped dict populated from the current transform.
        """

        transform_dict = copy.deepcopy(self.transform_dict)
        if name in transform_dict.keys():
            cap_name = 'has_' + name.split('_')[0]
            if cap_name in self.caps_dict.keys():
                allowed = self.caps_dict[cap_name] == True
                if name.endswith('_invert'):
                    allowed = allowed and self.has_invert == True
                if name.endswith('_offset'):
                    allowed = allowed and self.has_offsets == True
                if allowed == True:
                    try:
                        self.transform_dict[name] = float(value)
                    except:
                        pass
        if transform_dict != self.transform_dict:
            self.publish_transform()
            self.set_value('transform',self.transform_dict)



    def update_transforms_from_list(self,transform_list):
        if self.supports_updates == True:
            # Apply Caps: disabled axes keep their stored values, and the caller's
            # list is not modified
            cap_keys = list(self.CAPS_DICT)
            stored_list = nepi_nav.convert_transform_dict2list(self.transform_dict)
            update_list = list(transform_list)
            for i, value in enumerate(update_list):
                if len(cap_keys) > i and len(stored_list) > i:
                    cap_value = self.caps_dict[cap_keys[i]]
                    if cap_value == False:
                        update_list[i] = stored_list[i]
            transform_dict = copy.deepcopy(self.transform_dict)
            self.transform_dict = nepi_nav.update_transform_dict_from_list(transform_dict, transform_list = update_list)
            if transform_dict != self.transform_dict:
                self.publish_transform()
                self.set_value('transform',self.transform_dict)
                
                    
    def update_offsets_from_list(self,offset_list):
        if self.supports_updates == True and self.has_offsets == True:
            transform_dict = copy.deepcopy(self.transform_dict)
            self.transform_dict = nepi_nav.update_transform_dict_from_list(transform_dict, offset_list = offset_list)
            if transform_dict != self.transform_dict:
                self.publish_transform()


    def clear_transforms(self):
        """Reset the 3D transform to the zero/identity transform and publish the update.

        No-ops if updates are not supported (i.e., a get_transform_function was
        supplied at construction).
        """
        if self.supports_updates == True:
            transform_dict = copy.deepcopy(self.transform_dict)
            self.transform_dict = nepi_nav.clear_transforms(transform_dict)
            if transform_dict != self.transform_dict:
                self.publish_transform()
                self.set_value('transform',self.transform_dict)


    def clear_offsets(self):
        transform_dict = copy.deepcopy(self.transform_dict)
        self.transform_dict = nepi_nav.clear_offsets(transform_dict)



    #############################
    def get_transform_list(self):
        """Return the current 3D transform as a seven-element list.

        If a get_transform_function was provided at construction, it is called
        first. Falls back to the internally stored transform if the function returns
        None.

        Returns:
            list: [x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, heading_deg].
        """
        transform_list = None
        if self.get_transform_function is not None:
            transform_list = self.get_transform_function()
        if transform_list is None:
            transform_dict = copy.deepcopy(self.transform_dict)
            transform_list = nepi_nav.convert_transform_dict2list(transform_dict)

        return transform_list



    def get_source_description(self):
        """Return the source reference frame description string.

        Returns:
            str: Human-readable description of the source reference frame.
        """
        return self.source

    def set_source_description(self,source):
        """Set the source reference frame description and republish the transform.

        Args:
            source (str): Human-readable description of the source reference frame.
        """
        self.source = source
        self.status_msg.source_ref_description = self.source
        transform_dict = copy.deepcopy(self.transform_dict)
        self.transform_dict['source_ref_description'] = self.source
        if transform_dict != self.transform_dict:
            self.publish_transform()
            self.set_value('transform',self.transform_dict)
        

    def get_end_description(self):
        """Return the end (target) reference frame description string.

        Returns:
            str: Human-readable description of the end reference frame.
        """
        return self.end

    def set_end_description(self,end):
        """Set the end (target) reference frame description and republish the transform.

        Args:
            end (str): Human-readable description of the end reference frame.
        """
        self.end = end
        self.status_msg.end_ref_description = self.end
        transform_dict = copy.deepcopy(self.transform_dict)
        self.transform_dict['end_ref_description'] = self.end
        if transform_dict != self.transform_dict:
            self.publish_transform()
            self.set_value('transform',self.transform_dict)


    def publish_transform(self):
        """Build and publish the current transform as a ROS Transform message.

        Converts the current seven-element transform list to a ROS message and
        publishes it on the 'transform_pub' topic. No-ops if node_if is None.
        """

        transform_dict = copy.deepcopy(self.transform_dict)
        transform_msg = nepi_nav.convert_transform_dict2msg(transform_dict)
        if self.node_if is not None:
            self.publish_msg( 'transform_pub',transform_msg)


    def init(self, do_updates = True):
        """Load transform, source, and end parameters from the ROS param server.

        Reads the persisted values and publishes the updated status.

        Args:
            do_updates (bool, optional): Reserved for future use; has no additional
                effect currently. Defaults to True.
        """
        if self.node_if is not None:
            # Prefixed keys, matching how the params are registered in PARAMS_DICT and
            # how set_transform()/set_source_description()/set_end_description()
            # write them back. get_param() returns None for a name it does not know,
            # so the unprefixed names wiped the transform_dict on every config init, reset
            # and factory reset -- saved 3D transforms never restored.
            transform_dict = self.get_param('transform')
            if transform_dict is not None:
                # Merge: the defaults already in self.transform_dict, saved values on top
                for key in transform_dict.keys():
                    if key in self.transform_dict.keys():
                        self.transform_dict[key] = transform_dict[key]
            self.clear_offsets()


            #self.msg_if.pub_debug("Setting init values to param server values: " + str(self.init_settings_dict) )
            if do_updates:
                pass
            self.publish_status()



    ###############################
    # Class Private Methods
    ###############################
    def initCb(self, do_updates = False):
        self.init(do_updates = do_updates)


    def _setFrame3dTransformCb(self, msg):
        self.msg_if.pub_debug("Received Frame Transform update message: " + str(msg) )
        transform_msg = msg
        # The stored transform is a 7-element signed list; fold the per-axis
        # invert flags into the sign so the stored/republished value is unambiguous.
        x = -transform_msg.x_m if transform_msg.x_invert else transform_msg.x_m
        y = -transform_msg.y_m if transform_msg.y_invert else transform_msg.y_m
        z = -transform_msg.z_m if transform_msg.z_invert else transform_msg.z_m
        roll = -transform_msg.roll_deg if transform_msg.roll_invert else transform_msg.roll_deg
        pitch = -transform_msg.pitch_deg if transform_msg.pitch_invert else transform_msg.pitch_deg
        yaw = -transform_msg.yaw_deg if transform_msg.yaw_invert else transform_msg.yaw_deg
        heading = -transform_msg.heading_deg if transform_msg.heading_invert else transform_msg.heading_deg
        transform_list = [x, y, z, roll, pitch, yaw, heading]

        transform_dict = copy.deepcopy(self.transform_dict)
        self.transform_dict = nepi_nav.update_transform_dict_from_list(transform_dict, transform_list)
        if transform_dict != self.transform_dict:
            self.publish_transform()
            self.set_value('transform',self.transform_dict)



    def _clearFrame3dTransformCb(self, msg):
        self.msg_if.pub_debug("Recived Clear 3D Transform update message: " )
        self.clear_transforms()

    def _setSourceRefCb(self, msg):
        self.set_source_description(msg.data)

    def _publishTransformCb(self, timer):
        self.publish_transform()











############################################
# StatesIF


STATE_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]

EXAMPLE_STATES_DICT = {
                    "state_name": {
                        "value": False
                    }
}


class StatesIF(NodeIF):

    #########################
    ## NODE IF Required
    ## Class Variables Updated by NodeIF Init Class
    #########################

    STATUS_MSG = SystemStatesStatus()
    DATA_MSG = None

    ID = 'states'
    DISPLAY_NAME = 'States'
    DESCRIPTION = 'States'

    IF_DICT = dict()

    #########################
    ## CUSTOM
    #########################
    msg_if = None
    node_if = None

    ready = False

    get_states_dict_function = None



    #######################
    ### IF Initialization
    def __init__(self,
                name = 'states',
                namespace = None,
                states_name = None,
                get_states_dict_function = None,
                info_dict = dict(),
                msg_if = None,
                node_if = None,
                
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.IF_DICT = dict(self.IF_DICT)

    

        ##############################
        # Update Class Variables
        ##############################


        name = nepi_utils.get_clean_name(name)
        if states_name is not None:
            states_name = nepi_utils.get_clean_name(states_name)
            if states_name.strip() != '':
                name = states_name

        if get_states_dict_function is None:
            return
        self.get_states_dict_function = get_states_dict_function

   

        ##############################  
        # Setup NodeIF Updates
        ##############################  

        #######################
        # Sync System IF inputs to if_dict

        self.msg_if = msg_if
        self.IF_DICT['msg_if'] = msg_if
        self.node_if = node_if
        self.IF_DICT['node_if'] = node_if


        ##################
        if_info_dict = dict(
            description = self.DESCRIPTION,
            display_name = self.DISPLAY_NAME,
            type = 'system'
        )
        update_info_dict = copy.deepcopy(info_dict)
        update_info_dict.update(if_info_dict)
        ##################        
        update_caps_dict = dict()
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
        super().__init__(id = self.ID,
                name = name,
                namespace = namespace,
                status_msg = copy.deepcopy(self.STATUS_MSG),
                data_msg = copy.deepcopy(self.DATA_MSG),
                info_dict = update_info_dict,
                caps_dict = update_caps_dict, 
                stats_dict = update_stats_dict, 
                rui_dict = update_rui_dict, 
                config_control_dict = update_config_control_dict, 
                node_controls_dict = update_node_controls_dict, 
                callback_dict = update_callback_dict, 
                if_dict = dict(self.IF_DICT)
                )

        ready = self.wait_for_ready()


       #######################
        # Add Services
        #######################
       
        params_dict = {}
        self.add_params(params_dict)

        #######################
        # Add Services
        #######################
        services_dict = {
            'states_query': {
                'namespace': self.namespace,
                'topic': 'system_states_query',
                'srv': SystemStatesQuery,
                'req': SystemStatesQueryRequest(),
                'resp': SystemStatesQueryResponse(),                
                'callback': self._statesQueryHandler
            }
        }
        self.add_services(services_dict)

        #######################
        # Add Pubs 
        #######################
        pubs_dict = {}
        self.add_pubs(pubs_dict)

        #######################
        # Add Subs 
        #######################
        subs_dict =  {}
        self.add_subs(subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################

        #########################
        # Update status message for values not in NodeIF Dictionaries
        ##############################


        ##############################
        # Start Node Processes
        ##############################


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is StatesIF:
            self.start()




    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):
        states_dict = self.get_states_dict_function()

        state_names = []
        states_msg_list = []
        if states_dict is not None:
            for key in states_dict.keys():
                state_names.append(str(key))
                state = states_dict[key] == True
                states_msg_list.append(nepi_states.create_state_msg(str(key), state))

        status_msg.state_names = state_names
        status_msg.states_msg_list = states_msg_list
        return status_msg
    
    ###############################
    # Class Private Methods
    ###############################

    def _statesQueryHandler(self, req):
        resp = SystemStatesQueryResponse()
        try:
            states_dict = self.get_states_dict_function()
            resp = nepi_states.create_states_msg(states_dict)
        except Exception as e:
            self.msg_if.pub_debug("Failed to create resp msg: " + str(e) )
        return resp





EXAMPLE_TRIGGER_DICT = {
                    "name":"None",
                    "node_name": '~',
                    "description": "None",
                    "data_str_list":["None"],
                    "time":nepi_utils.get_time() 
}


EXAMPLE_TRIGGERS_DICT = {
                "trigger_name": {
                    "name":"None",
                    "node_name": '~',
                    "description": "None",
                    "data_str_list":["None"],
                    "time":nepi_utils.get_time() 
                    }

}



class TriggersIF(NodeIF):

    #########################
    ## NODE IF Required
    ## Class Variables Updated by NodeIF Init Class
    #########################

    STATUS_MSG = SystemTriggersStatus()
    DATA_MSG = SystemTriggers()

    ID = 'triggers'
    DISPLAY_NAME = 'Triggers'
    DESCRIPTION = 'Triggers'

    IF_DICT = dict()

    #########################
    ## CUSTOM
    #########################
    msg_if = None
    node_if = None

    ready = False

    triggers_dict = dict()
    triggers_list = []

    #######################
    ### IF Initialization
    def __init__(self, 
                name = 'triggers',
                namespace = None,
                triggers_name = None,
                triggers_dict = None,
                info_dict = dict(),
                msg_if = None,
                node_if = None,
                
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.IF_DICT = dict(self.IF_DICT)
        self.triggers_list = copy.deepcopy(self.triggers_list)
   
        ##############################
        # Update Class Variables
        ##############################

        name = nepi_utils.get_clean_name(name)
        if triggers_name is not None:
            triggers_name = nepi_utils.get_clean_name(triggers_name)
            if triggers_name.strip() != '':
                name = triggers_name
   

        if triggers_dict is None:
            self.triggers_dict = dict()
        else:
            self.triggers_dict = triggers_dict

        ##############################  
        # Setup NodeIF Updates
        ##############################  

        #######################
        # Sync System IF inputs to if_dict

        self.msg_if = msg_if
        self.IF_DICT['msg_if'] = msg_if
        self.node_if = node_if
        self.IF_DICT['node_if'] = node_if


        ##################
        if_info_dict = dict(
            description = self.DESCRIPTION,
            display_name = self.DISPLAY_NAME,
            type = 'system'
        )
        update_info_dict = copy.deepcopy(info_dict)
        update_info_dict.update(if_info_dict)
        ##################        
        update_caps_dict = dict()
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
        super().__init__(id = self.ID,
                name = name,
                namespace = namespace,
                status_msg = copy.deepcopy(self.STATUS_MSG),
                data_msg = copy.deepcopy(self.DATA_MSG),
                info_dict = update_info_dict,
                caps_dict = update_caps_dict, 
                stats_dict = update_stats_dict, 
                rui_dict = update_rui_dict, 
                config_control_dict = update_config_control_dict, 
                node_controls_dict = update_node_controls_dict, 
                callback_dict = update_callback_dict, 
                if_dict = dict(self.IF_DICT)
                )

        ready = self.wait_for_ready()


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
        pubs_dict = {
            'triggers': {
                'msg': SystemTrigger,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            'system_triggers': {
                'msg': SystemTrigger,
                'namespace': self.base_namespace,
                'topic': 'system_triggers',
                'qsize': 1,
                'latch': False
            }
        }
        self.add_pubs(pubs_dict)

        #######################
        # Add Subs 
        #######################
        subs_dict =  {}
        self.add_subs(subs_dict)

        nepi_sdk.sleep(0.1)

        ######################################################

        #########################
        # Update status message for values not in NodeIF Dictionaries
        ##############################


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_debug("IF Initialization Complete" )
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is TriggersIF:
            self.start()


    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):
        triggers_list = copy.deepcopy(self.triggers_list)
        self.triggers_list = []

        trigger_names = []
        triggers_msg_list = []
        for msg in triggers_list:
            trigger_names.append(msg.name)
            triggers_msg_list.append(msg)
        status_msg.trigger_names = trigger_names
        status_msg.triggers_msg_list = triggers_msg_list
        return status_msg

    ###############################
    # Class Public Methods
    ###############################

    def publish_trigger(self, trigger_dict):
        """Build and publish a system trigger message on the 'system_triggers' topic.

        Converts the trigger dictionary to a SystemTrigger ROS message and publishes
        it via the node_if. No-ops if node_if is None.

        Args:
            trigger_dict (dict): Trigger event dictionary with keys 'name',
                'node_name', 'description', 'data_str_list', and 'time'.
        """
        try:
            trig_msg = nepi_triggers.create_trigger_msg(self.namespace, trigger_dict)
            self.publish_msg( 'triggers',trig_msg)
            self.publish_msg( 'system_triggers',trig_msg)
            self.triggers_list.append(trig_msg)
        except Exception as e:
            self.msg_if.pub_debug("Failed to create trigger msg from dict", data = [trigger_dict,str(e)] )



    ###############################
    # Class Private Methods
    ###############################



