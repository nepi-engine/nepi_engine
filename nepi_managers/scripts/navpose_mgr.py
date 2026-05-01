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
import sys
import tf
import yaml
import threading
import copy



from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_nav

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Odometry

from nepi_interfaces.msg import MgrNavPoseStatus, NavPoseComponent, NavPoseSolution, MgrSystemStatus


from nepi_interfaces.msg import UpdateString, UpdateFloat, UpdateFloats, UpdateBool, UpdateInt, UpdateNavPose, UpdateTransform
from nepi_interfaces.msg import NavPose, NavPoses, NavPosesStatus


from nepi_interfaces.srv import MgrNavPoseCapabilitiesQuery, MgrNavPoseCapabilitiesQueryRequest, MgrNavPoseCapabilitiesQueryResponse
from nepi_interfaces.srv import NavPosesQuery, NavPosesQueryRequest, NavPosesQueryResponse


from nepi_interfaces.msg import Transform
#from nepi_interfaces.srv import TransformsRegister, TransformsRegisterRequest, TransformsRegisterResponse
#from nepi_interfaces.srv import TransformsDelete, TransformsDeleteRequest, TransformsDeleteResponse

from nepi_api.node_if import NodePublishersIF, NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import SaveDataIF



#########################################
# Node Class
#########################################


class NavPoseMgr(object):
    MGR_NODE_NAME = 'navpose_mgr'

    
    NAVPOSE_BASE_FRAME = 'base_frame'
    NAVPOSE_BASE_FRAME_DESC = 'NEPI base navpose frame'

    NAVPOSE_PUB_RATE_OPTIONS = [1.0,20.0] 
    NAVPOSE_NAV_FRAME_OPTIONS = nepi_nav.NAVPOSE_NAV_FRAME_OPTIONS 
    NAVPOSE_ALT_FRAME_OPTIONS = nepi_nav.NAVPOSE_ALT_FRAME_OPTIONS
    NAVPOSE_DEPTH_FRAME_OPTIONS = nepi_nav.NAVPOSE_DEPTH_FRAME_OPTIONS

    FACTORY_PUB_RATE_HZ = 10.0
    FACTORY_3D_FRAME = 'base_frame' 
    FACTORY_NAV_FRAME = 'ENU'
    FACTORY_ALT_FRAME = 'WGS84'

    ZERO_TRANSFORM_DICT = copy.deepcopy(nepi_nav.BLANK_TRANSFORM_DICT)

    NAVPOSE_COMPONENT_NAMES = ['navpose_init','navpose_source_replace','navpose_source_offset','navpose_update_offset','navpose_update_reset']

    BLANK_CONNECT_DICT = {
            'init_topic': "",
            'init_transform': copy.deepcopy(ZERO_TRANSFORM_DICT),
            'init_options': ['ONCE','TIMED','ALLWAYS'],
            'init_option': 'ALLWAYS',
            'init_timed_sec': 1,
            'source_topic': "",
            'source_transform': copy.deepcopy(ZERO_TRANSFORM_DICT),
            'source_options': ['REPLACES','OFFSETS'],
            'source_option': 'REPLACES',
            'update_topic': "",
            'update_transform': copy.deepcopy(ZERO_TRANSFORM_DICT),
            'update_options': ['OFFSETS','RESETS'],
            'update_option': 'RESETS',
            'update_resets_on_crossing': False,
            'update_resets_crossing': 0
        }
    
    BLANK_TIMES = {
            'times': [0,0,0,0,0,0,0],
            'last_time': 0.0
        }

    BLANK_SUB = {
            'topic': "None",
            'msg': "",
            'sub': None,
            'subs_dict': {
                'None': [] # Frames and Components Lists Added if Subscribed
            }
        }

    BLANK_AVIAL_TOPIC = {
            'topics': [],
            'msgs': [],
        }
    
    BLANK_CONNECTS_DICT = {
        'location': copy.deepcopy(BLANK_CONNECT_DICT),
        'heading': copy.deepcopy(BLANK_CONNECT_DICT),
        'orientation': copy.deepcopy(BLANK_CONNECT_DICT),
        'position': copy.deepcopy(BLANK_CONNECT_DICT),
        'altitude': copy.deepcopy(BLANK_CONNECT_DICT),
        'depth': copy.deepcopy(BLANK_CONNECT_DICT),
        'pan_tilt': copy.deepcopy(BLANK_CONNECT_DICT)
    }

    BLANK_STATES_DICT = {
        'location': None,
        'heading': None,
        'orientation': None,
        'position': None,
        'altitude': None,
        'depth': None,
        'pan_tilt': None,
    }

    BLANK_TIMES_DICT = {
        'location': copy.deepcopy(BLANK_TIMES),
        'heading': copy.deepcopy(BLANK_TIMES),
        'orientation': copy.deepcopy(BLANK_TIMES),
        'position': copy.deepcopy(BLANK_TIMES),
        'altitude': copy.deepcopy(BLANK_TIMES),
        'depth': copy.deepcopy(BLANK_TIMES),
        'pan_tilt': copy.deepcopy(BLANK_TIMES)
    }

    BLANK_SUBS_DICT = {
        'None': copy.deepcopy(BLANK_SUB)
    }

    BLANK_TRANSFORMS_DICT = {
        'location': copy.deepcopy(ZERO_TRANSFORM_DICT),
        'heading': copy.deepcopy(ZERO_TRANSFORM_DICT),
        'orientation': copy.deepcopy(ZERO_TRANSFORM_DICT),
        'position': copy.deepcopy(ZERO_TRANSFORM_DICT),
        'altitude': copy.deepcopy(ZERO_TRANSFORM_DICT),
        'depth': copy.deepcopy(ZERO_TRANSFORM_DICT),
        'pan_tilt': copy.deepcopy(ZERO_TRANSFORM_DICT)
    }


    node_namespace = "mgr_navpose"

    node_if = None

    save_data_if = None
    data_products_list = ['navposes']

    navposes_init_frames = [NAVPOSE_BASE_FRAME]
    navposes_topic = ''

    status_msg = MgrNavPoseStatus()
    status_published = False
    navposes_status_msg = NavPosesStatus()
    caps_response = MgrNavPoseCapabilitiesQueryResponse()

    navposes_settings_dict = dict()

    frame_nav = 'ENU'
    frame_alt = 'WGS84'
    frame_depth = 'DEPTH'


    navposes_info_dict = dict()
    last_navposes_info_dict = dict()
    navposes_info_dict_lock = threading.Lock()


    navposes_fixed_dict = dict()
    navposes_fixed_dict_lock = threading.Lock()

    navposes_init_dict = dict()
    navposes_init_states_dict = dict()
    navposes_init_times_dict = dict()
    navposes_init_dict_lock = threading.Lock()


    navposes_source_dict = dict()
    navposes_source_states_dict = dict()
    navposes_source_times_dict = dict()
    navposes_source_dict_lock = threading.Lock()


    navposes_update_dict = dict()
    navposes_update_states_dict = dict()
    navposes_update_offset_dict = dict()
    navposes_update_reset_dict = dict()
    navposes_update_times_dict = dict()
    navposes_update_dict_lock = threading.Lock()
    

    
    
    

    navposes_pubs_dict = dict()
    navposes_pubs_dict_lock = threading.Lock()

    navposes_max_pub_rate = 1
    last_navposes_pub_time = nepi_utils.get_time()

    navposes_solution_dict = dict()


    avail_topics_dict = {
        'all_topics_dict': dict(),
        'location': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'heading': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'orientation': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'position': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'altitude': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'depth': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'pan_tilt': copy.deepcopy(BLANK_AVIAL_TOPIC)
    }

    navpose_subs_registered = []
    navpose_subs_connecting = []
    navpose_subs_connected = []
    navpose_subs_dict = copy.deepcopy(BLANK_SUBS_DICT)
    navpose_subs_dict_lock = threading.Lock()


    navposes_pub_times_dict = dict()
    navposes_solution_dict = dict()
    navpose_pubs_published = False

    navposes_save_dict = dict()

    update_navposes = False

    active_nodes = []
    active_topics = []
    active_topic_types = []
    active_services = []

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "navpose_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        nepi_sdk.sleep(1)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################
        # Initialize Class Variables
        
        self.msg_if.pub_warn("Waiting for Config Mgr")
        config_folders = nepi_system.get_config_folders()

       
        self.navposes_topic = self.base_namespace + '/navposes'

        self.caps_response.navposes_topic = self.navposes_topic
        self.caps_response.frame_nav_options = self.NAVPOSE_NAV_FRAME_OPTIONS 
        self.caps_response.frame_alt_options = self.NAVPOSE_ALT_FRAME_OPTIONS
        self.caps_response.frame_depth_options = self.NAVPOSE_DEPTH_FRAME_OPTIONS

    
        self.status_msg.navposes_topic  = self.navposes_topic



        self.initCb(do_updates = False)

        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace': self.node_namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'navposes_max_pub_rate': {
                'namespace': self.node_namespace,
                'factory_val': self.navposes_max_pub_rate
            },
            'navposes_info_dict': {
                'namespace': self.node_namespace,
                'factory_val': self.navposes_info_dict
            },
            'navposes_fixed_dict': {
                'namespace': self.node_namespace,
                'factory_val': self.navposes_fixed_dict
            },
            'frame_nav': {
                'namespace': self.node_namespace,
                'factory_val': self.frame_nav
            },
            'frame_alt': {
                'namespace': self.node_namespace,
                'factory_val': self.frame_alt
            },
            'frame_depth': {
                'namespace': self.node_namespace,
                'factory_val': self.frame_depth
            },
            
        }

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'capabilities_query': {
                'namespace': self.node_namespace,
                'topic': 'capabilities_query',
                'srv': MgrNavPoseCapabilitiesQuery,
                'req': MgrNavPoseCapabilitiesQueryRequest(),
                'resp': MgrNavPoseCapabilitiesQueryResponse(),
                'callback': self._navposeCapsQueryHandler
            },
            'navpose_query': {
                'namespace': self.node_namespace,
                'topic': 'navpose_query',
                'srv': NavPosesQuery,
                'req': NavPosesQueryRequest(),
                'resp': NavPosesQueryResponse(),
                'callback': self._navposesDataQueryHandler
            }
        }


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': MgrNavPoseStatus,
                'qsize': 1,
                'latch': True
            },
            'navposes_pub': {
                'namespace': self.base_namespace,
                'topic': 'navposes',
                'msg': NavPoses,  # This should be the correct message type
                'qsize': 1,
                'latch': False
            },
            'navposes_status_pub': {
                'namespace': self.base_namespace + '/navposes',
                'topic': 'status',
                'msg': NavPosesStatus,  # This should be the correct message type
                'qsize': 1,
                'latch': False
            }
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'set_navposes_max_pub_rate': {
                'namespace': self.node_namespace,
                'topic': 'set_navposes_max_pub_rate',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setNavPosesPubRateCb, 
                'callback_args': ()
            },
            'set_frame_nav': {
                'namespace': self.node_namespace,
                'topic': 'set_frame_nav',
                'msg': String,
                'qsize': 1,
                'callback': self._setFrameNavCb, 
                'callback_args': ()
            },
            'set_frame_alt': {
                'namespace': self.node_namespace,
                'topic': 'set_frame_alt',
                'msg': String,
                'qsize': 1,
                'callback': self._setFrameAltCb, 
                'callback_args': ()
            },
            'set_frame_depth': {
                'namespace': self.node_namespace,
                'topic': 'set_frame_depth',
                'msg': String,
                'qsize': 1,
                'callback': self._setFrameDepthCb, 
                'callback_args': ()
            },
            'add_frame': {
                'namespace': self.node_namespace,
                'topic': 'add_frame',
                'msg': UpdateString,
                'qsize': 1,
                'callback': self._addFrameCb, 
                'callback_args': ()
            },
            'copy_frame': {
                'namespace': self.node_namespace,
                'topic': 'add_frame',
                'msg': UpdateString,
                'qsize': 1,
                'callback': self._copyFrameCb, 
                'callback_args': ()
            },
            'delete_frame': {
                'namespace': self.node_namespace,
                'topic': 'delete_frame',
                'msg': String,
                'qsize': 1,
                'callback': self._deleteFrameCb, 
                'callback_args': ()
            },
            'reset_frame': {
                'namespace': self.node_namespace,
                'topic': 'reset_frame',
                'msg': String,
                'qsize': 1,
                'callback': self._resetFrameCb, 
                'callback_args': ()
            },
            'reset_frame_navpose': {
                'namespace': self.node_namespace,
                'topic': 'reset_frame_navpose',
                'msg': String,
                'qsize': 1,
                'callback': self._resetFrameNavposeCb, 
                'callback_args': ()
            },
            'set_frame_description': {
                'namespace': self.node_namespace,
                'topic': 'set_frame_description',
                'msg': UpdateString,
                'qsize': 1,
                'callback': self._setFrameDescriptionCb, 
                'callback_args': ()
            },
            'set_frame_pub_rate': {
                'namespace': self.node_namespace,
                'topic': 'set_frame_pub_rate',
                'msg': UpdateFloat,
                'qsize': 1,
                'callback': self._setFramePublishRateCb, 
                'callback_args': ()
            },
            'set_frame_fixed_navpose': {
                'namespace': self.node_namespace,
                'topic': 'set_frame_fixed_navpose',
                'msg': UpdateNavPose,
                'qsize': 1,
                'callback': self. _setFrameFixedNavPoseCb, 
                'callback_args': ()
            },
            'clear_frame_fixed_navpose': {
                'namespace': self.node_namespace,
                'topic': 'clear_frame_fixed_navpose',
                'msg': String,
                'qsize': 1,
                'callback': self._clearFrameFixedNavPoseCb, 
                'callback_args': ()
            },
            'set_frame_comp_topic': {
                'namespace': self.node_namespace,
                'topic': 'set_frame_comp_topic',
                'msg': UpdateString,
                'qsize': 1,
                'callback': self._setFrameCompTopicCb, 
                'callback_args': ()
            },
            'set_frame_comp_transform': {
                'namespace': self.node_namespace,
                'topic': 'set_frame_comp_transform',
                'msg': UpdateTransform,
                'qsize': 1,
                'callback': self._setFrameCompTransformCb, 
                'callback_args': ()
            },
            'clear_frame_comp_transform': {
                'namespace': self.node_namespace,
                'topic': 'clear_frame_comp_transform',
                'msg': UpdateString,
                'qsize': 1,
                'callback': self._clearFrameCompTransformCb, 
                'callback_args': ()
            },
            'set_frame_comp_option': {
                'namespace': self.node_namespace,
                'topic': 'set_frame_comp_option',
                'msg': UpdateString,
                'qsize': 1,
                'callback': self._setFrameCompOptionCb, 
                'callback_args': ()
            },
            'reset_frame_comp_init_state': {
                'namespace': self.node_namespace,
                'topic': 'reset_frame_comp_init_state',
                'msg': UpdateString,
                'qsize': 1,
                'callback': self._resetFrameCompInittateCb, 
                'callback_args': ()
            },
            'reset_frame_comp_init_timed_sec': {
                'namespace': self.node_namespace,
                'topic': 'reset_frame_comp_init_timed_sec',
                'msg': UpdateFloat,
                'qsize': 1,
                'callback': self._setFrameCompInitTimedSecCb, 
                'callback_args': ()
            },
            'set_frame_comp_update_resets_on_crossing': {
                'namespace': self.node_namespace,
                'topic': 'set_frame_comp_update_resets_on_crossing',
                'msg': UpdateBool,
                'qsize': 1,
                'callback': self._setFrameCompUpdateResetsOnCrossingCb, 
                'callback_args': ()
            },
            'set_frame_comp_update_resets_crossing': {
                'namespace': self.node_namespace,
                'topic': 'set_frame_comp_update_resets_crossing',
                'msg': UpdateFloat,
                'qsize': 1,
                'callback': self._setFrameCompUpdateResetsCrossingCb, 
                'callback_args': ()
            },
            'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self._systemStatusCb
            }
        }


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        msg_if = self.msg_if
        )

        #ready = self.node_if.wait_for_ready()
        nepi_sdk.wait()

        ##############################
        # Set up save data services ########################################################
        factory_data_rates = {}
        for d in self.data_products_list:
            factory_data_rates[d] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
            if d == 'navposes':
                factory_data_rates[d][0] = self.FACTORY_PUB_RATE_HZ
        sd_namespace = self.base_namespace + '/navposes'  
        self.save_data_if = SaveDataIF(namespace = sd_namespace, data_products = self.data_products_list, factory_rate_dict = factory_data_rates)

        nepi_sdk.sleep(1)

        if self.save_data_if is not None:
            self.status_msg.save_data_topic = self.save_data_if.get_namespace()
            self.navposes_status_msg.save_data_topic = self.status_msg.save_data_topic
            self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic))

        
        ######################
        # initialize variables from param server

        self.initCb(do_updates = True)
        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)
        nepi_sdk.start_timer_process(5.0, self._updateAvailTopicsCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._updateConnectionsCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._getPublishSaveDataCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

        ##############################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        nepi_sdk.spin()


    def initCb(self,do_updates = False):
        if self.node_if is not None:

            self.navposes_max_pub_rate = self.node_if.get_param('navposes_max_pub_rate')

            navposes_info_dict = self.node_if.get_param('navposes_info_dict')
            if navposes_info_dict is not None:
                for key in self.BLANK_CONNECT_DICT.keys():
                    if key not in navposes_info_dict.keys():
                        navposes_info_dict[key] = self.BLANK_CONNECT_DICT[key]
                self.navposes_info_dict = navposes_info_dict

            navposes_fixed_dict = self.node_if.get_param('navposes_fixed_dict')
            if navposes_fixed_dict is not None:
                self.navposes_fixed_dict = navposes_fixed_dict


            self.frame_nav = self.node_if.get_param('frame_nav')
            self.frame_alt = self.node_if.get_param('frame_alt')
            self.frame_depth = self.node_if.get_param('frame_depth')

 
            if do_updates == True:
                navposes_info_dict = copy.deepcopy(self.navposes_info_dict)
                for frame_name in self.navposes_init_frames:
                    if frame_name in navposes_info_dict.keys():
                        navpose_info_dict = navposes_info_dict[frame_name]
                    else:
                        navpose_info_dict = None
                    
                    self.addNavpose(frame_name, init_dict_entry = navpose_info_dict, do_updates = False)
                    
                #self.msg_if.pub_warn("Initializing NavPoses Dict with navpose_info_dict: " + str(self.navposes_info_dict))
                self.updateNavposesData()


            self.publish_status()
            self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
            self.node_if.save_config()

    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    def factoryResetCb(self,do_updates = True):
        self.aifs_classes_dict = dict()
        self.aif_classes_dict = dict()
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    ######################
    ### Navpose Mgr Functions

    def _updaterCb(self, timer):
        self.updateNavposesData()
        nepi_sdk.start_timer_process(1.0, self._updaterCb, oneshot = True)

    def updateNavposesData(self):

        self.navposes_info_dict_lock.acquire()
        navposes_info_dict = copy.deepcopy(self.navposes_info_dict)
        navposes_init_times_dict = copy.deepcopy(self.navposes_init_times_dict)
        navposes_source_times_dict = copy.deepcopy(self.navposes_source_times_dict)
        navposes_update_times_dict = copy.deepcopy(self.navposes_update_times_dict)
        self.navposes_info_dict_lock.release()

        navpose_frames = list(navposes_info_dict.keys())


        navpose_topics = []
        for frame_name in navpose_frames:
            navpose_topics.append(navposes_info_dict[frame_name]['namespace'])
       
        ############
        ## Update System NavPoses Dict
        last_navposes_settings_dict = copy.deepcopy(self.navposes_settings_dict)
        navposes_settings_dict = dict()
        navposes_settings_dict['frame_nav'] = self.frame_nav
        navposes_settings_dict['frame_alt'] = self.frame_alt
        navposes_settings_dict['frame_depth'] = self.frame_depth

        navposes_settings_dict['navpose_options'] = list(navposes_info_dict.keys())     

        navposes_settings_dict['navpose_pub_topics'] = navpose_topics
        self.navposes_settings_dict = navposes_settings_dict
        if last_navposes_settings_dict != navposes_settings_dict:
            nepi_system.set_navposes_dict(navposes_settings_dict)

        
        ############
        # Update Caps Response
        self.caps_response.navpose_frames = navpose_frames
        self.caps_response.navpose_frames_topics = navpose_topics


        ############
        ## Update Status Message
        

        navpose_solution_msg_list = []


        for frame_name in navpose_frames:
    

            if frame_name in navposes_info_dict.keys():
            

                connect_dict = navposes_info_dict[frame_name]['connect_dict']

                #self.msg_if.pub_warn("Updating NavPose dict for frame: " + str(frame_name) + " connect_dict: " + str(connect_dict))
                comp_names = list(self.BLANK_CONNECTS_DICT.keys())



                
                comp_infos_msg = []
                for comp_name in comp_names:

                    comp_info_msg = NavPoseComponent()
                    
                    comp_info_msg.comp_name = comp_name
                    init_topic = connect_dict[comp_name]['init_topic']
                    source_topic = connect_dict[comp_name]['source_topic']
                    update_topic = connect_dict[comp_name]['update_topic']

                    ########
                    # Update comp init topics

                    comp_info_msg.init_topic = init_topic
                    comp_info_msg.init_topic_msg = self.avail_topics_dict['all_topics_dict'].get(init_topic, '')

                    avail_topics_dict = copy.deepcopy(self.avail_topics_dict)
                    available_init_topics = []
                    available_init_topic_msgs = []
                    for i, topic in enumerate(avail_topics_dict[comp_name]['topics']):
                        if topic != init_topic and topic != source_topic:
                            available_init_topics.append(avail_topics_dict[comp_name]['topics'][i])
                            available_init_topic_msgs.append(avail_topics_dict[comp_name]['msgs'][i])
                    comp_info_msg.available_init_topics = available_init_topics
                    comp_info_msg.available_init_topic_msgs = available_init_topic_msgs


                    comp_info_msg.init_topic_available = (init_topic in list(self.avail_topics_dict['all_topics_dict'].keys())) or init_topic == 'Fixed'
                    comp_info_msg.init_topic_connecting = (init_topic in self.navpose_subs_connecting)
                    init_connected = (init_topic in self.navpose_subs_connected) and init_topic != 'Fixed'
                    comp_info_msg.init_topic_connected =  init_connected or init_topic == 'Fixed'

                    if frame_name in navposes_init_times_dict.keys():
                        if comp_name in navposes_init_times_dict[frame_name].keys():
                            times_dict = navposes_init_times_dict[frame_name][comp_name]
                            times = times_dict['times']
                            avg_times = sum(times)/len(times)
                            if avg_times > .01:
                                avg_rate = float(1.0) / avg_times
                            else:
                                avg_rate = 0
                            comp_info_msg.init_topic_avg_rate = round( avg_rate, 3)
                            if avg_rate != 0:
                                comp_info_msg.init_topic_last_time = round( nepi_utils.get_time() - times_dict['last_time'], 3)

                    transform_dict = connect_dict[comp_name]['init_transform']
                    transform_dict['source_ref_description'] = init_topic
                    transform_dict['end_ref_description'] = frame_name
                    transform_dict = nepi_nav.check_tranform_dict(transform_dict)
                    connect_dict[comp_name]['init_transform'] = transform_dict
                    comp_info_msg.init_topic_transform = nepi_nav.convert_transform_dict2msg(transform_dict)

                    comp_info_msg.init_options_list = connect_dict[comp_name]['init_options']
                    comp_info_msg.init_option = connect_dict[comp_name]['init_option']
                    comp_info_msg.init_timed_sec = connect_dict[comp_name]['init_timed_sec']

                    ########
                    # Update comp source topics
                    comp_info_msg.source_topic = source_topic
                    avail_topics_dict = copy.deepcopy(self.avail_topics_dict)
                    available_source_topics = []
                    available_source_topic_msgs = []
                    for i, topic in enumerate(avail_topics_dict[comp_name]['topics']):
                        if topic != init_topic and topic != update_topic:
                            available_source_topics.append(avail_topics_dict[comp_name]['topics'][i])
                            available_source_topic_msgs.append(avail_topics_dict[comp_name]['msgs'][i])
                    comp_info_msg.available_source_topics = available_source_topics
                    comp_info_msg.available_source_topic_msgs = available_source_topic_msgs

                    comp_info_msg.source_topic_available = (source_topic in list(self.avail_topics_dict['all_topics_dict'].keys()))
                    comp_info_msg.source_topic_connecting = (source_topic in self.navpose_subs_connecting)
                    comp_info_msg.source_topic_connected =  (source_topic in self.navpose_subs_connected)

                    if frame_name in navposes_source_times_dict.keys():
                        if comp_name in navposes_source_times_dict[frame_name].keys():
                            times_dict = navposes_source_times_dict[frame_name][comp_name]
                            times = times_dict['times']
                            avg_times = sum(times)/len(times)
                            if avg_times > .01:
                                avg_rate = float(1.0) / avg_times
                            else:
                                avg_rate = 0
                            comp_info_msg.source_topic_avg_rate = round( avg_rate, 3)
                            if avg_rate != 0:
                                comp_info_msg.source_topic_last_time = round( nepi_utils.get_time() - times_dict['last_time'], 3)

                    transform_dict = connect_dict[comp_name]['source_transform']
                    transform_dict['source_ref_description'] = source_topic
                    transform_dict['end_ref_description'] = frame_name
                    transform_dict = nepi_nav.check_tranform_dict(transform_dict)
                    connect_dict[comp_name]['source_transform'] = transform_dict
                    comp_info_msg.source_topic_transform = nepi_nav.convert_transform_dict2msg(transform_dict)

                    comp_info_msg.source_options_list = connect_dict[comp_name]['source_options']
                    comp_info_msg.source_option = connect_dict[comp_name]['source_option']

                    ########
                    # Update Update topics

                    comp_info_msg.update_topic = update_topic

                    avail_topics_dict = copy.deepcopy(self.avail_topics_dict)
                    available_update_topics = []
                    available_update_topic_msgs = []
                    for i, topic in enumerate(avail_topics_dict[comp_name]['topics']):
                        if topic != init_topic and topic != source_topic:
                            available_update_topics.append(avail_topics_dict[comp_name]['topics'][i])
                            available_update_topic_msgs.append(avail_topics_dict[comp_name]['msgs'][i])
                    comp_info_msg.available_update_topics = available_update_topics
                    comp_info_msg.available_update_topic_msgs = available_update_topic_msgs


                    comp_info_msg.update_topic_available = (update_topic in list(self.avail_topics_dict['all_topics_dict'].keys()))
                    comp_info_msg.update_topic_connecting = (update_topic in self.navpose_subs_connecting)
                    update_connected = (update_topic in self.navpose_subs_connected)
                    comp_info_msg.update_topic_connected =  update_connected

                    comp_info_msg.update_resets_on_crossing =  connect_dict[comp_name]['update_resets_on_crossing']
                    comp_info_msg.update_resets_crossing =  connect_dict[comp_name]['update_resets_crossing']


                    if frame_name in navposes_update_times_dict.keys():
                        if comp_name in navposes_update_times_dict[frame_name].keys():
                            times_dict = navposes_update_times_dict[frame_name][comp_name]
                            times = times_dict['times']
                            avg_times = sum(times)/len(times)
                            if avg_times > .01:
                                avg_rate = float(1.0) / avg_times
                            else:
                                avg_rate = 0
                            comp_info_msg.update_topic_avg_rate = round( avg_rate, 3)
                            if avg_rate != 0:
                                comp_info_msg.update_topic_last_time = round( nepi_utils.get_time() - times_dict['last_time'], 3)

                    transform_dict = connect_dict[comp_name]['update_transform']
                    transform_dict['source_ref_description'] = update_topic
                    transform_dict['end_ref_description'] = frame_name
                    transform_dict = nepi_nav.check_tranform_dict(transform_dict)
                    connect_dict[comp_name]['update_transform'] = transform_dict
                    comp_info_msg.update_topic_transform = nepi_nav.convert_transform_dict2msg(transform_dict)

                    comp_info_msg.update_options_list = connect_dict[comp_name]['update_options']
                    comp_info_msg.update_option = connect_dict[comp_name]['update_option']

                    #####
                    # Update has comp
                    comp_info_msg.has_component = comp_info_msg.init_topic_connected
                    comp_infos_msg.append(comp_info_msg)

                solution_msg = NavPoseSolution()
                solution_msg.frame_name = frame_name

                namespace = navposes_info_dict[frame_name]['namespace'] 
                solution_msg.frame_namespace = namespace     
                solution_msg.navpose_topic = os.path.join(namespace,'navpose')
                solution_msg.navpose_fixed_topic = os.path.join(namespace,'navpose_fixed')
                navpose_component_topics = []
                for navpose_name in self.NAVPOSE_COMPONENT_NAMES:
                    navpose_component_topics.append( os.path.join(namespace,navpose_name))
                solution_msg.navpose_component_topics = navpose_component_topics


                states_true = 0
                states_false = 0
                if frame_name in self.navposes_init_states_dict.keys():
                    states_dict = self.navposes_init_states_dict[frame_name]
                    for comp_name in states_dict.keys():
                        if states_dict[comp_name] == False:
                            states_false += 1                
                        elif states_dict[comp_name] == True:
                            states_true += 1
                solution_msg.has_inited = states_false == 0 and states_true > 0

                states_true = 0
                states_false = 0
                if frame_name in self.navposes_source_states_dict.keys():
                    states_dict = self.navposes_source_states_dict[frame_name]
                    for comp_name in states_dict.keys():
                        if states_dict[comp_name] == False:
                            states_false += 1                
                        elif states_dict[comp_name] == True:
                            states_true += 1
                solution_msg.has_sourced = states_false == 0 and states_true > 0

                states_true = 0
                states_false = 0
                if frame_name in self.navposes_update_states_dict.keys():
                    states_dict = self.navposes_update_states_dict[frame_name]
                    for comp_name in states_dict.keys():
                        if states_dict[comp_name] == False:
                            states_false += 1                
                        elif states_dict[comp_name] == True:
                            states_true += 1          
                solution_msg.has_updated = states_false == 0 and states_true > 0

                state = False
                if frame_name in self.navposes_info_dict.keys():
                    info_dict = self.navposes_info_dict[frame_name]
                    if 'connect_dict' in info_dict.keys():
                        for comp_name in info_dict['connect_dict'].keys():
                            if info_dict['connect_dict'][comp_name]['init_topic'] == 'Fixed':
                                state = True            
                solution_msg.has_fixed = state


                state = False
                if frame_name in self.navposes_solution_dict.keys():
                    navpose_dict = self.navposes_solution_dict[frame_name]
                    state = navpose_dict['has_pan_tilt']           
                solution_msg.has_pan_tilt = state

                solution_msg.pan_tilt_adjusted = navposes_info_dict[frame_name]['pan_tilt_adjusted']
               
                solution_msg.max_pub_rate = navposes_info_dict[frame_name]['max_pub_rate']

                if frame_name in self.navposes_pub_times_dict.keys():
                    times_dict = navposes_init_times_dict[frame_name][comp_name]
                    times = times_dict['times']
                    avg_times = sum(times)/len(times)
                    if avg_times > .01:
                        avg_rate = float(1.0) / avg_times
                    else:
                        avg_rate = 0
                    solution_msg.avg_pub_rate = round( avg_rate, 3)




                solution_msg.components_list = comp_names
                solution_msg.components_info = comp_infos_msg

                navpose_solution_msg_list.append(solution_msg)


        self.status_msg.navpose_frames = navpose_frames
        self.status_msg.navpose_frames_topics = navpose_topics
        self.status_msg.navpose_frames_solutions = navpose_solution_msg_list

        for frame_name in self.navposes_info_dict.keys():
            try:
                self.navposes_info_dict[frame_name]['ready'] = True
            except:
                pass

        ############
        ## Update Navposes Status Message
        self.navposes_status_msg.navpose_frames = navpose_frames
        self.navposes_status_msg.navpose_frames_topics = navpose_topics

        ###########
        ## Update navposes data
        if self.last_navposes_info_dict != self.navposes_info_dict:
            self.publish_status()
            self.update_navposes = True
            if self.node_if is not None:
                self.node_if.save_config()
        self.last_navposes_info_dict = copy.deepcopy(self.navposes_info_dict)


    def registerCompTopic(self, frame_name, comp_name, topic):
        if topic != '' and topic != 'None' and topic != 'Fixed':
            if topic not in self.navpose_subs_dict.keys():
                    sub_dict = copy.deepcopy(self.BLANK_SUB)
                    sub_dict['topic'] = topic
                    sub_dict['msg'] = ''
                    sub_dict['subs_dict'][frame_name] = [comp_name]
                    sub_dict['has_subs'] = True
                    self.navpose_subs_dict[topic] = sub_dict
            else:
                    if frame_name not in self.navpose_subs_dict[topic]['subs_dict'].keys():
                        self.navpose_subs_dict[topic]['subs_dict'][frame_name] = [comp_name]
                    elif comp_name not in self.navpose_subs_dict[topic]['subs_dict'][frame_name]:
                        self.navpose_subs_dict[topic]['subs_dict'][frame_name].append(comp_name)
            if topic not in self.navpose_subs_registered:
                self.navpose_subs_registered.append(topic)

    def subscribeTopic(self, topic):
        success = self.unsubscribeTopic(topic) # Just in case it is registered
        if success == True and topic != '' and topic != 'None' and topic != 'Fixed' and topic in self.navpose_subs_dict.keys():
                if self.navpose_subs_dict[topic]['sub'] is None:
                    msg = None
                    if topic in self.avail_topics_dict['all_topics_dict'].keys():
                        msg_type_str = self.avail_topics_dict['all_topics_dict'][topic]
                        # Resolve type string to message class (rospy.Subscriber requires class, not string)
                        for comp_dict in nepi_nav.NAVPOSE_MSG_DICT.values():
                            if msg_type_str in comp_dict:
                                msg = comp_dict[msg_type_str]
                                break
                    if msg is not None and topic not in self.navpose_subs_connecting:
                        if topic not in self.navpose_subs_connecting:
                            self.navpose_subs_connecting.append(topic)
                        try:
                            self.navpose_subs_connected.remove(topic)
                        except:
                            pass
                        topic_sub = nepi_sdk.create_subscriber(topic, msg, self._compSubCb, queue_size = 1, callback_args= (topic), log_name_list = [])

                        self.navpose_subs_dict_lock.acquire()
                        self.navpose_subs_dict[topic]['sub'] = topic_sub
                        self.navpose_subs_dict_lock.release()
                       
                        success = True
        return success




    def addNavpose(self, frame_name, description = None, init_dict_entry = None, reset = False, do_updates = True):
        self.msg_if.pub_warn("Adding navpose frame: " + str(frame_name))

        if description is None:
            description = frame_name

        data_product = nepi_utils.get_clean_name(frame_name)

        # Reset frame data
        self.msg_if.pub_info("Initializing navpose data for: " + str(frame_name))
        self.navposes_source_dict_lock.acquire()
        self.navposes_source_dict[frame_name] = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        self.navposes_source_dict_lock.release()

        navposes_fixed_dict = copy.deepcopy(self.navposes_fixed_dict)
        ### Update From Fixed Navpose
        navpose_fixed = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        if frame_name in navposes_fixed_dict.keys():
            navpose_fixed = navposes_fixed_dict[frame_name]
        

                
        if frame_name not in self.navposes_info_dict.keys() or (reset == True and frame_name in self.navposes_info_dict.keys()):
            self.msg_if.pub_info("Updating navpose entry for: " + str(frame_name))
            if init_dict_entry is None or reset == True:
                info_dict_entry = dict()
                info_dict_entry['frame_name'] = frame_name
                info_dict_entry['data_product'] = data_product
                info_dict_entry['description'] = description
                info_dict_entry['max_pub_rate'] = self.FACTORY_PUB_RATE_HZ
                info_dict_entry['connect_dict'] = copy.deepcopy(self.BLANK_CONNECTS_DICT)
                info_dict_entry['pan_tilt_adjusted'] = False

            elif init_dict_entry is not None:
                init_frame_name = init_dict_entry['frame_name']
                info_dict_entry = copy.deepcopy(init_dict_entry)
                info_dict_entry['frame_name'] = frame_name
                info_dict_entry['data_product'] = data_product
                info_dict_entry['description'] = frame_name
                connects_dict = copy.deepcopy(self.BLANK_CONNECTS_DICT)
                connect_dict = copy.deepcopy(self.BLANK_CONNECT_DICT)
                for comp_name in connects_dict.keys():
                    if comp_name not in info_dict_entry['connect_dict'].keys():
                        info_dict_entry['connect_dict'][comp_name] = connects_dict[comp_name]
                    else:
                        for key in connect_dict.keys():
                            if key not in info_dict_entry['connect_dict'][comp_name].keys():
                                info_dict_entry['connect_dict'][comp_name][key] = connect_dict[key]
                if init_frame_name in navposes_fixed_dict.keys():
                    navpose_fixed = navposes_fixed_dict[init_frame_name]



            clean_name = nepi_utils.get_clean_name(frame_name)
            namespace = self.base_namespace + '/navposes/' +  clean_name 
            info_dict_entry['namespace'] = namespace

            info_dict_entry['ready'] = False
            self.msg_if.pub_info("Registering new navpose: " + str(frame_name))
            self.navposes_info_dict_lock.acquire()
            self.navposes_info_dict[frame_name] = info_dict_entry
            self.navposes_info_dict_lock.release()

            self.navposes_fixed_dict[frame_name] = navpose_fixed

            

        ###################
        # Update other navpose frame required components
        if frame_name in self.navposes_info_dict.keys():



            self.navposes_info_dict_lock.acquire()
            info_dict_entry = copy.deepcopy(self.navposes_info_dict[frame_name])
            self.navposes_info_dict_lock.release()


            self.msg_if.pub_warn("Updating navpose components for: " + str(frame_name) ) #+ ' : ' + str(info_dict_entry))
            self.navposes_info_dict[frame_name]['data_product'] = data_product

            if description is not None:
                self.navposes_info_dict[frame_name]['description'] = description

            

            ###################
            # Add to navposes_pubs_dict      
            namespace = info_dict_entry['namespace']       
            if frame_name not in self.navposes_pubs_dict.keys():
                self.msg_if.pub_info("Creating navpose publishers entries for: " + str(frame_name) + ' : ' + str(data_product))
                # Publishers Config Dict ####################
                PUBS_DICT = dict()

                PUBS_DICT['navpose'] = {
                    'namespace': namespace,
                    'topic': 'navpose',
                    'msg': NavPose,
                    'qsize': 1,
                }
                PUBS_DICT['navpose_fixed'] = {
                    'namespace': namespace,
                    'topic': 'navpose_fixed',
                    'msg': NavPose,
                    'qsize': 1,
                }
                for navpose_name in self.NAVPOSE_COMPONENT_NAMES:
                    PUBS_DICT[navpose_name] = {
                        'namespace': namespace,
                        'topic': navpose_name,
                        'msg': NavPose,
                        'qsize': 1,
                    }
                pubs_if = NodePublishersIF(PUBS_DICT,
                                           log_name = frame_name,
                                            msg_if = self.msg_if)

                ready = pubs_if.wait_for_ready()

                self.navposes_pubs_dict_lock.acquire()
                self.navposes_pubs_dict[frame_name] = pubs_if
                self.navposes_pubs_dict_lock.release()


            #######################
            # Initialize NavPose Solution Dict
            self.navposes_pub_times_dict[frame_name] = copy.deepcopy(self.BLANK_TIMES)
            self.resetFrameNavpose(frame_name)
        
            ###################
            # Register frame topics if set
            for comp_name in self.BLANK_CONNECTS_DICT.keys():
                try:
                    info_dict_entry['connect_dict'][comp_name]['connected'] = False
                    info_dict_entry['connect_dict'][comp_name]['avg_rate'] = 0
                    for type_name in ('init_topic', 'source_topic', 'update_topic'):
                        topic = info_dict_entry['connect_dict'][comp_name][type_name]
                        self.registerCompTopic(frame_name, comp_name, topic)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to register topic for frame: " + str(frame_name) + ' : ' + str(comp_name) + ' : ' + str(e))


            ##################
            # Saved Fixed if Changed
            if navposes_fixed_dict != self.navposes_fixed_dict:
                if self.node_if is not None:
                    self.node_if.set_param('navposes_fixed_dict',self.navposes_fixed_dict)
                    self.node_if.save_config()

            if do_updates == True:
                self.updateNavposesData()




    def removeNavpose(self, frame_name):
        if frame_name == self.NAVPOSE_BASE_FRAME:
            self.msg_if.pub_info("Can't Remove Base Frame: " + str(frame_name))
        elif frame_name not in self.navposes_info_dict.keys():
            self.msg_if.pub_info("NavPose entry does not exist for: " + str(frame_name))
        else:


            # Unregister Register frame topics if set
            topics = list(self.navpose_subs_dict.keys())
            for comp_name in self.BLANK_CONNECTS_DICT.keys():
                for topic in topics:
                    try:
                        self.unregisterCompTopic(frame_name, comp_name, topic)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to unregister topic for frame: " + str(frame_name) + ' : ' + str(comp_name) + ' : ' + str(topic) + ' : ' + str(e))


            # Remove from info dict
            self.navposes_info_dict_lock.acquire()
            del self.navposes_info_dict[frame_name]
            self.navposes_info_dict_lock.release()
            nepi_sdk.sleep(1)


            # Remove from node dict
            self.navposes_pubs_dict_lock.acquire()
            self.navposes_pubs_dict[frame_name].unregister_pubs()
            nepi_sdk.sleep(1)
            del self.navposes_pubs_dict[frame_name]
            self.navposes_pubs_dict_lock.release()

            # Remove from Navpose Dict
            self.navposes_source_dict_lock.acquire()
            del self.navposes_source_dict[frame_name]
            self.navposes_source_dict_lock.release()

            self.updateNavposesData()





    def unregisterCompTopic(self, frame_name, comp_name, topic):
        if topic != '' and topic != 'None' and topic != 'Fixed' and topic in self.navpose_subs_dict.keys():
            if frame_name in self.navpose_subs_dict[topic]['subs_dict'].keys():
                if comp_name in self.navpose_subs_dict[topic]['subs_dict'][frame_name]:
                    self.navpose_subs_dict[topic]['subs_dict'][frame_name].remove(comp_name)
                    if len(self.navpose_subs_dict[topic]['subs_dict'][frame_name]) == 0:
                        del self.navpose_subs_dict[topic]['subs_dict'][frame_name]

            if len(self.navpose_subs_dict[topic]['subs_dict'].keys()) == 0:
                if topic in self.navpose_subs_registered:
                    self.navpose_subs_registered.remove(topic)



    def unsubscribeTopic(self, topic):
        success = True
        if topic != '' and topic != 'None' and topic != 'Fixed' and topic in self.navpose_subs_dict.keys():
                if self.navpose_subs_dict[topic]['sub'] is not None:

                        self.navpose_subs_dict_lock.acquire()

                        try:
                            self.navpose_subs_dict[topic]['sub'].unregister()
                        except:
                            success = False
                        nepi_sdk.sleep(1)
                        self.navpose_subs_dict[topic]['sub'] = None

                        self.navpose_subs_dict_lock.release()       

                try:
                    self.navpose_subs_connecting.remove(topic)
                except:
                    pass
    
                try:
                    self.navpose_subs_connected.remove(topic)
                except:
                    pass
        return success





    #######################
    ### Node Methods


           
    def setFrameNav(self,frame_name):
        self.frame_nav = frame_name
        self.publish_status()
        self.updateNavposesData()
        if self.node_if is not None:
            self.node_if.set_param('frame_nav',frame_name)
            self.node_if.save_config()

    def setFrameAlt(self,frame_name):
        self.frame_alt = frame_name
        self.publish_status()
        self.updateNavposesData()
        if self.node_if is not None:
            self.node_if.set_param('frame_alt',frame_name)
            self.node_if.save_config()

    def setFrameDepth(self,frame_name):
        self.frame_depth = frame_name
        self.publish_status()
        self.updateNavposesData()
        if self.node_if is not None:
            self.node_if.set_param('frame_depth',frame_name)
            self.node_if.save_config()
     
    def resetFrameNavpose(self, frame_name):
        if frame_name in self.navposes_info_dict.keys():
            self.navposes_init_states_dict[frame_name] = copy.deepcopy(self.BLANK_STATES_DICT)
            self.navposes_init_times_dict[frame_name] = copy.deepcopy(self.BLANK_TIMES_DICT)
            self.navposes_init_dict[frame_name] = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)


            self.navposes_source_states_dict[frame_name] = copy.deepcopy(self.BLANK_STATES_DICT)
            self.navposes_source_times_dict[frame_name] = copy.deepcopy(self.BLANK_TIMES_DICT)
            self.navposes_source_dict[frame_name] = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

            self.navposes_update_states_dict[frame_name] = copy.deepcopy(self.BLANK_STATES_DICT)
            self.navposes_update_times_dict[frame_name] = copy.deepcopy(self.BLANK_TIMES_DICT)
            self.navposes_update_dict[frame_name] = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
            self.navposes_update_offset_dict[frame_name] = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
            self.navposes_update_reset_dict[frame_name] = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

            self.navposes_solution_dict[frame_name] = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
            self.publish_status()
            self.updateNavposesData()
        else:
            self.publish_status()


    def setFrameDescription(self,frame_name, desc):
        if frame_name in self.navposes_info_dict.keys():
            self.navposes_info_dict[frame_name]['description'] = desc
            self.publish_status()
            self.updateNavposesData()
            if self.node_if is not None:
                self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
                self.node_if.save_config()
        else:
            self.publish_status()


    def setNavPosesPubRate(self, rate):
        min = self.NAVPOSE_PUB_RATE_OPTIONS[0]
        max = self.NAVPOSE_PUB_RATE_OPTIONS[1]
        if rate < min:
            rate = min
        if rate > max:
            rate = max
        self.navposes_max_pub_rate = rate
        self.publish_status()
        self.updateNavposesData()
        if self.node_if is not None:
            self.node_if.set_param('navposes_max_pub_rate',self.navposes_max_pub_rate)
            self.node_if.save_config()


    def setFramePublishRate(self, frame_name, rate):
        min = self.NAVPOSE_PUB_RATE_OPTIONS[0]
        max = self.NAVPOSE_PUB_RATE_OPTIONS[1]
        if rate < min:
            rate = min
        if rate > max:
            rate = max
        if frame_name in self.navposes_info_dict.keys():
            self.navposes_info_dict[frame_name]['max_pub_rate'] = rate
            self.publish_status()
            self.updateNavposesData()
            if self.node_if is not None:
                self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
                self.node_if.save_config()
        else:
            self.publish_status()


    def setFrameCompTopic(self, frame_name, comp_name, type_name, topic):
        if frame_name in self.navposes_info_dict.keys():
            avail_topics_dict = copy.deepcopy(self.avail_topics_dict)
            if comp_name in self.navposes_info_dict[frame_name]['connect_dict'].keys() and (type_name == 'init' or type_name == 'source' or type_name == 'update'):


                cur_init_topic = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_topic']
                cur_source_topic = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['source_topic']
                cur_update_topic = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_topic']


                if type_name == 'init':
                    if (topic == 'None' or topic == '' or topic == 'Fixed') or ( cur_init_topic != topic and cur_source_topic != topic and cur_update_topic != topic):

                        self.unregisterCompTopic(frame_name, comp_name, cur_init_topic)
                        self.resetFrameNavpose(frame_name)

                        ## Update Connect Dict
                        if topic == 'Fixed':
                            self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_connected'] = True
                            self.navposes_init_states_dict[frame_name][comp_name] = True
                            self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_topic'] = 'Fixed'

                        elif topic == 'None' or topic == '':
                            self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_connected'] = False
                            self.navposes_init_states_dict[frame_name][comp_name] = None
                            self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_topic'] = 'None'
                        elif comp_name in avail_topics_dict.keys():
                            if topic in avail_topics_dict[comp_name]['topics']:
                                self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_connected'] = False
                                self.navposes_init_states_dict[frame_name][comp_name] = False
                                self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_topic'] = topic

                                self.registerCompTopic(frame_name, comp_name, topic)
                                self.subscribeTopic(topic)

                elif type_name == 'source':

                    if (topic == 'None' or topic == '') or ( cur_init_topic != topic and cur_source_topic != topic and cur_update_topic != topic):
                        self.unregisterCompTopic(frame_name, comp_name, cur_source_topic)
                        self.resetFrameNavpose(frame_name)

                        if topic == 'None' or topic == '':
                            self.navposes_source_states_dict[frame_name][comp_name] = None
                            self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['source_topic'] = 'None'
                        elif comp_name in avail_topics_dict.keys():
                            if topic in avail_topics_dict[comp_name]['topics']:
                                self.navposes_source_states_dict[frame_name][comp_name] = False
                                self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['source_topic'] = topic

                                self.registerCompTopic(frame_name, comp_name, topic)
                                self.subscribeTopic(topic)

                elif type_name == 'update':

                    if (topic == 'None' or topic == '') or ( cur_init_topic != topic and cur_source_topic != topic and cur_update_topic != topic):
                        self.unregisterCompTopic(frame_name, comp_name, cur_update_topic)
                        self.resetFrameNavpose(frame_name)

                        if topic == 'None' or topic == '':
                            self.navposes_update_states_dict[frame_name][comp_name] = None
                            self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_topic'] = 'None'
                        elif comp_name in avail_topics_dict.keys():
                            if topic in avail_topics_dict[comp_name]['topics']:
                                self.navposes_update_states_dict[frame_name][comp_name] = False
                                self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_topic'] = topic

                                self.registerCompTopic(frame_name, comp_name, topic)
                                self.subscribeTopic(topic)


                self.publish_status()
                self.updateNavposesData()
                if self.node_if is not None:
                    self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
                    self.node_if.save_config()
        else:
            self.publish_status()

    def clearFrameCompTopic(self,frame_name, comp_name, type_name):
        self.setFrameCompTopic(frame_name, comp_name, type_name, 'None')


    def setFrameCompTransform(self, frame_name, comp_name, type_name, transform_dict):
        if frame_name in self.navposes_info_dict.keys():
            if comp_name in self.navposes_info_dict[frame_name]['connect_dict'].keys():
                if type_name == 'init':
                    self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_transform'] = transform_dict
                elif type_name == 'source':
                    self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['source_transform'] = transform_dict
                elif type_name == 'update':
                    self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_transform'] = transform_dict
                self.publish_status()
                self.updateNavposesData()
                if self.node_if is not None:
                    self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
                    self.node_if.save_config()
        else:
            self.publish_status()

    def clearFrameCompTransform(self,frame_name, comp_name, type_name):
        transform_dict = copy.deepcopy(self.ZERO_TRANSFORM_DICT)
        self.setFrameCompTransform(frame_name, comp_name, type_name, transform_dict)





    def setFrameCompOption(self,frame_name, comp_name, type_name, option):
        if frame_name in self.navposes_info_dict.keys():
            if comp_name in self.navposes_info_dict[frame_name]['connect_dict'].keys():
                if type_name == 'init':
                    options = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_options']
                    if option in options:
                        self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_option'] = option
                elif type_name == 'source':
                    options = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['source_options']
                    if option in options:
                        self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['source_option'] = option
                elif type_name == 'update':
                    options = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_options']
                    if option in options:
                        self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_option'] = option
                self.publish_status()
                self.updateNavposesData()
                if self.node_if is not None:
                    self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
                    self.node_if.save_config()
        else:
            self.publish_status()

    def resetFrameCompInittate(self,frame_name, comp_name):
        if frame_name in self.navposes_init_states_dict.keys():
            if comp_name in self.navposes_init_states_dict[frame_name].keys():
                if self.navposes_init_states_dict[frame_name][comp_name] == True:
                    self.navposes_init_states_dict[frame_name][comp_name] = False
                self.publish_status()
                self.updateNavposesData()
        else:
            self.publish_status()





    def setFrameCompInitTimedSec(self,frame_name, comp_name, delay):
        if frame_name in self.navposes_info_dict.keys():
            if comp_name in self.navposes_info_dict[frame_name]['connect_dict'].keys():
                if delay < 0.1:
                    delay = 0.1
                self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_timed_sec'] = delay
                self.publish_status()
                self.updateNavposesData()
                if self.node_if is not None:
                    self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
                    self.node_if.save_config()
        else:
            self.publish_status()



    def setFrameCompUpdateResetsOnCrossing(self, frame_name, comp_name, enabled):
        if frame_name in self.navposes_info_dict.keys():
            if comp_name in self.navposes_info_dict[frame_name]['connect_dict'].keys():
                self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_resets_on_crossing'] = enabled
                self.publish_status()
                self.updateNavposesData()
                if self.node_if is not None:
                    self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
                    self.node_if.save_config()
        else:
            self.publish_status()

    def setFrameCompUpdateResetsCrossing(self, frame_name, comp_name, crossing):
        if frame_name in self.navposes_info_dict.keys():
            if comp_name in self.navposes_info_dict[frame_name]['connect_dict'].keys():
                self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_resets_crossing'] = crossing
                self.publish_status()
                self.updateNavposesData()
                if self.node_if is not None:
                    self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
                    self.node_if.save_config()
        else:
            self.publish_status()
    ###############################################
 

    def setFrameFixedNavPose(self, frame_name, navpose_dict = None):
        if navpose_dict is None:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        if frame_name in self.navposes_info_dict.keys():
            self.navposes_fixed_dict[frame_name] = navpose_dict
            self.publish_status()
            self.update_navposes = True
            if self.node_if is not None:
                self.node_if.set_param('navposes_fixed_dict',self.navposes_fixed_dict)
                self.node_if.save_config()


    def clearFrameFixedNavPose(self, frame_name):
        self.setFrameFixedNavPose(frame_name)



    def get_navpose_dict(self, frame_name):
        navpose_dict = None
        if frame_name in self.navposes_solution_dict.keys():
            self.navposes_source_dict_lock.acquire() 
            navpose_dict = copy.deepcopy(self.navposes_solution_dict[frame_name])
            self.navposes_source_dict_lock.release()
        return navpose_dict

    def get_navposes_dict(self):
        return self.navposes_solution_dict        



    #######################
    # Private Members
    #######################

    def _systemStatusCb(self,msg):
        self.active_nodes = msg.active_nodes
        self.active_topics = msg.active_topics
        self.active_topic_types = msg.active_topic_types
        self.active_services = msg.active_services

    def _navposeCapsQueryHandler(self,req):
        return self.caps_response

    def _navposesDataQueryHandler(self,req):
        self.navposes_source_dict_lock.acquire()
        navposes_solution_dict = copy.deepcopy(self.navposes_solution_dict)
        self.navposes_source_dict_lock.release()
        response = NavPosesQueryResponse()
        response.navpose_frames = list(navposes_solution_dict.keys())
        navpose_frames_data = []
        for frame_name in navposes_solution_dict.keys():
            navpose_dict = navposes_solution_dict[frame_name]
            navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict) 
            navpose_frames_data.append(navpose_msg)
        response.navpose_frames_data = navpose_frames_data
        return response


    def _updateConnectionsCb(self, timer):
        avail_topics = list(self.avail_topics_dict['all_topics_dict'].keys())
        registered_topics = copy.deepcopy(self.navpose_subs_registered)
        connecting_topics = copy.deepcopy(self.navpose_subs_connecting)
        connected_topics = copy.deepcopy(self.navpose_subs_connected)
        pub_status = False

        # First Purge Connections if Needed
        purge_list = []
        for topic in (connecting_topics + connected_topics):
            if topic not in avail_topics:
                if topic not in purge_list:
                    purge_list.append(topic)
            if topic not in registered_topics:
                if topic not in purge_list:
                    purge_list.append(topic)
        for topic in purge_list:
            self.unsubscribeTopic(topic)
            pub_status = True


        # Subscribe Topics if Needed
        for topic in registered_topics:
            if topic not in (connecting_topics + connected_topics):
                self.subscribeTopic(topic)
                pub_status = True
        
        if pub_status == True:
            self.publish_status()

        nepi_sdk.start_timer_process(1.0, self._updateConnectionsCb, oneshot = True)


    def _updateAvailTopicsCb(self,timer):
        last_dict = copy.deepcopy(self.avail_topics_dict)
        all_topics_dict = dict()
        # Query fresh topic list directly from ROS master rather than using the cached
        # active_topics from system status, which may be stale or incomplete.
        [cur_topics, cur_types] = nepi_sdk.get_topics_data_list()
        for comp_name in nepi_nav.NAVPOSE_MSG_DICT.keys():
            [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces(comp_name,cur_topics,cur_types)
            for i, topic in enumerate(topic_list):
                all_topics_dict[topic] = msg_list[i]
            self.avail_topics_dict[comp_name]['topics'] = copy.deepcopy(topic_list)
            self.avail_topics_dict[comp_name]['msgs'] = copy.deepcopy(msg_list)


        self.avail_topics_dict['all_topics_dict'] = all_topics_dict
        if self.avail_topics_dict != last_dict:
            self.publish_status()
        nepi_sdk.start_timer_process(5.0, self._updateAvailTopicsCb, oneshot = True)




    def _setNavPosesPubRateCb(self,msg):
        rate = msg.data
        self.setNavPosesPubRate(rate)
  

    def _setFrameNavCb(self,msg):
        self.setFrameNav(msg.data) 

    def _setFrameAltCb(self,msg):
        self.setFrameAlt(msg.data) 

    def _setFrameDepthCb(self,msg):
        self.setFrameDepth(msg.data) 

    def _addFrameCb(self,msg):
        frame_name = msg.name
        desc = msg.value
        self.addNavpose(frame_name, description = desc) 
    

    def _copyFrameCb(self,msg):
        frame_name = msg.name
        source = msg.value
        self.navposes_info_dict_lock.acquire()
        navposes_info_dict = copy.deepcopy(self.navposes_info_dict)
        self.navposes_info_dict_lock.release()
        if source in navposes_info_dict.keys():
            info_dict_entry = navposes_info_dict[source]
            self.addNavpose(frame_name, init_dict_entry = info_dict_entry)


    def _deleteFrameCb(self,msg):
        frame_name = msg.data
        self.removeNavpose(frame_name)

    def _resetFrameCb(self,msg):
        frame_name = msg.data
        self.addNavpose(frame_name, reset = True) 

    def _resetFrameNavposeCb(self,msg):
        frame_name = msg.data
        self.resetFrameNavpose(frame_name) 


    def _setFramePublishRateCb(self,msg):
        frame_name = msg.name
        value = msg.value
        self.setFramePublishRate(frame_name, value)
       


    def _setFrameDescriptionCb(self,msg):
        frame_name = msg.name
        value = msg.value
        self.setFrameDescription(frame_name, value)


    def _setFrameCompTopicCb(self,msg):
        frame_name = msg.name
        comp_name = msg.name2
        type_name = msg.name3
        topic = msg.value
        self.setFrameCompTopic(frame_name, comp_name, type_name, topic)
       
    def _setFrameCompTransformCb(self,msg):
        frame_name = msg.name
        comp_name = msg.name2
        type_name = msg.name3
        transform_dict = nepi_nav.convert_transform_msg2dict(msg.transform)
        self.setFrameCompTransform(frame_name, comp_name, type_name, transform_dict)

    def _clearFrameCompTransformCb(self,msg):
        frame_name = msg.name
        comp_name = msg.name2
        type_name = msg.name3
        transform_dict = copy.deepcopy(self.ZERO_TRANSFORM_DICT)
        self.setFrameCompTransform(frame_name, comp_name, type_name, transform_dict)

    def _setFrameCompOptionCb(self,msg):
        frame_name = msg.name
        comp_name = msg.name2
        type_name = msg.name3
        option = msg.value
        self.setFrameCompOption(frame_name, comp_name, type_name, option)

    def _resetFrameCompInittateCb(self,msg):
        frame_name = msg.name
        comp_name = msg.name2
        self.resetFrameCompInittate(frame_name, comp_name)


    def _setFrameCompInitTimedSecCb(self,msg):
        frame_name = msg.name
        comp_name = msg.name2
        delay = msg.value
        self.setFrameCompInitTimedSec(frame_name, comp_name, delay)


    def _setFrameCompUpdateResetsOnCrossingCb(self,msg):
        frame_name = msg.name
        comp_name = msg.name2
        enabled = msg.value
        self.setFrameCompUpdateResetsOnCrossing(frame_name, comp_name, enabled)

    def _setFrameCompUpdateResetsCrossingCb(self,msg):
        frame_name = msg.name
        comp_name = msg.name2
        crossing = msg.value
        self.setFrameCompUpdateResetsCrossing(frame_name, comp_name, crossing)

    def  _setFrameFixedNavPoseCb(self,msg):
        frame_name = msg.name
        navpose_dict = nepi_nav.convert_navpose_msg2dict(msg.navpose)
        if navpose_dict is not None:
            self.setFrameFixedNavPose(frame_name, navpose_dict)
       
    def _clearFrameFixedNavPoseCb(self,msg):
        frame_name = msg.data
        self.clearFrameFixedNavPose(frame_name)




    def _compSubCb(self, msg, args):
        topic = args
        if topic not in self.navpose_subs_connected:
            self.navpose_subs_connected.append(topic)

        try:
            self.navpose_subs_connecting.remove(topic)
        except:
            pass

        if topic in self.navpose_subs_dict.keys():
            frame_names = list(self.navpose_subs_dict[topic]['subs_dict'].keys())
            for frame_name in frame_names:
                if frame_name != 'None' and frame_name in self.navposes_source_dict.keys() and frame_name in self.navposes_info_dict.keys():
                    comp_names = self.navpose_subs_dict[topic]['subs_dict'][frame_name]
                    for comp_name in comp_names:
                        cur_init_topic = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_topic']
                        cur_source_topic = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['source_topic']
                        cur_update_topic = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_topic']


                        self.navposes_init_dict_lock.acquire()
                        navposes_init_dict = copy.deepcopy(self.navposes_init_dict)
                        self.navposes_init_dict_lock.release()
                    
                        self.navposes_source_dict_lock.acquire()
                        navposes_source_dict = copy.deepcopy(self.navposes_source_dict)
                        self.navposes_source_dict_lock.release()
                    
                        self.navposes_update_dict_lock.acquire()
                        navposes_update_dict = copy.deepcopy(self.navposes_update_dict)
                        navposes_update_offset_dict = copy.deepcopy(self.navposes_update_offset_dict)
                        navposes_update_reset_dict = copy.deepcopy(self.navposes_update_reset_dict)
                        self.navposes_update_dict_lock.release()    



                        
                        ####################
                        if cur_init_topic == topic:
                            type_name = 'init'

                            init_option = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_option']

                            init_state = True
                            if frame_name in self.navposes_init_states_dict.keys():
                                if comp_name in self.navposes_init_states_dict[frame_name].keys():
                                    init_state = self.navposes_init_states_dict[frame_name][comp_name]
                            
                            last_time = 0.0
                            if frame_name in self.navposes_init_times_dict.keys():
                                if comp_name in self.navposes_init_times_dict[frame_name].keys():
                                    times_dict = self.navposes_init_times_dict[frame_name][comp_name]
                                    last_time = times_dict['last_time']         
                            cur_time = nepi_utils.get_time()
                            timer = cur_time - last_time
                            delay = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['init_timed_sec']

                            should_init = (init_option == 'ALLWAYS') or (init_option == 'ONCE' and init_state != True) or (init_option == 'TIMED' and timer > delay)

                            if should_init == True:
                                if comp_name != 'pan_tilt':
                                    transform_dict = self.navposes_info_dict[frame_name]['connect_dict'][comp_name][type_name + '_transform']

                                navpose_update_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                                if frame_name in navposes_init_dict.keys():
                                    navpose_update_dict = navposes_init_dict[frame_name]


                                navpose_update_dict = nepi_nav.update_navpose_dict_from_msg(comp_name, navpose_update_dict, msg, transform_dict=transform_dict)

                                self.navposes_init_dict_lock.acquire()
                                self.navposes_init_dict[frame_name] = navpose_update_dict
                                self.navposes_init_dict_lock.release()

                                last_time = self.navposes_init_times_dict[frame_name][comp_name]['last_time']
                                cur_time = nepi_utils.get_time()
                                times = self.navposes_init_times_dict[frame_name][comp_name]['times']
                                times.pop(0)  # Remove first element
                                times.append(cur_time - last_time)  # Add new time difference
                                self.navposes_init_times_dict[frame_name][comp_name]['times'] = times  # Assign back to dict
                                self.navposes_init_times_dict[frame_name][comp_name]['last_time'] = cur_time

                                self.navposes_init_states_dict[frame_name][comp_name] = True



                        ####################
                        if cur_source_topic == topic:

                            type_name = 'source'

                            if comp_name != 'pan_tilt':
                                transform_dict = self.navposes_info_dict[frame_name]['connect_dict'][comp_name][type_name + '_transform']


                            navpose_source_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                            if frame_name in navposes_source_dict.keys():
                                navpose_source_dict = navposes_source_dict[frame_name]


                            navpose_source_dict = nepi_nav.update_navpose_dict_from_msg(comp_name, navpose_source_dict, msg, transform_dict=transform_dict)

                            self.navposes_source_dict_lock.acquire()
                            self.navposes_source_dict[frame_name] = navpose_source_dict
                            self.navposes_source_dict_lock.release()
                            
                            last_time = self.navposes_source_times_dict[frame_name][comp_name]['last_time']
                            cur_time = nepi_utils.get_time()
                            times = self.navposes_source_times_dict[frame_name][comp_name]['times']
                            times.pop(0)  # Remove first element
                            times.append(cur_time - last_time)  # Add new time difference
                            self.navposes_source_times_dict[frame_name][comp_name]['times'] = times  # Assign back to dict
                            self.navposes_source_times_dict[frame_name][comp_name]['last_time'] = cur_time

                            self.navposes_source_states_dict[frame_name][comp_name] = True



                        ####################
                        if cur_update_topic == topic:

                            type_name = 'update'

                            if comp_name != 'pan_tilt':
                                transform_dict = self.navposes_info_dict[frame_name]['connect_dict'][comp_name][type_name + '_transform']

                            last_navpose_update_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                            if frame_name in navposes_update_dict.keys():
                                last_navpose_update_dict = navposes_update_dict[frame_name]

                            navpose_update_offset = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                            if frame_name in navposes_update_offset_dict.keys():
                                navpose_update_offset = navposes_update_offset_dict[frame_name]

                            navpose_update_reset = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                            if frame_name in navposes_update_reset_dict.keys():
                                navpose_update_reset = navposes_update_reset_dict[frame_name]
                     
                            navpose_update_dict = nepi_nav.update_navpose_dict_from_msg(comp_name, last_navpose_update_dict, msg, transform_dict=transform_dict)

                            update_option = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_option']
                            if update_option == 'OFFSETS':
                                
                                navpose_update_offset = nepi_nav.update_navpose_dict_from_msg(comp_name, navpose_update_offset, msg, transform_dict=transform_dict)

                            elif update_option == 'RESETS':
                                update_resets_on_crossing = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_resets_on_crossing']
                                update_resets_crossing = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['update_resets_crossing']

                                navpose_source_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                                if frame_name in navposes_source_dict.keys():
                                    navpose_source_dict = navposes_source_dict[frame_name]

                                navpose_update_reset = nepi_nav.update_navpose_offsets_dict_from_updates(navpose_update_reset, navposes_source_dict, navpose_update_dict, last_navpose_update_dict, comp_name, update_resets_on_crossing, update_resets_crossing )

                            self.navposes_update_dict_lock.acquire()
                            self.navposes_update_dict[frame_name] = navpose_update_dict
                            self.navposes_update_offset_dict[frame_name] = navpose_update_offset
                            self.navposes_update_reset_dict[frame_name] = navpose_update_reset
                            self.navposes_update_dict_lock.release()

                            last_time = self.navposes_update_times_dict[frame_name][comp_name]['last_time']
                            cur_time = nepi_utils.get_time()
                            times = self.navposes_update_times_dict[frame_name][comp_name]['times']
                            times.pop(0)  # Remove first element
                            times.append(cur_time - last_time)  # Add new time difference
                            self.navposes_update_times_dict[frame_name][comp_name]['times'] = times  # Assign back to dict
                            self.navposes_update_times_dict[frame_name][comp_name]['last_time'] = cur_time

                            self.navposes_update_states_dict[frame_name][comp_name] = True



    ### Setup a regular background navpose get and publish timer callback
    def _getPublishSaveDataCb(self,timer):
        timestamp = nepi_utils.get_time()


        self.navposes_init_dict_lock.acquire()
        navposes_init_dict = copy.deepcopy(self.navposes_init_dict)
        self.navposes_init_dict_lock.release()

        self.navposes_source_dict_lock.acquire()
        navposes_source_dict = copy.deepcopy(self.navposes_source_dict)
        self.navposes_source_dict_lock.release()

        navposes_fixed_dict = copy.deepcopy(self.navposes_fixed_dict)

        self.navposes_update_dict_lock.acquire()
        navposes_update_dict = copy.deepcopy(self.navposes_update_dict)
        navposes_update_offset_dict = copy.deepcopy(self.navposes_update_offset_dict)
        self.navposes_update_dict_lock.release()


        navposes_solution_msg = NavPoses()
        navposes_solution_msg.navpose_frames = []
        navposes_solution_msg.navposes = []




        update_navposes = copy.deepcopy(self.update_navposes)
        for frame_name in navposes_source_dict.keys():

            navpose_info_dict = copy.deepcopy(self.navposes_info_dict[frame_name])

            if frame_name in self.navposes_info_dict.keys():

                # Publish NavPose if Needed
                cur_time = nepi_utils.get_time()
                last_time = cur_time
                if frame_name in self.navposes_pub_times_dict.keys():
                    last_time = self.navposes_pub_times_dict[frame_name]['last_time']
                timer = cur_time - last_time
                rate = navpose_info_dict['max_pub_rate']
                delay = float(1.0)/rate

                if self.node_if is not None and (timer > delay or update_navposes == True):                
                       

                    ### Update Solution Navpose
                    navpose_solution = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                    #self.msg_if.pub_warn("Navpose Solution Start: " + str(navpose_solution.keys()))

                    ###############navpose_solution
                    ## Apply Fixed and Init Updates

                    ### Update Fixed Navpose
                    navpose_fixed_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                    if frame_name in navposes_fixed_dict.keys():
                        navpose_fixed_dict = navposes_fixed_dict[frame_name]

                    ### Update Init Navpose
                    navpose_init_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                    if frame_name in navposes_init_dict.keys():
                        navpose_init_dict = navposes_init_dict[frame_name]

                    if navpose_fixed_dict != nepi_nav.BLANK_NAVPOSE_DICT:
                        fixed_for_init = copy.deepcopy(navpose_fixed_dict)
                        for comp_name in self.BLANK_CONNECTS_DICT.keys():
                            has_name = 'has_' + comp_name
                            comp_init_topic = navpose_info_dict['connect_dict'][comp_name]['init_topic']
                            if comp_init_topic == 'Fixed':
                                fixed_for_init[has_name] = True
                            else:
                                fixed_for_init[has_name] = False

                        navpose_init_dict = nepi_nav.update_navpose_dict_from_dict(navpose_init_dict, fixed_for_init)
                    
                    if navpose_init_dict != nepi_nav.BLANK_NAVPOSE_DICT:
                        navpose_solution = nepi_nav.update_navpose_dict_from_dict(navpose_solution, navpose_init_dict)

                    #self.msg_if.pub_warn("Navpose Solution Init: " + str(navpose_solution.keys()))
                        
                    ###############
                    ## Apply Source Updates

                    navpose_source_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                    if frame_name in navposes_source_dict.keys():
                        navpose_source_dict = navposes_source_dict[frame_name]

                    if navpose_source_dict != nepi_nav.BLANK_NAVPOSE_DICT:
                        navpose_source_replace_dict = copy.deepcopy(navpose_source_dict)
                        navpose_source_offset_dict = copy.deepcopy(navpose_source_dict)
                        for comp_name in self.BLANK_CONNECTS_DICT.keys():
                            source_option = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['source_option']
                            has_name = 'has_' + comp_name
                            if source_option == 'REPLACES':
                                navpose_source_offset_dict[has_name] = False
                            elif source_option == 'OFFSETS':
                                navpose_source_replace_dict[has_name] = False
                        navpose_solution = nepi_nav.update_navpose_dict_from_dict(navpose_solution, navpose_source_replace_dict)
                        #self.msg_if.pub_warn("Navpose Solution Source Update: " + str(navpose_solution.keys()))
                        navpose_solution = nepi_nav.update_navpose_dict_from_offsets(navpose_solution, navpose_source_offset_dict)
                        #self.msg_if.pub_warn("Navpose Solution Source Offsets: " + str(navpose_solution.keys()))
                    else:                         
                        navpose_source_replace_dict = navpose_source_dict
                        navpose_source_offset_dict = navpose_source_dict


                    ###############
                    ## Apply Update and Offset Updates


                    ### Update Offsets and Reset Navposes


                    navpose_update_offset_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                    if frame_name in navposes_update_offset_dict.keys():
                        navpose_update_offset_dict = navposes_update_offset_dict[frame_name]


                    navpose_update_reset_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                    if frame_name in navposes_update_dict.keys():
                        navpose_update_reset_dict = navposes_update_dict[frame_name]


                    if navpose_update_offset_dict != nepi_nav.BLANK_NAVPOSE_DICT:
                        navpose_solution = nepi_nav.update_navpose_dict_from_offsets(navpose_solution, navpose_update_offset_dict)
                        #self.msg_if.pub_warn("Navpose Solution Update Offsets: " + str(navpose_solution.keys()))

                    if navpose_update_reset_dict != nepi_nav.BLANK_NAVPOSE_DICT:
                        navpose_solution = nepi_nav.update_navpose_dict_from_offsets(navpose_solution, navpose_update_reset_dict)
                        #self.msg_if.pub_warn("Navpose Solution Update Offsets: " + str(navpose_solution.keys()))
                    


                    
                    ### Update To Pan Tilt If Enabled
                    # has_pan_tilt = navpose_dict['has_pan_tilt']
                    # pan_tilt_adjusted = navpose_info_dict['pan_tilt_adjusted']
                    # if (has_pan_tilt == True and pan_tilt_adjusted == True):
                    #     pan_deg = navpose_dict['pan_deg']
                    #     tilt_deg = navpose_dict['tilt_deg']
                    #     transform_dict = copy.deepcopy(nepi_nav.BLANK_TRANSFORM_DICT)
                    #     if 'pan_tilt' in self.navposes_info_dict[frame_name]['connect_dict'].keys():
                    #         transform_dict = self.navposes_info_dict[frame_name]['connect_dict']['pan_tilt']['tranform']
                    #     navpose_dict = nepi_nav.



                    ### Update Navposes Solutions
                    self.navposes_solution_dict[frame_name] = navpose_solution
                    navpose_solution_msg = nepi_nav.convert_navpose_dict2msg(navpose_solution)
                    navposes_solution_msg.navpose_frames.append(frame_name)
                    navposes_solution_msg.navposes.append(navpose_solution_msg)
        

                    self.navposes_pubs_dict_lock.acquire()

                    if frame_name in self.navposes_pubs_dict.keys():
                        try:
                            if self.navpose_pubs_published == False:
                                #self.msg_if.pub_warn("Navpose Solution Msg: " + str(navpose_solution_msg))
                                self.navpose_pubs_published = True
                            self.navposes_pubs_dict[frame_name].publish_pub('navpose',navpose_solution_msg)
                            self.navposes_pubs_dict[frame_name].publish_pub('navpose_fixed',nepi_nav.convert_navpose_dict2msg(navpose_fixed_dict))
                            self.navposes_pubs_dict[frame_name].publish_pub('navpose_init',nepi_nav.convert_navpose_dict2msg(navpose_init_dict))
                            self.navposes_pubs_dict[frame_name].publish_pub('navpose_source_replace',nepi_nav.convert_navpose_dict2msg(navpose_source_replace_dict))
                            self.navposes_pubs_dict[frame_name].publish_pub('navpose_source_offset',nepi_nav.convert_navpose_dict2msg(navpose_source_offset_dict))
                            self.navposes_pubs_dict[frame_name].publish_pub('navpose_update_offset',nepi_nav.convert_navpose_dict2msg(navpose_update_offset_dict))
                            self.navposes_pubs_dict[frame_name].publish_pub('navpose_update_reset',nepi_nav.convert_navpose_dict2msg(navpose_update_reset_dict))

                                

                        except Exception as e:
                            self.msg_if.pub_warn("Navpose Publishing Failed: " + str(e))

                    self.navposes_pubs_dict_lock.release()

                    last_time = self.navposes_pub_times_dict[frame_name]['last_time']
                    cur_time = nepi_utils.get_time()
                    times = self.navposes_pub_times_dict[frame_name]['times']
                    times.pop(0)  # Remove first element
                    times.append(cur_time - last_time)  # Add new time difference
                    self.navposes_pub_times_dict[frame_name]['times'] = times  # Assign back to dict
                    self.navposes_pub_times_dict[frame_name]['last_time'] = cur_time  # Assign back to dict

        
                    

        self.update_navposes = False


        if len(navposes_solution_msg.navpose_frames) > 0:
            ####################
            # Publish NavPoses
            cur_time = nepi_utils.get_time()
            last_time = self.last_navposes_pub_time
            timer = cur_time - last_time
            rate = self.navposes_max_pub_rate
            delay = float(1.0)/rate

            if self.node_if is not None and timer > delay:
                self.node_if.publish_pub('navposes_pub',navposes_solution_msg)
                self.last_navposes_pub_time = nepi_utils.get_time()

            ##### Save Data if Needed
            save_enabled = self.save_data_if.data_product_save_enabled('navposes') == True
            should_save = self.save_data_if.data_product_should_save('navposes') == True
            snapshot_enabled = self.save_data_if.data_product_snapshot_enabled('navposes') == True
            add_navposes = save_enabled or snapshot_enabled
            save_navpose = should_save or snapshot_enabled
            if add_navposes:
                time_ns = nepi_utils.get_time()
                data_time_str = nepi_utils.get_datetime_str_from_timestamp(time_ns, add_ms = True, add_us = True, add_tz = True, timezone = True)
                self.navposes_save_dict[data_time_str] = self.navposes_solution_dict

            if self.save_data_if is not None and len(list(self.navposes_save_dict.keys())) > 0 and save_navpose == True:
                    self.save_data_if.save('navposes',self.navposes_save_dict)
                    self.navposes_save_dict = dict()
                    
            

        process_delay = 0.1
        nepi_sdk.start_timer_process(process_delay, self._getPublishSaveDataCb, oneshot = True)




    def _publishStatusCb(self,timer):
        self.publish_status()


    def publish_status(self):
        
        self.status_msg.frame_nav_options = self.NAVPOSE_NAV_FRAME_OPTIONS 
        self.status_msg.frame_nav = self.frame_nav
        self.status_msg.frame_alt_options = self.NAVPOSE_ALT_FRAME_OPTIONS
        self.status_msg.frame_alt = self.frame_alt
        self.status_msg.frame_depth_options = self.NAVPOSE_DEPTH_FRAME_OPTIONS
        self.status_msg.frame_depth = self.frame_depth
        self.status_msg.navposes_max_pub_rate = self.navposes_max_pub_rate

        if self.node_if is not None:
            if self.status_published == False:
                # self.msg_if.pub_warn("Publishing first status msg: " + str(self.status_msg))
                # self.msg_if.pub_warn("Publishing NavPoses status msg: " + str(self.navposes_status_msg))
                pass
            #self.msg_if.pub_warn("Publishing status message: " + str(self.status_msg))
            self.status_msg.navposes_max_pub_rate = self.navposes_max_pub_rate
            self.node_if.publish_pub('status_pub',self.status_msg)
            self.status_published = True
            self.node_if.publish_pub('navposes_status_pub',self.navposes_status_msg)



    def _cleanupActions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################


if __name__ == '__main__':
    NavPoseMgr()



