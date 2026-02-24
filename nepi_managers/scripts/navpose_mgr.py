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

from nepi_interfaces.msg import MgrNavPoseStatus,NavPoseComponent, NavPoseSolution


from nepi_interfaces.msg import UpdateString, UpdateFloat, UpdateInt, UpdateNavPose, UpdateFrame3DTransform
from nepi_interfaces.msg import NavPose, NavPoses, NavPosesStatus


from nepi_interfaces.srv import MgrNavPoseCapabilitiesQuery, MgrNavPoseCapabilitiesQueryRequest, MgrNavPoseCapabilitiesQueryResponse
from nepi_interfaces.srv import NavPosesQuery, NavPosesQueryRequest, NavPosesQueryResponse


from nepi_interfaces.msg import Frame3DTransform, Frame3DTransforms
from nepi_interfaces.srv import Frame3DTransformsQuery, Frame3DTransformsQueryRequest, Frame3DTransformsQueryResponse
#from nepi_interfaces.srv import Frame3DTransformsRegister, Frame3DTransformsRegisterRequest, Frame3DTransformsRegisterResponse
#from nepi_interfaces.srv import Frame3DTransformsDelete, Frame3DTransformsDeleteRequest, Frame3DTransformsDeleteResponse

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import SaveDataIF

from nepi_api.data_if import NavPoseIF


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
    FACTORY_3D_FRAME = 'nepi_frame' 
    FACTORY_NAV_FRAME = 'ENU'
    FACTORY_ALT_FRAME = 'WGS84'

    ZERO_TRANSFORM = [0,0,0,0,0,0,0]
    ZERO_TRANSFORM_DICT = {0,0,0,0,0,0,0}

    times_list = [0,0,0,0,0,0,0]

    BLANK_CONNECT = {
            'has_component': False,
            'fixed': False,
            'topic': "",
            'transform': ZERO_TRANSFORM
        }
    
    BLANK_TIMES = {
            'times': times_list,
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
    
    BLANK_CONNECT_DICT = {
        'location': copy.deepcopy(BLANK_CONNECT),
        'heading': copy.deepcopy(BLANK_CONNECT),
        'orientation': copy.deepcopy(BLANK_CONNECT),
        'position': copy.deepcopy(BLANK_CONNECT),
        'altitude': copy.deepcopy(BLANK_CONNECT),
        'depth': copy.deepcopy(BLANK_CONNECT),
        'pan_tilt': copy.deepcopy(BLANK_CONNECT)
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
        'location': copy.deepcopy(ZERO_TRANSFORM),
        'heading': copy.deepcopy(ZERO_TRANSFORM),
        'orientation': copy.deepcopy(ZERO_TRANSFORM),
        'position': copy.deepcopy(ZERO_TRANSFORM),
        'altitude': copy.deepcopy(ZERO_TRANSFORM),
        'depth': copy.deepcopy(ZERO_TRANSFORM),
        'pan_tilt': copy.deepcopy(ZERO_TRANSFORM)
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

    system_navposes_dict = dict()

    frame_nav = 'ENU'
    frame_alt = 'WGS84'
    frame_depth = 'DEPTH'


    navposes_dict = dict()
    navposes_dict_lock = threading.Lock()

    navposes_info_dict = dict()
    last_navposes_info_dict = dict()
    navposes_info_dict_lock = threading.Lock()

    navpose_times_dict = dict()

    navposes_node_dict = dict()
    navposes_node_dict_lock = threading.Lock()

    navposes_pub_rate = 1
    last_navposes_pub_time = nepi_utils.get_time()

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
            'navposes_pub_rate': {
                'namespace': self.node_namespace,
                'factory_val': self.navposes_pub_rate
            },
            'navposes_info_dict': {
                'namespace': self.node_namespace,
                'factory_val': self.navposes_info_dict
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
            }
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
            'set_navposes_pub_rate': {
                'namespace': self.node_namespace,
                'topic': 'set_navposes_pub_rate',
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
            'update_frame_description': {
                'namespace': self.node_namespace,
                'topic': 'update_frame_description',
                'msg': UpdateString,
                'qsize': 1,
                'callback': self._updateFrameDescriptionCb, 
                'callback_args': ()
            },
            'update_frame_pub_rate': {
                'namespace': self.node_namespace,
                'topic': 'update_frame_pub_rate',
                'msg': UpdateFloat,
                'qsize': 1,
                'callback': self._updateFramePublishRateCb, 
                'callback_args': ()
            },
            'update_frame_fixed_navpose': {
                'namespace': self.node_namespace,
                'topic': 'update_frame_fixed_navpose',
                'msg': UpdateNavPose,
                'qsize': 1,
                'callback': self. _updateFrameFixedNavPoseCb, 
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
            'update_frame_comp_topic': {
                'namespace': self.node_namespace,
                'topic': 'update_frame_comp_topic',
                'msg': UpdateString,
                'qsize': 1,
                'callback': self._updateFrameCompTopicCb, 
                'callback_args': ()
            },
            'update_frame_comp_transform': {
                'namespace': self.node_namespace,
                'topic': 'update_frame_comp_transform',
                'msg': UpdateFrame3DTransform,
                'qsize': 1,
                'callback': self._updateFrameCompTransformCb, 
                'callback_args': ()
            },
            'clear_frame_comp_transform': {
                'namespace': self.node_namespace,
                'topic': 'clear_frame_comp_transform',
                'msg': UpdateString,
                'qsize': 1,
                'callback': self._clearFrameCompTransformCb, 
                'callback_args': ()
            }
        }


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT
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
            self.navposes_pub_rate = self.node_if.get_param('navposes_pub_rate')
            self.navposes_info_dict = self.node_if.get_param('navposes_info_dict')


            self.frame_nav = self.node_if.get_param('frame_nav')
            self.frame_alt = self.node_if.get_param('frame_alt')
            self.frame_depth = self.node_if.get_param('frame_depth')
 

        if do_updates == True:
            navposes_info_dict = copy.deepcopy(self.navposes_info_dict)
            navpose_frames = self.navposes_init_frames
            if len(list(navposes_info_dict.keys())) > 0:
                navpose_frames = list(navposes_info_dict.keys())
            for frame in navpose_frames:
                if frame in navposes_info_dict.keys():
                    navpose_info_dict = navposes_info_dict[frame]
                else:
                    navpose_info_dict = None
                self.addNavpose(frame,navpose_info_dict)
                
            if self.node_if is not None:
                self.node_if.save_config()

        self.publish_status()

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
        navposes_times_dict = copy.deepcopy(self.navpose_times_dict)
        self.navposes_info_dict_lock.release()

        navpose_frames = list(navposes_info_dict.keys())


        navpose_topics = []
        for frame_name in navpose_frames:
            navpose_topics.append(navposes_info_dict[frame_name]['namespace'])
       
        ############
        ## Update System NavPoses Dict
        last_system_navposes_dict = copy.deepcopy(self.system_navposes_dict)
        system_navposes_dict = dict()
        system_navposes_dict['frame_nav'] = self.frame_nav
        system_navposes_dict['frame_alt'] = self.frame_alt
        system_navposes_dict['frame_depth'] = self.frame_depth

        system_navposes_dict['navpose_options'] = list(navposes_info_dict.keys())     

        system_navposes_dict['navpose_pub_topics'] = navpose_topics
        self.system_navposes_dict = system_navposes_dict
        if last_system_navposes_dict != system_navposes_dict:
            nepi_system.set_navposes_dict(system_navposes_dict)

        
        ############
        # Update Caps Response
        self.caps_response.navpose_frames = navpose_frames
        self.caps_response.navpose_frames_topics = navpose_topics


        ############
        ## Update Status Message
        avail_topics_dict = copy.deepcopy(self.avail_topics_dict)




        navpose_solutions = []
        navpose_solutions_msg = []

        navpose_str_msg = []


        for frame_name in navpose_frames:
    

            if frame_name in navposes_info_dict.keys():

                navpose_str_msg.append('-----------------')
                navpose_str_msg.append(frame_name)
                navpose_str_msg.append('-----------------')
                navpose_str_msg.append('Description:' +  navposes_info_dict[frame_name]['description'] )
                navpose_str_msg.append('Set Pub Rate: ' + str(navposes_info_dict[frame_name]['pub_rate']) )
                navpose_str_msg.append('Use Pan Tilt for Heading: ' + str(navposes_info_dict[frame_name]['pan_tilt_adjusted']) )
            

                connect_dict = navposes_info_dict[frame_name]['connect_dict']


                comp_names = list(connect_dict.keys())

                
                comp_infos_msg = []
                for comp_name in comp_names:

                    comp_info_msg = NavPoseComponent()
                    comp_info_msg.comp_name = comp_name
                    comp_info_msg.available_topics = avail_topics_dict[comp_name]['topics']
                    comp_info_msg.available_topic_msgs = avail_topics_dict[comp_name]['msgs']


                    comp_info_msg.fixed = connect_dict[comp_name]['fixed']

                    comp_info_msg.topic = connect_dict[comp_name]['topic']

                    comp_info_msg.has_component = connect_dict[comp_name]['has_component']

                    if comp_info_msg.fixed == True:
                        comp_info_msg.available = True
                        comp_info_msg.connecting = False
                        comp_info_msg.connected = True
                    elif comp_info_msg.has_component == True:
                        comp_info_msg.available = (comp_info_msg.topic in list(self.avail_topics_dict['all_topics_dict'].keys()))
                        comp_info_msg.connecting = (comp_info_msg.topic in self.navpose_subs_connecting)
                        comp_info_msg.connected =  (comp_info_msg.topic in self.navpose_subs_connected)

                    if frame_name in navposes_times_dict.keys():
                        times_dict = navposes_times_dict[frame_name]
                        times = times_dict[comp_name]['times']
                        avg_times = sum(times)/len(times)
                        if avg_times > .01:
                            avg_rate = float(1.0) / avg_times
                        else:
                            avg_rate = 0
                        comp_info_msg.avg_rate = round( avg_rate, 3)
                        comp_info_msg.last_time = round( nepi_utils.get_time() - times_dict[comp_name]['last_time'], 3)

                    transform = connect_dict[comp_name]['transform']
                    comp_info_msg.transform = nepi_nav.convert_transform_list2msg(transform, source_ref_description = comp_info_msg.topic, end_ref_description = frame_name)



                    # Update string msg
                    navpose_str_msg.append('')
                    navpose_str_msg.append(comp_name)
                    navpose_str_msg.append('-----------------')

                    if comp_info_msg.has_component == False:
                        navpose_str_msg.append('Not Set')
                    elif comp_info_msg.fixed == True:
                        navpose_str_msg.append('Fixed')
                        for key in navposes_info_dict[frame_name]['fixed_navpose'].keys():
                            if comp_name in key and 'has' not in key:
                                 navpose_str_msg.append(key + ': ' + str(navposes_info_dict[frame_name]['fixed_navpose'][key]))

                    elif comp_info_msg.topic != '':
                        navpose_str_msg.append('Set to Topic: ' + comp_info_msg.topic)   
                        navpose_str_msg.append('Connected: ' + str(comp_info_msg.connected) )        
                        navpose_str_msg.append('Avg Receive Rate: ' + str(comp_info_msg.avg_rate) )  

                    comp_infos_msg.append(comp_info_msg)   
                    navpose_str_msg.append('')


                navpose_str_msg.append('')

                solution_msg = NavPoseSolution()
                solution_msg.frame_name = frame_name
                solution_msg.navpose_topic = navposes_info_dict[frame_name]['namespace']
                fixed_navpose_msg = nepi_nav.convert_navpose_dict2msg(navposes_info_dict[frame_name]['fixed_navpose'])
                solution_msg.fixed_navpose = fixed_navpose_msg
                solution_msg.pan_tilt_adjusted = navposes_info_dict[frame_name]['pan_tilt_adjusted']
                solution_msg.pub_rate = navposes_info_dict[frame_name]['pub_rate']
                solution_msg.components_list = comp_names
                solution_msg.components_info = comp_infos_msg

                navpose_solutions_msg.append(solution_msg)

        self.status_msg.navpose_frames = navpose_frames
        self.status_msg.navpose_frames_topics = navpose_topics
        self.status_msg.navpose_frames_solutions = navpose_solutions_msg
        self.status_msg.navpose_frames_str_msg = navpose_str_msg
        for frame_name in self.navposes_info_dict.keys():
            try:
                self.navposes_info_dict[frame_name]['ready'] = True
            except:
                pass

        ############
        ## Update Navposes Status Message
        self.navposes_status_msg.navpose_frames = navpose_frames
        self.navposes_status_msg.navpose_frames_topics = navpose_topics

        if (self.last_navposes_info_dict != self.navposes_info_dict):
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
                if self.navpose_subs_dict[topic]['sub'] is None and topic in self.avail_topics_dict['all_topics_dict'].keys():
                    msg = self.avail_topics_dict['all_topics_dict'][topic]
                    if topic not in self.navpose_subs_connecting:

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




    def addNavpose(self, frame_name, description = None, info_dict_entry = None, reset = False):
        self.msg_if.pub_warn("Adding navpose: " + str(frame_name))

        ################
        # Add to navposes_dict
        if frame_name not in self.navposes_dict.keys() or (reset == True and frame_name in self.navposes_dict.keys()):
            self.msg_if.pub_info("Initializing navpose data for: " + str(frame_name))
            self.navposes_dict_lock.acquire()
            self.navposes_dict[frame_name] = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
            self.navposes_dict_lock.release()

        ###################
        # update or create navposes_info_dict
        if frame_name not in self.navposes_info_dict.keys() or (reset == True and frame_name in self.navposes_info_dict.keys()):
            self.msg_if.pub_info("Updating navpose entry for: " + str(frame_name))
            if info_dict_entry is None or reset == True:
                info_dict_entry = dict()
                info_dict_entry['frame_name'] = frame_name
                info_dict_entry['data_product'] = frame_name + 'navpose'
                info_dict_entry['description'] = frame_name
                info_dict_entry['pub_rate'] = self.FACTORY_PUB_RATE_HZ
                info_dict_entry['fixed_navpose'] = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                info_dict_entry['connect_dict'] = copy.deepcopy(self.BLANK_CONNECT_DICT)
                info_dict_entry['transforms_dict'] = copy.deepcopy(self.BLANK_TRANSFORMS_DICT)
                info_dict_entry['pan_tilt_adjusted'] = False
            else:
                info_dict_entry['frame_name'] = frame_name
                info_dict_entry['data_product'] = frame_name + '-navpose'
                info_dict_entry['connect_dict']['connected'] = False
                info_dict_entry['connect_dict']['avg_rate'] = 0
                info_dict_entry['description'] = frame_name
            


            if description is not None:
                info_dict_entry['description'] = description

            clean_name = nepi_utils.get_clean_name(frame_name)
            namespace = self.base_namespace + '/navposes/' +  clean_name 
            info_dict_entry['namespace'] = namespace

            info_dict_entry['ready'] = False
            self.msg_if.pub_info("Registering new navpose: " + str(frame_name))
            self.navposes_info_dict_lock.acquire()
            self.navposes_info_dict[frame_name] = info_dict_entry
            self.navposes_info_dict_lock.release()

            # Register frame topics if set
            for comp_name in info_dict_entry['connect_dict'].keys():
                topic = info_dict_entry['connect_dict'][comp_name]['topic']
                self.registerCompTopic(frame_name, comp_name, topic)


            ###################
            # Add to Save Data IF
            if self.save_data_if is not None:
                self.save_data_if.register_data_product(info_dict_entry['data_product'])
                self.data_products_list.append(info_dict_entry['data_product'])
       
 
            ###################
            # Add to times_dict 
            self.navpose_times_dict[frame_name] = copy.deepcopy(self.BLANK_TIMES_DICT)

            ###################
            # Add to navposes_node_dict             
            if frame_name not in self.navposes_node_dict.keys():
                self.msg_if.pub_info("Creating navpose node entries for: " + str(frame_name))
                navpose_if = NavPoseIF(namespace = namespace,
                                    data_product = 'navpose',
                                    data_source_description =  frame_name + ' frame',
                                    data_ref_description = info_dict_entry['description'],
                                    pub_navpose = True,
                                    pub_location = False, pub_heading = False,
                                    pub_orientation = False, pub_position = False,
                                    pub_altitude = False, pub_depth = False,
                                    pub_pan_tilt = False,
                                    save_data_if = self.save_data_if,
                                    msg_if = self.msg_if
                                    )
                # Add a local pub to start publishing before navpose_if is ready
                navpose_pub = nepi_sdk.create_publisher(namespace + '/navpose',NavPose, queue_size = 1, log_name_list = [])


                node_dict_entry = dict()
                node_dict_entry['navpose_if'] = navpose_if
                node_dict_entry['navpose_pub'] = navpose_pub

                self.navposes_node_dict_lock.acquire()
                self.navposes_node_dict[frame_name] = node_dict_entry
                self.navposes_node_dict_lock.release()

                #######################
                # Initialize NavPose Solution Dict
                self.navposes_pub_times_dict[frame_name] = nepi_utils.get_time()
                self.navposes_solution_dict[frame_name] = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

        self.updateNavposesData()






    def unregisterCompTopic(self, frame_name, comp_name, topic):
        if topic != '' and topic != 'None' and topic != 'Fixed' and topic in self.navpose_subs_dict.keys():
            if frame_name in self.navpose_subs_dict[topic]['subs_dict'].keys():
                if comp_name not in self.navpose_subs_dict[topic]['subs_dict'][frame_name]:
                    self.navpose_subs_dict[topic]['subs_dict'][frame_name].remove(comp_name)
                    if len(self.navpose_subs_dict[topic]['subs_dict'][frame_name]) == 0:
                        del self.navpose_subs_dict[topic]['subs_dict'][frame_name]

            if len(self.navpose_subs_dict[topic]['subs_dict'].keys()) == 1:
                if topic not in self.navpose_subs_registered:
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





    def removeNavpose(self, frame_name):
        if frame_name == self.NAVPOSE_BASE_FRAME:
            self.msg_if.pub_info("Can't Remove Base Frame: " + str(frame_name))
        elif frame_name not in self.navposes_info_dict.keys():
            self.msg_if.pub_info("NavPose entry does not exist for: " + str(frame_name))
        else:


            # Unregister Register frame topics if set
            topics = list(self.navpose_subs_dict.keys())
            for comp_name in self.self.navposes_info_dict[frame_name].keys():
                for topic in topics:
                    self.unregisterCompTopic(frame_name, comp_name, topic)

            self.msg_if.pub_info("Removing navpose: " + str(frame_name))
            if self.save_data_if is not None:
                self.data_products_list.remove(self.navposes_info_dict[frame_name]['data_product'])
                self.save_data_if.unregister_data_product(self.navposes_info_dict[frame_name]['data_product'])
                

            # Remove from info dict
            self.navposes_info_dict_lock.acquire()
            del self.navposes_info_dict[frame_name]
            self.navposes_info_dict_lock.release()
            nepi_sdk.sleep(1)


            # Remove from node dict
            self.navposes_node_dict_lock.acquire()
            self.navposes_node_dict[frame_name]['navpose_if'].unregister()
            self.navposes_node_dict[frame_name]['navpose_pub'].unregister()
            nepi_sdk.sleep(1)
            del self.navposes_node_dict[frame_name]
            self.navposes_node_dict_lock.release()

            # Remove from Navpose Dict
            self.navposes_dict_lock.acquire()
            del self.navposes_dict[frame_name]
            self.navposes_dict_lock.release()

            self.updateNavposesData()



    #######################
    ### Node Methods


           
    def setFrameNav(self,frame):
        self.frame_nav = frame
        self.publish_status()
        self.updateNavposesData()
        if self.node_if is not None:
            self.node_if.set_param('frame_nav',frame)
            self.node_if.save_config()

    def setFrameAlt(self,frame):
        self.frame_alt = frame
        self.publish_status()
        self.updateNavposesData()
        if self.node_if is not None:
            self.node_if.set_param('frame_alt',frame)
            self.node_if.save_config()

    def setFrameDepth(self,frame):
        self.frame_depth = frame
        self.publish_status()
        self.updateNavposesData()
        if self.node_if is not None:
            self.node_if.set_param('frame_depth',frame)
            self.node_if.save_config()
     

    def updateFrameDescription(self,frame_name, desc):
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
        self.navposes_pub_rate = rate
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('navposes_pub_rate',self.navposes_pub_rate)
            self.node_if.save_config()


    def updateFramePublishRate(self, frame_name, rate):
        min = self.NAVPOSE_PUB_RATE_OPTIONS[0]
        max = self.NAVPOSE_PUB_RATE_OPTIONS[1]
        if rate < min:
            rate = min
        if rate > max:
            rate = max
        if frame_name in self.navposes_info_dict.keys():
            self.navposes_info_dict[frame_name]['pub_rate'] = rate
            self.publish_status()
            self.updateNavposesData()
            if self.node_if is not None:
                self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
                self.node_if.save_config()
        else:
            self.publish_status()


    def updateFrameCompTopic(self, frame_name, comp_name, topic):
        if frame_name in self.navposes_info_dict.keys():
            avail_topics_dict = copy.deepcopy(self.avail_topics_dict)
            if comp_name in self.navposes_info_dict[frame_name]['connect_dict'].keys():
                cur_topic = self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['topic']
                self.unregisterCompTopic(frame_name, comp_name, cur_topic)
                if topic == 'Fixed':
                    self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['has_component'] = True
                    self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['fixed'] = True
                    self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['topic'] = ''
                elif topic == 'None' or topic == '':
                    self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['has_component'] = False
                    self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['fixed'] = False
                    self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['topic'] = ''
                elif comp_name in avail_topics_dict.keys():
                    if topic in avail_topics_dict[comp_name]['topics']:
                        self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['has_component'] = True
                        self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['fixed'] = False
                        self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['topic'] = topic
                self.publish_status()
                self.registerCompTopic(frame_name, comp_name, topic)
                self.updateNavposesData()
                if self.node_if is not None:
                    self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
                    self.node_if.save_config()
        else:
            self.publish_status()

    def clearFrameCompTopic(self,frame_name, comp_name):
        self.updateFrameCompTopic(frame_name, comp_name, 'None')

    def updateFrameCompTransform(self, frame_name, comp_name, transform):
        if frame_name in self.navposes_info_dict.keys():
            if comp_name in self.navposes_info_dict[frame_name]['connect_dict'].keys():
                self.navposes_info_dict[frame_name]['connect_dict'][comp_name]['transform'] = transform
                self.publish_status()
                self.updateNavposesData()
                if self.node_if is not None:
                    self.node_if.set_param('navposes_info_dict',self.navposes_info_dict)
                    self.node_if.save_config()
        else:
            self.publish_status()

    def clearFrameCompTransform(self,frame_name, comp_name):
        transform = self.ZERO_TRANSFORM
        self.updateFrameCompTransform(frame_name, comp_name, transform)


    ###############################################
 

    def updateFrameFixedNavPose(self, frame_name, navpose_dict = None):
        if navpose_dict is None:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        if frame_name in self.navpose_info_dict.keys():
            self.navpose_info_dict.acquire()
            self.navpose_info_dict[frame_name]['fixed_navpose'] = navpose_dict
            self.navpose_info_dict.release()


    def clearFrameFixedNavPose(self, frame_name):
        self.updateFrameFixedNavPose(frame_name)



    def get_navpose_dict(self, frame_name):
        navpose_dict = None
        if frame_name in self.navposes_dict.keys():
            self.navposes_dict_lock.acquire() 
            navpose_dict = copy.deepcopy(self.navposes_dict[frame_name])
            self.navposes_dict_lock.release()
        return navpose_dict

    def get_navposes_dict(self):
        return self.navposes_dict        



    #######################
    # Private Members
    #######################

    def _navposeCapsQueryHandler(self,req):
        return self.caps_response

    def _navposesDataQueryHandler(self,req):
        self.navposes_dict_lock.acquire()
        navposes_dict = copy.deepcopy(self.navposes_dict)
        self.navposes_dict_lock.release()
        response = NavPosesQueryResponse()
        response.navpose_frames = list(navposes_dict.keys())
        navpose_frames_data = []
        for frame in navposes_dict.keys():
            navpose_dict = navposes_dict[frame]
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
                self.subscribeTopic()
                pub_status = True
        
        if pub_status == True:
            self.publish_status()



    def _updateAvailTopicsCb(self,timer):
        last_dict = copy.deepcopy(self.avail_topics_dict)
        all_topics_dict = dict()
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('location')
        for i, topic in enumerate(topic_list):
            all_topics_dict[topic] = msg_list[i]
        self.avail_topics_dict['location']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['location']['msgs'] = copy.deepcopy(msg_list)


        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('heading')
        for i, topic in enumerate(topic_list):
            all_topics_dict[topic] = msg_list[i]
        self.avail_topics_dict['heading']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['heading']['msgs'] = copy.deepcopy(msg_list)


        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('orientation')
        for i, topic in enumerate(topic_list):
            all_topics_dict[topic] = msg_list[i]
        self.avail_topics_dict['orientation']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['orientation']['msgs'] = copy.deepcopy(msg_list)

      
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('position')
        for i, topic in enumerate(topic_list):
            all_topics_dict[topic] = msg_list[i]
        self.avail_topics_dict['position']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['position']['msgs'] = copy.deepcopy(msg_list)

        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('altitude')
        for i, topic in enumerate(topic_list):
            all_topics_dict[topic] = msg_list[i]
        self.avail_topics_dict['altitude']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['altitude']['msgs'] = copy.deepcopy(msg_list)


        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('depth')
        for i, topic in enumerate(topic_list):
            all_topics_dict[topic] = msg_list[i]
        self.avail_topics_dict['depth']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['depth']['msgs'] = copy.deepcopy(msg_list)


        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('pan_tilt')
        for i, topic in enumerate(topic_list):
            all_topics_dict[topic] = msg_list[i]
        self.avail_topics_dict['pan_tilt']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['pan_tilt']['msgs'] = copy.deepcopy(msg_list)


        self.avail_topics_dict['all_topics_dict'] = all_topics_dict
        if self.avail_topics_dict != last_dict:
            self.publish_status()
        #self.msg_if.pub_warn("Updater - Got avail_topics_dict: " + str(self.avail_topics_dict))
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
            self.addNavpose(frame_name, info_dict_entry)


    def _deleteFrameCb(self,msg):
        frame_name = msg.data
        self.removeNavpose(frame_name)

    def _resetFrameCb(self,msg):
        frame_name = msg.data
        self.addNavpose(frame_name, reset = True) 




    def _updateFramePublishRateCb(self,msg):
        frame_name = msg.name
        value = msg.value
        self.updateFramePublishRate(frame_name, value)
       


    def _updateFrameDescriptionCb(self,msg):
        frame_name = msg.name
        value = msg.value
        self.updateFrameDescription(frame_name, value)


    def _updateFrameCompTopicCb(self,msg):
        frame_name = msg.name
        comp_name = msg.name2
        topic = msg.value
        self.updateFrameCompTopic(frame_name, comp_name, topic)
       
    def _updateFrameCompTransformCb(self,msg):
        frame_name = msg.name
        comp_name = msg.name2
        transform = nepi_nav.convert_transform_msg2list(msg.transform)
        self.updateFrameCompTransform(frame_name, comp_name, transform)

    def _clearFrameCompTransformCb(self,msg):
        frame_name = msg.name
        comp_name = msg.value
        transform = copy.deepcopy(nepi_nav.ZERO_TRANSFORM)
        self.updateFrameCompTransform(frame_name, comp_name, transform)

    def  _updateFrameFixedNavPoseCb(self,msg):
        frame_name = msg.name
        navpose_dict = nepi_nav.convert_navpose_msg2dict(msg.navpose)
        if navpose_dict is not None:
            self.updateFrameFixedNavPose(frame_name, navpose_dict)
       
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
            frames = list(self.navpose_subs_dict[topic]['subs_dict'].keys())
            for frame in frames:
                if frame != 'None' and frame in self.navposes_dict.keys() and frame in self.navposes_info_dict.keys():
                    comps = self.navpose_subs_dict[topic]['subs_dict'][frame]
                    for comp in comps:
                        transform = None
                        if comp != 'pan_tilt':
                            transform = self.navposes_info_dict[frame]['connect_dict'][comp]['tranform']

                        self.navposes_dict_lock.acquire()
                        navposes_dict = copy.deepcopy(self.navposes_dict)
                        self.navposes_dict_lock.release()

                        navposes_dict = nepi_nav.update_navpose_dict_from_msg(comp, navposes_dict, msg, transform=transform)

                        self.navposes_dict_lock.acquire()
                        self.navposes_dict = navposes_dict
                        self.navposes_dict_lock.release()


                        last_time = self.navpose_times_dict[frame]['last_time']
                        cur_time = nepi_utils.get_time()
                        times = self.navpose_times_dict[frame]['times']
                        times.pop(0)  # Remove first element
                        times.append(cur_time - last_time)  # Add new time difference
                        self.navpose_times_dict[frame]['times'] = times  # Assign back to dict
                        self.navpose_times_dict[frame]['last_time'] = cur_time



    ### Setup a regular background navpose get and publish timer callback
    def _getPublishSaveDataCb(self,timer):
        timestamp = nepi_utils.get_time()
        # Get current NEPI NavPose data from NEPI ROS navpose_query service call
        self.navposes_dict_lock.acquire()
        navposes_dict = copy.deepcopy(self.navposes_dict)
        self.navposes_dict_lock.release()

        navposes_msg = NavPoses()
        navposes_msg.navpose_frames = []
        navposes_msg.navposes = []
        for frame_name in navposes_dict.keys():
            navpose_info_dict = copy.deepcopy(self.navposes_info_dict[frame_name])
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
            if frame_name in self.navposes_info_dict.keys():
                navpose_dict = navposes_dict[frame_name]

            fixed_navpose_dict = navpose_info_dict['fixed_navpose']
            for comp_name in navpose_info_dict['connect_dict'].keys():
                fixed = navpose_info_dict['connect_dict'][comp_name]['fixed']
                has_key = 'has_' + comp_name
                if has_key in fixed_navpose_dict.keys():
                    fixed_navpose_dict[has_key] = fixed
            navpose_dict = nepi_nav.update_navpose_dict_from_dict(navpose_dict,fixed_navpose_dict)

            # has_pan_tilt = navpose_dict['has_pan_tilt']
            # pan_tilt_adjusted = navpose_info_dict['pan_tilt_adjusted']
            # if (has_pan_tilt == True and pan_tilt_adjusted == True):
            #     pan_deg = navpose_dict['pan_deg']
            #     tilt_deg = navpose_dict['tilt_deg']
            #     transform = self.ZERO_TRANSFORM
            #     if 'pan_tilt' in self.navposes_info_dict[frame]['connect_dict'].keys():
            #         transform = self.navposes_info_dict[frame]['connect_dict']['pan_tilt']['tranform']
            #     navpose_dict = nepi_nav.

            self.navposes_solution_dict[frame_name] = copy.deepcopy(navpose_dict)
            navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
            navposes_msg.navpose_frames.append(frame_name)
            navposes_msg.navposes.append(navpose_msg)
            

            # Publish NavPose if Needed
            cur_time = nepi_utils.get_time()
            last_time = cur_time
            if frame_name in self.navposes_pub_times_dict.keys():
                last_time = self.navposes_pub_times_dict[frame_name]
            timer = cur_time - last_time
            rate = navpose_info_dict['pub_rate']
            delay = float(1.0)/rate


            if self.node_if is not None and timer > delay:
                self.navposes_node_dict_lock.acquire()

                try:
                    if_ready = self.navposes_node_dict[frame_name]['navpose_if'].ready
                except:
                    if_ready = False

                if if_ready == True:
                    self.navposes_node_dict[frame_name]['navpose_if'].publish_navpose(navpose_dict)
                elif self.navposes_node_dict[frame_name]['navpose_pub'] is not None:
                    pub = self.navposes_node_dict[frame_name]['navpose_pub']
                    nepi_sdk.publish_pub(pub, navpose_msg)
                self.navposes_node_dict_lock.release()

                self.navposes_pub_times_dict[frame_name] = nepi_utils.get_time()


            # Save Data if Needed
            if self.save_data_if is not None:
                data_product = navpose_info_dict['data_product']
                self.save_data_if.save(data_product,navpose_dict)



        ####################
        # Publish NavPoses
        cur_time = nepi_utils.get_time()
        last_time = self.last_navposes_pub_time
        timer = cur_time - last_time
        rate = data_product = self.navposes_pub_rate
        delay = float(1.0)/rate

        if self.node_if is not None and timer > delay:
            self.node_if.publish_pub('navposes_pub',navposes_msg)
            self.last_navposes_pub_time = nepi_utils.get_time()

        if self.save_data_if is not None:
            self.save_data_if.save('navposes',self.navposes_solution_dict)


        delay = 0.1
        nepi_sdk.start_timer_process(delay, self._getPublishSaveDataCb, oneshot = True)




    def _publishStatusCb(self,timer):
        self.publish_status()


    def publish_status(self):
        
        self.status_msg.frame_nav_options = self.NAVPOSE_NAV_FRAME_OPTIONS 
        self.status_msg.frame_nav = self.frame_nav
        self.status_msg.frame_alt_options = self.NAVPOSE_ALT_FRAME_OPTIONS
        self.status_msg.frame_alt = self.frame_alt
        self.status_msg.frame_depth_options = self.NAVPOSE_DEPTH_FRAME_OPTIONS
        self.status_msg.frame_depth = self.frame_depth
        self.status_msg.navposes_pub_rate = self.navposes_pub_rate

        if self.node_if is not None:
            # if self.status_published == False:
            #     self.msg_if.pub_warn("Publishing first status msg: " + str(self.status_msg))
            #     self.msg_if.pub_warn("Publishing NavPoses status msg: " + str(self.navposes_status_msg))

            self.status_msg.navposes_pub_rate = self.navposes_pub_rate
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



