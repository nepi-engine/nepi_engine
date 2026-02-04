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

from nepi_interfaces.msg import MgrNavPoseStatus,MgrNavPoseCompInfo

from nepi_interfaces.msg import UpdateTopic, UpdateNavPoseTopic, UpdateFrame3DTransform

from nepi_interfaces.msg import NavPose, NavPoses, NavPosesStatus


from nepi_interfaces.srv import MgrNavPoseCapabilitiesQuery, MgrNavPoseCapabilitiesQueryRequest, MgrNavPoseCapabilitiesQueryResponse
from nepi_interfaces.srv import NavPoseQuery, NavPoseQueryRequest, NavPoseQueryResponse


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
            'fixed': True,
            'topic': "",
            'msg': "",
            'connected': False,
            'transform': ZERO_TRANSFORM,
            'times': times_list,
            'last_time': 0.0
        }

    BLANK_SUB = {
            'topic': "",
            'msg': "",
            'sub': None
        }

    BLANK_AVIAL_TOPIC = {
            'topics': [],
            'msgs': [],
        }


    mgr_namespace = "mgr_navpose"

    node_if = None

    status_msg = MgrNavPoseStatus()
    navposes_status_msg = NavPosesStatus()
    caps_response = MgrNavPoseCapabilitiesQueryResponse()
    navposes_topic = ''

    save_data_if = None
    data_products_list = ['navposes']
    navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    navpose_dict['navpose_frame'] = 'nepi_frame'
    navpose_dict['navpose_description'] = 'nepi base frame'
    navpose_dict_lock = threading.Lock()
    init_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    last_npdata_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

    navpose_description = 'Undefined'
    frame_nav = 'ENU'
    frame_alt = 'WGS84'
    frame_depth = 'DEPTH'

    navpose_info_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_INFO_DICT)


    navpose_frames = ['None','nepi_frame']
    navpose_frames_info_dict = dict()


    # navposes_status_msg


    # navpose_frame_topics = ['',''] # Updated later
    # navpose_pubs_dict = dict()
    # data_source_description = 'nepi_navpose_solution'
    # data_ref_description = 'nepi_frame'

    set_pub_rate = FACTORY_PUB_RATE_HZ


    connect_dict = {
        'location': copy.deepcopy(BLANK_CONNECT),
        'heading': copy.deepcopy(BLANK_CONNECT),
        'orientation': copy.deepcopy(BLANK_CONNECT),
        'position': copy.deepcopy(BLANK_CONNECT),
        'altitude': copy.deepcopy(BLANK_CONNECT),
        'depth': copy.deepcopy(BLANK_CONNECT),
        'pan_tilt': copy.deepcopy(BLANK_CONNECT)
    }

    subs_dict_lock = threading.Lock()
    subs_dict = {
        'location': copy.deepcopy(BLANK_SUB),
        'heading': copy.deepcopy(BLANK_SUB),
        'orientation': copy.deepcopy(BLANK_SUB),
        'position': copy.deepcopy(BLANK_SUB),
        'altitude': copy.deepcopy(BLANK_SUB),
        'depth': copy.deepcopy(BLANK_SUB),
        'pan_tilt': copy.deepcopy(BLANK_SUB)
    }

    transforms_dict = {
        'location': copy.deepcopy(ZERO_TRANSFORM),
        'heading': copy.deepcopy(ZERO_TRANSFORM),
        'orientation': copy.deepcopy(ZERO_TRANSFORM),
        'position': copy.deepcopy(ZERO_TRANSFORM),
        'altitude': copy.deepcopy(ZERO_TRANSFORM),
        'depth': copy.deepcopy(ZERO_TRANSFORM),
        'pan_tilt': copy.deepcopy(ZERO_TRANSFORM)
    }



    avail_topics_dict = {
        'location': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'heading': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'orientation': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'position': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'altitude': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'depth': copy.deepcopy(BLANK_AVIAL_TOPIC),
        'pan_tilt': copy.deepcopy(BLANK_AVIAL_TOPIC)
    }






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
        nepi_system.set_navpose_settings(self.navpose_info_dict)
        nepi_system.set_navpose_frames(self.navpose_frames)
        self.msg_if.pub_warn("Waiting for Config Mgr")
        config_folders = nepi_system.get_config_folders()

        self.mgr_namespace = nepi_sdk.create_namespace(self.base_namespace, self.MGR_NODE_NAME)
        self.navposes_topic = self.base_namespace + '/navposes'


        for frame in self.navpose_frames:
            self.navpose_frames_info_dict[frame] = dict()
            self.navpose_frames_info_dict[frame]['name'] = frame
            self.navpose_frames_info_dict[frame]['topic'] = self.navposes_topic + '/' + frame + '/navpose'

        self.cb_dict = {
            'location': self._locationSubCb,
            'heading': self._headingSubCb,
            'orientation': self._orientationSubCb,
            'position': self._positionSubCb,
            'altitude': self._altitudeSubCb,
            'depth': self._depthSubCb,
            'pan_tilt': self._panTiltSubCb,
        }



        self.caps_response.frame_nav_options = self.NAVPOSE_NAV_FRAME_OPTIONS 
        self.caps_response.frame_alt_options = self.NAVPOSE_ALT_FRAME_OPTIONS
        self.caps_response.frame_depth_options = self.NAVPOSE_DEPTH_FRAME_OPTIONS


    
        self.status_msg.navposes_topic  = self.navposes_topic

        self.status_msg.frame_nav_options = self.NAVPOSE_NAV_FRAME_OPTIONS 
        self.status_msg.frame_alt_options = self.NAVPOSE_ALT_FRAME_OPTIONS
        self.status_msg.frame_depth_options = self.NAVPOSE_DEPTH_FRAME_OPTIONS
        self.status_msg.publishing = False
        self.status_msg.pub_rate = self.set_pub_rate


        self.navposes_status_msg.node_name = self.node_name
        self.navposes_status_msg.navposes_topic = self.base_namespace + '/navposes'
        self.navposes_status_msg.navpose_frames = []
        self.navposes_status_msg.navpose_topics = []
        self.navposes_status_msg.save_data_topic = ''

        self.initCb(do_updates = False)

        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace': self.mgr_namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'navpose_description': {
                'namespace': self.mgr_namespace,
                'factory_val': self.navpose_description
            },
            'frame_nav': {
                'namespace': self.mgr_namespace,
                'factory_val': self.frame_nav
            },
            'frame_alt': {
                'namespace': self.mgr_namespace,
                'factory_val': self.frame_alt
            },
            'frame_depth': {
                'namespace': self.mgr_namespace,
                'factory_val': self.frame_depth
            },
            'pub_rate': {
                'namespace': self.mgr_namespace,
                'factory_val': self.FACTORY_PUB_RATE_HZ
            },
            'connect_dict': {
                'namespace': self.mgr_namespace,
                'factory_val': self.connect_dict
            },
            'transforms_dict': {
                'namespace': self.mgr_namespace,
                'factory_val': self.transforms_dict

            },
            'init_navpose_dict': {
                'namespace': self.mgr_namespace,
                'factory_val': self.init_navpose_dict
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
                'srv': NavPoseQuery,
                'req': NavPoseQueryRequest(),
                'resp': NavPoseQueryResponse(),
                'callback': self._navposeDataQueryHandler
            }
        }


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.mgr_namespace,
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
            'set_frame_desc': {
                'namespace': self.mgr_namespace,
                'topic': 'set_frame_description',
                'msg': String,
                'qsize': 1,
                'callback': self._setFrameDescCb, 
                'callback_args': ()
            },
            'set_frame_nav': {
                'namespace': self.mgr_namespace,
                'topic': 'set_frame_nav',
                'msg': String,
                'qsize': 1,
                'callback': self._setFrameNavCb, 
                'callback_args': ()
            },
            'set_frame_alt': {
                'namespace': self.mgr_namespace,
                'topic': 'set_frame_alt',
                'msg': String,
                'qsize': 1,
                'callback': self._setFrameAltCb, 
                'callback_args': ()
            },
            'set_frame_depth': {
                'namespace': self.mgr_namespace,
                'topic': 'set_frame_depth',
                'msg': String,
                'qsize': 1,
                'callback': self._setFrameDepthCb, 
                'callback_args': ()
            },
            'pub_rate': {
                'namespace': self.mgr_namespace,
                'topic': 'set_pub_rate',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setPublishRateCb, 
                'callback_args': ()
            },
            'set_topic': {
                'namespace': self.mgr_namespace,
                'topic': 'set_topic',
                'msg': UpdateNavPoseTopic,
                'qsize': 1,
                'callback': self._setTopicCb, 
                'callback_args': ()
            },
            'clear_topic': {
                'namespace': self.mgr_namespace,
                'topic': 'clear_topic',
                'msg': String,
                'qsize': 1,
                'callback': self._clearTopicCb, 
                'callback_args': ()
            },
            'set_transform': {
                'namespace': self.mgr_namespace,
                'topic': 'set_transform',
                'msg': UpdateFrame3DTransform,
                'qsize': 1,
                'callback': self._setTransformCb, 
                'callback_args': ()
            },
            'clear_transform': {
                'namespace': self.mgr_namespace,
                'topic': 'clear_transform',
                'msg': String,
                'qsize': 1,
                'callback': self._clearTransformCb, 
                'callback_args': ()
            },
            'set_navpose': {
                'namespace': self.mgr_namespace,
                'topic': 'set_navpose',
                'msg': NavPose,
                'qsize': 1,
                'callback': self._setNavPoseCb, 
                'callback_args': ()
            },
            'reset_navpose': {
                'namespace': self.mgr_namespace,
                'topic': 'reset_navpose',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetNavPoseCb, 
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
                factory_data_rates[d][0] = float(1.0) / self.FACTORY_PUB_RATE_HZ
        sd_namespace = self.base_namespace + '/navposes'  
        self.save_data_if = SaveDataIF(namespace = sd_namespace, data_products = self.data_products_list, factory_rate_dict = factory_data_rates)

        self.navposes_status_msg.save_data_topic = sd_namespace + '/save_data'
        ######################
        # initialize variables from param server

        self.initCb(do_updates = True)
        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._getPublishSaveDataCb, oneshot = True)
        nepi_sdk.start_timer_process(5.0, self._updateAvailTopicsCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._updateConnectionsCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)
        np_pub_delay = float(1.0)/self.set_pub_rate

        ##############################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        nepi_sdk.spin()


    def initCb(self,do_updates = False):
        if self.node_if is not None:
            self.navpose_description = self.node_if.get_param('navpose_description')
            self.frame_nav = self.node_if.get_param('frame_nav')
            self.frame_alt = self.node_if.get_param('frame_alt')
            self.frame_depth = self.node_if.get_param('frame_depth')
            self.set_pub_rate = self.node_if.get_param('pub_rate')
            connect_dict = self.node_if.get_param('connect_dict')
            for entry in self.connect_dict.keys():
                if entry not in connect_dict.keys():
                    connect_dict[entry] = self.connect_dict[entry]
            try:
                transforms_dict = self.node_if.get_param('transforms_dict')
                for entry in self.transforms_dict.keys():
                    if entry not in transforms_dict.keys():
                        transforms_dict[entry] = self.transforms_dict[entry]
                self.transforms_dict = transforms_dict
            except:
                self.node_if.set_param('transforms_dict',self.transforms_dict)

            self.msg_if.pub_warn("initCB self.transforms_dict" + str(self.transforms_dict))
            self.init_navpose_dict = self.node_if.get_param('init_navpose_dict')
  

        self.navpose_dict_lock.acquire()
        self.navpose_dict = self.init_navpose_dict
        self.navpose_dict_lock.release()
        if do_updates == True:
            self.update_navpose_frames()
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



    def update_navpose_frames(self):
        nepi_system.set_navpose_frames(self.navpose_frames)

        topics = []
        for frame in self.navpose_frames:
            if frame == 'None':
                topics.append('')
            else:
                topics.append(self.base_namespace + '/navposes/' + frame + '/navpose')
        self.navpose_frame_topics = topics
        #nepi_system.set_navpose_frame_topics(topics)
        
        self.navpose_info_dict['frame_nav'] = self.frame_nav
        self.navpose_info_dict['frame_alt'] = self.frame_alt
        self.navpose_info_dict['frame_depth'] = self.frame_depth
        nepi_system.set_navpose_settings(self.navpose_info_dict)

    #######################
    ### Node Methods

    def setFrame3dDescription(self,desc):
        self.navpose_description = desc
        self.publish_status(do_updates = False)
        self.update_navpose_frames()
        if self.node_if is not None:
            self.node_if.set_param('navpose_description',desc)
            self.node_if.save_config()
            
            
    def setFrameNav(self,frame):
        self.frame_nav = frame
        self.publish_status(do_updates = False)
        self.update_navpose_frames()
        if self.node_if is not None:
            self.node_if.set_param('frame_nav',frame)
            self.node_if.save_config()

    def setFrameAlt(self,frame):
        self.frame_alt = frame
        self.publish_status(do_updates = False)
        self.update_navpose_frames()
        if self.node_if is not None:
            self.node_if.set_param('frame_alt',frame)
            self.node_if.save_config()

    def setFrameDepth(self,frame):
        self.frame_depth = frame
        self.publish_status(do_updates = False)
        self.update_navpose_frames()
        if self.node_if is not None:
            self.node_if.set_param('frame_depth',frame)
            self.node_if.save_config()
     

    def setPublishRateCb(self,rate):
        min = self.NAVPOSE_PUB_RATE_OPTIONS[0]
        max = self.NAVPOSE_PUB_RATE_OPTIONS[1]
        if rate < min:
            rate = min
        if rate > max:
            rate = max
        self.set_pub_rate = rate
        self.publish_status(do_updates = False)
        if self.node_if is not None:
            self.node_if.set_param('pub_rate',rate)
            self.node_if.save_config()


    def setTopic(self,name,topic,transform = None):
        self.connect_dict[name]['fixed'] = False
        self._connectTopic(name,topic,transform = transform)
        if self.node_if is not None:
            self.node_if.set_param('connect_dict',self.connect_dict)
            self.node_if.save_config()

    def clearTopic(self,name):
        self._unregisterTopic(name)

    def setTransform(self,name,transform = None):
        if transform is not None:
            if len(transform) == 7:
                topic = self.connect_dict[name]['topic']
                self.connect_dict[name]['transform'] = transform
                if topic != '':
                    self.transforms_dict[topic] = transform
            if self.node_if is not None:
                self.node_if.set_param('connect_dict',self.connect_dict)
                self.node_if.save_config()

    def clearTransform(self,name):
        if name in self.connect_dict.keys():
            self.connect_dict[name]['transform'] = self.ZERO_TRANSFORM
            topic = self.connect_dict[name]['topic']
            if topic != '':
                self.transforms_dict[topic] = self.ZERO_TRANSFORM
            if self.node_if is not None:
                self.node_if.set_param('transforms_dict',self.transforms_dict)
                self.node_if.save_config()


    def _updateConnectionsCb(self, timer):
        # Register new topic requests
        for name in self.connect_dict.keys():
            topic = self.connect_dict[name]['topic']
            if topic != subs_dict[name]['topic']:
                transform = self.ZERO_TRANSFORM
                # Fixed: Use self.transforms_dict instead of undefined transform_dict 
                if topic in self.transforms_dict.keys():
                    transform = self.transforms_dict[topic]
                self._connectTopic(name, topic, transform=transform)
        nepi_sdk.start_timer_process(1.0, self._updateConnectionsCb, oneshot=True)

    def _connectTopic(self, name, topic, transform=None):
        self.navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        if name in self.avail_topics_dict.keys():
            if topic == 'None' or topic == '':
                success = self._unregisterTopic(name)
            elif topic == 'fixed':
                self.connect_dict[name]['fixed'] = True

            elif topic in self.avail_topics_dict[name]['topics']:
                self.connect_dict[name]['fixed'] = False

                if self.subs_dict[name]['topic'] != '':
                    success = self._unregisterTopic(name)
                topic_index = self.avail_topics_dict[name]['topics'].index(topic)
                msg_str = self.avail_topics_dict[name]['msgs'][topic_index]
                
                if msg_str in nepi_nav.NAVPOSE_MSG_DICT[name].keys():
                    msg = nepi_nav.NAVPOSE_MSG_DICT[name][msg_str]
                    cb = self.cb_dict[name]

                    self.subs_dict_lock.acquire()
                    self.subs_dict[name]['topic'] = topic
                    self.publish_status(do_updates = False)
                    sub = nepi_sdk.create_subscriber(topic, msg, cb, callback_args=(name,))
                    self.subs_dict[name]['msg'] = msg_str
                    self.connect_dict[name]['msg'] = msg_str
                    self.subs_dict[name]['sub'] = sub  
                    self.connect_dict[name]['topic'] = topic
                    self.subs_dict_lock.release()
                    
                    new_transform = self.ZERO_TRANSFORM
                    if transform is not None:
                        new_transform = transform
                        self.transforms_dict[topic] = new_transform
                    elif topic in self.transforms_dict.keys():
                        new_transform = self.transforms_dict[topic]
                    self.connect_dict[name]['transform'] = new_transform
                    if self.node_if is not None:
                        self.node_if.set_param('connect_dict',self.connect_dict)
                        self.node_if.set_param('transforms_dict',self.transforms_dict)
                        self.node_if.save_config()

    def _compSubCb(self, msg, args):
        name = args
        self.connect_dict[name]['connected'] = True
        # Update copy of dict
        self.navpose_dict_lock.acquire()
        npdata_dict = copy.deepcopy(self.navpose_dict)
        self.navpose_dict_lock.release()
        transform = self.connect_dict[name]['transform']
        # Fixed: typo in 'transform' parameter name
        npdata_dict = nepi_nav.update_navpose_dict_from_msg(name, npdata_dict, msg, transform=transform)

        # Now update class dict
        self.navpose_dict_lock.acquire()
        self.navpose_dict = npdata_dict
        self.navpose_dict_lock.release()            
        
        # Update time info - Fixed list manipulation
        last_time = self.connect_dict[name]['last_time']
        cur_time = nepi_utils.get_time()
        times = self.connect_dict[name]['times']
        times.pop(0)  # Remove first element
        times.append(cur_time - last_time)  # Add new time difference
        self.connect_dict[name]['times'] = times  # Assign back to dict
        self.connect_dict[name]['last_time'] = cur_time

    def setNavPose(self, npdata_dict):
        if npdata_dict is not None:
            if npdata_dict['has_location'] == True and self.connect_dict['location']['fixed'] == True:
                success = self._unregisterTopic('location')
                self.transforms_dict['location'] = self.ZERO_TRANSFORM
            else:
                self.connect_dict['location']['fixed'] = False

            if npdata_dict['has_heading'] == True and self.connect_dict['heading']['fixed'] == True:
                success = self._unregisterTopic('heading') 
                self.transforms_dict['heading'] = self.ZERO_TRANSFORM
            else:
                self.connect_dict['heading']['fixed'] = False

            if npdata_dict['has_orientation'] == True and self.connect_dict['orientation']['fixed'] == True:
                success = self._unregisterTopic('orientation') 
                self.transforms_dict['orientation'] = self.ZERO_TRANSFORM
            else:
                self.connect_dict['orientation']['fixed'] = False

            if npdata_dict['has_position'] == True and self.connect_dict['position']['fixed'] == True:
                success = self._unregisterTopic('position') 
                
                self.transforms_dict['position'] = self.ZERO_TRANSFORM
            else:
                self.connect_dict['position']['fixed'] = False

            if npdata_dict['has_altitude'] == True and self.connect_dict['altitude']['fixed'] == True:
                success = self._unregisterTopic('altitude')  
                self.transforms_dict['altitude'] = self.ZERO_TRANSFORM
            else:
                self.connect_dict['altitude']['fixed'] = False

            if npdata_dict['has_depth'] == True and self.connect_dict['depth']['fixed'] == True:
                success = self._unregisterTopic('depth')  
                self.transforms_dict['depth'] = self.ZERO_TRANSFORM
            else:
                self.connect_dict['depth']['fixed'] = False

            if npdata_dict['has_pan_tilt'] == True and self.connect_dict['has_pan_tilt']['fixed'] == True:
                success = self._unregisterTopic('depth')  
                self.transforms_dict['pan_tilt'] = self.ZERO_TRANSFORM
            else:
                self.connect_dict['has_pan_tilt']['fixed'] = False

            navpose_dict = copy.deepcopy(self.navpose_dict)
            navpose_dict = nepi_nav.update_navpose_dict_from_dict(navpose_dict, npdata_dict)
            self.navpose_dict_lock.acquire()
            self.navpose_dict = navpose_dict
            self.navpose_dict_lock.release()


    def resetNavPose(self):
        self.navpose_dict_lock.acquire()
        self.navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        self.navpose_dict['navpose_frame'] = 'nepi_frame'
        self.navpose_dict['navpose_description'] = 'nepi base frame'
        self.navpose_dict_lock.release()
        if self.node_if is not None:
            self.node_if.set_param('init_nav_pose',navpose_dict)

    def applyInitNavPose(self,npdata_dict):
        # Need to add 
        return npdata_dict




    def get_navpose_dict(self):
        self.navpose_dict_lock.acquire() 
        navpose_dict = copy.deepcopy(self.navpose_dict)
        self.navpose_dict_lock.release()
        return navpose_dict

        



    #######################
    # Private Members
    #######################

    def _navposeCapsQueryHandler(self,req):
        return self.caps_response

    def _navposeDataQueryHandler(self,req):
        response = NavPoseQueryResponse()
        self.navpose_dict_lock.acquire()
        npdata_dict = copy.deepcopy(self.navpose_dict)
        self.navpose_dict_lock.release()
        npdata_msg = nepi_nav.convert_navpose_dict2msg(npdata_dict) 
        response.navpose_data = npdata_msg
        return response

    def _updateAvailTopicsCb(self,timer):
        last_dict = copy.deepcopy(self.avail_topics_dict)
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('location')
        self.avail_topics_dict['location']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['location']['msgs'] = copy.deepcopy(msg_list)
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('heading')
        self.avail_topics_dict['heading']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['heading']['msgs'] = copy.deepcopy(msg_list)
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('orientation')
        self.avail_topics_dict['orientation']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['orientation']['msgs'] = copy.deepcopy(msg_list)

      
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('position')
        self.avail_topics_dict['position']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['position']['msgs'] = copy.deepcopy(msg_list)

        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('altitude')
        self.avail_topics_dict['altitude']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['altitude']['msgs'] = copy.deepcopy(msg_list)
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('depth')
        self.avail_topics_dict['depth']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['depth']['msgs'] = copy.deepcopy(msg_list)

        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('pan_tilt')
        self.avail_topics_dict['pan_tilt']['topics'] = copy.deepcopy(topic_list)
        self.avail_topics_dict['pan_tilt']['msgs'] = copy.deepcopy(msg_list)

        if self.avail_topics_dict != last_dict:
            self.publish_status()
        #self.msg_if.pub_warn("Updater - Got avail_topics_dict: " + str(self.avail_topics_dict))
        nepi_sdk.start_timer_process(5.0, self._updateAvailTopicsCb, oneshot = True)

    def _updateConnectionsCb(self, timer):
        # Register new topic requests
        for name in self.connect_dict.keys():
            topic = self.connect_dict[name]['topic']
            if topic != self.subs_dict[name]['topic']:
                transform = self.ZERO_TRANSFORM
                if topic in self.transforms_dict.keys():
                    transform = self.transforms_dict[topic]
                self._connectTopic(name, topic, transform=transform)
        nepi_sdk.start_timer_process(1.0, self._updateConnectionsCb, oneshot=True)

    def _connectTopic(self, name, topic, transform=None):
        if name in self.avail_topics_dict.keys():
            self.connect_dict[name]['fixed'] = False
            if topic == 'None' or topic == '':
                success = self._unregisterTopic(name)
            elif topic in self.avail_topics_dict[name]['topics']:
                if self.subs_dict[name]['topic'] != '':
                    success = self._unregisterTopic(name)
                # Fixed: Get message index properly
                topic_index = self.avail_topics_dict[name]['topics'].index(topic)
                msg_str = self.avail_topics_dict[name]['msgs'][topic_index]
                
                if msg_str in nepi_nav.NAVPOSE_MSG_DICT[name].keys():
                    msg = nepi_nav.NAVPOSE_MSG_DICT[name][msg_str]
                    cb = self.cb_dict[name]

                    self.subs_dict_lock.acquire()
                    sub = nepi_sdk.create_subscriber(topic, msg, cb, callback_args=(name,))
                    self.subs_dict[name]['topic'] = topic
                    self.subs_dict[name]['msg'] = msg_str
                    self.connect_dict[name]['msg'] = msg_str
                    self.subs_dict[name]['sub'] = sub  # Fixed: assign to 'sub' key
                    self.connect_dict[name]['topic'] = topic
                    self.subs_dict_lock.release()
                    
                    new_transform = self.ZERO_TRANSFORM
                    if transform is not None:
                        new_transform = transform
                        self.transforms_dict[topic] = new_transform
                    elif topic in self.transforms_dict.keys():
                        new_transform = self.transforms_dict[topic]
                    # Fixed: assign transform properly
                    self.connect_dict[name]['transform'] = new_transform


    def _unregisterTopic(self,name):
            self.subs_dict_lock.acquire()
            if self.subs_dict[name]['sub'] is not None:
                self.subs_dict[name]['sub'].unregister()
                nepi_sdk.sleep(1)
            self.subs_dict[name] = self.BLANK_SUB
            self.connect_dict[name] = self.BLANK_CONNECT
            self.connect_dict[name]['fixed'] = True
            self.subs_dict_lock.release()
            
            


    def _compSubCb(self, msg, args):
        name = args
        self.connect_dict[name]['connected'] = True
        # Update copy of dict
        self.navpose_dict_lock.acquire()
        npdata_dict = copy.deepcopy(self.navpose_dict)
        self.navpose_dict_lock.release()
        transform = self.connect_dict[name]['transform']
        npdata_dict = nepi_nav.update_navpose_dict_from_msg(name, npdata_dict, msg, transform=transform)

        # Now update class dict
        self.navpose_dict_lock.acquire()
        self.navpose_dict = npdata_dict
        self.navpose_dict_lock.release()            
        
        # Update time info - Fixed list manipulation
        last_time = copy.deepcopy(self.connect_dict[name]['last_time'])
        times = copy.deepcopy(self.connect_dict[name]['times'])
        times.pop(0)  # Remove first element
        times.append(nepi_utils.get_time() - last_time)  # Add new time difference
        self.connect_dict[name]['times'] = times  # Assign back to dict
        self.connect_dict[name]['last_time'] = nepi_utils.get_time()

    def _setPublishRateCb(self,msg):
        rate = msg.data
        self.setPublishRate(rate)
       

    def _setTopicCb(self,msg):
        name = msg.name
        topic = msg.topic
        apply_tr = msg.apply_transform
        transform = None
        if apply_tr:
            transform = msg.transform

        self.setTopic(name, topic, transform = transform)
       

    def _clearTopicCb(self,msg):
        self.clearTopic(msg.data)
        

    def _setFrameDescCb(self,msg):
        self.setFrame3dDescription(msg.data) 


    def _setFrameNavCb(self,msg):
        self.setFrameNav(msg.data) 

    def _setFrameAltCb(self,msg):
        self.setFrameAlt(msg.data) 

    def _setFrameDepthCb(self,msg):
        self.setFrameDepth(msg.data) 

    def _setTransformCb(self,msg):
        name = msg.name
        transform = nepi_nav.convert_transform_msg2list(msg.transform)
        self.setTransform(name, transform)

    def _clearTransformCb(self,msg):
        self.clearTransform(msg.data)

    def _setNavPoseCb(self,msg):
        npdata_dict = nepi_nav.convert_navpose_msg2dict(msg)
        self.setNavPose(npdata_dict)
       
    def _resetNavPoseCb(self,msg):
        self.resetNavPose()

    def _setInitNavPoseCb(self,msg):
        npdata_dict = nepi_nav.convert_navpose_msg2dict(msg)
        self.setInitNavPoseCb(npdata_dict)
       
    def _resetInitNavPoseCb(self,msg):
        self.resetInitNavPose()


    def _locationSubCb(self, msg, args=None):
        self._compSubCb(msg, 'location')

    def _headingSubCb(self, msg, args=None):
        self._compSubCb(msg, 'heading')

    def _orientationSubCb(self, msg, args=None):
        self._compSubCb(msg, 'orientation')

    def _positionSubCb(self, msg, args=None):
        self._compSubCb(msg, 'position')

    def _altitudeSubCb(self, msg, args=None):
        self._compSubCb(msg, 'altitude')

    def _depthSubCb(self, msg, args=None):
        self._compSubCb(msg, 'depth')

    def _panTiltSubCb(self, msg, args=None):
        self._compSubCb(msg, 'pan_tilt')


    ### Setup a regular background navpose get and publish timer callback
    def _getPublishSaveDataCb(self,timer):
        timestamp = nepi_utils.get_time()
        # Get current NEPI NavPose data from NEPI ROS navpose_query service call
        self.navpose_dict_lock.acquire()
        npdata_dict = copy.deepcopy(self.navpose_dict)
        self.navpose_dict_lock.release()
        npdata_dict = self.applyInitNavPose(npdata_dict)
        npsdata_dict = dict()
        if npdata_dict is not None:
            #self.msg_if.pub_warn("Got navpose data dict: " + str(npdata_dict))

            npdata_msg = nepi_nav.convert_navpose_dict2msg(npdata_dict)
            #self.msg_if.pub_warn("Got navpose data msg: " + str(npdata_msg))
            npsdata_msg = NavPoses()
            frames = [npdata_dict['navpose_frame']]
            npsdata_msg.navposes = [npdata_msg]

            npsdata_dict[npdata_dict['navpose_frame']] = npdata_dict
            if npdata_msg is not None:
                self.status_msg.publishing = True
                if self.node_if is not None:
                    self.node_if.publish_pub('navpose_pub',npsdata_msg)  
            if self.last_npdata_dict != npdata_dict:
                if self.save_data_if is not None:
                    self.save_data_if.save('navposes',npsdata_dict,timestamp)
            # Setup nex update check
            self.last_npdata_dict = npdata_dict

        delay = float(1.0)/self.set_pub_rate
        nepi_sdk.start_timer_process(delay, self._getPublishSaveDataCb, oneshot = True)

    def _publishStatusCb(self,timer):
        self.publish_status()


    def publish_status(self, do_updates = False):
        #self.msg_if.pub_warn("========publish_status called========")
        #self.msg_if.pub_warn("publish_status self.transforms_dict" + str(self.transforms_dict))


        frames_dict = copy.deepcopy(self.navpose_frames_info_dict)
        frames = []
        topics = []
        for frame_name in frames_dict.keys():
            frames.append(frame_name)
            topics.append(frames_dict[frame_name]['topic'])
        
        self.status_msg.navpose_frames = frames
        self.status_msg.navpose_frame_topics = topics


        connect_dict = copy.deepcopy(self.connect_dict)
        avail_topics_dict = copy.deepcopy(self.avail_topics_dict)


        comp_names = []
        comp_infos = []
        for name in self.connect_dict.keys():
            comp_names.append(name)

            comp_info = MgrNavPoseCompInfo()
            comp_info.name = name
            comp_info.available_topics = avail_topics_dict[name]['topics']
            comp_info.available_topic_msgs = avail_topics_dict[name]['msgs']
            comp_info.fixed = connect_dict[name]['fixed']
            comp_info.topic = connect_dict[name]['topic']
            comp_info.topic_msg = connect_dict[name]['msg']

            times = connect_dict[name]['times']
            avg_times = sum(times)/len(times)
            if avg_times > .01:
                avg_rate = float(1.0) / avg_times
            else:
                avg_rate = 0
            comp_info.avg_rate = round( avg_rate, 3)
            comp_info.last_time = round( nepi_utils.get_time() - connect_dict[name]['last_time'], 3)
            transform = connect_dict[name]['transform']

            comp_info.transform = nepi_nav.convert_transform_list2msg(transform, source_ref_description = 'data_source', end_ref_description = 'nepi_frame')

            comp_infos.append(comp_info)
        self.status_msg.comp_names = comp_names
        self.status_msg.comp_infos = comp_infos

        is_saving = False
        save_rate = 0.0
        if self.save_data_if is not None:
            is_saving = self.save_data_if.get_saving_enabled()
            save_rate = self.save_data_if.data_product_save_rate('navposes')
        
        self.status_msg.is_saving = is_saving
        self.status_msg.save_rate = save_rate

        self.status_msg.pub_rate = self.set_pub_rate
        #self.msg_if.pub_warn("will publish status msg: " + str(self.status_msg))
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub',self.status_msg)



        self.navposes_status_msg.navpose_frames = frames
        self.navposes_status_msg.navpose_topics = topics

        if self.node_if is not None:
            self.node_if.publish_pub('navposes_status_pub',self.navposes_status_msg)



    def _cleanupActions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################


if __name__ == '__main__':
    NavPoseMgr()



