#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

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

from nepi_interfaces.msg import NavPose
from nepi_interfaces.msg import NavPoseLocation, NavPoseHeading
from nepi_interfaces.msg import NavPoseOrientation, NavPosePosition
from nepi_interfaces.msg import NavPoseAltitude, NavPoseDepth

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

    node_if = None
    data_products_list = ['navpose']
    navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
    navpose_dict['frame_3d'] = 'nepi_frame'
    navpose_dict_lock = threading.Lock()
    init_navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
    last_npdata_dict = nepi_nav.BLANK_NAVPOSE_DICT

    mgr_namespace = ""
    status_msg = MgrNavPoseStatus()
    caps_response = MgrNavPoseCapabilitiesQueryResponse()

    set_pub_rate = FACTORY_PUB_RATE_HZ


    connect_dict = {
        'location': BLANK_CONNECT,
        'heading': BLANK_CONNECT,
        'orientation': BLANK_CONNECT,
        'position': BLANK_CONNECT,
        'altitude': BLANK_CONNECT,
        'depth': BLANK_CONNECT
    }

    subs_dict_lock = threading.Lock()
    subs_dict = {
        'location': BLANK_SUB,
        'heading': BLANK_SUB,
        'orientation': BLANK_SUB,
        'position': BLANK_SUB,
        'altitude': BLANK_SUB,
        'depth': BLANK_SUB
    }

    transforms_dict = dict()

    avail_topics_dict = {
        'location': BLANK_AVIAL_TOPIC,
        'heading': BLANK_AVIAL_TOPIC,
        'orientation': BLANK_AVIAL_TOPIC,
        'position': BLANK_AVIAL_TOPIC,
        'altitude': BLANK_AVIAL_TOPIC,
        'depth': BLANK_AVIAL_TOPIC
    }


    nepi_frame_desc = 'Undefined'
    frame_nav = 'ENU'
    frame_alt = 'WGS84'
    frame_depth = 'DEPTH'

    navpose_frames_dict = nepi_nav.BLANK_NAVPOSE_FRAMES_DICT

    data_source_description = 'nepi_navpose_solution'
    data_ref_description = 'nepi_frame'
    device_mount_description = 'fixed'
    add_mount_description = ''


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
        self.mgr_namespace = nepi_sdk.create_namespace(self.base_namespace, self.MGR_NODE_NAME)

        self.cb_dict = {
            'location': self._locationSubCb,
            'heading': self._headingSubCb,
            'orientation': self._orientationSubCb,
            'position': self._positionSubCb,
            'altitude': self._altitudeSubCb,
            'depth': self._depthSubCb,
        }


        self.caps_response.frame_nav_options = self.NAVPOSE_NAV_FRAME_OPTIONS 
        self.caps_response.frame_alt_options = self.NAVPOSE_ALT_FRAME_OPTIONS
        self.caps_response.frame_depth_options = self.NAVPOSE_DEPTH_FRAME_OPTIONS


        self.status_msg.frame_nav_options = self.NAVPOSE_NAV_FRAME_OPTIONS 
        self.status_msg.frame_alt_options = self.NAVPOSE_ALT_FRAME_OPTIONS
        self.status_msg.frame_depth_options = self.NAVPOSE_DEPTH_FRAME_OPTIONS
        self.status_msg.publishing = False
        self.status_msg.pub_rate = self.set_pub_rate

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
            'nepi_frame_desc': {
                'namespace': self.mgr_namespace,
                'factory_val': self.nepi_frame_desc
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
                'namespace': self.base_namespace,
                'topic': 'capabilities_query',
                'srv': MgrNavPoseCapabilitiesQuery,
                'req': MgrNavPoseCapabilitiesQueryRequest(),
                'resp': MgrNavPoseCapabilitiesQueryResponse(),
                'callback': self._navposeCapsQueryHandler
            },
            'navpose_query': {
                'namespace': self.base_namespace,
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
            'navpose_data': {
                'namespace': self.base_namespace,
                'topic': 'navpose_data',
                'msg': NavPose,  # This should be the correct message type
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

        # Setup navpose data IF
        np_namespace = self.base_namespace
        self.navpose_if = NavPoseIF(namespace = np_namespace,
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        log_name = 'navpose',
                        log_name_list = [],
                        msg_if = self.msg_if
                        )

        ##############################
        # Set up save data services ########################################################
        factory_data_rates = {}
        for d in self.data_products_list:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
            if d == 'navpose':
                factory_data_rates[d][0] = float(1.0) / self.FACTORY_PUB_RATE_HZ
        self.save_data_if = SaveDataIF(data_products = self.data_products_list, factory_rate_dict = factory_data_rates,namespace = self.node_namespace)


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
        nepi_sdk.start_timer_process(1.0, self._publishNavPoseCb, oneshot = True)

        ##############################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        nepi_sdk.spin()


    def initCb(self,do_updates = False):
        if self.node_if is not None:
            self.nepi_frame_desc = self.node_if.get_param('nepi_frame_desc')
            self.frame_nav = self.node_if.get_param('frame_nav')
            self.frame_alt = self.node_if.get_param('frame_alt')
            self.frame_depth = self.node_if.get_param('frame_depth')
            self.set_pub_rate = self.node_if.get_param('pub_rate')
            self.connect_dict = self.node_if.get_param('connect_dict')
            self.transforms_dict = self.node_if.get_param('transforms_dict')
            self.init_navpose_dict = self.node_if.get_param('init_navpose_dict')
        self.navpose_dict_lock.acquire()
        self.navpose_dict = self.init_navpose_dict
        self.navpose_dict_lock.release()
        if do_updates == True:
            self.update_navpose_frames()
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
        self.navpose_frames_dict['nepi_frame_desc'] = self.nepi_frame_desc
        self.navpose_frames_dict['frame_nav'] = self.frame_nav
        self.navpose_frames_dict['frame_alt'] = self.frame_alt
        self.navpose_frames_dict['frame_depth'] = self.frame_depth
        nepi_system.set_navpose_frames(self.navpose_frames_dict)

    #######################
    ### Node Methods

    def setFrame3dDescription(self,desc):
        self.nepi_frame_desc = desc
        self.publish_status(do_updates = False)
        self.update_navpose_frames()
        if self.node_if is not None:
            self.node_if.set_param('nepi_frame_desc',desc)
            
            
    def setFrameNav(self,frame):
        self.frame_nav = frame
        self.publish_status(do_updates = False)
        self.update_navpose_frames()
        if self.node_if is not None:
            self.node_if.set_param('frame_nav',frame)

    def setFrameAlt(self,frame):
        self.frame_alt = frame
        self.publish_status(do_updates = False)
        self.update_navpose_frames()
        if self.node_if is not None:
            self.node_if.set_param('frame_alt',frame)

    def setFrameDepth(self,frame):
        self.frame_depth = frame
        self.publish_status(do_updates = False)
        self.update_navpose_frames()
        if self.node_if is not None:
            self.node_if.set_param('frame_depth',frame)
     

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


    def setTopic(self,name,topic,transform = None):
        self._connectTopic(name,topic,transform = transform)

    def clearTopic(self,name):
        self._unregisterTopic(name)

    def setTransform(self,name,transform = None):
        if transform is not None:
            if len(transform) == 7:
                topic = self.connect_dict[name]['topic']
                self.connect_dict[name]['transform'] = transform
                if topic != '':
                    self.transforms_dict[topic] = transform

    def clearTransform(self,name):
        if name in self.connect_dict.keys():
            self.connect_dict[name]['transform'] = self.ZERO_TRANSFORM
            topic = self.connect_dict[name]['topic']
            if topic != '':
                self.transforms_dict[topic] = self.ZERO_TRANSFORM

    def _updateConnectionsCb(self, timer):
        # Register new topic requests
        self.subs_dict_lock.acquire()
        subs_dict = copy.deepcopy(self.subs_dict)
        self.subs_dict_lock.release()
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

    def setNavPoseCb(self, npdata_dict):
        if npdata_dict is not None:
            if npdata_dict['has_location'] == True:
                success = self._unregisterTopic('location')  # Fixed: removed self parameter
                self.connect_dict['location']['fixed'] = True
                self.transforms_dict['location'] = self.ZERO_TRANSFORM
            if npdata_dict['has_heading'] == True:
                success = self._unregisterTopic('heading')  # Fixed: removed self parameter
                self.connect_dict['heading']['fixed'] = True
                self.transforms_dict['heading'] = self.ZERO_TRANSFORM
            if npdata_dict['has_orientation'] == True:
                success = self._unregisterTopic('orientation')  # Fixed: removed self parameter
                self.connect_dict['orientation']['fixed'] = True
                self.transforms_dict['orientation'] = self.ZERO_TRANSFORM
            if npdata_dict['has_position'] == True:
                success = self._unregisterTopic('position')  # Fixed: removed self parameter
                self.connect_dict['position']['fixed'] = True
                self.transforms_dict['position'] = self.ZERO_TRANSFORM
            if npdata_dict['has_altitude'] == True:
                success = self._unregisterTopic('altitude')  # Fixed: removed self parameter
                self.connect_dict['altitude']['fixed'] = True
                self.transforms_dict['altitude'] = self.ZERO_TRANSFORM
            if npdata_dict['has_depth'] == True:
                success = self._unregisterTopic('depth')  # Fixed: removed self parameter
                self.connect_dict['depth']['fixed'] = True
                self.transforms_dict['depth'] = self.ZERO_TRANSFORM
            navpose_dict = copy.deepcopy(self.navpose_dict)
            navpose_dict = nepi_nav.update_navpose_dict_from_dict(navpose_dict, npdata_dict)
            self.navpose_dict_lock.acquire()
            self.navpose_dict = navpose_dict
            self.navpose_dict_lock.release()


    def resetNavPose(self):
        self.navpose_dict_lock.acquire()
        self.navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
        self.navpose_dict['frame_3d'] = 'nepi_frame'
        self.navpose_dict_lock.release()
        if self.node_if is not None:
            self.node_if.set_param('init_nav_pose',navpose_dict)

    def applyInitNavPose(self,npdata_dict):
        # Need to add 
        return npdata_dict





    def publish_status(self, do_updates = False):
        #self.msg_if.pub_warn("========publish_status called========")

        connect_dict = copy.deepcopy(self.connect_dict)
        subs_dict = copy.deepcopy(self.subs_dict)
        avail_topics_dict = copy.deepcopy(self.avail_topics_dict)

        self.status_msg.nepi_frame_description = self.nepi_frame_desc

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
            comp_info.avg_rate = avg_rate
            comp_info.last_time = connect_dict[name]['last_time']
            transform = connect_dict[name]['transform']

            comp_info.transform = nepi_nav.convert_transform_list2msg(transform, source_ref_description = 'data_source', end_ref_description = 'nepi_frame')

            comp_infos.append(comp_info)
        self.status_msg.comp_names = comp_names
        self.status_msg.comp_infos = comp_infos


        self.status_msg.pub_rate = self.set_pub_rate
        #self.msg_if.pub_warn("will publish status msg: " + str(self.status_msg))
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub',self.status_msg)



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
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('location')
        self.avail_topics_dict['location']['topics'] = topic_list
        self.avail_topics_dict['location']['msgs'] = msg_list
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('heading')
        self.avail_topics_dict['heading']['topics'] = topic_list
        self.avail_topics_dict['heading']['msgs'] = msg_list
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('orientation')
        self.avail_topics_dict['orientation']['topics'] = topic_list
        self.avail_topics_dict['orientation']['msgs'] = msg_list
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('position')
        self.avail_topics_dict['position']['topics'] = topic_list
        self.avail_topics_dict['position']['msgs'] = msg_list
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('altitude')
        self.avail_topics_dict['altitude']['topics'] = topic_list
        self.avail_topics_dict['altitude']['msgs'] = msg_list
        [topic_list,msg_list] = nepi_nav.get_navpose_comp_publisher_namespaces('depth')
        self.avail_topics_dict['depth']['topics'] = topic_list
        self.avail_topics_dict['depth']['msgs'] = msg_list

        nepi_sdk.start_timer_process(5.0, self._updateAvailTopicsCb, oneshot = True)

    def _updateConnectionsCb(self, timer):
        # Register new topic requests
        self.subs_dict_lock.acquire()
        subs_dict = copy.deepcopy(self.subs_dict)
        self.subs_dict_lock.release()
        for name in self.connect_dict.keys():
            topic = self.connect_dict[name]['topic']
            if topic != subs_dict[name]['topic']:
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
        last_time = self.connect_dict[name]['last_time']
        cur_time = nepi_utils.get_time()
        times = self.connect_dict[name]['times']
        times.pop(0)  # Remove first element
        times.append(cur_time - last_time)  # Add new time difference
        self.connect_dict[name]['times'] = times  # Assign back to dict
        self.connect_dict[name]['last_time'] = cur_time


    def _setPublishRateCb(self,msg):
        rate = msg.data
        self.setPublishRate(rate)
       

    def _setTopicCb(self,msg):
        name = msg.name
        topic = msg.topic
        apply_tr = msg.include_transform
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
        self.setTransform(msg.name, msg.transform)

    def _clearTransformCb(self,msg):
        self.clearTransform(msg.data)

    def _setNavPoseCb(self,msg):
        npdata_dict = nepi_nav.convert_navpose_msg2data(msg)
        self.setNavPose(npdata_dict)
       
    def _resetNavPoseCb(self,msg):
        self.resetNavPose()

    def _setInitNavPoseCb(self,msg):
        npdata_dict = nepi_nav.convert_navpose_msg2data(msg)
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


    ### Setup a regular background navpose get and publish timer callback
    def _getPublishSaveDataCb(self,timer):
        timestamp = nepi_utils.get_time()
        # Get current NEPI NavPose data from NEPI ROS navpose_query service call
        self.navpose_dict_lock.acquire()
        npdata_dict = copy.deepcopy(self.navpose_dict)
        self.navpose_dict_lock.release()
        npdata_dict = self.applyInitNavPose(npdata_dict)

        if npdata_dict is not None:
            #self.msg_if.pub_warn("Got navpose data dict: " + str(npdata_dict))
            npdata_msg = nepi_nav.convert_navpose_dict2msg(npdata_dict)
            #self.msg_if.pub_warn("Got navpose data msg: " + str(npdata_msg))
            if npdata_msg is not None:
                self.status_msg.publishing = True
                if self.node_if is not None:
                    self.node_if.publish_pub('navpose_data',npdata_msg)  
            if self.last_npdata_dict != npdata_dict:
                if self.save_data_if is not None:
                    self.save_data_if.save('navpose',npdata_dict,timestamp)
            # Setup nex update check
            self.last_npdata_dict = npdata_dict

        delay = float(1.0)/self.set_pub_rate
        nepi_sdk.start_timer_process(delay, self._getPublishSaveDataCb, oneshot = True)

    def _publishStatusCb(self,timer):
        self.publish_status()


    def _publishNavPoseCb(self,timer):
        self.status_msg.publishing = True
        if self.navpose_if is not None:
            np_dict = self.get_navpose_dict()
            self.navpose_if.publish_navpose(np_dict,
                        device_mount_description = self.device_mount_description)

    def _cleanupActions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################


if __name__ == '__main__':
    NavPoseMgr()


