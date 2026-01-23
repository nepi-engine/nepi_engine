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

import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_nav

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image

from nepi_interfaces.msg import ImageStatus

from sensor_msgs.msg import PointCloud2
from nepi_interfaces.msg import PointcloudStatus

from nepi_interfaces.msg import NavPose, NavPoses, NavPoseStatus, NavPosesStatus

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.connect_node_if import ConnectNodeClassIF



##################################################
## ConnectNavPosesIF

EXAMPLE_NAVPOSE_DATA_DICT = {
    'navpose_frame': 'nepi_frame',
    'frame_nav': 'ENU',
    'frame_altitude': 'WGS84',
    'frame_depth': 'MSL',

    'geoid_height_meters': 0,


    'has_location': True,
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'latitude': 47.080909,
    'longitude': -120.8787889,

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
    'time_orientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees in specified 3d frame
    'roll_deg': 30.51,
    'pitch_deg': 30.51,
    'yaw_deg': 30.51,

    'has_altitude': True,
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters in specified altitude_m frame
    'altitude_m': 12.321,

    'has_depth': False,
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive meters
    'depth_m': 0.0,

    'has_pan_tilt': False,
    'time_pan_tilt': nepi_utils.get_time(),
    # Pan Tilt should be provided in positive degs
    'pan_deg': 0.0,
    'tilt_deg': 0.0
}

EXAMPLE_NAVPOSES_DATA_DICT = {
    'navpose_frames': ['nepi_frame'],
    'navposes': [copy.deepcopy(EXAMPLE_NAVPOSE_DATA_DICT)]
}


class ConnectNavPosesIF:


    NAVPOSE_NAV_FRAME_OPTIONS = ['ENU','NED','UKNOWN']
    NAVPOSE_ALT_FRAME_OPTIONS = ['WGS84','AMSL','UKNOWN'] # ['WGS84','AMSL','AGL','MSL','HAE','BAROMETER','UKNOWN']
    NAVPOSE_DEPTH_FRAME_OPTIONS = ['DEPTH','UKNOWN'] # ['MSL','TOC','DF','KB','DEPTH','UKNOWN']


    DEFAULT_CALLBACK_DICT = dict(
        frame_updated_callback = None,
        navpose_updated_callback = None,
        navposes_updated_callback = None
    )
    callback_dict = copy.deepcopy(DEFAULT_CALLBACK_DICT)

    ready = False
    namespace = '~'

    node_if = None
    save_data_if = None

    status_msg = NavPosesStatus()

    data_poduct = 'navpose'
    data_products_list = ['navpose']

    navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    navposes_dict = dict()
    navpose_settings_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_INFO_DICT)
    navpose_frames = ['None']
    navpose_frame = 'None'

    connected = False

 
    def __init__(self, namespace = None,
                callback_dict = dict(),
                save_data_if = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            log_name = nepi_utils.get_clean_name(log_name)
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################    
        # Initialize Class Variables
        if namespace is not None:
            self.namespace = namespace
        namespace = nepi_sdk.create_namespace(namespace,'navpose')
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        
        for key in self.DEFAULT_CALLBACK_DICT.keys():
            if key not in callback_dict.keys():
                callback_dict[key] = self.DEFAULT_CALLBACK_DICT[key]
        self.callback_dict = callback_dict


        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Services Config Dict ####################     
        self.SRVS_DICT = None

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'navpose_frame': {
                'namespace': self.namespace,
                'factory_val': self.navpose_frame
            }
        }



        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'navpose_pub': {
                    'msg': NavPose,
                    'namespace': self.namespace,
                    'topic': '',
                    'qsize': 1,
                    'latch': True
                },
            'status_pub': {
                    'msg': NavPosesStatus,
                    'namespace': self.namespace,
                    'topic': 'status',
                    'qsize': 1,
                    'latch': True
                }
        }
     
        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'navposes_sub': {
                'namespace': self.base_namespace,
                'topic': 'navposes',
                'msg': NavPoses,
                'qsize': 1,
                'callback': self._navposesSubCb, 
                'callback_args': ()
            },
            'set_navpose_frame': {
                'namespace': self.namespace,
                'topic': 'set_navpose_frame',
                'msg': String,
                'qsize': 1,
                'callback': self._setNavposeFrameCb, 
                'callback_args': ()
            }
        }
        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        params_dict = self.PARAMS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                         msg_if = self.msg_if
                                            )

        success = nepi_sdk.wait()

        if save_data_if is not None:
            self.save_data_if = save_data_if
        else:
            
            # Setup Save Data IF Class 
            self.msg_if.pub_info("Starting Save Data IF Initialization")
            factory_data_rates= {}
            for d in self.data_products_list:
                factory_data_rates[d] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
            self.msg_if.pub_warn("Starting data products list: " + str(self.data_products_list))

            factory_filename_dict = {
                'prefix': "", 
                'add_timestamp': True, 
                'add_ms': True,
                'add_us': False,
                'suffix': "",
                'add_node_name': True
                }

            sd_namespace = self.namespace
            self.save_data_if = SaveDataIF(namespace = sd_namespace,
                                    data_products = self.data_products_list,
                                    factory_rate_dict = factory_data_rates,
                                    factory_filename_dict = factory_filename_dict,
                                    log_name_list = self.log_name_list,
                                    msg_if = self.msg_if)
            nepi_sdk.sleep(1)
        if self.save_data_if is not None:
            data_products = self.save_data_if.get_data_products()
            for data_product in self.data_products_list:
                if data_product not in data_products:
                    self.save_data_if.register_data_product(data_product)
            self.status_msg.save_data_topic = self.save_data_if.get_namespace()
            self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic))

       
        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)
        nepi_sdk.start_timer_process(1.0, self._saveDataCb, oneshot = True)


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def get_namespace(self):
        return self.namespace
    
    def get_data_products(self):
        return [self.data_product]

    def get_blank_navpose_dict(self):
        blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return blank_navpose_dict
    

    def get_navpose_frames(self):
        return self.navpose_frames
    
    def get_navpose_frame(self):
        return self.navpose_frame

    def set_navpose_frame(self,frame):
        if frame in self.navpose_frames:
            self.navpose_frame = frame
            if 'frame_updated_callback' in self.callback_dict.keys():
                self.callback_dict['frame_updated_callback'](frame)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.save_config()

    def get_navpose_dict(self):
        navpose_dict =  copy.deepcopy(self.navpose_dict)
        return navpose_dict
    
    def get_navposes_dict(self):
        navposes_dict =  copy.deepcopy(self.navposes_dict)
        return navposes_dict

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def save_data(self):
        if self.save_data_if is not None:
            navpose_dict = copy.deepcopy(self.navpose_dict)
            self.save_data_if.save('navpose',navpose_dict)
        nepi_sdk.start_timer_process(0.1, self._saveDataCb, oneshot = True)


    def get_navpose_callback_options(self):
        return list(self.callback_dict.keys())
    
    def set_navpose_callback(self,name,function):
        self.msg_if.pub_warn("Got set callback for: " + str(name), log_name_list = self.log_name_list)
        if name in self.callback_dict.keys():
            self.msg_if.pub_warn("Callback set for: " + str(name), log_name_list = self.log_name_list)
            self.callback_dict[name] = function
        #self.msg_if.pub_info("Updated callback dict: " + str(self.callback_dict), log_name_list = self.log_name_list)

    def clear_navpose_callback(self,name):
        self.msg_if.pub_warn("Got clear callback for: " + str(name), log_name_list = self.log_name_list)
        if name in self.callback_dict.keys():
            self.callback_dict[name] = None 

    def unsubsribe(self):
        self.ready = False
        if self.node_if is not None:
            self.node_if.unregister_class()
        time.sleep(1)
        self.namespace = None
        self.status_msg = NavPosesStatus()



    def publish_status(self):
        if self.node_if is not None and self.status_msg is not None:

            self.status_msg.navpose_frames = self.navpose_frames
            navpose_frame = copy.deepcopy(self.navpose_frame)
            if navpose_frame in self.navposes_dict.keys():
                navpose_frame = 'None'
            self.status_msg.navpose_frame = navpose_frame
           
            self.node_if.publish_pub('status_pub', self.status_msg)




    def init(self, do_updates = False):
        if self.node_if is not None:
            self.navpose_frame = self.node_if.get_param('navpose_frame')
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        if self.node_if is not None:
            pass
        self.init()

    def factory_reset(self):
        if self.node_if is not None:
            pass
        self.init()


    ###############################
    # Class Private Methods
    ###############################


    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)


    def _setNavposeFrameCb(self,msg):
        frame = msg.data
        if frame in self.navpose_frames:
            self.navpose_frame = frame
            if 'frame_updated_callback' in self.callback_dict.keys():
                self.callback_dict['frame_updated_callback'](frame)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.save_config()

    def _navposesSubCb(self,msg):
        self.connected = True
        self.navposes_dict = nepi_nav.convert_navposes_msg2dict(msg,self.log_name_list)
        self.navpose_frames = list(self.navposes_dict.keys())
        if 'navposes_updated_callback' in self.callback_dict.keys():
            self.callback_dict['navposes_updated_callback'](self.navposes_dict)

        last_navpose_dict = copy.deepcopy(self.navpose_dict)
        navpose_frame = copy.deepcopy(self.navpose_frame)
        if navpose_frame in self.navposes_dict.keys():
            self.navpose_dict = self.navposes_dict[navpose_frame]
        else:
            self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        if last_navpose_dict != self.navpose_dict and 'navpose_updated_callback' in self.callback_dict.keys():
            self.callback_dict['navpose_updated_callback'](self.navpose_dict)
            if self.node_if is not None:
                navpose_msg = nepi_nav.convert_navpose_dict2msg(self.navpose_dict,self.log_name_list)
                self.node_if.publish_pub('navpose_pub',navpose_msg)


    def _publishStatusCb(self,timer):
        self.publish_status()

    def _saveDataCb(self,timer):
        self.save_data()





############################################################
## ConnectImageIF
'''
EXAMPLE_DATA_DICT = {   
    'namespace':  self.node_namespace,     
    'data': None,
    'width': 0,
    'height': 0,
    'timestamp': nepi_sdk.get_time(),
    'ros_img_header': Header(),
    'ros_img_stamp': Header().stamp,
    'get_latency_time': 0,
    'got_latency_time': 0,
    'process_time': 0 
}
'''

class ConnectImageIF:

    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

    data_dict = None
    data_dict_lock = threading.Lock()

    get_data = False
    got_data = False



    #######################
    ### IF Initialization
    def __init__(self, 
                namespace,
                preprocess_function = None,
                callback_function = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

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
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)


        ##############################    
        # Initialize Class Variables


        self.preprocessFunction = preprocess_function
        self.callbackFunction = callback_function



        ##############################   
        ## Node Setup

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'img_sub': {
                'msg': Image,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'callback': self._dataCb
            },
            'status_sub': {
                'msg': ImageStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'callback': self._statusCb
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(
                        subs_dict = self.SUBS_DICT,
                                            log_name_list = self.log_name_list,
                                            msg_if = self.msg_if
                                            )


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    #######################
    # Class Public Methods
    #######################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Status Connected", log_name_list = self.log_name_list)
        return self.status_connected

    def get_status_dict(self):
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return self.img_status_dict

    def set_get_img(self,state):
        self.get_data = state
        return True

    def read_get_got_states(self):
        return [self.get_data, self.got_data]

    def get_image_dict(self):
        self.data_dict_lock.acquire()
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = None
        self.data_dict_lock.release()
        return data_dict

    def unregister(self):
        self._unsubscribeTopic()

    #################
    ## Save Config Functions

    def call_save_config(self):
        self.con_node_if.publish_pub('save_config',Empty())

    def call_reset_config(self):
        self.con_node_if.publish_pub('reset_config',Empty())

    def call_factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config',Empty())

    ###############################
    # Class Private Methods
    ###############################
   

    def _unsubscribeTopic(self):
        success = False
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_info("Unregistering topic: " + str(self.namespace))
            try:
                self.connected = False 
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.connected = False 
                self.data_dict = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image:  " + str(e))
        return success


    def _dataCb(self,data_msg):      
        #self.msg_if.pub_warn("Got img for topic:  " + self.namespace)

        # Process ros image message
        current_time = nepi_sdk.get_msg_stamp()
        get_msg_stampstamp = data_msg.header.stamp
        latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
        data_dict['get_latency_time'] = latency
        #self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(latency))

        start_time = nepi_sdk.get_time()   


        get_data = (self.callbackFunction is not None or self.get_data == True)
        #self.msg_if.pub_warn("Got Img with get_data: " + str(self.get_data) + " got_data: " + str(self.got_data))
        if get_data == True:
            self.get_data = False
           # self.msg_if.pub_warn("Got get_data flag. Processing img for topic:  " + self.namespace)
            ##############################
            ### Preprocess Image
            
            data = nepi_img.rosimg_to_cv2img(data_msg)

            if self.preprocessFunction is not None:
                try:
                    data = self.preprocessFunction(data)
                except Exception as e:
                    self.msg_if.pub_warn("Provided Image Preprocess Function failed:  " + str(e))

            data_dict = dict()
            data_dict['namespace'] = self.namespace
            data_dict['data'] = data
            height, width = data.shape[:2]
            data_dict['width'] = width 
            data_dict['height'] = height 
            get_msg_stampstamp = data_msg.header.stamp
            data_dict['timestamp'] = nepi_sdk.sec_from_msg_stamp(get_msg_stampstamp)
            data_dict['ros_img_header'] = data_msg.header
            data_dict['ros_img_stamp'] = get_msg_stampstamp
            ##############################

            process_time = round( (nepi_sdk.get_time() - start_time) , 3)
            data_dict['process_time'] = process_time

            latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
            data_dict['got_latency_time'] = latency
            #self.msg_if.pub_debug("Img Pub Latency: {:.2f}".format(latency))

            if self.callbackFunction is not None:
                self.callbackFunction(data_dict)
            else:
                self.data_dict_lock.acquire()
                self.data_dict = data_dict
                self.data_dict_lock.release()
                self.got_data = True
        self.connected = True


    def _statusCb(self,status_msg):      
        self.status_connected = True
        self.status_msg = status_msg



############################################################


'''
EXAMPLE_DATA_DICT = {      
    'namespace':  self.node_namespace, 
    'data': None,
    'width': 0,
    'height': 0,
    'depth': 0,
    'point_count': 0,
    'has_rgb': False,
    'timestamp': nepi_sdk.get_time(),
    'ros_pc_header': Header(),
    'ros_pc_stamp': Header().stamp,
    'get_latency_time':0,
    'pub_latency_time':0,
    'process_time':0
}
'''

class ConnectPointcloudIF:
    """AI is creating summary for 

    :return: [description]
    :rtype: [type]
    """

    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False


    data_dict = None
    data_dict_lock = threading.Lock()

    get_data = False
    got_data = False


 
    #######################
    ### IF Initialization
    def __init__(self, 
                namespace,
                preprocess_function = None,
                callback_function = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

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
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)


        ##############################    
        # Initialize Class Variables

        

        self.preprocessFunction = preprocess_function
        self.callbackFunction = callback_function


        ##############################   
        ## Node Setup

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'pc_sub': {
                'msg': PointCloud2,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'callback': self._dataCb
            },
            'status_sub': {
                'msg': PointcloudStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'callback': self._statusCb
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(
                        subs_dict = self.SUBS_DICT,
                                            log_name_list = self.log_name_list,
                                            msg_if = self.msg_if
                                            )

        #self.con_node_if.wait_for_ready()
        nepi_sdk.wait()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Status Connected", log_name_list = self.log_name_list)
        return self.status_connected

    def get_status_dict(self):
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return self.img_status_dict

    def set_get_img(self,state):
        self.get_data = state
        return True

    def read_get_got_states(self):
        return [self.get_data, self.got_data]

    def get_pointcloud_dict(self):
        self.data_dict_lock.acquire()
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = None
        self.data_dict_lock.release()
        return data_dict

    def unregister(self):
        self._unsubscribeTopic()

    #################
    ## Save Config Functions

    def call_save_config(self):
        self.con_node_if.publish_pub('save_config',Empty())

    def call_reset_config(self):
        self.con_node_if.publish_pub('reset_config',Empty())

    def call_factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config',Empty())
    ###############################
    # Class Private Methods
    ###############################
   

    def _unsubscribeTopic(self):
        success = False
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_warn("Unregistering topic: " + str(self.namespace))
            try:
                self.connected = False 
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.connected = False 
                self.data_dict = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image:  " + str(e))
        return success



    def _dataCb(self,data_msg):      
        self.connected = True
        #self.msg_if.pub_warn("Got pc for topic:  " + self.namespace)

        # Process ros pointcloud message
        current_time = nepi_sdk.get_msg_stamp()
        get_msg_stampstamp = data_msg.header.stamp
        latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
        self.data_dict['get_latency_time'] = latency
        #self.msg_if.pub_debug("Get Pc Latency: {:.2f}".format(latency))

        start_time = nepi_sdk.get_time()   

        get_data = (self.callbackFunction is not None or self.get_data == True)
        #self.msg_if.pub_warn("Got Pc with get_data: " + str(self.get_data) + " got_data: " + str(self.got_data))
        if get_data == True:
            self.get_data = False
           # self.msg_if.pub_warn("Got get_data flag. Processing pc for topic:  " + self.namespace)
            ##############################
            ### Preprocess Pointcloud
            
            data = nepi_pc.rospc_to_o3dpc(data_msg, remove_nans=True)

            if self.pointcloudPreprocessFunction is not None:
                try:
                    data = self.pointcloudPreprocessFunction(data)
                except Exception as e:
                    self.msg_if.pub_warn("Provided Pointcloud Preprocess Function failed:  " + str(e))

            data_dict = dict()
            data_dict['namespace'] = self.namespace
            data_dict['data'] = data

            data_dict['has_rgb'] = data.has_colors() 
            data_dict['has_intensity'] = False # Need to add

            data_dict['point_count'] = data.point["colors"].shape[0]

            data_dict['width'] = 0  # Need to add
            data_dict['height'] = 0 # Need to add
            data_dict['depth'] = 0 # Need to add

            get_msg_stampstamp = data_msg.header.stamp
            data_dict['timestamp'] = nepi_sdk.sec_from_msg_stamp(get_msg_stampstamp)
            data_dict['ros_pc_header'] = data_msg.header
            data_dict['ros_pc_stamp'] = get_msg_stampstamp
            ##############################

            process_time = round( (nepi_sdk.get_time() - start_time) , 3)
            data_dict['process_time'] = process_time

            latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
            data_dict['got_latency_time'] = latency
            #self.msg_if.pub_debug("Img Pub Latency: {:.2f}".format(latency))

            if self.callbackFunction is not None:
                self.callbackFunction(data_dict)
            else:
                self.data_dict_lock.acquire()
                self.data_dict = data_dict
                self.data_dict_lock.release()
                self.got_data = True
        self.connected = True


    def _statusCb(self,status_msg):      
        self.status_connected = True
        self.pc_status_msg = status_msg






