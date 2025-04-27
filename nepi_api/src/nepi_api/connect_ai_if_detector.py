#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#



import os
import time
import copy

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes
from nepi_ros_interfaces.msg import AiDetectorInfo, AiDetectorStatus
from nepi_ros_interfaces.srv import SystemStorageFolderQuery
from nepi_ros_interfaces.srv import AiDetectorInfoQuery,  AiDetectorInfoQueryRequest


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_ais


from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF


self.BOXES_INFO_DICT_ENTRY = {
    'model_name': 'test_model',
    'image_header': Header(),
    'image_topic': 'my/test_topic',
    'src_height': 600,
    'src_width': 1000,
    'prc_height': 300,
    'prc_width': 500
}


self.BOX_DICT_ENTRY = {
    'name': 'chair', # Class String Name
    'id': 1, # Class Index from Classes List
    'uid': '', # Reserved for unique tracking by downstream applications
    'prob': .3, # Probability of detection
    'xmin': 10,
    'ymin': 10,
    'xmax': 100,
    'ymax': 100,
    'area_ratio': 0.054,
    'area_pixels': 8100
}

self.BOXES_DICT = {
    'info': self.BOXES_INFO_DICT_ENTRY,
    'boxes': [self.BOX_DICT_ENTRY]
}

class ConnectAiDetectorIF:

    ready = False

    det_connected = False
    det_namespace = None
    det_info_msg = None
    det_status_sub = None
    det_status_msg = None

    det_active_img_dict = dict()
    det_classes = []
    det_classes_colors = []
    det_img_width = 0
    det_img_height = 0


    det_img_data_dict = dict()
    det_img_sub_dict = dict()

    det_auto_reg = False
    det_auto_dereg = False



    namespace = None
    connected = False
    active = False





 
    found_obj_msg = None
    found_obj_count = 0

    bounding_boxes_msg = None
    boxes_dict = dict()

    first_det_time = None
    last_det_time = nepi_ros.get_time()

    src_img_topic = None
    pub_img_topic = None
    

    services_dict = {
        'detector_info_query': {
            'connected': False,
            'msg': AiDetectorInfoQuery,
            'req': AiDetectorInfoQueryRequest(),
            'psn': None,
            'service': None
        }
    }


    #######################
    ### IF Initialization
    log_name = "ConnectAiDetectorIF"
    def __init__(self, det_namespace, auto_reg_img = False, auto_dereg_img = False, timeout = 500):
        #################################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name + ": " + pc_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################    
        # Initialize Class Variables
        self.det_auto_reg = auto_reg_img
        self.det_auto_dereg = auto_dereg_img

        #######################
        # Connect Detector
        success = True
        det_img_namespace = nepi_utils.clear_end_slash(det_img_namespace)
        self.det_namespace = det_namespace
        self.msg_if.pub_info("Connecting to Detector: " + det_img_namespace)


        ##################################################
        ### Node Connect Class

        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'namespace': '~'
        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'service_name': {
                'namespace': '~',
                'topic': 'empty_query',
                'srv': EmptySrv,
                'req': EmptySrvRequest(),
                'resp': EmptySrvResponse(),
                'callback': self.CALLBACK_FUNCTION
            }
        }


        self.add_img_topic_pub = rospy.Publisher(DET_PUB_NS + '/add_img_topic', String, queue_size=10)
        self.add_img_topics_pub = rospy.Publisher(DET_PUB_NS + '/add_img_topics', StringArray, queue_size=10)
        self.remove_img_topic_pub = rospy.Publisher(DET_PUB_NS + '/remove_img_topic', String, queue_size=10)
        self.remove_img_topics_pub = rospy.Publisher(DET_PUB_NS + '/remove_img_topics', StringArray, queue_size=10)
        self.set_img_tiling_pub = rospy.Publisher(DET_PUB_NS + '/set_img_tiling', Bool, queue_size=10)
        self.set_overlay_labels_pub = rospy.Publisher(DET_PUB_NS + '/set_overlay_labels', Bool, queue_size=10)
        self.set_overlay_clf_name_pub = rospy.Publisher(DET_PUB_NS + '/set_overlay_clf_name', Bool, queue_size=10)
        self.set_overlay_img_name_pub = rospy.Publisher(DET_PUB_NS + '/set_overlay_img_name', Bool, queue_size=10)          
        self.set_threshold_pub = rospy.Publisher(DET_PUB_NS + '/set_threshold', Float32, queue_size=10)
        self.set_max_rate_pub = rospy.Publisher(DET_PUB_NS + '/set_max_rate', Int32, queue_size=10)

        self.set_sleep_enable_pub = rospy.Publisher(DET_PUB_NS + '/set_sleep_enable', Bool, queue_size=10) 
        self.set_sleep_suspend_sec_pub = rospy.Publisher(DET_PUB_NS + '/set_sleep_suspend_sec', Int32, queue_size=10)
        self.set_sleep_run_sec_pub = rospy.Publisher(DET_PUB_NS + '/set_sleep_run_sec', Int32, queue_size=10)

        self.reset_factory_pub = rospy.Publisher(DET_PUB_NS + '/reset_factory', Empty, queue_size=10) # start local callback
        self.save_config_pub = rospy.Publisher(DET_PUB_NS + '/save_config', Empty, queue_size=10) # start local callback
        self.reset_config_pub = rospy.Publisher(DET_PUB_NS + '/reset_config', Empty, queue_size=10) # start local callback
        self.enable_pub = rospy.Publisher(DET_PUB_NS + '/enable', Bool, queue_size=10) # start local callback


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'pub_name': {
                'namespace': '~',
                'topic': 'set_empty',
                'msg': EmptyMsg,
                'qsize': 1,
                'latch': False
            }
        }


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'sub_name': {
                'namespace': '~',
                'topic': 'set_empty',
                'msg': EmptyMsg,
                'qsize': 1,
                'callback': self.SUB_CALLBACK,
                'callback_args': ()
            }
        }


        # Create Node Class ####################
        
        self.NODE_IF = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        
        
        DET_PUB_NS = det_namespace




        #######################
        #Setup Detector Subscribers
        DET_SUB_NS = det_namespace
        # Setup status msg subscriber for base detection det_img_namespace
        status_topic = self.det_namespace + "/status"
        self.det_status_sub = rospy.Subscriber(status_topic, AiDetectorStatus, self.detectorStatusCb, queue_size=10)


        #######################
        # Wait for Status Message
        self.msg_if.pub_info("Waiting for status message on topic: " + status_topic)
        timer = 0
        time_start = nepi_ros.get_time()
        while self.det_status_msg is None and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.2)
            timer = nepi_ros.get_time() - time_start
        if self.det_status_msg is None:
            self.msg_if.pub_warn("Status msg topic subscribe timed out " + str(status_topic))
            success = False
        else:
            self.msg_if.pub_warn("Got status msg " + str(self.det_status_msg))


        ##################################
        ### Set up Services
        for service_name in self.services_dict.keys():
            service = None
            service_dict = self.services_dict[service_name]
            if service_dict['psn'] is None: 
                service_namespace = os.path.join(self.base_namespace, service_name)
            else:
                service_namespace = os.path.join(self.det_namespace,service_dict['psn'], service_name)
            self.msg_if.pub_info("Waiting for " + service_name + " on det_namespace " + service_namespace)
            ret = nepi_ros.wait_for_service(service_namespace, timeout = timeout )
            if ret == "":
                self.msg_if.pub_warn("Wait for service: " + service_name + " timed out") 
            else:
                self.msg_if.pub_info("Creating service call for: " + service_name)
                #self.msg_if.pub_warn("Creating service with det_namespace: " + service_namespace)
                #self.msg_if.pub_warn("Creating service with msg: " + str(service_dict['msg']))
                try:
                    service = nepi_ros.create_service(service_namespace, service_dict['msg'])
                    time.sleep(1)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get service connection: " + service_name + " " + str(e))  
                if service is not None:
                    self.services_dict[service_name]['service'] = service
                    self.services_dict[service_name]['connected'] = True

        # Initialize info_dict
        info_dict = self.get_info_dict()

        ####################
        # Wrap Up
        if success == False:
            self.msg_if.pub_warn("Detector Connection Failed: " + det_namespace)
            self.unregister_detector()
        else:
            self.det_connected = True
            self.msg_if.pub_info("Detector Connected: " + det_namespace)

        if success == True and auto_reg_img == True:
            self.msg_if.pub_info("Starting auto reg check service")
            nepi_ros.start_timer_process(nepi_ros.ros_duration(.1), self.autoRegImgCb, oneshot = True)

        #################################
        self.ready = True

        self.msg_if.pub_info("IF Initialization Complete")
        #################################



    #######################
    # Class Public Methods
    #######################



    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready  


    #######################
    # Base Detector Functions

    def check_det_connection(self):
        return self.det_connected

    def wait_for_det_connection(self, timout = float('inf') ):
        success = False
        self.msg_if.pub_info("Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.det_connected == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.det_connected == False:
            self.msg_if.pub_info("Failed to Connect")
        else:
            self.msg_if.pub_info("Connected")
        return self.det_connected


    def get_det_namespace(self):
        return self.det_namespace

    def get_det_classes(self):
        return self.det_classes

    def get_det_classes_colors(self):
        return self.det_classes_colors

    def get_det_img_size(self):
        img_size = [self.det_img_height, self.det_img_width]
        return img_size

    def get_det_img_namespaces(self):
        return list(self.det_active_img_dict.keys())

    def get_det_img_dict(self):
        return self.det_active_img_dict

    def get_info_dict(self):
        service_name =  'detector_info_query'
        srv_dict = self.services_dict[service_name]

        info_dict = None

        if srv_dict['service'] is None:
            nepi_msg.publishMsgWarn(self,"Sevice is not connected: " + service_name)
        else:
            # Create service request
            request = None
            try:
                request = srv_dict['req']
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to create service request: " + service_name + " " + str(e))
                
            # Call service
            response = None
            if request is not None:
                response = nepi_ros.call_service(srv_dict['service'],  request)

            # Process Response
            if response is None:
                nepi_msg.publishMsgWarn(self,"Failed to get response for service: " + service_name)
            else:
                #nepi_msg.publishMsgInfo(self,"Got status response" + str(response) + " for service: " + service_name)
                det_info_msg = response.detector_info
                self.det_img_width = det_info_msg.det_img_width
                self.det_img_height = det_info_msg.det_img_height
                self.det_classes = det_info_msg.det_classes
                self.det_classes_colors = nepi_ais.get_classes_colors_list(det_info_msg.det_classes)
                info_dict = nepi_ros.convert_msg2dict(det_info_msg)
                self.msg_if.pub_info("Got detector info dict" + str(info_dict))
        return info_dict


    def get_status_dict(self):
        status_dict = None
        if self.det_connected != False: 
            if self.det_status_msg is not None:
                status_dict = nepi_ros.convert_msg2dict(self.det_status_msg)
            else:
                self.msg_if.pub_warn("Status Dict not Available")
        else:
            self.msg_if.pub_warn("Detector Not connected")
        return status_dict



    def unregister_detector(self):
        success = False
        # Unsubsribe from img subs
        for img_ns in self.det_img_sub_dict.keys():
            sub_dict = self.det_img_sub_dict[img_ns]
            for sub_name in sub_dict.keys():
                sub = sub_dict[sub_name]
                try:
                    sub.unregister()
                except:
                    pass
        time.sleep(1)
        self.det_img_sub_dict = dict()

        # Unregister det status subs
        self.det_connected = False
        try:
            self.det_status_sub.unregister()
        except:
            pass
        time.sleep(1)
        self.det_status_sub = None
        self.det_active_img_dict = dict()
        self.det_info_dict = None
        self.det_status_msg = None

        success = detection_trigger_all_pub
        return success

    def get_det_auto_reg(self):
        return self.det_auto_reg

    def set_det_auto_reg(self,value):
        self.det_auto_reg = value

    def get_det_auto_dereg(self):
        return self.det_auto_dereg

    def set_det_auto_dereg(self,value):
        self.det_auto_dereg = value



    #######################
    # Detector Img Functions

    def check_det_img_connection(self,det_img_ns):
        connected = None
        if det_img_ns in self.det_img_data_dict.keys():
            connected = self.det_img_data_dict['connected']
        else:
            self.msg_if.pub_info("Requested img detector not registered: " + det_img_ns)
        return connected


    def wait_for_det_img_connection(self, det_img_ns, timout = float('inf') ):
        success = False
        if det_img_ns in self.det_img_data_dict.keys():
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            connected = False
            while connected == False and timer < timeout and not nepi_ros.is_shutdown():
                check = self.det_img_data_dict['connected']
                if check is None:
                    connected = False
                else:
                    conected = check
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        else:
            self.msg_if.pub_info("Requested img detector not registered: " + det_img_ns)
        return connected


    def get_det_img_data_dict(self):
        return self.det_img_data_dict


    def get_det_img_state(self,det_img_ns):
        state = None
        if det_img_ns in self.det_img_data_dict.keys():
            state = self.det_img_data_dict['active']
        else:
            self.msg_if.pub_info("Requested img detector not registered: " + det_img_ns)
        return state

    def get_det_src_img_topic(self,det_img_ns):
        namespace = None
        if det_img_ns in self.det_img_data_dict.keys():
            namespace = self.det_img_data_dict['src_img_topic']
        else:
            self.msg_if.pub_info("Requested img detector not registered: " + det_img_ns)
        return namespace

    def get_det_pub_img_topic(self,det_img_ns):
        namespace = None
        if det_img_ns in self.det_img_data_dict.keys():
            namespace = self.det_img_data_dict['pub_img_topic']
        else:
            self.msg_if.pub_info("Requested img detector not registered: " + det_img_ns)
        return namespace

    def get_clear_found_obj_count(self,det_img_ns):
        count = None
        if det_img_ns in self.det_img_data_dict.keys():
            count = self.det_img_data_dict['found_object_count']
            self.det_img_data_dict['found_object_count'] = None
        else:
            self.msg_if.pub_info("Requested img detector not registered: " + det_img_ns)
        return count

    def get_clear_boxes_dict(self,det_img_ns):
        boxes_dict = None
        if det_img_ns in self.det_img_data_dict.keys():
            boxes_dict = self.det_img_data_dict['boxes_dict']
            self.det_img_data_dict['boxes_dict'] = None
        else:
            self.msg_if.pub_info("Requested img detector not registered: " + det_img_ns)
        return boxes_dict

    def get_first_det_time(self,det_img_ns):
        first_det_time = None
        if det_img_ns in self.det_img_data_dict.keys():
            first_det_time = self.det_img_data_dict['first_det_time']
        else:
            self.msg_if.pub_info("Requested img detector not registered: " + det_img_ns)
        return first_det_time
        

    def get_last_det_time(self,det_img_ns):
        last_det_time = None
        if det_img_ns in self.det_img_data_dict.keys():
            last_det_time = self.det_img_data_dict['last_det_time']
        else:
            self.msg_if.pub_info("Requested img detector not registered: " + det_img_ns)
        return last_det_time


    def get_source_img_topic(self,det_img_ns):
        src_img_topic = None
        if det_img_ns in self.det_img_data_dict.keys():
            src_img_topic = self.det_img_data_dict['src_img_topic']
        else:
            self.msg_if.pub_info("Requested img detector not registered: " + det_img_ns)
        return src_img_topic


    def register_detector_img(self, det_img_namespace, timeout = 20):
        #######################
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ":" + det_img_namespace + ":" + det_img_namespace + ": Initialization Detection Img Registration for namespace")
        success = True

        #### Check ns
        #####self.det_img_dict[det_img_namespace]

        #######################
        # Check if currently connected and unregister if so
        if det_img_namespace != "None" and det_img_namespace != "None":
            if det_img_namespace == self.det_img_namespace:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ":" + det_img_namespace + ": Specified Detector already connected, Exiting registration process")
                return True
            elif self.sub_dict['status_sub'] is not None:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ":" + det_img_namespace + ": Different Detector already connected, unregistering current detector first")
                unreged = self.unregister_detector()
                if unreged == False:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ":" + det_img_namespace + ": Failed to unregister current detector: " + str(self.det_img_namespace))

        #######################
        # Start Initialize 
        nepi_msg.publishMsgWarn(self,":" + self.log_name + ":" + det_img_namespace + ": Initializing detector with det_img_namespace: " + str(det_img_namespace))
        self.det_img_namespace = det_img_namespace


        #######################
        #Setup Detector Subscribers
        DET_SUB_NS = det_img_namespace
  
        found_obj_sub = rospy.Subscriber(DET_SUB_NS + "/found_object", ObjectCount, self.foundObjectCb, queue_size = 1, callback_args=(det_img_namespace))
        self.sub_dict['found_obj_sub'] = found_obj_sub

        bounding_boxes_sub = rospy.Subscriber(DET_SUB_NS + "/bounding_boxes", BoundingBoxes, self.objectDetectedCb, queue_size = 1, callback_args=(det_img_namespace))
        self.sub_dict['bounding_boxes_sub'] = bounding_boxes_sub



        ####################
        # Wrap Up
        if success == False:
            self.unregister_detector()
        else:
            self.det_connected = True
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ":" + det_img_namespace + ": Detector Registration Complete")
        return success

    def unregister_detector_img(self, det_img_namespace):
        success = True
        # Unsubsribe from pubs and subs
        for pub_sub_name in self.sub_dict.keys():
            pub_sub = self.sub_dict[pub_sub_name]
            if pub_sub is not None:
                try:
                    pub_sub.unregister()
                except Exception as e:
                    success = False
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ":" + det_img_namespace + ": Failed to unregister: " + pub_sub_name + " " + str(e))
        time.sleep(1)
        for pub_sub_name in self.sub_dict.keys():
            self.sub_dict[pub_sub_name] = None
            
        # Reset some variables
        self.det_img_namespace = None
        self.det_namespace = None
        self.det_connected = False
        self.active = False

        self.status_msg = None
        
        self.found_obj_msg = None
        self.bounding_boxes_msg = None

        return success







    #######################
    # Class Private Methods
    #######################




    def detectorStatusCb(self, status_msg):
        self.status_msg = status_msg
        # Create some useful class objects from status message
        namespaces = status_msg.image_detector_namespaces
        topics = status_msg.image_source_topics
        states = status_msg.image_detector_states
        cons_dict = dict()
        for i, namespace in enumerate(namespaces):
            con_dict = dict()
            con_dict['namespace'] = namespace
            con_dict['topic'] = topics[i]
            con_dict['state'] = states[i]
            cons_dict['namespace'] = con_dict
        self.det_active_img_dict = cons_dict
            

        

    ### Monitor Output of AI detector to clear detection status
    def foundObjectCb(self,found_obj_msg, args):
        det_img_namespace = args
        self.det_img_dict[det_img_namespace]['found_obj_msg'] = found_obj_msg
        self.det_img_dict[det_img_namespace]['found_obj_count'] = found_obj_msg.count

    def objectDetectedCb(self,bounding_boxes_msg, args):
        det_img_namespace = args
        self.det_img_dict[det_img_namespace]['object_detect_msg'] = bounding_boxes_msg
        binfo = nepi_ais.get_boxes_info_from_msg(bounding_boxes_msg)
        blist = nepi_ais.get_boxes_list_from_msg(bounding_boxes_msg)
        self.det_img_dict[det_img_namespace]['boxes_dict'] = {
                            'info': binfo,
                            'boxes': blist
                            }

        time = nepi_ros.get_time()
        if self.det_img_dict[det_img_namespace]['first_det_time'] is None:
            self.det_img_dict[det_img_namespace]['first_det_time'] = time
        self.det_img_dict[det_img_namespace]['last_det_time'] = time

    def autoRegImgCb(self,timer):
        pass # future work