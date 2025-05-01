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
import sys
import copy
import time
import numpy as np
import math
import cv2
import threading


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_ais
from nepi_sdk import nepi_img

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image

from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes
from nepi_ros_interfaces.msg import AiDetectorInfo, AiDetectorStatus
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryRequest, AiDetectorInfoQueryResponse



from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodePublishersIF, NodeSubscribersIF, NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.data_if import ImageIF



EXAMPLE_BOXES_INFO_DICT_ENTRY = {
    'model_name': 'test_model',
    'image_header': Header(),
    'image_topic': 'my/test_topic',
    'src_height': 600,
    'src_width': 1000,
    'prc_height': 300,
    'prc_width': 500,
}


EXAMPLE_BOX_DICT_ENTRY = {
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


NONE_IMG_DICT = {       
    'cv2_img': None,
    'width': 0,
    'height': 0,
    'timestamp': nepi_ros.get_time(),
    'ros_img_topic': 'None',
    'ros_img_header': Header(),
    'ros_img_stamp': Header().stamp
}


BLANK_SIZE_DICT = { 'h': 350, 'w': 700, 'c': 3}

BLANK_CV2_IMAGE = nepi_img.create_blank_image((BLANK_SIZE_DICT['h'],BLANK_SIZE_DICT['w'],BLANK_SIZE_DICT['c']))

BLANK_IMG_DICT = {       
    'cv2_img': BLANK_CV2_IMAGE,
    'width': 0,
    'height': 0,
    'timestamp': nepi_ros.get_time(),
    'ros_img_topic': 'None',
    'ros_img_header': Header(),
    'ros_img_stamp': Header().stamp
}


class AiDetectorImgPub:


    data_products = ['detection_image']

    det_sub_names = ['found_object', 'bounding_boxes']

    self_managed = True
    model_name = "None"

    img_ifs_dict = dict()
    img_ifs_lock = threading.Lock()
    imgs_info_dict = dict()
    imgs_img_proc_dict = dict()

    save_cfg_if = None

    state = 'Loading'

    cur_img_topic = "None"
    get_img_topic = "None"
    last_get_image_time = 0
    got_img_topic = "None"

    img_dict = None
    img_dict_lock = threading.Lock()
    clear_img_time = 1.0

    first_detect_complete = False
    clear_det_time = 1.0

    detection_state = False
    
    DEFAULT_NODE_NAME = "detector_img_pub" # Can be overwitten by luanch command
    def __init__(self):
        ####  IF INIT SETUP ####
        nepi_ros.init_node(name = self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################  
        # Init Class Variables 

        backup_det_namespace = self.node_namespace.replace("_img_pub","")
        self.det_namespace = nepi_ros.get_param(self.node_namespace + "/det_namespace",backup_det_namespace)

        backup_all_namespace = os.path.join(self.base_namespace,"ai","all_detectors")
        self.all_namespace = nepi_ros.get_param(self.node_namespace + "/all_namespace",backup_all_namespace)

        self.classes_list = nepi_ros.get_param(self.node_namespace + "/classes_list",[])
        self.classes_color_list = nepi_ais.get_classes_colors_list(self.classes_list)
        #self.msg_if.pub_warn("Detector provided classes color list: " + str(self.classes_color_list))

        self.status_msg = AiDetectorStatus()
        self.model_name = os.path.basename(self.det_namespace)
        self.enabled = False
        self.sleep_state = False
        self.max_rate = 1.0

        self.overlay_labels = True
        self.overlay_clf_name = False
        self.overlay_img_name = False

        self.img_source_topics = []
        self.img_det_namespaces = []
        self.img_det_states = []


        self.img_image_if = ImageIF(namespace = self.det_namespace , topic = 'detection_image')
        self.img_image_if_all = ImageIF(namespace = self.all_namespace , topic = 'detection_image')
        ##############################  
        # Create NodeClassIF Class  

        # Configs Dict ########################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.det_namespace,
        }


        # Params Config Dict ####################
        self.PARAMS_DICT = None


        # Services Config Dict ####################
        self.SRVS_DICT = None


        self.PUBS_DICT = None


        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'found_object': {
                'msg': ObjectCount,
                'namespace': self.det_namespace,
                'topic': 'found_object',
                'qsize': 10,
                'callback': self.foundObjectCb, 
                'callback_args': ()
            },
            'bounding_boxes': {
                'msg': BoundingBoxes,
                'namespace': self.det_namespace,
                'topic': 'bounding_boxes',
                'qsize': 10,
                'callback': self.objectDetectedCb, 
                'callback_args': ()
            },
            'status_sub': {
                'msg': AiDetectorStatus,
                'namespace': self.det_namespace,
                'topic': 'status',
                'qsize': 10,
                'callback': self.statusCb, 
                'callback_args': ()
            }
        }


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        self.node_if.wait_for_ready()

        ###############################
        # Create System IFs

       
        # Setup Save Data IF
        factory_data_rates= {}
        for d in self.data_products:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if 'detection_image' in self.data_products:
            factory_data_rates['detection_image'] = [1.0, 0.0, 100.0] 

        self.save_data_if = SaveDataIF(data_products = self.data_products, factory_rate_dict = factory_data_rates, namespace = self.det_namespace)
        
        time.sleep(1)


        ##########################
        # Complete Initialization

        # Start Timer Processes
        nepi_ros.start_timer_process((0.1), self.updaterCb, oneshot = True)

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################


    def initCb(self,do_updates = False):
        self.msg_if.pub_info(" Setting init values to param values")
        if do_updates == True:
            self.resetCb(do_updates)


    def resetCb(self):
        self.publish_status()

    def factoryResetCb(self):
        self.last_det_dict_list = []
        self.publish_status()


    def updaterCb(self,timer):
        # Clear boxes if stall
        current_time = nepi_utils.get_time()
        for img_topic in self.imgs_info_dict.keys():
            last_time = self.imgs_info_dict[img_topic]['last_det_time']
            check_time = current_time - last_time
            if check_time > self.clear_det_time or self.enabled == False or self.sleep_state == True:
                self.imgs_info_dict[img_topic]['det_dict_list'] = None

        #self.msg_if.pub_warn("Updating with image topic: " +  self.img_topic)
        img_topics = self.img_source_topics
        self.img_ifs_lock.acquire()
        img_ifs_keys = self.img_ifs_dict.keys()
        self.img_ifs_lock.release()
        # Update Image subscribers
        for i,img_topic in enumerate(img_topics):
            img_topic = nepi_ros.find_topic(img_topic)
            if img_topic != '':
                img_topics[i] = img_topic
                if img_topic not in img_ifs_keys:
                    success = self.subscribeImgTopic(img_topic)    
          
        purge_list = []
        for img_topic in img_ifs_keys:
            if img_topic not in img_topics:
                purge_list.append(img_topic)
        #self.msg_if.pub_warn('Purging image topics: ' + str(purge_list))
        for topic in purge_list:
            self.msg_if.pub_warn('Will unsubscribe topics: ' + topic)
            success = self.unsubscribeImgTopic(topic)




        '''
        self.pub_img_if.publish_cv2_msg_img(self.not_connected_msg)
        self.pub_img_if.publish_cv2_msg_img(self.not_enabled_msg)
        self.pub_img_if.publish_cv2_msg_img(self.waiting_img_msg)
        self.pub_img_det_if.publish_cv2_msg_img(self.waiting_img_msg)
        '''



        nepi_ros.start_timer_process((.5), self.updaterCb, oneshot = True)


    def subscribeImgTopic(self,img_topic):
        if img_topic == "None" or img_topic == "":
            return False
        else:

            img_sub_dict = {
                        'namespace': img_topic,
                        'msg': Image,
                        'topic': '',
                        'qsize': 1,
                        'callback': self.imageCb,
                        'callback_args': (img_topic)
                    }

            imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
            # Check if exists
            if img_topic in imgs_info_dict.keys():
                if self.img_ifs_dict[img_topic]:
                    self.msg_if.pub_info('Subsribing to image topic: ' + img_topic)  
                    # Try and Reregister img_sub
                    self.img_ifs_lock.acquire()
                    self.img_ifs_dict[img_topic]['subs_if'].register_sub('img_sub',img_sub_dict)
                    self.img_ifs_lock.release()
                    self.msg_if.pub_warn('Registered : ' + img_topic +  ' ' + str(self.img_ifs_dict[img_topic]))
                    # Set back to active
                    self.imgs_info_dict[img_topic]['active'] = True
            else:
                    
                # Create register new image topic
                self.msg_if.pub_info('Subsribing to image topic: ' + img_topic)
                img_name = img_topic.replace(self.base_namespace,"")
                pub_namespace = os.path.join(self.det_namespace,img_name)

                ####################
                # Create img info dict
                self.imgs_info_dict[img_topic] = dict()  
                self.imgs_info_dict[img_topic]['namespace'] = pub_namespace
                self.imgs_info_dict[img_topic]['connected'] = False 
                self.imgs_info_dict[img_topic]['get_latency_time'] = 0
                self.imgs_info_dict[img_topic]['pub_latency_time'] = 0
                self.imgs_info_dict[img_topic]['process_time'] = 0 
                self.imgs_info_dict[img_topic]['last_img_time'] = 0
                self.imgs_info_dict[img_topic]['last_det_time'] = 0
                self.imgs_info_dict[img_topic]['det_dict_list'] = []   

                self.imgs_info_dict[img_topic]['active'] = True
                self.imgs_info_dict[img_topic]['img_publishing'] = False

  
                img_image_if = ImageIF(namespace = pub_namespace , topic = 'detection_image')

                ####################
                # Pubs Config Dict 
                SUBS_DICT = {
                    'img_sub': img_sub_dict
                }

                ####################
                # Subs Config Dict 
                img_subs_if = NodeSubscribersIF(
                                subs_dict = SUBS_DICT,
                                log_class_name = False
                )


                time.sleep(1)

                ####################
                # Add Img Subs and Pubs IFs to Img IFs Dict
                self.img_ifs_lock.acquire()
                self.img_ifs_dict[img_topic] = {
                                                'img_if': img_image_if,
                                                'subs_if': img_subs_if
                                                }   

                self.img_ifs_lock.release()
                self.msg_if.pub_warn('Registered : ' + img_topic)


                ####################
                # Publish blank msg img to prime topics
                #img_image_if
 
    
            return True

    def publishImgData(self, img_topic, cv2_img, encoding = "bgr8", timestamp = None, frame_id = 'nepi_base', add_overlay_list = []):
        self.img_image_if.publish_pub(cv2_img, encoding = encoding, timestamp = timestamp, frame_id = frame_id, add_overlay_list = add_overlay_list)
        self.img_image_if_all.publish_pub(cv2_img, encoding = encoding, timestamp = timestamp, frame_id = frame_id, add_overlay_list = add_overlay_list)
        if img_topic in self.img_ifs_dict.keys():
            self.img_ifs_lock.acquire()
            self.img_ifs_dict[img_topic]['img_if'].publish_cv2_image(cv2_img, encoding = encoding, timestamp = timestamp, frame_id = frame_id, add_overlay_list = add_overlay_list)
            self.img_ifs_lock.release()
                 

    def unsubscribeImgTopic(self,img_topic):
        if img_topic in self.img_ifs_dict.keys():
            self.msg_if.pub_warn('Unregistering image topic: ' + img_topic)
            self.img_ifs_lock.acquire()
            self.img_ifs_dict[img_topic]['subs_if'].unregister_sub('img_sub')
            self.img_ifs_lock.release()

        #Leave img pub running in case it is switched back on
    
        # Clear info dict
        if img_topic in self.imgs_info_dict.keys():
            self.imgs_info_dict[img_topic]['active'] = False
            self.imgs_info_dict[img_topic]['connected'] = False 
            self.imgs_info_dict[img_topic]['get_latency_time'] = 0
            self.imgs_info_dict[img_topic]['pub_latency_time'] = 0
            self.imgs_info_dict[img_topic]['process_time'] = 0 
            self.imgs_info_dict[img_topic]['last_img_time'] = 0 
            self.imgs_info_dict[img_topic]['last_det_time'] = 0 
            self.imgs_info_dict[img_topic]['det_dict_list'] = []

        return True




    def imageCb(self, image_msg, args):     
        start_time = nepi_ros.get_time()   
        img_topic = args
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict) 

        last_img_time = self.imgs_info_dict[img_topic]['last_img_time']
        self.imgs_info_dict[img_topic]['last_img_time'] = nepi_utils.get_time()

        if img_topic in imgs_info_dict.keys():
            enabled = self.enabled
            sleep_state = self.sleep_state
            active = imgs_info_dict[img_topic]['active']
            if active == True and enabled == True and sleep_state == False:
                # Process ros image message
                imgs_info_dict[img_topic]['connected'] = True


                # Check if time to publish
                delay_time = float(1) / self.max_rate 
                current_time = nepi_utils.get_time()
                timer = round((current_time - last_img_time), 3)
                #self.msg_if.pub_warn("Delay and Timer: " + str(delay_time) + " " + str(timer))
                if timer > delay_time: 
                    ros_timestamp = image_msg.header.stamp
                    ros_frame_id = image_msg.header.frame_id

                    current_time = nepi_ros.ros_time_now()
                    latency = (current_time.to_sec() - ros_timestamp.to_sec())
                    imgs_info_dict[img_topic]['get_latency_time'] = latency
                    #self.msg_if.pub_info("Detect Pub Latency: {:.2f}".format(latency))

                    # Request new img
                    det_dict_list = self.imgs_info_dict[img_topic]['det_dict_list']
                    if det_dict_list == None:
                        det_dict_list = []
                    cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

                    success = self.processDetImage(img_topic, cv2_img, det_dict_list, timestamp = ros_timestamp, frame_id = ros_frame_id)

                    current_time = nepi_ros.ros_time_now()
                    latency = (current_time.to_sec() - ros_timestamp.to_sec())
                    imgs_info_dict[img_topic]['pub_latency_time'] = latency                              



    def processDetImage(self,image_topic, cv2_img, detect_dict_list, timestamp = None, frame_id = 'nepi_base'):
        if 'cv2_img' not in img_dict.keys():
            return False
        if img_dict['cv2_img'] is None:
            return False
        
        # Post process image with overlays
        if detect_dict_list is not None:
            # Publish image first for consumers
            #self.msg_if.pub_warn("Starting detect image: " + str(cv2_img.shape))
            cv2_img = self.apply_detection_overlay(image_topic, detect_dict_list, cv2_img)
            #self.msg_if.pub_warn("Return detect image: " + str(cv2_img.shape)

            add_overlay_list = []
            ## Overlay Detector Name
            overlay_clf_name = self.overlay_clf_name
            if overlay_clf_name:
                add_overlay_list.append(self.det_status_dict['name'])
            if overlay_img_name:
                add_overlay_list.append(nepi_img.getImgShortName(image_topic))
            publishImgData(self, img_topic, cv2_img, timestamp = timestamp, frame_id = frame_id, add_overlay_list = add_overlay_list)
        
            # Save Image Data if needed
            data_product = 'detection_image'
            if image_topic is not None:
                image_text = image_topic.replace(self.base_namespace,"")
                image_text = image_text.replace('/idx',"")
                image_text = image_text.replace('/','_')
            else:
                imagee_text = ""
            nepi_save.save_ros_img2file(self,data_product,det_img_msg,ros_timestamp, add_text = image_text)
        return True


    def apply_detection_overlay(self,image_topic, detect_dict_list, cv2_img):
        cv2_det_img = copy.deepcopy(cv2_img)
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1] 
        img_height = cv2_shape[0] 



        if self.det_status_dict is not None:
            for detect_dict in detect_dict_list:
                img_size = cv2_img.shape[:2]

                # Overlay text data on OpenCV image
                font = cv2.FONT_HERSHEY_DUPLEX
                fontScale, font_thickness  = nepi_img.optimal_font_dims(cv2_det_img,font_scale = 1.5e-3, thickness_scale = 1.5e-3)
                fontColor = (255, 255, 255)
                fontColorBk = (0,0,0)
                lineType = cv2.LINE_AA


                ###### Apply Image Overlays and Publish Image ROS Message
                # Overlay adjusted detection boxes on image 
                class_name = detect_dict['name']
                xmin = detect_dict['xmin']
                ymin = detect_dict['ymin']
                xmax = detect_dict['xmax']
                ymax = detect_dict['ymax']

                if xmin <= 0:
                    xmin = 5
                if ymin <= 0:
                    ymin = 5
                if xmax >= img_size[1]:
                    xmax = img_size[1] - 5
                if ymax >= img_size[0]:
                    ymax = img_size[0] - 5


                bot_left_box = (xmin, ymin)
                top_right_box = (xmax, ymax)

                class_color = (255,0,0)
                if class_name in self.classes_list:
                    class_ind = self.classes_list.index(class_name)
                    #self.msg_if.pub_warn("Got Class Index: " + str(class_ind))
                    if class_ind < len(self.classes_color_list):
                        class_color = tuple(self.classes_color_list[class_ind])
                        #self.msg_if.pub_warn("Got Class Color: " + str(class_color))
                class_color =  [int(c) for c in class_color]

                line_thickness = font_thickness


                success = False
                try:
                    cv2.rectangle(cv2_det_img, bot_left_box, top_right_box, class_color, thickness=line_thickness)
                    success = True
                except Exception as e:
                    self.msg_if.pub_warn("Failed to create bounding box rectangle: " + str(e))

                # Overlay text data on OpenCV image
                if success == True:

                    ## Overlay Labels
                    overlay_labels =  self.overlay_labels
                    if overlay_labels:
                        text2overlay=class_name
                        text_size = cv2.getTextSize(text2overlay, 
                            font, 
                            fontScale,
                            font_thickness)
                        #self.msg_if.pub_warn("Text Size: " + str(text_size))
                        line_height = text_size[0][1]
                        line_width = text_size[0][0]
                        x_padding = int(line_height*0.4)
                        y_padding = int(line_height*0.4)
                        bot_left_text = (xmin + (line_thickness * 2) + x_padding , ymin + line_height + (line_thickness * 2) + y_padding)
                        # Create Text Background Box

                        bot_left_box =  (bot_left_text[0] - x_padding , bot_left_text[1] + y_padding)
                        top_right_box = (bot_left_text[0] + line_width + x_padding, bot_left_text[1] - line_height - y_padding )
                        box_color = [0,0,0]

                        try:
                            cv2.rectangle(cv2_det_img, bot_left_box, top_right_box, box_color , -1)
                            cv2.putText(cv2_det_img,text2overlay, 
                                bot_left_text, 
                                font, 
                                fontScale,
                                fontColor,
                                font_thickness,
                                lineType)
                        except Exception as e:
                            self.msg_if.pub_warn("Failed to apply overlay text: " + str(e))

                        # Start name overlays    
                        x_start = int(img_width * 0.05)
                        y_start = int(img_height * 0.05)


        return cv2_det_img



    ### Monitor Output of AI detector to clear detection status
    def foundObjectCb(self,msg):
        stamp = msg.header.stamp
        img_stamp = msg.image_header.stamp
        img_topic = msg.image_topic
        det_count = msg.count
        self.imgs_info_dict[img_topic]['stamp'] = stamp
        self.imgs_info_dict[img_topic]['img_stamp'] = img_stamp
        self.imgs_info_dict[img_topic]['det_count'] = det_count

        current_time = nepi_utils.get_time()
        self.imgs_info_dict[img_topic]['last_det_time'] = current_time

        if det_count == 0:
            self.imgs_info_dict[img_topic]['det_dict_list'] = None

    def objectDetectedCb(self,msg):
        img_topic = msg.image_topic
        
        self.imgs_info_dict[img_topic]['object_detect_msg'] = msg
        binfo = nepi_ais.get_boxes_info_from_msg(msg)
        blist = nepi_ais.get_boxes_list_from_msg(msg)
        self.imgs_info_dict[img_topic]['det_dict_list'] = blist

    def statusCb(self,msg):
        self.status_msg = msg

        self.model_name = self.status_msg.name
        self.enabled = self.status_msg.enabled
        self.sleep_state = self.status_msg.sleep_state
        self.max_rate = self.status_msg.max_img_rate_hz

        self.overlay_labels = self.status_msg.overlay_labels
        self.overlay_clf_name = self.status_msg.overlay_clf_name
        self.overlay_img_name = self.status_msg.overlay_img_name

        img_topics = self.status_msg.image_source_topics
        img_states = self.status_msg.image_detector_states
        img_source_topics = []
        for i, img_topic in enumerate(img_topics):
            if img_states[i] == True:
                img_source_topics.append(img_topic)
        self.img_source_topics = img_source_topics
        

#########################################
# Main
#########################################
if __name__ == '__main__':
  AiDetectorImgPub()





