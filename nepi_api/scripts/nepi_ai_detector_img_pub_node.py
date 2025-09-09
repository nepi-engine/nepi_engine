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
import copy
import numpy as np
import math
import cv2
import threading


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_ais
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image

from nepi_interfaces.msg import NavPose
from nepi_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes
from nepi_interfaces.msg import AiDetectorInfo, AiDetectorStatus
#from nepi_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryRequest, AiDetectorInfoQueryResponse



from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.data_if import ImageIF





EXAMPLE_BOXES_INFO_DICT_ENTRY = {
    'model_name': 'test_model',
    'image_header': Header(),
    'image_topic': 'my/test_topic',
    'image_height': 600,
    'image_width': 1000,
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
    'timestamp': nepi_sdk.get_time(),
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
    'timestamp': nepi_sdk.get_time(),
    'ros_img_topic': 'None',
    'ros_img_header': Header(),
    'ros_img_stamp': Header().stamp
}


WATCHDOG_TIMEOUT=10

class AiDetectorImgPub:

    IMG_DATA_PRODUCT = 'detection_image'

    det_sub_names = ['found_object', 'bounding_boxes']

    self_managed = True
    model_name = "None"

    img_if = None
    img_if_all = None

    cv2_img = None
    cv2_img_lock = threading.Lock()

    img_det_dict = dict()
    img_det_lock = threading.Lock()


    imgs_info_dict = dict()
    imgs_info_lock = threading.Lock()
    imgs_img_proc_dict = dict()

    save_cfg_if = None

    state = 'Loading'

    cur_img_topic = "None"
    get_img_topic = "None"
    last_get_image_time = 0
    got_img_topic = "None"

    clear_img_time = 1.0

    first_detect_complete = False
    clear_det_time = 1.0

    detection_state = False

    classes_list = []
    classes_colors_list = []
    
    last_status_time=None
    DEFAULT_NODE_NAME = "detector_img_pub" # Can be overwitten by luanch command
    def __init__(self):
        ####  IF INIT SETUP ####
        nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################  
        # Init Class Variables 

            

        self.data_product = nepi_sdk.get_param(self.node_namespace + "/data_product",self.IMG_DATA_PRODUCT)
        self.data_products = [self.data_product]
        self.msg_if.pub_warn("Starting with Data Products: " + str(self.data_products))
        
        backup_det_namespace = self.node_namespace.replace("_img_pub","")
        self.det_namespace = nepi_sdk.get_param(self.node_namespace + "/det_namespace",backup_det_namespace)
        self.msg_if.pub_warn("Starting with Detector Namespace: " + str(self.det_namespace))

        self.all_namespace = nepi_sdk.get_param(self.node_namespace + "/all_namespace",'')
        self.msg_if.pub_warn("Starting with All Namespace: " + str(self.all_namespace))



        self.status_msg = AiDetectorStatus()
        self.model_name = os.path.basename(self.det_namespace)
        self.enabled = False
        self.state = "Unknown"
        self.max_rate = 1.0
        self.use_last_image = True

        self.overlay_labels = True
        self.overlay_clf_name = False
        self.overlay_img_name = False

        self.selected_img_topics = []
        self.img_det_namespaces = []
        self.img_det_states = []

        

        ##############################  
        # Create NodeClassIF Class  

        # Configs Dict ########################
        self.CONFIGS_DICT = None
        '''
                {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.det_namespace,
        }
        '''


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
                'topic': 'detector_status',
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
                        msg_if = self.msg_if
                                            )


        nepi_sdk.wait()
        

        # Create image publisher
        self.img_if = ImageIF(namespace = self.det_namespace ,
                        data_product_name = self.data_product,
                        data_source_description = 'image',
                        data_ref_description = 'image',
                        perspective = 'pov',
                        init_overlay_list = [],
                        get_navpose_function = None,
                        log_name = self.data_product,
                        log_name_list = [],
                        msg_if = self.msg_if
                        )

        ###############################
        # Create System IFs

       
        # Setup Save Data IF
        factory_data_rates= {}
        factory_data_rates[self.data_product] = [1.0, 0.0, 100] 

        self.save_data_if = SaveDataIF(data_products = self.data_products, factory_rate_dict = factory_data_rates, namespace = self.det_namespace,
                        msg_if = self.msg_if
                                            )

        
        time.sleep(1)


        ##########################
        # Complete Initialization

        # Start Timer Processes
        nepi_sdk.start_timer_process((0.1), self.updaterCb, oneshot = True)
        self.last_status_time=nepi_utils.get_time()
        nepi_sdk.on_shutdown(self.shutdownCb)
        nepi_sdk.start_timer_process((1), self.watchdogCb)
        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_sdk.spin()
        #########################################################


    def initCb(self,do_updates = False):
        self.msg_if.pub_info(" Setting init values to param values")
        if do_updates == True:
            pass


    def resetCb(self,do_updates = True):
        self.last_det_dict_list = []
        if do_updates == True:
            pass
        self.initCb()

    def factoryResetCb(self,do_updates = True):
        self.last_det_dict_list = []
        if do_updates == True:
            pass
        self.initCb()


    def getImgInfoDict(self):
        self.imgs_info_lock.acquire()
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        self.imgs_info_lock.release()
        return imgs_info_dict

    def getActiveImgTopics(self):
        self.imgs_info_lock.acquire()
        active_img_topics = list(self.imgs_info_dict.keys())
        self.imgs_info_lock.release()
        return active_img_topics


    def updaterCb(self,timer):
        # Clear boxes if stall
        imgs_info_dict = self.getImgInfoDict()
        selected_img_topics = copy.deepcopy(self.selected_img_topics)
        #self.msg_if.pub_warn("Updating with image topics: " +  str(selected_img_topics))
        #self.msg_if.pub_warn("Updating with active image topics: " +  str(active_img_topics))
        active_img_topics = self.getActiveImgTopics()
        current_time = nepi_utils.get_time()
        for img_topic in imgs_info_dict.keys():
            last_time = imgs_info_dict[img_topic]['last_det_time']
            check_time = current_time - last_time
            '''
            if check_time > self.clear_det_time or self.enabled == False or self.state != 'Running':
                try:
                    self.imgs_info_dict[img_topic]['det_dict_list'] = None
                except:
                    pass
            '''


        # Do Image Subs updating
        #self.msg_if.pub_warn("")
        #self.msg_if.pub_warn("Subscriber Check with image topics: " +  str(selected_img_topics))
        #self.msg_if.pub_warn("Subscriber Check  with active image topics: " +  str(active_img_topics))
        purge_list = []
        if self.max_rate == -1:
            purge_list = selected_img_topics
        else:
            # Update Image subscribers
            found_img_topics = []
            for img_topic in selected_img_topics:
                img_topic = nepi_sdk.find_topic(img_topic, exact = True)
                if img_topic != '':
                    found_img_topics.append(img_topic)
                    if img_topic not in active_img_topics:
                        self.msg_if.pub_warn('Will subscribe to image topic: ' + img_topic)
                        success = self.subscribeImgTopic(img_topic)
                       
            # Update Image Subs purge list      
            active_img_topics = self.getActiveImgTopics() 
            #self.msg_if.pub_warn('Purge Check with active topics: ' + str(active_img_topics))       
            #self.msg_if.pub_warn('Purge Check with found topics: ' + str(active_img_topics)) 
            for img_topic in active_img_topics:
                if img_topic not in found_img_topics:
                    purge_list.append(img_topic)

        # Do image sub purging if required
        #self.msg_if.pub_warn('Purging image topics: ' + str(purge_list))
        #self.msg_if.pub_warn("")
        if len(purge_list) > 0:
            self.msg_if.pub_warn('Purging image topics: ' + str(purge_list))
        for img_topic in purge_list:
            self.msg_if.pub_warn('Will unsubscribe from image topic: ' + img_topic)
            success = self.unsubscribeImgTopic(img_topic)
            nepi_sdk.sleep(1)

        '''
        self.pub_img_if.publish_msg_img("Detector not Enabled")

        self.pub_img_if.publish_msg_img("Detector Sleeping")

        self.pub_img_if.publish_msg_img("Waiting for Image")

        self.pub_img_if.publish_msg_img("Image not Connected")
        '''


        nepi_sdk.start_timer_process((.5), self.updaterCb, oneshot = True)

    def watchdogCb(self,timer):
        cur_time=nepi_utils.get_time()
        timer=cur_time-self.last_status_msg
        if timer > WATCHDOG_TIMEOUT:
            msg="Lost connection to parent node.  Shutting down"
            nepi_sdk.signal_shutdown(msg)


    def subscribeImgTopic(self,img_topic):
        success = False
        imgs_info_dict = self.getImgInfoDict()
        #self.msg_if.pub_warn('Subscribing with image dict keys: ' + str(imgs_info_dict.keys()))
        if img_topic == "None" or img_topic == "":
            pass
        if img_topic in imgs_info_dict.keys():
            success = True    
        else:
            # Create Publishers
            self.msg_if.pub_warn('Subscribing to image topic: ' + img_topic)
            img_sub = nepi_sdk.create_subscriber(img_topic,Image, self.imageCb, queue_size = 1, callback_args= (img_topic), log_name_list = [])
            img_name = img_topic.replace(self.base_namespace,"")
            self.msg_if.pub_warn('Creating namespace for image name: ' + img_name)

            nav_topic = nepi_sdk.create_namespace(img_topic,'navpose')
            self.msg_if.pub_warn('Subscribing to image topic navpose: ' + nav_topic)
            nav_sub = nepi_sdk.create_subscriber(nav_topic,NavPose, self.navposeCb, queue_size = 1, callback_args= (img_topic), log_name_list = [])

            pub_namespace = nepi_sdk.create_namespace(self.det_namespace,img_name)
            self.msg_if.pub_warn('Publishing imgage ' + img_name + ' on namespace: ' + pub_namespace)
            self.img_if.add_pub_namespace(pub_namespace)
           
            # Subscribe to new image topic
            self.img_det_lock.acquire()
            self.img_det_dict[img_topic] = {
                                            'img_sub': img_sub,
                                            'nav_sub': nav_sub
                                            }   
            self.img_det_lock.release()


            ####################
            # Create img info dict
            img_info_dict = dict()  
            img_info_dict['pub_namespace'] = pub_namespace
            img_info_dict['navpose_dict'] = nepi_nav.BLANK_NAVPOSE_DICT
            img_info_dict['active'] = True
            img_info_dict['connected'] = False 
            img_info_dict['publishing'] = False
            img_info_dict['get_latency_time'] = 0
            img_info_dict['pub_latency_time'] = 0
            img_info_dict['process_time'] = 0 
            img_info_dict['last_img_time'] = 0
            img_info_dict['last_det_time'] = 0
            img_info_dict['det_dict_list'] = []   

            self.imgs_info_lock.acquire()
            self.imgs_info_dict[img_topic] = img_info_dict
            #self.msg_if.pub_warn('Subscribed with image dict keys: ' + str(self.imgs_info_dict.keys()))
            self.imgs_info_lock.release()


            return True


    def unsubscribeImgTopic(self,img_topic):
        self.msg_if.pub_warn('Unsubscribing from image topic: ' + img_topic)

        imgs_info_dict = self.getImgInfoDict()
        # Remove img pubs
        pub_namespace = None
        try:
            pub_namespace = imgs_info_dict[img_topic]['pub_namespace']
        except:
            pass
        if pub_namespace is not None:
            self.img_if.remove_pub_namespace(pub_namespace)

        # Unsubscribe
        self.img_det_lock.acquire()
        if img_topic in self.img_det_dict.keys():
            self.img_det_dict[img_topic]['img_sub'].unregister()
            self.img_det_dict[img_topic]['nav_sub'].unregister()
        self.img_det_lock.release()

        # Purge Dict

        self.imgs_info_lock.acquire()
        if img_topic in self.imgs_info_dict.keys():
            del self.imgs_info_dict[img_topic]
        self.imgs_info_lock.release()
        nepi_sdk.sleep(1)
        
        

            
        return True

    def publishImgData(self, img_topic, cv2_img, encoding = "bgr8", timestamp = None, frame_3d = 'nepi_base', add_overlay_list = [], navpose_dict = None):
        if self.img_if is not None:
            if self.img_if.has_subscribers_check():
                add_pubs = [self.all_namespace]
                imgs_info_dict = self.getImgInfoDict()
                if img_topic in imgs_info_dict.keys():
                    pub_namespace = imgs_info_dict[img_topic]['pub_namespace']
                    add_pubs.append(pub_namespace)
                self.img_if.publish_cv2_img(cv2_img, 
                                        encoding = encoding, 
                                        timestamp = timestamp, 
                                        frame_3d = frame_3d, 
                                        add_overlay_list = add_overlay_list,
                                        add_pubs = add_pubs,
                                        navpose_dict = navpose_dict)


                 





    def imageCb(self, image_msg, args):     
        start_time = nepi_sdk.get_time()   
        img_topic = args

        imgs_info_dict = self.getImgInfoDict()
        if img_topic in imgs_info_dict.keys():
            navpose_dict = imgs_info_dict[img_topic]['navpose_dict']
        else:
            navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT

        sel_imgs = copy.deepcopy(self.selected_img_topics)        
        max_rate = copy.deepcopy(self.max_rate)
        if img_topic in imgs_info_dict.keys() and img_topic in sel_imgs and max_rate > .01:
            imgs_info_dict[img_topic]['connected'] = True
            if self.enabled == True and self.state == 'Running':
                # Process ros image message
                #self.msg_if.pub_warn("Processing image topic: " + str(img_topic))
                imgs_info_dict[img_topic]['publishing'] = True

                # Check if time to publish
                delay_time = float(1) / max_rate 
                last_img_time = imgs_info_dict[img_topic]['last_img_time']
                current_time = nepi_utils.get_time()
                timer = round((current_time - last_img_time), 3)
                #self.msg_if.pub_warn("Delay and Timer: " + str(delay_time) + " " + str(timer))
                if timer > delay_time: 
                    imgs_info_dict[img_topic]['last_img_time'] = current_time


                    get_msg_stampstamp = image_msg.header.stamp
                    ros_frame_id = image_msg.header.frame_id

                    current_time = nepi_sdk.get_msg_stamp()
                    latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
                    imgs_info_dict[img_topic]['get_latency_time'] = latency
                    #self.msg_if.pub_info("Detect Pub Latency: {:.2f}".format(latency)
                    # Request new img
                    det_dict_list = imgs_info_dict[img_topic]['det_dict_list']
                    #self.msg_if.pub_info("Got Detection List: " + str(det_dict_list))
                    if det_dict_list == None:
                        det_dict_list = []
                    
                    if self.use_last_image == False:
                        # process image for next time
                        use_cv2_img = nepi_img.rosimg_to_cv2img(image_msg)
                    else: 
                        # Use last image to align with detection data
                        self.cv2_img_lock.acquire()
                        use_cv2_img = copy.deepcopy(self.cv2_img)
                        self.cv2_img_lock.release()
                        #self.msg_if.pub_info("Image updated is None: " + str(use_cv2_img is None))

                    if use_cv2_img is not None:
                    
                        success = self.processDetImage(img_topic, 
                                                    use_cv2_img, 
                                                    det_dict_list, 
                                                    timestamp = get_msg_stampstamp, 
                                                    frame_3d = ros_frame_id, 
                                                    navpose_dict = navpose_dict)

                        current_time = nepi_sdk.get_msg_stamp()
                        latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
                        imgs_info_dict[img_topic]['pub_latency_time'] = latency                              

                        self.imgs_info_lock.acquire()
                        if img_topic in self.imgs_info_dict.keys():
                            imgs_info_dict[img_topic]['navpose_dict'] = navpose_dict
                            self.imgs_info_dict = imgs_info_dict
                        self.imgs_info_lock.release()
                        
                    if self.use_last_image == True:
                        # process image for next time
                        cv2_img = nepi_img.rosimg_to_cv2img(image_msg)
                        self.cv2_img_lock.acquire()
                        use_cv2_img = copy.deepcopy(self.cv2_img)
                        self.cv2_img = cv2_img
                        self.cv2_img_lock.release()
                        #self.msg_if.pub_info("Image updated is None: " + str(use_cv2_img is None))


    def processDetImage(self,img_topic, cv2_img, detect_dict_list, timestamp = None, frame_3d = 'nepi_base',navpose_dict = None):
      
        # Post process image with overlays
        if detect_dict_list is not None:
            # Publish image first for consumers
            #self.msg_if.pub_warn("Starting detect image: " + str(cv2_img.shape))
            cv2_img = self.apply_detection_overlay(img_topic, detect_dict_list, cv2_img)
            #self.msg_if.pub_warn("Return detect image: " + str(cv2_img.shape)

            add_overlay_list = []
            ## Overlay Detector Name

            if self.overlay_clf_name:
                add_overlay_list.append(self.model_name)

            if self.overlay_img_name:
                add_overlay_list.append(nepi_img.getImgShortName(img_topic))
            self.publishImgData(img_topic, 
                                cv2_img, 
                                timestamp = timestamp, 
                                frame_3d = frame_3d, 
                                add_overlay_list = add_overlay_list,
                                navpose_dict = navpose_dict)
        
            # Save Image Data if needed
            data_product = self.data_product
            self.save_data_if.save(data_product,cv2_img,timestamp)
        return True


    def apply_detection_overlay(self,img_topic, detect_dict_list, cv2_img):
        cv2_det_img = copy.deepcopy(cv2_img)
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1] 
        img_height = cv2_shape[0] 

        for detect_dict in detect_dict_list:
            img_size = cv2_img.shape[:2]

            # Overlay text data on OpenCV image
            font = cv2.FONT_HERSHEY_DUPLEX
            scale = 1.5e-3 - 0.1e-3 * math.ceil(max([img_height, img_width])/500)
            fontScale, fontThickness  = nepi_img.optimal_font_dims(cv2_img,font_scale = scale, thickness_scale = scale) 
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

            class_color = (0,0,127)
            if class_name in self.classes_list:
                class_ind = self.classes_list.index(class_name)
                if class_ind < len(self.classes_colors_list):
                    class_color = self.classes_colors_list[class_ind]
            line_thickness = 1 + math.ceil(max([img_height, img_width])/2000)

            #self.msg_if.pub_warn("Got Class Color: " + str(class_color))
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
                        fontThickness) 
                    #self.msg_if.pub_warn("Text Size: " + str(text_size))
                    line_height = text_size[0][1]
                    line_width = text_size[0][0]
                    x_padding = int(line_height*0.4)
                    y_padding = int(line_height*0.4)
                    center = bot_left_box[0] + int(( top_right_box[0] - bot_left_box[0]) / 2 )
                    #bot_left_text = (xmin + (line_thickness * 2) + x_padding , ymin + line_height + (line_thickness * 2) + y_padding)
                    bot_left_text = (center + x_padding , ymin - (line_thickness * 2) - y_padding)
                    # Create Text Background Box
                    #bot_left_box =  (bot_left_text[0] - x_padding , bot_left_text[1] + y_padding)
                    bot_left_box =  ( center - x_padding, bot_left_text[1] + y_padding)
                    top_right_box = (center + line_width + x_padding, bot_left_text[1] - line_height - y_padding )
                    box_color = [0,0,0]

                    try:
                        cv2.rectangle(cv2_det_img, bot_left_box, top_right_box, box_color , -1)
                        cv2.putText(cv2_det_img,text2overlay, 
                            bot_left_text, 
                            font, 
                            fontScale,
                            fontColor,
                            fontThickness,
                            lineType)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to apply overlay label text: " + str(e))

                    # Start name overlays    
                    x_start = int(img_width * 0.05)
                    y_start = int(img_height * 0.05)


        return cv2_det_img


    def navposeCb(self, navpose_msg, args):     
        start_time = nepi_sdk.get_time()   
        img_topic = args
        imgs_info_dict = self.getImgInfoDict() 
        if img_topic in imgs_info_dict.keys():
            navpose_dict = nepi_nav.convert_navpose_msg2dict(navpose_msg)
            if navpose_dict is not None:
                self.imgs_info_lock.acquire()
                if img_topic in self.imgs_info_dict.keys():
                    self.imgs_info_dict[img_topic]['navpose_dict'] = navpose_dict 
                self.imgs_info_lock.release()



    ### Monitor Output of AI detector to clear detection status
    def foundObjectCb(self,msg):
        stamp = msg.header.stamp
        img_stamp = msg.image_header.stamp
        img_topic = msg.image_topic
        det_count = msg.count
        self.imgs_info_lock.acquire()
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict) 
        self.imgs_info_lock.release()
        if img_topic in imgs_info_dict.keys() and det_count == 0:   
            imgs_info_dict[img_topic]['det_count'] = 0         
            imgs_info_dict[img_topic]['det_dict_list'] = None
        self.imgs_info_lock.acquire()
        if img_topic in self.imgs_info_dict.keys():
            self.imgs_info_dict = imgs_info_dict
        self.imgs_info_lock.release()

    def objectDetectedCb(self,msg):
        #self.msg_if.pub_info("Got Detection Msg: " + str(msg))
        stamp = msg.header.stamp
        img_stamp = msg.image_header.stamp
        img_topic = msg.image_topic
        self.imgs_info_lock.acquire()
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict) 
        self.imgs_info_lock.release()
        if img_topic in imgs_info_dict.keys():
            imgs_info_dict[img_topic]['object_detect_msg'] = msg
            blist = nepi_ais.get_boxes_list_from_msg(msg)
            imgs_info_dict[img_topic]['det_dict_list'] = blist
            imgs_info_dict[img_topic]['stamp'] = stamp
            imgs_info_dict[img_topic]['img_stamp'] = img_stamp
            imgs_info_dict[img_topic]['det_count'] = len(blist)
            #self.msg_if.pub_info("Updated detection data: " + str(blist))
            current_time = nepi_utils.get_time()
            imgs_info_dict[img_topic]['last_det_time'] = current_time
            
        self.imgs_info_lock.acquire()
        if img_topic in self.imgs_info_dict.keys():
            self.imgs_info_dict = imgs_info_dict
        self.imgs_info_lock.release()
        

    def statusCb(self,msg):
        self.last_status_msg=nepi_utils.get_time()
        self.status_msg = msg

        self.model_name = self.status_msg.name
        self.enabled = self.status_msg.enabled
        self.state = self.status_msg.state
        self.max_rate = self.status_msg.max_img_rate_hz
        self.use_last_image = self.status_msg.use_last_image

        self.classes_list = self.status_msg.selected_classes
        class_colors = self.status_msg.selected_classes_colors
        #self.msg_if.pub_warn("Got Classes Colors Msg " + str(class_colors))
        classes_colors_list = []
        for color_msg in class_colors:
            class_color = (int(color_msg.b),int(color_msg.g),int(color_msg.r))
            classes_colors_list.append(class_color)
        self.classes_colors_list = classes_colors_list
        #self.msg_if.pub_warn("Got Classes Colors List " + str(self.classes_colors_list))

        self.overlay_labels = self.status_msg.overlay_labels
        self.overlay_clf_name = self.status_msg.overlay_clf_name
        self.overlay_img_name = self.status_msg.overlay_img_name
        last_sel_imgs = copy.deepcopy(self.selected_img_topics)
        self.selected_img_topics = self.status_msg.selected_img_topics
        if last_sel_imgs != self.selected_img_topics:
            self.msg_if.pub_warn("Updating selected images topics: " + str(self.selected_img_topics))

    def shutdownCb(self):
        pass
        

#########################################
# Main
#########################################
if __name__ == '__main__':
  AiDetectorImgPub()





