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

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image 

from nepi_interfaces.msg import ImageStatus, AiBoundingBoxes
from nepi_interfaces.msg import AiDetectorStatus
#from nepi_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryRequest, AiDetectorInfoQueryResponse



from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.data_if import ColorImageIF






EXAMPLE_BOXES_INFO_DICT_ENTRY = {
    'model_name': 'test_model',
    'detect_timestamp': 0.0 ,
    'image_topic': '/test_topic',
    'image_timestamp': 0.0 ,
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
    'width': 0,
    'height': 0,
    'timestamp': nepi_sdk.get_time(),
    'image_topic': 'None',
    'ros_img_header': Header(),
    'image_timestamp': Header().stamp
}


BLANK_SIZE_DICT = { 'h': 350, 'w': 700, 'c': 3}

BLANK_CV2_IMAGE = nepi_img.create_blank_image((BLANK_SIZE_DICT['h'],BLANK_SIZE_DICT['w'],BLANK_SIZE_DICT['c']))

BLANK_IMG_DICT = {       
    'width': 0,
    'height': 0,
    'timestamp': nepi_sdk.get_time(),
    'image_topic': 'None',
    'image_timestamp': nepi_sdk.get_time()
}


WATCHDOG_DELAY=60
WATCHDOG_TIMEOUT=3

class AiDetectorImgPub:

    IMG_DATA_PRODUCT = 'detection_image'

    det_sub_names = ['bounding_boxes']
    
    node_if = None


    self_managed = True
    model_name = "None"

    # img_if = None
    # img_if_all = None

    save_data_if = None

    cv2_img = None
    cv2_img_lock = threading.Lock()

    img_node_dict = dict()
    img_det_lock = threading.Lock()


    imgs_info_dict = dict()
    imgs_info_lock = threading.Lock()
    imgs_img_proc_dict = dict()

    save_cfg_if = None

    state_str_msg = 'Loading'

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

    data_product = IMG_DATA_PRODUCT


    DEFAULT_NODE_NAME = "detector_img_pub" # Can be overwitten by luanch command

    connected = False

    watchdog_timeout = None

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

        
        backup_data_products = ['bounding_boxes','detection_images'] 
        self.data_products = nepi_sdk.get_param(self.node_namespace + "/data_products",backup_data_products)
        self.data_product = self.data_products[-1]
        self.msg_if.pub_warn("Starting with Data Products: " + str(self.data_products))
        
        backup_det_namespace = self.node_namespace.replace("_img_pub","")
        self.det_namespace = nepi_sdk.get_param(self.node_namespace + "/det_namespace",backup_det_namespace)
        self.msg_if.pub_warn("Starting with Detector Namespace: " + str(self.det_namespace))

        self.all_namespace = nepi_sdk.get_param(self.node_namespace + "/all_namespace",'')
        self.msg_if.pub_warn("Starting with All Namespace: " + str(self.all_namespace))


        self.status_msg = AiDetectorStatus()
        self.model_name = os.path.basename(self.det_namespace)
        self.enabled = False
        self.state_str_msg = "Unknown"
        self.max_rate = 1.0
        self.use_last_image = True

        self.pub_image_enabled = True
        self.overlay_labels = True
        self.overlay_range_bearing = True
        self.overlay_clf_name = False
        self.overlay_img_name = False

        self.selected_img_topics = []
        self.selected_img_navpose_topics = []
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
            'status_sub': {
                'msg': AiDetectorStatus,
                'namespace': self.det_namespace,
                'topic': 'status',
                'qsize': 10,
                'callback': self.statusCb, 
                'callback_args': ()
            },
            'bounding_boxes': {
                'msg': AiBoundingBoxes,
                'namespace': self.det_namespace,
                'topic': 'bounding_boxes',
                'qsize': 10,
                'callback': self.objectDetectedCb, 
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
        



        ###############################
        # Create System IFs

       
        # Setup Save Data IF
        factory_data_rates= {}
        for d in self.data_products:
            factory_data_rates[d] = [1.0, 0.0, 100] 
            
        self.save_data_if = SaveDataIF(data_products = self.data_products, pub_status = False, factory_rate_dict = factory_data_rates, namespace = self.det_namespace,
                        msg_if = self.msg_if
                        )
        
        nepi_sdk.sleep(1)
        if self.save_data_if is not None:
            self.status_msg.save_data_topic = self.save_data_if.get_namespace()
            self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic))


        time.sleep(1)


        ##########################
        # Complete Initialization

        # Start Timer Processes
        
        nepi_sdk.start_timer_process((0.1), self.updaterCb, oneshot = True)
        #nepi_sdk.start_timer_process((0.1), self.updateImgSubsCb, oneshot = True)
        self.last_status_time=nepi_utils.get_time()
        nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)
        nepi_sdk.on_shutdown(self.shutdownCb)
        
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
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        self.imgs_info_lock.release()
        img_topics = list(imgs_info_dict.keys())
        #self.msg_if.pub_warn("Updating active topics: " +  str(img_topics))
        active_img_topics = []
        for img_topic in img_topics:
            if img_topic in imgs_info_dict.keys():
                if 'active' in imgs_info_dict[img_topic].keys():
                    if imgs_info_dict[img_topic]['active'] == True:
                        #self.msg_if.pub_warn("Found active topic: " +  str(img_topic))
                        active_img_topics.append(img_topic)
        return active_img_topics


    def updaterCb(self,timer):
        # Clear boxes if stall
        selected_img_topics = copy.deepcopy(self.selected_img_topics)
        active_img_topics = self.getActiveImgTopics()
        #self.msg_if.pub_warn("")
        #self.msg_if.pub_warn("Updating with image topics: " +  str(selected_img_topics))
        #self.msg_if.pub_warn("Updating with active image topics: " +  str(active_img_topics))
        current_time = nepi_utils.get_time()
        for img_topic in self.imgs_info_dict.keys():
            last_time = self.imgs_info_dict[img_topic]['last_det_time']
            check_time = current_time - last_time
            '''
            if check_time > self.clear_det_time or self.enabled == False or self.state_str_msg != 'Detecting':
                try:
                    self.imgs_info_dict[img_topic]['det_dict_list'] = None
                except:
                    pass
            '''


        # Do Image Subs updating

        #self.msg_if.pub_warn("Subscriber Check with selected image topics: " +  str(selected_img_topics))
        #self.msg_if.pub_warn("Subscriber Check  with active image topics: " +  str(active_img_topics))
        purge_list = []
        if self.max_rate == -1 :
            purge_list = selected_img_topics
        elif self.pub_image_enabled == True:
            # Update Image subscribers
            found_img_topics = []
            for img_topic in selected_img_topics:
                img_topic = nepi_sdk.find_topic(img_topic, exact = True)
                if img_topic != '':
                    found_img_topics.append(img_topic)
                    if img_topic not in active_img_topics:
                        self.msg_if.pub_warn('Will subscribe to image topic: ' + img_topic)
                        success = self.subscribeImgTopic(img_topic)
                        #self.msg_if.pub_warn('Subscribe process returned: ' + str(success))
                       
            # Update Image Subs purge list      
            active_img_topics = self.getActiveImgTopics() 
            #self.msg_if.pub_warn('Purge Check with active topics: ' + str(active_img_topics))       
            #self.msg_if.pub_warn('Purge Check with found topics: ' + str(active_img_topics)) 
            for img_topic in active_img_topics:
                if img_topic not in found_img_topics:
                    purge_list.append(img_topic)
        elif self.pub_image_enabled == False:
             purge_list = copy.deepcopy(list(self.imgs_info_dict.keys()))

        # Do image sub purging if required
        #self.msg_if.pub_warn('Purging image topics: ' + str(purge_list))
        #self.msg_if.pub_warn("")
        for img_topic in purge_list:
                if img_topic not in active_img_topics:
                    purge_list.remove(img_topic)
        if len(purge_list) > 0:
            self.msg_if.pub_warn('Purging image topics: ' + str(purge_list))
        for img_topic in purge_list:
            self.msg_if.pub_warn('Will unsubscribe from image topic: ' + img_topic)
            success = self.unsubscribeImgTopic(img_topic)
            self.msg_if.pub_warn('Unsubsribe process returned: ' + str(success))
            nepi_sdk.sleep(1)

        '''
        self.pub_img_if.publish_msg_img("Detector not Enabled")

        self.pub_img_if.publish_msg_img("Detector Sleeping")

        self.pub_img_if.publish_msg_img("Waiting for Image")

        self.pub_img_if.publish_msg_img("Image not Connected")
        '''
        nepi_sdk.start_timer_process((1), self.updaterCb, oneshot = True)


    def watchdogCb(self,timer):
        cur_time=nepi_utils.get_time()
        timer=cur_time-self.last_status_time
        if self.watchdog_timeout is None:
            self.watchdog_timeout = WATCHDOG_TIMEOUT
            nepi_sdk.sleep(WATCHDOG_DELAY)
        else:
            if timer > WATCHDOG_TIMEOUT:
                
                msg="Lost connection to parent node status msg.  Shutting down"
                self.msg_if.pub_warn(msg)
                nepi_sdk.signal_shutdown(msg)
            
        nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)


    def subscribeImgTopic(self,img_topic):
        success = False
        #self.msg_if.pub_warn('Subscribing with image dict keys: ' + str(self.imgs_info_dict.keys()))
        if img_topic == "None" or img_topic == "":
            self.msg_if.pub_warn('Skipping subscribe, Image topic is None: ' + str(img_topic))
            return False
        
        # Create Publishers
        self.msg_if.pub_warn('Subscribing to image topic: ' + img_topic)

        img_source_topic = os.path.dirname(img_topic)
        det_name = os.path.basename(self.det_namespace)
        #self.msg_if.pub_warn('Creating namespace for image name: ' + img_source_topic)

        pub_namespace = os.path.join(os.path.dirname(img_topic),det_name)
        img_pub_topic = os.path.join(pub_namespace,self.data_product)
        self.msg_if.pub_warn('Publishing imgage ' + img_source_topic + ' on namespace: ' + img_pub_topic)


        if img_topic in self.imgs_info_dict.keys():
            # num_active = self.getActiveImgTopics()
            # if num_active == 0 and self.img_if is not None:
            #     self.img_if.register_pubs(self.PUBS_DICT)
            #self.msg_if.pub_warn('Checking if Image Topic is active: ' + img_topic)
            imgs_info_dict = self.getImgInfoDict()
            if imgs_info_dict[img_topic]['active'] == True:
                self.msg_if.pub_warn('Skipping subscribe, Image Topic is active: ' + img_topic)
                return  False    
            self.img_det_lock.acquire()
            if img_topic in self.img_node_dict.keys():
                self.img_node_dict[img_topic]['img_pub'] = nepi_sdk.create_publisher(img_pub_topic,Image, queue_size = 1, log_name_list = [])
                nepi_sdk.sleep(1)
                self.img_node_dict[img_topic]['img_sub'] = nepi_sdk.create_subscriber(img_topic,Image, self.imageCb, queue_size = 1, callback_args= (img_topic), log_name_list = [])
                self.img_node_dict[img_topic]['img_if'].register_pubs()
            self.img_det_lock.release()

            return True


        img_pub = nepi_sdk.create_publisher(img_pub_topic,Image, queue_size = 1, log_name_list = [])
        nepi_sdk.sleep(1)
        img_sub = nepi_sdk.create_subscriber(img_topic,Image, self.imageCb, queue_size = 1, callback_args= (img_topic), log_name_list = [])
        img_status_topic = nepi_sdk.create_namespace(img_topic, 'status')
        img_stutus_sub = nepi_sdk.create_subscriber(img_status_topic,ImageStatus, self.imageStatusCb, queue_size = 1, callback_args= (img_topic), log_name_list = [])

        # if self.img_if is None:
        #     # Create image publisher
        #     self.img_if = ColorImageIF(namespace = self.det_namespace,
        #                     data_product_name = self.data_product,
        #                     data_source_description = 'image',
        #                     data_ref_description = 'image',
        #                     perspective = 'pov',
        #                     save_data_if = self.save_data_if,
        #                     init_overlay_list = [],
        #                     log_name = self.data_product,
        #                     log_name_list = [],
        #                     msg_if = self.msg_if
        #                     )


        # Create image publisher
        img_if = ColorImageIF(namespace = pub_namespace ,
                        data_product_name = self.data_product,
                        data_source_description = 'image',
                        data_ref_description = 'image',
                        perspective = 'pov',
                        save_data_if = self.save_data_if,
                        init_overlay_list = [],
                        log_name = self.data_product,
                        log_name_list = [],
                        msg_if = self.msg_if
                        )
        # Subscribe to new image topic
        self.img_det_lock.acquire()
        self.img_node_dict[img_topic] = {
                                        'img_sub': img_sub,
                                        'img_status_sub': img_stutus_sub,
                                        'img_pub': img_pub,
                                        'img_if': img_if
                                        }   
        self.img_det_lock.release()


        ####################
        # Create img info dict
        img_info_dict = dict()  
        img_info_dict['active'] = True
        img_info_dict['img_connected'] = False
        img_info_dict['img_published'] = False
        img_info_dict['status_msg'] = None
        img_info_dict['pub_namespace'] = pub_namespace

        img_info_dict['connected'] = False 
        img_info_dict['publishing'] = False
        img_info_dict['get_latency_time'] = 0
        img_info_dict['pub_latency_time'] = 0
        img_info_dict['process_time'] = 0 
        img_info_dict['last_img_time'] = 0
        img_info_dict['last_det_time'] = 0
        img_info_dict['det_dict_list'] = []   
        img_info_dict['loc_dict_list'] = []

        self.imgs_info_lock.acquire()
        self.imgs_info_dict[img_topic] = img_info_dict
        self.imgs_info_lock.release()
        #self.msg_if.pub_warn('Subscribed with image dict key: ' + str(img_info_dict.keys()))
        #self.msg_if.pub_warn('Subscribed with images dict: ' + str(self.imgs_info_dict))
       
        return True
    


    def unsubscribeImgTopic(self,img_topic):
        if img_topic in self.imgs_info_dict.keys():
            if self.imgs_info_dict[img_topic]['active'] == True:
                self.msg_if.pub_warn('Unsubscribing from image topic: ' + img_topic)


                # Unsubscribe
                self.img_det_lock.acquire()
                if img_topic in self.img_node_dict.keys():
                    self.img_node_dict[img_topic]['img_sub'].unregister()
                    self.img_node_dict[img_topic]['img_status_sub'].unregister
                    self.img_node_dict[img_topic]['img_pub'].unregister()
                    self.img_node_dict[img_topic]['img_if'].unregister_pubs()
                    nepi_sdk.sleep(1)
                    self.img_node_dict[img_topic]['img_sub'] = None
                    self.img_node_dict[img_topic]['img_status_sub'] = None
                    self.img_node_dict[img_topic]['img_pub'] = None
                self.img_det_lock.release()

                if img_topic in self.imgs_info_dict.keys():
                    self.msg_if.pub_warn('Setting image topic inactive: ' + img_topic)
                    self.imgs_info_lock.acquire()
                    self.imgs_info_dict[img_topic]['active'] = False
                    self.imgs_info_dict[img_topic]['status_msg'] = None
                    self.imgs_info_dict[img_topic]['connected'] = False 
                    self.imgs_info_dict[img_topic]['publishing'] = False
                    self.imgs_info_dict[img_topic]['img_connected'] = False
                    self.imgs_info_dict[img_topic]['img_published'] = False
                   
                    self.imgs_info_lock.release()
                    #self.msg_if.pub_warn('Unubscribed with images dict: ' + str(self.imgs_info_dict))

                nepi_sdk.sleep(1)

                # num_active = self.getActiveImgTopics()
                # if num_active == 0 and self.img_if is not None:
                #     self.img_if.unregister_pubs()
                    
                return True
        return False




    def imageStatusCb(self, status_msg, args):   
            img_topic = args  
            if img_topic not in self.imgs_info_dict.keys():
                return
            self.imgs_info_lock.acquire()
            if img_topic in self.imgs_info_dict.keys():
                    if self.imgs_info_dict[img_topic]['status_msg'] is None:
                        self.msg_if.pub_warn('Connected to image status topic: ' + img_topic + '/status')
                    self.imgs_info_dict[img_topic]['status_msg'] = status_msg
            self.imgs_info_lock.release()                


    def imageCb(self, image_msg, args):   
            img_topic = args  

            if img_topic not in self.imgs_info_dict.keys():
                return

            needs_save = False
            if self.save_data_if is not None:
                needs_save = self.save_data_if.data_product_should_save(self.data_product)
            
            

            if self.imgs_info_dict[img_topic]['img_connected'] == False:
                self.msg_if.pub_warn('Connected to image topic: ' + img_topic)
            self.imgs_info_dict[img_topic]['img_connected'] = True



            needs_img = True
            # needs_img = False
            # if img_topic in self.imgs_info_dict.keys():
            #     self.img_det_lock.acquire()
            #     if  self.img_node_dict[img_topic]['img_if'] is not None:
            #         has_subs = self.img_node_dict[img_topic]['img_if'].has_subscribers_check()
            #         if has_subs == True:
            #             needs_img = True
            #     self.img_det_lock.release()
            # if self.imgs_info_dict[img_topic]['publishing'] == False:
            #     self.msg_if.pub_warn('Subscriber got need_img, need_save, and pub_img checks for image topic: ' + img_topic + ' : ' + str([needs_img, needs_save,self.pub_image_enabled]))
            # if ( needs_img or needs_save ) and self.pub_image_enabled:


            if self.imgs_info_dict[img_topic]['publishing'] == False:
                pass #self.msg_if.pub_warn('Subscriber got need_img, need_save, and pub_img checks for image topic: ' + img_topic + ' : ' + str([needs_img, needs_save,self.pub_image_enabled]))
            
            if ( needs_img or needs_save ) and self.pub_image_enabled:
                start_time = nepi_sdk.get_time()   
                


                sel_imgs = copy.deepcopy(self.selected_img_topics) 
                max_rate = copy.deepcopy(self.max_rate)
                if img_topic in self.imgs_info_dict.keys() and img_topic in sel_imgs and max_rate > .01:
                    if self.imgs_info_dict[img_topic]['connected'] == False:
                        self.msg_if.pub_warn("Got image topic: " + str(img_topic))
                    self.imgs_info_dict[img_topic]['connected'] = True
                    if self.enabled == True and self.state_str_msg == 'Detecting':

                        if self.imgs_info_dict[img_topic]['publishing'] == False:
                            self.msg_if.pub_warn("Processing image topic: " + str(img_topic))

                        # Check if time to publish
                        delay_time = float(1) / max_rate 
                        last_img_time = self.imgs_info_dict[img_topic]['last_img_time']
                        current_time = nepi_utils.get_time()
                        timer = round((current_time - last_img_time), 3)
                        if self.imgs_info_dict[img_topic]['publishing'] == False:   
                            self.msg_if.pub_warn("Process Delay and Timer: " + str(delay_time) + " " + str(timer))
                        if timer > delay_time: 
                            self.imgs_info_dict[img_topic]['last_img_time'] = current_time


                            stamp = image_msg.header.stamp
                            timestamp = copy.deepcopy(float(stamp.to_sec()))

                            ros_frame_id = image_msg.header.frame_id

                            current_time = nepi_utils.get_time()
                            latency = (current_time - timestamp )
                            self.imgs_info_dict[img_topic]['get_latency_time'] = latency
                            #self.msg_if.pub_info("Detect Pub Latency: {:.2f}".format(latency)
                            # Request new img
                            det_dict_list = copy.deepcopy(self.imgs_info_dict[img_topic]['det_dict_list'])
                            #self.msg_if.pub_info("Got Detection List: " + str(det_dict_list))
                            if det_dict_list == None:
                                det_dict_list = []
                                
                            loc_dict_list = copy.deepcopy(self.imgs_info_dict[img_topic]['loc_dict_list'])
                            #self.msg_if.pub_info("Got Detection List: " + str(det_dict_list))
                            if loc_dict_list == None:
                                loc_dict_list = []
                            

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
                                if self.imgs_info_dict[img_topic]['publishing'] == False:   
                                    self.msg_if.pub_warn("Will process img with shape: " + str(use_cv2_img.shape) )
                                success = self.processDetImage(img_topic, 
                                                            use_cv2_img, 
                                                            det_dict_list, 
                                                            loc_dict_list,
                                                            timestamp = timestamp,  
                                )

                                current_time = nepi_utils.get_time()
                                latency = (current_time - timestamp )
                                self.imgs_info_dict[img_topic]['pub_latency_time'] = latency                              

                                
                            if self.use_last_image == True:
                                # process image for next time
                                cv2_img = nepi_img.rosimg_to_cv2img(image_msg)
                                self.cv2_img_lock.acquire()
                                use_cv2_img = copy.deepcopy(self.cv2_img)
                                self.cv2_img = cv2_img
                                self.cv2_img_lock.release()
                                #self.msg_if.pub_info("Image updated is None: " + str(use_cv2_img is None))

    def processFileImg(self, img_file,det_dict_list,loc_dict_list):   
        img_topic = 'img_file'      
        max_rate = copy.deepcopy(self.max_rate)
        if max_rate > .01:
            if self.enabled == True and self.state_str_msg == 'Detecting':


                # Check if time to publish
                delay_time = float(1) / max_rate 
                last_img_time = 0
                if 'last_img_time' in self.imgs_info_dict['img_file'].keys():
                    last_img_time = self.imgs_info_dict['img_file']['last_img_time']
                current_time = nepi_utils.get_time()
                timer = round((current_time - last_img_time), 3)
                #self.msg_if.pub_warn("Delay and Timer: " + str(delay_time) + " " + str(timer))
                if timer > delay_time: 
                    cv2_img = cv2.imread(img_file)
                    if cv2_img is not None:
                        self.imgs_info_dict['img_file']['last_img_time'] = current_time


                        timestamp = nepi_utils.get_time()
                        ros_frame_id = 'nepi_base'

                        current_time = nepi_utils.get_time()
                        latency = (current_time - timestamp)
                        self.imgs_info_dict['img_file']['get_latency_time'] = latency
                        #self.msg_if.pub_info("Detect Pub Latency: {:.2f}".format(latency)
                        # Request new img
                        
                        #self.msg_if.pub_info("Got Detection List: " + str(det_dict_list))
                        if det_dict_list == None:
                            det_dict_list = []   
                            loc_dict_list = []             
                        success = self.processDetImage(img_topic, 
                                                    cv2_img, 
                                                    det_dict_list, 
                                                    loc_dict_list,
                                                    timestamp = timestamp,  
                                                    )

                        current_time = nepi_utils.get_time()
                        latency = (current_time - timestamp)
                        self.imgs_info_dict['img_file']['pub_latency_time'] = latency                              

                           


    def processDetImage(self,img_topic, cv2_img, detect_dict_list, loc_dict_list, timestamp = None):
      
        # Post process image with overlays
        if detect_dict_list is not None:
            # Publish image first for consumers
            #self.msg_if.pub_warn("Starting detect image: " + str(cv2_img.shape))
            cv2_img = self.apply_detection_overlay(img_topic, detect_dict_list, loc_dict_list, cv2_img)
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
                                add_overlay_list = add_overlay_list
                                )
            
            if self.imgs_info_dict[img_topic]['img_published'] == False:
                namespace = self.imgs_info_dict[img_topic]['pub_namespace']
                topic = os.path.join(namespace,self.data_product)
                self.msg_if.pub_warn('Published image topic: ' + topic)
            self.imgs_info_dict[img_topic]['img_published'] = True
                
        
            # Save Image Data if needed
            data_product = self.data_product
            if self.save_data_if is not None:
                self.save_data_if.save(data_product,cv2_img,timestamp)
        return True


    def publishImgData(self, img_topic, cv2_img, encoding = "bgr8", timestamp = None, add_overlay_list = []):

            needs_save = False
            if self.save_data_if is not None:
                needs_save = self.save_data_if.data_product_should_save(self.data_product)

            needs_img = True
            # needs_img = False
            # if img_topic in self.imgs_info_dict.keys():
            #     self.img_det_lock.acquire()
            #     if  self.img_node_dict[img_topic]['img_if'] is not None:
            #         has_subs = self.img_node_dict[img_topic]['img_if'].has_subscribers_check()
            #         if has_subs == True:
            #             needs_img = True
            #     self.img_det_lock.release()
            
            if self.imgs_info_dict[img_topic]['publishing'] == False:
                pass #self.msg_if.pub_warn('Subscriber got need_img, need_save, and pub_img checks for image topic: ' + img_topic + ' : ' + str([needs_img, needs_save,self.pub_image_enabled]))
            if ( needs_img or needs_save ) and self.pub_image_enabled:

                if img_topic in self.imgs_info_dict.keys():

                    if self.imgs_info_dict[img_topic]['publishing'] == False:
                        self.msg_if.pub_warn("Publishing image topic: " + str(img_topic))
                    self.imgs_info_dict[img_topic]['publishing'] = True
                        
                       
                    self.img_det_lock.acquire()
                    img_if = self.img_node_dict[img_topic]['img_if']
                    img_pub = self.img_node_dict[img_topic]['img_pub']
                    
                    img_if_ready = img_if.ready
                    if img_if_ready == False:
                        img_msg = nepi_img.cv2img_to_rosimg(cv2_img)
                        nepi_sdk.publish_pub(img_pub,img_msg)
                    else:
                        img_if.publish_cv2_img(cv2_img, 
                                            encoding = encoding, 
                                            timestamp = timestamp, 
                                            add_overlay_list = add_overlay_list
                                            )
                    self.img_det_lock.release()
                # if self.img_if is not None:
                #     self.img_if.publish_cv2_img(cv2_img, 
                #                             encoding = encoding, 
                #                             timestamp = timestamp, 
                #                             add_overlay_list = add_overlay_list,
                #                             add_pubs = add_pubs
                #                             )


    def apply_detection_overlay(self,img_topic, detect_dict_list, loc_dict_list, cv2_img):
        cv2_det_img = copy.deepcopy(cv2_img)
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1] 
        img_height = cv2_shape[0] 

        for i, detect_dict in enumerate(detect_dict_list):
            img_size = cv2_img.shape[:2]

            # Overlay text data on OpenCV image
            font = cv2.FONT_HERSHEY_DUPLEX
            scale = 1.5e-3 - 0.1e-3 * math.ceil(max([img_height, img_width])/700)
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
                #self.msg_if.pub_warn("Got Class Index: " + str(class_ind))
                if class_ind < len(self.classes_colors_list):
                    class_color = self.classes_colors_list[class_ind]

            #self.msg_if.pub_warn("Got Class Color: " + str(class_color) + ' type: ' + str(type(class_color)) + " type: " + str(type(class_color[0])) )
            line_thickness = 1 + math.ceil(max([img_height, img_width])/2000)
            

            success = False
            try:
                cv2.rectangle(cv2_det_img, bot_left_box, top_right_box, class_color, thickness=line_thickness)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to create bounding box rectangle: " + str(e))

            # Overlay text data on OpenCV image
            if success == True:


                ## Overlay Text
                overlay_labels =  self.overlay_labels
                overlay_range_bearing =  self.overlay_range_bearing

                overlay_text = ""

                if overlay_labels:
                    overlay_text = overlay_text + class_name + " "
                if overlay_range_bearing:
                    rb_text = ''
                    if len(loc_dict_list) > i:
                        loc_dict = loc_dict_list[i]
                        if loc_dict['range_m'] != -999 and loc_dict['range_m'] != '':
                            rb_text = rb_text + str(round(loc_dict['range_m'],1)) + 'm :'
                        if loc_dict['azimuth_deg'] != -999 and loc_dict['elevation_deg'] != -999:
                            rb_text = rb_text + str(round(loc_dict['azimuth_deg'],1)) + 'deg '
                            rb_text = rb_text + str(round(loc_dict['elevation_deg'],1)) + 'deg '
                    if len(rb_text) > 0:
                        overlay_text = overlay_text + rb_text



                if len(overlay_text) > 0:
                    text2overlay=overlay_text
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



    def objectDetectedCb(self,msg):
        self.connected = True
        #self.msg_if.pub_info("Got Detection Msg: " + str(msg))
        img_stamp = msg.image_timestamp
        img_topic = msg.image_topic
        current_time = nepi_utils.get_time()
        blist = nepi_ais.get_boxes_list_from_msg(msg)
        llist = []
        l_msg_list = msg.localizations
        for l_msg in l_msg_list:
            l_dict = dict()
            l_dict['range_m'] = l_msg.range_m
            l_dict['azimuth_deg'] = l_msg.azimuth_deg
            l_dict['elevation_deg'] = l_msg.elevation_deg
            llist.append(l_dict)
        if img_topic in self.imgs_info_dict.keys():
            self.imgs_info_dict[img_topic]['det_dict_list'] = blist
            self.imgs_info_dict[img_topic]['loc_dict_list'] = llist
            self.imgs_info_dict[img_topic]['img_stamp'] = img_stamp      
            self.imgs_info_dict[img_topic]['last_det_time'] = current_time
        else:
            if os.path.exists(img_topic):
                self.imgs_info_dict['img_file'] = dict()
                self.imgs_info_dict['img_file']['img_stamp'] = img_stamp      
                self.imgs_info_dict['img_file']['last_det_time'] = current_time
                self.processFileImg(img_topic,blist,llist)

  



    def statusCb(self,msg):
        self.last_status_time=nepi_utils.get_time()

        self.status_msg = msg

        self.model_name = self.status_msg.name
        self.enabled = self.status_msg.enabled
        self.state_str_msg = self.status_msg.state_str_msg
        self.max_rate = self.status_msg.max_img_rate_hz
        self.use_last_image = self.status_msg.use_last_image

        self.classes_list = self.status_msg.selected_classes

        if len(self.classes_colors_list) != len(self.classes_list) :
            #self.msg_if.pub_warn("Detector provided classes list: " + str(self.classes))
            num_colors = len(self.classes_list)
            self.classes_colors_list = nepi_img.create_bgr_jet_colormap_list(num_colors)
            self.msg_if.pub_warn("Created classes color list: " + str(self.classes_colors_list))


        self.pub_image_enabled = self.status_msg.pub_image_enabled
        self.overlay_labels = self.status_msg.overlay_labels
        self.overlay_range_bearing = self.status_msg.overlay_range_bearing
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





