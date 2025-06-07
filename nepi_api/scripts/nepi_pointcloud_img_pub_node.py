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

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image

from nepi_sdk_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes
from nepi_sdk_interfaces.msg import DepthMapStatus

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.data_if import ImageIF





NONE_IMG_DICT = {       
    'cv2_img': None,
    'width': 0,
    'height': 0,
    'timestamp': nepi_sdk.get_time(),
    'ros_img_topic': 'None',
    'ros_img_header': Header(),
    'ros_img_stamp': Header().stamp
}


SOURCE_TOPIC_TIMEOUT = 5

class DepthMapImgPub:

    IMG_DATA_PRODUCT = 'depth_map_image'

    img_ifs_dict = dict()
    img_ifs_lock = threading.Lock()
    
    img_image_if = None
    img_info_dict = None
 
    save_cfg_if = None
    
    DEFAULT_NODE_NAME = "depth_map_img_pub" # Can be overwitten by luanch command
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
        self.msg_if.pub_warn("Starting IF Initialization Processes")

        ##############################  
        # Init Class Variables 

            

        self.data_product = nepi_sdk.get_param(self.node_namespace + "/data_product",self.IMG_DATA_PRODUCT)
        self.data_products = [self.data_product]
        self.msg_if.pub_info("Starting with Image Data Product: " + str(self.data_product))
        
        backup_dm_namespace = self.node_namespace.replace("_img_pub","/depth_map")
        self.dm_namespace = nepi_sdk.get_param(self.node_namespace + "/dm_namespace",backup_dm_namespace)


        self.status_msg = DepthMapStatus()
        self.dm_name = os.path.basename(self.dm_namespace)
        self.enabled = True
        self.max_pub_rate = 5
        self.min_range_m = 0.0
        self.max_range_m = 20.0
        self.min_range_ratio = 0.0
        self.max_range_ratio = 1.0

  
        ##############################  
        # Create NodeClassIF Class  

        # Configs Dict ########################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.dm_namespace,
        }


        # Params Config Dict ####################
        self.PARAMS_DICT = None


        # Services Config Dict ####################
        self.SRVS_DICT = None


        self.PUBS_DICT = None


        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'msg': DepthMapStatus,
                'namespace': self.dm_namespace,
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
                        msg_if = self.msg_if
                                            )


        self.node_if.wait_for_ready()

        ###############################
        # Create System IFs

       
        # Setup Save Data IF
        factory_data_rates= {}
        factory_data_rates[self.data_product] = [1.0, 0.0, 100.0] 

        self.save_data_if = SaveDataIF(data_products = self.data_products, factory_rate_dict = factory_data_rates, namespace = self.dm_namespace,
                        msg_if = self.msg_if
                                            )

        
        time.sleep(1)


        ##########################
        # Complete Initialization

        # Start Timer Processes
        nepi_sdk.start_timer_process((0.1), self.updaterCb, oneshot = True)

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever 
        nepi_sdk.spin()
        #########################################################


    def initCb(self,do_updates = False):
        self.msg_if.pub_info(" Setting init values to param values")
        if do_updates == True:
            self.resetCb(do_updates)


    def resetCb(self):
        pass

    def factoryResetCb(self):
        pass

    def updaterCb(self,timer):
        # Unsubsribe if image not publishing
        if self.img_info_dict is not None:
            connected = self.img_info_dict['connected']
            last_time = self.img_info_dict['last_img_time']
        else:
            connected = False
            last_time = None
        current_time = nepi_utils.get_time()
        if connected == True and last_time is not None:
            check_time = current_time - last_time
            if check_time > SOURCE_TOPIC_TIMEOUT:
                success = self.unsubscribeImgTopic(self.dm_namespace)


        if connected == False:
                depth_map_topic = nepi_sdk.find_topic(self.dm_namespace)
                if depth_map_topic != '':
                    success = self.subscribeImgTopic(depth_map_topic)     

        nepi_sdk.start_timer_process((.5), self.updaterCb, oneshot = True)


    def subscribeImgTopic(self,depth_map_topic):
        if depth_map_topic == "None" or depth_map_topic == "":
            return False
        else:

            img_sub_dict = {
                        'namespace': depth_map_topic,
                        'msg': Image,
                        'topic': '',
                        'qsize': 1,
                        'callback': self.depthMapCb,
                        'callback_args': (depth_map_topic)
                    }

            img_info_dict = copy.deepcopy(self.img_info_dict)
            # Check if exists
            if img_info_dict is not None:
                if self.img_ifs_dict:
                    self.msg_if.pub_info('Subsribing to image topic: ' + depth_map_topic)  
                    # Try and Reregister img_sub
                    self.img_ifs_lock.acquire()
                    self.img_ifs_dict['subs_if'].register_sub('img_sub',img_sub_dict)
                    self.img_ifs_lock.release()
                    self.msg_if.pub_warn('Registered : ' + depth_map_topic +  ' ' + str(self.img_ifs_dict))
                    # Set back to active
                    self.img_info_dict['active'] = True
            else:
                    
                # Create register new image topic
                self.msg_if.pub_info('Registering to topic: ' + depth_map_topic)
                dm_name = os.path.basename(self.dm_namespace)
                pub_namespace = self.dm_namespace.replace(dm_name,self.data_product)

                ####################
                # Create img info dict
                self.img_info_dict = dict()  
                self.img_info_dict['namespace'] = pub_namespace
                self.img_info_dict['active'] = True
                self.img_info_dict['publishing'] = False
                self.img_info_dict['connected'] = False 
                self.img_info_dict['get_latency_time'] = 0
                self.img_info_dict['pub_latency_time'] = 0
                self.img_info_dict['process_time'] = 0 
                self.img_info_dict['last_img_time'] = 0
 

                self.msg_if.pub_info('Subsribing to topic: ' + depth_map_topic)
                img_image_if = ImageIF(namespace = pub_namespace , log_name = self.data_product,
                        msg_if = self.msg_if
                                            )


                ####################
                # Pubs Config Dict 
                SUBS_DICT = {
                    'img_sub': img_sub_dict
                }

                ####################
                # Subs Config Dict 
                img_subs_if = NodeClassIF(
                                subs_dict = SUBS_DICT,
                        msg_if = self.msg_if
                                            )



                time.sleep(1)

                ####################
                # Add Img Subs and Pubs IFs to Img IFs Dict
                self.img_ifs_lock.acquire()
                self.img_ifs_dict = {
                                                'img_if': img_image_if,
                                                'subs_if': img_subs_if
                                                }   

                self.img_ifs_lock.release()
                self.msg_if.pub_warn('Registered : ' + depth_map_topic)


                ####################
                # Publish blank msg img to prime topics
                #img_image_if
 
    
            return True
                 

    def unsubscribeImgTopic(self,depth_map_topic):
        self.msg_if.pub_warn('Unregistering image topic: ' + depth_map_topic)
        if depth_map_topic in self.img_ifs_dict.keys():
            self.img_ifs_lock.acquire()
            self.img_ifs_dict['subs_if'].unregister_sub('img_sub')
            self.img_ifs_lock.release()

        #Leave img pub running in case it is switched back on
    
        # Clear info dict
        if depth_map_topic in self.img_info_dict.keys():
            self.img_info_dict['active'] = False
            self.img_info_dict['publishing'] = False
            self.img_info_dict['connected'] = False 
            self.img_info_dict['get_latency_time'] = 0
            self.img_info_dict['pub_latency_time'] = 0
            self.img_info_dict['process_time'] = 0 
            self.img_info_dict['last_img_time'] = 0 

        return True




    def depthMapCb(self, depth_map_msg, args):     
        start_time = nepi_sdk.get_time()   
        depth_map_topic = args
        img_info_dict = copy.deepcopy(self.img_info_dict) 


        self.img_info_dict['connected'] = True
        
        max_rate = copy.deepcopy(self.max_rate)
        if depth_map_topic in img_info_dict.keys() and max_rate > .01:
            enabled = self.enabled
            state = self.state
            active = img_info_dict['active']
            if active == True and enabled == True and state == 'Running':
                # Process ros image message
                img_info_dict['publishing'] = True


                # Check if time to publish
                delay_time = float(1) / max_rate 
                last_img_time = self.img_info_dict['last_img_time']
                current_time = nepi_utils.get_time()
                timer = round((current_time - last_img_time), 3)
                #self.msg_if.pub_warn("Delay and Timer: " + str(delay_time) + " " + str(timer))
                if timer > delay_time: 
                    self.img_info_dict['last_img_time'] = current_time

                    get_msg_stampstamp = depth_map_msg.header.stamp
                    ros_frame_id = depth_map_msg.header.frame_id

                    current_time = nepi_sdk.get_msg_stamp()
                    latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
                    img_info_dict['get_latency_time'] = latency

                    # Apply Colormap
                    cv2_depth_map = nepi_img.rosimg_to_cv2img(depth_map_msg, encoding="passthrough")
                    depth_data = (np.array(cv2_depth_map, dtype=np.float32)) # replace nan values
                    # Get range data
                    start_range_ratio = self.min_range_ratio
                    stop_range_ratio = self.max_range_ratio
                    min_range_m = self.min_range_m
                    max_range_m = self.max_range_m
                    delta_range_m = max_range_m - min_range_m
                    # Adjust range Limits if IDX Controls enabled and range ratios are not min/max
                    if (start_range_ratio > 0 or stop_range_ratio < 1):
                        max_range_m = min_range_m + stop_range_ratio * delta_range_m
                        min_range_m = min_range_m + start_range_ratio * delta_range_m
                        delta_range_m = max_range_m - min_range_m
                        # Filter depth_data in range
                        depth_data[np.isnan(depth_data)] = max_range_m 
                        depth_data[depth_data <= min_range_m] = max_range_m # set to max
                        depth_data[depth_data >= max_range_m] = max_range_m # set to max
                        # Create colored cv2 depth image
                        depth_data = depth_data - min_range_m # Shift down 
                        depth_data = np.abs(depth_data - max_range_m) # Reverse for colormap
                        depth_data = np.array(255*depth_data/delta_range_m,np.uint8) # Scale for bgr colormap
                        cv2_img = cv2.applyColorMap(depth_data, cv2.COLORMAP_JET)

                    # Publish Data
                    if self.img_image_if is not None:
                        self.img_image_if.publish_cv2_img(cv2_img, encoding = encoding, timestamp = timestamp, frame_id = frame_id, add_overlay_list = add_overlay_list)
                    
                    # Save Image Data if needed
                    data_product = self.data_product
                    self.save_data_if.save_img2file(data_product,cv2_img,timestamp)

                    current_time = nepi_sdk.get_msg_stamp()
                    latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
                    img_info_dict['pub_latency_time'] = latency                              





    def statusCb(self,msg):
        self.status_msg = msg
        self.enabled = self.status_msg.image_pub_enabled
        self.max_pub_rate = self.status_msg.max_image_pub_rate
        self.min_range_m = self.status_msg.min_range_m
        self.max_range_m = self.status_msg.max_range_m
        self.min_range_ratio = self.status_msg.range_ratios.start_range
        self.max_range_ratio = self.status_msg.range_ratios.stop_range
       
        

#########################################
# Main
#########################################
if __name__ == '__main__':
  DepthMapImgPub()





