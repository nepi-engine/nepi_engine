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

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_sdk import nepi_sdk

from nepi_api.sys_if_msg import MsgIF

class MMapIF(object):

    #######################
    ### IF Initialization
    def __init__(self, log_name = None):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.node_name = nepi_sdk.get_node_name()
        self.base_namespace = nepi_sdk.get_base_namespace()

        ############################## 
        self.msg_if = MsgIF(log_name = self.class_name)
        nepi_sdk.sleep(1)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        ##############################   

        ##############################
        self.msg_if.pub_info("IF Initialization Complete")

''' Future Function
# Try and save cv2_img to memory map]
img_topic = os.path.join(self.base_namespace, self.node_name,'~idx/',data_product)
mmap_id = nepi_mmap.get_mmap_id_from_topic(img_topic)
if mmap_id not in self.mmap_dict.keys():
    [success,msg] = nepi_mmap.create_cv2img_mmap(mmap_id, cv2_img,img_encoding = encoding)
    self.mmap_dict[mmap_id] = dict()
    self.mmap_dict[mmap_id]['id'] = mmap_id
    if success == True:
        self.mmap_dict[mmap_id]['valid'] = True
        self.msg_if.pub_info("Created mmap for topic: " + img_topic + " mmap_id: " + mmap_id)
    else:
        self.mmap_dict[mmap_id]['valid'] = False
        self.msg_if.pub_warn("Failed to create mmap for topic: " + img_topic + " mmap_id: " + mmap_id + " with msg " + msg)
elif self.mmap_dict[mmap_id]['valid'] == True:
    locked = nepi_mmap.check_lock_mmap(mmap_id)
    if locked == False:
        locked = nepi_mmap.lock_mmap(mmap_id)
        if locked == True:
            [success,msg] = nepi_mmap.write_cv2img_mmap_data(mmap_id, cv2_img, encoding = encoding, get_msg_stampstamp = get_msg_stampstamp)
            unlocked = nepi_mmap.unlock_mmap(mmap_id)
            if success == False:
                #self.msg_if.pub_warn("Failed to save mmap for topic: " + img_topic + " mmap_id: " + mmap_id + " with msg " + msg)
                self.mmap_dict[mmap_id]['valid'] = False
'''


'''
# Check if topic has memory mapping
has_mmap = False
if self.img_info_dict['has_mmap'] is None:
    mmap_id = nepi_mmap.get_mmap_id_from_topic(img_topic)
    mmap_exists = nepi_mmap.check_for_mmap(mmap_id)
    if mmap_exists == True:
        [success, msg, info_dict]  = nepi_mmap.get_cv2img_mmap_info(mmap_id)
        if info_dict is not None:
            has_mmap = True
            self.img_info_dict['has_mmap'] = True
            self.img_info_dict['mmap_id'] = mmap_id
            self.img_info_dict['mmap_info_dict'] = info_dict
else:
    has_mmap = self.img_info_dict['has_mmap']
if has_mmap == True:
    # Start get mmap image thread
    pass
else:
'''



'''
def imageMmapThread(self,img_topic, mmap_id):    
    mmap_list = nepi_mmap.get_mmap_list()
    while img_topic in self.imgs_pub_sub_dict.keys() and mmap_id in mmap_list and not nepi_sdk.is_shutdown():
        current_time = nepi_sdk.get_time_now()
        mmap_list = nepi_mmap.get_mmap_list()

        start_time = nepi_sdk.get_time()   

        get_image = (self.get_img == True or self.throttle_image == False)
        #self.msg_if.pub_warn(":" + self.log_name + ": Callback got image from topic:  " + img_topic + " with get topic " + str(self.get_img))
        if get_image == True:
            self.get_img = False

            if mmap_id in mmap_list:
                # Get cv2 data
                unlocked = nepi_mmap.wait_for_unlock_mmap(mmap_id, timeout = 1)
                if unlocked == True:
                    locked = nepi_mmap.lock_mmap(mmap_id)
                    if locked == True:
                        mmap_read_response = nepi_mmap.read_cv2img_mmap_data(mmap_id)
                        [success, msg, cv2_img, img_encoding, timestamp, latency_sec] = mmap_read_response
                        if success:
                            latency = current_time - timestamp
                            self.img_info_dict['get_latency_time'] = latency
                            #self.msg_if.pub_info(":" + self.log_name + ": Detect Pub Latency: {:.2f}".format(latency))



                            ### Preprocess Image
                            
                            cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

                            if self.imagePreprocessFunction is not None:
                                try:
                                    cv2_img = self.imagePreprocessFunction(cv2_img)
                                except Exception as e:
                                    self.msg_if.pub_warn(":" + self.log_name + ": Provided Image Preprocess Function failed:  " + str(e))

                            cv2_img = self.imagePreprocessFunction(cv2_img)
                            img_dict = dict()
                            img_dict['cv2_img'] = cv2_img
                            height, width = cv2_img.shape[:2]
                            img_dict['width'] = width 
                            img_dict['height'] = height 

                            ros_header = Header()
                            ros_header.stamp = nepi_sdk.msg_stamp_from_sec(timestamp)
                            img_dict = self.imagePreprocessFunction(cv2_img, options_dict)
                            img_dict['timestamp'] = timestamp
                            img_dict['ros_img_topic'] = img_topic
                            img_dict['ros_img_header'] = ros_header
                            img_dict['ros_img_stamp'] = ros_header.stamp
                            ##############################

                            self.img_dict_lock.acquire()
                            self.img_dict = img_dict
                            self.img_dict_lock.release()
                            self.got_img = True

                            process_time = round( (nepi_sdk.get_time() - start_time) , 3)
                            self.img_info_dict['process_time'] = process_time

                            unlocked = nepi_mmap.unlock_mmap(mmap_id)
                        else:
                            self.img_info_dict['has_mmap'] = True
                            break
                    else:
                    time.sleep(.01)
  '''