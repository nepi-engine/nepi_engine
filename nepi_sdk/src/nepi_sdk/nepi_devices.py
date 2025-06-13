#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI pointcloud utility functions include
# 1) Pointcloud conversion functions
# 2) Pointcloud filter functions
# 3) Pointcloud manipulation functions
# 4) Pointcloud rendering functions
# 5) Pointcloud saving functions


import numpy as np

import os


from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_pc"
logger = Logger(log_name = log_name)

VERBOSITY_LEVELS = ["Debug","Error","Info","Warning"]

def get_verbosity_level():
    return o3d.utility.get_verbosity_level()

def set_verbosity_level(level = "Error"):
    if level in VERBOSITY_LEVELS:
        eval("o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel." + level + ")")
    return get_verbosity_level()





###########################################
### IDX Utility Functions 

def get_idx_device_namespaces(name):
    msg_type = 'sensor_msgs/IDXStatus'
    s_topics = nepi_sdk.find_topics_by_msg(msg_type)
    topics = []
    for topic in s_topics:
    	topic = topic.replace('/status','')
    	topics.append(topic)
    return topics


###########################################
### PTX Utility Functions 

def get_ptx_device_namespaces(name):
    msg_type = 'sensor_msgs/PTXStatus'
    s_topics = nepi_sdk.find_topics_by_msg(msg_type)
    topics = []
    for topic in s_topics:
    	topic = topic.replace('/status','')
    	topics.append(topic)
    return topics
    
###########################################
### LSX Utility Functions 

def get_lsx_device_namespaces(name):
    msg_type = 'sensor_msgs/LSXStatus'
    s_topics = nepi_sdk.find_topics_by_msg(msg_type)
    topics = []
    for topic in s_topics:
    	topic = topic.replace('/status','')
    	topics.append(topic)
    return topics
    
###########################################
### NPX Utility Functions 

def get_npx_device_namespaces(name):
    msg_type = 'sensor_msgs/NPXStatus'
    s_topics = nepi_sdk.find_topics_by_msg(msg_type)
    topics = []
    for topic in s_topics:
    	topic = topic.replace('/status','')
    	topics.append(topic)
    return topics
    

###########################################
### RBX Utility Functions 

def get_rbx_device_namespaces(name):
    msg_type = 'sensor_msgs/RBXStatus'
    s_topics = nepi_sdk.find_topics_by_msg(msg_type)
    topics = []
    for topic in s_topics:
    	topic = topic.replace('/status','')
    	topics.append(topic)
    return topics
    
