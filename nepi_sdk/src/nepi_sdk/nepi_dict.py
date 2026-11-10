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


# NEPI System States utility functions 

  
import os
import time
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_data
from nepi_sdk import nepi_controls


from nepi_interfaces.msg import Value, ValuesDict, Topic, TopicsDict
from nepi_interfaces.msg import DataGroup, DataGroupsDict, ControlGroup, ControlGroupsDict


logger = nepi_sdk.sdk_logger


#####################################
### Values  Helper Functions
#####################################

def convert_values_dict2msg(input_dict):
    output_msg = ValuesDict()

    key_names = []
    entry_msgs = []
    for key_name in input_dict.keys():
            entry_values = []
            
            values = input_dict[key_name]
            is_list = isinstance(values,list)
            success = True
            if isinstance(values,list) == False:
                values = [values]

            if len(values) > 0:
                entry_type = {'str':'Strings','bool':'Bools','int':'Ints','float':'Floats'}.get(values[0].__class__.__name__,'')

                for value in values:
                    entry_value = None

                    if entry_type == 'Strings':
                        entry_value = str(value)

                    elif entry_type == 'Bools':
                        if isinstance(value, bool):
                            entry_value = str(value)

                    elif entry_type == 'Ints':
                        if isinstance(value, int):
                            entry_value = str(value)
                        elif isinstance(value, float):
                            entry_value = str(int(value))
                    elif entry_type == 'Floats':
                        if isinstance(value, float):
                            entry_value = str(value)

                    if entry_value is None:
                        success = False
                    else:
                        entry_values.append(entry_value)
            
            if success == True and len(entry_values) > 0:
                key_names.append(str(key_name))
                entry_msg = Value()
                entry_msg.name = str(key_name)
                entry_msg.display_name = str(key_name).replace('_',' ').capitalize()
                entry_msg.type = entry_type
                entry_msg.is_list = is_list
                entry_msg.values = entry_values
                entry_msgs.append(entry_msg)

    output_msg.key_names = key_names
    output_msg.entries = entry_msgs
    return output_msg




def convert_values_msg2dict(input_msg):
    output_dict = dict()
    key_names = input_msg.key_names
    for i, key_name in enumerate(key_names):
        entry_msg = input_msg.entries[i]
        entry_name = entry_msg.name
        entry_type = entry_msg.type
        is_list = entry_msg.is_list
        values = []
        success = True
        for entry_value in entry_msg.values:
            try:
                        value = None
                        if entry_type == 'Strings':
                            value = entry_value
                        elif entry_type == 'Bools':
                            value = entry_value == 'True' or entry_value == 'true'
                        elif entry_type == 'Ints':
                            value = int(float(entry_value))
                        elif entry_type == 'Floats':
                            value = float(entry_value)
                        if value is not None:
                            values.append(value)
            except:
                success = False

        if success == True:
            if len(values) == 1 and is_list == False:
                output_dict[key_name] = values[0]
            elif len(values) > 0:
                output_dict[key_name] = values
       
    return output_dict

#####################################
### Topics  Helper Functions
#####################################


def get_blank_topic_dict():
    return nepi_sdk.convert_msg2dict(Topic())



def convert_topic_dict2msg(input_dict, key_name = 'topic_dict'):
    name = str(input_dict.get('name', key_name))
    display_name = input_dict.get('display_name', None)
    if display_name is None:
        display_name = ''
    if display_name.strip() == '':
        display_name = name.replace('_',' ').capitalize()
    available_topics = input_dict.get('available_topics', [])
    available_topic_names = input_dict.get('available_topic_names', [])
    if len(available_topics) > 0 and len(available_topic_names) != len(available_topics):
        available_topic_names = nepi_sdk.get_display_names_from_namespaces(available_topics)
    topic = str(input_dict.get('topic',''))
    topic_name = input_dict.get('topic_name',None)
    if topic_name is None:
        topic_name = ''
    if topic_name.strip() == '':
        topic_name = nepi_sdk.get_display_name_from_namespace(topic)

    output_msg = Topic()
    output_msg.name = name
    output_msg.display_name = display_name

    output_msg.status_msg_type = str(input_dict.get('status_msg_type',''))
    output_msg.data_msg_type = str(input_dict.get('data_msg_type',''))

    output_msg.type = str(input_dict.get('type',''))
    output_msg.sub_type = str(input_dict.get('sub_type',''))
    output_msg.group = str(input_dict.get('group',''))
    output_msg.sub_group = str(input_dict.get('sub_group',''))


    output_msg.available_topics = available_topics
    output_msg.available_topic_names = available_topic_names
    output_msg.topic = topic
    output_msg.topic_name = topic_name
    
    output_msg.connecting = input_dict.get('connecting',False)
    output_msg.connected = input_dict.get('connected',False)

    return output_msg

def convert_topics_dict2msg(input_dict):
    output_msg = TopicsDict()

    key_names = []
    entry_msgs = []
    for key_name in input_dict.keys():
            key_names.append(str(key_name))
            entry_msgs.append(convert_topic_dict2msg(input_dict[key_name], key_name))

    output_msg.key_names = key_names
    output_msg.entries = entry_msgs

    return output_msg


def convert_topic_msg2dict(input_msg):
    output_dict = nepi_sdk.convert_msg2dict(input_msg)        
    return output_dict



def convert_topics_msg2dict(input_msg):
    output_dict = dict()

    key_names = input_msg.key_names
    for i, key_name in enumerate(key_names):
        output_dict[key_name] = convert_topic_msg2dict(input_msg.entries[i])
        
    return output_dict



#####################################
### Data Group  Helper Functions
#####################################

def convert_data_group_dict2msg(input_dict, key_name = 'topic_dict'):
    name = str(input_dict.get('name', key_name))

    #####
    display_name = input_dict.get('display_name', None)
    if display_name is None:
        display_name = ''
    if display_name.strip() == '':
        display_name = name.replace('_',' ').capitalize()

    #####
    namespace = input_dict.get('namespace', None)
    if namespace is None:
        try:
            namespace = nepi_sdk.get_node_namespace() + '/' + name
        except:
            namespace = ''
    
    ###########################
    output_msg = DataGroup()
    output_msg.name = name 
    output_msg.display_name = display_name
    output_msg.namespace = namespace


    ### Data
    output_msg.data_topic = str(input_dict.get('data_topic',''))
    data_dict = input_dict.get('data_dict',dict())
    if data_dict is not None:
        try:
            output_msg.data_msg = nepi_data.convert_data_dict2msg(data_dict)
        except:
            pass

    ### Config
    config_dict = input_dict.get('config_dict', None)
    if config_dict is not None:
        try:
            output_msg.config_dict = convert_values_dict2msg(config_dict)
        except:
            pass

    return output_msg




def convert_data_group_msg2dict(input_msg):
    output_dict = dict()
    output_dict['name'] = input_msg.name
    output_dict['display_name'] = input_msg.display_name
    output_dict['namespace'] = input_msg.namespace

    output_dict['data_topic'] = input_msg.data_topic
    output_dict['data_dict'] = nepi_data.convert_data_msg2dict(input_msg.data_msg)

    output_dict['config_dict'] = convert_values_msg2dict(input_msg.config_dict)
    return output_dict


def convert_data_groups_dict2msg(input_dict):
    output_msg = DataGroupsDict()

    key_names = []
    entry_msgs = []
    
    for key_name in input_dict.keys():
            key_names.append(str(key_name))
            entry_msg = convert_data_group_dict2msg(input_dict[key_name], key_name)
            entry_msgs.append(entry_msg)

    output_msg.key_names = key_names
    output_msg.entries = entry_msgs

    return output_msg




def convert_data_groups_msg2dict(input_msg):
    output_dict = dict()

    key_names = input_msg.key_names
    for i, key_name in enumerate(key_names):
            output_dict[key_name] = convert_data_group_msg2dict(input_msg.entries[i])
    
    return output_dict





#####################################
### Control Group  Helper Functions
#####################################

def convert_control_group_dict2msg(input_dict, key_name = 'topic_dict'):
    name = str(input_dict.get('name', key_name))

    #####
    display_name = input_dict.get('display_name', None)
    if display_name is None:
        display_name = ''
    if display_name.strip() == '':
        display_name = name.replace('_',' ').capitalize()

    #####
    namespace = input_dict.get('namespace', None)
    if namespace is None:
        try:
            namespace = nepi_sdk.get_node_namespace() + '/' + name
        except:
            namespace = ''
    
    ###########################
    output_msg = ControlGroup()
    output_msg.name = name 
    output_msg.display_name = display_name
    output_msg.namespace = namespace

    ### Controls
    output_msg.controls_topic = str(input_dict.get('controls_topic',''))
    controls_dict = input_dict.get('controls_dict',dict())
    if controls_dict is not None:
        try:
            output_msg.controls_msg = nepi_controls.convert_controls_dict2msg(controls_dict)
        except:
            pass

    ### Data
    output_msg.data_topic = str(input_dict.get('data_topic',''))
    data_dict = input_dict.get('data_dict',dict())
    if data_dict is not None:
        try:
            output_msg.data_msg = nepi_data.convert_data_dict2msg(data_dict)
        except:
            pass

    ### Config
    config_dict = input_dict.get('config_dict', None)
    if config_dict is not None:
        try:
            output_msg.config_dict = convert_values_dict2msg(config_dict)
        except:
            pass

    return output_msg




def convert_control_group_msg2dict(input_msg):
    output_dict = dict()
    output_dict['name'] = input_msg.name
    output_dict['display_name'] = input_msg.display_name
    output_dict['namespace'] = input_msg.namespace

    output_dict['controls_topic'] = input_msg.controls_topic
    output_dict['controls_dict'] = nepi_controls.convert_controls_msg2dict(input_msg.controls_msg)

    output_dict['data_topic'] = input_msg.data_topic
    output_dict['data_dict'] = nepi_data.convert_data_msg2dict(input_msg.data_msg)

    output_dict['config_dict'] = convert_values_msg2dict(input_msg.config_dict)
    return output_dict


def convert_control_groups_dict2msg(input_dict):
    output_msg = ControlGroupsDict()

    key_names = []
    entry_msgs = []
    
    for key_name in input_dict.keys():
            key_names.append(str(key_name))
            entry_msg = convert_control_group_dict2msg(input_dict[key_name], key_name)
            entry_msgs.append(entry_msg)

    output_msg.key_names = key_names
    output_msg.entries = entry_msgs

    return output_msg




def convert_control_groups_msg2dict(input_msg):
    output_dict = dict()

    key_names = input_msg.key_names
    for i, key_name in enumerate(key_names):
            output_dict[key_name] = convert_control_group_msg2dict(input_msg.entries[i])
    
    return output_dict