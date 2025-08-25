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
import numpy as np
import sys
import declxml as xml
import xml.etree.ElementTree as ET

from nepi_sdk import nepi_utils

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_ais"
logger = Logger(log_name = log_name)


########################
## Library Data


IMAGE_FILE_TYPES = ['jpg','JPG','jpeg','png','PNG']


EXAMPLE_AI_MGR_STATUS_DICT = {
    'ai_frameworks': [],

    'ai_models': [],
    'ai_models_frameworks': [],
    'ai_models_types': [],

    'active_ai_framework': 'framework_name',

    'active_ai_models': [],
    'active_ai_models_frameworks': [],
    'active_ai_models_types': [],
    'active_ai_models_nodes': [],
    'active_ai_models_namespaces': [],

    'all_namespace': '/base_namespace/ai/all'
}



EXAMPLE_BOXES_INFO_DICT_ENTRY = {
    'model_name': 'test_model',
    'image_header': Header(),
    'image_topic': '/test_topic',
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


########################
## Misc AI Helper Functions


'''
def get_classes_colors_list(classes_str_list):
    rgb_list = []
    if len(classes_str_list) > 0:
        cmap = plt.get_cmap('viridis')
        color_list = cmap(np.linspace(0, 1, len(classes_str_list))).tolist()
        for color in color_list:
            rgb = []
            for i in range(3):
                rgb.append(int(color[i]*255))
            rgb_list.append(rgb)
    return rgb_list
'''



def get_boxes_info_from_msg(bboxes_msg):
    boxes_info_dict  = {
        'model_name': bboxes_msg.model_name ,
        'image_header': bboxes_msg.image_header ,
        'image_topic': bboxes_msg.image_topic ,
        'image_height': bboxes_msg.image_height ,
        'image_width': bboxes_msg.image_width ,
        'prc_height': bboxes_msg.prc_height ,
        'prc_width': bboxes_msg.prc_width ,
    }
    return boxes_info_dict


def get_boxes_list_from_msg(bboxes_msg):
    bboxes_list = bboxes_msg.bounding_boxes
    boxes_list = []
    for bbox in bboxes_list:
        box_dict = {
            'name': bbox.Class ,
            'id': bbox.id ,
            'uid': bbox.uid ,
            'prob': bbox.probability ,
            'xmin': bbox.xmin ,
            'ymin': bbox.ymin ,
            'xmax': bbox.xmax ,
            'ymax': bbox.ymax ,
            'area_ratio': bbox.area_ratio ,
            'area_pixels': bbox.area_pixels
        }
        boxes_list.append(box_dict)
    return boxes_list



def scale_boxes_list(boxes_list, width_scaler, height_scaler):
    sboxes_list = []
    for box in boxes_list:
        box['xmin'] = box['xmin'] * width_scaler
        box['ymin'] = box['ymin'] * height_scaler
        box['xmax'] = box['xmax'] * width_scaler
        box['ymax'] = box['ymax'] * height_scaler
        box['area_pixels'] = box['area_pixels'] * width_scaler * height_scaler
        boxes_list.append(box)
    return sboxes_list


###############################
### AI label file functions


def get_num_files(folder_path):
    img_files = []
    xml_files = []
    txt_files = []
    if os.path.exists(folder_path) == False:
        print('Get stats folder not found: ' + folder_path)
    else:
        path, dirs, files = next(os.walk(folder_path))
        for file in files:
            f_ext = os.path.splitext(file)[1]
            f_ext = f_ext.replace(".","")
            if f_ext in IMAGE_FILE_TYPES:
                img_files.append(file)
            if f_ext == 'xml':
                xml_files.append(file)
            if f_ext == 'txt':
                txt_files.append(file)
    return img_files,xml_files,txt_files


def read_detect_dict_from_xml_file(file_path,classes = []):
    detect_dict_list = []
    f_ext = os.path.splitext(file_path)[1]
    if f_ext == '.xml':    
        tree = ET.parse(file_path)
        root = tree.getroot()
        labels = []
        bboxes = []
        size = root.find("size")
        image_width = 1.0 * int(size.find("width").text)
        image_height = 1.0 * int(size.find("height").text)
        for ind, o_entry in enumerate(root.findall("object")):
            try:
                label = o_entry.find("name").text
                labels.append(label)
                if label not in classes:
                    class_ind = -1
                else:
                    class_ind = classes.index(label)
                    
                box = o_entry.find("bndbox")
                xmax = int(box.find("xmax").text)
                xmin = int(box.find("xmin").text)
                ymax = int(box.find("ymax").text)
                ymin = int(box.find("ymin").text)

                absolute_x = xmin + 0.5 * (xmax - xmin)
                absolute_y = ymin + 0.5 * (ymax - ymin)

                absolute_width = xmax - xmin
                absolute_height = ymax - ymin

                x = absolute_x / image_width
                y = absolute_y / image_height
                width = absolute_width / image_width
                height = absolute_height / image_height

                det_area = (xmax - xmin) * (ymax - ymin)
                det_prob = 1
                detect_dict = {
                    'name': label, # Class String Name
                    'id': class_ind, # Class Index from Classes List
                    'uid': '', # Reserved for unique tracking by downstream applications
                    'prob': det_prob, # Probability of detection
                    'xmin': xmin,
                    'ymin': ymin,
                    'xmax': xmax,
                    'ymax': ymax,
                    'area_pixels': int(det_area),
                    'area_ratio': det_area / (image_width * image_height)
                }
                detect_dict_list.append(detect_dict)
            except Exception as e:
                print("Failed to convert xml data to detect data: " + str(e))

    return detect_dict_list




def update_xml_label_file(file_path,classes,classes_dict):
    success = False
    f_ext = os.path.splitext(file_path)[1]
    if f_ext == '.xml':    
        tree_orig = ET.parse(file_path)
        tree = ET.parse(file_path)
        root = tree.getroot()
        for ind, o_entry in enumerate(root.findall("object")):
            label = o_entry.find("name").text
            if label in classes_dict.keys():
                index = classes_dict[label]
                if index != -1:
                    o_entry.find("name").text = classes[index]
                    label = o_entry.find("name").text
                    #print(label)
                else:
                    root.remove(o_entry)
            else:
                print('No match for label: ' + label)
        try:
            orig_file = file_path+'.org'
            copy_file(file_path,orig_file)
            #print('Saved original annotation labels: ' + orig_file)
            success = True
        except Exception as e:
            print('Failed to copy labels to file: ' + orig_file)
        try:
            tree.write(file_path)
            #print('Updated annotation labels in file: ' + file_path)
            success = True
        except Exception as e:
            print('Failed to save annotation labels to file: ' + file_path)
    return success




def read_detect_dict_from_txt_file(file_path,image_width,image_height,classes_list = []):
    dets_list = []
    f_ext = os.path.splitext(file_path)[1]
    if f_ext == '.txt':    
        with open(file_path,'r') as f:
            reader = csv.reader(f, delimiter=' ') 
        for row in reader:
            if len(row) == 5:
                try:
                    dets_int = []
                    for i, entry in enumerate(row):
                        dets_int.append(float(row[i]))
                        dets_list.append(dets_int)
                except Exception as e:
                    print('Failed top convert det strings to floats: ' + str(e))
            
        detect_dict_list = []
        for det in dets_list: 

            try:
                class_id = int(det[0])       
                if len(classes_list) > (class_id):
                    class_name = classes_list[class_id]
                else:
                    class_name = 'class_' + str(id)
                    det_name = class_name
                    det_id = class_id
                    det_prob = 1
                    xwidth = det[3]/2.0
                    xmin = int(image_width * (det[1] - xwidth))
                    xmax = int(image_width * (det[1] + xwidth))
                    yheight = det[4]/2.0
                    ymin = int(image_height * (det[2] - yheight))
                    ymax = int(image_height * (det[2] + yheight))
                    det_area = (xmax - xmin) * (ymax - ymin)
                    detect_dict = {
                        'name': det_name, # Class String Name
                        'id': det_id, # Class Index from Classes List
                        'uid': '', # Reserved for unique tracking by downstream applications
                        'prob': det_prob, # Probability of detection
                        'xmin': xmin,
                        'ymin': ymin,
                        'xmax': xmax,
                        'ymax': ymax,
                        'area_pixels': int(det_area),
                        'area_ratio': det_area / (image_width * image_height)
                    }
                    detect_dict_list.append(detect_dict)
                    #self.msg_if.pub_info("Got detect dict entry: " + str(detect_dict))
            except Exception as e:
                print("Failed to convert txt data to detect data: " + str(e))
    return detect_dict_list


def save_txt_label_file(bounding_boxes,file_path):
    success = False
    lines = []
    for box in bounding_boxes:
        lines.append("%d %.6f %.6f %.6f %.6f" % (box[0],box[1],box[2],box[3],box[4]))
    success = nepi_utils.write_list_to_file(lines,file_path)
    return success


def convert_dict_to_txt(detect_dict_list,image_width,image_height):
    labels = []
    bboxes = []
    for detect_dict in detect_dict_list:
        try:
            class_ind = detect_dict['id']
                
            xmax = detect_dict['xmax']
            xmin = detect_dict['xmin']
            ymax = detect_dict['ymax']
            ymin = detect_dict['ymin']

            absolute_x = xmin + 0.5 * (xmax - xmin)
            absolute_y = ymin + 0.5 * (ymax - ymin)

            absolute_width = xmax - xmin
            absolute_height = ymax - ymin

            x = absolute_x / image_width
            y = absolute_y / image_height
            width = absolute_width / image_width
            height = absolute_height / image_height

            bbox = [class_ind, x, y, width, height]                


            labels.append(detect_dict['name'])
            bboxes.append(bbox)
        except Exception as e:
            print("Failed to convert xml data to txt data: " + str(e))
    return labels, bboxes


def convert_xml_to_txt(folder_path, classes, classes_dict):
    files = nepi_utils.get_file_list(folder_path, ext_list = ['xml'])
    for file in files:
        [detect_dict_list, image_width, image_height] = read_detect_dict_from_xml_file(file,classes)
        [labels, bboxes] = convert_dict_to_txt(detect_dict_list,image_width,image_height)
        broken_labels = [i for i, box in enumerate(bboxes) if box[0] == -1]
        if len(broken_labels) > 0:
            [new_classes, new_classes_dict] = fix_brocken_labels(labels,classes,classes_dict)
            classes = new_classes
            classes_dict = new_classes_dict
            new_bboxes = []
            for i, label in enumerate(labels):
                if label in classes_dict.keys():
                    ind = classes_dict[label]
                    if ind != -1:
                        bbox = bboxes[i]
                        bbox[0] = ind
                        new_bboxes.append(bbox)
            bboxes = new_bboxes
            success = update_xml_label_file(file,classes,classes_dict)
        txt_file = file.replace('.xml','.txt')
        success = save_txt_label_file(bboxes,txt_file)
    return classes,classes_dict



def update_label_files(classes, classes_file, label_folder):
    success = True
    labeled_images = 0
    unlabled_images = 0
    unlabled_label_files = []

    ## Update Data Labeling Folder
    new_classes = classes
    orig_classes = classes
    if os.path.exists(classes_file) == False:
        orig_classes = classes
        nepi_utils.write_list_to_file(classes,classes_file)
    else:
        orig_classes = read_list_from_file(classes_file)
        if orig_classes != classes:
            print('Updated classes.txt labels from ' + str(orig_classes) + ' to ' + str(classes))
            nepi_utils.write_list_to_file(classes,classes_file)
    
    folders_to_process=get_folder_list(label_folder)
    classes_dict = dict()
    for i, label in enumerate(classes):
        classes_dict[label] = i
    for folder in folders_to_process:       
        print('Preparing txt label files in: ' + str(folder))
        labels_changed = orig_classes != new_classes
        has_labels = False
        for f in os.listdir(folder):
            if f.endswith(".xml"):  
                if has_labels == False:
                    print('')
                    print('**************************')
                    print('Updating xml label files in folder : ' + str(folder))
                has_labels = True
            if has_labels == True:       
                [convert_classes,classes_dict] = convert_xml_to_txt(label_folder,new_classes,classes_dict)
                if convert_classes != new_classes:
                    print('Saving classes file to: ' + str(classes_file))
                    nepi_utils.write_list_to_file(convert_classes, classes_file)
                    new_classes = convert_classes
        folder_classes_file = os.path.join(folder,CLASSES_FILE_NAME)
        print('Saving classes file to: ' + str(folder_classes_file))
        nepi_utils.write_list_to_file(new_classes, folder_classes_file)
    return new_classes


def read_detect_list_from_file(image_file, width = None, height = None, classes_list = []):
    detect_dict_list = []
    success = False
    label_file_path = image_file.rsplit('.', 1)[0] + '.yaml'
    if os.path.exists(label_file_path):
            detect_dict_list = nepi_utils.read_dict_from_file(label_file_path)
            success = True
    if success == False:
        label_file_path = image_file.rsplit('.', 1)[0] + '.xml'
        if os.path.exists(label_file_path):
            detect_dict_list = read_detect_dict_from_xml_file(label_file_path,classes_list)
            success = True
    if success == False:
        label_file_path = image_file.rsplit('.', 1)[0] + '.txt'
        if os.path.exists(label_file_path) and width is not None and height is not None:
            detect_dict_list = read_detect_dict_from_txt_file(label_file_path,width,height,classes_list)
            success = True
    return detect_dict_list