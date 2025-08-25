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
from os.path import exists
import subprocess
import sys
import glob
import fileinput
import random

import logging
import declxml as xml

import ai_utils

##########################################
# SETUP - Edit as Necessary 
##########################################
MAX_NUM_IMAGES = -1 #100 # set to -1 to use all
VAL_DATA_PERCENTAGE = 10
TEST_DATA_PERCENTAGE = 10
MAKE_TRAIN_TEST_UNIQUE = True

TEST_DATA_PERCENTAGE = 20

data_folder_path = os.path.join(ai_utils.current_folder,'data_labeling')
train_folder_path = os.path.join(ai_utils.current_folder,'model_training')

classes_file_path = os.path.join(data_folder_path,'classes.txt')
train_file_path = os.path.join(train_folder_path,'train_data.txt')
val_file_path = os.path.join(train_folder_path,'val_data.txt')
test_file_path = os.path.join(train_folder_path,'test_data.txt')

custom_data_file_path = os.path.join(train_folder_path,'data_custom.yaml')
##########################################
# Methods
##########################################

def convert_xml_files(xml_dir):
  transformer = Transformer(xml_dir)
  transformer.transform()

def process_folder():

  exist_files = []
  train_files = []
  val_files = []
  test_files = []
  ulab_files = []

  ### Load existing files
  if os.path.exists(train_file_path) == True:
    train_files = ai_utils.read_list_from_file(train_file_path)
    exist_files = train_files
  if os.path.exists(val_file_path) == True:
    val_files = ai_utils.read_list_from_file(val_file_path)
    exist_files = exist_files + val_files
  if os.path.exists(test_file_path) == True:
    test_files = ai_utils.read_list_from_file(test_file_path)
    exist_files = exist_files + test_files
  ### Walk through folder folders
  print("Processing folders in: " + data_folder_path)
  folders_to_process=ai_utils.get_folder_list(data_folder_path)
  print('')
  print('Found folders: ' + str(folders_to_process))
  for folder in folders_to_process:
    print('Processing folder: ' + folder)
    path, dirs, files = next(os.walk(folder))
    data_size = len(files)
    ind = 0
    data_val_size = int(float(1)/float(VAL_DATA_PERCENTAGE) * data_size)
    val_indexes = random.sample(range(data_size), k=data_val_size)
    data_test_size = int(float(1)/float(TEST_DATA_PERCENTAGE) * data_size)
    test_indexes = random.sample(range(data_size), k=data_test_size)
    files = os.listdir(folder)
    #print("Found " + str(len(files)) + " files in folder")
    #print('Found image files: ' + str(files))
    for f in files:
      f_ext = os.path.splitext(f)[1]
      f_ext = f_ext.replace(".","")
      try:
        if f_ext in ai_utils.IMAGE_FILE_TYPES:
          image_file = (folder + '/' + f)
          #print('Found image file: ' + image_file)
          #print(image_file)
          label_file = (folder + '/' + f.split(f_ext)[0]+'txt')
          #print('Looking for label file: ' + label_file)
          if os.path.exists(label_file):
            #print('Found label file' + label_file)
            ind += 1
            if ind in val_indexes and image_file not in exist_files: ##### FINISH THIS
              #print('Adding image to val file list')
              val_files.append(image_file)
            elif ind in test_indexes and image_file not in exist_files: 
              #print('Adding image to test file list')
              test_files.append(image_file)
            elif image_file not in exist_files: 
              #print('Adding image to train file list')
              train_files.append(image_file)
          else:
            # print("Warning: No label file for image: " + image_file)
            ulab_files.append(image_file)
          max_num_images = len(files)
          if MAX_NUM_IMAGES > 0:
            max_num_images = MAX_NUM_IMAGES
          if ind > max_num_images:
            break
      except Exception as e:
        print("Excepton on file write: " + str(e))

  print("Found " + str(len(ulab_files)) + " unlabeled files")
  
  ### Create return data
  # data = ulab_files

  ### Create train/test data set file
  ai_utils.write_list_to_file(train_files, train_file_path)
  if os.path.exists(val_file_path) == False:
    ai_utils.write_list_to_file(val_files, val_file_path)
  if os.path.exists(test_file_path) == False:
    ai_utils.write_list_to_file(test_files, test_file_path)

  ### Create dictionary

  classes_list = ai_utils.read_list_from_file(classes_file_path)
  number_of_classes = len(classes_list)
  #print(test_files)
  #print(test_label_files)
  #print(train_files)
  #print(val_files)


  #data : dict[str, any] = {
  data = {
    'path' : train_folder_path,
    'train' : os.path.basename(train_file_path),
    'val' : os.path.basename(val_file_path),
    'test' : os.path.basename(test_file_path),
    'nc' : number_of_classes,
    'names' : classes_list
  }
  return data
            


def write_data_to_file( data):
  
  ### Create data
  success = ai_utils.write_dict_2_yaml(data, custom_data_file_path)
  return success



##########################################
# Classes
##########################################

class Transformer(object):
    def __init__(self, xml_dir):
        self.xml_dir = xml_dir
        self.out_dir = xml_dir
        self.class_file = (xml_dir + '/classes.txt')

    def transform(self):
        reader = Reader(xml_dir=self.xml_dir)
        xml_files = reader.get_xml_files()
        #print(xml_files)
        classes = reader.get_classes(self.class_file)
        #print(classes)
        object_mapper = ObjectMapper()
        annotations = object_mapper.bind_files(xml_files, xml_dir=self.xml_dir)
        self.write_to_txt(annotations, classes)

    def write_to_txt(self, annotations, classes):
        for annotation in annotations:
            output_path = os.path.join(self.out_dir, self.darknet_filename_format(annotation.filename))
            if not os.path.exists(os.path.dirname(output_path)):
                os.makedirs(os.path.dirname(output_path))
            with open(output_path, "w+") as f:
                f.write(self.to_darknet_format(annotation, classes))

    def to_darknet_format(self, annotation, classes):
        result = []
        for obj in annotation.objects:
            if obj.name not in classes:
                print("Please, add '%s' to classes.txt file." % obj.name)
                exit()
            x, y, width, height = self.get_object_params(obj, annotation.size)
            result.append("%d %.6f %.6f %.6f %.6f" % (classes[obj.name], x, y, width, height))
        return "\n".join(result)

    @staticmethod
    def get_object_params(obj, size):
        image_width = 1.0 * size.width
        image_height = 1.0 * size.height

        box = obj.box
        absolute_x = box.xmin + 0.5 * (box.xmax - box.xmin)
        absolute_y = box.ymin + 0.5 * (box.ymax - box.ymin)

        absolute_width = box.xmax - box.xmin
        absolute_height = box.ymax - box.ymin

        x = absolute_x / image_width
        y = absolute_y / image_height
        width = absolute_width / image_width
        height = absolute_height / image_height

        return x, y, width, height

    @staticmethod
    def darknet_filename_format(filename):
        pre, ext = os.path.splitext(filename)
        return "%s.txt" % pre


class Reader(object):
    def __init__(self, xml_dir):
        self.xml_dir = xml_dir

    def get_xml_files(self):
        xml_filenames = []
        for root, subdirectories, files in os.walk(self.xml_dir):
            for filename in files:
                if filename.endswith(".xml"):
                    file_path = os.path.join(root, filename)
                    file_path = os.path.relpath(file_path, start=self.xml_dir)
                    xml_filenames.append(file_path)    
        return xml_filenames

    @staticmethod
    def get_classes(filename):
        with open(os.path.join(os.path.dirname(os.path.realpath('__file__')), filename), "r", encoding="utf8") as f:
            lines = f.readlines()
            return {value: key for (key, value) in enumerate(list(map(lambda x: x.strip(), lines)))}

import logging
import os
import declxml as xml


class ObjectMapper(object):
    def __init__(self):
        self.processor = xml.user_object("annotation", Annotation, [
            xml.user_object("size", Size, [
                xml.integer("width"),
                xml.integer("height"),
            ]),
            xml.array(
                xml.user_object("object", Object, [
                    xml.string("name"),
                    xml.user_object("bndbox", Box, [
                        xml.integer("xmin"),
                        xml.integer("ymin"),
                        xml.integer("xmax"),
                        xml.integer("ymax"),
                    ], alias="box")
                ]),
                alias="objects"
            ),
            xml.string("filename")
        ])

    def bind(self, xml_file_path, xml_dir):
        ann = xml.parse_from_file(self.processor, xml_file_path=os.path.join(xml_dir, xml_file_path))
        ann.filename = xml_file_path
        return ann

    def bind_files(self, xml_file_paths, xml_dir):
        result = []
        for xml_file_path in xml_file_paths:
            try:
                result.append(self.bind(xml_file_path=xml_file_path, xml_dir=xml_dir))
            except Exception as e:
                logging.error("%s", e.args)
        return result


class Annotation(object):
    def __init__(self):
        self.size = None
        self.objects = None
        self.filename = None

    def __repr__(self):
        return "Annotation(size={}, object={}, filename={})".format(self.size, self.objects, self.filename)


class Size(object):
    def __init__(self):
        self.width = None
        self.height = None

    def __repr__(self):
        return "Size(width={}, height={})".format(self.width, self.height)


class Object(object):
    def __init__(self):
        self.name = None
        self.box = None

    def __repr__(self):
        return "Object(name={}, box={})".format(self.name, self.box)


class Box(object):
    def __init__(self):
        self.xmin = None
        self.ymin = None
        self.xmax = None
        self.ymax = None

    def __repr__(self):
        return "Box(xmin={}, ymin={}, xmax={}, ymax={})".format(self.xmin, self.ymin, self.xmax, self.ymax)

###############################################
# Main
###############################################

if __name__ == '__main__':

    ### (Create txt from xml labels)
    data_folder = os.path.join(ai_utils.current_folder,'data_labeling')
    ### Converting XML Label Files to TXT
    print('Searching for folders in: ' + data_folder)
    folders_to_process=ai_utils.get_folder_list(data_folder)
    print('')
    print('Found folders: ' + str(folders_to_process))
    for folder in folders_to_process:
        classes_file = (folder + '/classes.txt')
        if exists(classes_file):
            print('converting xml files in folder:')
            print(folder)
            convert_xml_files(folder)
        else:
            print('no classes file found in folder, skipping')

    ### Process Data (create update training files)
    data = process_folder()
    ### Write File
    success = write_data_to_file(data)
    ### Wrap Up
    print('')
    print('All done')