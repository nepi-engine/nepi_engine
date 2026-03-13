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


# NEPI ros utility functions include
# 1) NEPI IDX AI utility functions

import os
import sys
import importlib
import glob
import yaml

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
  
from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_aifs"
logger = Logger(log_name = log_name)
 
#***************************
# NEPI AI Framework Utility Functions
#***************************

##################
# Framework Functions
##################

def getAIFsDict(params_path, api_path):
    aifs_dict = dict()
    # Find AIF files
    ind = 0
    if os.path.exists(params_path):
        params_path = nepi_utils.clear_end_slash(params_path)
        if os.path.exists(api_path):
          api_path = nepi_utils.clear_end_slash(api_path)
          sys.path.append(params_path)
          logger.log_debug("Searching for AIFs in path: " + params_path, throttle_s = 5.0)
          for f in os.listdir(params_path):
            if f.endswith(".yaml") and f.find("params") != -1: 
              try:
                file_path = os.path.join(params_path,f)
                new_dict = nepi_utils.read_yaml_2_dict(file_path)
                framework_name = new_dict['framework_name']
                #logger.log_warn("Got ais dict: " + str(new_dict))
                new_dict['api_path'] = api_path
                new_dict['active'] = False
                aifs_dict[framework_name] = new_dict
              except Exception as e:
                logger.log_warn("Failed to import param file: " + f + " " + str(e))
        else:
              logger.log_warn("AIF api path does not exist: " +  api_path)
          # Check for node file   
    else:
        logger.log_warn("AIF params search path does not exist: " +  params_path)
    # Check for node file   

    purge_list = []
    for aif_name in aifs_dict.keys():
      purge = False
      pkg_name = aifs_dict[aif_name]['pkg_name']
      if_file = aifs_dict[aif_name]['if_file_name']
      if_file_path = os.path.join(api_path,if_file)
      if os.path.exists(if_file_path) == False:
        logger.log_warn("Could not find ai file: " + if_file_path)
        purge = True
      if purge == True:
        purge_list.append(aif_name)
    for aif_name in purge_list:
      del aifs_dict[aif_name]
    return aifs_dict




  

def getAIFsSortedList(aifs_dict):
  aifs_names = list(aifs_dict.keys())
  sorted_names = sorted(aifs_names)
  sorted_list = []
  for name in sorted_names:
    sorted_list.append(str(name))
  return sorted_names



def getModelsSortedList(models_dict):
  sorted_models = []
  model_names = []
  for model in models_dict.keys():
    if 'display_name' in models_dict[model].keys():
      model_names.append(models_dict[model]['display_name'])
    else:
      models_dict['display_name'] = model
      model_names.append(model)
  sorted_model_names = sorted(model_names)
  sorted_models = []
  for model_name in sorted_model_names:
    for model in models_dict.keys():
      if model_name == models_dict[model]['display_name']:
        sorted_models.append(model)
        break

  return sorted_models


def importAIFClass(file_name,file_path,module_name,class_name):
      module_class = None
      success = False
      msg = "failed"
      file_path = os.path.join(file_path,file_name)
      if os.path.exists(file_path):
        sys.path.append(file_path)
        try:
          module = importlib.import_module(module_name)
          try:
            module_class = getattr(module, class_name)
            success = True
            msg = 'success'
          except Exception as e:
            logger.log_warn("Failed to import class with exception: " + class_name + " " + module_name + " " + str(e))
        except Exception as e:
            logger.log_warn("Failed to import module with exception: "  + module_name + " " + str(e))
      else:
        logger.log_warn("Failed to find file in path for module: " + file_name + " " + file_path + " " + module_name)
      return success, msg, module_class



def unimportAIFClass(module_name):
    success = True
    if module_name in sys.modules:
        try:
           sys.modules.pop(module_name)
        except:
            logger.log_info("Failed to clordered_unimport module: " + module_name)
        if module_name in sys.modules:
          success = False
    return success

        
##################
# Framework Model Functions
##################


def loadModelsDict(framework_name, pkg_name, models_folder_path):
    models_dict = dict()
    if os.path.exists(models_folder_path) == False:
        logger.log_warn("Failed to find models folder: " + models_folder_path)
        return models_dict
    else:
        cfg_files = glob.glob(os.path.join(models_folder_path,'*.yaml'))

        # Remove the ros.yaml file -- that one doesn't represent a selectable trained neural net
        for f in cfg_files:
            cfg_dict = dict()
            success = False
            try:
                logger.log_warn("Opening yaml file: " + f) 
                yaml_stream = open(f, 'r')
                success = True
                  #logger.log_warn("Opened yaml file: " + f) 
            except Exception as e:
                logger.log_warn("Failed to open yaml file: " + str(e))
            if success:
                try:
                    # Validate that it is a proper config file and gather weights file size info for load-time estimates
                    logger.log_warn("Loading yaml data from file: " + f) 
                    cfg_dict = yaml.load(yaml_stream, Loader=yaml.FullLoader)
                    model_keys = list(cfg_dict.keys())
                    model_key = model_keys[0]
                      #logger.log_warn("Loaded yaml data from file: " + f) 
                except Exception as e:
                    logger.log_warn("Failed load yaml data: " + str(e)) 
                    success = False 
            try: 
                  #logger.log_warn("Closing yaml data stream for file: " + f) 
                yaml_stream.close()
            except Exception as e:
                logger.log_warn("Failed close yaml file: " + str(e))
            
            if success == False:
                logger.log_warn("File does not appear to be a valid A/I model config file: " + f + "... not adding this model")
                continue
              #logger.log_warn("Import success: " + str(success) + " with cfg_dict " + str(cfg_dict))
            cfg_dict_keys = cfg_dict[model_key].keys()
            logger.log_warn("Imported model key names: " + str(cfg_dict_keys))
            if ("framework" not in cfg_dict_keys):
                logger.log_warn("Framework does not specified in model yaml file: " + f + "... not adding this model")
                continue
            if ("weight_file" not in cfg_dict_keys):
                logger.log_warn("File does not appear to be a valid A/I model config file: " + f + "... not adding this model")
                continue
            if ("image_size" not in cfg_dict_keys):
                logger.log_warn("File does not specify a image size: " + f + "... not adding this model")
                continue
            if ("classes" not in cfg_dict_keys):
                logger.log_warn("File does not specify a classes: " + f + "... not adding this model")
                continue

            param_file = os.path.basename(f)
            framework = cfg_dict[model_key]["framework"]["name"]
            model_name = os.path.splitext(param_file)[0]

            if framework != framework_name:
                logger.log_warn("Model " + model_name + " not a MODEL_FRAMEWORK model" + framework + "... not adding this model")
                continue


            weight_file = cfg_dict[model_key]["weight_file"]["name"]
            weight_file_path = os.path.join(models_folder_path,weight_file)
            logger.log_warn("Checking that model weights file exists: " + weight_file_path + " for model name " + model_name)
            if not os.path.exists(weight_file_path):
                logger.log_warn("Model " + model_name + " specifies non-existent weights file " + weight_file_path + "... not adding this model")
                continue
            model_type = cfg_dict[model_key]['type']['name']

            model_size_mb = float(os.path.getsize(weight_file_path) / 1000000)

            display_name = model_name
            if 'display_name' in cfg_dict_keys:
                display_name = cfg_dict[model_key]['display_name']['name']

            node_name = display_name
            if 'node_name' in cfg_dict_keys:
                node_name = cfg_dict[model_key]['node_name']['name']


            model_dict = dict()
            model_dict['model_name'] = model_name
            model_dict['node_name'] = node_name
            model_dict['param_file'] = param_file
            model_dict['framework'] = framework
            model_dict['display_name'] = display_name
            model_dict['path'] = models_folder_path
            model_dict['type'] = model_type
            model_dict['description'] = cfg_dict[model_key]['description']['name']
            model_dict['pkg_name'] = pkg_name
            model_dict['img_height'] = cfg_dict[model_key]['image_size']['image_height']['value']
            model_dict['img_width'] = cfg_dict[model_key]['image_size']['image_width']['value']
            model_dict['classes'] = cfg_dict[model_key]['classes']['names']
            model_dict['weight_file']= weight_file
            model_dict['size'] = model_size_mb
            logger.log_info("Model dict created for model : " + model_name)
            models_dict[model_name] = model_dict
    return models_dict
      
   

def launchModelNode(model_dict, node_file_dict, launch_namespace, node_dict):
        success = False
        node_namespace = None
        try:
            model_name = model_dict['model_name']
            model_type = model_dict['type']

            if model_type not in node_file_dict.keys():
                logger.log_warn("Model " + str(model_name) + " specifies non-supported model type " + str(model_type) + "... not adding this model")
                return success, node_namespace, node_dict
            else:
                node_file_name = node_file_dict[model_type]
            logger.log_warn("Starting launch process for model " + str(model_name) + " type: " + str(model_type) + " node_file: " + str(node_file_name))

            node_name = model_dict['node_name']
            node_namespace = os.path.join(launch_namespace,node_name)
            pkg_name = model_dict['pkg_name']
            node_file_folder = os.path.join("/opt/nepi/nepi_engine/lib",pkg_name)
            
            param_file_path = os.path.join(model_dict['path'],model_dict['param_file'])
            weight_file_path = os.path.join(model_dict['path'],model_dict['weight_file'])

            logger.log_warn("Launching Model Node with with settings " + str([pkg_name, node_file_name, node_name]))
            ###############################
            # Launch Node
            node_file_path = os.path.join(node_file_folder,node_file_name)
            if node_name in node_dict.keys():
                logger.log_info("Node Already Launched: " + str(node_name))
            elif os.path.exists(node_file_path) == False:
                logger.log_info("Could not find Node File at: " + str(node_file_path))
            else: 

                # Pre Set Node Params
                logger.log_warn("Updating model param file path param to " + str(param_file_path))
                param_ns = nepi_sdk.create_namespace(node_namespace,'param_file_path')
                nepi_sdk.set_param(param_ns,param_file_path)


                logger.log_warn("Updating model weight file path param to " + str(param_file_path))
                param_ns = nepi_sdk.create_namespace(node_namespace,'weight_file_path')
                nepi_sdk.set_param(param_ns,weight_file_path)
                    
                #Try and launch node
                
                [success, msg, node_process] = nepi_sdk.launch_node(pkg_name, node_file_name, node_name, namespace=launch_namespace)
                if success == True:
                    node_dict[model_name] = {'node_name': node_name, 'namesapce':node_namespace, 'process':node_process}
                else:
                    logger.log_info("Node launch failed with msg: " + str(msg))
        except Exception as e:
            logger.log_info("Node launch failed with exception: " + str(e))
        
        return success, node_namespace, node_dict


def killModelNode(model_name, node_dict):       
    success = False 
    if model_name in node_dict.keys():
        node_name = node_dict[model_name]['node_name']
        node_namespace = node_dict[model_name]['node_namespace']
        node_process = node_dict[model_name]['process']
        logger.log_info("Killing model node: " + node_name)
        if not (None == node_process):
            success = nepi_sdk.kill_node_process(node_namespace,node_process)
        if success == True:
            logger.log_info("Killed model node: " + node_name)
        del node_dict[model_name]
    return success, node_dict