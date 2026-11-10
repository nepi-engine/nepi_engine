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

# Sample NEPI Process Script. 
# 1) 

# Requires the following additional scripts are running
# a)


import time
import sys

from nepi_sdk import nepi_sdk

from std_msgs.msg import Empty, Float32

#########################################
# USER SETTINGS - Edit as Necessary 
#########################################


#########################################
# ROS NAMESPACE SETUP
#########################################

# ROS namespace setup
NEPI_BASE_NAMESPACE = "/nepi/device1/"

#########################################
# Node Class
#########################################

class name_of_file_without_script(object):

  #######################
  ### Node Initialization
  def __init__(self):
    nepi_sdk.log_msg_info("Starting Initialization Processes")
    ## Initialize Class Variables
    ## Define Class Namespaces
    ## Define Class Services Calls
    ## Create Class Sevices    
    ## Create Class Publishers
    ## Start Class Subscribers
    ## Start Node Processes
    ## Initiation Complete
    nepi_sdk.log_msg_info("Initialization Complete")

  #######################
  ### Node Methods
  

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_sdk.log_msg_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  current_filename = sys.argv[0].split('/')[-1]
  current_filename = current_filename.split('.')[0]
  nepi_sdk.log_msg_info(("Starting " + current_filename), disable_signals=True) # Disable signals so we can force a shutdown
  nepi_sdk.init_node(name=current_filename)
  #Launch the node
  node_name = current_filename.rpartition("_")[0]
  nepi_sdk.log_msg_info("Launching node named: " + node_name)
  node_class = eval(node_name)
  node = node_class()
  #Set up node shutdown
  nepi_sdk.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  nepi_sdk.spin()

