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
import copy
import threading
import inspect

from nepi_sdk import nepi_sdk
from nepi_sdk.nepi_sdk import logger
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_interfaces.msg import Message


class MsgIF(logger):


    add_functions = ['pub_msg','pub_info','pub_warn','pub_error','pub_debug']
    add_files = []

    #######################
    ### IF Initialization
    def __init__(self, namespace = None):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.node_name = nepi_sdk.get_node_name()
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_namespace = nepi_sdk.get_node_namespace()
        ############################## 


        if namespace is None:
            namespace = self.node_namespace
        # Call the parent class constructor
        super().__init__( 
                         namespace = namespace, 
                         ignore_functions = self.add_functions, 
                         ignore_files = self.add_files
                )


    ###############################
    # Class Public Methods



    
    def pub_info(self, *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
        super().log_info(*args, **kwargs)
    
    def pub_warn(self,  *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
        super().log_warn(*args, **kwargs)
    
    def pub_error(self,  *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
        super().log_error(*args, **kwargs)
    
    def pub_fatal(self, *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
        super().log_fatal(*args, **kwargs)
      

    def pub_debug(self,  *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
        super().log_debug(*args, **kwargs)