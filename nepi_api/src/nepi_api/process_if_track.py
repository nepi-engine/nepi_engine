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

import copy
import math

import numpy as np

from nepi_sdk import nepi_utils
from nepi_sdk import nepi_sdk

from nepi_sdk import nepi_process_track

from nepi_api.process_if import ProcessIF

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_process_track"
logger = Logger(log_name = log_name)




##################################################
# ProcessTrackIF



class ProcessTrackIF(ProcessIF):

    
    def __init__(self, process_name = nepi_process_track.DEFAULT_PROCESS_NAME,
                process_group = nepi_process_track.DEFAULT_PROCESS_NAME,
                process_description = nepi_process_track.DEFAULT_PROCESS_NAME,
                callback_dict = None,
                config_dict = None,
                show_dict = None,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None,
                save_data_if = None
                ):


        process_module = nepi_process_track

        ###############################
        ####  IF INIT SETUP ####
        class_name = type(self).__name__
        log_name=class_name
        ###############################

        super().__init__(process_name,
                process_group,
                process_description,
                process_module,
                config_dict,
                callback_dict,
                show_dict,
                log_name,
                log_name_list,
                msg_if,
                node_if,
                save_data_if,
                )

        self.msg_if.pub_info(str(class_name) + " Initialization Complete")
        ###############################

