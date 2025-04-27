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

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils

from nepi_sdk.nepi_ros import logger as Logger
log_name = "nepi_cfg"
logger = Logger(log_name = log_name)
  
#***************************
# NEPI Save Config utility functions


def get_save_config_subscriber_namespaces():
    topics_list = nepi_ros.find_topics_by_name('save_config')
    namespaces_list = []
    for topic in topics_list:
        namespaces_list.append(os.path.dirname(topic))
    return namespaces_list

