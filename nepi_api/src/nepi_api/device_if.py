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
import time 
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils


from nepi_interfaces.msg import DeviceIFStatus
from nepi_interfaces.srv import DeviceIFStatusQuery, DeviceIFStatusQueryRequest, DeviceIFStatusQueryResponse
from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryResponse, DeviceInfoQueryRequest

from nepi_api.node_if import NodeIF


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeIF

from nepi_api.system_if import SettingsIF, SaveDataIF, TransformIF, NavPoseIF
from nepi_api.connect_system_if import ConnectNavPoseIF

#######################################
# DeviceIF
#######################################

class DeviceIF(NodeIF):
    #########################
    ## Managed by NodeIF
    #########################

    # id = ''
    # name = ''
    # display_name = ''
    # namespace = '~'

    # status_msg = None
    # data_msg = None

    # msg_if = None
    # node_if = None
    # save_data_if = None
    # transform_if = None
    # navpose_if = None    
    
    # callback_dict = dict()

    # active_topics = []
    # active_topic_types = []
    # active_services = []

    #########################
    ## CUSTOM DataProductsIF
    #########################

    IF_DICT = dict()

    info_report = DeviceInfoQueryResponse()

    data_source_description = 'sensor'
    data_ref_description = 'sensor'

    device_name = ''
    path = ''
    serial_num = ''
    hw_version = ''
    sw_version = ''

    factory_controls_dict = dict()

    caps_dict = dict()

    msg_if = None
    node_if = None
    settings_if = None
    save_data_if = None
    transform_if = None
    navpose_if = None

    color_image_if = None
    depthmap_if = None
    pointcloud_if = None

    connect_navpose_if = None

    def __init__(self,
                device_info = dict(), 
                getSettingsFunction=None, setSettingFunction=None, 
                factoryControls = None, 
                data_source_description = None,
                data_ref_description = None,
                info_dict = dict(), # self.info_dict can be created/updated later
                caps_dict = dict(), # self.caps_dict can be created/updated later
                stats_dict = dict(),
                rui_dict = dict(),
                config_control_dict = dict(), # self.config_controls_if can be created/updated later
                node_controls_dict = dict(),
                callback_dict = dict(),              
                msg_if = None,
                node_if = None,
                settings_if = None,
                save_data_if = None,
                connect_navpose_if = None
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.IF_DICT = dict(self.IF_DICT)
        self.info_report = copy.deepcopy(self.info_report)
              
        ##############################
        # Update Class Variables
        ##############################
        self.start_time = nepi_utils.get_time()

        #############################
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        display_name = str(self.node_name).replace("_", '').capitalize()

        # Initialize Status Msg.  Updated on each publish
        if data_source_description is None:
            data_source_description = self.DATA_SOURCE_DESCRIPTION
        self.data_source_description = data_source_description

        if data_ref_description is None:
            data_ref_description = self.DATA_REF_DESCRIPTION
        self.data_ref_description = data_ref_description

        ###################################
        device_name = device_info.get("device_name", self.node_name)
        self.device_name = device_name
        self.path = device_info.get("path", 'unknown')
        self.serial_num = device_info.get("serial_number", 'unknown')
        self.hw_version = device_info.get("hw_version", 'unknown')
        self.sw_version = device_info.get("sw_version", 'unknown')


        ################################
        # Create and update factory controls dictionary
        self.factory_controls_dict = copy.deepcopy(self.FACTORY_CONTROLS_DICT)
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]

        #############################
        self.info_report.device_name = self.device_name
        self.info_report.device_path = self.path
        self.info_report.node_name = self.node_name
        self.info_report.node_namespace = self.node_namespace
        self.info_report.serial_num = self.serial_num
        self.info_report.hw_version = self.hw_version
        self.info_report.sw_version = self.sw_version
        self.info_report.type = self.ID.upper()

        ##############################  
        # Setup NodeIF Updates
        ##############################  

        #######################
        # Sync System IF inputs to if_dict
        self.msg_if = msg_if
        self.IF_DICT['msg_if'] = msg_if
        self.node_if = node_if
        self.IF_DICT['node_if'] = node_if
        self.settings_if = settings_if
        self.IF_DICT['settings_if'] = settings_if
        self.save_data_if = save_data_if
        self.IF_DICT['save_data_if'] = save_data_if
        self.connect_navpose_if = connect_navpose_if
        self.IF_DICT['connect_navpose_if'] = connect_navpose_if

        ##################
        update_info_dict = dict(

            type = 'device',
            sub_type = self.ID,
            group = '',
            sub_group = '',

            device_name = self.device_name,
            serial_num = self.serial_num,
            hw_version = self.hw_version,
            sw_version = self.sw_version,


            display_name = display_name,
            description = self.DESCRIPTION,
            source_description = data_source_description,
            source_ref_description = data_ref_description,
            end_description = data_source_description,
            end_ref_description = data_ref_description,

        )

        info_dict.update(update_info_dict)
        
        
        ###########
        # NODE IF SETUP

        # Call the parent class constructor
        super().__init__(id = self.ID,
                namespace = nepi_sdk.create_namespace(self.node_namespace, nepi_sdk.get_clean_name(device_name)),
                name = self.ID,
                status_msg = copy.deepcopy(self.STATUS_MSG),
                data_msg = copy.deepcopy(self.DATA_MSG),
                info_dict = info_dict,
                caps_dict = caps_dict,
                stats_dict = stats_dict, 
                rui_dict = rui_dict, 
                config_control_dict = config_control_dict,
                node_controls_dict = node_controls_dict, 
                callback_dict = callback_dict, 
                if_dict = dict(self.IF_DICT)
                )

        self.status_msg.device_name = self.device_name
        self.status_msg.device_path = self.path
        self.status_msg.device_node_name = self.node_name
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version

        self.msg_if.pub_debug("Got Factory Controls: " + str(factoryControls))

        nepi_sdk.sleep(0.1)

        if_srvs_dict = {
            'device_if_status_query': {
                'namespace': self.namespace,
                'topic': 'device_if_status_query',
                'srv': DeviceIFStatusQuery,
                'req': DeviceIFStatusQueryRequest(),
                'resp': DeviceIFStatusQueryResponse(),
                'callback': self._deviceIFStatusQueryCb
            },
            'device_info_query': {
                'namespace': self.namespace,
                'topic': 'device_info_query',
                'srv': DeviceInfoQuery,
                'req': DeviceInfoQueryRequest(),
                'resp': DeviceInfoQueryResponse(),
                'callback': self._deviceInfoQueryCb
            },

        }

        self.add_services(if_srvs_dict)


        ##############################
        # Create Required System IFs
        ##############################]

        ###############################
        # Setup Settings IF Class ####################
        if settings_if is not None:
            self.msg_if.pub_info("Using Passed Settings IF")
            self.settings_if = settings_if
        else:
            self.getSettingsFunction = getSettingsFunction
            self.setSettingFunction = setSettingFunction
            if self.getSettingsFunction is not None and self.setSettingFunction is not None:
                self.msg_if.pub_debug("Starting New Settings IF Initialization" )
                self.node_namespace

                self.settings_if = SettingsIF(namespace = self.node_namespace,
                                getSettingsFunction=self.getSettingsFunction, 
                                setSettingFunction=self.setSettingFunction, 
                                info_dict = self.info_dict,
                                msg_if = self.msg_if,
                                node_if = self.node_if
                                )
        self.if_dict['settings_if'] = self.settings_if


        ##################################
        # Setup Save Data IF Class ####################
        if save_data_if is not None:
            self.msg_if.pub_info("Using Passed Save Data IF", color = 'BLUE')
            self.save_data_if = save_data_if


        else:
            self.msg_if.pub_info("Starting New Save Data IF Initialization" , color = 'RED')
            self.save_data_if = SaveDataIF(namespace = self.node_namespace,
                                    info_dict = self.info_dict,
                                    msg_if = self.msg_if,
                                    node_if = self.node_if)
        self.if_dict['save_data_if'] = self.save_data_if


        ##################################
        # Setup Transform IF Class ####################

        self.msg_if.pub_info("Starting New Transfrom IF Initialization" , color = 'RED')
        self.transform_if = TransformIF(namespace = self.node_namespace,
                                info_dict = self.info_dict,
                                msg_if = self.msg_if,
                                node_if = self.node_if)
        self.if_dict['transform_if'] = self.transform_if



        ##################################
        # Setup NavPose IF Class ####################
        self.msg_if.pub_info("Starting New NavPose IF Initialization" , color = 'RED')
        self.navpose_if = NavPoseIF(namespace = self.node_namespace,
                                frame_name = self.node_name,
                                info_dict = self.info_dict,
                                msg_if = self.msg_if,
                                node_if = self.node_if,
                                save_data_if = self.save_data_if,
                                transform_if = self.transform_if)
        self.if_dict['navpose_if'] = self.navpose_if


        ##################################
        # Setup Connect NavPose IF Class ####################
        if connect_navpose_if is not None:
            self.msg_if.pub_info("Using Passed NavPose IF", color = 'BLUE')
            self.connect_navpose_if = connect_navpose_if


        else:
            self.msg_if.pub_info("Starting New Connect NavPose IF Initialization" , color = 'RED')
            # No default source. ConnectSystemIF passes its namespace argument through
            # as the connect's selected_topic, and ConnectNodeIF restores the saved
            # selection only when that default is "None". Passing node_namespace here
            # made every device select itself (not a navpose topic, so its navpose
            # never published) and threw away the operator's saved pick on restart.
            self.connect_navpose_if = ConnectNavPoseIF(
                                    namespace = self.node_namespace,
                                    navpose_callback = self._pubNavposeCb,
                                    msg_if = self.msg_if,
                                    node_if = self.node_if)
            # The source's data is republished on this device's own navpose topic, so
            # selecting that topic as the source would feed every message back to itself
            self.connect_navpose_if.exclude_namespaces_list.append(self.navpose_if.get_namespace())
            # Configs saved under the old default hold the node namespace as the
            # selection. Clear it, persisted value included, so the device starts
            # unselected instead of stuck on a topic that can never connect.
            if self.connect_navpose_if.get_selected_topic() == self.node_namespace:
                self.connect_navpose_if.set_selected_topic('None')
        self.connect_if_dict['connect_navpose_if'] = self.connect_navpose_if

        nepi_sdk.sleep(0.1)
        ##############################
        # Start Device IF Processes
        ##############################]

        ##############################
        # Initialization Complete
        ###############################



    ###############################
    # Device IF Private Methods
    ###############################


    def _deviceIFStatusQueryCb(self, _):
        # The response carries a DeviceIFStatus, which wraps the NodeIFStatus
        if_status_msg = DeviceIFStatus()
        if_status_msg.if_status = self.get_if_status_msg()
        return if_status_msg

        ### Info callback
    def _deviceInfoQueryCb(self, _):
        """Handles a ROS service request for device information.

        Returns the pre-populated ``DeviceInfoQueryResponse`` containing
        device name, path, node name, namespace, serial number, hardware
        version, software version, and device type.

        Args:
            _ (DeviceInfoQueryRequest): Unused service request object.

        Returns:
            DeviceInfoQueryResponse: The device information report.
        """
        return self.info_report


    def _pubNavposeCb(self, navpose_dict):
        if navpose_dict is not None and self.navpose_if is not None:
            try:
                self.navpose_if.publish_navpose(navpose_dict)
            except:
                pass