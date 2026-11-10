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

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_interfaces.msg import DataProductIFStatus
from nepi_interfaces.srv import DataProductIFStatusQuery, DataProductIFStatusQueryRequest, DataProductIFStatusQueryResponse

from nepi_api.node_if import NodeIF




#######################################
# DataProductIF
#######################################

class DataProductIF(NodeIF):
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

    data_source_description = 'sensor'
    data_ref_description = 'sensor'

    data_product = 'data_product'

    msg_if = None
    node_if = None
    save_data_if = None
    transform_if = None
    navpose_if = None

    save_config_needed = False

    needs_data = False

    time_list = [0,0,0,0,0,0,0,0,0,0]

    last_pub_time = None
    publishing = False

    def __init__(self,
                name = 'data_product',
                namespace = None,
                data_product = None,
                data_source_description = 'sensor',
                data_ref_description = 'sensor',
                info_dict = dict(), # self.info_dict can be created/updated later
                caps_dict = dict(), # self.caps_dict can be created/updated later
                stats_dict = dict(),
                rui_dict = dict(),
                config_control_dict = dict(), # self.config_controls_if can be created/updated later
                node_controls_dict = dict(),
                callback_dict = dict(),            
                msg_if = None,
                node_if = None,
                save_data_if = None,
                transform_if = None,
                navpose_if = None,
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.IF_DICT = dict(self.IF_DICT)
        self.time_list = copy.deepcopy(self.time_list)
              
        ##############################
        # Update Class Variables
        ##############################

        name = nepi_utils.get_clean_name(name)
        if data_product is None:
            data_product = self.DATA_PRODUCT
        data_product = nepi_utils.get_clean_name(data_product)
        if data_product.strip() != '':
            self.data_product = data_product


        # Initialize Status Msg.  Updated on each publish
        if data_source_description is None:
            data_source_description = self.data_source_description
        self.data_source_description = data_source_description

        if data_ref_description is None:
            data_ref_description = self.data_ref_description
        self.data_ref_description = data_ref_description


        ##############################  
        # Setup NodeIF Updates
        ##############################  

        #######################
        # Sync System IF inputs to if_dict
        self.msg_if = msg_if
        self.IF_DICT['msg_if'] = msg_if
        self.node_if = node_if
        self.IF_DICT['node_if'] = node_if
        self.save_data_if = save_data_if
        self.IF_DICT['save_data_if'] = save_data_if
        self.transform_if = transform_if
        self.IF_DICT['transform_if'] = transform_if
        self.navpose_if = navpose_if
        self.IF_DICT['navpose_if'] = navpose_if

        ##################
        update_info_dict = dict(
            description = self.DESCRIPTION,
            display_name = self.DISPLAY_NAME,
            type = 'data_product',
            source_description = data_source_description,
            source_ref_description = data_source_description,
            end_description = data_ref_description,
            end_ref_description = data_ref_description,
        )

        info_dict.update(update_info_dict)
        
        
        ###########
        # NODE IF SETUP

        # Call the parent class constructor
        super().__init__(id = self.ID,
                name = name,
                namespace = namespace,
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

        nepi_sdk.sleep(0.1)

        if_srvs_dict = {
            'data_product_if_status_query': {
                'namespace': self.namespace,
                'topic': 'data_product_if_status_query',
                'srv': DataProductIFStatusQuery,
                'req': DataProductIFStatusQueryRequest(),
                'resp': DataProductIFStatusQueryResponse(),
                'callback': self._dataProductIFStatusQueryCb
            }
        }

        self.add_services(if_srvs_dict)

        ##############################
        # Start Data Product IF Processes

        self.msg_if.pub_debug("Staring subscribers check process" )
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)

        ##############################
        # Initialization Complete
        ###############################



    ###############################
    # Data Product IF Public Methods
    ###############################

    def get_data_source_description(self):
        """Return the human-readable data source description string.

        Returns:
            str: Description of the data source (e.g. 'imaging_sensor').
        """
        return self.data_source_description

    def get_data_product(self):
        """Return the data product name for this interface.

        Returns:
            str: The data product identifier string (e.g. 'color_image').
        """
        return self.data_product


    def needs_data_check(self):
        """Return whether downstream consumers currently need image data.

        Returns:
            bool: True if there are active subscribers or save/snapshot requests.
        """
        return self.needs_data


    def needs_update(self):
        """Signal that a parameter change requires a new frame to be captured or processed.

        Sets the save-config flag and fires the registered needs_update_callback
        if one has been set.
        """
        self.save_config_needed = True
        if self.callback_dict['needs_update_callback'] is not None:
            self.callback_dict['needs_update_callback']()


    ###############################
    # Data Product IF Private Methods
    ###############################

    def _needsDataCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers(self.node_if_prefix + 'data_pub')
        needs_save = False
        needs_snapshot = False
        if self.save_data_if is not None:
            needs_save = self.save_data_if.data_product_save_enabled(self.data_product)
            needs_snapshot = self.save_data_if.data_product_snapshot_enabled(self.data_product)
        needs_data = has_subs or needs_save or needs_snapshot
        if needs_data == False and self.status_msg is not None:
            self.status_msg.publishing = False
        self.needs_data = needs_data
        #self.msg_if.pub_debug("Needs Data Check End: " + self.namespace + " : " + str([has_subs,needs_save, needs_snapshot]) )
        nepi_sdk.start_timer_process(1.0, self._needsDataCheckCb, oneshot = True)

    def _dataProductIFStatusQueryCb(self, _):
        # The response carries a DataProductIFStatus, which wraps the NodeIFStatus
        if_status_msg = DataProductIFStatus()
        if_status_msg.if_status = self.get_if_status_msg()
        return if_status_msg