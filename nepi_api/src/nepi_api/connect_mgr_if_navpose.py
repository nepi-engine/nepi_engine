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
import rospy


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_nav

from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest


class ConnectMgrNavPoseIF:
 
    NODE_NAME = 'navpose_mgr'
    PUB_SUB_NAME = ''

    connected = False


    services_dict = {
        'navpose_query': {
            'connected': False,
            'msg': SystemStorageFolderQuery,
            'req': SystemStorageFolderQueryRequest(),
            'psn': PUB_SUB_NAME,
            'service': None
        }
    }



    #######################
    ### IF Initialization
    log_name = "ConnectMgrNavPoseIF"

    def __init__(self):
        #################################
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting IF Initialization Processes")
        ##############################

        self.mgr_namespace = os.path.join(self.base_namespace,self.NODE_NAME)
        
        self.pub_sub_namespace = os.path.join(self.base_namespace,self.PUB_SUB_NAME)

        ##################################
        ### Set up Services
        for service_name in self.services_dict.keys():
            service_dict = self.services_dict[service_name]
            if service_dict['psn'] is None: 
                service_namespace = os.path.join(self.base_namespace, service_name)
            else:
                service_namespace = os.path.join(self.base_namespace,service_dict['psn'], service_name)
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for " + service_name + " on namespace " + service_namespace)
            ret = nepi_ros.wait_for_service(service_namespace, timeout = float('inf') )
            if ret == "":
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Wait for service: " + service_name + " timed out") 
            else:
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Creating service call for: " + service_name)
                #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Creating service with namespace: " + service_namespace)
                #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Creating service with msg: " + str(service_dict['msg']))
                service = None
                try:
                    service = nepi_ros.create_service(service_namespace, service_dict['msg'])
                    time.sleep(1)
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to get service connection: " + service_name + " " + str(e))  
            if service is not None:
                self.services_dict[service_name]['service'] = service
                self.services_dict[service_name]['connected'] = True


        #################################
        self.connected = True
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": IF Initialization Complete")
        #################################



    #######################
    # Class Public Sub Methods
    #######################

    def check_connection(self):
        return self.connection

    def wait_for_connection(self, timout = float('inf') ):
        success = False
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.connected == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.connected == False:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Connect")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Connected")
        return self.connected


    def get_navpose_query_response(self):
        resp = None
        service = self.services_dict[service_name]['service']
        if self.connected is False:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Manager not connected")
        elif service is None:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Service not available")
        else:
            try:
                request = NavPoseQueryRequest()
                resp = nepi_ros.call_service(service, request)
                nav_dict = nepi_ros.convert_msg2dict(resp)
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to get valid navpose service response: " + str(resp) + " " + str(e))                
        return resp


    def get_navpose_query_dict(self):
        nav_dict = None
        service = self.services_dict[service_name]['service']
        if self.connected is False:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Manager not connected")
        elif service is None:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Service not available")
        else:
            try:
                request = NavPoseQueryRequest()
                resp = nepi_ros.call_service(service, request)
                nav_dict = nepi_ros.convert_msg2dict(resp)
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to get valid navpose service response: " + str(resp) + " " + str(e))                
        return nav_dict

    def get_navpose_data_dict(self):
        nav_dict = None
        service = self.services_dict[service_name]['service']
        if self.connected is False:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Manager not connected")
        elif service is None:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Service not available")
        else:
            try:
                request = NavPoseQueryRequest()
                resp = nepi_ros.call_service(service, request)
                nav_dict = nepi_nav.convert_navpose_resp2data_dict(resp)
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to get valid navpose service response: " + str(resp) + " " + str(e))                
        return nav_dict


    #######################
    # Class Private Methods
    #######################



