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
import time

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nepi_ros_interfaces.msg import PTXStatus, PanTiltLimits, PanTiltPosition, SingleAxisTimedMove, AbsolutePanTiltWaypoint
from nepi_ros_interfaces.srv import PTXCapabilitiesQuery, PTXCapabilitiesQueryRequest, PTXCapabilitiesQueryResponse

from tf.transformations import quaternion_from_euler

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF

#from nepi_api.device_if_npx import NPXDeviceIF



class PTXActuatorIF:
    PTX_DIRECTION_POSITIVE = 1
    PTX_DIRECTION_NEGATIVE = -1
    WAYPOINT_COUNT = 256

    # Backup Factory Control Values 
    FACTORY_CONTROLS_DICT = {
                'frame_id' : 'ptx_frame',
                'yaw_joint_name' : 'ptx_yaw_joint',
                'pitch_joint_name' : 'ptx_pitch_joint',
                'reverse_yaw_control' : False,
                'reverse_pitch_control' : False,
                'speed_ratio' : 0.5,
                'status_update_rate_hz' : 10
    }

    navpose_dict = {
                          'frame_3d': 'ENU',
                          'frame_alt': 'WGS84',

                          'geoid_height_meters': 0,

                          'has_heading': False,
                          'time_heading': 0,
                          'heading_deg': 0,

                          'has_oreientation': True,
                          'time_oreientation': nepi_utils.get_time(),
                          # Orientation Degrees in selected 3d frame (roll,pitch,yaw)
                          'roll_deg': 0,
                          'pitch_deg': 0,
                          'yaw_deg': 0,

                          'has_position': False,
                          'time_position': 0,
                          # Relative Position Meters in selected 3d frame (x,y,z) with x forward, y right/left, and z up/down
                          'x_m': 0,
                          'y_m': 0,
                          'z_m': 0,

                          'has_location': False,
                          'time_location': 0,
                          # Global Location in set altitude frame (lat,long,alt) with alt in meters
                          'lat': 0,
                          'long': 0,

                          'has_altitude': False,
                          'time_altitude': 0,
                          'alt_m': 0,
    
                          'has_depth': False,
                          'time_depth': 0,
                          'alt_m': 0
    }


    ready = False

    has_absolute_positioning = False
    has_speed_control = False
    # Define some member variables
    yaw_now_deg = 0.0
    yaw_goal_deg = -999
    yaw_home_pos_deg = 0.0
    min_yaw_softstop_deg = 0.0
    max_yaw_softstop_deg = 0.0
    pitch_now_deg = 0.0
    pitch_goal_deg = -999
    pitch_home_pos_deg = 0.0
    min_pitch_softstop_deg = 0.0
    max_pitch_softstop_deg = 0.0

    reverse_yaw_control = False
    ryi = 1
    reverse_pitch_control = False
    rpi = 1

    last_yaw = 0
    last_pitch = 0

    max_yaw_hardstop_deg = 0
    min_yaw_hardstop_deg = 0
    max_yaw_softstop_deg = 0
    min_yaw_softstop_deg = 0

    max_pitch_hardstop_deg = 0
    min_pitch_hardstop_deg = 0
    max_pitch_softstop_deg = 0
    min_pitch_softstop_deg = 0



    ### IF Initialization
    def __init__(self,  device_info, capSettings, 
                 factorySettings, settingUpdateFunction, getSettingsFunction,
                 factoryControls , # Dictionary to be supplied by parent, specific key set is required
                 defaultSettings,
                 capabilities_dict, # Dictionary to be supplied by parent, specific key set is required
                 stopMovingCb, # Required; no args
                 moveYawCb, # Required; direction and time args
                 movePitchCb, # Required; direction and time args
                 setSpeedCb=None, # None ==> No speed adjustment capability; Speed ratio arg
                 getSpeedCb=None, # None ==> No speed adjustment capabilitiy; Returns speed ratio
                 gotoPositionCb=None, # None ==> No absolute positioning capability (yaw_deg, pitch_deg, speed, float move_timeout_s) 
                 getCurrentPositionCb=None, # None ==> no positional feedback; 
                 goHomeCb=None, # None ==> No native driver homing capability, can still use homing if absolute positioning is supported
                 setHomePositionCb=None, # None ==> No native driver home absolute setting capability, can still use it if absolute positioning is supported
                 setHomePositionHereCb=None, # None ==> No native driver home instant capture capability, can still use it if absolute positioning is supported
                 gotoWaypointCb=None, # None ==> No native driver support for waypoints, can still use if absolute positioning is supported
                 setWaypointCb=None, # None ==> No native driver support for absolute waypoints, can still use if absolute positioning is supported
                 setWaypointHereCb=None, # None ==> No native driver support for instant waypoints, can still use if absolute positioning is supported
                ):

        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ############################## 
        # Initialize Class Variables
        self.device_name = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        # Create and update factory controls dictionary
        self.factory_controls_dict = self.FACTORY_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls_dict.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]


        self.factory_device_name = device_info["device_name"] + "_" + device_info["identifier"]

        self.setSpeedCb = setSpeedCb
        self.getSpeedCb = getSpeedCb
        self.gotoPositionCb = gotoPositionCb
        self.getCurrentPositionCb = getCurrentPositionCb

        self.stopMovingCb = stopMovingCb
        self.moveYawCb = moveYawCb
        self.movePitchCb = movePitchCb

        self.goHomeCb = goHomeCb
        self.setHomePositionCb = setHomePositionCb
        self.setHomePositionHereCb = setHomePositionHereCb
        self.gotoWaypointCb = gotoWaypointCb
        self.setWaypointCb = setWaypointCb
        self.setWaypointHereCb = setWaypointHereCb


        self.defaultSettings = defaultSettings                
        if (self.getCurrentPositionCb is None and capabilities_dict['has_absolute_positioning']):
            # Hard limits
            self.defaultSettings['max_yaw_hardstop_deg'] = 0
            self.defaultSettings['min_yaw_hardstop_deg'] = 0
            self.defaultSettings['max_pitch_hardstop_deg'] = 0
            self.defaultSettings['min_pitch_hardstop_deg'] = 0
  
            # Soft limits
            self.defaultSettings['max_yaw_softstop_deg'] = 0
            self.defaultSettings['min_yaw_softstop_deg'] = 0
            self.defaultSettings['max_pitch_softstop_deg'] = 0
            self.defaultSettings['min_pitch_softstop_deg'] = 0


        # Gather capabilities - Config file takes precedence over parent-supplied defaults
        if capabilities_dict['has_speed_control'] is True:
            if self.setSpeedCb is not None and self.getSpeedCb is not None:
                self.has_adjustable_speed = True
            else:
                self.msg_if.pub_warn("Inconsistent capabilities: adjustable speed reports true, but no callback provided")
                self.has_adjustable_speed = False
        else:
            self.has_adjustable_speed = False
            self.setSpeedCb = None
            self.getSpeedCb = self.getZeroCb

        
        # Positioning and soft limits setup if available
        if (capabilities_dict['has_absolute_positioning'] is True and self.getCurrentPositionCb is None):
            self.msg_if.pub_warn("Inconsistent capabilities: absolute positioning reports true, but no callback provided")
            self.has_position_feedback = False
        else:
            self.has_position_feedback = True


        self.capabilities_report = PTXCapabilitiesQueryResponse()



        # Set up status message static values
        self.status_msg = PTXStatus()
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version


        ##################################################
        ### Node Class Setup

        self.msg_if.pub_info("Starting Node IF Initialization")
        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace':  self.node_namespace
        }


        # Params Config Dict ####################

        self.PARAMS_DICT = {
           'status_update_rate_hz': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['status_update_rate_hz']
            },
            'ptx/has_absolute_positioning': {
                'namespace': self.node_namespace,
                'factory_val': capabilities_dict['has_absolute_positioning']
            },
            'ptx/has_speed_control': {
                'namespace': self.node_namespace,
                'factory_val': capabilities_dict['has_speed_control']
            },
            'ptx/speed_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['speed_ratio']
            },
            'ptx/has_homing': {
                'namespace': self.node_namespace,
                'factory_val': capabilities_dict['has_homing']
            },
            'ptx/has_waypoints': {
                'namespace': self.node_namespace,
                'factory_val': capabilities_dict['has_waypoints']
            },

            'ptx/home_position/yaw_deg': {
                'namespace': self.node_namespace,
                'factory_val': 0.0
            },
            'ptx/home_position/pitch_deg': {
                'namespace': self.node_namespace,
                'factory_val': 0.0
            }, 
            'ptx/frame_id': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['frame_id']
            },            
            'ptx/yaw_joint_name': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['yaw_joint_name']
            },            
            'ptx/pitch_joint_name': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['pitch_joint_name']
            },            
            'ptx/reverse_yaw_control': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['reverse_yaw_control']
            },            
            'ptx/reverse_pitch_control': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['reverse_pitch_control']
            },            
            'ptx/limits/max_yaw_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.defaultSettings['max_yaw_hardstop_deg']
            },            
            'ptx/limits/min_yaw_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.defaultSettings['min_yaw_hardstop_deg']
            },            
            'ptx/limits/max_pitch_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.defaultSettings['max_pitch_hardstop_deg']
            },            
            'ptx/limits/min_pitch_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.defaultSettings['min_pitch_hardstop_deg']
            },            
            'ptx/limits/max_yaw_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.defaultSettings['max_yaw_hardstop_deg']
            },            
            'ptx/limits/min_yaw_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.defaultSettings['min_yaw_hardstop_deg']
            },            
            'ptx/limits/max_pitch_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.defaultSettings['max_pitch_hardstop_deg']
            },           
            'ptx/limits/min_pitch_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.defaultSettings['min_pitch_hardstop_deg']
            }
        }
        

        # Services Config Dict ####################

        self.SRVS_DICT = {
            'capabilities_query': {
                'namespace': self.node_namespace,
                'topic': 'ptx/capabilities_query',
                'srv': PTXCapabilitiesQuery,
                'req': PTXCapabilitiesQueryRequest(),
                'resp': PTXCapabilitiesQueryResponse(),
                'callback': self.provideCapabilities
            }
        }


        self.PUBS_DICT = {
            'joint_pub': {
                'namespace': self.node_namespace,
                'topic': 'ptx/joint_states',
                'msg': JointState,
                'qsize': 10,
                'latch': False
            },
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'ptx/status',
                'msg': PTXStatus,
                'qsize': 10,
                'latch': False
            },
            'odom_pub': {
                'namespace': self.node_namespace,
                'topic': 'ptx/odometry',
                'msg': Odometry,
                'qsize': 10,
                'latch': False
            }
        }



        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'speed_ratio': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setSpeedRatioHandler, 
                'callback_args': ()
            },
            'stop_moving': {
                'namespace': self.node_namespace,
                'topic': 'ptx/stop_moving',
                'msg': Empty,
                'qsize': 1,
                'callback': self.stopMovingHandler, 
                'callback_args': ()
            },
            'jog_to_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_to_position',
                'msg': PanTiltPosition,
                'qsize': 1,
                'callback': self.jogToPositionHandler, 
                'callback_args': ()
            },
            'jog_to_yaw_ratio': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_to_yaw_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.jogToYawRatioHandler, 
                'callback_args': ()
            },
            'jog_to_pitch_ratio': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_to_pitch_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.jogToPitchRatioHandler, 
                'callback_args': ()
            },
            'jog_timed_yaw': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_timed_yaw',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self.jogTimedYawHandler, 
                'callback_args': ()
            },
            'jog_timed_pitch': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_timed_pitch',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self.jogTimedPitchHandler, 
                'callback_args': ()
            },
            'reverse_yaw_control': {
                'namespace': self.node_namespace,
                'topic': 'ptx/reverse_yaw_control',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setReverseYawControl, 
                'callback_args': ()
            },
            'reverse_pitch_control': {
                'namespace': self.node_namespace,
                'topic': 'ptx/reverse_pitch_control',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setReversePitchControl, 
                'callback_args': ()
            },
            'set_hard_limits': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_hard_limits',
                'msg': PanTiltLimits,
                'qsize': 1,
                'callback': self.setHardstopHandler, 
                'callback_args': ()
            },
            'set_soft_limits': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_soft_limits',
                'msg': PanTiltLimits,
                'qsize': 1,
                'callback': self.setSoftstopHandler, 
                'callback_args': ()
            },
            'go_home': {
                'namespace': self.node_namespace,
                'topic': 'ptx/go_home',
                'msg': Empty,
                'qsize': 1,
                'callback': self.goHomeHandler, 
                'callback_args': ()
            },
            'set_home_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_home_position',
                'msg': PanTiltPosition,
                'qsize': 1,
                'callback': self.setHomePositionHandler, 
                'callback_args': ()
            },
            'set_home_position_here': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_home_position_here',
                'msg': Empty,
                'qsize': 1,
                'callback': self.setHomePositionHereHandler, 
                'callback_args': ()
            },
            'goto_waypoint': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_waypoint',
                'msg': UInt8,
                'qsize': 1,
                'callback': self.gotoWaypointHandler, 
                'callback_args': ()
            },
            'set_waypoint': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_waypoint',
                'msg': AbsolutePanTiltWaypoint,
                'qsize': 1,
                'callback': self.setWaypointHandler, 
                'callback_args': ()
            },
            'set_waypoint_here': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_waypoint_here',
                'msg': UInt8,
                'qsize': 1,
                'callback': self.setWaypointHereHandler, 
                'callback_args': ()
            },
        }


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        ready = self.node_if.wait_for_ready()


        # Setup Settings IF Class ####################
        self.msg_if.pub_info("Starting Settings IF Initialization")
        if capSettings is not None:
            self.SETTINGS_DICT = {
                    'capSettings': capSettings, 
                    'factorySettings': factorySettings,
                    'setSettingFunction': settingUpdateFunction, 
                    'getSettingsFunction': getSettingsFunction, 
                    'namespace': self.node_namespace
        }
        else:
            self.SETTINGS_DICT = {
                    'capSettings': nepi_settings.NONE_CAP_SETTINGS, 
                    'factorySettings': nepi_settings.NONE_SETTINGS,
                    'setSettingFunction': nepi_settings.UPDATE_NONE_SETTINGS_FUNCTION, 
                    'getSettingsFunction': nepi_settings.GET_NONE_SETTINGS_FUNCTION, 
                    'namespace': self.node_namespace
        }
        self.settings_if = SettingsIF(self.SETTINGS_DICT)

        '''
        # Setup System IF Classes ####################
        self.msg_if.pub_info("Starting Save Data IF Initialization")
        factory_data_rates = {}
        for d in self.data_products_list:
            factory_data_rates[d] = [1.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_ns': False,
            'suffix': "",
            'add_node_name': True
            }

        self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                                    factory_rate_dict = factory_data_rates,
                                    factory_filename_dict = factory_filename_dict)


        time.sleep(1)


        # Create a NavPose Device IF
        if self.capabilities_report.absolute_positioning is True and self.getCurrentPositionCb is not None:
            self.msg_if.pub_info("Starting NPX Device IF Initialization")
            navpose_if = NPXDeviceIF(device_info, 
                                    has_location = False,
                                    has_position = False,
                                    has_orientation = True,
                                    has_heading = False,
                                    getNavPoseDictFunction = self.getNavPoseDictFunction,
                                    pub_rate = 10)
        '''
        time.sleep(1)
   
        ###############################
        # Finish Initialization


        self.frame_id = self.node_if.get_param('ptx/frame_id')
        self.yaw_joint_name = self.node_if.get_param('ptx/yaw_joint_name')
        self.pitch_joint_name = self.node_if.get_param('ptx/pitch_joint_name')
        self.reverse_yaw_control = self.node_if.get_param('ptx/reverse_yaw_control')
        self.reverse_pitch_control = self.node_if.get_param('ptx/reverse_pitch_control')
        #self.msg_if.pub_warn("Factory Controls Dict: " + str(self.factory_controls_dict))
        #self.msg_if.pub_warn("reverse_yaw_control: " + str(self.reverse_yaw_control))
        # set class reverse int values
        ryi = 1
        if self.reverse_yaw_control:
            ryi = -1
        self.ryi = ryi
        rpi = 1
        if self.reverse_pitch_control:
            rpi = -1
        self.rpi = rpi
        

        self.status_msg.header.frame_id = self.frame_id


        # And joint state status static values
        self.joint_state_msg = JointState()
        # Skip the header -- we just copy it from the status message each time
        self.joint_state_msg.name = (self.yaw_joint_name, self.pitch_joint_name)
        self.joint_state_msg.position.append(0.0) # Yaw
        self.joint_state_msg.position.append(0.0) # Pitch

        # And odom static values
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.device_name + '_fixed_frame'
        self.odom_msg.child_frame_id = self.device_name + '_rotating_frame'
        

        self.has_absolute_positioning = self.node_if.get_param('ptx/has_absolute_positioning')
        self.capabilities_report.absolute_positioning = self.has_absolute_positioning
        
        self.capabilities_report.adjustable_speed = self.node_if.get_param('ptx/has_speed_control')
        if self.capabilities_report.adjustable_speed is False:
            if self.setSpeedCb is not None and self.getSpeedCb is not None:
                speed_ratio = self.node_if.get_param('ptx/speed_ratio')
                self.setSpeedCb(speed_ratio)
                self.capabilities_report.adjustable_speed = True
            else:
                self.msg_if.pub_warn("Inconsistent capabilities: adjustable speed reports true, but no callback provided")
                self.capabilities_report.adjustable_speed = False
        self.capabilities_report.homing = self.node_if.get_param('ptx/has_homing')
        self.capabilities_report.waypoints = self.node_if.get_param('ptx/has_waypoints')
        


        
        # Positioning and soft limits setup if available
        if self.capabilities_report.absolute_positioning is True:
            if (self.getCurrentPositionCb is None):
                self.msg_if.pub_warn("Inconsistent capabilities: absolute positioning reports true, but no callback provided")
                self.capabilities_report.adjustable_speed = False
                # We require both command and feedback reporting to support absolute positioning
                self.capabilities_report.absolute_positioning = False


        # Homing setup
        if self.capabilities_report.homing is True:
            self.goHomeCb = goHomeCb
            self.setHomePositionCb = setHomePositionCb
            self.setHomePositionHereCb = setHomePositionHereCb
        
            if self.goHomeCb is None and self.capabilities_report.absolute_positioning is False:
                self.msg_if.pub_warn("Inconsistent capabilities: homing reports true, but no goHome callback provided and no absolute positioning")
                self.capabilities_report.homing = False
                
            self.home_yaw_deg = self.node_if.get_param('ptx/home_position/yaw_deg')
            self.home_pitch_deg = self.node_if.get_param('ptx/home_position/pitch_deg')
                
        # Waypoint setup
        if self.capabilities_report.waypoints is True:
            self.gotoWaypointCb = gotoWaypointCb
            self.setWaypointCb = setWaypointCb
            self.setWaypointHereCb = setWaypointHereCb

            if self.gotoWaypointCb is None:
                self.msg_if.pub_warn("Inconsistent capabilities: waypoints reports true, but no gotoWaypoint callback provided")
                self.capabilities_report.waypoints = False
                              




        self.initCb(do_updates = True)

        # Periodic publishing
        self.status_update_rate = self.node_if.get_param('status_update_rate_hz')   
        self.msg_if.pub_info("Starting pt status publisher at hz: " + str(self.status_update_rate))    
        status_pub_delay = float(1.0) / self.status_update_rate
        self.msg_if.pub_info("Starting pt status publisher at sec delay: " + str(status_pub_delay))
        nepi_ros.start_timer_process(status_pub_delay, self.publishJointStateAndStatus, oneshot = True)


        self.publish_status()
        self.ready = True
        self.msg_if.pub_info("Initialization Complete")



    ###############################
    # Class Methods

    def check_ready(self):
        return self.ready  

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready   


    def getNavPoseDictFunction():
        if self.capabilities_report.absolute_positioning is True and self.getCurrentPositionCb is not None:
            yaw_now_deg, pitch_now_deg = self.getCurrentPositionCb()
            self.navpose_dict['pitch_deg'] = pitch_now_deg
            self.navpose_dict['yaw_deg'] = yaw_now_deg
        self.navpose_dict['time_oreintation'] = nepi_utils.get_time()
        return self.navpose_dict 


    def yawRatioToDeg(self, ratio):
        yaw_deg = 0
        if self.reverse_yaw_control == False:
           yaw_deg =  self.min_yaw_softstop_deg + (1-ratio) * (self.max_yaw_softstop_deg - self.min_yaw_softstop_deg)
        else:
           yaw_deg =  self.max_yaw_softstop_deg - (1-ratio)  * (self.max_yaw_softstop_deg - self.min_yaw_softstop_deg)
        return  yaw_deg
    
    def yawDegToRatio(self, deg):
        ratio = 0.5
        if self.reverse_yaw_control == False:
          ratio = 1 - (deg - self.min_yaw_softstop_deg) / (self.max_yaw_softstop_deg - self.min_yaw_softstop_deg)
        else:
          ratio = (deg - self.min_yaw_softstop_deg) / (self.max_yaw_softstop_deg - self.min_yaw_softstop_deg)
        return (ratio)
     
    def pitchDegToRatio(self, deg):
        ratio = 0.5
        if self.reverse_pitch_control == False:
          ratio = 1 - (deg - self.min_pitch_softstop_deg) / (self.max_pitch_softstop_deg - self.min_pitch_softstop_deg)
        else:
          ratio = (deg - self.min_pitch_softstop_deg) / (self.max_pitch_softstop_deg - self.min_pitch_softstop_deg)
        return ratio
    
    def pitchRatioToDeg(self, ratio):
        pitch_deg = 0
        if self.reverse_pitch_control == False:
           pitch_deg =  self.min_pitch_softstop_deg + (1-ratio) * (self.max_pitch_softstop_deg - self.min_pitch_softstop_deg)
        else:
           pitch_deg =  (self.max_pitch_softstop_deg) - (1-ratio) * (self.max_pitch_softstop_deg - self.min_pitch_softstop_deg)
        return  pitch_deg

    def publishJointStateAndStatus(self, Timer):
        #self.msg_if.pub_warn("Publishing status")
        pub_time = self.publish_status()
        pub_time = round(pub_time, 3)
        #self.msg_if.pub_warn("Published status with process time: " + str(pub_time))

        status_pub_delay = float(1.0) / self.status_update_rate
        if pub_time > status_pub_delay:
            status_pub_delay = 0.01
        else:
            status_pub_delay = status_pub_delay - pub_time
        nepi_ros.start_timer_process(status_pub_delay, self.publishJointStateAndStatus, oneshot = True)

    def publish_status(self):
        start_time = nepi_utils.get_time()
        self.status_msg.header.stamp = nepi_ros.ros_time_now()
        #self.msg_if.pub_info("Entering Publish Status")
  
        if self.capabilities_report.absolute_positioning is True and self.getCurrentPositionCb is not None:
            yaw_now_deg, pitch_now_deg = self.getCurrentPositionCb()
            got_time = nepi_utils.get_time() - start_time
            got_time = round(got_time, 3)
            #self.msg_if.pub_warn("Got PT status with time: " + str(got_time))

            if round(yaw_now_deg,5) != round(self.yaw_goal_deg,5) or round(pitch_now_deg,5) != round(self.pitch_goal_deg,5):
                self.status_msg.yaw_now_deg = yaw_now_deg 
                self.status_msg.pitch_now_deg = pitch_now_deg 
                '''
                if self.yaw_goal_deg == -999:
                    self.yaw_goal_deg = yaw_now_deg
                if self.pitch_goal_deg == -999:
                    self.pitch_goal_deg = pitch_now_deg
                '''
                self.status_msg.reverse_yaw_control = self.reverse_yaw_control
                max_yaw_hardstop_deg = self.max_yaw_hardstop_deg
                min_yaw_hardstop_deg = self.min_yaw_hardstop_deg
                max_yaw_softstop_deg = self.max_yaw_softstop_deg
                min_yaw_softstop_deg = self.min_yaw_softstop_deg
                if self.reverse_yaw_control:
                    max_yaw_hardstop_deg = -1*min_yaw_hardstop_deg
                    min_yaw_hardstop_deg = -1*max_yaw_hardstop_deg
                    max_yaw_softstop_deg = -1*min_yaw_softstop_deg
                    min_yaw_softstop_deg = -1*max_yaw_softstop_deg
                else:
                    max_yaw_hardstop_deg = max_yaw_hardstop_deg
                    min_yaw_hardstop_deg = min_yaw_hardstop_deg
                    max_yaw_softstop_deg = max_yaw_softstop_deg
                    min_yaw_softstop_deg = min_yaw_softstop_deg
                self.status_msg.yaw_max_hardstop_deg = max_yaw_hardstop_deg
                self.status_msg.yaw_min_hardstop_deg = min_yaw_hardstop_deg
                self.status_msg.yaw_max_softstop_deg = max_yaw_softstop_deg
                self.status_msg.yaw_min_softstop_deg = min_yaw_softstop_deg
                self.status_msg.yaw_goal_deg = self.yaw_goal_deg * self.ryi
                self.status_msg.yaw_home_pos_deg = self.home_yaw_deg * self.ryi

                yaw_now_ratio =  1 - (yaw_now_deg - min_yaw_softstop_deg) / (max_yaw_softstop_deg - min_yaw_softstop_deg) 
                #self.msg_if.pub_warn("yaw_now, min_yaw, max_yaw, yaw_now_ratio: " + str([yaw_now_deg,min_yaw_softstop_deg,max_yaw_softstop_deg,yaw_now_ratio]))
                if yaw_now_ratio < 0:
                    yaw_now_ratio = 0
                elif yaw_now_ratio > 1:
                    yaw_now_ratio = 1
                self.status_msg.yaw_now_ratio = yaw_now_ratio 
                yaw_goal_deg = self.yaw_goal_deg * self.ryi
                yaw_goal_ratio =  1 - (yaw_goal_deg - min_yaw_softstop_deg) / (max_yaw_softstop_deg - min_yaw_softstop_deg) 
                #self.msg_if.pub_warn("yaw_now, min_yaw, max_yaw, yaw_goal_ratio: " + str([yaw_now_deg,min_yaw_softstop_deg,max_yaw_softstop_deg,yaw_goal_ratio]))
                if yaw_goal_ratio < 0:
                    yaw_goal_ratio = 0
                elif yaw_goal_ratio > 1:
                    yaw_goal_ratio = 1
                self.status_msg.yaw_goal_ratio = yaw_goal_ratio 


                self.status_msg.reverse_pitch_control = self.reverse_pitch_control
                max_pitch_hardstop_deg = self.max_pitch_hardstop_deg
                min_pitch_hardstop_deg = self.min_pitch_hardstop_deg
                max_pitch_softstop_deg = self.max_pitch_softstop_deg
                min_pitch_softstop_deg = self.min_pitch_softstop_deg
                if self.reverse_pitch_control:
                    max_pitch_hardstop_deg = -1*min_pitch_hardstop_deg
                    min_pitch_hardstop_deg = -1*max_pitch_hardstop_deg
                    max_pitch_softstop_deg = -1*min_pitch_softstop_deg
                    min_pitch_softstop_deg = -1*max_pitch_softstop_deg
                else:
                    max_pitch_hardstop_deg = max_pitch_hardstop_deg
                    min_pitch_hardstop_deg = min_pitch_hardstop_deg
                    max_pitch_softstop_deg = max_pitch_softstop_deg
                    min_pitch_softstop_deg = min_pitch_softstop_deg
                self.status_msg.pitch_max_hardstop_deg = max_pitch_hardstop_deg
                self.status_msg.pitch_min_hardstop_deg = min_pitch_hardstop_deg
                self.status_msg.pitch_max_softstop_deg = max_pitch_softstop_deg
                self.status_msg.pitch_min_softstop_deg = min_pitch_softstop_deg
                self.status_msg.pitch_goal_deg = self.pitch_goal_deg * self.ryi
                self.status_msg.pitch_home_pos_deg = self.home_pitch_deg * self.ryi

                pitch_now_ratio =  1 - (pitch_now_deg - min_pitch_softstop_deg) / (max_pitch_softstop_deg - min_pitch_softstop_deg) 
                #self.msg_if.pub_warn("pitch_now, min_pitch, max_pitch,pitch_now_ratio: " + str([pitch_now_deg,min_pitch_softstop_deg,max_pitch_softstop_deg,pitch_now_ratio]))
                if pitch_now_ratio < 0:
                    pitch_now_ratio = 0
                elif pitch_now_ratio > 1:
                    pitch_now_ratio = 1
                self.status_msg.pitch_now_ratio = pitch_now_ratio 
                pitch_goal_deg = self.pitch_goal_deg * self.ryi
                pitch_goal_ratio =  1 - (pitch_goal_deg - min_pitch_softstop_deg) / (max_pitch_softstop_deg - min_pitch_softstop_deg) 
                #self.msg_if.pub_warn("pitch_now, min_pitch, max_pitch, pitch_goal_ratio: " + str([pitch_now_deg,min_pitch_softstop_deg,max_pitch_softstop_deg,pitch_goal_ratio]))
                if pitch_goal_ratio < 0:
                    pitch_goal_ratio = 0
                elif pitch_goal_ratio > 1:
                    pitch_goal_ratio = 1
                self.status_msg.pitch_goal_ratio = pitch_goal_ratio 


                if self.capabilities_report.adjustable_speed is True:
                    self.status_msg.speed_ratio = self.getSpeedCb()

                self.status_msg.has_position_feedback = self.has_position_feedback
                self.status_msg.has_adjustable_speed = self.has_adjustable_speed

                #self.msg_if.pub_warn("Publishing Status")

                self.node_if.publish_pub('status_pub',self.status_msg)


                yaw_rad = 0.01745329 * self.status_msg.yaw_now_deg
                pitch_rad = 0.01745329 * self.status_msg.pitch_now_deg

                # And joint state if appropriate
                self.joint_state_msg.header.stamp = self.status_msg.header.stamp
                self.joint_state_msg.position[0] = yaw_rad
                self.joint_state_msg.position[1] = pitch_rad
                #self.msg_if.pub_warn("Publishing Joint")
                self.node_if.publish_pub('joint_pub',self.joint_state_msg)

                self.odom_msg.header.stamp = self.status_msg.header.stamp
                self.odom_msg.pose.pose.orientation = quaternion_from_euler(0.0, pitch_rad, yaw_rad)
                #self.msg_if.pub_warn("Publishing Odom")
                self.node_if.publish_pub('odom_pub',self.odom_msg)
        pub_time = nepi_utils.get_time() - start_time
        return pub_time


    def positionWithinSoftLimits(self, yaw_deg, pitch_deg):

        if (yaw_deg < self.min_yaw_softstop_deg) or (yaw_deg > self.max_yaw_softstop_deg) or \
           (pitch_deg < self.min_pitch_softstop_deg) or (pitch_deg > self.max_pitch_softstop_deg):
            return False
        
        return True


    def setHardstopHandler(self, msg):
        min_yaw = msg.min_yaw_deg
        max_yaw = msg.max_yaw_deg
        min_pitch = msg.min_pitch_deg
        max_pitch = msg.max_pitch_deg

        valid = False
        if min_yaw < max_yaw and min_pitch < max_pitch:
            if min_yaw >= self.defaultSettings['min_yaw_hardstop_deg'] and max_yaw <= self.defaultSettings['max_yaw_hardstop_deg']:
                if min_pitch >= self.defaultSettings['min_pitch_hardstop_deg'] and max_pitch <= self.defaultSettings['max_pitch_hardstop_deg']:

                    self.node_if.set_param('ptx/limits/max_yaw_hardstop_deg', max_yaw)
                    self.node_if.set_param('ptx/limits/min_yaw_hardstop_deg', min_yaw)
                    self.node_if.set_param('ptx/limits/max_pitch_hardstop_deg', max_pitch)
                    self.node_if.set_param('ptx/limits/min_pitch_hardstop_deg', min_pitch)

                    self.node_if.set_param('ptx/limits/max_yaw_softstop_deg', max_yaw)
                    self.node_if.set_param('ptx/limits/min_yaw_softstop_deg', min_yaw)
                    self.node_if.set_param('ptx/limits/max_pitch_softstop_deg', max_pitch)
                    self.node_if.set_param('ptx/limits/min_pitch_softstop_deg', min_pitch)
                    valid = True

                    self.initCb(do_updates = True)


        if valid == False:
            self.msg_if.pub_warn("Invalid hardstop requested " + str(msg))
        

    def setSoftstopHandler(self, msg):
        min_yaw = msg.min_yaw_deg
        max_yaw = msg.max_yaw_deg
        min_pitch = msg.min_pitch_deg
        max_pitch = msg.max_pitch_deg

        max_yaw_hardstop_deg = self.node_if.get_param('ptx/limits/max_yaw_hardstop_deg')
        min_yaw_hardstop_deg = self.node_if.get_param('ptx/limits/min_yaw_hardstop_deg')
        max_pitch_hardstop_deg = self.node_if.get_param('ptx/limits/max_pitch_hardstop_deg')
        min_pitch_hardstop_deg = self.node_if.get_param('ptx/limits/min_pitch_hardstop_deg')

        valid = False
        if min_yaw < max_yaw and max_yaw <= max_yaw_hardstop_deg and min_yaw < max_yaw:  
            if min_pitch >= min_pitch_hardstop_deg and max_pitch <= max_pitch_hardstop_deg and min_pitch < max_pitch:
                self.node_if.set_param('ptx/limits/max_yaw_softstop_deg', max_yaw)
                self.node_if.set_param('ptx/limits/min_yaw_softstop_deg', min_yaw)
                self.node_if.set_param('ptx/limits/max_pitch_softstop_deg', max_pitch)
                self.node_if.set_param('ptx/limits/min_pitch_softstop_deg', min_pitch)
                valid = True

                self.initCb(do_updates = True)


        if valid == False:
            self.msg_if.pub_warn("Invalid softstop requested " + str(msg))
        

   
    def setSpeedRatioHandler(self, msg):
        if self.capabilities_report.adjustable_speed is True:
            speed_cur = self.getSpeedCb()
            speed_ratio = msg.data
            if (speed_ratio < 0.0) or (speed_ratio > 1.0):
                self.msg_if.pub_warn("Invalid speed ratio requested " + "%.2f" % speed_ratio)
            elif speed_cur != speed_ratio and self.setSpeedCb is not None:
                self.setSpeedCb(speed_ratio)
                self.msg_if.pub_info("Updated speed ratio to " + str(speed_ratio))
        

    def setHomePositionHandler(self, msg):
        if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
            self.msg_if.pub_warn("Requested home position is invalid... ignoring")
            return

        if self.setHomePositionCb is not None:
            # Driver supports absolute positioning, so just let it operate
            self.home_yaw_deg = msg.yaw_deg
            self.home_pitch_deg = msg.pitch_deg
            self.setHomePositionCb(self.home_yaw_deg, self.home_pitch_deg)
        else:
            self.msg_if.pub_warn("Absolution position home setpoints not available... ignoring")
            return
        
        self.msg_if.pub_info("Updated home position to " + "%.2f" % self.home_yaw_deg + " " + "%.2f" %  self.home_pitch_deg)
            

    def goHomeHandler(self, _):
        if self.goHomeCb is not None:
            self.yaw_goal_deg = self.home_yaw_deg
            self.pitch_goal_deg = self.home_pitch_deg
            self.goHomeCb()
        

    def jogToPositionHandler(self, msg):
        if self.positionWithinSoftLimits is not None:
            if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
                self.msg_if.pub_warn("Requested jog position is invalid... ignoring")
                return
            self.yaw_goal_deg = msg.yaw_deg
            self.pitch_goal_deg = msg.pitch_deg
            self.msg_if.pub_info("Driving to  " + "%.2f" % self.yaw_goal_deg + " " + "%.2f" % self.pitch_goal_deg)
            self.gotoPositionCb(yaw_deg = (self.yaw_goal_deg * self.ryi), pitch_deg = (self.pitch_goal_deg * self.rpi))
        

    def jogToYawRatioHandler(self, msg):
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            self.msg_if.pub_warn("Invalid yaw position ratio " + "%.2f" % ratio)
            return
        if self.gotoPositionCb is not None:
            self.yaw_goal_deg = self.yawRatioToDeg(ratio)
            pitch_now_deg = self.status_msg.pitch_now_deg
            self.msg_if.pub_info("Driving to  " + "%.2f" % self.yaw_goal_deg + " " + "%.2f" % pitch_now_deg)
            self.gotoPositionCb(yaw_deg = self.yaw_goal_deg, pitch_deg = pitch_now_deg)
        

    def jogToPitchRatioHandler(self, msg):
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            self.msg_if.pub_warn("Invalid pitch position ratio " + "%.2f" % ratio)
            return
        if self.gotoPositionCb is not None:
            self.pitch_goal_deg = self.pitchRatioToDeg(ratio)
            yaw_now_deg = self.status_msg.yaw_now_deg
            self.msg_if.pub_info("Driving to  " + "%.2f" % yaw_now_deg + " " + "%.2f" % self.pitch_goal_deg)
            self.gotoPositionCb(yaw_deg = yaw_now_deg, pitch_deg = self.pitch_goal_deg)
        

    def stopMovingHandler(self, _):
        if self.stopMovingCb is not None:
            self.stopMovingCb()
            self.yaw_goal_deg = self.status_msg.yaw_now_deg
            self.pitch_goal_deg = self.status_msg.pitch_now_deg
            self.msg_if.pub_info("Stopping motion by request")
        

    def jogTimedYawHandler(self, msg):
        if self.moveYawCb is not None:
            direction = msg.direction if self.reverse_yaw_control is False else (-1 * msg.direction)
            duration = 1000000.0 if (msg.duration_s < 0.0) else msg.duration_s
            self.moveYawCb(direction,  duration)
            self.msg_if.pub_info("Jogging yaw")
        

    def jogTimedPitchHandler(self, msg):
        if self.movePitchCb is not None:
            direction = msg.direction if self.reverse_pitch_control is False else (-1 * msg.direction)
            duration = 1000000.0 if (msg.duration_s < 0.0) else msg.duration_s
            self.movePitchCb(direction, duration)
            self.msg_if.pub_info("Jogging pitch")
        

    def setReverseYawControl(self, msg):
        self.reverse_yaw_control = msg.data
        rpi = 1
        if msg.data:
            rpi = -1
        self.rpi = rpi
        self.msg_if.pub_info("Set yaw control to reverse=" + str(self.reverse_yaw_control))
        

    def setReversePitchControl(self, msg):
        self.reverse_pitch_control = msg.data
        ryi = 1
        if msg.data:
            ryi = -1
        self.ryi = ryi
        self.msg_if.pub_info("Set pitch control to reverse=" + str(self.reverse_pitch_control))
        

    def setHomePositionHereHandler(self, _):
        if self.setHomePositionHereCb is not None:
            # Driver supports it directly
            # Capture home position if possible
            if self.getCurrentPositionCb is not None:
                self.home_yaw_deg = self.status_msg.yaw_now_deg
                self.home_pitch_deg = self.status_msg.pitch_now_deg
            self.setHomePositionHereCb()
        else:
            self.msg_if.pub_warn("Instant home position not available for this device")
            return
        self.msg_if.pub_info("Updated home position to current position")
        

    def setWaypointHandler(self, msg):
        if self.positionWithinSoftLimits is not None:
            yaw_deg = msg.yaw_deg
            pitch_deg = msg.pitch_deg
            waypoint_index = msg.waypoint_index
            if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
                self.msg_if.pub_warn("Requested waypoint position is invalid... ignoring")
                return
            if self.setWaypointCb is not None:
                self.setWaypointCb(waypoint_index, yaw_deg, pitch_deg)
        

    def setWaypointHereHandler(self, msg):
        waypoint_index = msg.data
        if self.setWaypointHereCb is not None:
            self.setWaypointHereCb(waypoint_index)
        
    
    def gotoWaypointHandler(self, msg):
        waypoint_index = msg.data
        if self.gotoWaypointCb is not None:
            self.gotoWaypointCb(waypoint_index)
            self.msg_if.pub_info("Going to waypoint by command " + str(waypoint_index))
        
    
    def provideCapabilities(self, _):
        return self.capabilities_report
    

    def initConfig(self):
        self.initCb(do_updates = True)

    def initCb(self, do_updates = False):
        if do_updates == True:
            self.resetCb()



    def resetCb(self, do_updates = True):
        self.max_yaw_hardstop_deg = self.node_if.get_param('ptx/limits/max_yaw_hardstop_deg')
        self.min_yaw_hardstop_deg = self.node_if.get_param('ptx/limits/min_yaw_hardstop_deg')
        self.max_yaw_softstop_deg = self.node_if.get_param('ptx/limits/max_yaw_softstop_deg')
        self.min_yaw_softstop_deg = self.node_if.get_param('ptx/limits/min_yaw_softstop_deg')

        self.max_pitch_hardstop_deg = self.node_if.get_param('ptx/limits/max_pitch_hardstop_deg')
        self.min_pitch_hardstop_deg = self.node_if.get_param('ptx/limits/min_pitch_hardstop_deg')
        self.max_pitch_softstop_deg = self.node_if.get_param('ptx/limits/max_pitch_softstop_deg')
        self.min_pitch_softstop_deg = self.node_if.get_param('ptx/limits/min_pitch_softstop_deg')

        #**********************
        # This one comes from the parent
        self.node_if.set_param('ptx/speed_ratio', self.getSpeedCb()) 
        #**********************
        



    def factoryResetCb(self, do_updates = True):
        self.resetCb()

    def passFunction(self):
        return 0

