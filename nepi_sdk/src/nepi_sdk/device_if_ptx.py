#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import rospy
from std_msgs.msg import UInt8, Float32, Empty, Bool
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nepi_ros_interfaces.msg import PanTiltStatus, PanTiltLimits, PanTiltPosition, SingleAxisTimedMove, AbsolutePanTiltWaypoint
from nepi_ros_interfaces.srv import PTXCapabilitiesQuery, PTXCapabilitiesQueryResponse

from tf.transformations import quaternion_from_euler

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg

from nepi_sdk.settings_if import SettingsIF
from nepi_sdk.save_cfg_if import SaveCfgIF

class ROSPTXActuatorIF:
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
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting IF Initialization Processes")
        ############################## 

        self.device_name = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.factory_device_name = device_info["device_name"] + "_" + device_info["identifier"]


        # Create and update factory controls dictionary
        self.factory_controls_dict = self.FACTORY_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls_dict.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]

        self.frame_id = rospy.get_param('~ptx/frame_id', self.factory_controls_dict['frame_id'])
        self.yaw_joint_name = rospy.get_param("~ptx/yaw_joint_name", self.factory_controls_dict['yaw_joint_name'])
        self.pitch_joint_name = rospy.get_param("~ptx/pitch_joint_name", self.factory_controls_dict['pitch_joint_name'])
        self.reverse_yaw_control = rospy.get_param("~ptx/reverse_yaw_control", self.factory_controls_dict['reverse_yaw_control'])
        self.reverse_pitch_control = rospy.get_param("~ptx/reverse_pitch_control", self.factory_controls_dict['reverse_pitch_control'])
        #nepi_msg.publishMsgWarn(self,"Factory Controls Dict: " + str(self.factory_controls_dict))
        #nepi_msg.publishMsgWarn(self,"reverse_yaw_control: " + str(self.reverse_yaw_control))
        # set class reverse int values
        ryi = 1
        if self.reverse_yaw_control:
            ryi = -1
        self.ryi = ryi
        rpi = 1
        if self.reverse_pitch_control:
            rpi = -1
        self.rpi = rpi
        
        # Set up status message static values
        self.status_msg = PanTiltStatus()
        self.status_msg.header.frame_id = self.frame_id
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version

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
        
        # Gather capabilities - Config file takes precedence over parent-supplied defaults
        self.capabilities_report = PTXCapabilitiesQueryResponse()
        self.has_absolute_positioning = rospy.get_param('~ptx/capabilities/has_absolute_positioning', capabilities_dict['has_absolute_positioning'])
        self.capabilities_report.absolute_positioning = self.has_absolute_positioning
        
        self.capabilities_report.adjustable_speed = rospy.get_param('~ptx/capabilities/has_speed_control', capabilities_dict['has_speed_control'])
        self.capabilities_report.homing = rospy.get_param('ptx/capabilities/has_homing', capabilities_dict['has_homing'])
        self.capabilities_report.waypoints = rospy.get_param('ptx/capabilities/has_waypoints', capabilities_dict['has_waypoints'])
        

        rospy.Subscriber('~reset_device', Empty, self.resetAppHandler, queue_size=1)

        # Stop motion setup
        self.stopMovingCb = stopMovingCb
        rospy.Subscriber('~ptx/stop_moving', Empty, self.stopMovingHandler, queue_size=1)

        # Timed jog setup
        self.moveYawCb = moveYawCb
        rospy.Subscriber('~ptx/jog_timed_yaw', SingleAxisTimedMove, self.jogTimedYawHandler, queue_size=1)
        self.movePitchCb = movePitchCb
        rospy.Subscriber('~ptx/jog_timed_pitch', SingleAxisTimedMove, self.jogTimedPitchHandler, queue_size=1)

        # Reverse controls setup
        rospy.Subscriber('~ptx/reverse_yaw_control', Bool, self.setReverseYawControl, queue_size=1)
        rospy.Subscriber('~ptx/reverse_pitch_control', Bool, self.setReversePitchControl, queue_size=1)

        # Speed setup if available
        if self.capabilities_report.adjustable_speed is True:
            self.setSpeedCb = setSpeedCb
            self.getSpeedCb = getSpeedCb
            if self.setSpeedCb is not None and self.getSpeedCb is not None:
                rospy.Subscriber('~ptx/set_speed_ratio', Float32, self.setSpeedRatioHandler, queue_size=1)
                speed_ratio = rospy.get_param('~ptx/speed_ratio', self.factory_controls_dict['speed_ratio'])
                self.setSpeedCb(speed_ratio)
                self.capabilities_report.adjustable_speed = True
                self.has_adjustable_speed = True
            else:
                nepi_msg.publishMsgWarn(self,'Inconsistent capabilities: adjustable speed reports true, but no callback provided')
                self.capabilities_report.adjustable_speed = False
                self.has_adjustable_speed = False
        
        # Positioning and soft limits setup if available
        if self.capabilities_report.absolute_positioning is True:
            self.gotoPositionCb = gotoPositionCb
            self.getCurrentPositionCb = getCurrentPositionCb
            if (self.getCurrentPositionCb is None):
                nepi_msg.publishMsgWarn(self,'Inconsistent capabilities: absolute positioning reports true, but no callback provided')
                self.capabilities_report.adjustable_speed = False
                # We require both command and feedback reporting to support absolute positioning
                self.capabilities_report.absolute_positioning = False
                self.has_position_feedback = False
            else:
                self.has_position_feedback = True
        else:
            self.has_position_feedback = False
                
        self.defaultSettings = defaultSettings
        if self.has_position_feedback is True:
            # Hard limits
            self.max_yaw_hardstop_deg = rospy.get_param('~ptx/limits/max_yaw_hardstop_deg', self.defaultSettings['max_yaw_hardstop_deg'])
            self.min_yaw_hardstop_deg = rospy.get_param('~ptx/limits/min_yaw_hardstop_deg', self.defaultSettings['min_yaw_hardstop_deg'])
            self.max_pitch_hardstop_deg = rospy.get_param('~ptx/limits/max_pitch_hardstop_deg', self.defaultSettings['max_pitch_hardstop_deg'])
            self.min_pitch_hardstop_deg = rospy.get_param('~ptx/limits/min_pitch_hardstop_deg', self.defaultSettings['min_pitch_hardstop_deg'])
            rospy.Subscriber('~ptx/set_hard_limits', PanTiltLimits, self.setHardstopHandler, queue_size=1)
   
            # Soft limits
            self.max_yaw_softstop_deg = rospy.get_param('~ptx/limits/max_yaw_softstop_deg', self.defaultSettings['max_yaw_softstop_deg'])
            self.min_yaw_softstop_deg = rospy.get_param('~ptx/limits/min_yaw_softstop_deg', self.defaultSettings['min_yaw_softstop_deg'])
            self.max_pitch_softstop_deg = rospy.get_param('~ptx/limits/max_pitch_softstop_deg', self.defaultSettings['max_pitch_softstop_deg'])
            self.min_pitch_softstop_deg = rospy.get_param('~ptx/limits/min_pitch_softstop_deg', self.defaultSettings['min_pitch_softstop_deg'])
            rospy.Subscriber('~ptx/set_soft_limits', PanTiltLimits, self.setSoftstopHandler, queue_size=1)

            # Jog to position
            rospy.Subscriber('~ptx/jog_to_position', PanTiltPosition, self.jogToPositionHandler, queue_size=1)

            # Jog to yaw ratio
            rospy.Subscriber('~ptx/jog_to_yaw_ratio', Float32, self.jogToYawRatioHandler, queue_size=1)

            # Jog to pitch ratio
            rospy.Subscriber('~ptx/jog_to_pitch_ratio', Float32, self.jogToPitchRatioHandler, queue_size=1)

            # Joint state publisher
            self.joint_pub = rospy.Publisher('ptx/joint_states', JointState, queue_size=10)

            # Odometry publisher
            self.odom_pub = rospy.Publisher('~ptx/odometry', Odometry, queue_size=10)
        
        else:
            self.max_yaw_hardstop_deg = 0.0
            self.min_yaw_hardstop_deg = 0.0
            self.max_pitch_hardstop_deg = 0.0
            self.min_pitch_hardstop_deg = 0.0
                        
            # Soft limits
            self.max_yaw_softstop_deg = 0.0
            self.min_yaw_softstop_deg = 0.0
            self.max_pitch_softstop_deg = 0.0
            self.min_pitch_softstop_deg = 0.0

            self.joint_pub = None
            self.odom_pub = None


        # Homing setup
        if self.capabilities_report.homing is True:
            self.goHomeCb = goHomeCb
            self.setHomePositionCb = setHomePositionCb
            self.setHomePositionHereCb = setHomePositionHereCb
        
            if self.goHomeCb is None and self.capabilities_report.absolute_positioning is False:
                nepi_msg.publishMsgWarn(self,'Inconsistent capabilities: homing reports true, but no goHome callback provided and no absolute positioning')
                self.capabilities_report.homing = False
                
        if self.capabilities_report.homing is True:
            rospy.Subscriber('~ptx/go_home', Empty, self.goHomeHandler, queue_size=1)
            
            if self.setHomePositionCb is not None:
                rospy.Subscriber('~ptx/set_home_position', PanTiltPosition, self.setHomePositionHandler,  queue_size=1)

            if self.setHomePositionHereCb is not None:
                rospy.Subscriber('~ptx/set_home_position_here', Empty, self.setHomePositionHereHandler, queue_size=1)
            
            self.home_yaw_deg = rospy.get_param('~ptx/home_position/yaw_deg', 0.0)
            self.home_pitch_deg = rospy.get_param('~ptx/home_position/pitch_deg', 0.0)
                
        # Waypoint setup
        if self.capabilities_report.waypoints is True:
            self.gotoWaypointCb = gotoWaypointCb
            self.setWaypointCb = setWaypointCb
            self.setWaypointHereCb = setWaypointHereCb

            if self.gotoWaypointCb is None:
                nepi_msg.publishMsgWarn(self,'Inconsistent capabilities: waypoints reports true, but no gotoWaypoint callback provided')
                self.capabilities_report.waypoints = False
                
        if self.capabilities_report.waypoints is True:
            rospy.Subscriber('~ptx/goto_waypoint', UInt8, self.gotoWaypointHandler, queue_size=1)

            if self.setWaypointCb is not None:
                rospy.Subscriber('~ptx/set_waypoint', AbsolutePanTiltWaypoint, self.setWaypointHandler, queue_size=1)

            if self.setWaypointHereCb is not None:
                rospy.Subscriber('~ptx/set_waypoint_here', UInt8, self.setWaypointHereHandler, queue_size=1)
            
        # TODO: Read waypoints in from a config. file? 

        # Set up publishers        
        self.status_pub = rospy.Publisher('~ptx/status', PanTiltStatus, queue_size=10, latch=True)

        # Periodic publishing
        self.status_update_rate = rospy.get_param('~status_update_rate_hz', self.factory_controls_dict['status_update_rate_hz'])   
        nepi_msg.publishMsgInfo(self,"Starting pt status publisher at hz: " + str(self.status_update_rate))    
        status_joint_state_pub_period = rospy.Duration(float(1.0) / self.status_update_rate)
        nepi_msg.publishMsgInfo(self,"Starting pt status publisher at sec delay: " + str(status_joint_state_pub_period))
        rospy.Timer(status_joint_state_pub_period, self.publishJointStateAndStatus)
        
        # Set up service providers
        rospy.Service('~ptx/capabilities_query', PTXCapabilitiesQuery, self.provideCapabilities)

        nepi_msg.publishMsgInfo(self,"Initializing Settings IF")
        self.settings_if = SettingsIF(capSettings, factorySettings, settingUpdateFunction, getSettingsFunction)
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initializeParamServer, paramsModifiedCallback=self.updateFromParamServer)


        # Update and Publish Status Message
        self.publishStatus()
        ## Initiation Complete
        nepi_msg.publishMsgInfo(self,"Initialization Complete")


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
        self.publishStatus()

    def publishStatus(self):
        self.status_msg.header.stamp = rospy.Time.now()
        #nepi_msg.publishMsgInfo(self,"Entering Publish Status")
  
        if self.capabilities_report.absolute_positioning is True and self.getCurrentPositionCb is not None:
            yaw_now_deg, pitch_now_deg = self.getCurrentPositionCb()

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
                max_yaw_hs = rospy.get_param('~ptx/limits/max_yaw_hardstop_deg', self.defaultSettings['max_yaw_hardstop_deg'])
                min_yaw_hs = rospy.get_param('~ptx/limits/min_yaw_hardstop_deg', self.defaultSettings['min_yaw_hardstop_deg'])
                max_yaw_ss = rospy.get_param('~ptx/limits/max_yaw_softstop_deg', self.defaultSettings['max_yaw_softstop_deg'])
                min_yaw_ss = rospy.get_param('~ptx/limits/min_yaw_softstop_deg', self.defaultSettings['min_yaw_softstop_deg'])
                if self.reverse_yaw_control:
                    max_yaw_hardstop_deg = -1*min_yaw_hs
                    min_yaw_hardstop_deg = -1*max_yaw_hs
                    max_yaw_softstop_deg = -1*min_yaw_ss
                    min_yaw_softstop_deg = -1*max_yaw_ss
                else:
                    max_yaw_hardstop_deg = max_yaw_hs
                    min_yaw_hardstop_deg = min_yaw_hs
                    max_yaw_softstop_deg = max_yaw_ss
                    min_yaw_softstop_deg = min_yaw_ss
                self.status_msg.yaw_max_hardstop_deg = max_yaw_hardstop_deg
                self.status_msg.yaw_min_hardstop_deg = min_yaw_hardstop_deg
                self.status_msg.yaw_max_softstop_deg = max_yaw_softstop_deg
                self.status_msg.yaw_min_softstop_deg = min_yaw_softstop_deg
                self.status_msg.yaw_goal_deg = self.yaw_goal_deg * self.ryi
                self.status_msg.yaw_home_pos_deg = self.home_yaw_deg * self.ryi

                yaw_now_ratio =  1 - (yaw_now_deg - min_yaw_softstop_deg) / (max_yaw_softstop_deg - min_yaw_softstop_deg) 
                #nepi_msg.publishMsgWarn(self,"yaw_now, min_yaw, max_yaw, yaw_now_ratio: " + str([yaw_now_deg,min_yaw_softstop_deg,max_yaw_softstop_deg,yaw_now_ratio]))
                if yaw_now_ratio < 0:
                    yaw_now_ratio = 0
                elif yaw_now_ratio > 1:
                    yaw_now_ratio = 1
                self.status_msg.yaw_now_ratio = yaw_now_ratio 
                yaw_goal_deg = self.yaw_goal_deg * self.ryi
                yaw_goal_ratio =  1 - (yaw_goal_deg - min_yaw_softstop_deg) / (max_yaw_softstop_deg - min_yaw_softstop_deg) 
                #nepi_msg.publishMsgWarn(self,"yaw_now, min_yaw, max_yaw, yaw_goal_ratio: " + str([yaw_now_deg,min_yaw_softstop_deg,max_yaw_softstop_deg,yaw_goal_ratio]))
                if yaw_goal_ratio < 0:
                    yaw_goal_ratio = 0
                elif yaw_goal_ratio > 1:
                    yaw_goal_ratio = 1
                self.status_msg.yaw_goal_ratio = yaw_goal_ratio 


                self.status_msg.reverse_pitch_control = self.reverse_pitch_control
                max_pitch_hs = rospy.get_param('~ptx/limits/max_pitch_hardstop_deg', self.defaultSettings['max_pitch_hardstop_deg'])
                min_pitch_hs = rospy.get_param('~ptx/limits/min_pitch_hardstop_deg', self.defaultSettings['min_pitch_hardstop_deg'])
                max_pitch_ss = rospy.get_param('~ptx/limits/max_pitch_softstop_deg', self.defaultSettings['max_pitch_softstop_deg'])
                min_pitch_ss = rospy.get_param('~ptx/limits/min_pitch_softstop_deg', self.defaultSettings['min_pitch_softstop_deg'])
                if self.reverse_pitch_control:
                    max_pitch_hardstop_deg = -1*min_pitch_hs
                    min_pitch_hardstop_deg = -1*max_pitch_hs
                    max_pitch_softstop_deg = -1*min_pitch_ss
                    min_pitch_softstop_deg = -1*max_pitch_ss
                else:
                    max_pitch_hardstop_deg = max_pitch_hs
                    min_pitch_hardstop_deg = min_pitch_hs
                    max_pitch_softstop_deg = max_pitch_ss
                    min_pitch_softstop_deg = min_pitch_ss
                self.status_msg.pitch_max_hardstop_deg = max_pitch_hardstop_deg
                self.status_msg.pitch_min_hardstop_deg = min_pitch_hardstop_deg
                self.status_msg.pitch_max_softstop_deg = max_pitch_softstop_deg
                self.status_msg.pitch_min_softstop_deg = min_pitch_softstop_deg
                self.status_msg.pitch_goal_deg = self.pitch_goal_deg * self.ryi
                self.status_msg.pitch_home_pos_deg = self.home_pitch_deg * self.ryi

                pitch_now_ratio =  1 - (pitch_now_deg - min_pitch_softstop_deg) / (max_pitch_softstop_deg - min_pitch_softstop_deg) 
                #nepi_msg.publishMsgWarn(self,"pitch_now, min_pitch, max_pitch,pitch_now_ratio: " + str([pitch_now_deg,min_pitch_softstop_deg,max_pitch_softstop_deg,pitch_now_ratio]))
                if pitch_now_ratio < 0:
                    pitch_now_ratio = 0
                elif pitch_now_ratio > 1:
                    pitch_now_ratio = 1
                self.status_msg.pitch_now_ratio = pitch_now_ratio 
                pitch_goal_deg = self.pitch_goal_deg * self.ryi
                pitch_goal_ratio =  1 - (pitch_goal_deg - min_pitch_softstop_deg) / (max_pitch_softstop_deg - min_pitch_softstop_deg) 
                #nepi_msg.publishMsgWarn(self,"pitch_now, min_pitch, max_pitch, pitch_goal_ratio: " + str([pitch_now_deg,min_pitch_softstop_deg,max_pitch_softstop_deg,pitch_goal_ratio]))
                if pitch_goal_ratio < 0:
                    pitch_goal_ratio = 0
                elif pitch_goal_ratio > 1:
                    pitch_goal_ratio = 1
                self.status_msg.pitch_goal_ratio = pitch_goal_ratio 


                if self.capabilities_report.adjustable_speed is True:
                    self.status_msg.speed_ratio = self.getSpeedCb()

                self.status_msg.has_position_feedback = self.has_position_feedback
                self.status_msg.has_adjustable_speed = self.has_adjustable_speed

                #nepi_msg.publishMsgWarn(self,"Publishing Status")

                self.status_pub.publish(self.status_msg)


                yaw_rad = 0.01745329 * self.status_msg.yaw_now_deg
                pitch_rad = 0.01745329 * self.status_msg.pitch_now_deg

                # And joint state if appropriate
                if self.joint_pub is not None:
                    self.joint_state_msg.header.stamp = self.status_msg.header.stamp
                    self.joint_state_msg.position[0] = yaw_rad
                    self.joint_state_msg.position[1] = pitch_rad
                    #nepi_msg.publishMsgWarn(self,"Publishing Joint")
                    self.joint_pub.publish(self.joint_state_msg)

                if self.odom_pub is not None:
                    self.odom_msg.header.stamp = self.status_msg.header.stamp
                    self.odom_msg.pose.pose.orientation = quaternion_from_euler(0.0, pitch_rad, yaw_rad)
                    #nepi_msg.publishMsgWarn(self,"Publishing Odom")
                    self.odom_pub.publish(self.odom_msg)


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
                    rospy.set_param('~ptx/limits/max_yaw_hardstop_deg', max_yaw)
                    rospy.set_param('~ptx/limits/min_yaw_hardstop_deg', min_yaw)
                    rospy.set_param('~ptx/limits/max_pitch_hardstop_deg', max_pitch)
                    rospy.set_param('~ptx/limits/min_pitch_hardstop_deg', min_pitch)

                    rospy.set_param('~ptx/limits/max_yaw_softstop_deg', max_yaw)
                    rospy.set_param('~ptx/limits/min_yaw_softstop_deg', min_yaw)
                    rospy.set_param('~ptx/limits/max_pitch_softstop_deg', max_pitch)
                    rospy.set_param('~ptx/limits/min_pitch_softstop_deg', min_pitch)
                    valid = True
        if valid == False:
            nepi_msg.publishMsgWarn(self,"Invalid hardstop requested " + str(msg))
        

    def setSoftstopHandler(self, msg):
        min_yaw = msg.min_yaw_deg
        max_yaw = msg.max_yaw_deg
        min_pitch = msg.min_pitch_deg
        max_pitch = msg.max_pitch_deg

        max_yaw_hs = rospy.get_param('~ptx/limits/max_yaw_hardstop_deg', self.defaultSettings['max_yaw_hardstop_deg'])
        min_yaw_hs = rospy.get_param('~ptx/limits/min_yaw_hardstop_deg', self.defaultSettings['min_yaw_hardstop_deg'])
        max_pitch_hs = rospy.get_param('~ptx/limits/max_pitch_hardstop_deg', self.defaultSettings['max_pitch_hardstop_deg'])
        min_pitch_hs = rospy.get_param('~ptx/limits/min_pitch_hardstop_deg', self.defaultSettings['min_pitch_hardstop_deg'])

        valid = False
        if min_yaw < max_yaw and max_yaw <= max_yaw_hs and min_yaw < max_yaw:  
            if min_pitch >= min_pitch_hs and max_pitch <= max_pitch_hs and min_pitch < max_pitch:
                rospy.set_param('~ptx/limits/max_yaw_softstop_deg', max_yaw)
                rospy.set_param('~ptx/limits/min_yaw_softstop_deg', min_yaw)
                rospy.set_param('~ptx/limits/max_pitch_softstop_deg', max_pitch)
                rospy.set_param('~ptx/limits/min_pitch_softstop_deg', min_pitch)
                valid = True
        if valid == False:
            nepi_msg.publishMsgWarn(self,"Invalid softstop requested " + str(msg))
        

   
    def setSpeedRatioHandler(self, msg):
        speed_cur = self.getSpeedCb()
        speed_ratio = msg.data
        if (speed_ratio < 0.0) or (speed_ratio > 1.0):
            nepi_msg.publishMsgWarn(self,"Invalid speed ratio requested " + "%.2f" % speed_ratio)
        elif speed_cur != speed_ratio:
            self.setSpeedCb(speed_ratio)
            nepi_msg.publishMsgInfo(self,"Updated speed ratio to " + str(speed_ratio))
        

    def setHomePositionHandler(self, msg):
        if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
            nepi_msg.publishMsgWarn(self,"Requested home position is invalid... ignoring")
            return

        if self.setHomePositionCb is not None:
            # Driver supports absolute positioning, so just let it operate
            self.home_yaw_deg = msg.yaw_deg
            self.home_pitch_deg = msg.pitch_deg
            self.setHomePositionCb(self.home_yaw_deg, self.home_pitch_deg)
        else:
            nepi_msg.publishMsgWarn(self,"Absolution position home setpoints not available... ignoring")
            return
        
        nepi_msg.publishMsgInfo(self,"Updated home position to " + "%.2f" % self.home_yaw_deg + " " + "%.2f" %  self.home_pitch_deg)
            

    def goHomeHandler(self, _):
        if self.goHomeCb is not None:
            self.yaw_goal_deg = self.home_yaw_deg
            self.pitch_goal_deg = self.home_pitch_deg
            self.goHomeCb()
        

    def jogToPositionHandler(self, msg):
        if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
            nepi_msg.publishMsgWarn(self,"Requested jog position is invalid... ignoring")
            return
        self.yaw_goal_deg = msg.yaw_deg
        self.pitch_goal_deg = msg.pitch_deg
        nepi_msg.publishMsgInfo(self,"Driving to  " + "%.2f" % self.yaw_goal_deg + " " + "%.2f" % self.pitch_goal_deg)
        self.gotoPositionCb(yaw_deg = (self.yaw_goal_deg * self.ryi), pitch_deg = (self.pitch_goal_deg * self.rpi))
        

    def jogToYawRatioHandler(self, msg):
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            nepi_msg.publishMsgWarn(self,"Invalid yaw position ratio " + "%.2f" % ratio)
            return
        self.yaw_goal_deg = self.yawRatioToDeg(ratio)
        pitch_now_deg = self.status_msg.pitch_now_deg
        nepi_msg.publishMsgInfo(self,"Driving to  " + "%.2f" % self.yaw_goal_deg + " " + "%.2f" % pitch_now_deg)
        self.gotoPositionCb(yaw_deg = self.yaw_goal_deg, pitch_deg = pitch_now_deg)
        

    def jogToPitchRatioHandler(self, msg):
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            nepi_msg.publishMsgWarn(self,"Invalid pitch position ratio " + "%.2f" % ratio)
            return
        
        self.pitch_goal_deg = self.pitchRatioToDeg(ratio)
        yaw_now_deg = self.status_msg.yaw_now_deg
        nepi_msg.publishMsgInfo(self,"Driving to  " + "%.2f" % yaw_now_deg + " " + "%.2f" % self.pitch_goal_deg)
        self.gotoPositionCb(yaw_deg = yaw_now_deg, pitch_deg = self.pitch_goal_deg)
        

    def stopMovingHandler(self, _):
        self.stopMovingCb()
        self.yaw_goal_deg = self.status_msg.yaw_now_deg
        self.pitch_goal_deg = self.status_msg.pitch_now_deg
        nepi_msg.publishMsgInfo(self,"Stopping motion by request")
        

    def jogTimedYawHandler(self, msg):
        direction = msg.direction if self.reverse_yaw_control is False else (-1 * msg.direction)
        duration = 1000000.0 if (msg.duration_s < 0.0) else msg.duration_s
        self.moveYawCb(direction,  duration)
        nepi_msg.publishMsgInfo(self,"Jogging yaw")
        

    def jogTimedPitchHandler(self, msg):
        direction = msg.direction if self.reverse_pitch_control is False else (-1 * msg.direction)
        duration = 1000000.0 if (msg.duration_s < 0.0) else msg.duration_s
        self.movePitchCb(direction, duration)
        nepi_msg.publishMsgInfo(self,"Jogging pitch")
        

    def setReverseYawControl(self, msg):
        self.reverse_yaw_control = msg.data
        rpi = 1
        if msg.data:
            rpi = -1
        self.rpi = rpi
        nepi_msg.publishMsgInfo(self,"Set yaw control to reverse=" + str(self.reverse_yaw_control))
        

    def setReversePitchControl(self, msg):
        self.reverse_pitch_control = msg.data
        ryi = 1
        if msg.data:
            ryi = -1
        self.ryi = ryi
        nepi_msg.publishMsgInfo(self,"Set pitch control to reverse=" + str(self.reverse_pitch_control))
        

    def setHomePositionHereHandler(self, _):
        if self.setHomePositionHereCb is not None:
            # Driver supports it directly
            # Capture home position if possible
            if self.getCurrentPositionCb is not None:
                self.home_yaw_deg = self.status_msg.yaw_now_deg
                self.home_pitch_deg = self.status_msg.pitch_now_deg
            self.setHomePositionHereCb()
        else:
            nepi_msg.publishMsgWarn(self,"Instant home position not available for this device")
            return
        nepi_msg.publishMsgInfo(self,"Updated home position to current position")
        

    def setWaypointHandler(self, msg):
        yaw_deg = msg.yaw_deg
        pitch_deg = msg.pitch_deg
        waypoint_index = msg.waypoint_index
        if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
            nepi_msg.publishMsgWarn(self,"Requested waypoint position is invalid... ignoring")
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
            nepi_msg.publishMsgInfo(self,"Going to waypoint by command " + str(waypoint_index))
        
    
    def provideCapabilities(self, _):
        return self.capabilities_report
    

    def resetAppHandler(self,msg):
        self.resetParamServer()


    def resetParamServer(self):
        rospy.set_param('~status_update_rate_hz', self.factory_controls_dict['status_update_rate_hz'])

        rospy.set_param('~ptx/frame_id', self.frame_id)
        rospy.set_param("~ptx/yaw_joint_name", self.factory_controls_dict['frame_id'])
        rospy.set_param("~ptx/pitch_joint_name", self.factory_controls_dict['pitch_joint_name'])
        rospy.set_param("~ptx/reverse_yaw_control", self.factory_controls_dict['reverse_yaw_control'])
        rospy.set_param("~ptx/reverse_pitch_control", self.factory_controls_dict['reverse_pitch_control'])

        rospy.set_param('~ptx/capabilities/has_speed_control', self.capabilities_report.adjustable_speed)
        rospy.set_param('~ptx/capabilities/has_absolute_positioning', self.capabilities_report.absolute_positioning)
        rospy.set_param('~ptx/capabilities/has_homing', self.capabilities_report.homing)
        rospy.set_param('~ptx/capabilities/has_waypoints', self.capabilities_report.waypoints)

        if (self.capabilities_report.adjustable_speed is True):
            rospy.set_param("~ptx/speed_ratio", self.getSpeedCb()) # This one comes from the parent
        
        if (self.capabilities_report.absolute_positioning is True):
            rospy.set_param('~ptx/limits/max_yaw_hardstop_deg', self.defaultSettings['max_yaw_hardstop_deg'])
            rospy.set_param('~ptx/limits/min_yaw_hardstop_deg', self.defaultSettings['min_yaw_hardstop_deg'])
            rospy.set_param('~ptx/limits/max_pitch_hardstop_deg', self.defaultSettings['max_pitch_hardstop_deg'])
            rospy.set_param('~ptx/limits/min_pitch_hardstop_deg', self.defaultSettings['min_pitch_hardstop_deg'])
            rospy.set_param('~ptx/limits/max_yaw_softstop_deg', self.defaultSettings['max_yaw_hardstop_deg'])
            rospy.set_param('~ptx/limits/min_yaw_softstop_deg', self.defaultSettings['min_yaw_hardstop_deg'])
            rospy.set_param('~ptx/limits/max_pitch_softstop_deg', self.defaultSettings['max_pitch_hardstop_deg'])
            rospy.set_param('~ptx/limits/min_pitch_softstop_deg', self.defaultSettings['min_pitch_hardstop_deg'])

        if (self.capabilities_report.homing is True):
            rospy.set_param('~ptx/home_position/yaw_deg', 0.0)
            rospy.set_param('~ptx/home_position/pitch_deg', 0.0)



    def initializeParamServer(self):
        rospy.set_param('~status_update_rate_hz', self.status_update_rate)

        rospy.set_param('~ptx/frame_id', self.frame_id)
        rospy.set_param("~ptx/yaw_joint_name", self.yaw_joint_name)
        rospy.set_param("~ptx/pitch_joint_name", self.pitch_joint_name)
        rospy.set_param("~ptx/reverse_yaw_control", self.reverse_yaw_control)
        rospy.set_param("~ptx/reverse_pitch_control", self.reverse_pitch_control)

        rospy.set_param('~ptx/capabilities/has_speed_control', self.capabilities_report.adjustable_speed)
        rospy.set_param('~ptx/capabilities/has_absolute_positioning', self.capabilities_report.absolute_positioning)
        rospy.set_param('~ptx/capabilities/has_homing', self.capabilities_report.homing)
        rospy.set_param('~ptx/capabilities/has_waypoints', self.capabilities_report.waypoints)


        if (self.capabilities_report.adjustable_speed is True):
            rospy.set_param("~ptx/speed_ratio", self.getSpeedCb()) # This one comes from the parent
        
        if (self.capabilities_report.absolute_positioning is True):
            rospy.set_param('~ptx/limits/max_yaw_hardstop_deg', self.max_yaw_hardstop_deg)
            rospy.set_param('~ptx/limits/min_yaw_hardstop_deg', self.min_yaw_hardstop_deg)
            rospy.set_param('~ptx/limits/max_pitch_hardstop_deg', self.max_pitch_hardstop_deg)
            rospy.set_param('~ptx/limits/min_pitch_hardstop_deg', self.min_pitch_hardstop_deg)
            rospy.set_param('~ptx/limits/max_yaw_softstop_deg', self.max_yaw_softstop_deg)
            rospy.set_param('~ptx/limits/min_yaw_softstop_deg', self.min_yaw_softstop_deg)
            rospy.set_param('~ptx/limits/max_pitch_softstop_deg', self.max_pitch_softstop_deg)
            rospy.set_param('~ptx/limits/min_pitch_softstop_deg', self.min_pitch_softstop_deg)


        if (self.capabilities_report.homing is True):
            rospy.set_param('~ptx/home_position/yaw_deg', self.home_yaw_deg)
            rospy.set_param('~ptx/home_position/pitch_deg', self.home_pitch_deg)

    def updateFromParamServer(self):
        
        self.status_update_rate = rospy.get_param('~status_update_rate_hz', self.factory_controls_dict['status_update_rate_hz'])

        self.frame_id = rospy.get_param('~ptx/frame_id', self.frame_id)
        self.yaw_joint_name = rospy.get_param("~ptx/yaw_joint_name", self.factory_controls_dict['frame_id'])
        self.pitch_joint_name = rospy.get_param("~ptx/pitch_joint_name", self.factory_controls_dict['pitch_joint_name'])
        self.reverse_yaw_control = rospy.get_param("~ptx/reverse_yaw_control", self.factory_controls_dict['reverse_yaw_control'])
        self.reverse_pitch_control = rospy.get_param("~ptx/reverse_pitch_control", self.factory_controls_dict['reverse_pitch_control'])

       
        if (self.capabilities_report.absolute_positioning is True):
            self.max_yaw_hardstop_deg = rospy.get_param('~ptx/limits/max_yaw_hardstop_deg', self.defaultSettings['max_yaw_hardstop_deg'])
            self.min_yaw_hardstop_deg = rospy.get_param('~ptx/limits/min_yaw_hardstop_deg', self.defaultSettings['min_yaw_hardstop_deg'])
            self.max_pitch_hardstop_deg = rospy.get_param('~ptx/limits/max_pitch_hardstop_deg', self.defaultSettings['max_pitch_hardstop_deg'])
            self.max_pitch_softstop_deg = rospy.get_param('~ptx/limits/min_pitch_hardstop_deg', self.defaultSettings['min_pitch_hardstop_deg'])
            self.max_yaw_softstop_deg = rospy.get_param('~ptx/limits/max_yaw_softstop_deg', self.defaultSettings['max_yaw_hardstop_deg'])
            self.min_yaw_softstop_deg = rospy.get_param('~ptx/limits/min_yaw_softstop_deg', self.defaultSettings['min_yaw_hardstop_deg'])
            self.max_pitch_softstop_deg = rospy.get_param('~ptx/limits/max_pitch_softstop_deg', self.defaultSettings['max_pitch_hardstop_deg'])
            self.min_pitch_softstop_deg = rospy.get_param('~ptx/limits/min_pitch_softstop_deg', self.defaultSettings['min_pitch_hardstop_deg'])

        if (self.capabilities_report.homing is True):
            self.home_yaw_deg = rospy.get_param('~ptx/home_position/yaw_deg', 0.0)
            self.home_pitch_deg = rospy.get_param('~ptx/home_position/pitch_deg', 0.0)
