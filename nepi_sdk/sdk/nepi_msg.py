#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI msg utility functions

  
import time

from rospy import loginfo, logwarn, logdebug, logerr, logfatal
from rospy import loginfo_throttle, logwarn_throttle, logdebug_throttle, logerr_throttle, logfatal_throttle
from rospy import get_name, Publisher
from std_msgs.msg import String
from nepi_ros_interfaces.msg import Message



def printMsg(msg, level = "None"):
  msg_str = str(msg)
  level = level.lower()
  if level == 'debug':
    logdebug(msg_str)
  elif level == 'warn':
    logwarn(msg_str)
  elif level == 'error':
    logerr(msg_str)
  elif level == 'fatal':
    logfatal(msg_str)
  else:
    loginfo(msg_str)

def printMsgInfo(msg):
  msg_str = str(msg)
  loginfo(msg_str)

def printMsgWarn(msg):
  msg_str = str(msg)
  logwarn(msg_str)

def printMsgDebug(msg):
  msg_str = str(msg)
  logdebug(msg_str)

def printMsgErr(msg):
  msg_str = str(msg)
  logerr(msg_str)

def printMsgFatal(msg):
  msg_str = str(msg)
  logfatal(msg_str)


def printMsgThrottle(throttle_s,msg):
  msg_str = str(msg)
  loginfo_throttle(throttle_s,msg_str)

def printMsgInfoThrottle(throttle_s,msg):
  msg_str = str(msg)
  loginfo_throttle(throttle_s,msg_str)

def printMsgWarnThrottle(throttle_s,msg):
  msg_str = str(msg)
  logwarn_throttle(throttle_s,msg_str)

def printMsgDebugThrottle(throttle_s,msg):
  msg_str = str(msg)
  logdebug_throttle(throttle_s,msg_str)

def printMsgErrThrottle(throttle_s,msg):
  msg_str = str(msg)
  logerr_throttle(throttle_s,msg_str)

def printMsgFatalThrottle(throttle_s,msg):
  msg_str = str(msg)
  logfatal_throttle(throttle_s,msg_str)




  
def createMsgPublishers(self):
  self.node_str = get_name().split('/')[-1]
  self.msg_pub = Publisher("~messages", Message, queue_size=1)
  self.msg_pub_sys = Publisher("messages", Message, queue_size=1)
  time.sleep(1)
    
  
def publishMsg(self,msg,level = "None"):
  msg_str = (self.node_str + ": " + str(msg))
  printMsg(msg_str, level)
  self.msg_pub.publish(msg_str)
  self.msg_pub_sys.publish(msg_str)
  
def publishMsgInfo(self,msg):
  msg_str = str(msg)
  publishMsg(self,msg_str,level = 'Info')
   
def publishMsgWarn(self,msg):
  msg_str = str(msg)
  publishMsg(self,msg_str,level = 'Warn')
  
def publishMsgDebug(self,msg):
  msg_str = str(msg) 
  publishMsg(self,msg_str,level = 'Debut')
  
def publishMsgErr(self,msg):
  msg_str = str(msg)
  publishMsg(self,msg_str,level = 'Error')
  
def publishMsgFatal(self,msg):
  msg_str = str(msg) 
  publishMsg(self,msg_str,level = 'Fatal')


  



  
