/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import { observable, action } from "mobx"
import moment from "moment"
import ROS from "roslib"
import cannon from "cannon"
import yaml from "js-yaml"

const ROS_WS_URL = `ws://${window.location.hostname}:9090`
//const FLASK_URL = `http://${window.location.hostname}:5003`
const LICENSE_SERVER_WS_URL = `ws://${window.location.hostname}:9092`

const TRIGGER_MASKS = {
  OUTPUT_ENABLED: 0xffffffff,
  DEFAULT: 0x7fffffff
}

const EULER_ORDER_FOR_CANNON = "YZX" // Only supported angle ordering for Cannon library quaternion.toEuler() function

// TODO: Would be better to query the display_name property of all nodes to generate
// this dictionary... requires a new SDKNode service to do so
const NODE_DISPLAY_NAMES = {
  config_mgr: "Config Manager",
  nav_pose_mgr: "Nav./Pose/GPS",
  network_mgr: "Network",
  ai_detector_mgr: "Classifier",
  system_mgr: "System",
  time_sync_mgr: "Time Sync",
  trigger_mgr: "Triggering",
  nepi_link_ros_bridge: "NEPI Connect",
  gpsd_ros_client: "GPSD Client",
  illumination_mgr: "Illumination",
  automation_mgr: "Automation",
  app_image_sequencer: "Sequencer"
}

const UPDATE_PERIOD = 100 // ms between sending updates

function displayNameFromNodeName(node_name) {
  var display_name = NODE_DISPLAY_NAMES[node_name]
  if (display_name) {
    return display_name
  }
  return node_name
}

function nodeNameFromDisplayName(display_name) {
  for( var node_name in NODE_DISPLAY_NAMES ) {
    if (NODE_DISPLAY_NAMES[node_name] === display_name) {
      return node_name
    }
  }
  // Don't return anything if we don't find the display name -- callers can check for undefined
}

export { TRIGGER_MASKS, displayNameFromNodeName, nodeNameFromDisplayName }

/*
async function apiCall(endpoint) {
  try {
    const r = await fetch(`${FLASK_URL}/api/${endpoint}`, {
      method: "GET"
    })
    const json = await r.json()
    return json
  } catch (err) {
    console.error(err)
  }
}
*/

/*
// gets a file through the flask api and parses it as Json
async function getFileJson(filename) {
  try {
    const r = await fetch(`${FLASK_URL}/files/${filename}`, {
      method: "GET"
    })
    const json = await r.json()
    return json
  } catch (err) {
    console.error(err)
    return null
  }
}
*/

function getLocalTZ() {
  return Intl.DateTimeFormat().resolvedOptions().timeZone
}

//////////////////////////////////////////////////////////////
// from: https://stackoverflow.com/questions/7837456/how-to-compare-arrays-in-javascript
// Warn if overriding existing method
if (Array.prototype.equals)
  console.warn(
    "Overriding existing Array.prototype.equals. Possible causes: New API defines the method, there's a framework conflict or you've got double inclusions in your code."
  )
// attach the .equals method to Array's prototype to call it on any array
// eslint-disable-next-line no-extend-native
Array.prototype.equals = function(array) {
  // if the other array is a falsy value, return
  if (!array) return false

  // compare lengths - can save a lot of time
  if (this.length !== array.length) return false

  for (var i = 0, l = this.length; i < l; i++) {
    // Check if we have nested arrays
    if (this[i] instanceof Array && array[i] instanceof Array) {
      // recurse into the nested arrays
      if (!this[i].equals(array[i])) return false
    } else if (this[i] !== array[i]) {
      // Warning - two different object instances will never be equal: {x:20} != {x:20}
      return false
    }
  }
  return true
}
// Hide method from for-in loops
// eslint-disable-next-line no-extend-native
Object.defineProperty(Array.prototype, "equals", { enumerable: false })
//////////////////////////////////////////////////////////////

class ROSConnectionStore {
  @observable rosCheckStarted = false
  @observable connectedToROS = false
  @observable rosAutoReconnect = true
  @observable messageLog = ""
  rosListeners = []

  @observable namespacePrefix = null
  @observable deviceType = null
  @observable deviceId = null
  @observable deviceSerial = null
  @observable deviceInWater = false

  @observable systemDefs = null
  @observable systemDefsFirmwareVersion = null
  @observable systemDefsDiskCapacityMB = null

  @observable systemSoftwareStatus = null

  @observable systemStatus = null
  @observable heartbeat = false
  @observable systemInContainer = false
  @observable systemStatusDiskUsageMB = null
  @observable systemStatusDiskRate = null
  @observable systemStatusTempC = null
  @observable systemStatusWarnings = []
  @observable diskUsagePercent = null

  @observable systemStatusTime = null
  @observable clockUTCMode = false
  @observable clockTZ = getLocalTZ()
  @observable clockNTP = false
  @observable clockPPS = false

  @observable navPose = null
  @observable navPoseLocationLat = null
  @observable navPoseLocationLng = null
  @observable navPoseLocationAlt = null
  @observable navPoseDirectionHeadingDeg = null
  @observable navPoseDirectionSpeedMpS = null
  @observable navPoseOrientationYawAngle = null
  @observable navPoseOrientationYawRate = null
  @observable navPoseOrientationPitchAngle = null
  @observable navPoseOrientationPitchRate = null
  @observable navPoseOrientationRollAngle = null
  @observable navPoseOrientationRollRate = null
  @observable navPoseOrientationFrame = null
  @observable navPoseOrientationIsTransformed = false
  @observable navSatFixStatus = null
  @observable navSatFixService = null

  @observable navPoseStatus = null
  @observable gpsClockSyncEnabled = false
  @observable selectedNavSatFixTopic = null
  @observable lastNavSatFix = null
  @observable navSatFixRate = null
  @observable selectedOrientationTopic = null
  @observable lastOrientation = null
  @observable orientationRate = null
  @observable selectedHeadingTopic = null
  @observable lastHeading = null
  @observable headingRate = null  
  @observable navSrcFrame = null
  @observable navTargetFrame = null
  @observable navTransformXTrans = null
  @observable navTransformYTrans = null
  @observable navTransformZTrans = null
  @observable navTransformXRot = null
  @observable navTransformYRot = null
  @observable navTransformZRot = null
  @observable navHeadingOffset = null
  @observable transformNavPoseOrientation = null
    
  @observable imageRecognitions = []

  @observable triggerStatus = null
  @observable triggerAutoRateHz = 0
  @observable triggerMask = TRIGGER_MASKS.DEFAULT

  @observable saveFreqHz = 1.0

  @observable topicQueryLock = false
  @observable topicNames = null
  @observable topicTypes = null
  @observable appNames = []
  @observable appNamesLast = []
  @observable appNameList = []
  @observable appStatusList = []
  @observable imageTopics = []
  @observable imageDetectionTopics = []
  @observable pointcloudTopics = []
  @observable idxSensors = {}
  @observable settingCaps = {}
  @observable ptxUnits = {}
  @observable lsxUnits = {}
  @observable rbxRobots = {}
  @observable saveDataTopics = []
  @observable resetTopics = []
  @observable navSatFixTopics = []
  @observable orientationTopics = []
  @observable headingTopics = []
  @observable messageTopics = []

  @observable imageFilterDetection = null
  @observable imageFilterSequencer = null
  @observable imageFilterPTX = null
  
  @observable lastUpdate = new Date()

  @observable targLocalizerImgTopic = null

  @observable ip_query_response = null
  @observable bandwidth_usage_query_response = null
  @observable wifi_query_response = null
  /*
  @observable NUID = "INVALID"
  @observable NEPIConnectStatus = null
  @observable alias = ""
  @observable ssh_public_key = ""
  @observable bot_running = null
  @observable lb_last_connection_time = null
  @observable hb_last_connection_time = null
  @observable lb_do_msg_count = "1"
  @observable lb_dt_msg_count = null
  @observable hb_do_transfered_mb = null
  @observable hb_dt_transfered_mb = null
  @observable lb_data_sets_per_hour = null
  @observable lb_enabled = null
  @observable hb_enabled = null
  @observable lb_available_data_sources = null
  @observable lb_selected_data_sources = null
  @observable lb_comms_types = null
  @observable auto_attempts_per_hour = null
  @observable lb_data_queue_size_kb = null
  @observable hb_data_queue_size_mb = null
  @observable hb_auto_data_offloading_enabled = null
  @observable log_storage_enabled = null
  */

  @observable streamingImageQuality = 95
  @observable nepiLinkHbAutoDataOffloadingCheckboxVisible = false

  @observable scripts = []
  @observable running_scripts = []
  @observable launchScript = false
  @observable stopScript = false
  @observable systemStats = null
  @observable scriptForPolledStats = null

  @observable imgMuxSequences = null
  @observable drivers_list_query

  @observable license_server = null
  @observable license_valid = true // Default to true to avoid initial DEVELOPER message]
  @observable license_type = 'Unlicensed'
  @observable license_info = null
  @observable license_request_info = null
  @observable license_request_mode = false



  async checkLicense() {
    var retry_delay_ms = 3000
    if (!this.license_server) {
      try {
        this.license_server = new WebSocket(LICENSE_SERVER_WS_URL)

        this.license_server.onmessage = (event) => {
          var response_dict = yaml.load(event.data)         
          
          if ('licensed_components' in response_dict)
          {
            this.license_info = yaml.load(event.data)
            this.license_type = this.license_info['licensed_components']['nepi_base']['commercial_license_type']
            if ( this.license_type === 'Unlicensed') {
              this.license_valid = false
            }
            else {
              if (this.license_request_mode && !this.license_valid) {
                this.license_request_mode = false
              }
              this.license_valid = true
            }
          }

          else if ('license_request' in response_dict)
          {
            this.license_request_info = yaml.load(event.data)
          }
        }

        retry_delay_ms = 250 // Fast to avoid a lot of latency while connecting
      } catch (e) {
        // Note: Failure to contact the server does not result in this exception, instead
        // an event is dispatched, so we don't get into this block just because the
        // server isn't present: 
        // https://stackoverflow.com/questions/31002592/javascript-doesnt-catch-error-in-websocket-instantiation
        //console.error(e)
        console.error("License server not running")
        this.license_server = null
        this.license_yaml = null
        this.license_valid = false 
        this.license_info = null
        this.license_request_info = null
        this.license_request_mode = false
      }
    }

    else if (this.license_server.readyState === 1) { // READY
      // Check for license updates
      this.license_server.send("license_check") 
      retry_delay_ms = 5000 // Slow down the updates now that we are connected
    }

    else if (this.license_server.readyState === 3) { // CLOSED
      this.license_server = null
      this.license_valid = false
      this.license_info = null
      this.license_request_info = null
      this.license_request_mode = false
    }

    setTimeout(async () => {
      await this.checkLicense()
    }, retry_delay_ms)
  }

  async checkROSConnection() {
    this.rosCheckStarted = true
    if (!this.connectedToROS) {
      try {
        // setup rosbridge connection
        if (!this.ros || !this.connectedToROS) {
          this.ros = new ROS.Ros({
            url: ROS_WS_URL
          })
          this.ros.on("connection", this.onConnectedToROS)
          this.ros.on("error", this.onErrorConnectingToROS)
          this.ros.on("close", this.onDisconnectedToROS)
        } else {
          this.ros.connect(ROS_WS_URL)
        }
      } catch (e) {
        console.error(e)
      }
    }

    // update the topics periodically
    this.updateTopics()

    if (this.rosAutoReconnect) {
      setTimeout(async () => {
        await this.checkROSConnection()
      }, 1000)
    }
  }

  @action.bound
  updateTopics() {
    // topicQueryLock is used so we don't call getTopics many times
    // while witing for it to return.  With many topics on a slow
    // target it takes a few seconds to retrun.
    if (this.ros && !this.connectedToROS && !this.rosCheckStarted){
      this.checkROSConnection()
    }
    else if (this.ros && !this.topicQueryLock && this.connectedToROS) {
      this.topicQueryLock = true
      this.ros.getTopics(result => {
        this.topicNames = result.topics
        this.topicTypes = result.types
        var newPrefix = this.updatePrefix()
        var newResetTopics = this.updateResetTopics()
        var newSaveDataTopics = this.updateSaveDataTopics()
        var newImageTopics = this.updateImageTopics()
        var newMessageTopics = this.updateMessageTopics()
        var newPointcloudTopics = this.updatePointcloudTopics()
        this.updateAppStatusList()
        this.updateIDXSensorList()
        this.updatePTXUnits()
		    this.updateLSXUnits()
        this.updateRBXRobotsList()
        this.updateNavPoseSourceTopics()

        if (newPrefix || newResetTopics || newSaveDataTopics || newMessageTopics || newImageTopics || newPointcloudTopics) {
          this.initalizeListeners()
        }
        this.topicQueryLock = false
      })
    }
  }

  validPrefix() {
    return this.namespacePrefix && this.deviceId
  }

  @action.bound
  updatePrefix() {
    // Function for testing if we need to update the device prefix variables.
    // It loops though the topics and uses the testTopicForPrefix to test and
    // perform the update.  If it updates we are done, break.

    // we return true if the prefix was updated
    var ret = false
    for (var i = 0; i < this.topicNames.length; i++) {
      var topic_name_parts = this.topicNames[i].split("/")
      if (
        topic_name_parts[topic_name_parts.length - 1] === "system_status" &&
        this.topicTypes[i] === "nepi_ros_interfaces/SystemStatus"
      ) {
        if (
          this.namespacePrefix !== topic_name_parts[1] &&
          this.deviceId !== topic_name_parts[2]
        ) {
          this.namespacePrefix = topic_name_parts[1]
          this.deviceId = topic_name_parts[2]
          if (this.validPrefix()) {
            ret = true
            this.rosLog(
              `Fetched device info ${this.namespacePrefix}/${this.deviceId}`
            )
          }
        }
      }
    }
    return ret
  }

  @action.bound
  updateMessageTopics() {
    // Function for updating image topics list
    var newMessageTopics = []
    for (var i = 0; i < this.topicNames.length; i++) {
      if (this.topicTypes[i] === "nepi_ros_interfaces/Message") {
        newMessageTopics.push(this.topicNames[i])
      }
    }

    // sort the image topics for comparison to work
    newMessageTopics.sort()

    if (!this.messageTopics.equals(newMessageTopics)) {
      this.messageTopics = newMessageTopics
      return true
    } else {
      return false
    }
  }


  @action.bound
  updateImageTopics() {
    // Function for updating image topics list
    var newImageTopics = []
    var newImageDetectionTopics = []
    for (var i = 0; i < this.topicNames.length; i++) {
      if (this.topicTypes[i] === "sensor_msgs/Image" && this.topicNames[i].indexOf("zed_node") === -1) {
        newImageTopics.push(this.topicNames[i])
        if (this.topicNames[i].indexOf('detection_image') !== -1){
          newImageDetectionTopics.push(this.topicNames[i])
        }
      }
    }

    // sort the image topics for comparison to work
    newImageTopics.sort()

    if (!this.imageTopics.equals(newImageTopics)) {
      this.imageTopics = newImageTopics
      this.imageDetectionTopics = newImageDetectionTopics
      return true
    } else {
      return false
    }
  }


  @action.bound
  updatePointcloudTopics() {
    // Function for updating image topics list
    var newPointcloudTopics = []
    for (var i = 0; i < this.topicNames.length; i++) {
      if (this.topicTypes[i] === "sensor_msgs/PointCloud2") {
        newPointcloudTopics.push(this.topicNames[i])
      }
    }

    // sort the image topics for comparison to work
    newPointcloudTopics.sort()

    if (!this.pointcloudTopics.equals(newPointcloudTopics)) {
      this.pointcloudTopics = newPointcloudTopics
      return true
    } else {
      return false
    }
  }

  @action.bound
  updateAppStatusList() {
    const appNames = this.appNames
    const appNamesLast = this.appNamesLast
    if (appNames.length > 0 && appNames !== appNamesLast) {
      for (var i = 0; i < appNames.length; i++) {
          this.callAppStatusQueryService(appNames[i])
      }
      this.appNamesLast = appNames
    }

  }

  @action.bound
  updateIDXSensorList() {
    var idx_sensors_changed = false
    var sensors_detected = []
    for (var i = 0; i < this.topicNames.length; i++) {
      if (this.topicNames[i].endsWith("/idx/status")) {
        const idx_sensor_namespace = this.topicNames[i].split("/idx")[0]
        if (!(sensors_detected.includes(idx_sensor_namespace))) {
          this.callIDXCapabilitiesQueryService(idx_sensor_namespace) // Testing
          this.callSettingsCapabilitiesQueryService(idx_sensor_namespace)
          const idxSensor = this.idxSensors[idx_sensor_namespace]
          const settingCaps = this.settingCaps[idx_sensor_namespace]
          if (idxSensor && settingCaps) { // Testing
            sensors_detected.push(idx_sensor_namespace)
          }
          idx_sensors_changed = true // Testing -- always declare changed
        }
      }
    }

    // Now clean out any sensors that are no longer detected
    const previously_known = Object.keys(this.idxSensors)
    for (i = 0; i < previously_known.length; ++i) {
      if (!(sensors_detected.includes(previously_known[i]))) {
        delete this.idxSensors[previously_known[i]]
        idx_sensors_changed = true
      }
    }
    return idx_sensors_changed
  }

  @action.bound
  updatePTXUnits() {
    var ptx_units_changed = false
    var ptx_units_detected = []
    for (var i = 0; i < this.topicNames.length; i++) {
      if (this.topicNames[i].endsWith("/ptx/status")) {
        const ptx_unit_namespace = this.topicNames[i].split("/ptx")[0]
        if (!(ptx_units_detected.includes(ptx_unit_namespace))) {
          this.callPTXCapabilitiesQueryService(ptx_unit_namespace)
          this.callSettingsCapabilitiesQueryService(ptx_unit_namespace)
          const ptxUnit = this.ptxUnits[ptx_unit_namespace]
          const settingCaps = this.settingCaps[ptx_unit_namespace]
          if (ptxUnit)
          {
            ptx_units_detected.push(ptx_unit_namespace)
          }
        }
        ptx_units_changed = true
      }
    }

    // Now clean out any units that are no longer detected
    const previously_known = Object.keys(this.ptxUnits)
    for (i = 0; i < previously_known.length; ++i) {
      if (!(ptx_units_detected.includes(previously_known[i]))) {
        delete this.ptxUnits[previously_known[i]]
        ptx_units_changed = true
      }
    }
    return ptx_units_changed
  }
  
  @action.bound
  updateLSXUnits() {
    var lsx_units_changed = false
    var lsx_units_detected = []
    for (var i = 0; i < this.topicNames.length; i++) {
      if (this.topicNames[i].endsWith("/lsx/status")) {
        const lsx_unit_namespace = this.topicNames[i].split("/lsx")[0]
        if (!(lsx_units_detected.includes(lsx_unit_namespace))) {
          this.callLSXCapabilitiesQueryService(lsx_unit_namespace)
          this.callSettingsCapabilitiesQueryService(lsx_unit_namespace)
          const lsxUnits = this.lsxUnits[lsx_unit_namespace]
          const settingCaps = this.settingCaps[lsx_unit_namespace]
          if (lsxUnits && settingCaps)
          {
            lsx_units_detected.push(lsx_unit_namespace)
          }
        }
        lsx_units_changed = true
      }
    }

    // Now clean out any units that are no longer detected
    const previously_known = Object.keys(this.lsxUnits)
    for (i = 0; i < previously_known.length; ++i) {
      if (!(lsx_units_detected.includes(previously_known[i]))) {
        delete this.lsxUnits[previously_known[i]]
        lsx_units_changed = true
      }
    }
    return lsx_units_changed
  }  
  
  @action.bound
  updateRBXRobotsList() {
    var rbx_robots_changed = false
    var robots_detected = []
    for (var i = 0; i < this.topicNames.length; i++) {
      if (this.topicNames[i].endsWith("/rbx/status")) {
        const rbx_robot_namespace = this.topicNames[i].split("/rbx")[0]
        if (!(robots_detected.includes(rbx_robot_namespace))) {
          this.callRBXCapabilitiesQueryService(rbx_robot_namespace)
          this.callSettingsCapabilitiesQueryService(rbx_robot_namespace)
          const rbxRobot = this.rbxRobots[rbx_robot_namespace]
          const settingCaps = this.settingCaps[rbx_robot_namespace]
          if (rbxRobot && settingCaps) { // Testing
            robots_detected.push(rbx_robot_namespace)
          }
          rbx_robots_changed = true // Testing -- always declare changed
        }
      }
    }

    // Now clean out any sensors that are no longer detected
    const previously_known = Object.keys(this.rbxRobots)
    for (i = 0; i < previously_known.length; ++i) {
      if (!(robots_detected.includes(previously_known[i]))) {
        delete this.rbxRobots[previously_known[i]]
        rbx_robots_changed = true
      }
    }
    return rbx_robots_changed
  }


  @action.bound
  updateResetTopics() {
    var newResetTopics = []
    for (var i = 0; i < this.topicNames.length; i++) {
      var topic_name_parts = this.topicNames[i].split("/")
      var last_element = topic_name_parts.pop()
      var topic_base = topic_name_parts.join("/")
      if (
        last_element === "reset" &&
        this.topicTypes[i] === "nepi_ros_interfaces/Reset"
      ) {
        newResetTopics.push(topic_base)
      }
    }

    // sort the topics for comparison to work
    newResetTopics.sort()

    if (!this.resetTopics.equals(newResetTopics)) {
      this.resetTopics = newResetTopics
      return true
    } else {
      return false
    }
  }

  @action.bound
  updateSaveDataTopics() {
    var newSaveDataTopics = []
    var topic = ""
    for (var i = 0; i < this.topicNames.length; i++) {
      topic = this.topicNames[i]
      if (topic.indexOf('save_data_status') !== -1){
        newSaveDataTopics.push(topic) 
      }
    }

    // sort the topics for comparison to work
    newSaveDataTopics.sort()

    if (!this.saveDataTopics.equals(newSaveDataTopics)) {
      this.saveDataTopics = newSaveDataTopics
      return true
    } else {
      return false
    }
  }

  @action.bound
  updateNavPoseSourceTopics() {
    // Function for updating image topics list
    var newNavSatFixTopics = []
    var newOrientationTopics = []
    var newHeadingTopics = []
    for (var i = 0; i < this.topicNames.length; i++) {
      if (this.topicTypes[i] === "sensor_msgs/NavSatFix") {
        newNavSatFixTopics.push(this.topicNames[i])
      }
      else if ((this.topicTypes[i] === "nav_msgs/Odometry") || (this.topicTypes[i] === "sensor_msgs/Imu")) {
        newOrientationTopics.push(this.topicNames[i])
      }
      else if (this.topicTypes[i] === "std_msgs/Float64") {
        // Float64 ==> Not a very good differentiator!
        newHeadingTopics.push(this.topicNames[i])
      }
    }

    var topicsChanged = false
    newNavSatFixTopics.sort()
    if (!this.navSatFixTopics.equals(newNavSatFixTopics)) {
      this.navSatFixTopics = newNavSatFixTopics
      topicsChanged = true
    }

    newOrientationTopics.sort()
    if (!this.orientationTopics.equals(newOrientationTopics)) {
      this.orientationTopics = newOrientationTopics
      topicsChanged = true
    }

    newHeadingTopics.sort()
    if (!this.headingTopics.equals(newHeadingTopics)) {
      this.headingTopics = newHeadingTopics
      topicsChanged = true
    }

    return topicsChanged
  }

  @action.bound
  destroyROSConnection() {
    this.rosAutoReconnect = false
    this.ros.off("connection", this.onConnectedToROS)
    this.ros.off("error", this.onErrorConnectingToROS)
    this.ros.off("close", this.onDisconnectedToROS)

    this.rosListeners.forEach(listener => {
      listener.unsubscribe()
    })
  }

  @action.bound
  rosLog(text) {
    this.messageLog = `${text}\n${this.messageLog}`
  }

  get rosPrefix() {
    return `/${this.namespacePrefix}/${this.deviceId}`
  }

  publishMessage({ name, messageType, data, noPrefix = false }) {
    const publisher = new ROS.Topic({
      ros: this.ros,
      name: noPrefix ? name : `${this.rosPrefix}/${name}`,
      messageType
    })
    const message = new ROS.Message(data)
    publisher.publish(message)
  }

  addListener({
    name,
    messageType,
    callback,
    noPrefix = false,
    manageListener = true
  }) {
    const listener = new ROS.Topic({
      ros: this.ros,
      name: noPrefix ? name : `${this.rosPrefix}/${name}`,
      messageType
    })
    listener.subscribe(action(callback))

    // add to listeners that get unsubscribed
    if (manageListener) {
      this.rosListeners.push(listener)
    }

    // return listener for clients that manage their own
    return listener
  }

  callService({ name, messageType, args = null, msgKey = null }) {
    return new Promise(resolve => {
      const client = new ROS.Service({
        ros: this.ros,
        name: name.startsWith('/')? name : `${this.rosPrefix}/${name}`,
        serviceType: messageType
      })
      const request = new ROS.ServiceRequest(args)
      client.callService(
        request,
        action(result => {
          resolve(msgKey ? result[msgKey] : result)
        })
      )
    })
  }

  @action.bound
  onConnectedToROS() {
    this.connectedToROS = true
    this.rosLog("Connected to NEPI device")
    this.checkLicense()
  }

  @action.bound
  initalizeListeners() {
    this.rosListeners.forEach(listener => {
      listener.unsubscribe()
    })

    // listeners
    this.setupImageSystemStatusListener()
    this.setupRUISettingsListener()

    // services
    this.callSystemDefsService()
    this.callSystemSoftwareStatusQueryService()
    this.startPollingIPAddrQueryService()
    this.startPollingBandwidthUsageService()
    this.startPollingWifiQueryService()
    this.startPollingOpEnvironmentQueryService()
    this.startPollingNavPoseService()
    this.startPollingNavPoseStatusService()
    this.startPollingTimeStatusService()
    
    // automation manager services
    this.startPollingGetScriptsService()  // populate listbox with files
    this.startPollingGetRunningScriptsService()  // populate listbox with active files
    //this.startPollingLaunchScriptService() // invoke script execution
    //this.startPollingStopScriptService() // stop script execution
    //this.callGetSystemStatsQueryService() // get script and system status

    // sequential image mux services
    //this.callMuxSequenceQuery(true) // Start it polling


  }

  @action.bound
  onErrorConnectingToROS() {
    this.connectedToROS = false
    this.rosLog("Error connecting to NEPI device, retrying")
  }

  @action.bound
  onDisconnectedToROS() {
    this.connectedToROS = false

    this.namespacePrefix = null
    this.deviceType = null
    this.deviceId = null
    this.deviceSerial = null

    this.rosLog("Connection to NEPI device closed")
  }


  /*******************************/
  // Generic Listener Functions
  /*******************************/

  @action.bound
  setupStatusListener(namespace, msg_type, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: msg_type,
        noPrefix: true,
        callback: callback,
        manageListener: false
      })
    }
  }

  @action.bound
  setupStringListener(namespace,callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: "std_msgs/String",
        noPrefix: true,
        callback: callback,
        manageListener: false
      })
    }
  }

  @action.bound
  setupVector3Listener(namespace, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: "geometry_msgs/Vector3",
        noPrefix: true,
        callback: callback,
        manageListener: false
      })
    }
  }

  @action.bound
  setupFloatListener(namespace, callback) {
    if (namespace) {
      return this.addListener({
        name: namespace,
        messageType: "std_msgs/Float32",
        noPrefix: true,
        callback: callback,
        manageListener: false
      })
    }
  }

  @action.bound
  setupImageSystemStatusListener() {
    this.addListener({
      name: "system_status",
      messageType: "nepi_ros_interfaces/SystemStatus",
      callback: message => {
        // turn heartbeat on for half a second
        this.heartbeat = true
        setTimeout(() => {
          this.heartbeat = false
        }, 500)
        this.systemStatus = message
        this.systemInContainer = message.in_container
        this.systemStatusDiskUsageMB = message.disk_usage
        this.systemStatusDiskRate = message.storage_rate

        if (this.clockUTCMode) {
          this.systemStatusTime = moment.unix(message.sys_time.secs).utc()
        } else {
          this.systemStatusTime = moment.unix(message.sys_time.secs)
        }

        this.diskUsagePercent = `${parseInt(
          100 * this.systemStatusDiskUsageMB / this.systemDefsDiskCapacityMB,
          10
        )}%`

        this.systemStatusTempC =
          message.temperatures.length && message.temperatures[0]
        this.systemStatusWarnings = message.warnings && message.warnings.flags
        //this.rosLog("Received Status Message:")
        var i
        for(i in message.info_strings) {
          this.rosLog(message.info_strings[i].payload)
        }
      }
    })
  }



  /*******************************/
  // Custom Listener Functions
  /*******************************/

  @action.bound
  setupPTXStatusListener(ptxNamespace, callback) {
    if (ptxNamespace) {
      return this.addListener({
        name: ptxNamespace + "/ptx/status",
        messageType: "nepi_ros_interfaces/PanTiltStatus",
        noPrefix: true,
        callback: callback,
        manageListener: false
      })
    }
  }
    
  @action.bound
  setupLSXStatusListener(lsxNamespace, callback) {
    if (lsxNamespace) {
      return this.addListener({
        name: lsxNamespace + "/lsx/status",
        messageType: "nepi_ros_interfaces/LSXStatus",
        noPrefix: true,
        callback: callback,
        manageListener: false
      })
    }
  }

  setupRUISettingsListener() {
    this.addListener({
      name: "rui_config_mgr/settings",
      messageType: "nepi_ros_interfaces/RUISettings",
      callback: message => {
        this.streamingImageQuality = message.streaming_image_quality
        this.nepiLinkHbAutoDataOffloadingCheckboxVisible = message.nepi_hb_auto_offload_visible
      }
    })
  }


  @action.bound
  setupIDXStatusListener(idxSensorNamespace, callback) {
    if (idxSensorNamespace) {
      return this.addListener({
        name: idxSensorNamespace + "/idx/status",
        messageType: "nepi_ros_interfaces/IDXStatus",
        noPrefix: true,
        callback: callback,
        manageListener: false
      })
    }
  }





  @action.bound
  setupSaveDataStatusListener(saveNamespace, callback) {
    if (saveNamespace) {
      return this.addListener({
        name: saveNamespace + "/save_data_status",
        messageType: "nepi_ros_interfaces/SaveDataStatus",
        noPrefix: true,
        callback: callback,
        manageListener: false
      })
    }
  }

  @action.bound
  setupSettingsStatusListener(settingsNamespace, callback) {
    if (settingsNamespace) {
      return this.addListener({
        name: settingsNamespace + "/settings_status",
        messageType: "nepi_ros_interfaces/Settings",
        noPrefix: true,
        callback: callback,
        manageListener: false
      })
    }
  }



  /*******************************/
  // Generic Send Data Functions
  /*******************************/
  @action.bound
  sendTriggerMsg(namespace) {
    this.publishMessage({
      name: namespace,
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    })
  }

  @action.bound
  sendBoolMsg(namespace, value) {
    this.publishMessage({
      name: namespace,
      messageType: "std_msgs/Bool",
      data: {data: value},
      noPrefix: true
    })
    
  }

  @action.bound
  sendStringMsg(strNamespace,str) {
    const namespace = strNamespace
    this.publishMessage({
      name: namespace,
      messageType: "std_msgs/String",
      data: {'data':str},
      noPrefix: true
    })
  }

  @action.bound
  sendStringArrayMsg(strNamespace,strArray) {
    const namespace = strNamespace
    this.publishMessage({
      name: namespace,
      messageType: "nepi_ros_interfaces/StringArray",
      data: {'entries':strArray},
      noPrefix: true
    })
  }


  @action.bound
  sendIntMsg(namespace, int_str) {
    let intVal = parseInt(int_str, 10)
    if (!isNaN(intVal)) {
      this.publishMessage({
        name: namespace,
        messageType: "std_msgs/Int32",
        data: {data: intVal},
        noPrefix: true
      })
    }
  }

  @action.bound
  sendInt8Msg(namespace, int_str) {
    let intVal = parseInt(int_str, 10)
    if (!isNaN(intVal)) {
      this.publishMessage({
        name: namespace,
        messageType: "std_msgs/Int8",
        data: {data: intVal},
        noPrefix: true
      })
    }
  }

  @action.bound
  sendFloatMsg(namespace, float_str) {
    let floatVal = parseFloat(float_str)
    if (!isNaN(floatVal)) {
      this.publishMessage({
        name: namespace,
        messageType: "std_msgs/Float32",
        data: {data: floatVal},
        noPrefix: true
      })
    }
  }


  @action.bound
  sendErrorBoundsMsg(namespace, max_m,max_d,min_stab) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_ros_interfaces/RBXErrorBounds",
      data: { 
        data: {
          max_distance_error_m: max_m,
          max_rotation_error_deg: max_d,
          min_stabilize_time_s: min_stab
        }
      },
      noPrefix: true
    })
  }

  @action.bound
  sendUpdateOptionMsg(namespace, comp_name, option_str) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_ros_interfaces/UpdateOption",

        data: {
        name: comp_name,
        option_str: option_str
      },
      noPrefix: true
    })
  }

  @action.bound
  sendUpdateOrderMsg(namespace, comp_name, move_cmd) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_ros_interfaces/UpdateOrder",
      data: {    
        name: comp_name,
        move_cmd: move_cmd
      },
      noPrefix: true
    })
  }

  @action.bound
  sendUpdateActiveStateMsg(namespace, comp_name, active_state) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_ros_interfaces/UpdateState",
      data: {
        name: comp_name,
        active_state: active_state
      },
      noPrefix: true
    })
  }

  @action.bound
  sendImageSelectionMsg(namespace, image_index, image_topic) {
    this.publishMessage({
      name: namespace,
      messageType: "nepi_ros_interfaces/ImageSelection",
      data: {
        image_index: image_index,
        image_topic: image_topic
      },
      noPrefix: true
    })
  }

  @action.bound
  sendFrame3DTransformUpdateMsg(namespace, transformNamespace, transformFloatList) {
    if (transformFloatList.length === 7){
      this.publishMessage({
        name: namespace,
        messageType: "nepi_ros_interfaces/Frame3DTransformUpdate",
        data: { 
          topic_namespace: transformNamespace,
          transform: {
            translate_vector: {
              x: transformFloatList[0],
              y: transformFloatList[1],
              z: transformFloatList[2]
            },
            rotate_vector: {
              x: transformFloatList[3],
              y: transformFloatList[4],
              z: transformFloatList[5]
            },
            heading_offset: transformFloatList[6]
          }
        },
        noPrefix: true
      })
    }
  }

  @action.bound
  sendFrame3DTransformMsg(namespace, transformFloatList) {
    if (transformFloatList.length === 7){
      this.publishMessage({
        name: namespace,
        messageType: "nepi_ros_interfaces/Frame3DTransform",
        data: { 
            translate_vector: {
              x: transformFloatList[0],
              y: transformFloatList[1],
              z: transformFloatList[2]
            },
            rotate_vector: {
              x: transformFloatList[3],
              y: transformFloatList[4],
              z: transformFloatList[5]
            },
            heading_offset: transformFloatList[6]

        },
        noPrefix: true
      })
    }
  }

  @action.bound
  sendClearFrame3DTransformMsg(namespace, transformFloatList) {
    if (transformFloatList.length === 7){
      this.publishMessage({
        name: namespace,
        messageType: "nepi_ros_interfaces/Frame3DTransform",
        data: { 
            translate_vector: {
              x: transformFloatList[0],
              y: transformFloatList[1],
              z: transformFloatList[2]
            },
            rotate_vector: {
              x: transformFloatList[3],
              y: transformFloatList[4],
              z: transformFloatList[5]
            },
            heading_offset: transformFloatList[6]

        },
        noPrefix: true
      })
    }
  }

  @action.bound
  sendFloatVector3Msg(namespace, float1_str,float2_str,float3_str) {
    let float1Val = parseFloat(float1_str)
    let float2Val = parseFloat(float2_str)
    let float3Val = parseFloat(float3_str)
    if (!isNaN(float1Val) && !isNaN(float2Val) && !isNaN(float3Val)) {
      this.publishMessage({
        name: namespace,
        messageType: "geometry_msgs/Vector3",
        data: { 
          x: float1Val,
          y: float2Val,
          z: float3Val
        },
        noPrefix: true
      })
    }
  }


  @action.bound
  sendGeoPointMsg(namespace, lat_str,long_str,alt_str) {
    let latVal = parseFloat(lat_str)
    let longVal = parseFloat(long_str)
    let altVal = parseFloat(alt_str)
    if (!isNaN(latVal) && !isNaN(longVal) && !isNaN(altVal)) {
      this.publishMessage({
        name: namespace,
        messageType: "geographic_msgs/GeoPoint",
        data: { 
          latitude: latVal,
          longitude: longVal,
          altitude: altVal
        },
        noPrefix: true
      })
    }
  }

  @action.bound
  sendFloatGotoPoseMsg(namespace, float1_str,float2_str,float3_str) {
    let float1Val = parseFloat(float1_str)
    let float2Val = parseFloat(float2_str)
    let float3Val = parseFloat(float3_str)
    if (!isNaN(float1Val) && !isNaN(float2Val) && !isNaN(float3Val)) {
      this.publishMessage({
        name: namespace,
        messageType: "nepi_ros_interfaces/RBXGotoPose",
        data: { 
          roll_deg: float1Val,
          pitch_deg: float2Val,
          yaw_deg: float3Val
        },
        noPrefix: true
      })
    }
  }

  @action.bound
  sendFloatGotoPositionMsg(namespace, float1_str,float2_str,float3_str,float4_str) {
    let float1Val = parseFloat(float1_str)
    let float2Val = parseFloat(float2_str)
    let float3Val = parseFloat(float3_str)
    let float4Val = parseFloat(float4_str)
    if (!isNaN(float1Val) && !isNaN(float2Val) && !isNaN(float3Val) && !isNaN(float4Val)) {
      this.publishMessage({
        name: namespace,
        messageType: "nepi_ros_interfaces/RBXGotoPosition",
        data: { 
          x_meters: float1Val,
          y_meters: float2Val,
          z_meters: float3Val,
          yaw_deg: float4Val
        },
        noPrefix: true
      })
    }
  }

  @action.bound
  sendFloatGotoLocationMsg(namespace, float1_str,float2_str,float3_str,float4_str) {
    let float1Val = parseFloat(float1_str)
    let float2Val = parseFloat(float2_str)
    let float3Val = parseFloat(float3_str)
    let float4Val = parseFloat(float4_str)
    if (!isNaN(float1Val) && !isNaN(float2Val) && !isNaN(float3Val) && !isNaN(float4Val)) {
      this.publishMessage({
        name: namespace,
        messageType: "nepi_ros_interfaces/RBXGotoLocation",
        data: { 
          lat: float1Val,
          long: float2Val,
          altitude_meters: float3Val,
          yaw_deg: float4Val
        },
        noPrefix: true
      })
    }
  }

  /*******************************/
  // Custom Send Data Functions
  /*******************************/


  @action.bound
  updateSaveDataPrefix(saveDataNamespace,saveDataPrefix) {
    const namespace = saveDataNamespace + "/save_data_prefix"
    this.publishMessage({
      name: namespace,
      messageType: "std_msgs/String",
      data: {'data':saveDataPrefix},
      noPrefix: true
    })
  }

  @action.bound
  updateSaveDataRate(saveDataNamespace,data_product,rate_hz) {
    this.publishMessage({
      name: saveDataNamespace + "/save_data_rate",
      messageType: "nepi_ros_interfaces/SaveDataRate",
      data: {
        data_product: data_product,
        save_rate_hz: rate_hz,
      },
      noPrefix: true
    })
  }

  @action.bound
  updateSaveDataEnable(saveDataNamespace,checked) {
     this.publishMessage({
      name: saveDataNamespace + "/save_data",
      messageType: "nepi_ros_interfaces/SaveData",
      data: {
        save_continuous: checked,
        save_raw: false
      },
      noPrefix: true
    })     
  }

  @action.bound
  onSnapshotEventTriggered(saveDataNamespace) {
    this.publishMessage({
      name: saveDataNamespace + "/snapshot_trigger",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    })
  }

  @action.bound
  resetSaveDataTriggered(saveDataNamespace) {
    this.publishMessage({
      name: saveDataNamespace + "/save_data_reset",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    })
  }


 




  @action.bound
  updateSetting(settingsNamespace,nameStr,typeStr,valueStr) {
    this.publishMessage({
      name: settingsNamespace + "/update_setting",
      messageType: "nepi_ros_interfaces/Setting",
      data: {type_str:typeStr,
        name_str:nameStr,
        value_str:valueStr
      },
      noPrefix: true
    })
  }



 

  @action.bound
  saveConfigTriggered(namespace) {
    this.publishMessage({
      name: namespace + "/save_config",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    })
  }


  @action.bound
  resetIdxFactoryTriggered(idxSensorNamespace) {
    this.publishMessage({
      name: idxSensorNamespace + "/idx/reset_factory",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    })
  }




  @action.bound
  setIdxControlsEnable(idxSensorNamespace,idxControls) {
     this.publishMessage({
      name: idxSensorNamespace + "/idx/set_controls_enable",
      messageType: "std_msgs/Bool",
      data: {'data':idxControls},
      noPrefix: true
    })     
  }


  @action.bound
  setIdxAutoAdjust(idxSensorNamespace,autoAdjust) {
     this.publishMessage({
      name: idxSensorNamespace + "/idx/set_auto_adjust",
      messageType: "std_msgs/Bool",
      data: {'data':autoAdjust},
      noPrefix: true
    })     
  }
  
  @action.bound
  setFrame3D(namespace, value) {
      this.publishMessage({
        name: namespace + '/set_frame_3d',
        messageType: "std_msgs/String",
        data: {'data':value},
        noPrefix: true
      })     
    }


  async callSystemDefsService() {
    this.systemDefs = await this.callService({
      name: "system_defs_query",
      messageType: "nepi_ros_interfaces/SystemDefs",
      msgKey: "defs"
    })
    this.deviceType = this.systemDefs.device_type
    this.deviceSerial = this.systemDefs.device_sn
    this.systemDefsFirmwareVersion = this.systemDefs.firmware_version
    this.systemDefsDiskCapacityMB = this.systemDefs.disk_capacity
  }

  @action.bound
  async callSettingsCapabilitiesQueryService(namespace) {
    const capabilities = await this.callService({
      name: namespace + "/settings_capabilities_query",
      messageType: "nepi_ros_interfaces/SettingsCapabilitiesQuery",  
      args: {namespace : namespace},
    })
    this.settingCaps[namespace] = capabilities
  }


  @action.bound
  async callIDXCapabilitiesQueryService(idxSensorNamespace) {
    const capabilities = await this.callService({
      name: idxSensorNamespace + "/idx/capabilities_query",
      messageType: "nepi_ros_interfaces/IDXCapabilitiesQuery",  
    })
    this.idxSensors[idxSensorNamespace] = capabilities
  }


  @action.bound
  async callPTXCapabilitiesQueryService(ptxUnitNamespace) {
    const capabilities = await this.callService({
      name: ptxUnitNamespace + "/ptx/capabilities_query",
      messageType: "nepi_ros_interfaces/PTXCapabilitiesQuery",
    })
    this.ptxUnits[ptxUnitNamespace] = capabilities
  }

  @action.bound
  async callLSXCapabilitiesQueryService(lsxUnitNamespace) {
    const capabilities = await this.callService({
      name: lsxUnitNamespace + "/lsx/capabilities_query",
      messageType: "nepi_ros_interfaces/LSXCapabilitiesQuery",
    })
    this.lsxUnits[lsxUnitNamespace] = capabilities
  }


  @action.bound
  async callRBXCapabilitiesQueryService(rbxRobotNamespace) {
    const capabilities = await this.callService({
      name: rbxRobotNamespace + "/rbx/capabilities_query",
      messageType: "nepi_ros_interfaces/RBXCapabilitiesQuery",  
    })
    this.rbxRobots[rbxRobotNamespace] = capabilities
  }

  @action.bound
  async callSystemSoftwareStatusQueryService() {
    this.systemSoftwareStatus = await this.callService({
      name: "sw_update_status_query",
      messageType: "nepi_ros_interfaces/SystemSoftwareStatusQuery"
    })
  }

  @action.bound
  async callAppStatusQueryService(appName) {
    const appStatus = await this.callService({
      name: 'apps_mgr/app_status_query',
      messageType: "nepi_ros_interfaces/AppStatusQuery",
      args: {app_name : appName},
    })
    const appNames = this.appNameList
    const appInd = appNames.indexOf(appName)
    if (appInd === -1){
      this.appStatusList.push(appStatus)
      this.appNameList.push(appName)

    }
    else {

      this.appNameList[appInd] = appName
      this.appStatusList[appInd] = appStatus
    }
  }

  @action.bound
  async onInstallFullSysImg(new_img_filename) {
    this.publishMessage({
      name: "install_new_image",
      messageType: "std_msgs/String",
      data: { data: new_img_filename }
    })    
  }

  @action.bound
  async onSwitchActiveInactiveRootfs() {
    this.publishMessage({
      name: "switch_active_inactive_rootfs",
      messageType: "std_msgs/Empty",
      data: {}
    })
  }

  @action.bound
  async onStartSysBackup() {
    this.publishMessage({
      name: "archive_inactive_rootfs",
      messageType: "std_msgs/Empty",
      data: {}
    })
  }  

  async startPollingIPAddrQueryService() {
    const _pollOnce = async () => {
      this.ip_query_response = await this.callService({
        name: "ip_addr_query",
        messageType: "nepi_ros_interfaces/IPAddrQuery"
      })

      if (this.connectedToROS) {
        setTimeout(_pollOnce, 3000)
      }
    }

    _pollOnce()
  }

  async startPollingBandwidthUsageService() {
    const _pollOnce = async () => {
      this.bandwidth_usage_query_response = await this.callService({
        name: "bandwidth_usage_query",
        messageType: "nepi_ros_interfaces/BandwidthUsageQuery",
      })

      if (this.connectedToROS) {
        setTimeout(_pollOnce, 3000)
      }
    }

    _pollOnce()
  }

  async startPollingWifiQueryService() {
    const _pollOnce = async () => {
      this.wifi_query_response = await this.callService({
        name: "wifi_query",
        messageType: "nepi_ros_interfaces/WifiQuery",
      })

      if (this.connectedToROS) {
        setTimeout(_pollOnce, 3000)
      }
    }

    _pollOnce()    
  }

  async startPollingOpEnvironmentQueryService() {
    const _pollOnce = async () => {
      this.opEnv = await this.callService({
        name: "op_environment_query",
        messageType: "nepi_ros_interfaces/OpEnvironmentQuery",
        msgKey: "op_env"
      })

      this.deviceInWater = (this.opEnv === "water")

      if (this.connectedToROS) {
        setTimeout(_pollOnce, 5000)
      }
    }

    _pollOnce()
  }


  startPollingNavPoseService() {
    var resp = null
    const _pollOnce = async () => {
      resp = await this.callService({
        name: "nav_pose_query",
        messageType: "nepi_ros_interfaces/NavPoseQuery",
        args: {query_time: 0.0, 
               transform: (this.transformNavPoseOrientation !== null)? this.transformNavPoseOrientation : true}
      })

      this.navPose = resp.nav_pose
      this.navPoseOrientationIsTransformed = resp.transformed

      this.navPoseLocationLat = this.navPose.fix.latitude
      this.navPoseLocationLng = this.navPose.fix.longitude
      this.navPoseLocationAlt = this.navPose.fix.altitude
      this.navPoseDirectionHeadingDeg = this.navPose.heading.heading

      this.navSatFixStatus = this.navPose.fix.status.status
      this.navSatFixService = this.navPose.fix.status.service

      this.navPosePositionEnuX = this.navPose.odom.pose.pose.position.x
      this.navPosePositionEnuY = this.navPose.odom.pose.pose.position.y
      this.navPosePositionEnuZ = this.navPose.odom.pose.pose.position.z

      // magnitude of linear_velocity?
      let { x, y, z } = this.navPose.odom.twist.twist.linear
      this.navPoseDirectionSpeedMpS = Math.sqrt(x * x + y * y + z * z)

      this.navPoseOrientationYawRate = this.navPose.odom.twist.twist.angular.z * (180/Math.PI)
      this.navPoseOrientationPitchRate = this.navPose.odom.twist.twist.angular.y * (180/Math.PI)
      this.navPoseOrientationRollRate = this.navPose.odom.twist.twist.angular.x * (180/Math.PI)

      const q = new cannon.Quaternion(
        this.navPose.odom.pose.pose.orientation.x,
        this.navPose.odom.pose.pose.orientation.y,
        this.navPose.odom.pose.pose.orientation.z,
        this.navPose.odom.pose.pose.orientation.w
      )
      const vec = new cannon.Vec3()
      // cannon exception says "Euler order XYZ not supported yet,:" so we must switch X and Z manually here
      //q.toEuler(vec, "XYZ")
      q.toEuler(vec, EULER_ORDER_FOR_CANNON)
      //this.navPoseOrientationYawAngle = vec.z * (180/Math.PI)
      this.navPoseOrientationYawAngle = vec.x * (180/Math.PI)
      this.navPoseOrientationPitchAngle = vec.y * (180/Math.PI)
      //this.navPoseOrientationRollAngle = vec.x * (180/Math.PI)
      this.navPoseOrientationRollAngle = vec.z * (180/Math.PI)

      this.navPoseOrientationFrame = this.navPose.odom.child_frame_id
      
      if (this.transformNavPoseOrientation === null){
        this.transformNavPoseOrientation = this.navPoseOrientationIsTransformed
      }


      if (this.connectedToROS) {
        setTimeout(_pollOnce, 500)
      }
    }

    _pollOnce()
  }

  startPollingNavPoseStatusService() {
    const _pollOnce = async () => {
      this.navPoseStatus = await this.callService({
        name: "nav_pose_status_query",
        messageType: "nepi_ros_interfaces/NavPoseStatusQuery",
        msgKey: "status"
      })

      this.selectedNavSatFixTopic = this.navPoseStatus.nav_sat_fix_topic
      this.lastNavSatFix = this.navPoseStatus.last_nav_sat_fix
      this.navSatFixRate = this.navPoseStatus.nav_sat_fix_rate
      
      this.selectedOrientationTopic = this.navPoseStatus.orientation_topic
      this.lastOrientation = this.navPoseStatus.last_orientation
      this.orientationRate = this.navPoseStatus.orientation_rate

      this.selectedHeadingTopic = this.navPoseStatus.heading_topic
      this.lastHeading = this.navPoseStatus.last_heading
      this.headingRate = this.navPoseStatus.heading_rate
      
      this.navSrcFrame = this.navPoseStatus.transform.header.frame_id
      this.navTargetFrame = this.navPoseStatus.transform.child_frame_id
      this.navTransformXTrans = this.navPoseStatus.transform.transform.translation.x
      this.navTransformYTrans = this.navPoseStatus.transform.transform.translation.y
      this.navTransformZTrans = this.navPoseStatus.transform.transform.translation.z

      const q = new cannon.Quaternion(
        this.navPoseStatus.transform.transform.rotation.x,
        this.navPoseStatus.transform.transform.rotation.y,
        this.navPoseStatus.transform.transform.rotation.z,
        this.navPoseStatus.transform.transform.rotation.w
      )
      const vec = new cannon.Vec3()
      q.toEuler(vec, EULER_ORDER_FOR_CANNON)
      this.navTransformXRot = vec.x * (180/Math.PI)
      this.navTransformYRot = vec.y * (180/Math.PI)
      this.navTransformZRot = vec.z * (180/Math.PI)

      this.navHeadingOffset = this.navPoseStatus.heading_offset

      this.gpsClockSyncEnabled = this.navPoseStatus.gps_clock_sync_enabled

      if (this.connectedToROS) {
        setTimeout(_pollOnce, 2000)
      }
    }

    _pollOnce()
  }

  startPollingTimeStatusService() {
    const _pollOnce = async () => {
      this.timeStatus = await this.callService({
        name: "time_status_query",
        messageType: "nepi_ros_interfaces/TimeStatus",
        msgKey: "time_status"
      })

      // if last_ntp_sync is 10y, no sync has happened
      this.clockNTP = false
      const currentlySyncd = this.timeStatus.currently_syncd
      currentlySyncd &&
      currentlySyncd.length &&
      currentlySyncd.forEach(syncd => {
          if (syncd !== false) {
            this.clockNTP = true
          }
        })

      // if last_pps – current_time < 1 second no sync has happened
      this.clockPPS = true
      const lastPPSTS = moment.unix(this.timeStatus.last_pps.secs).unix()
      const currTS = this.systemStatusTime && this.systemStatusTime.unix()
      if (currTS && lastPPSTS - currTS < 1) {
        this.clockPPS = false
      }

      if (this.connectedToROS) {
        setTimeout(_pollOnce, 1000)
      }
    }

    _pollOnce()
  }


  async startPollingGetScriptsService() {
    const _pollOnce = async () => {
      this.scripts = await this.callService({
        name: "get_scripts",
        messageType: "nepi_ros_interfaces/GetScriptsQuery"
      })

      if (this.connectedToROS) {
        setTimeout(_pollOnce, 1000)
      }
    }

    _pollOnce()
  }

  async startPollingGetRunningScriptsService() {
    const _pollOnce = async () => {
      this.running_scripts = await this.callService({
        name: "get_running_scripts",
        messageType: "nepi_ros_interfaces/GetRunningScriptsQuery"
      })

      if (this.connectedToROS) {
        setTimeout(_pollOnce, 1000)
      }
    }

    _pollOnce()
  }

  async startLaunchScriptService(item) {
    const _pollOnce = async () => {
      this.launchScript = await this.callService({
        name: "launch_script",
        messageType: "nepi_ros_interfaces/LaunchScript",
        args: {script : item}
      })
    }
    _pollOnce()
  }


  async stopLaunchScriptService(item) {
    const _pollOnce = async () => {
      this.stopScript = await this.callService({
        name: "stop_script",
        messageType: "nepi_ros_interfaces/StopScript",
        args: {script : item}
      })
    }
    _pollOnce()
  }

  async callGetSystemStatsQueryService(item, poll = true) {
    const _pollOnce = async () => {
      this.systemStats = await this.callService({
        name: "get_system_stats",
        messageType: "nepi_ros_interfaces/GetSystemStatsQuery",
        args: {script : this.scriptForPolledStats}
      })

      if (this.connectedToROS && poll) {
        setTimeout(_pollOnce, 1000)
      }
    }

    const firstCall = (this.scriptForPolledStats === null)
    this.scriptForPolledStats = item
    
    // Only launch this once, then just change state to start polling a different script
    if (firstCall === true || !poll) {
      _pollOnce()
    }
    
  }

  @action.bound
  setDeviceID({newDeviceID}) {
    this.publishMessage({
      name: "set_device_id",
      messageType: "std_msgs/String",
      data: { data: newDeviceID }
    })
  }

  @action.bound
  onToggleDeviceInWater() {
    this.deviceInWater = !this.deviceInWater

    let newOpEnv = (this.deviceInWater === true)? "water" : "air"
    this.publishMessage({
      name: "set_op_environment",
      messageType: "std_msgs/String",
      data: { data: newOpEnv }
    })
  }

  @action.bound
  onToggleClockUTCMode() {
    this.clockUTCMode = !this.clockUTCMode
    this.clockTZ = this.clockUTCMode ? "UTC" : getLocalTZ()
  }

  @action.bound
  onSyncUTCToDevice() {
    const utcTS = moment.utc()
      .unix()
    this.publishMessage({
      name: "set_time",
      messageType: "std_msgs/Time",
      data: {
        data: {
          secs: utcTS,
          nsecs: 0
        }
      }
    })
  }

  @action.bound
  onChangeTriggerRate(rate) {
    let freq = parseFloat(rate)

    if (isNaN(freq)) {
      freq = 0
    }

    this.publishMessage({
      name: "set_periodic_sw_trig",
      messageType: "nepi_ros_interfaces/PeriodicSwTrig",
      data: {
        enabled: freq > 0,
        sw_trig_mask: this.triggerMask,
        rate_hz: freq
      }
    })

    // Update the state variable -- if rejected, will get set back on next periodic update
    // This allows the text entry box to immediately switch to using the state variable
    this.triggerAutoRateHz = freq
  }

  @action.bound
  onChangeTXRateLimit(limit) {
    let lim = parseInt(limit, 10)

    if (isNaN(lim)) {
      lim = -1
    }

    this.publishMessage({
      name: "set_tx_bw_limit_mbps",
      messageType: "std_msgs/Int32",
      data: { data: lim }
    })

    // Update locally for display purposes... will be corrected on next periodic update if value is rejected
    this.bandwidth_usage_query_response.tx_limit_mbps = lim
  }

  @action.bound
  onToggleHWTriggerOutputEnabled(e) {
    const checked = e.target.checked

    // If HW Trig Output Enable selected, trig mask is 0xFFFFFFFF, otherwise trig mask is 0x7FFFFFFF
    this.triggerMask = checked
      ? TRIGGER_MASKS.OUTPUT_ENABLED
      : TRIGGER_MASKS.DEFAULT

    // republish rate change with new mask
    this.onChangeTriggerRate({
      target: {
        value: this.triggerAutoRateHz
      }
    })
  }

  @action.bound
  onToggleHWTriggerInputEnabled(e) {
    const checked = e.target.checked

    this.publishMessage({
      name: "hw_trigger_in_enab",
      messageType: "std_msgs/UInt32",
      data: { data: checked ? this.triggerMask : 0 }
    })
  }

  @action.bound
  onToggleDHCPEnabled(e) {
    const checked = e.target.checked

    this.publishMessage({
      name: "enable_dhcp",
      messageType: "std_msgs/Bool",
      data: { data: checked }
    })

    // Set local immediately, will correct on next update if values is rejected
    this.ip_query_response.dhcp_enabled = checked
  }

  @action.bound
  onToggleWifiAPEnabled(e) {
    const checked = e.target.checked

    this.publishMessage({
      name: "enable_wifi_access_point",
      messageType: "std_msgs/Bool",
      data: { data: checked }
    })
  }

  @action.bound
  onToggleWifiClientEnabled(e) {
    const checked = e.target.checked

    this.publishMessage({
      name: "enable_wifi_client",
      messageType: "std_msgs/Bool",
      data: { data: checked }
    })
  }

  @action.bound
  onUpdateWifiClientCredentials(new_ssid, new_passphrase) {

    this.publishMessage({
      name: "set_wifi_client_credentials",
      messageType: "nepi_ros_interfaces/WifiCredentials",
      data: { ssid: new_ssid, passphrase: new_passphrase }
    })
  }

  @action.bound
  onUpdateWifiAPCredentials(new_ssid, new_passphrase) {

    this.publishMessage({
      name: "set_wifi_access_point_credentials",
      messageType: "nepi_ros_interfaces/WifiCredentials",
      data: { ssid: new_ssid, passphrase: new_passphrase }
    })
  }  

  @action.bound
  onRefreshWifiNetworks() {
    this.publishMessage({
      name: "refresh_available_wifi_networks",
      messageType: "std_msgs/Empty",
      data: {}
    })    
  }

  @action.bound
  onGenerateLicenseRequest() {
    if (this.license_server && (this.license_server.readyState === 1)) { // Connected
      this.license_server.send("license_request")
      this.license_request_mode = true
    }
    else {
      this.license_request_mode = false
    }
  }

  @action.bound
  onPressManualTrigger() {
    // Pressing Manual Trigger publishes mask on the sw_trigger topic.
    this.publishMessage({
      name: "sw_trigger",
      messageType: "std_msgs/UInt32",
      data: { data: this.triggerMask }
    })
  }

  @action.bound
  onToggleSaveDataAll(checked) {
      this.publishMessage({
      name: "save_data",
      messageType: "nepi_ros_interfaces/SaveData",
      data: {
        save_continuous: checked,
        save_raw: false
      }
    })
  }

  @action.bound
  onChangeSaveFreqAll(rate) {
    let freq = parseFloat(rate)

    if (isNaN(freq)) {
      freq = 0
    }

    this.publishMessage({
      name: "save_data_rate",
      messageType: "nepi_ros_interfaces/SaveDataRate",
      data: {
        data_product: "all",
        save_rate_hz: freq,
      }
    })

    this.saveFreqHz = freq
  }




  @action.bound
  addIPAddr(addr) {
    this.publishMessage({
      name: "add_ip_addr",
      messageType: "std_msgs/String",
      data: { data: addr }
    })
  }

  @action.bound
  removeIPAddr(addr) {
    this.publishMessage({
      name: "remove_ip_addr",
      messageType: "std_msgs/String",
      data: { data: addr }
    })
  }

  @action.bound
  saveCfg({baseTopic}) {
    this.publishMessage({
      name: baseTopic + "/save_config",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    })
  }

  @action.bound
  resetCfg({baseTopic, resetVal}) {
    this.publishMessage({
      name: baseTopic + "/reset",
      messageType: "nepi_ros_interfaces/Reset",
      data: { reset_type: resetVal },
      noPrefix: true
    })
  }

  @action.bound
  onUserCfgRestore() {
    this.publishMessage({
      name: "full_user_restore",
      messageType: "std_msgs/Empty",
      data: {}
    })
  }

  @action.bound
  saveSettingsFilePrefix({newFilePrefix}) {
    this.publishMessage({
      name: "save_data_prefix",
      messageType: "std_msgs/String",
      data: { data: newFilePrefix }
    })
  }

  @action.bound
  deleteAllData() {
    this.publishMessage({
      name: "clear_data_folder",
      messageType: "std_msgs/Empty",
      data: {}
    })
  }

  @action.bound
  onToggleAutoStartEnabled(autoStartScriptName, isEnabled) {
    this.publishMessage({
      name: "enable_script_autostart",
      messageType: "nepi_ros_interfaces/AutoStartEnabled",
      data: { 
        script: autoStartScriptName,
        enabled: isEnabled
       }
    })
  }

  @action.bound
  onReinitNavPoseSolution() {
    this.publishMessage({
      name: "nav_pose_mgr/reinit_solution",
      messageType: "std_msgs/Empty",
    })
  }

  // Control methods //////////////////////////////////////////////
  @action.bound
  isThrottled() {
    var now = new Date()
    if (now - this.lastUpdate < UPDATE_PERIOD) {
      return true
    }
    this.lastUpdate = now
    return false
  }
  
  @action.bound
  publishValue(
    topic,
    msgType,
    value,
    throttle = true,
    noPrefix = false,
  ) {
    if (throttle && this.isThrottled()) {
      return
    }

    this.publishMessage({
      name: topic,
      messageType: msgType,
      data: {data: Number(value)},
      noPrefix: noPrefix,
    })
  }


  @action.bound
  publishRangeWindow(topic, min, max, throttle = true) {
    if (throttle){
      if (throttle && this.isThrottled()) {
        return
      }
    }
    if (topic) {
      this.publishMessage({
        name: topic,
        messageType: "nepi_ros_interfaces/RangeWindow",
        noPrefix: true,
        data: {
          start_range: min,
          stop_range: max
        }
      })
    } else {
      console.warn("publishRangeWindow: topic not set")
    }
  }



  /////////////////////////////////////////////////////////////////////////

  // Nav/Pose Control methods /////////////////////////////////////////////
  @action.bound
  onSetInitOrientation(roll_deg, pitch_deg, yaw_deg) {
    var q = new cannon.Quaternion()
    q.setFromEuler(roll_deg * (Math.PI/180), pitch_deg * (Math.PI/180), yaw_deg * (Math.PI/180), EULER_ORDER_FOR_CANNON) // Use YZX order because that's the only one available for the reverse operation
    this.publishMessage({
      name: "nav_pose_mgr/set_init_orientation",
      messageType: "geometry_msgs/QuaternionStamped",
      data: {
        header: {
          seq: 0,
          stamp: {
            sec: moment().unix(),
            nsec: 0
          },
          frame_id: "na"
        },
        quaternion: {
          x: q.x,
          y: q.y,
          z: q.z,
          w: q.w
        }
      }
    })
  }

  @action.bound
  onSetInitGPS(latitude_deg, longitude_deg, altitude_m) {
    this.publishMessage({
      name: "nav_pose_mgr/set_init_gps_fix",
      messageType: "sensor_msgs/NavSatFix",
      data: {
        header: {
          seq: 0,
          stamp: {
            sec: 0,
            nsec: 0
          },
          frame_id: "na"
        },
        status: {
          status: 0, // Valid FIX
          service: 1 // GPS
        },
        latitude: parseFloat(latitude_deg),
        longitude: parseFloat(longitude_deg),
        altitude: parseFloat(altitude_m),
        position_covariance_type: 0 // Unknown
      }
    })
  }

  @action.bound
  onSetInitHeading(heading_mag_deg) {
    this.publishMessage({
      name: "nav_pose_mgr/set_init_heading",
      messageType: "std_msgs/Float64",
      data: {
        data: parseFloat(heading_mag_deg),
      }
    })
  }

  @action.bound
  onSetAHRSOffsets(x_trans, y_trans, z_trans, x_rot, y_rot, z_rot, heading) {
    this.publishMessage({
      name: "nav_pose_mgr/set_ahrs_offset",
      messageType: "nepi_ros_interfaces/Offset",
      data: {
        translation: {
          x: parseFloat(x_trans),
          y: parseFloat(y_trans),
          z: parseFloat(z_trans),
        },
        rotation: {
          x: parseFloat(x_rot),
          y: parseFloat(y_rot),
          z: parseFloat(z_rot)
        },
        heading: parseFloat(heading)
      }
    })
  }

  @action.bound
  onSetAHRSOutFrame(ahrs_out_frame) {
    this.publishMessage({
      name: "nav_pose_mgr/set_ahrs_out_frame",
      messageType: "std_msgs/String",
      data: {
        data: ahrs_out_frame
      }
    })
  }

  @action.bound
  onToggletransformNavPoseOrientation() {
    this.transformNavPoseOrientation = !(this.transformNavPoseOrientation)
  }

  @action.bound
  onSetGPSFixTopic(topic) {
    this.publishMessage({
      name: "nav_pose_mgr/set_gps_fix_topic",
      messageType: "std_msgs/String",
      data: {
        data: topic
      }
    })
  }

  @action.bound
  onToggleGPSClockSync() {
    this.gpsClockSyncEnabled = !(this.gpsClockSyncEnabled)
    this.publishMessage({
      name: "nav_pose_mgr/enable_gps_clock_sync",
      messageType: "std_msgs/Bool",
      data: {
        data: this.gpsClockSyncEnabled
      }
    })
  }

  @action.bound
  onSetOrientationTopic(topic) {
    this.publishMessage({
      name: "nav_pose_mgr/set_orientation_topic",
      messageType: "std_msgs/String",
      data: {
        data: topic
      }
    })
  }

  @action.bound
  onSetHeadingTopic(topic) {
    this.publishMessage({
      name: "nav_pose_mgr/set_heading_topic",
      messageType: "std_msgs/String",
      data: {
        data: topic
      }
    })
  }

  @action.bound
  onChangeStreamingImageQuality(quality) {
    this.streamingImageQuality = quality
    this.publishMessage({
      name: "rui_config_mgr/set_streaming_image_quality",
      messageType: "std_msgs/UInt8",
      data: { data: quality }
    })
  }

  /////////////////////////////////////////////////////////////////////////

  // Mux Sequencer
  @action.bound
  onConfigureMuxSequence(updated_sequence_dict) {
    this.publishMessage({
      name: "app_image_sequencer/configure_mux_sequence",
      messageType: "nepi_app_image_sequencer/ImageMuxSequence",
      data: updated_sequence_dict
    })
  }

  @action.bound
  onDeleteMuxSequence(sequence_id) {
    this.publishMessage({
      name: "app_image_sequencer/delete_mux_sequence",
      messageType: "std_msgs/String",
      data: { data: sequence_id }
    })
  }

  // PTX
  @action.bound
  onPTXGoHome(ptxNamespace) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/go_home",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    })    
  }

  @action.bound
  onPTXSetHomeHere(ptxNamespace) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/set_home_position_here",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    })    
  }  

  @action.bound
  onPTXGotoWaypoint(ptxNamespace, waypoint_index) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/goto_waypoint",
      messageType: "std_msgs/UInt8",
      data: { data: waypoint_index },
      noPrefix: true
    })    
  }

  @action.bound
  onPTXSetWaypointHere(ptxNamespace, waypoint_index) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/set_waypoint_here",
      messageType: "std_msgs/UInt8",
      data: { data: waypoint_index },
      noPrefix: true
    })    
  }

  @action.bound
  onPTXStop(ptxNamespace) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/stop_moving",
      messageType: "std_msgs/Empty",
      data: {},
      noPrefix: true
    }) 
  }

  @action.bound
  onSetPTXHomePos(ptxNamespace, yawHomePos, pitchHomePos) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/set_home_position",
      messageType: "nepi_ros_interfaces/PanTiltPosition",
      data: {"yaw_deg": yawHomePos, "pitch_deg": pitchHomePos},
      noPrefix: true
    })
  }

  @action.bound
  onSetPTXGotoPos(ptxNamespace, yawHomePos, pitchHomePos) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/jog_to_position",
      messageType: "nepi_ros_interfaces/PanTiltPosition",
      data: {"yaw_deg": yawHomePos, "pitch_deg": pitchHomePos},
      noPrefix: true
    })
  }

  @action.bound
  onSetPTXSoftStopPos(ptxNamespace, yawMin, yawMax, pitchMin, pitchMax) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/set_soft_limits",
      messageType: "nepi_ros_interfaces/PanTiltLimits",
      data: {"min_yaw_deg": yawMin,
             "max_yaw_deg": yawMax,
             "min_pitch_deg": pitchMin,
             "max_pitch_deg": pitchMax},
      noPrefix: true
    })
  }

  @action.bound
  onSetPTXHardStopPos(ptxNamespace, yawMin, yawMax, pitchMin, pitchMax) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/set_hard_limits",
      messageType: "nepi_ros_interfaces/PanTiltLimits",
      data: {"min_yaw_deg": yawMin,
             "max_yaw_deg": yawMax,
             "min_pitch_deg": pitchMin,
             "max_pitch_deg": pitchMax},
      noPrefix: true
    })
  }

  @action.bound
  onPTXJogYaw(ptxNamespace, direction) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/jog_timed_yaw",
      messageType: "nepi_ros_interfaces/SingleAxisTimedMove",
      data: {"direction": direction,
             "duration_s": -1},
      noPrefix: true
    })
  }

  @action.bound
  onPTXJogPitch(ptxNamespace, direction) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/jog_timed_pitch",
      messageType: "nepi_ros_interfaces/SingleAxisTimedMove",
      data: {"direction": direction,
             "duration_s": -1},
      noPrefix: true
    })
  }

  @action.bound
  onSetReverseYawControl(ptxNamespace, reverse) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/reverse_yaw_control",
      messageType: "std_msgs/Bool",
      data: {"data" : reverse},
      noPrefix: true
    })
  }

  @action.bound
  onSetReversePitchControl(ptxNamespace, reverse) {
    this.publishMessage({
      name: ptxNamespace + "/ptx/reverse_pitch_control",
      messageType: "std_msgs/Bool",
      data: {"data" : reverse},
      noPrefix: true
    })
  }
  
  @action.bound
  onLSXSetStandby(lsxNamespace, enable) {
    this.publishMessage({
      name: lsxNamespace + "/lsx/set_standby",
      messageType: "std_msgs/Bool",
      data: {"data" : enable},
      noPrefix: true
    })
  }

  @action.bound
  onLSXSetStrobeEnable(lsxNamespace, enable) {
    this.publishMessage({
      name: lsxNamespace + "/lsx/set_strobe_enable",
      messageType: "std_msgs/Bool",
      data: {"data" : enable},
      noPrefix: true
    })
  }

}
const stores = {
  ros: new ROSConnectionStore(),
}

export default stores
