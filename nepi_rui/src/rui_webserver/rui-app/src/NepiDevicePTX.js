/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"
import Toggle from "react-toggle"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import { SliderAdjustment } from "./AdjustmentWidgets"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import {createShortUniqueValues, setElementStyleModified, clearElementStyleModified, onChangeSwitchSendBoolValue, onUpdateSetStateValue} from "./Utilities"
import {createShortValuesFromNamespaces} from "./Utilities"

import NepiDeviceInfo from "./Nepi_IF_DeviceInfo"
import ImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFSettings from "./Nepi_IF_Settings"
import NepiIFSaveData from "./Nepi_IF_SaveData"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}

@inject("ros")
@observer

// Component that contains the PTX controls
class NepiDevicePTX extends Component {
  constructor(props) {
    super(props)

    this.state = {
      imageTopic: null,
      imageText: null,

      ptSerialNum: null,
      ptHwVersion: null,
      ptSwVersion: null,
      
      yawPositionDeg: null,
      pitchPositionDeg: null,

      yawGotoDeg: null,
      pitchGotoDeg: null,

      yawHomePosEdited: null,
      yawHomePosDeg: null,
      pitchHomePosEdited: null,
      pitchHomePosDeg: null,



      showLimits: false,

      yawMaxHardstopDeg: null,
      yawMaxHardstopEdited: null,
      pitchMaxHardstopDeg: null,
      pitchMaxHardstopEdited: null,

      yawMinHardstopDeg: null,
      yawMinHardstopEdited: null,
      pitchMinHardstopDeg: null,
      pitchMinHardstopEdited: null,
      
      yawMaxSoftstopDeg: null,
      yawMaxSoftstopEdited: null,
      pitchMaxSoftstopDeg: null,
      pitchMaxSoftstopEdited: null,

      yawMinSoftstopDeg: null,
      yawMinSoftstopEdited: null,
      pitchMinSoftstopDeg: null,
      pitchMinSoftstopEdited: null,




      yawNowRatio: null,
      pitchNowRatio: null,
      yawGoalRatio: null,
      pitchGoalRatio: null,
      yawGoalRatioLast: null,
      pitchGoalRatioLast: null,
      yawRatio: null,
      pitchRatio: null,
      speedRatio: null,

      reverseYawControl: false,
      reversePitchControl: false,

      auto_pan: false,
      auto_pan_min: -1,
      auto_pan_max: 1,
      auto_tilt: false,
      auto_tilt_min: -1,
      auto_tilt_max: 1,

      selectedWaypoint: 0,

      listener: null,
      disabled: true
    }

    this.onUpdateText = this.onUpdateText.bind(this)
    this.onKeyText = this.onKeyText.bind(this)
    this.createImageTopicsOptions = this.createImageTopicsOptions.bind(this)
    this.onImageTopicSelected = this.onImageTopicSelected.bind(this)
    this.onptxDeviceselected = this.onptxDeviceselected.bind(this)
    this.ptxStatusListener = this.ptxStatusListener.bind(this)
    this.renderControlPanel = this.renderControlPanel.bind(this)
    this.createPTXOptions = this.createPTXOptions.bind(this)
    this.createWaypointOptions = this.createWaypointOptions.bind(this)
    this.onWaypointSelected = this.onWaypointSelected.bind(this)
    this.onClickToggleShowLimits = this.onClickToggleShowLimits.bind(this)

    this.onEnterSendInputBoxRangeWindowValue = this.onEnterSendInputBoxRangeWindowValue.bind(this)
  }

  onUpdateText(e) {
    var yawElement = null
    var pitchElement = null
    var yawMinElement = null
    var yawMaxElement = null
    var pitchMinElement = null
    var pitchMaxElement = null
    if ((e.target.id === "PTXYawHomePos") || (e.target.id === "PTXPitchHomePos"))
    {
      yawElement = document.getElementById("PTXYawHomePos")
      setElementStyleModified(yawElement)
      
      pitchElement = document.getElementById("PTXPitchHomePos")
      setElementStyleModified(pitchElement)
      
      this.setState({yawHomePosEdited: yawElement.value,
                     pitchHomePosEdited: pitchElement.value})
    }
    else if ((e.target.id === "PTXYawSoftStopMin") || (e.target.id === "PTXYawSoftStopMax") ||
             (e.target.id === "PTXPitchSoftStopMin") || (e.target.id === "PTXPitchSoftStopMax"))
    {
      yawMinElement = document.getElementById("PTXYawSoftStopMin")
      setElementStyleModified(yawMinElement)

      yawMaxElement = document.getElementById("PTXYawSoftStopMax")
      setElementStyleModified(yawMaxElement)

      pitchMinElement = document.getElementById("PTXPitchSoftStopMin")
      setElementStyleModified(pitchMinElement)

      pitchMaxElement = document.getElementById("PTXPitchSoftStopMax")
      setElementStyleModified(pitchMaxElement)

      this.setState({yawMinSoftstopEdited: yawMinElement.value, yawMaxSoftstopEdited: yawMaxElement.value, 
                     pitchMinSoftstopEdited: pitchMinElement.value, pitchMaxSoftstopEdited: pitchMaxElement.value})
    }

  }

  onKeyText(e) {
    const {onSetPTXGotoPos, onSetPTXGotoPanPos, onSetPTXGotoTiltPos, onSetPTXHomePos, onSetPTXSoftStopPos, onSetPTXHardStopPos} = this.props.ros
    var yawElement = null
    var pitchElement = null
    var yawMinElement = null
    var yawMaxElement = null
    var pitchMinElement = null
    var pitchMaxElement = null
    const namespace = this.state.ptxNamespace
    if(e.key === 'Enter'){
      if ((e.target.id === "PTXYawHomePos") || (e.target.id === "PTXPitchHomePos"))
      {
        yawElement = document.getElementById("PTXYawHomePos")
        clearElementStyleModified(yawElement)
        
        pitchElement = document.getElementById("PTXPitchHomePos")
        clearElementStyleModified(pitchElement)
                
        onSetPTXHomePos(namespace, Number(yawElement.value), Number(pitchElement.value))
        this.setState({yawHomePosEdited:null, pitchHomePosEdited:null})
      }
      else if ((e.target.id === "PTXYawSoftStopMin") || (e.target.id === "PTXYawSoftStopMax") ||
               (e.target.id === "PTXPitchSoftStopMin") || (e.target.id === "PTXPitchSoftStopMax"))
      {
        yawMinElement = document.getElementById("PTXYawSoftStopMin")
        clearElementStyleModified(yawMinElement)

        yawMaxElement = document.getElementById("PTXYawSoftStopMax")
        clearElementStyleModified(yawMaxElement)

        pitchMinElement = document.getElementById("PTXPitchSoftStopMin")
        clearElementStyleModified(pitchMinElement)

        pitchMaxElement = document.getElementById("PTXPitchSoftStopMax")
        clearElementStyleModified(pitchMaxElement)

        onSetPTXSoftStopPos(namespace, Number(yawMinElement.value), Number(yawMaxElement.value), 
                            Number(pitchMinElement.value), Number(pitchMaxElement.value))
        this.setState({yawMaxSoftstopEdited: null, yawMinSoftstopEdited: null, pitchMaxSoftstopEdited: null, pitchMinSoftstopEdited: null})
      }
      else if (e.target.id === "PTXYawGoto") 
        {
          yawElement = document.getElementById("PTXYawGoto")
          clearElementStyleModified(yawElement)
                            
          onSetPTXGotoPanPos(namespace, Number(yawElement.value))
        }
        else if  (e.target.id === "PTXPitchGoto")
          {
            
            pitchElement = document.getElementById("PTXPitchGoto")
            clearElementStyleModified(pitchElement)
                    
            onSetPTXGotoTiltPos(namespace, Number(pitchElement.value))
          }

    }
  }


  // Function for creating image topic options.
  createImageTopicsOptions() {
    var items = []
    items.push(<Option>{"None"}</Option>) 
    const { imageTopics } = this.props.ros
    var imageTopicShortnames = createShortValuesFromNamespaces(imageTopics)
    for (var i = 0; i < imageTopics.length; i++) {
      items.push(<Option value={imageTopics[i]}>{imageTopicShortnames[i]}</Option>)
    }
    return items
  }

  // Handler for Image topic selection
  onImageTopicSelected(event) {
    var ind = event.nativeEvent.target.selectedIndex
    var text = event.nativeEvent.target[ind].text
    var value = event.target.value

    this.setState({
      imageTopic: value,
      imageText: text === "None" ? null : text,
    })
  }

  
  // Callback for handling ROS Status3DX messages
  ptxStatusListener(message) {
    this.setState({
      ptSerialNum: message.serial_num,
      ptHwVersion: message.hw_version,
      ptSwVersion: message.sw_version,
      yawNowRatio: message.yaw_now_ratio,
      pitchNowRatio: message.pitch_now_ratio,
      yawGoalRatio: message.yaw_goal_ratio,
      pitchGoalRatio: message.pitch_goal_ratio,
      speedRatio: message.speed_ratio,
      yawPositionDeg: message.yaw_now_deg,
      pitchPositionDeg: message.pitch_now_deg,
      yawHomePosDeg: message.yaw_home_pos_deg,
      pitchHomePosDeg: message.pitch_home_pos_deg,
      yawMaxHardstopDeg: message.yaw_max_hardstop_deg,
      pitchMaxHardstopDeg: message.pitch_max_hardstop_deg,
      yawMinHardstopDeg: message.yaw_min_hardstop_deg,
      pitchMinHardstopDeg: message.pitch_min_hardstop_deg,
      yawMaxSoftstopDeg: message.yaw_max_softstop_deg,
      pitchMaxSoftstopDeg: message.pitch_max_softstop_deg,
      yawMinSoftstopDeg: message.yaw_min_softstop_deg,
      pitchMinSoftstopDeg: message.pitch_min_softstop_deg,
      reverseYawControl: message.reverse_yaw_control,
      reversePitchControl: message.reverse_pitch_control,
      auto_pan: message.auto_pan,
      auto_pan_min: message.auto_pan_range_window.start,
      auto_pan_max: message.auto_pan_range_window.stop,
      auto_tilt: message.auto_tilt,
      auto_tilt_min: message.auto_tilt_range_window.start,
      auto_tilt_max: message.auto_tilt_range_window.stop
    })
  

    
  }

  // Function for configuring and subscribing to ptx/status
  onptxDeviceselected(event) {
    if (this.state.listener) {
      this.state.listener.unsubscribe()
    }

    var ind = event.nativeEvent.target.selectedIndex
    var value = event.target.value

    // Handle the "None" option -- always index 0
    if (ind === 0) {
      this.setState({ disabled: true })
      return
    }

    this.setState({ ptxNamespace: value })

    var listener = this.props.ros.setupPTXStatusListener(
        value,
        this.ptxStatusListener
      )
      
    this.setState({ ptxNamespace: value, listener: listener, disabled: false })
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status3DX message
  componentWillUnmount() {
    if (this.state.listener) {
      this.state.listener.unsubscribe()
      this.setState({listener : null})
    }
  }

  // Function for creating topic options for Select input
  createPTXOptions(caps_dictionaries, filter) {
    const topics = Object.keys(caps_dictionaries)
    var filteredTopics = topics
    var i
    if (filter) {
      filteredTopics = []
      for (i = 0; i < topics.length; i++) {
        // includes does a substring search
        if (topics[i].includes(filter)) {
          filteredTopics.push(topics[i])
        }
      }
    }

    var items = []
    items.push(<Option>{""}</Option>)
    //var unique_names = createShortUniqueValues(filteredTopics)
    var device_name = ""
    for (i = 0; i < filteredTopics.length; i++) {
      device_name = filteredTopics[i].split('/ptx')[0].split('/').pop()
      items.push(<Option value={filteredTopics[i]}>{device_name}</Option>)
    }

    return items
  }

  createWaypointOptions(waypoint_count) {
    var items = []
    for (var i = 0; i < waypoint_count; ++i) {
      items.push(<Option value={i}>{i.toString()}</Option>)
    }
    return items
  }

  onWaypointSelected(event)
  {
    const ind = event.nativeEvent.target.selectedIndex
    this.setState({selectedWaypoint: ind})
  }

  onClickToggleShowLimits(){
    const currentVal = this.state.showLimits 
    this.setState({showLimits: !currentVal})
    this.render()
  }




onEnterSendInputBoxRangeWindowValue(event, topicName, entryName, other_val) {
  const {publishRangeWindow} = this.props.ros
  const ptxNamespace = this.state.ptxNamespace
  const namespace = ptxNamespace + topicName
  var min = -60
  var max = 60
  if(event.key === 'Enter'){
    const value = parseFloat(event.target.value)
    if (!isNaN(value)){
      if (entryName === "min"){
        min = value
        max = other_val
      }
      else if (entryName === "max"){
        min = other_val
        max = value
      }
      publishRangeWindow(namespace,min,max,false)
    }
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }
}

  renderControlPanel() {
    const { ptxNamespace, ptSerialNum, ptHwVersion, ptSwVersion,
            yawPositionDeg, pitchPositionDeg, yawHomePosDeg, pitchHomePosDeg,
            yawMaxHardstopDeg, pitchMaxHardstopDeg, yawMinHardstopDeg, pitchMinHardstopDeg,
            yawMinHardstopEdited, pitchMinHardstopEdited, yawMaxHardstopEdited, pitchMaxHardstopEdited,
            yawMaxSoftstopDeg, pitchMaxSoftstopDeg, yawMinSoftstopDeg, pitchMinSoftstopDeg,
            yawMinSoftstopEdited, pitchMinSoftstopEdited, yawMaxSoftstopEdited, pitchMaxSoftstopEdited,
            speedRatio, yawHomePosEdited, pitchHomePosEdited,
            reverseYawControl, reversePitchControl, selectedWaypoint } = this.state
    const { ptxDevices, onPTXGoHome, onPTXSetHomeHere, onPTXGotoWaypoint, onPTXSetWaypointHere,
            onSetReverseYawControl, onSetReversePitchControl } = this.props.ros
    const ptx_id = ptxNamespace? ptxNamespace.split('/').slice(-1) : "No Pan/Tilt Selected"

    const yawPositionDegClean = yawPositionDeg + .001
    const pitchPositionDegClean = pitchPositionDeg + .001

    const ptx_caps = ptxDevices[ptxNamespace]
    const has_abs_pos = ptx_caps && (ptx_caps.has_absolute_positioning)
    const has_timed_pos = ptx_caps && (ptx_caps.has_timed_positioning)
    const has_sep_pan_tilt = ptx_caps && (ptx_caps.has_seperate_pan_tilt)
    const has_auto_pan = ptx_caps && (ptx_caps.has_auto_pan)
    const has_auto_tilt = ptx_caps && (ptx_caps.has_auto_tilt)
    const has_speed_control = ptx_caps && (ptx_caps.adjustable_speed)
    const has_homing = ptx_caps && (ptx_caps.has_homing)
    const has_waypoints = ptx_caps && (ptx_caps.has_waypoints)
    
    const yawHomePos = (yawHomePosEdited === null)? round(yawHomePosDeg, 1) : yawHomePosEdited
    const pitchHomePos = (pitchHomePosEdited === null)? round(pitchHomePosDeg, 1) : pitchHomePosEdited

    const yawHardStopMin = (yawMinHardstopEdited === null)? round(yawMinHardstopDeg, 1) : yawMinHardstopEdited
    const pitchHardStopMin = (pitchMinHardstopEdited === null)? round(pitchMinHardstopDeg, 1) : pitchMinHardstopEdited
    const yawHardStopMax = (yawMaxHardstopEdited === null)? round(yawMaxHardstopDeg, 1) : yawMaxHardstopEdited
    const pitchHardStopMax = (pitchMaxHardstopEdited === null)? round(pitchMaxHardstopDeg, 1) : pitchMaxHardstopEdited


    const yawSoftStopMin = (yawMinSoftstopEdited === null)? round(yawMinSoftstopDeg, 1) : yawMinSoftstopEdited
    const pitchSoftStopMin = (pitchMinSoftstopEdited === null)? round(pitchMinSoftstopDeg, 1) : pitchMinSoftstopEdited
    const yawSoftStopMax = (yawMaxSoftstopEdited === null)? round(yawMaxSoftstopDeg, 1) : yawMaxSoftstopEdited
    const pitchSoftStopMax = (pitchMaxSoftstopEdited === null)? round(pitchMaxSoftstopDeg, 1) : pitchMaxSoftstopEdited

    const namespace = this.state.ptxNamespace
    
    return (
      <Section title={ptx_id} >
        <Label title={"Serial Number"}>
          <Input disabled={true} value={ptSerialNum}/>
        </Label>
        <Label title={"H/W Rev."}>
          <Input disabled={true} value={ptHwVersion}/>
        </Label>
        <Label title={"S/W Rev."}>
          <Input disabled={true} value={ptSwVersion}/>
        </Label>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Angles in ENU frame (Tilt+:Down , Pan+:Left)"}
         </label>


         <Columns>
                  <Column>

                  <div hidden={has_auto_pan === false}>

                  <Label title="Enable Auto Pan">
                    <Toggle
                      checked={this.state.auto_pan===true}
                      onClick={() => this.onChangeSwitchSendBoolValue("/set_auto_pan_enable",!this.state_auto_pan)}>
                    </Toggle>
                  </Label>



                <Label title={"Set Pan Min"}>
                    <Input id="set_pan_min" 
                      value={this.state.set_pan_min} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"set_pan_min")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_auto_tilt_window","min",this.state.set_pan_max)} />
              </Label>
            

                  <Label title={"Set Pan Max"}>
                    <Input id="set_pan_max" 
                     value={this.state.set_pan_max} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"set_pan_max")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_auto_tilt_window","max",this.state.set_pan_min)} />                      
                  </Label>  

                  </div>

                </Column>
                <Column>

                <div hidden={has_auto_tilt === false}>

                <Label title="Enable Auto Tilt">
                    <Toggle
                      checked={this.state.auto_pan===true}
                      onClick={() => this.onChangeSwitchSendBoolValue("/set_auto_tilt_enable",!this.state_auto_pan)}>
                    </Toggle>
                  </Label>


                  <Label title={"Set Tilt Min"}>
                    <Input id="set_tilt_min" 
                      value={this.state.set_tilt_min} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"set_tilt_min")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_auto_tilt_window","min",this.state.set_tilt_max)} />
              </Label>
            

                  <Label title={"Set Tilt Max"}>
                    <Input id="set_tilt_max" 
                     value={this.state.set_tilt_max} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"set_tilt_max")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_auto_tilt_window","max",this.state.set_tilt_min)} />                      
                  </Label>  

                  </div>

                  </Column>
                </Columns>



        <Label title={""}>
        <div style={{ display: "inline-block", width: "45%", float: "left" }}>
          {"Pan"}
        </div>
        <div style={{ display: "inline-block", width: "45%" }}>{"Tilt"}</div>
        </Label>

        <Label title={"Reverse Control"}>
          <div style={{ display: "inline-block", width: "45%", float: "left" }}>
            <Toggle style={{justifyContent: "flex-left"}} checked={reverseYawControl} onClick={() => onSetReverseYawControl(ptxNamespace, reverseYawControl===false)} />
          </div>
          <div style={{ display: "inline-block", width: "45%", float: "right" }}>
            <Toggle style={{justifyContent: "flex-right"}} checked={reversePitchControl} onClick={() => onSetReversePitchControl(ptxNamespace, reversePitchControl===false)} />
          </div>
        </Label>



        <div hidden={(has_speed_control === false)}>

        <SliderAdjustment
          disabled={!has_speed_control}
          title={"Speed"}
          msgType={"std_msgs/Float32"}
          adjustment={speedRatio}
          topic={ptxNamespace + "/set_speed_ratio"}
          scaled={0.01}
          min={0}
          max={100}
          tooltip={"Speed as a percentage (0%=min, 100%=max)"}
          unit={"%"}
        />

        </div>

        <div hidden={(has_abs_pos === false)}>

        <Label title={"Present Position"}>
          <Input
            disabled
            style={{ width: "45%", float: "left" }}
            value={round(yawPositionDegClean, 2)}
          />
          <Input
            disabled
            style={{ width: "45%" }}
            value={round(pitchPositionDegClean, 2)}
          />
        </Label>


        <Label title={"GoTo Position"}>
          <Input
            disabled={!has_abs_pos}
            id={"PTXYawGoto"}
            style={{ width: "45%", float: "left" }}
            value={this.state.yawGotoDeg}
            onChange= {this.onUpdateText}
            onKeyDown= {this.onKeyText}
          />
          <Input
            disabled={!has_abs_pos}
            id={"PTXPitchGoto"}
            style={{ width: "45%" }}
            value={this.state.pitchGotoDeg}
            onChange= {this.onUpdateText}
            onKeyDown= {this.onKeyText}
          />
        </Label>




        <Columns>
          <Column>

            <Label title="Show Limits">
                    <Toggle
                      checked={this.state.showLimits===true}
                      onClick={this.onClickToggleShowLimits}>
                    </Toggle>
                  </Label>


             
          </Column>
          <Column>
 
          </Column>
          <Column>

          </Column>
        </Columns>


        <div hidden={(this.state.showLimits === false)}>

        <Label title={"Hard Limit Min"}>
          <Input
            disabled={!has_abs_pos}
            id={"PTXYawHardStopMin"}
            style={{ width: "45%", float: "left" }}
            value={yawHardStopMin}
            disabled={true}
          />
          <Input
            disabled={!has_abs_pos}
            id={"PTXPitchHardStopMin"}
            style={{ width: "45%" }}
            value={pitchHardStopMin}
            disabled={true}
          />
        </Label>
        <Label title={"Hard Limit Max"}>
          <Input
            disabled={!has_abs_pos}
            id={"PTXYawHardStopMax"}
            style={{ width: "45%", float: "left" }}
            value={yawHardStopMax}
            disabled={true}
          />
          <Input
            disabled={!has_abs_pos}
            id={"PTXPitchHardStopMax"}
            style={{ width: "45%" }}
            value={pitchHardStopMax}
            disabled={true}
          />
        </Label>


        <Label title={"Soft Limit Min"}>
          <Input
            disabled={!has_abs_pos}
            id={"PTXYawSoftStopMin"}
            style={{ width: "45%", float: "left" }}
            value={yawSoftStopMin}
            onChange= {this.onUpdateText}
            onKeyDown= {this.onKeyText}
          />
          <Input
            disabled={!has_abs_pos}
            id={"PTXPitchSoftStopMin"}
            style={{ width: "45%" }}
            value={pitchSoftStopMin}
            onChange= {this.onUpdateText}
            onKeyDown= {this.onKeyText}
          />
        </Label>
        <Label title={"Soft Limit Max"}>
          <Input
            disabled={!has_abs_pos}
            id={"PTXYawSoftStopMax"}
            style={{ width: "45%", float: "left" }}
            value={yawSoftStopMax}
            onChange= {this.onUpdateText}
            onKeyDown= {this.onKeyText}
          />
          <Input
            disabled={!has_abs_pos}
            id={"PTXPitchSoftStopMax"}
            style={{ width: "45%" }}
            value={pitchSoftStopMax}
            onChange= {this.onUpdateText}
            onKeyDown= {this.onKeyText}
          />
        </Label>

        </div>

        <Label title={"Home Position"}>
          <Input
            disabled={!has_homing}
            id={"PTXYawHomePos"}
            style={{ width: "45%", float: "left" }}
            value={yawHomePos}
            onChange= {this.onUpdateText}
            onKeyDown= {this.onKeyText}
          />
          <Input
            disabled={!has_homing}
            id={"PTXPitchHomePos"}
            style={{ width: "45%" }}
            value={pitchHomePos}
            onChange= {this.onUpdateText}
            onKeyDown= {this.onKeyText}
          />
        </Label>

        </div>

        <div hidden={(has_homing === false)}>

        <ButtonMenu>
          <Button disabled={!has_homing} onClick={() => onPTXGoHome(ptxNamespace)}>{"Go Home"}</Button>
          <Button disabled={!has_homing} onClick={() => onPTXSetHomeHere(ptxNamespace)}>{"Set Home Here"}</Button>
        </ButtonMenu>

        </div>

        <div hidden={(has_waypoints === false)}>

        <Label title={"Waypoint Selection"}>
          <Select
            disabled={!has_waypoints}
            onChange={this.onWaypointSelected}
            value={selectedWaypoint}>
            {this.createWaypointOptions(has_waypoints? 256 : 0)}
          </Select>
        </Label>
        <ButtonMenu>
          <Button disabled={!has_waypoints} onClick={() => onPTXGotoWaypoint(ptxNamespace, selectedWaypoint)}>{"Goto Waypoint"}</Button>
          <Button disabled={!has_waypoints} onClick={() => onPTXSetWaypointHere(ptxNamespace, selectedWaypoint)}>{"Set Waypoint"}</Button>
        </ButtonMenu>

        </div>


      </Section>
    )
  }

  render() {
    const { ptxDevices, onPTXJogYaw, onPTXJogPitch, onPTXStop, sendTriggerMsg } = this.props.ros
    const { ptxNamespace, yawNowRatio, pitchNowRatio} = this.state

    const ptxImageViewerElement = document.getElementById("ptxImageViewer")
    const pitchSliderHeight = (ptxImageViewerElement)? ptxImageViewerElement.offsetHeight : "100px"

    const ptx_caps = ptxDevices[ptxNamespace]
    const has_abs_pos = ptx_caps && (ptx_caps.has_absolute_positioning === true)
    const has_timed_pos = ptx_caps && (ptx_caps.has_timed_positioning === true)

    const namespace = this.state.ptxNamespace

    return (
      <React.Fragment>
        <Columns>
          <Column equalWidth = {false} >

                <div hidden={(namespace === null)}>
                      <NepiDeviceInfo
                            deviceNamespace={namespace}
                            status_topic={"/status"}
                            status_msg_type={"nepi_ros_interfaces/PTXStatus"}
                            name_update_topic={"/update_device_name"}
                            name_reset_topic={"/reset_device_name"}
                            title={"NepiSensorsImagingInfo"}
                        />

                </div>


                <div id="ptxImageViewer">
                  <ImageViewer
                    id="ptxImageViewer"
                    imageTopic={this.state.imageTopic}
                    title={this.state.imageText}
                    hideQualitySelector={false}
                  />
                </div>
                <SliderAdjustment
                  disabled={!has_abs_pos}
                  title={"Yaw"}
                  msgType={"std_msgs/Float32"}
                  adjustment={yawNowRatio}
                  topic={ptxNamespace + "/jog_to_yaw_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Yaw as a percentage (0%=min, 100%=max)"}
                  unit={"%"}
                  noTextBox={true}
                  noLabel={true}
                />

              <div hidden={(has_timed_pos === false)}>

              <ButtonMenu>

                  <Button 
                    buttonDownAction={() => onPTXJogYaw(ptxNamespace, - 1)}
                    buttonUpAction={() => onPTXStop(ptxNamespace)}>
                    {'\u25C0'}
                    </Button>
                  <Button 
                    buttonDownAction={() => onPTXJogYaw(ptxNamespace, 1)}
                    buttonUpAction={() => onPTXStop(ptxNamespace)}>
                    {'\u25B6'}
                  </Button>
                  <Button 
                    buttonDownAction={() => onPTXJogPitch(ptxNamespace, 1)}
                    buttonUpAction={() => onPTXStop(ptxNamespace)}>
                    {'\u25B2'}
                  </Button>
                  <Button 
                    buttonDownAction={() => onPTXJogPitch(ptxNamespace, -1)}
                    buttonUpAction={() => onPTXStop(ptxNamespace)}>
                    {'\u25BC'}
                  </Button>
                  
                </ButtonMenu>


                </div>



                <ButtonMenu>

                  <Button onClick={() => onPTXStop(ptxNamespace)}>{"STOP"}</Button>
                  
                  </ButtonMenu>


          </Column>
          <Column style={{flex: 0.05}}>
            <SliderAdjustment
              disabled={!has_abs_pos}
              title={"Pitch"}
              msgType={"std_msgs/Float32"}
              adjustment={pitchNowRatio}
              topic={ptxNamespace + "/jog_to_pitch_ratio"}
              scaled={0.01}
              min={0}
              max={100}
              tooltip={"Pitch as a percentage (0%=min, 100%=max)"}
              unit={"%"}
              vertical={true}
              verticalHeight={pitchSliderHeight}
              noTextBox={true}
              noLabel={true}
            />
          </Column>
          <Column>
            <Label title={"Device"}>
              <Select
                onChange={this.onptxDeviceselected}
                value={namespace}
              >
                {this.createPTXOptions(ptxDevices)}
              </Select>
            </Label>
            <Label title={"Select Image"}>
              <Select
                id="ptxImageTopicSelect"
                onChange={this.onImageTopicSelected}
                value={this.state.imageTopic}
              >
              {this.createImageTopicsOptions()}
              </Select>
            </Label>


            <div align={"left"} textAlign={"left"} hidden={namespace == null}>

              <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                <Columns>
                  <Column>


                    <ButtonMenu>
                        <Button onClick={() => sendTriggerMsg(namespace + "/save_config")}>{"Save"}</Button>
                  </ButtonMenu>


                    </Column>
                  <Column>


                  <ButtonMenu>
                      <Button onClick={() => sendTriggerMsg( namespace + "/reset_config")}>{"Reset"}</Button>
                    </ButtonMenu>

                  </Column>
                  <Column>

                  <ButtonMenu>
                        <Button onClick={() => sendTriggerMsg( namespace + "/factory_reset_config")}>{"Factory Reset"}</Button>
                  </ButtonMenu>


                  </Column>
                </Columns>
            </div>


            { ptxNamespace?
              this.renderControlPanel()
              : null
            }

            <div hidden={(namespace == null)}>
              <NepiIFSettings
                settingsNamespace={namespace}
                title={"Nepi_IF_Settings"}
              />
            </div>

          </Column>
        </Columns>
      </React.Fragment>
    )
  }
}

export default NepiDevicePTX
