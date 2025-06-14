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

import Section from "./Section"
import Button, { ButtonMenu } from "./Button"
import RangeAdjustment from "./RangeAdjustment"
import {RadioButtonAdjustment, SliderAdjustment} from "./AdjustmentWidgets"
import Toggle from "react-toggle"
import Label from "./Label"
import Input from "./Input"
import { Column, Columns } from "./Columns"
import { round, onUpdateSetStateValue, onEnterSetStateFloatValue,  } from "./Utilities"

@inject("ros")
@observer

// Component that contains the NPX Device controls
class NepiDeviceNPXControls extends Component {
  constructor(props) {
    super(props)

    // these states track the values through NPX Status messages
    this.state = {
      frame3D: null,
      showTransform: false,
      transform_topic_list: [],
      transforms_list: [],
      transform_msg: null,
      transformTX: 0,
      transformTY: 0,
      transformTZ: 0,
      transformRX: 0,
      transformRY: 0,
      transformRZ: 0,
      transformHO: 0,
      include_transform: false,
      age_filter_s: null
    }

    this.sendTransformUpdateMessage = this.sendTransformUpdateMessage.bind(this)
    this.sendTransformClearMessage = this.sendTransformClearMessage.bind(this)
    this.onClickToggleShowTransform = this.onClickToggleShowTransform.bind(this)
  }

  onClickToggleShowTransform(){
    const newVal = this.state.showTransform === false
    this.setState({showTransform: newVal})
  }

  sendTransformUpdateMessage(){
    const {sendFrame3DTransformMsg} = this.props.ros
    const namespace = this.props.namespace + "/set_3d_transform"
    const TX = parseFloat(this.state.transformTX)
    const TY = parseFloat(this.state.transformTY)
    const TZ = parseFloat(this.state.transformTZ)
    const RX = parseFloat(this.state.transformRX)
    const RY = parseFloat(this.state.transformRY)
    const RZ = parseFloat(this.state.transformRZ)
    const HO = parseFloat(this.state.transformHO)
    const transformList = [TX,TY,TZ,RX,RY,RZ,HO]
    sendFrame3DTransformMsg(namespace,transformList)
  }

  sendTransformZeroMessage(){
    this.setState({
      transformTX: 0,
      transformTY: 0,
      transformTZ: 0,
      transformRX: 0,
      transformRY: 0,
      transformRZ: 0,
      transformHO: 0,      
    })
    const {sendClearFrame3DTransformMsg} = this.props.ros
    const namespace = this.props.namespace + "/set_3d_transform"
    const transformList = [0,0,0,0,0,0,0]
    sendClearFrame3DTransformMsg(namespace,transformList)
  }

  sendTransformClearMessage(){
    const {sendTriggerMsg} = this.props.ros
    const namespace = this.props.namespace + "/clear_3d_transform"
    sendTriggerMsg(namespace)
  }

  renderControls() {
    const { npxDevices, sendBoolMsg, sendTriggerMsg, setIdxControlsEnable, setIdxAutoAdjust, setFrame3D } = this.props.ros
    const namespace = this.props.namespace ? this.props.namespace : null
    const message = this.props.status_msg ? this.props.status_msg : null
    const capabilities = npxDevices[namespace] ? npxDevices[namespace] : null


    if (namespace != null && capabilities != null && message != null){
      const update_rate = message.update_rate
      const frame_3d = message.frame_3d
      const transform_msg = message.nepi_frame_3d_transform
      const apply_tf = message.include_transform_enabled
      const frame_nav = message.frame_nav
      const frame_altitude = message.frame_altitude
      const frame_depth = message.frame_depth
      const has_loc = message.has_location
      const set_loc = message.set_as_location_source
      const has_head = message.has_heading
      const set_head = message.set_as_heading_source
      const has_orien = message.has_orientation
      const set_orien = message.set_as_orientation_source
      const has_pos = message.has_position
      const set_pos = message.set_as_position_source
      const has_alt = message.has_altitude
      const set_alt = message.set_as_altitude_source
      const has_depth = message.has_depth
      const set_depth = message.set_as_depth_source

      const transform = this.state.transform_msg
      if (transform !== transform_msg){
        this.setState({
          transform_msg: transform_msg,
          transformTX: transform_msg.translate_vector.x,
          transformTY: transform_msg.translate_vector.y,
          transformTZ: transform_msg.translate_vector.z,
          transformRX: transform_msg.rotate_vector.x,
          transformRY: transform_msg.rotate_vector.y,
          transformRZ: transform_msg.rotate_vector.z,
          transformHO: transform_msg.heading_offset
        })
      }

      const include_transform = message.include_transform_enabled
    
      return (
        <Section title={"NavPose Controls"}>
          
          <div hidden={!has_loc}>    
            <Label title="Set as Location Source">
              <Toggle
                checked={set_loc===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_location_source",!set_loc)}>
              </Toggle>
            </Label>
          </div>

          <div hidden={!has_head}>    
            <Label title="Set as Heading Source">
              <Toggle
                checked={set_head===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_heading_source",!set_head)}>
              </Toggle>
            </Label>
          </div>

          <div hidden={!has_orien}>    
            <Label title="Set as Orientation Source">
              <Toggle
                checked={set_orien===true}
                onClick={() => {this.props.ros.sendBoolMsg(namespace + "/set_as_orientation_source", !set_orien);}}>
              </Toggle>
            </Label>
          </div>
          <div hidden={!has_pos}>    
            <Label title="Set as Position Source">
              <Toggle
                checked={set_pos===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_position_source",!set_pos)}>
              </Toggle>
            </Label>
          </div>

          <div hidden={!has_alt}>    
            <Label title="Set as Altitude Source">
              <Toggle
                checked={set_alt===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_altitude_source",!set_alt)}>
              </Toggle>
            </Label>
          </div>

          <div hidden={!has_depth}>    
            <Label title="Set as Depth Source">
              <Toggle
                checked={set_depth===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_depth_source",!set_depth)}>
              </Toggle>
            </Label>
          </div>

          <Columns>
            <Column>
              <Label title="Enable Nepi Frame 3D Transform">
                <Toggle
                  checked={include_transform}
                  onClick={() => this.props.ros.sendBoolMsg(namespace + '/set_include_transform',!include_transform)}>
                </Toggle>
              </Label>
            </Column>
            <Column>
            </Column>
          </Columns>

          <Columns>
            <Column>
              <Label title="Show 3D Transform">
                <Toggle
                  checked={this.state.showTransform}
                  onClick={this.onClickToggleShowTransform}>
                </Toggle>
              </Label>
            </Column>
            <Column>
            </Column>
          </Columns>

          <div hidden={this.state.showTransform === false}>
            <Columns>
              <Column>
                <Label title={"X (m)"}>
                  <Input
                    value={this.state.transformTX}
                    id="XTranslation"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event,"transformTX")}
                    onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTX")}
                    style={{ width: "80%" }}
                  />
                </Label>

                <Label title={"Y (m)"}>
                  <Input
                    value={this.state.transformTY}
                    id="YTranslation"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event,"transformTY")}
                    onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTY")}
                    style={{ width: "80%" }}
                  />
                </Label>

                <Label title={"Z (m)"}>
                  <Input
                    value={this.state.transformTZ}
                    id="ZTranslation"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event,"transformTZ")}
                    onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTZ")}
                    style={{ width: "80%" }}
                  />
                </Label>

                <ButtonMenu>
                  <Button onClick={() => this.sendTransformUpdateMessage()}>{"Update Transform"}</Button>
                </ButtonMenu>
              </Column>
              
              <Column>
                <Label title={"Roll (deg)"}>
                  <Input
                    value={this.state.transformRX}
                    id="XRotation"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event,"transformRX")}
                    onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRX")}
                    style={{ width: "80%" }}
                  />
                </Label>

                <Label title={"Pitch (deg)"}>
                  <Input
                    value={this.state.transformRY}
                    id="YRotation"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event,"transformRY")}
                    onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRY")}
                    style={{ width: "80%" }}
                  />
                </Label>

                <Label title={"Yaw (deg)"}>
                  <Input
                    value={this.state.transformRZ}
                    id="ZRotation"
                    onChange={(event) => onUpdateSetStateValue.bind(this)(event,"transformRZ")}
                    onKeyDown={(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRZ")}
                    style={{ width: "80%" }}
                  />
                </Label>

                <ButtonMenu>
                  <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + "/clear_3d_transform")}>{"Clear Transform"}</Button>
                </ButtonMenu>
              </Column>
            </Columns>

            <Columns>
              <Column>
                <div align={"left"} textAlign={"left"}>
                  <Label title={"Output Frame"}>
                  </Label>
                </div>
              </Column>
              <Column>
                <div align={"left"} textAlign={"left"}>
                  <Label title={"Current Frame"}>
                    <Input value={this.state.frame_3d} />
                  </Label>
                </div>
              </Column>
            </Columns>

            <Columns>
              <Column>
                <div align={"center"} textAlign={"center"}>
                  <Label title={"NEPI"} align={"center"}>
                  </Label>
                  <Toggle 
                    checked={this.state.frame_3d === "nepi_frame"} 
                    disabled={(!this.state.disabled)? false : true}
                    onClick={() => this.props.ros.setFrame3D(this.props.namespace,"nepi_frame")}
                    />
                </div>
              </Column>
              <Column>
                <div align={"center"} textAlign={"center"}>
                  <Label title={"Earth"} align={"center"}>
                  </Label>
                  <Toggle 
                    checked={this.state.frame_3d === "earth_frame"} 
                    disabled={(!this.state.disabled)? false : true}
                    onClick={() => this.props.ros.setFrame3D(this.props.namespace,"earth_frame")}
                    />
                </div>
              </Column>
            </Columns>
          </div>

          <Columns>
            <Column>
            </Column>
            <Column>
              <ButtonMenu>
                <Button onClick={() => sendTriggerMsg(namespace + "/reset_controls")}>{"Reset Controls"}</Button>
              </ButtonMenu>
            </Column>
          </Columns>

        </Section>
      )
    }
  }

  render() {
    return (
      <Columns>
        <Column>
          {this.renderControls()}
        </Column>
      </Columns>
    )
  }
}

export default NepiDeviceNPXControls