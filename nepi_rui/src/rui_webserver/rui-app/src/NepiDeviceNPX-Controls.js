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
//import EnableAdjustment from "./EnableAdjustment"
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

      showTransforms: false,
      transforms_topic_list: [],
      transforms_list: [],
      transforms_msg: null,
      transformTX: 0,
      transformTY: 0,
      transformTZ: 0,
      transformRX: 0,
      transformRY: 0,
      transformRZ: 0,
      transformHO: 0,
      age_filter_s: null



    }


    this.onClickToggleShowTransforms = this.onClickToggleShowTransforms.bind(this)
    
    this.updateListener = this.updateListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.sendTransformUpdateMessage = this.sendTransformUpdateMessage.bind(this)
    

    
    
  }

  onClickToggleShowTransforms(){
    const newVal = this.state.showTransforms === false
    this.setState({showTransforms: newVal})
    this.render()
  }

  settransform(event){
    const pointcloud = event.target.value
    const pointclouds = this.state.transforms_topic_list
    const transforms = this.state.transforms_list
    const tf_index = pointclouds.indexOf(pointcloud)
    if (tf_index !== -1){
      this.setState({
        transformPointcloud: pointcloud,
        transformInd: tf_index
      })
      const transform = transforms[tf_index]
      this.setState({
        transformTX: round(transform[0]),
        transformTY: round(transform[1]),
        transformTZ: round(transform[2]),
        transformRX: round(transform[3]),
        transformRY: round(transform[4]),
        transformRZ: round(transform[5]),
        transformHO: round(transform[6])
      })
      
    }
  }

  sendTransformUpdateMessage(){
    const {sendFrame3DTransformMsg} = this.props.ros
    const namespace = this.props.namespace + "/set_frame_transform"
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


  sendClearTransformUpdateMessage(){
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
    const namespace = this.props.namespace + "/set_frame_transform"
    const transformList = [0,0,0,0,0,0,0]
    sendClearFrame3DTransformMsg(namespace,transformList)
  }



  renderControls() {
    const { npxDevices, sendTriggerMsg, setIdxControlsEnable, setIdxAutoAdjust, setFrame3D } = this.props.ros
    const namespace = this.props.namespace ? this.props.namespace : null
    const message = this.props.navposeData ? this.props.navposeData : null
    const capabilities = npxDevices[namespace] ? npxDevices[namespace] : null

    if (namespace != null && capabilities != null && message != null){
      const transforms = this.state.transforms_msg
      
      const has_gps_pub = capabilities ? capabilities.has_gps_pub : false
      const has_elevation_pub = capabilities ? capabilities.has_elevation_pub : false
      const has_pose_pub = capabilities ? capabilities.has_pose_pub : false
      const has_heading_pub = capabilities ? capabilities.has_heading_pub : false

      const update_rate = message.update_rate
      const frame_id = message.frame_id
      const frame_3d = message.frame_3d
      const frame_altitude = message.frame_altitude

      const has_heading = message.has_heading
      const has_position = message.has_position
      const has_orientation = message.has_orientation
      const has_location = message.has_location
      const has_altitude = message.has_altitude
      const has_depth = message.has_depth
      const set_as_gps_source = message.set_as_gps_source
      const set_as_elevation_source = message.set_as_elevation_source
      const set_as_pose_source = message.set_as_pose_source
      const set_as_heading_source = message.set_as_heading_source

      const transforms_msg = message.frame_transform

    if (transforms !== message.frame_transform){
      this.setState({
        transforms_msg: transforms_msg,
        transformTX: message.frame_transform.translate_vector.x,
        transformTY: message.frame_transform.translate_vector.y,
        transformTZ: message.frame_transform.translate_vector.z,
        transformRX: message.frame_transform.rotate_vector.x,
        transformRY: message.frame_transform.rotate_vector.y,
        transformRZ: message.frame_transform.rotate_vector.z,
        transformHO: message.frame_transform.heading_offset
      })
    }
      return (

        <Section title={"NavPose Controls"}>
        <Columns>
        <Column>
      <div hidden={!has_gps_pub}>    
            <Label title="Set as GPS Source">
              <Toggle
                checked={this.state.set_as_gps_source===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_gps_source",!this.state.set_as_gps_source)}>
              </Toggle>
            </Label>
      </div>
      <div hidden={!has_elevation_pub}>    
            <Label title="Enable Elevation">
              <Toggle
                checked={this.state.set_as_elevation_source===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_elevation_source",!this.state.set_as_elevation_source)}>
              </Toggle>
            </Label>
      </div>
      <div hidden={!has_pose_pub}>    
            <Label title="Enable Pose">
              <Toggle
                checked={this.state.set_as_pose_source===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_pose_source",!this.state.set_as_pose_source)}>
              </Toggle>
            </Label>
      </div>
      <div hidden={!has_heading_pub}>    
            <Label title="Enable Heading">
              <Toggle
                checked={this.state.set_as_heading_source===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_heading_source",!this.state.set_as_heading_source)}>
              </Toggle>
            </Label>
      </div>
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
                    <Input value = {this.state.frame_id} />
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
                    checked={this.state.frame_id === "nepi_center_frame"} 
                    disabled={(!this.state.disabled)? false : true}
                    onClick={() => this.props.ros.setFrame3D(namespace + '',"nepi_center_frame")}
                  />
            </div>

    

        </Column>
        <Column>
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
                    <Input value = {this.state.frame_3d} />
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
                    checked={this.state.frame_3d === "nepi_center_frame"} 
                    disabled={(!this.state.disabled)? false : true}
                    onClick={() => this.props.ros.setFrame3D(this.props.npxNamespace + '',"nepi_center_frame")}
                  />
            </div>

    

        </Column>
        <Column>
        </Column>
      </Columns>
      </Column> 
      </Columns> 
    <Columns>
      <Column>
      <Label title="Show 3D Transforms">
        <Toggle
          checked={this.state.showTransforms}
          onClick={this.onClickToggleShowTransforms}>
        </Toggle>
      </Label>

      </Column>
      <Column>
      </Column>
      </Columns>



      <div hidden={ this.state.showTransforms === false}>

            <Columns>
            <Column>

            <Label title={"X (m)"}>
              <Input
                value={this.state.transformTX}
                id="XTranslation"
                onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformTX")}
                onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTX")}
                style={{ width: "80%" }}
              />
            </Label>

            <Label title={"Y (m)"}>
              <Input
                value={this.state.transformTY}
                id="YTranslation"
                onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformTY")}
                onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTY")}
                style={{ width: "80%" }}
              />
            </Label>

            <Label title={"Z (m)"}>
              <Input
                value={this.state.transformTZ}
                id="ZTranslation"
                onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformTZ")}
                onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTZ")}
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
                onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformRX")}
                onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRX")}
                style={{ width: "80%" }}
              />
            </Label>

            <Label title={"Pitch (deg)"}>
              <Input
                value={this.state.transformRY}
                id="YRotation"
                onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformRY")}
                onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRY")}
                style={{ width: "80%" }}
              />
            </Label>

                <Label title={"Yaw (deg)"}>
                  <Input
                    value={this.state.transformRZ}
                    id="ZRotation"
                    onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformRZ")}
                    onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRZ")}
                    style={{ width: "80%" }}
                  />
                </Label>


                <ButtonMenu>
                        <Button onClick={() => this.props.ros.sendTriggerMsg( namespace + "/clear_frame_transform")}>{"Clear Transform"}</Button>
                </ButtonMenu>


              </Column>
            </Columns>

  
    
          </div>
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
