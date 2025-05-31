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
      transformTX: 0,
      transformTY: 0,
      transformTZ: 0,
      transformRX: 0,
      transformRY: 0,
      transformRZ: 0,
      transformHO: 0,
      age_filter_s: null,

      listener: null,

      disabled: false,

    }


    this.onClickToggleShowTransforms = this.onClickToggleShowTransforms.bind(this)
    
    this.updateListener = this.updateListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.sendTransformUpdateMessage = this.sendTransformUpdateMessage.bind(this)
    

    
    
  }

  // Callback for handling ROS StatusNPX messages
  statusListener(message) {
    this.setState({

      frame_3d: message.frame_3d,
      transformTX: message.frame_3d_transform.translate_vector.x,
      transformTY: message.frame_3d_transform.translate_vector.y,
      transformTZ: message.frame_3d_transform.translate_vector.z,
      transformRX: message.frame_3d_transform.rotate_vector.x,
      transformRY: message.frame_3d_transform.rotate_vector.y,
      transformRZ: message.frame_3d_transform.rotate_vector.z,
      transformHO: message.frame_3d_transform.heading_offset

    })
  }

  // Function for configuring and subscribing to StatusNPX
  updateListener() {
    const { npxNamespace } = this.props
    if (this.state.listener) {
      this.state.listener.unsubscribe()
    }
    var listener = this.props.ros.setupNPXStatusListener(
      npxNamespace,
      this.statusListener
    )
    this.setState({ listener: listener, disabled: false })

  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { npxNamespace } = this.props
    if (prevProps.npxNamespace !== npxNamespace){
      if (npxNamespace != null) {
        this.updateListener()
      } else if (npxNamespace == null){
        this.setState({ disabled: true })
      }
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to StatusNPX message
  componentWillUnmount() {
    if (this.state.listener) {
      this.state.listener.unsubscribe()
    }
  }

  onClickToggleShowTransforms(){
    const newVal = this.state.showTransforms === false
    this.setState({showTransforms: newVal})
    this.render()
  }

  settransform(event){
    const transforms = this.state.transforms_list

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
    const namespace = this.props.npxNamespace + "/set_frame_transform"
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
    const namespace = this.props.npxNamespace + "/set_frame_transform"
    const transformList = [0,0,0,0,0,0,0]
    sendClearFrame3DTransformMsg(namespace,transformList)
  }



  renderControls() {
    const { npxDevices, sendTriggerMsg, setIdxControlsEnable, setIdxAutoAdjust, setFrame3D } = this.props.ros
    const capabilities = npxDevices[this.props.npxNamespace]
    

    return (

      <Section title={"NavPose Controls"}>

    <div hidden={!has_on_off_control}>    
          <Label title="Set On_Off State">
                  <Toggle
                    checked={this.state.lsxOnOffState===true}
                    onClick={() => this.props.ros.sendBoolMsg(namespace + "/turn_on_off",!this.state.lsxOnOffState)}>
                  </Toggle>
            </Label>
            </div>


                       
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
                  onClick={() => setFrame3D(this.props.npxNamespace + '',"nepi_center_frame")}
                />
          </div>

  

      </Column>
      <Column>

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
                      <Button onClick={() => sendTriggerMsg( namespace + "/clear_3d_transform")}>{"Clear Transform"}</Button>
              </ButtonMenu>


            </Column>
          </Columns>

 
  
        </div>

      </Section>
    )
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
