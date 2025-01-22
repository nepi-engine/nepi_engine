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

// Component that contains the IDX Sensor controls
class NepiSensorsImagingControls extends Component {
  constructor(props) {
    super(props)

    // these states track the values through IDX Status messages
    this.state = {
      controlsEnable: null,
      autoAdjust: null,
      resolutionAdjustment: null,
      framerateAdjustment: null,
      framerateCurrent: null,
      contrastAdjustment: null,
      brightnessAdjustment: null,
      thresholdingAdjustment: null,
      rangeMax: null,
      rangeMin: null,
      rangeLimitMinM: null,
      rangeLimitMaxM: null,
      zoomAdjustment: null,
      rotateAdjustment: null,
      tiltAdjustment: null,
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
      frame_3d: null,
      frame_status: null
    }


    this.onClickToggleShowTransforms = this.onClickToggleShowTransforms.bind(this)
    
    this.updateListener = this.updateListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.sendTransformUpdateMessage = this.sendTransformUpdateMessage.bind(this)
    

    
    
  }

  // Callback for handling ROS StatusIDX messages
  statusListener(message) {
    this.setState({
      controlsEnable: message.controls_enable,
      autoAdjust: message.auto_adjust,
      resolutionAdjustment: message.resolution_mode,
      framerateAdjustment: message.framerate_mode,
      framerateCurrent: message.framerate_current,
      contrastAdjustment: message.contrast,
      brightnessAdjustment: message.brightness,
      thresholdingAdjustment: message.thresholding,
      rangeMax: message.range_window.stop_range,
      rangeMin: message.range_window.start_range,
      rangeLimitMinM: message.min_range_m,
      rangeLimitMaxM: message.max_range_m,
      zoomAdjustment: message.zoom,
      rotateAdjustment: message.rotate,
      tiltAdjustment: message.tilt,
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

  // Function for configuring and subscribing to StatusIDX
  updateListener() {
    const { idxSensorNamespace } = this.props
    if (this.state.listener) {
      this.state.listener.unsubscribe()
    }
    var listener = this.props.ros.setupIDXStatusListener(
      idxSensorNamespace,
      this.statusListener
    )
    this.setState({ listener: listener, disabled: false })

  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { idxSensorNamespace } = this.props
    if (prevProps.idxSensorNamespace !== idxSensorNamespace){
      if (idxSensorNamespace != null) {
        this.updateListener()
      } else if (idxSensorNamespace == null){
        this.setState({ disabled: true })
      }
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to StatusIDX message
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
    const namespace = this.props.idxSensorNamespace + "/idx/set_frame_3d_transform"
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
    const namespace = this.props.idxSensorNamespace + "/idx/set_frame_3d_transform"
    const transformList = [0,0,0,0,0,0,0]
    sendClearFrame3DTransformMsg(namespace,transformList)
  }

  render() {
    const { idxSensors, sendTriggerMsg, setIdxControlsEnable, setIdxAutoAdjust, setFrame3D } = this.props.ros
    const capabilities = idxSensors[this.props.idxSensorNamespace]
    const has_auto_adjust = (capabilities && capabilities.auto_adjustment && !this.state.disabled)
    const has_range_adjust = (capabilities && capabilities.adjustable_range && !this.state.disabled)
    const resetControlsNamespace = this.props.idxSensorNamespace + "/idx/reset_controls"
    const imageName = this.props.idxImageName 
    return (


      <Section title={"Post Processing Controls"}>

              <RadioButtonAdjustment
                  title={"Framerate"}
                  topic={this.props.idxSensorNamespace + '/idx/set_framerate_mode'}
                  msgType={"std_msgs/UInt8"}
                  adjustment={(capabilities && capabilities.adjustable_framerate)? this.state.framerateAdjustment : null}
                  disabled={(capabilities && capabilities.adjustable_framerate && !this.state.disabled)? false : true}
                  entries={["Low", "Medium", "High", "Ultra"]}
              />


          <Columns>
            <Column>

              <Label title={"Current Framerate"}>
              <Input
                value={this.state.framerateCurrent}
                id="framerate"
                style={{ width: "40%" }}
                disabled={true}
              />
            </Label>
   
              <div align={"left"} textAlign={"left"}>
                <Label title={"Enable Controls"}>
                  <Toggle
                  checked={this.state.controlsEnable}
                  onClick={() => setIdxControlsEnable(this.props.idxSensorNamespace,!this.state.controlsEnable)}
                  />
                </Label>
              </div>

                </Column>
                <Column>

                    <ButtonMenu>
                      <Button onClick={() => sendTriggerMsg(resetControlsNamespace)}>{"Reset Controls"}</Button>
                    </ButtonMenu>
   
                </Column>
              </Columns>
        



          <div hidden={!this.state.controlsEnable }>

            <div hidden={(imageName !== 'bw_2d_image' && imageName !== 'color_2d_image')}>


              <div align={"left"} textAlign={"left"} hidden={!has_auto_adjust}>
                <Columns>
                  <Column>

                        <Label title={"Auto Adjust"}>
                            <Toggle
                              checked={this.state.autoAdjust}
                              onClick={() => setIdxAutoAdjust(this.props.idxSensorNamespace,!this.state.autoAdjust)}
                            /> 
                          </Label>
                

                      </Column>
                      <Column>

                      </Column>
                    </Columns>

              </div>


              <div hidden={this.state.autoAdjust}>
                <SliderAdjustment
                    title={"Brightness"}
                    msgType={"std_msgs/Float32"}
                    adjustment={this.state.brightnessAdjustment}
                    topic={this.props.idxSensorNamespace + "/idx/set_brightness"}
                    scaled={0.01}
                    min={0}
                    max={100}
                    disabled={(capabilities && capabilities.adjustable_brightness && !this.state.disabled)? false : true}
                    tooltip={"Adjustable brightness"}
                    unit={"%"}
                />
                <SliderAdjustment
                  title={"Contrast"}
                  msgType={"std_msgs/Float32"}
                  adjustment={this.state.contrastAdjustment}
                  topic={this.props.idxSensorNamespace + "/idx/set_contrast"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  disabled={(capabilities && capabilities.adjustable_contrast && !this.state.disabled)? false : true}
                  tooltip={"Adjustable contrast"}
                  unit={"%"}
                />
                <SliderAdjustment
                    title={"Thresholding"}
                    msgType={"std_msgs/Float32"}
                    adjustment={this.state.thresholdingAdjustment}
                    topic={this.props.idxSensorNamespace + "/idx/set_thresholding"}
                    scaled={0.01}
                    min={0}
                    max={100}
                    disabled={(capabilities && capabilities.adjustable_thresholding && !this.state.disabled)? false : true}
                    tooltip={"Adjustable thresholding"}
                    unit={"%"}
                />
              </div>
           
              <RadioButtonAdjustment
                  title={"Resolution"}
                  topic={this.props.idxSensorNamespace + '/idx/set_resolution_mode'}
                  msgType={"std_msgs/UInt8"}
                  adjustment={(capabilities && capabilities.adjustable_resolution)? this.state.resolutionAdjustment : null}
                  disabled={(capabilities && capabilities.adjustable_resolution && !this.state.disabled)? false : true}
                  entries={["Low", "Medium", "High", "Ultra"]}
              />

            </div>

            <div hidden={!has_range_adjust || (imageName !== 'depth_image' && imageName !== 'depth_map' && imageName !== 'pointcloud_image')}>
              <RangeAdjustment
                title="Range Clip"
                min={this.state.rangeMin}
                max={this.state.rangeMax}
                min_limit_m={this.state.rangeLimitMinM}
                max_limit_m={this.state.rangeLimitMaxM}
                topic={this.props.idxSensorNamespace + "/idx/set_range_window"}
                disabled={(capabilities && capabilities.adjustable_range && !this.state.disabled)? false : true}
                tooltip={"Adjustable range"}
                unit={"m"}
              />
            </div>


            <div hidden={ imageName !== 'pointcloud_image'}>

                <SliderAdjustment
                      title={"Zoom"}
                      msgType={"std_msgs/Float32"}
                      adjustment={this.state.zoomAdjustment}
                      topic={this.props.idxSensorNamespace + "/idx/set_zoom_ratio"}
                      scaled={0.01}
                      min={0}
                      max={100}
                      disabled={false}
                      tooltip={"Zoom controls for pointcloud image rendering"}
                      unit={"%"}
                  />


                <SliderAdjustment
                      title={"Rotate"}
                      msgType={"std_msgs/Float32"}
                      adjustment={this.state.rotateAdjustment}
                      topic={this.props.idxSensorNamespace + "/idx/set_rotate_ratio"}
                      scaled={0.01}
                      min={0}
                      max={100}
                      disabled={false}
                      tooltip={"Rotate controls for pointcloud image rendering"}
                      unit={"%"}
                  />

                  <SliderAdjustment
                      title={"Tilt"}
                      msgType={"std_msgs/Float32"}
                      adjustment={this.state.tiltAdjustment}
                      topic={this.props.idxSensorNamespace + "/idx/set_tilt_ratio"}
                      scaled={0.01}
                      min={0}
                      max={100}
                      disabled={false}
                      tooltip={"Tilt controls for pointcloud image rendering"}
                      unit={"%"}
                  />
                          </div>
                          </div>

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
            <Button onClick={() => this.sendClearTransformUpdateMessage()}>{"Clear Transform"}</Button>
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
                    onClick={() => setFrame3D(this.props.idxSensorNamespace + '/idx',"nepi_center_frame")}
                  />
                </div>
     
                </Column>
                <Column>
                <div align={"center"} textAlign={"center"}>
                  <Label title={"Earth"} align={"center"}>
                  </Label>
                  <Toggle 
                    checked={this.state.frame_3d === "map"} 
                    disabled={(!this.state.disabled)? false : true}
                    onClick={() => setFrame3D(this.props.idxSensorNamespace + '/idx',"map")}
                  />
                   </div>

                </Column>
              </Columns>
        </div>

      </Section>
    )
  }

}
export default NepiSensorsImagingControls
