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

      rtsp_url: "",
      rtsp_username: "",
      rtsp_password: "",
      controlsEnable: true,
      autoAdjust: null,
      resolutionAdjustment: null,
      resolutionString: null,
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
      rtsp_url: message.rtsp_url,
      rtsp_username: message.rtsp_username,
      rtsp_password: message.rtsp_password,
      controlsEnable: message.controls_enable,
      autoAdjust: message.auto_adjust,
      resolutionAdjustment: message.resolution_ratio,
      resolutionString : message.resolution_current,
      framerateAdjustment: message.framerate_ratio,
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



 renderLive() {
    const { idxSensors} = this.props.ros
    const rtsp_url = this.state.rtsp_url
    const rtsp_username = this.state.rtsp_username
    const rtsp_password = this.state.rtsp_password
    
    return (

      <Section title={"Open RSTP Camera Stream"}>

      <Columns>
      <Column>


      <ButtonMenu>
        <Button onClick={() => window.open(rtsp_url, '_blank').focus()}>{"Open Live Stream"}</Button>
      </ButtonMenu>

      </Column>
      <Column>

      <pre style={{ height: "50px", overflowY: "auto" }} align={"center"} textAlign={"center"}>
        {"URL: " + rtsp_url + "\nUsername: " + rtsp_username + "\nPassword: " + rtsp_password}
        </pre>


      </Column>
      </Columns>

      </Section>
    )
  }

  renderControls() {
    const { idxSensors, sendTriggerMsg, setIdxControlsEnable, setIdxAutoAdjust, setFrame3D } = this.props.ros
    const capabilities = idxSensors[this.props.idxSensorNamespace]
    const has_auto_adjust = (capabilities && capabilities.auto_adjustment && !this.state.disabled)
    const has_range_adjust = (capabilities && capabilities.adjustable_range && !this.state.disabled)
    const resetControlsNamespace = this.props.idxSensorNamespace + "/idx/reset_controls"
    const imageName = this.props.idxImageName 
    const framerate_str = round(this.state.framerateCurrent,1).toString()
    return (

      <Section title={"Post Processing Controls"}>



      <div hidden={!this.state.controlsEnable }>


              <div hidden={(imageName !== 'bw_2d_image' && imageName !== 'color_2d_image')}>


              <SliderAdjustment
                              title={"Resolution"}
                              msgType={"std_msgs/Float32"}
                              adjustment={this.state.resolutionAdjustment}
                              topic={this.props.idxSensorNamespace + '/idx/set_resolution_ratio'}
                              scaled={0.01}
                              min={0}
                              max={100}
                              disabled={(capabilities && !this.state.disabled)? false : true}
                              tooltip={"Adjustable Resolution"}
                              unit={"%"}
                          />

                  <Columns>
                  <Column>



                      </Column>
                      <Column>

                      <Label title={"Current Resolution"}>
                    <Input
                      value={this.state.resolutionString}
                      id="framerate"
                      style={{ width: "100%" }}
                      disabled={true}
                    />
                  </Label>

                      </Column>
                    </Columns>             


                    <SliderAdjustment
                              title={"Framerate"}
                              msgType={"std_msgs/Float32"}
                              adjustment={this.state.framerateAdjustment}
                              topic={this.props.idxSensorNamespace + '/idx/set_framerate_ratio'}
                              scaled={0.01}
                              min={0}
                              max={100}
                              disabled={(capabilities && !this.state.disabled)? false : true}
                              tooltip={"Adjustable Framerate"}
                              unit={"%"}
                          />

                  <Columns>
                    <Column>


        
                      </Column>
                      <Column>
                      <Label title={"Current Framerate"}>
                    <Input
                      value={framerate_str}
                      id="framerate"
                      style={{ width: "100%" }}
                      disabled={true}
                    />
                  </Label>
        
                      </Column>
                    </Columns>
        



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

             <Columns>
              <Column>

        
                </Column>
                <Column>

                    <ButtonMenu>
                      <Button onClick={() => sendTriggerMsg(resetControlsNamespace)}>{"Reset Controls"}</Button>
                    </ButtonMenu>
   
                </Column>
              </Columns>

      </Section>
    )
  }

  render() {
    return (
      <Columns>
          <Column>

      <div hidden={this.state.rtsp_url === ""}>

      {this.renderLive()}

      </div>

      {this.renderControls()}

      </Column>
      </Columns>

    )
  }


}
export default NepiSensorsImagingControls
