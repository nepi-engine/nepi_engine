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
import { round, onUpdateSetStateValue, onEnterSetStateFloatValue} from "./Utilities"

import NepiIFReset from "./Nepi_IF_Reset"
import NepiIF3DTransform from "./Nepi_IF_3DTransform"

@inject("ros")
@observer

// Component that contains the IDX Device controls
class NepiDeviceIDXControls extends Component {
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
      dataProducts: [],
      frameratesCurrent: [],
      contrastAdjustment: null,
      brightnessAdjustment: null,
      thresholdAdjustment: null,
      rangeMax: null,
      rangeMin: null,
      rangeLimitMinM: null,
      rangeLimitMaxM: null,
      zoomAdjustment: null,
      rotateAdjustment: null,
      tiltAdjustment: null,
      frame3D: null,

      age_filter_s: null,

      listener: null,

      disabled: false,
      frame_3d: null,
  
      avail_pantilt_topics: [],
      pantilt_mounted: false,
      sel_pantilt_name: 'None',
      sel_pantilt_device_topic: 'None',
      sel_pantilt_navpose_topic: 'None',
      sel_pantilt_connected: false
    }


    



    this.updateListener = this.updateListener.bind(this)
    this.statusListener = this.statusListener.bind(this)

    

    
    
  }

  // Callback for handling ROS StatusIDX messages
  statusListener(message) {
    this.setState({
      rtsp_url: message.rtsp_url,
      rtsp_username: message.rtsp_username,
      rtsp_password: message.rtsp_password,
      autoAdjust: message.auto_adjust_enabled,
      resolutionAdjustment: message.resolution_ratio,
      resolutionString : message.resolution_current,
      framerateAdjustment: message.framerate_ratio,
      dataProducts: message.data_products,
      frameratesCurrent: message.framerates,
      contrastAdjustment: message.contrast_ratio,
      brightnessAdjustment: message.brightness_ratio,
      thresholdAdjustment: message.threshold_ratio,
      rangeMax: message.range_window_ratios.stop_range,
      rangeMin: message.range_window_ratios.start_range,
      rangeLimitMinM: message.min_range_m,
      rangeLimitMaxM: message.max_range_m,
      frame_3d: message.frame_3d,
      transform_msg: message.frame_3d_transform,
      avail_pantilt_topics: message.avail_pantilt_topics,
      pantilt_mounted: message.pantilt_mounted,
      sel_pantilt_name: message.sel_pantilt_name,
      sel_pantilt_device_topic: message.sel_pantilt_device_topic,
      sel_pantilt_navpose_topic: message.sel_pantilt_navpose_topic,
      sel_pantilt_connected: message.sel_pantilt_connected,
    })
   
  }

  // Function for configuring and subscribing to StatusIDX
  updateListener() {
    const { namespace } = this.props
    if (this.state.listener) {
      this.state.listener.unsubscribe()
    }
    var listener = this.props.ros.setupIDXStatusListener(
      namespace,
      this.statusListener
    )
    this.setState({ listener: listener, disabled: false })

  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { namespace } = this.props
    if (prevProps.namespace !== namespace){
      if (namespace != null) {
        this.updateListener()
      } else if (namespace === null){
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




 renderLive() {
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

  render() {
    const { idxDevices, sendTriggerMsg, sendBoolMsg, sendStringMsg } = this.props.ros
    const namespace = this.props.namespace ? this.props.namespace : 'None'
    var capabilities = null
    if (namespace != 'None'){
      capabilities = idxDevices[namespace] ? idxDevices[namespace] : null
    }
    if (capabilities == null){
      return (
        <Columns>
          <Column>

          </Column>
        </Columns>
      )
    }
    else {
      const has_resolution = (capabilities && capabilities.has_resolution && !this.state.disabled)
      const has_framerate = (capabilities && capabilities.has_framerate && !this.state.disabled)
      const has_auto_adjust = (capabilities && capabilities.has_auto_adjustment && !this.state.disabled)
      const has_contrast = (capabilities && capabilities.has_contrast && !this.state.disabled)
      const has_brightness = (capabilities && capabilities.has_brightness && !this.state.disabled)
      const has_threshold = (capabilities && capabilities.has_threshold && !this.state.disabled)
      const has_range = (capabilities && capabilities.has_range && !this.state.disabled)
      const data_products = (capabilities && capabilities.data_products && !this.state.disabled)

      const framerates = this.state.frameratesCurrent
      const data_product = this.props.dataProduct ? this.props.dataProduct : 'None'
      if (data_product === 'None'){
        return (
          <Columns>
            <Column>
  
            </Column>
          </Columns>
        )
      }
      else {
        const dp_index = framerates ? this.state.dataProducts.indexOf(data_product) : -1
        var framerate_str = "0"
        if (dp_index !== -1) {
          framerate_str = round(framerates[dp_index],1).toString()
        }

        return (

          <Section title={"Controls"}>

              {/*
              <div hidden={this.state.rtsp_url === ""}>

              {this.renderLive()}

              </div>
              */}

                    <div hidden={(has_framerate === false)}>

                    <SliderAdjustment
                                  title={"Framerate"}
                                  msgType={"std_msgs/Float32"}
                                  adjustment={this.state.framerateAdjustment}
                                  topic={namespace + '/set_framerate_ratio'}
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

                  </div>


                  <div hidden={(has_resolution === false)}>

                    <SliderAdjustment
                                    title={"Resolution"}
                                    msgType={"std_msgs/Float32"}
                                    adjustment={this.state.resolutionAdjustment}
                                    topic={namespace + '/set_resolution_ratio'}
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

                    </div>



                    <div align={"left"} textAlign={"left"} hidden={!has_auto_adjust}>


                      <Columns>
                        <Column>

                              <Label title={"Auto Adjust"}>
                                  <Toggle
                                    checked={this.state.autoAdjust}
                                    onClick={() => sendBoolMsg(namespace + '/set_auto_adjust_enable' ,!this.state.autoAdjust)}
                                  /> 
                                </Label>
                      

                            </Column>
                            <Column>

                            </Column>
                          </Columns>

                      </div>


                      <div hidden={this.state.autoAdjust === true && has_brightness === false}>
                        <SliderAdjustment
                            title={"Brightness"}
                            msgType={"std_msgs/Float32"}
                            adjustment={this.state.brightnessAdjustment}
                            topic={namespace + "/set_brightness_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            tooltip={"Adjustable brightness"}
                            unit={"%"}
                        />

                      </div>


                      <div hidden={this.state.autoAdjust === true && has_contrast === false}>
                        <SliderAdjustment
                          title={"Contrast"}
                          msgType={"std_msgs/Float32"}
                          adjustment={this.state.contrastAdjustment}
                          topic={namespace + "/set_contrast_ratio"}
                          scaled={0.01}
                          min={0}
                          max={100}
                          tooltip={"Adjustable contrast"}
                          unit={"%"}
                        />

                      </div>

                      <div hidden={this.state.autoAdjust === true && has_threshold === false}>
                        <SliderAdjustment
                            title={"Thresholding"}
                            msgType={"std_msgs/Float32"}
                            adjustment={this.state.thresholdAdjustment}
                            topic={namespace + "/set_threshold_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            tooltip={"Adjustable threshold"}
                            unit={"%"}
                        />
                      </div>






                    <div hidden={(has_range === false)}>
                      <RangeAdjustment
                        title="Range Clip"
                        min={this.state.rangeMin}
                        max={this.state.rangeMax}
                        min_limit_m={this.state.rangeLimitMinM}
                        max_limit_m={this.state.rangeLimitMaxM}
                        topic={namespace + "/set_range_window"}
                        tooltip={"Adjustable range"}
                        unit={"m"}
                      />
                    </div>




            <Columns>
                  <Column>

                          <NepiIF3DTransform
                              namespace={namespace}
                              supports_updates={true}
                              title={"Nepi_IF_3DTransform"}
                          />

                  </Column>
              </Columns>



                            
                  <Columns>
                    <Column>
                    <div align={"left"} textAlign={"left"}>
                        <Label title={"Data Output Frame"}>
                        <Input value = {this.state.frame_3d} />
                        </Label>
                      </div>
                    </Column>
                    <Column>

                    </Column>
                  </Columns>


               <NepiIFReset
                        namespace={namespace}
                        title={"Nepi_IF_Reset"}
                  />
          </Section>
        )
      }
    }
  }


}
export default NepiDeviceIDXControls
