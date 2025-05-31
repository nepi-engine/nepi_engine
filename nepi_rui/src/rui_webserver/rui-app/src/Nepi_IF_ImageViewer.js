/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
//import moment from "moment"
import { observer, inject } from "mobx-react"

import Section from "./Section"
//import EnableAdjustment from "./EnableAdjustment"
import Button, { ButtonMenu } from "./Button"
import Toggle from "react-toggle"
import Label from "./Label"
import RangeAdjustment from "./RangeAdjustment"
import {RadioButtonAdjustment, SliderAdjustment} from "./AdjustmentWidgets"
import { Column, Columns } from "./Columns"
import Styles from "./Styles"
import Select from "./Select"
import Input from "./Input"


import { createMenuListFromStrList, onChangeSwitchStateValue, onChangeSwitchSendBoolValue, onEnterSendFloatValue } from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}


const styles = Styles.Create({
  canvas: {
    width: "100%",
    height: "auto",
    transform: "scale(1)"
  }
})

const COMPRESSION_HIGH_QUALITY = 95
const COMPRESSION_MED_QUALITY = 50
const COMPRESSION_LOW_QUALITY = 10

const PORT = 9091
const ROS_WEBCAM_URL_BASE = `http://${
  window.location.hostname
}:${PORT}/stream?topic=`

@inject("ros")
@observer
class ImageViewer extends Component {
  constructor(props) {
    super(props)

    this.state = {
      hasInitialized: false,
      shouldUpdate: true,
      streamWidth: null,
      streamHeight: null,
      streamSize: 0,
      currentStreamingImageQuality: COMPRESSION_HIGH_QUALITY,
      status_listenter: null,
      status_msg: null,
      show_stats: false,
      show_controls: false,
      show_overlays: false,
      connected: false
    }
    this.updateFrame = this.updateFrame.bind(this)
    this.onCanvasRef = this.onCanvasRef.bind(this)
    this.updateImageSource = this.updateImageSource.bind(this)
    this.onChangeImageQuality = this.onChangeImageQuality.bind(this)
    this.renderControls = this.renderControls.bind(this)
    this.renderOverlays = this.renderOverlays.bind(this)
    this.renderStats = this.renderStats.bind(this)
    this.getImgStatsText = this.getImgStatsText.bind(this)

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)

  }

  updateFrame() {
    const square_canvas = this.props.squareCanvas ? this.props.squareCanvas : false
    const shouldUpdate = this.state.shouldUpdate
    const hasInitialized = this.state.hasInitialized
    const connected = this.state.connected

    if (shouldUpdate === true && hasInitialized === true && this.canvas) {

      // Firset check for image size change
      const { width, height } = this.image
      const streamWidth = this.state.streamWidth
      const streamHeight = this.state.streamHeight

      const width_changed = width !== streamWidth
      const height_changed = height  !== streamHeight
      const size_changed = (width_changed === true || height_changed === true)
      if (size_changed === true || connected === false) {
        this.setState({
            hasInitialized: true,
            streamWidth: width,
            streamHeight: height,
            streamSize: width * height,
            connected: true
            })

        if (square_canvas === true){
          var size = 0
          if (width > height){
            size = width
          }
          else {
            size = height
          }
          this.canvas.width = size
          this.canvas.height = size

        }
        else {
          this.canvas.width = width
          this.canvas.height = height
        }
      }

      // Then update
      const context = this.canvas.getContext("2d")
      context.fillStyle = "red"
      context.textAlign = "center"
      context.font = "50px Arial"
      context.clearRect(0, 0, streamWidth, streamHeight)
      context.drawImage(this.image, 0, 0, streamWidth, streamHeight)
      //this.setState({ clockTime: moment() })
      requestAnimationFrame(this.updateFrame)
      

    }
  }
  

  onCanvasRef(ref) {
    this.canvas = ref
    //this.updateImageSource()
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({
      status_msg: message
    })    

  }

  // Function for configuring and subscribing to Status
  updateStatusListener() {
    const statusNamespace = this.props.imageTopic + '/status'
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({status_msg: null})

    }
    var status_listenter = this.props.ros.setupStatusListener(
          statusNamespace,
          "nepi_ros_interfaces/ImageStatus",
          this.statusListener
        )
    this.setState({ status_listenter: status_listenter})
  }



  updateImageSource() {
    if (this.props.imageTopic) {
      if (!this.image) {
        this.image = new Image() // EXPERIMENT -- Only create a new Image when strictly required
      }
      this.image.crossOrigin = "Anonymous"
      this.image.onload = () => {
        const { width, height } = this.image
        this.setState(
          {
            hasInitialized: true,
            connected: false,
            streamWidth: width,
            streamHeight: height,
            streamSize: width * height
          },
          () => {
            requestAnimationFrame(this.updateFrame)
          }
        )
      }
    }
    if (this.image) {
      const { streamingImageQuality } = this.props.ros
      this.image.src = ROS_WEBCAM_URL_BASE + this.props.imageTopic + '&quality=' + streamingImageQuality
    }
  }

  // Lifecycle method called when the props change.
  // Used to track changes in the image topic value
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { imageTopic } = this.props
    const size = this.state.streamSize
    const width = (this.image) ? this.image.width : 0
    const height = (this.image) ? this.image.height : 0
    const got_size = width * height
    const size_changed = (size !== got_size)
    if (prevProps.imageTopic !== imageTopic || size_changed === true || prevState.currentStreamingImageQuality !== this.state.currentStreamingImageQuality){
      this.updateImageSource()
      if (prevProps.imageTopic !== imageTopic) {
        this.updateStatusListener()
      }
    }
  }

  componentWillUnmount() {
    this.setState({ shouldUpdate: false })
    if (this.image) {
      this.image.src = null
    }
  }

  componentDidMount() {
    this.updateImageSource()
  }

  onChangeImageQuality(quality) {
    this.props.ros.onChangeStreamingImageQuality(quality)
    this.setState({currentStreamingImageQuality: quality})
    this.updateImageSource()

  }


  getImgStatsText(){
    const status_msg = this.state.status_msg
    var msg = ""
    if (status_msg != null){
      const get_lat = round(status_msg.get_latency_time, 3)
      const pup_lat = round(status_msg.pub_latency_time, 3)
      const proc_time = round(status_msg.process_time, 3)
      msg = ("\n Get Latency: " + get_lat + "  Publish Latency: " + pup_lat + 
      "\n Process Times (Image): " + proc_time)
    }
    else {
      msg = "No Stats Available"

    }
    return msg
  }

  renderStats() {
   
    if (this.state.status_msg != null){
      const show_stats = this.state.show_stats
      const img_stats_text = this.getImgStatsText()
      return (
        <Columns>
        <Column>
    
        <Label title="Show Stats">
        <Toggle
        checked={show_stats===true}
        onClick={() => onChangeSwitchStateValue.bind(this)("show_stats",this.state.show_stats)}>
        </Toggle>
        </Label>

        <div hidden={show_stats == false}>

        <pre style={{ height: "100px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {img_stats_text}
                  </pre>
    
        </div>
        </Column>
        </Columns>

      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }



  renderControls() {

    const namespace = this.props.imageTopic

    const { imageDevices, sendTriggerMsg, setIdxControlsEnable, setIdxAutoAdjust, setFrame3D } = this.props.ros
    const capabilities = imageDevices[namespace]

   
    if (this.state.status_msg != null && namespace != null){

      const has_auto_adjust = (capabilities && capabilities.has_auto_adjustment && !this.state.disabled)
      const has_contrast = (capabilities && capabilities.has_contrast && !this.state.disabled)
      const has_brightness = (capabilities && capabilities.has_brightness && !this.state.disabled)
      const has_threshold = (capabilities && capabilities.has_threshold && !this.state.disabled)
      const has_resolution = (capabilities && capabilities.has_resolution && !this.state.disabled)
      const has_framerate = (capabilities && capabilities.has_framerate && !this.state.disabled)
      const has_range = (capabilities && capabilities.has_range && !this.state.disabled)
      const has_zoom = (capabilities && capabilities.has_zoom && !this.state.disabled)
      const has_pan = (capabilities && capabilities.has_pan && !this.state.disabled)
      const has_window = (capabilities && capabilities.has_window && !this.state.disabled)
      const has_rotate = (capabilities && capabilities.has_rotate && !this.state.disabled)
      const has_tilt = (capabilities && capabilities.has_tilt && !this.state.disabled)


      const message = this.state.status_msg
      const resolution_ratio = message.resolution_ratio
      const auto_adjust_enabled = message.auto_adjust_enabled
      const brightness_ratio = message.brightness_ratio
      const contrast_ratio = message.contrast_ratio
      const threshold_ratio = message.threshold_ratio
      const range_start_ratio = message.range_window.start_range
      const range_stop_ratio = message.range_window.stop_range
      const zoom_ratio = message.zoom_ratio
      const pan_lr_ratio = message.pan_left_right_ratio
      const pan_ud_ratio = message.pan_up_down_ratio
      const window_ratios = message.window_ratios
      const rotate_ratio = message.rotate_ratio
      const tilt_ratio = message.tilt_ratio

      const show_controls = this.state.show_controls



      return (

        <Columns>
        <Column>


                <Columns>
                  <Column>
              
                  <Label title="Show Controls">
                  <Toggle
                  checked={show_controls===true}
                  onClick={() => onChangeSwitchStateValue.bind(this)("show_controls",this.state.show_controls)}>
                  </Toggle>
                  </Label>

                  </Column>
                  <Column>

                </Column>
              </Columns>

      
 

              <div hidden={(has_resolution !== true  || show_controls == false)}>

                            <SliderAdjustment
                                  title={"Resolution"}
                                  msgType={"std_msgs/Float32"}
                                  adjustment={this.state.resolutionAdjustment}
                                  topic={this.props.idxNamespace + '/set_resolution_ratio'}
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




            <div hidden={(has_auto_adjust !== true  || show_controls == false)}>
            
                    <Columns>
                    <Column>



                            <Label title={"Auto Adjust"}>
                                <Toggle
                                  checked={this.state.autoAdjust}
                                  onClick={() => setIdxAutoAdjust(this.props.idxNamespace,!this.state.autoAdjust)}
                                /> 
                              </Label>


                        </Column>
                        <Column>

                        </Column>
                      </Columns>

                </div>


          <div hidden={(this.state.autoAdjust !== true  || show_controls == false)}>
                      <SliderAdjustment
                          title={"Brightness"}
                          msgType={"std_msgs/Float32"}
                          adjustment={this.state.brightnessAdjustment}
                          topic={this.props.idxNamespace + "/set_brightness"}
                          scaled={0.01}
                          min={0}
                          max={100}
                          disabled={(capabilities && capabilities.has_brightness && !this.state.disabled)? false : true}
                          tooltip={"Adjustable brightness"}
                          unit={"%"}
                      />

            </div>


            <div hidden={(this.state.autoAdjust !== true  || show_controls == false)}>
                      <SliderAdjustment
                        title={"Contrast"}
                        msgType={"std_msgs/Float32"}
                        adjustment={this.state.contrastAdjustment}
                        topic={this.props.idxNamespace + "/set_contrast"}
                        scaled={0.01}
                        min={0}
                        max={100}
                        disabled={(capabilities && capabilities.has_contrast && !this.state.disabled)? false : true}
                        tooltip={"Adjustable contrast"}
                        unit={"%"}
                      />

            </div>

            <div hidden={(this.state.autoAdjust !== true  || show_controls == false)}>
                      <SliderAdjustment
                          title={"Thresholding"}
                          msgType={"std_msgs/Float32"}
                          adjustment={this.state.thresholdAdjustment}
                          topic={this.props.idxNamespace + "/set_threshold"}
                          scaled={0.01}
                          min={0}
                          max={100}
                          disabled={(capabilities && capabilities.has_threshold && !this.state.disabled)? false : true}
                          tooltip={"Adjustable threshold"}
                          unit={"%"}
                      />
              </div>




            <div hidden={(has_range !== true  || show_controls == false)}>
                    <RangeAdjustment
                      title="Range Clip"
                      min={this.state.rangeMin}
                      max={this.state.rangeMax}
                      min_limit_m={this.state.rangeLimitMinM}
                      max_limit_m={this.state.rangeLimitMaxM}
                      topic={this.props.idxNamespace + "/set_range_window"}
                      tooltip={"Adjustable range"}
                      unit={"m"}
                    />
          </div>


          <div hidden={(has_zoom !== true  || show_controls == false)}>

                      <SliderAdjustment
                            title={"Zoom"}
                            msgType={"std_msgs/Float32"}
                            adjustment={this.state.zoomAdjustment}
                            topic={this.props.idxNamespace + "/set_zoom_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Zoom controls"}
                            unit={"%"}
                        />
          </div>

          <div hidden={(has_pan !== true  || show_controls == false)}>

                      <SliderAdjustment
                            title={"Pan Left-Right"}
                            msgType={"std_msgs/Float32"}
                            adjustment={this.state.rotateAdjustment}
                            topic={this.props.idxNamespace + "/set_pan_lr_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Pan left-right controls"}
                            unit={"%"}
                        />

                      <SliderAdjustment
                            title={"Pan Up-Down"}
                            msgType={"std_msgs/Float32"}
                            adjustment={this.state.rotateAdjustment}
                            topic={this.props.idxNamespace + "/set_pan_up_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Pan up-down controls"}
                            unit={"%"}
                        />

          </div>

          <div hidden={(has_rotate !== true  || show_controls == false)}>

                      <SliderAdjustment
                            title={"Rotate"}
                            msgType={"std_msgs/Float32"}
                            adjustment={this.state.rotateAdjustment}
                            topic={this.props.idxNamespace + "/set_rotate_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Rotate controls"}
                            unit={"%"}
                        />

          </div>

          <div hidden={(has_tilt !== true  || show_controls == false)}>

                        <SliderAdjustment
                            title={"Tilt"}
                            msgType={"std_msgs/Float32"}
                            adjustment={this.state.tiltAdjustment}
                            topic={this.props.idxNamespace + "/set_tilt_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Tilt controls"}
                            unit={"%"}
                        />
          </div>




      </Column>
      </Columns>
      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }



  renderOverlays() {

    const namespace = this.props.imageTopic
   
    if (this.state.status_msg != null && namespace != null){
      const show_overlays = this.state.show_overlays
      const message = this.state.status_msg
      const name = message.overlay_img_name
      const date = message.overlay_date_time
      const nav = message.overlay_nav
      const pose = message.overlay_pose


      return (

        <Columns>
        <Column>


              <Columns>
              <Column>
          
              <Label title="Show Overlays">
              <Toggle
              checked={show_overlays===true}
              onClick={() => onChangeSwitchStateValue.bind(this)("show_overlays",this.state.show_overlays)}>
              </Toggle>
              </Label>

              </Column>
              <Column>

              </Column>
            </Columns>

        <div hidden={show_overlays == false}>

            <Columns>
            <Column>

                  <Label title={"Source Name"}>
                      <Toggle
                        checked={name}
                        onClick={() => onChangeSwitchSendBoolValue.bind(this)(namespace + '/set_overlay_source_name',!name)}
                      /> 
                    </Label>

                    <Label title={"Date Time"}>
                      <Toggle
                        checked={date}
                        onClick={() => onChangeSwitchSendBoolValue.bind(this)(namespace + '/set_overlay_date_time',!date)}
                      /> 
                    </Label>

                    <Label title={"Location"}>
                      <Toggle
                        checked={nav}
                        onClick={() => onChangeSwitchSendBoolValue.bind(this)(namespace + '/set_overlay_nav',!nav)}
                      /> 
                    </Label>

                    <Label title={"Pose"}>
                      <Toggle
                        checked={pose}
                        onClick={() => onChangeSwitchSendBoolValue.bind(this)(namespace + '/set_overlay_pose',!pose)}
                      /> 
                    </Label>


                </Column>
                <Column>

                </Column>
              </Columns>

       </div>

      </Column>
      </Columns>
      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }





  render() {
    const {
      streamingImageQuality
    } = this.props.ros
    const hideQualitySelector = this.props.hideQualitySelector ? this.props.hideQualitySelector: false


    if (streamingImageQuality !== this.state.currentStreamingImageQuality)
    {
      this.setState({currentStreamingImageQuality: streamingImageQuality})
    }

    return (
      <Section title={this.props.title}>

        <canvas style={styles.canvas} ref={this.onCanvasRef} />

        <Columns>
          <Column>

          {this.renderControls()}

          </Column>
          <Column>

          {this.renderOverlays()}

          </Column>
          <Column>

        {this.renderStats()}


          </Column>
        </Columns>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>      

        <Columns>
          <Column>
          <div align={"left"} textAlign={"left"}>
            { hideQualitySelector ?
              null :
              <Label title={"Compression Level"} />
            }
          </div>
          </Column>
          <Column>
          <div align={"left"} textAlign={"left"}>
            { hideQualitySelector ?
              null :
              <div>
                <Label title={"Low"} />
                <Toggle
                  checked={streamingImageQuality >= COMPRESSION_HIGH_QUALITY}
                  onClick={() => {this.onChangeImageQuality(COMPRESSION_HIGH_QUALITY)}}
                />
              </div>
            }
          </div>
          </Column>
          <Column>
          <div align={"left"} textAlign={"left"}>
            { hideQualitySelector ?
              null :
              <div>
                <Label title={"Medium"} />
                <Toggle
                  checked={streamingImageQuality >= COMPRESSION_MED_QUALITY && streamingImageQuality < COMPRESSION_HIGH_QUALITY}
                  onClick={() => {this.onChangeImageQuality(COMPRESSION_MED_QUALITY)}}
                />
              </div>
            }
          </div>
          </Column>
          <Column>
          <div align={"left"} textAlign={"left"}>
            { hideQualitySelector ?
              null :
              <div>
                <Label title={"High"} />
                <Toggle
                  checked={streamingImageQuality <= COMPRESSION_LOW_QUALITY}
                  onClick={() => {this.onChangeImageQuality(COMPRESSION_LOW_QUALITY)}}
                />
              </div>
            }
          </div>
          </Column>
        </Columns>

        
      </Section>
    )
  }
}

ImageViewer.defaultProps = {
  imageRecognitions: [
    // {
    //   label: "foobar",
    //   roi: { x_offset: 500, y_offset: 100, width: 300, height: 400 }
    // }
  ]
}

export default ImageViewer
