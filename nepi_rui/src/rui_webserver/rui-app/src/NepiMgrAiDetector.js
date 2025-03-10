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
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Input from "./Input"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import Toggle from "react-toggle"
import BooleanIndicator from "./BooleanIndicator"
import {SliderAdjustment} from "./AdjustmentWidgets"

import CameraViewer from "./CameraViewer"

import NepiIFSaveData from "./Nepi_IF_SaveData"

import {filterStrList, createShortImagesFromNamespaces,onChangeSwitchStateValue} from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}

@inject("ros")
@observer

// Component that contains the LSX controls
class AiDetectorMgr extends Component {
  detector_info = []
  constructor(props) {
    super(props)

    this.state = {

      mgrName: "ai_model_mgr",
      mgrNamespace: null,

      mgrListener: null,

      frameworks_list: [],
      active_framework: "None",

      models_list: [],
      models_frameworks: [],
      models_types: [],

      active_models_list: [],
      active_models_types: [],
      active_models_nodes: [],
      active_models_namespaces: [],

      all_namespace: null,

      mgr_connected: false,

      showSettingsControl: this.props.showSettingsControl ? this.props.showSettingsControl : false,      
      showSettings: false,

      selected_detector: "None",
      last_selected_detector: "None",
      selected_detector_namespace: "None",
      detector_info_resp: null,
      detector_status_msg: null,

      img_list_viewable: false,
      img_filter_str_list: ['detection_image','targeting_image','alert_image','tracking_image'],
      selected_img_topic: "",
      selected_img_text: "",

      needs_update: false


    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getMgrNamespace = this.getMgrNamespace.bind(this)

    this.getDetectorOptions = this.getDetectorOptions.bind(this)
    this.onDetectorSelected = this.onDetectorSelected.bind(this)

    this.updateMgrStatusListener = this.updateMgrStatusListener.bind(this)
    this.mgrStatusListener = this.mgrStatusListener.bind(this)

    this.updateDetectorStatusListener = this.updateDetectorStatusListener.bind(this)
    this.detectorStatusListener = this.detectorStatusListener.bind(this)

    this.createImageTopicsOptions = this.createImageTopicsOptions.bind(this)

    this.toggleImagesListViewable = this.toggleImagesListViewable.bind(this)
    this.onImagesTopicSelected = this.onImagesTopicSelected.bind(this)

    this.getDisplayImgOptions = this.getDisplayImgOptions.bind(this)
    this.onDisplayImgSelected = this.onDisplayImgSelected.bind(this)
    this.getSaveNamespace = this.getSaveNamespace.bind(this)

  }
  
  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    }
    return baseNamespace
  }


  getMgrNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var mgrNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      mgrNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.mgrName
    }
    return mgrNamespace
  }

  // Callback for handling ROS Status messages
  mgrStatusListener(message) {

    this.setState({
      frameworks_list: message.ai_frameworks,
      active_framework: message.active_ai_framework,

      models_list: message.ai_models,
      models_frameworks: message.ai_models_frameworks,
      models_types: message.ai_models_types,

      active_models_list: message.active_ai_models,
      active_models_types: message.active_ai_models_types,
      active_mdoels_nodes: message.active_ai_models_nodes,
      active_models_namespaces: message.active_ai_models_namespaces,

      
      all_namespace: message.all_namespace,
      mgr_connected: true
    })    

  }

  // Function for configuring and subscribing to Status
  updateMgrStatusListener() {
    const statusNamespace = this.getMgrNamespace() + '/status'
    if (this.state.mgrListener) {
      this.state.mgrListener.unsubscribe()
      this.setState({detector_status_msg: null})
    }
    var mgrListener = this.props.ros.setupStatusListener(
          statusNamespace,
          "nepi_ros_interfaces/AiModelMgrStatus",
          this.mgrStatusListener
        )
    this.setState({ mgrListener: mgrListener,
      needs_update: false})
  }


  // Callback for handling ROS Status messages
  detectorStatusListener(message) {
    const sel_detector = this.state.selected_detector
    const got_detector = message.name
    if (sel_detector === got_detector){
      this.setState({
      detector_status_msg: message,
      detector_connected: true
      })
    }

  }

  // Function for configuring and subscribing to Status
  updateDetectorStatusListener() {
    if (this.state.detectorListener) {
      this.state.detectorListener.unsubscribe()
      this.setState({detector_namespace: null, 
        detector_status_msg: null,
        detector_connected: false})
    }
    const models = this.state.active_models_list
    const active_models_namespaces = this.state.active_models_namespaces
    const selected_detector = this.state.selected_detector
    const detector_ind = models.indexOf(selected_detector)
    if (detector_ind !== -1){
      const detector_namespace = active_models_namespaces[detector_ind]
      const statusNamespace = detector_namespace + '/status'
      var detectorListener = this.props.ros.setupStatusListener(
        statusNamespace,
        "nepi_ros_interfaces/AiDetectorStatus",
        this.detectorStatusListener
      )
      this.setState({ 
        detectorListener: detectorListener,
        detector_namespace: detector_namespace
      })
    }

  }


  componentDidMount(){
    this.setState({
      selected_detector: "None",
      last_selected_detector: "None", 
      selected_detector_namespace: "None",
      detector_status_msg: null,
      detector_connected: false
    })
  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    // First update manager status
    const mgr_namespace = this.getMgrNamespace()
    const cur_namespace = this.state.mgrNamespace
    const namespace_updated = (cur_namespace !== mgr_namespace && mgr_namespace !== null)
    const needs_update = (this.state.needs_update && mgr_namespace !== null)
    if (namespace_updated || needs_update) {
      if (mgr_namespace.indexOf('null') === -1){
        this.setState({
          mgrNamespace: mgr_namespace
        })
        this.updateMgrStatusListener()
      } 
    }

    // Once manager is connected update Model status on change
    if (this.state.mgr_connected === true){
      const selected_detector = this.state.selected_detector
      const last_detector = this.state.last_selected_detector

      if (last_detector !== selected_detector) {
          this.setState({      
            detector_status_msg: null,
            last_selected_detector: selected_detector
          })  
          this.updateDetectorStatusListener()

      }
    }
     
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.mgrListener) {
      this.state.mgrListener.unsubscribe()
    }
  }





  // Function for creating image topic options.
  getDetectorOptions() {
    const active_models_list = this.state.active_models_list
    const active_models_types = this.state.active_models_types
    var items = []
    var check_type = 'detection'
    var type = 'Unknown'
    items.push(<Option value={'None'}>{'None'}</Option>)
    for (var i = 0; i < active_models_list.length; i++) {
        type = active_models_types[i]
        if (type === check_type ){
          items.push(<Option value={active_models_list[i]}>{active_models_list[i]}</Option>)
        }
    }
    return items
  }

  onDetectorSelected(event){
    const detector = event.target.value
    this.setState({selected_detector: detector})
  }

  renderAiDetector() {
    const {sendTriggerMsg} = this.props.ros
    const mgr_namespace = this.getMgrNamespace()
    const mgr_connected = this.state.mgr_connected == true

    const has_framework = this.active_framework !== "None"

    const detector_options = this.getDetectorOptions()
    const has_models = detector_options.length > 1

    const selected_detector = this.state.selected_detector
    const detector_selected = (selected_detector !== "None")

    const message = this.state.detector_status_msg
    const detector_name = message == null? "None" : message.name
    const detector_loading = (detector_name === selected_detector && selected_detector !== "None")? this.state.detector_connected === false : selected_detector !== "None"
    const detector_connected = (detector_name === selected_detector)? (this.state.detector_connected === true && detector_name === selected_detector):false

    const Spacer = ({ size }) => <div style={{ height: size, width: size }}></div>;

    if (mgr_connected === false){
      return(
        <Columns>
        <Column>

        <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {"LOADING..."}
                  </pre>
   
        </Column>
        </Columns>
      )
    }

    else if (has_framework === false){
      return(
        <Columns>
        <Column>

        <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {"NO AI FRAMEWORK FOUND.  Enable frameworks on the AI Model Manager page"}
                  </pre>
   
        </Column>
        </Columns>
      )
    }

    else if (has_models === false){
      return(
        <Columns>
        <Column>

        <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {"No AI MODELS FOUND.  Enable models on the AI Model Manager page"}
                  </pre>
   
        </Column>
        </Columns>
      )
    }

    else{
      return (


        <Section title={"AI Detection Manager"}>

          <Columns>
          <Column>

             <Label title="Select Detector">
                  <Select id="DetectorSelect" onChange={this.onDetectorSelected} 
                      value={selected_detector}
                      disabled={false}>
                    {detector_options}
                  </Select>
              </Label>
    

              <div hidden={detector_loading === false}>

                  <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {"Loading..."}
                  </pre>

              </div>

          </Column>
          </Columns>

          <div hidden={detector_connected === false}>
   
                        {this.renderDetectorSettings()}

          </div>

  
          </Section>

      )
    }

  }



  // Function for creating image topic options.
  createImageTopicsOptions() {
    const filter_str_list = this.state.img_filter_str_list
    const { imageTopics } = this.props.ros
    const img_options = filterStrList(imageTopics,filter_str_list)
    const baseNamespace = this.getBaseNamespace()
    var imageTopicShortnames = createShortImagesFromNamespaces(baseNamespace, img_options)
    var items = []
    items.push(<Option value={'None'}>{'None'}</Option>)
    items.push(<Option value={'All'}>{'All'}</Option>)
    var img_text = ""
    for (var i = 0; i < img_options.length; i++) {
       items.push(<Option value={img_options[i]}>{imageTopicShortnames[i]}</Option>)
    }
    return items
  }

  toggleImagesListViewable() {
    const set = !this.state.img_list_viewable
    this.setState({img_list_viewable: set})
  }


  onImagesTopicSelected(event){
    const {imageTopics, sendStringMsg, sendStringArrayMsg} = this.props.ros
    const detector_namespace = this.state.detector_namespace
    const add_img_namespace = detector_namespace + "/add_img_topic"
    const add_imgs_namespace = detector_namespace + "/add_img_topics"
    const remove_img_namespace = detector_namespace + "/remove_img_topic"
    const remove_imgs_namespace = detector_namespace + "/remove_img_topics"
    const filter_str_list = this.state.img_filter_str_list
    const img_options = filterStrList(imageTopics,filter_str_list)
    const img_topics = this.state.detector_status_msg.image_source_topics
    const img_topic = event.target.value
    //this.setState({selected_img_topic: img_topic})

    if (img_topic === "None"){
        sendStringArrayMsg(remove_imgs_namespace,img_options)
    }
    else if (img_topic === "All"){
        sendStringArrayMsg(add_imgs_namespace,img_options)
    }
    else if (img_topics.indexOf(img_topic) === -1){
      sendStringMsg(add_img_namespace,img_topic)
    }
    else {
      sendStringMsg(remove_img_namespace,img_topic)
    }
  }




renderDetectorSettings() {
  const { sendTriggerMsg, sendBoolMsg } = this.props.ros
  const message = this.state.detector_status_msg
  const selected_detector = this.state.selected_detector
  const sel_img = this.state.selected_img_topic
  if (message != null){
    const detector_name = message.name 
    if (selected_detector === detector_name){
      const detector_namespace = message.namespace

      const detector_enabled = message.enabled
      const detector_state = message.state


      const img_tiling = message.img_tiling
      const overlay_labels = message.overlay_labels
      const overlay_detector_name = message.overlay_detector_name
      const overlay_img_name = message.overlay_img_name

      const threshold = message.threshold
      const max_rate = message.max_rate_hz 


      const img_topics = message.image_source_topics
      const img_selected = message.image_selected
      const img_connected = message.image_connected

      const using_latest = message.using_latest_img
      const has_tiling = message.has_tiling
      const is_tiling = message.img_tiling

      const pre_time = round(message.avg_preprocess_time, 3)
      const det_time = round(message.avg_detect_time, 3)
      const post_time = round(message.avg_postprocess_time, 3)

      const img_options = this.createImageTopicsOptions()

      const img_list_viewable = this.state.img_list_viewable

      const detector_display_name = detector_name.toUpperCase()

      
  
      

      return (

      <Columns>
      <Column>



        <Columns>
        <Column>

        <Label title="Enable Detector">
              <Toggle
              checked={detector_enabled===true}
              onClick={() => sendBoolMsg(detector_namespace + "/enable",!detector_enabled)}>
              </Toggle>
        </Label>

        </Column>
        <Column>
        </Column>
        </Columns>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Columns>
        <Column>

          <Label title={"Image Selected"}>
                <BooleanIndicator value={img_selected} />
              </Label>

          </Column>
          <Column>

              <Label title={"Connected"}>
                <BooleanIndicator value={img_connected} />
              </Label>

          </Column>
          </Columns>




        <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
        {"\n Process Times (Pre,Detect,Post): " + pre_time + " , " + det_time  + " , " + post_time }
        </pre>



          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

          <Columns>
        <Column>



          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {"Select Image Streams to Connect"}
          </label>

          <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

            <div onClick={this.toggleImagesListViewable} style={{backgroundColor: Styles.vars.colors.grey0}}>
              <Select style={{width: "10px"}}/>
            </div>
            <div hidden={this.state.img_list_viewable}>
            {img_options.map((image) =>
            <div onClick={this.onImagesTopicSelected}
              style={{
                textAlign: "center",
                padding: `${Styles.vars.spacing.xs}`,
                color: Styles.vars.colors.black,
                backgroundColor: (image.props.value === sel_img) ?
                  Styles.vars.colors.green :
                  (img_topics.includes(image.props.value)) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
                cursor: "pointer",
                }}>
                <body image-topic ={image} style={{color: Styles.vars.colors.black}}>{image}</body>
            </div>
            )}
            </div>

            </Column>
          <Column>


          </Column>
          </Columns>
          

          <SliderAdjustment
                    title={"Threshold"}
                    msgType={"std_msgs/Float32"}
                    adjustment={threshold}
                    topic={detector_namespace + "/set_threshold"}
                    scaled={0.01}
                    min={0}
                    max={100}
                    disabled={false}
                    tooltip={"Sets detection confidence threshold"}
                    unit={"%"}
                  />
                  
          <SliderAdjustment
                  title={"Max Rate"}
                  msgType={"std_msgs/Float32"}
                  adjustment={max_rate}
                  topic={detector_namespace + "/set_max_rate"}
                  scaled={1.0}
                  min={1}
                  max={20}
                  disabled={false}
                  tooltip={"Sets detection max rate in hz"}
                  unit={"Hz"}
            />




            <Columns>
            <Column>

                <Label title="Overlay Labels">
                  <Toggle
                  checked={overlay_labels===true}
                  onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_overlay_labels", overlay_labels===false)}>
                  </Toggle>
                </Label>

                <Label title="Overlay Image">
                  <Toggle
                  checked={overlay_img_name===true}
                  onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_overlay_img_name", overlay_img_name===false)}>
                  </Toggle>
                  </Label>


                  <Label title="Overlay Classifier">
                  <Toggle
                  checked={overlay_detector_name===true}
                  onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_overlay_clf_name", overlay_detector_name===false)}>
                  </Toggle>
                  </Label>


              </Column>
              <Column>


                 
                <Label title="Overlay Latest Image">
                  <Toggle
                  checked={using_latest===true}
                  onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_use_latest_img", using_latest===false)}>
                  </Toggle>
                  </Label>



            <div hidden={has_tiling === false}>           
                <Label title="Enable Image Tiling">
                  <Toggle
                  checked={is_tiling===true}
                  onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_img_tiling", is_tiling===false)}>
                  </Toggle>
                  </Label>

                  </div>

            </Column>
            </Columns>



        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                      
        <Columns>
          <Column>
          
                <ButtonMenu style={{marginTop: "10px"}}>
                  <Button onClick={() => this.props.ros.sendTriggerMsg(detector_namespace + "/reset_config")}>{"Reset Config"}</Button>
                </ButtonMenu>


        </Column>
        <Column>
                  
                <ButtonMenu style={{marginTop: "10px"}}>
                  <Button onClick={() => this.props.ros.sendTriggerMsg(detector_namespace + "/save_config")}>{"Save Config"}</Button>
                </ButtonMenu>

        </Column>
        <Column>
              
              <ButtonMenu style={{marginTop: "10px"}}>
                  <Button onClick={() => this.props.ros.sendTriggerMsg(detector_namespace + "/reset_factory")}>{"Factory Reset"}</Button>
                </ButtonMenu>

          </Column>
        </Columns>

    </Column>
    </Columns>


        )
      }
    }
  }





  onDisplayImgSelected(event){
    const img_topic = event.target.value
    const detector_name = this.state.detector_name
    const img_name = detector_name + img_topic.split(detector_name)[1]
    this.setState({selected_img_topic: img_topic,
                   selected_img_text: img_name
    })
  }   



    // Function for creating image topic for a selected detector.
    getDisplayImgOptions() {
      const img_topics = this.props.ros.imageDetectionTopics
      const all_ns = this.state.all_namespace
      const detector_name = this.state.detector_name
      const detector_ns = this.state.detector_namespace
      const img_nss = this.state.detector_img_detect_namespaces
      var img_ns = ""
      var img_text = ""
      var img_topic = ""
      var items = []
      if (detector_ns){
        const detector_img = detector_ns + "/detection_image"
        for (var i = 0; i < img_topics.length; i++) {
          img_topic = img_topics[i]
          if (img_topic.indexOf(detector_ns) !== -1){
            if (img_topic === detector_img ){
              img_text = "All"
              items.push(<Option value={detector_img}>{img_text}</Option>)
            }
            else if (img_topic.indexOf('detection_image') !== -1) {
              img_text = img_topic.replace(detector_ns + '/','')
              items.push(<Option value={img_topic}>{img_text}</Option>)
            }
          }
        }
        const sel_img = this.state.selected_img_topic
        if (sel_img === ""){
          this.setState({selected_img_topic: detector_img,
                        selected_img_text: img_text
          })
        }
      }

      if (items.length === 0) {
        img_text = "None"
        items.push(<Option value={img_text}>{img_text}</Option>)
      }
      return items
    }

    // Function for creating image topic options.
    getSaveNamespace() {
      const detector_namespace = this.state.detector_namespace
      var saveNamespace = "None"
      if (detector_namespace){
        saveNamespace = detector_namespace
      }
      return saveNamespace
    }





  render() {
    const {topicNames} = this.props.ros
    const sel_img_topic = this.state.selected_img_topic
    const img_publishing = topicNames.indexOf(sel_img_topic) !== -1
    const sel_img = img_publishing? sel_img_topic : ""
    const sel_img_text = img_publishing?  this.state.selected_img_text : 'Waiting for image to publish'
    const img_options = this.getDisplayImgOptions()
    const saveNamespace = this.getSaveNamespace()


    return (



      <Columns>
      <Column equalWidth={false}>


          <Columns>
          <Column>

                  <Label title="Select Detector Image Stream">
                      <Select id="ImgSelect" onChange={this.onDisplayImgSelected} 
                      value={sel_img}
                      disabled={false}>
                        {img_options}
                    </Select>
                    </Label>
          </Column>
          <Column>

          </Column>
          </Columns>


      <CameraViewer
        imageTopic={sel_img}
        title={sel_img_text}
        hideQualitySelector={false}
      />

      <div hidden={saveNamespace === 'None'}>
        <NepiIFSaveData
              saveNamespace={saveNamespace}
              title={"Nepi_IF_SaveData"}
          />
      </div>

      </Column>
      <Column>


      {this.renderAiDetector()}
      


      </Column>
      </Columns>



      )
    }

  

}

export default AiDetectorMgr
