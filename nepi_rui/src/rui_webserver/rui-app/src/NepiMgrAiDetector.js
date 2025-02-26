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
import Select, { Option } from "./Select"
import Styles from "./Styles"
import Toggle from "react-toggle"
import {SliderAdjustment} from "./AdjustmentWidgets"

import CameraViewer from "./CameraViewer"

import NepiIFSaveData from "./Nepi_IF_SaveData"

import {filterStrList, createShortImagesFromNamespaces,onChangeSwitchStateValue} from "./Utilities"

@inject("ros")
@observer

// Component that contains the LSX controls
class AiDetectorMgr extends Component {
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

      last_selected_model: "None",

      modelListener: null,
      model_name: "None",
      model_namespace: null,
   
      model_status_msg: null,
      model_connected: false,  
      model_enabled: false,  
      model_img_source_topics: [],
      model_img_detect_namespaces: [],
      sel_img_topic: "None",

      showSettingsControl: this.props.showSettingsControl ? this.props.showSettingsControl : false,      
      showSettings: false,

      selected_model: "None",
      selected_model_namespace: "None",

      img_list_viewable: false,
      img_filter_str_list: ['detection_image','targeting_image','alert_image','tracking_image'],
      selected_img: "",
      selected_img_text: "",

      needs_update: false


    }

    this.getMgrNamespace = this.getMgrNamespace.bind(this)

    this.getModelOptions = this.getModelOptions.bind(this)
    this.onModelSelected = this.onModelSelected.bind(this)
    this.getModelImageOptions = this.getModelImageOptions.bind(this)
    this.onModelImageTopicSelected = this.onModelImageTopicSelected.bind(this)

    this.updateMgrStatusListener = this.updateMgrStatusListener.bind(this)
    this.mgrStatusListener = this.mgrStatusListener.bind(this)

    this.updateModelStatusListener = this.updateModelStatusListener.bind(this)
    this.modelStatusListener = this.modelStatusListener.bind(this)

    this.createImageTopicsOptions = this.createImageTopicsOptions.bind(this)

    this.toggleImagesListViewable = this.toggleImagesListViewable.bind(this)
    this.onImagesTopicSelected = this.onImagesTopicSelected.bind(this)

    this.getDisplayImgOptions = this.getDisplayImgOptions.bind(this)
    this.onDisplayImgSelected = this.onDisplayImgSelected.bind(this)
    this.getSaveNamespace = this.getSaveNamespace.bind(this)

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
      this.setState({model_status_msg: null})
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
  modelStatusListener(message) {
    this.setState({
     model_status_msg: message,
     model_name: message.model_name,
     model_enabled: message.enabled,
     model_namespace: message.namespace,
     model_img_source_topics: message.image_source_topics,
     model_img_detect_namespaces: message.image_detect_namespaces,
     sel_img_topic: message.selected_image_topic
    })    
    this.setState({model_connected: true})

  }

  // Function for configuring and subscribing to Status
  updateModelStatusListener() {
    if (this.state.modelListener) {
      this.state.modelListener.unsubscribe()
      this.setState({
        model_namespace: null, 
        model_status_msg: null,
        model_name: "None",
        model_enabled: false,
        model_img_source_topics: [],
        model_img_detect_namespaces: [],
        sel_img_topic: "None",

        model_connected: false})
    }
    const models = this.state.active_models_list
    const active_models_namespaces = this.state.active_models_namespaces
    const selected_model = this.state.selected_model
    const model_ind = models.indexOf(selected_model)
    if (model_ind !== -1){
      const model_namespace = active_models_namespaces[model_ind]
      const statusNamespace = model_namespace + '/status'
      var modelListener = this.props.ros.setupStatusListener(
        statusNamespace,
        "nepi_ros_interfaces/AiDetectorsStatus",
        this.modelStatusListener
      )
      this.setState({ 
        modelListener: modelListener,
        model_namespace: model_namespace
      })
    }

  }


  componentDidMount(){
    this.setState({
      selected_model: "None",
      last_selected_model: "None", 
      sel_img_topic: "None",
      needs_update: true})
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
      const selected_model = this.state.selected_model
      const last_model = this.state.last_selected_model

      if (last_model !== selected_model) {
          this.setState({      
            model_status_msg: null,
            last_selected_model: selected_model
          })  
          this.updateModelStatusListener()

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
  getModelOptions() {
    const active_models_list = this.state.active_models_list
    const active_models_types = this.state.active_models_types
    var items = []
    var type = 'Unknown'
    items.push(<Option value={'None'}>{'None'}</Option>)
    for (var i = 0; i < active_models_list.length; i++) {
        type = active_models_types[i]
        if (type === 'detection' ){
          items.push(<Option value={active_models_list[i]}>{active_models_list[i]}</Option>)
        }
    }
    return items
  }

  onModelSelected(event){
    const model = event.target.value
    this.setState({selected_model: model})
  }


    // Function for creating image topic options.
    getModelImageOptions() {
      const { namespacePrefix, deviceId} = this.props.ros
      const base_namespace = "/" + namespacePrefix + "/" + deviceId + "/"
      var items = []
      const img_topics = this.state.model_img_source_topics
      var imageTopicShortnames = createShortImagesFromNamespaces(base_namespace, img_topics)
      items.push(<Option value={'None'}>{'None'}</Option>)
      if (img_topics.length > 0) {
        items.push(<Option value={'All'}>{'Cycle All'}</Option>)
      }
      for (var i = 0; i < img_topics.length; i++) {
         items.push(<Option value={img_topics[i]}>{imageTopicShortnames[i]}</Option>)
      }
      return items
    }
  
  
    onModelImageTopicSelected(event){
      const {sendStringMsg} = this.props.ros
      const model_namespace = this.state.model_namespace
      const model_img_namespace = model_namespace + "/set_detection_img_topic"
      const img_topic = event.target.value
  
      sendStringMsg(model_img_namespace,img_topic)
    }




  renderAiDetector() {
    const {sendTriggerMsg, sendBoolMsg} = this.props.ros
    const mgr_namespace = this.getMgrNamespace()
    const mgr_connected = this.state.mgr_connected === true

    const has_framework = this.active_framework !== "None"

    const model_options = this.getModelOptions()
    const has_models = model_options.length > 1

    const selected_model = this.state.selected_model

    const model_name = this.state.model_name
    const model_loading = (model_name === selected_model && selected_model !== "None")? this.state.model_connected === false : true
     const model_connected = (model_name === selected_model)? (this.state.model_connected === true && model_name === selected_model):false

    const model_namespace = this.state.model_namespace
    const img_options = this.getModelImageOptions()
    const img_topic = this.state.sel_img_topic

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
                  <Select id="ModelSelect" onChange={this.onModelSelected} 
                      value={selected_model}
                      disabled={false}>
                    {model_options}
                  </Select>
              </Label>
    

              <div hidden={model_loading === false}>

                  <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {"Loading..."}
                  </pre>

              </div>

          </Column>
          </Columns>

          <div hidden={model_connected === false}>

              <Columns>
              <Column>

              <Label title="Enable Model">
                    <Toggle
                    checked={this.state.model_enabled===true}
                    onClick={() => sendBoolMsg(model_namespace + "/enable",!this.state.model_enabled)}>
                    </Toggle>
              </Label>

              </Column>
              <Column>

              </Column>
              </Columns>



        
              <Columns>
                  <Column>

                    <Label title="Select Active Image Stream">

                      <Select id="ImgSelect" onChange={this.onModelImageTopicSelected} 
                      value={img_topic}
                      disabled={false}>
                        {img_options}
                      </Select>
                    </Label>


                    <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

              </Column>
              </Columns>





                  <Columns>
                  <Column>



                    <Label title="Show Model Settings">
                      <Toggle
                      checked={(this.state.showSettings === true)}
                      onClick={() => onChangeSwitchStateValue.bind(this)("showSettings",this.state.showSettings)}>
                      </Toggle>
                    </Label>

                    <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                  </Column>
                  <Column>


                  </Column>
                  </Columns>
   

                    <div hidden={this.state.showSettings === false} >
      
                        {this.renderModelSettings()}

                    </div>

                      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                         
                      <Columns>
                        <Column>
                        <ButtonMenu style={{marginTop: "10px"}}>
                          <Button onClick={() => sendTriggerMsg(mgr_namespace + "/reset_config")}>{"Reset Config"}</Button>
                        </ButtonMenu>


                        </Column>
                        <Column>
                          
                        <ButtonMenu style={{marginTop: "10px"}}>
                          <Button onClick={() => sendTriggerMsg(mgr_namespace+ "/save_config")}>{"Save Config"}</Button>
                        </ButtonMenu>

                      </Column>
                      <Column>
                      
                      <ButtonMenu style={{marginTop: "10px"}}>
                          <Button onClick={() => sendTriggerMsg(mgr_namespace + "/reset_factory")}>{"Factory Reset"}</Button>
                        </ButtonMenu>

                      </Column>
                    </Columns>


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
    const { namespacePrefix, deviceId} = this.props.ros
    const baseNamespace = "/" + namespacePrefix + "/" + deviceId + "/"
    var imageTopicShortnames = createShortImagesFromNamespaces(baseNamespace, img_options)
    var items = []
    items.push(<Option value={'None'}>{'None'}</Option>)
    items.push(<Option value={'All'}>{'All'}</Option>)

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
    const model_namespace = this.state.model_namespace
    const add_img_namespace = model_namespace + "/add_img_topic"
    const add_imgs_namespace = model_namespace + "/add_img_topics"
    const remove_img_namespace = model_namespace + "/remove_img_topic"
    const remove_imgs_namespace = model_namespace + "/remove_img_topics"
    const filter_str_list = this.state.img_filter_str_list
    const img_options = filterStrList(imageTopics,filter_str_list)
    const img_topics = this.state.model_img_source_topics
    const img_topic = event.target.value
    //this.setState({selected_img: img_topic})

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


renderModelSettings() {
  const { sendTriggerMsg } = this.props.ros
  const message = this.state.model_status_msg
  const model_namespace = this.state.model_namespace
  const selected_model = this.state.selected_model
  const sel_img = this.state.selected_img
  if (message !== null && model_namespace !== null && selected_model !== "None"){
    const model_name = message.model_name 

    if (selected_model === model_name){

      const img_topics = message.image_source_topics
      const overlay_labels = message.overlay_labels
      const overlay_model_name = message.overlay_model_name
      const overlay_img_name = message.overlay_img_name

      const threshold = message.threshold
      const max_rate = message.max_rate_hz 

      const img_options = this.createImageTopicsOptions()

      return (


        <Columns>
        <Column>

  

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
                    topic={model_namespace + "/set_threshold"}
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
                  topic={model_namespace + "/set_max_rate"}
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
                  onClick={() => this.props.ros.sendBoolMsg(model_namespace + "/set_overlay_labels", overlay_labels===false)}>
                  </Toggle>
                </Label>

                <Label title="Overlay Image">
                  <Toggle
                  checked={overlay_img_name===true}
                  onClick={() => this.props.ros.sendBoolMsg(model_namespace + "/set_overlay_img_name", overlay_img_name===false)}>
                  </Toggle>
                  </Label>

                  </Column>
                  <Column>

                <Label title="Overlay Model">
                  <Toggle
                  checked={overlay_model_name===true}
                  onClick={() => this.props.ros.sendBoolMsg(model_namespace + "/set_overlay_model_name", overlay_model_name===false)}>
                  </Toggle>
                  </Label>
  {/*                      
                <Label title="Enable Tiling">
                  <Toggle
                  checked={img_tiling===true}
                  onClick={() => this.props.ros.sendBoolMsg(model_namespace + "/set_img_tiling", img_tiling===false)}>
                  </Toggle>
                  </Label>
  */}

            </Column>
            </Columns>

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
                    
                    <Columns>
                      <Column>
                
                      <ButtonMenu style={{marginTop: "10px"}}>
                        <Button onClick={() => sendTriggerMsg(model_namespace + "/reset_config")}>{"Reset Config"}</Button>
                      </ButtonMenu>

            
                      </Column>
                      <Column>
                        
                      <ButtonMenu style={{marginTop: "10px"}}>
                        <Button onClick={() => sendTriggerMsg(model_namespace + "/save_config")}>{"Save Config"}</Button>
                      </ButtonMenu>
            
                    </Column>
                    <Column>
                    
                    <ButtonMenu style={{marginTop: "10px"}}>
                        <Button onClick={() => sendTriggerMsg(model_namespace + "/reset_factory")}>{"Factory Reset"}</Button>
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
    const model_name = this.state.model_name
    const img_name = model_name + img_topic.split(model_name)[1]
    this.setState({selected_img: img_topic,
                   selected_img_text: img_name
    })
  }   



    // Function for creating image topic for a selected detector.
    getDisplayImgOptions() {
      const img_topics = this.props.ros.imageDetectionTopics
      const all_ns = this.state.all_namespace
      const model_name = this.state.model_name
      const model_ns = this.state.model_namespace
      const img_nss = this.state.model_img_detect_namespaces
      var img_ns = ""
      var img_text = ""
      var items = []
      if (model_ns){
        const model_img = model_ns + "/detection_image"
        img_text = "All"
        if (img_topics.indexOf(model_img) !== -1){
          items.push(<Option value={model_img}>{img_text}</Option>)
        }
        const sel_img = this.state.selected_img
        if (sel_img === ""){
          this.setState({selected_img: model_img,
                        selected_img_text: img_text
          })

        }

        var img_img = ""
        for (var i = 0; i < img_nss.length; i++) {
          img_ns = img_nss[i]
          img_img = img_ns + "/detection_image"
          if (img_topics.indexOf(img_img) !== -1){
            img_text = img_ns.split(model_name + '/')[1]
            items.push(<Option value={model_ns}>{img_text}</Option>)
          }
        }
      }
      else if (all_ns) {
        const all_img = all_ns + "/detection_image"
        if (img_topics.indexOf(all_img) !== -1){
          img_text = "All"
          items.push(<Option value={all_ns}>{img_text}</Option>)
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
      const model_namespace = this.state.model_namespace
      var saveNamespace = "None"
      if (model_namespace){
        saveNamespace = model_namespace
      }
      return saveNamespace
    }





  render() {
    const sel_img = this.state.selected_img
    const sel_img_text = this.state.selected_img_text
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
