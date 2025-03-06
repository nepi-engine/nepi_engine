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

import {filterStrList, createShortImagesFromNamespaces,onChangeSwitchStateValue} from "./Utilities"

@inject("ros")
@observer

// Component that contains the LSX controls
class AiSegmentorMgr extends Component {
  constructor(props) {
    super(props)

    this.state = {

      mgrName: "ai_model_mgr",
      mgrNamespace: null,


      mgrListener: null,
      models_list: [],
      model_types_list: [],
      active_models_list: [],
      active_model_namespace_list: [],
      selected_model: "None",
      selected_model_namespace: "None",
      mgr_connected: false,

      last_selected_model: "None",

      modelListener: null,
      model_namespace: null,
      model_status_msg: null,
      model_connected: false,  
      model_enabled: false,  
      model_img_topics: [],
      model_img_topic: "None",

      showSettingsControl: this.props.showSettingsControl ? this.props.showSettingsControl : false,      
      showSettings: this.props.showSettings ? this.props.showSettings : false,

      img_list_viewable: false,
      img_filter_str_list: ['detection_image','targeting_image','alert_image','tracking_image'],
      selected_img: "",

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
    const models_list = message.ai_models
    const model_types_list = message.ai_model_types
    const active_models_list = message.active_ai_models
    const active_model_namespaces = message.active_ai_model_namespaces

    this.setState({
      models_list: models_list,
      model_types_list: model_types_list,
      active_models_list: active_models_list,
      active_model_namespace_list: active_model_namespaces,
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
          "nepi_ros_interfaces/AiMgrStatus",
          this.mgrStatusListener
        )
    this.setState({ mgrListener: mgrListener,
      needs_update: false})
  }


  // Callback for handling ROS Status messages
  modelStatusListener(message) {
    this.setState({
     model_status_msg: message,
     model_enabled: message.model_enabled,
     model_img_topics: message.image_source_topics,
     model_img_topic: message.image_topic
    })    
    this.setState({model_connected: true})

  }

  // Function for configuring and subscribing to Status
  updateModelStatusListener() {
    if (this.state.modelListener) {
      this.state.modelListener.unsubscribe()
      this.setState({model_namespace: null, 
        model_status_msg: null,
        model_img_topics: [],
        model_img_topic: "None",
        model_enabled: false,
        model_connected: false})
    }
    const models = this.state.active_models_list
    const active_model_namespaces = this.state.active_model_namespace_list
    const selected_model = this.state.selected_model
    const model_ind = models.indexOf(selected_model)
    if (model_ind !== -1){
      const model_namespace = active_model_namespaces[model_ind]
      const statusNamespace = model_namespace + '/status'
      var modelListener = this.props.ros.setupStatusListener(
        statusNamespace,
        "nepi_ros_interfaces/AiDetectorStatus",
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
      model_img_topic: "None",
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
    const modelsList = this.state.models_list
    const modelTypeList = this.state.model_types_list 
    const activeModelsList = this.state.active_models_list
    var items = []
    var type = 'Unknown'
    var ind = 0
    items.push(<Option value={'None'}>{'None'}</Option>)
    for (var i = 0; i < activeModelsList.length; i++) {
        ind = modelsList.indexOf(modelsList[i])
        if (ind !== -1){
          type = modelTypeList[ind]
          if (type === 'detection' ){
            items.push(<Option value={activeModelsList[i]}>{activeModelsList[i]}</Option>)
          }
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
      const img_topics = this.state.model_img_topics
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
      const model_img_namespace = model_namespace + "/set_img_topic"
      const img_topic = event.target.value
  
      sendStringMsg(model_img_namespace,img_topic)
    }



  renderAIManager() {
    const {sendTriggerMsg, sendBoolMsg} = this.props.ros
    const mgr_namespace = this.getMgrNamespace()
    const mgr_connected = this.state.mgr_connected === true
    const modelsList = this.state.models_list
    const no_models = (modelsList.length === 0)
    const model_options = this.getModelOptions()
    const selected_model = this.state.selected_model
    const model_selected = (selected_model !== "None")
    const model_not_connected = (this.state.model_connected === false)
    const show_loading_msg = (model_selected && model_not_connected)
    const hide_model_info = (show_loading_msg === true || model_not_connected === true)

    const model_namespace = this.state.model_namespace
    const img_options = this.getModelImageOptions()
    const img_topic = this.state.model_img_topic

    const Spacer = ({ size }) => <div style={{ height: size, width: size }}></div>;
    return (


        <Columns>
        <Column>


        <div hidden={mgr_connected === true}>

        <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {"Loadding..."}
                  </pre>
        </div>

        <div hidden={no_models === false && mgr_connected === false}>

        <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {"No Detector Models Enabled.  Enable models on the AI Model Manager page"}
                  </pre>
        </div>


        <div hidden={no_models === true}>
        <Columns>
        <Column>

             <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
              {"Select Detector"}
             </label>
    
             <div class="break"></div> 

                  <Select id="ModelSelect" onChange={this.onModelSelected} 
                      value={selected_model}
                      disabled={false}>
                    {model_options}
                  </Select>
    
                  <Spacer size="20px" />


                  <div hidden={show_loading_msg === false}>

                  <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {"Loading..."}
                  </pre>

                  </div>

                  <div hidden={model_selected === false}>

                  <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {"No Model Selected"}
                  </pre>

                  </div>

                  <div hidden={hide_model_info === true}>

                          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                        {"Select Active Image Stream"}
                      </label>
        
                      <div class="break"></div> 

                      <Select id="ImgSelect" onChange={this.onModelImageTopicSelected} 
                      value={img_topic}
                      disabled={false}>
                        {img_options}
                      </Select>

                </div>

            </Column>
          </Columns>


                <Spacer size="20px" />


          <div hidden={hide_model_info === true} >


          <Columns>
            <Column>


            <Label title="Enable">
                    <Toggle
                    checked={this.state.model_enabled===true}
                    onClick={() => sendBoolMsg(model_namespace + "/enable",!this.state.model_enabled)}>
                    </Toggle>
              </Label>


              </Column>
              <Column>

              </Column>
          </Columns>

        </div>



      <Spacer size="20px" />


      <div hidden={this.state.showSettingsControl === false} >


      <Columns>
        <Column>


            <Label title="Show Model Settings">
                      <Toggle
                      checked={(this.state.showSettings === true)}
                      onClick={() => onChangeSwitchStateValue.bind(this)("showSettings",this.state.showSettings)}>
                      </Toggle>
            </Label>


          </Column>
          <Column>

          </Column>
      </Columns>

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

      </Column>
      </Columns>

    )
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
    const model_namespace = this.state.model_namespace
    const add_img_namespace = model_namespace + "/add_img_topic"
    const remove_img_namespace = model_namespace + "/remove_img_topic"
    const { imageTopics } = this.props.ros
    const filter_str_list = this.state.img_filter_str_list
    const img_options = filterStrList(imageTopics,filter_str_list)
    const img_topics = this.state.model_img_topics
    const img_topic = event.target.value
    //this.setState({selected_img: img_topic})
    const {sendStringMsg} = this.props.ros
    if (img_topic === "None"){
      for (var i = 0; i < img_topics.length; i++) {
        sendStringMsg(remove_img_namespace,img_topics[i])
      }
    }
    else if (img_topic === "All"){
      for (var i = 0; i < img_options.length; i++) {
        sendStringMsg(add_img_namespace,img_options[i])
      }
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

      const img_topics = message.img_topics
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





  render() {

    return (

      <Section title={"AI Detection Manager"}>
        
          {this.renderAIManager()}

          <div hidden={!this.state.showSettings}>
          {this.renderModelSettings()}
          </div>
    
      </Section>
    )
  }


}

export default AiSegmentorMgr
