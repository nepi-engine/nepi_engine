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

import {filterStrList, createShortValuesFromNamespaces,onChangeSwitchStateValue} from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}

@inject("ros")
@observer

// Component that contains the LSX controls
class AiDetectorMgr extends Component {
  constructor(props) {
    super(props)

    this.state = {

      mgrName: "ai_detector_mgr",
      mgrNamespace: null,


      mgrListener: null,
      classifiers_list: [],
      classifier_namespace_list: [],
      active_clf: "None",
      active_clf_namespace: "None",
      mgr_connected: false,

      last_active_clf: "None",

      clfListener: null,
      clf_namespace: null,
      clf_status_msg: null,
      clf_connected: true,  
      clf_enabled: false,  
      clf_img_topics: [],
      clf_img_topic: "None",

      showSettingsControl: this.props.showSettingsControl ? this.props.showSettingsControl : false,      
      showSettings: this.props.showSettings ? this.props.showSettings : false,

      img_list_viewable: false,
      img_filter_str_list: ['detection_image','targeting_image','alert_image','tracking_image'],
      selected_img: "",

      needs_update: false


    }

    this.getMgrNamespace = this.getMgrNamespace.bind(this)

    this.getClassifierOptions = this.getClassifierOptions.bind(this)
    this.onClassifierSelected = this.onClassifierSelected.bind(this)
    this.getClfImageOptions = this.getClfImageOptions.bind(this)
    this.onClfImageTopicSelected = this.onClfImageTopicSelected.bind(this)

    this.updateMgrStatusListener = this.updateMgrStatusListener.bind(this)
    this.mgrStatusListener = this.mgrStatusListener.bind(this)

    this.updateClfStatusListener = this.updateClfStatusListener.bind(this)
    this.clfStatusListener = this.clfStatusListener.bind(this)

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
    const classifiers_list = message.ai_classifiers
    const clf_namespaces = message.ai_classifier_namespaces
    const active_clf = message.active_ai_classifier

    this.setState({
      classifiers_list: classifiers_list,
      classifier_namespace_list: clf_namespaces,
      active_clf: active_clf,
      mgr_connected: true
    })    

  }

  // Function for configuring and subscribing to Status
  updateMgrStatusListener() {
    const statusNamespace = this.getMgrNamespace() + '/status'
    if (this.state.mgrListener) {
      this.state.mgrListener.unsubscribe()
      this.setState({clf_status_msg: null})
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
  clfStatusListener(message) {
    this.setState({
     clf_status_msg: message,
     clf_enabled: message.classifier_enabled,
     clf_img_topics: message.img_topics,
     clf_img_topic: message.img_topic
    })    
    this.setState({clf_connected: true})

  }

  // Function for configuring and subscribing to Status
  updateClfStatusListener() {
    if (this.state.clfListener) {
      this.state.clfListener.unsubscribe()
      this.setState({clf_namespace: null, 
        clf_status_msg: null,
        clf_img_topics: [],
        clf_img_topic: "None",
        clf_enabled: false,
        clf_connected: false})
    }
    const clfs = this.state.classifiers_list
    const clf_namespaces = this.state.classifier_namespace_list
    const active_clf = this.state.active_clf
    const clf_ind = clfs.indexOf(active_clf)
    if (clf_ind !== -1){
      const clf_namespace = clf_namespaces[clf_ind]
      const statusNamespace = clf_namespace + '/status'
      var clfListener = this.props.ros.setupStatusListener(
        statusNamespace,
        "nepi_ros_interfaces/ImageClassifierStatus",
        this.clfStatusListener
      )
      this.setState({ 
        clfListener: clfListener,
        clf_namespace: clf_namespace
      })
    }

  }


  componentDidMount(){
    this.setState({
      active_clf: "None",
      last_active_clf: "None", 
      clf_img_topic: "None",
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

    // Once manager is connected update Classifier status on change
    if (this.state.mgr_connected === true){
      const active_clf = this.state.active_clf
      const last_clf = this.state.last_active_clf

      if (last_clf !== active_clf) {
          this.setState({      
            clf_status_msg: null,
            last_active_clf: active_clf
          })  
          this.updateClfStatusListener()

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
  getClassifierOptions() {
    const classifiersList = this.state.classifiers_list  
    const active_clf = this.state.active_clf
    var items = []
    items.push(<Option value={'None'}>{'None'}</Option>)
    for (var i = 0; i < classifiersList.length; i++) {
        items.push(<Option value={classifiersList[i]}>{classifiersList[i]}</Option>)
    }
    return items
  }

  onClassifierSelected(event){
    const {sendStringMsg} = this.props.ros
    const mgr_namespace = this.getMgrNamespace()
    const namespace = mgr_namespace + '/set_active_classifier'
    const classifier = event.target.value
    sendStringMsg(namespace,classifier)
  }


    // Function for creating image topic options.
    getClfImageOptions() {
      var items = []
      const img_topics = this.state.clf_img_topics
      var uniqueNames = createShortValuesFromNamespaces(img_topics)
      items.push(<Option value={'None'}>{'None'}</Option>)
      for (var i = 0; i < img_topics.length; i++) {
         items.push(<Option value={img_topics[i]}>{uniqueNames[i]}</Option>)
      }
      return items
    }
  
  
    onClfImageTopicSelected(event){
      const {sendStringMsg} = this.props.ros
      const clf_namespace = this.state.clf_namespace
      const clf_img_namespace = clf_namespace + "/set_img_topic"
      const img_topic = event.target.value
  
      sendStringMsg(clf_img_namespace,img_topic)
    }



  renderAIManager() {
    const {sendTriggerMsg} = this.props.ros
    const mgr_namespace = this.getMgrNamespace()
    const clf_options = this.getClassifierOptions()
    const active_clf = this.state.active_clf
    const show_loading_msg = (message === null && (active_clf !== "None"))

    const clf_enabled = this.state.clf_enabled
    const message = this.state.clf_status_msg
    const img_options = this.getClfImageOptions()
    const img_topic = this.state.clf_img_topic

    const Spacer = ({ size }) => <div style={{ height: size, width: size }}></div>;
    return (


      <Section title={"AI Detection Manager"}>


        <div style={{ display: 'flex' }}>
        <div style={{ width: '70%' }}>

             <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
              {"Active Classifier"}
             </label>
    
             <div class="break"></div> 

                  <Select id="ClfSelect" onChange={this.onClassifierSelected} 
                      value={active_clf}
                      disabled={false}>
                    {clf_options}
                  </Select>
    
                  <Spacer size="20px" />

                  <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                {"Active Image Stream"}
              </label>
    
              <div class="break"></div> 

                  <Select id="ImgSelect" onChange={this.onClfImageTopicSelected} 
                  value={img_topic}
                  disabled={false}>
                    {img_options}
                  </Select>

        </div>

        <div style={{ width: '5%' }}>
          {}
        </div>

        <div style={{ width: '25%' }}>

                <div hidden={show_loading_msg === false}>

                <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                {"Loading..."}
                </pre>

                </div>

        </div>
      </div>

      <Spacer size="20px" />


      <div hidden={this.state.showSettingsControl === false} >


      <Columns>
        <Column>


            <Label title="Show Classifier Settings">
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


          </Section>

    )
  }





  // Function for creating image topic options.
  createImageTopicsOptions() {
    const filter_str_list = this.state.img_filter_str_list
    const { imageTopics } = this.props.ros
    const img_options = filterStrList(imageTopics,filter_str_list)
    var uniqueNames = createShortValuesFromNamespaces(img_options)
    var items = []
    items.push(<Option value={'None'}>{'None'}</Option>)
    items.push(<Option value={'All'}>{'All'}</Option>)

    for (var i = 0; i < img_options.length; i++) {
       items.push(<Option value={img_options[i]}>{uniqueNames[i]}</Option>)
    }
    return items
  }


  toggleImagesListViewable() {
    const set = !this.state.img_list_viewable
    this.setState({img_list_viewable: set})
  }


  onImagesTopicSelected(event){
    const clf_namespace = this.state.clf_namespace
    const add_img_namespace = clf_namespace + "/add_img_topic"
    const remove_img_namespace = clf_namespace + "/remove_img_topic"
    const { imageTopics } = this.props.ros
    const filter_str_list = this.state.img_filter_str_list
    const img_options = filterStrList(imageTopics,filter_str_list)
    const img_topics = this.state.clf_img_topics
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


renderClfSettings() {
  const { sendTriggerMsg } = this.props.ros
  const mgr_namespace = this.getMgrNamespace()
  const message = this.state.clf_status_msg
  const clf_namespace = this.state.clf_namespace
  const active_clf = this.state.active_clf
  const sel_img = this.state.selected_img
  if (message !== null && clf_namespace !== null && active_clf !== "None"){
    const clf_name = message.classifier_name 
    const clf_enabled = message.classifier_enabled
    const clf_state = message.classifier_state
    const img_connected = message.img_connected

    if (active_clf === clf_name){

      const classifier_name = message.classifier_name 
      const classifier_enabled = message.classifier_enabled
      const classifier_state = message.classifier_state
      const classifier_classes = message.classifier_classes

      const img_topics = message.img_topics
      const img_tiling = message.img_tiling
      const overlay_labels = message.overlay_labels
      const overlay_clf_name = message.overlay_clf_name
      const overlay_img_name = message.overlay_img_name
      const img_topic = message.img_topic
      const img_connected = message.img_connected

      const threshold = message.threshold
      const max_rate = message.max_rate_hz 

      const img_options = this.createImageTopicsOptions()

      const img_list_viewable = this.state.img_list_viewable

      const clf_name = classifier_name.toUpperCase()

      return (
        <Section title={clf_name}>

  

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
                    topic={clf_namespace + "/set_threshold"}
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
                  topic={clf_namespace + "/set_max_rate"}
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
                  onClick={() => this.props.ros.sendBoolMsg(clf_namespace + "/set_overlay_labels", overlay_labels===false)}>
                  </Toggle>
                </Label>

                <Label title="Overlay Img">
                  <Toggle
                  checked={overlay_img_name===true}
                  onClick={() => this.props.ros.sendBoolMsg(clf_namespace + "/set_overlay_img_name", overlay_img_name===false)}>
                  </Toggle>
                  </Label>

                  </Column>
                  <Column>

                <Label title="Overlay CLF">
                  <Toggle
                  checked={overlay_clf_name===true}
                  onClick={() => this.props.ros.sendBoolMsg(clf_namespace + "/set_overlay_clf_name", overlay_clf_name===false)}>
                  </Toggle>
                  </Label>
  {/*                      
                <Label title="Enable Tiling">
                  <Toggle
                  checked={img_tiling===true}
                  onClick={() => this.props.ros.sendBoolMsg(clf_namespace + "/set_img_tiling", img_tiling===false)}>
                  </Toggle>
                  </Label>
  */}

            </Column>
            </Columns>

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
                    
                    <Columns>
                      <Column>
                
                      <ButtonMenu style={{marginTop: "10px"}}>
                        <Button onClick={() => sendTriggerMsg(clf_namespace + "/reset_config")}>{"Reset Config"}</Button>
                      </ButtonMenu>

            
                      </Column>
                      <Column>
                        
                      <ButtonMenu style={{marginTop: "10px"}}>
                        <Button onClick={() => sendTriggerMsg(clf_namespace + "/save_config")}>{"Save Config"}</Button>
                      </ButtonMenu>
            
                    </Column>
                    <Column>
                    
                    <ButtonMenu style={{marginTop: "10px"}}>
                        <Button onClick={() => sendTriggerMsg(clf_namespace + "/reset_factory")}>{"Factory Reset"}</Button>
                      </ButtonMenu>


            
                    </Column>
                  </Columns>

        </Section>

        )
      }
    }
  }





  render() {

    return (

      <Columns>
        <Column>
        
          {this.renderAIManager()}

          <div hidden={!this.state.showSettings}>
          {this.renderClfSettings()}
          </div>
    
        </Column>
      </Columns>
    )
  }


}

export default AiDetectorMgr
