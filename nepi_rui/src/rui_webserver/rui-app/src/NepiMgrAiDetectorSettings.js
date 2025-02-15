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

import {onChangeSwitchStateValue} from "./Utilities"

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
      active_classifier: "None",
      mgr_connected: false,

      mgrClfListener: null,

      mgr_classifier_name: "None",
      mgr_classifier_enabled: false,
      mgr_classifier_state: "Stopped",
      mgr_classifier_classes: [],
      mgr_img_topic: "None",
      mgr_img_connected: false,
      mgr_img_width: 0,
      mgr_img_height: 0,
      mgr_overlay_img_name: false,
      mgr_has_depth_map: false,
      mgr_depth_map_topic: false,
      mgr_has_pointcloud: false,
      mgr_pointcloud_topic: false,
      mgr_threshold: 0,
      mgr_max_rate_hz: 0,

      mgr_clf_connected: false,

      viewable_active_list: false,


      clfListener: null,

      classifier_name: "None",
      classifier_enabled: false,
      classifier_state: "Stopped",
      classifier_classes: [],
      img_topic: "None",
      img_connected: false,
      img_width: 0,
      img_height: 0,
      overlay_img_name: false,
      has_depth_map: false,
      depth_map_topic: false,
      has_pointcloud: false,
      pointcloud_topic: false,
      threshold: 0,
      max_rate_hz: 0,

      clf_connected: true,    

      showSettings: this.props.showSettings ? this.props.showSettings : true,
      viewable_clf_list: false,     
      clf_classifier: "None",
      clf_namespace: "None", 

      needs_update: true

    }

    this.getMgrNamespace = this.getMgrNamespace.bind(this)

    this.toggleActiveClassifiersList = this.toggleActiveClassifiersList.bind(this)
    this.getClassifierOptions = this.getClassifierOptions.bind(this)
    this.onToggleActiveClassifierList = this.onToggleActiveClassifierList.bind(this)

    this.updateMgrStatusListener = this.updateMgrStatusListener.bind(this)
    this.mgrStatusListener = this.mgrStatusListener.bind(this)

    this.updateMgrClfStatusListener = this.updateMgrClfStatusListener.bind(this)
    this.mgrClfStatusListener = this.mgrClfStatusListener.bind(this)

    this.toggleSetClassifierList = this.toggleSetClassifierList.bind(this)
    this.onToggleSetClassifierList = this.onToggleSetClassifierList.bind(this)

    this.updateClfStatusListener = this.updateClfStatusListener.bind(this)
    this.ClfStatusListener = this.ClfStatusListener.bind(this)
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
      classifiers_list: message.ai_classifiers,
      classifier_namespace_list: message.ai_classifier_namespaces,
      active_classifier: message.active_ai_classifier,
      mgr_connected: true
    })    
  }

  // Function for configuring and subscribing to Status
  updateMgrStatusListener() {
    const statusNamespace = this.getMgrNamespace() + '/mgr_status'
    if (this.state.mgrListener) {
      this.state.mgrListener.unsubscribe()
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
  mgrClfStatusListener(message) {
    this.setState({
      mgr_classifier_name: message.classifier_name ,
      mgr_classifier_enabled: message.classifier_enabled ,
      mgr_classifier_state: message.classifier_state ,
      mgr_classifier_classes: message.classifier_classes ,

      mgr_img_topic: message.img_topic ,
      mgr_img_connected: message.img_connected ,
      mgr_img_width: message.img_width ,
      mgr_img_height: message.img_height ,
      mgr_overlay_img_name: message.overlay_img_name ,

      mgr_has_depth_map: message.has_depth_map ,
      mgr_depth_map_topic: message.depth_map_topic ,
      mgr_has_pointcloud: message.has_pointcloud ,
      mgr_pointcloud_topic: message.pointcloud_topic ,

      mgr_threshold: message.threshold ,
      mgr_max_rate_hz: message.max_rate_hz 
    })    
    const classifier = this.active_classifier
    if (classifier === message.classifier_name){
      this.setState({mgr_clf_connected: true})
    }
  }

  // Function for configuring and subscribing to Status
  updateMgrClfStatusListener() {
    const mgr_namespace = this.state.getMgrNamespace()
    const statusNamespace = mgr_namespace + '/clf_status'
    if (this.state.mgrClfListener) {
      this.state.mgrClfListener.unsubscribe()
    }
    if (mgr_namespace !== 'None'){
      var mgrClfListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_ros_interfaces/ImageClassifierStatus",
            this.mgrClfStatusListener
          )
      this.setState({ mgrClfListener: mgrClfListener,
        needs_update: false})

    }
  }





  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    // First update manager status
    const namespace = this.getMgrNamespace()
    const namespace_updated = (prevState.mgrNamespace !== namespace && namespace !== null)
    const needs_update = (this.state.needs_update && namespace !== null)
    if (namespace_updated || needs_update) {
      if (namespace.indexOf('null') === -1){
        this.setState({
          mgrNamespace: namespace
        })
        this.updateMgrStatusListener()
      } 
    }

    const mgr_connected = this.state.mgr_connected
    const active_classifier = this.state.active_classifier
    if (active_classifier === "None"){
      this.setState({      
        mgr_classifier_name: "None",
        mgr_classifier_enabled: false,
        mgr_classifier_state: "Stopped",
        mgr_classifier_classes: [],
        mgr_img_topic: "None",
        mgr_img_connected: false,
        mgr_img_width: 0,
        mgr_img_height: 0,
        mgr_overlay_img_name: false,
        mgr_has_depth_map: false,
        mgr_depth_map_topic: false,
        mgr_has_pointcloud: false,
        mgr_pointcloud_topic: false,
        mgr_threshold: 0,
        mgr_max_rate_hz: 0,
        mgr_clf_connected: false
      })  
    }
    else if (mgr_connected === true){


    }
    


    // Once manager is connected update Classifier status on change
    if (mgr_connected === true){
      const clf_namespace = this.state.clf_namespace
      if (prevState.clf_namespace !== clf_namespace) {
          this.updateclfStatusListener()
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
    var items = []
    items.push(<Option>{"NONE"}</Option>) 
    items.push(<Option value={'NONE'}>{'NONE'}</Option>)
    if (classifiersList.length > 0){
      for (var i = 0; i < classifiersList.length; i++) {
          items.push(<Option value={classifiersList[i]}>{classifiersList[i]}</Option>)
     }
    }
    else{
      //items.push(<Option value={'TEST1'}>{'TEST1'}</Option>)
      //items.push(<Option value={'TEST2'}>{'TEST2'}</Option>)
    }
    return items
  }


  toggleActiveClassifiersList() {
    const set = !this.state.viewable_active_list
    this.setState({viewable_active_list: set})
  }

  onToggleActiveClassifierList(event){
    const {sendStringMsg} = this.props.ros
    const namespace = this.getMgrNamespace() + '/set_active_classifier'
    const classifier = event.target.value
    sendStringMsg(namespace,classifier)
  }



  toggleSetClassifiersList() {
    const set = !this.state.viewable_settings_list
    this.setState({viewable_settings_list: set})
  }

  onToggleSetClassifierList(event){
    const classifier_list = this.state.classifier_list
    const namespace_list = this.state.classifier_namespace_list
    const mgrNamespace = this.getMgrNamespace()
    const classifier = event.target.value
    const ind = namespace_list.indexOf(classifier)
    if (ind !== -1){
      this.setState({clf_namespace: namespace_list[ind]})
    }
  }




  
  renderAiActive() {
    const {sendTriggerMsg} = this.props.ros
    const mgrNamespace = this.getMgrNamespace()
    const active_classifier = this.state.active_classifier
    const selectable_classifiers = this.state.selectable_classifiers
    const classifier_options = this.getClassifierOptions()
    const classifiers_list = this.state.classifiers_active_list

    return (


      <Section title={"AI Detector"}>


      <Columns equalWidth={true}>
      <Column>

      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Select AI Framework"}
         </label>

          <div onClick={this.toggleActiveClassifiersList} style={{backgroundColor: Styles.vars.colors.grey0}}>
            <Select style={{width: "10px"}}/>
          </div>
          <div hidden={!selectable_classifiers}>
          {classifier_options.map((classifier) =>
          <div onClick={this.onToggleActiveClassifierList}
            style={{
              textAlign: "center",
              padding: `${Styles.vars.spacing.xs}`,
              color: Styles.vars.colors.black,
              backgroundColor: (classifier.props.value === active_classifier) ?
                Styles.vars.colors.green :
                (classifiers_list.includes(classifier.props.value)) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
              cursor: "pointer",
              }}>
              <body classifier-topic ={classifier} style={{color: Styles.vars.colors.black}}>{classifier}</body>
          </div>
          )}
          </div>


        </Column>
        <Column>

        </Column>
        </Columns> 



        <Columns>
        <Column>

              <Label title="Show Settings">
              <Toggle
              checked={(this.state.showSettings === true)}
              onClick={() => onChangeSwitchStateValue.bind(this)("showSettings",this.state.showSettings)}>
              </Toggle>
               </Label>

               </Column>
                <Column>

                </Column>
              </Columns>

            <div hidden={!this.state.showSettings}>

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
            
            </div>



        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
                         
                         <Columns>
                         <Column>
                 
         
                         <ButtonMenu style={{marginTop: "10px"}}>
                         <Button onClick={() => sendTriggerMsg(mgrNamespace + "/reset_factory")}>{"Reset Settings"}</Button>
                       </ButtonMenu>
                     
         
                       </Column>
                       <Column>
                         
                       <ButtonMenu style={{marginTop: "10px"}}>
                         <Button onClick={() => sendTriggerMsg(mgrNamespace + "/save_config")}>{"Save Config"}</Button>
                       </ButtonMenu>
         
                     </Column>
                     <Column>
                     
                       <ButtonMenu style={{marginTop: "10px"}}>
                         <Button onClick={() => sendTriggerMsg(mgrNamespace + "/reset_config")}>{"Reset Config"}</Button>
                       </ButtonMenu>
         
                     </Column>
                   </Columns>


          </Section>

    )
  }

  renderAiSettings() {
    const {sendTriggerMsg} = this.props.ros
    const mgrNamespace = this.getMgrNamespace()
    const active_classifier = this.state.active_classifier
    const selectable_classifiers = this.state.selectable_classifiers
    const classifier_options = this.getClassifierOptions()
    const classifiers_list = this.state.classifiers_active_list

    return (


      <Section title={"Classifier Settings"}>


              <Label title="Show Settings">
              <Toggle
              checked={(this.state.showSettings === true)}
              onClick={() => onChangeSwitchStateValue.bind(this)("showSettings",this.state.showSettings)}>
              </Toggle>
               </Label>

               </Column>
                <Column>

                </Column>
              </Columns>

            <div hidden={!this.state.showSettings}>

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
            
            </div>


          </Section>

    )
  }


renderAiControls(classifier, state, threshold, max_rate, overlay_source_name, namespace) {


  const appNamespace = this.getAppNamespace()
  return (

        <Section title={"AI Detector"}>


            <Columns>
            <Column>



            <div hidden={(status_text !== "Stopped")} >

            <ButtonMenu style={{marginTop: "10px"}}>
                <Button onClick={this.onApplyButtonPressed}  >{"Start"} </Button>
              </ButtonMenu>   

            </div>

            <div hidden={(status_text === "Stopped")} >

              <ButtonMenu style={{marginTop: "10px"}}>
                <Button onClick={this.onStopButtonPressed}>{"Stop"}</Button>
              </ButtonMenu>  

              </div>


              </Column>
              <Column>

              <ColoredTextIndicator indicator_color={status_color} text={status_text} style={{width:"100%", fontWeight:"bold"}}/>
              {(status_text === "Loading")?
                <progress value={reportedClassifier? reportedClassifier.loading_progress : 0.0} style={{width: '100%'}}/>
                : null
              }

              </Column>
              <Column>

              </Column>
            </Columns>




              <Columns>
            <Column>

              <Label title={"Image Topic"}>
            <Select id="ImgSelect" onChange={this.onImageTopicSelected} disabled={status_text !== "Stopped"}>
              {this.createImageTopicsOptions()}
            </Select>
          </Label>



              <Label title={"Image Classifier"}>
            <Select id="ClassifierSelect" onChange={this.onClassifierSelected} disabled={status_text !== "Stopped"}>
              {this.createImageClassifierOptions()}
            </Select>
          </Label>

              </Column>
            </Columns>



            <Columns>
            <Column>

            <Label title="Show Settings">
            <Toggle
            checked={(this.state.showSettings === true)}
            onClick={() => onChangeSwitchStateValue.bind(this)("showSettings",this.state.showSettings)}>
            </Toggle>
             </Label>

             </Column>
              <Column>

              </Column>
            </Columns>

          <div hidden={!this.state.showSettings}>

          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

 
          <SliderAdjustment
                    title={"Detection Threshold"}
                    msgType={"std_msgs/Float32"}
                    adjustment={thresholdVal}
                    topic={"ai_detector_mgr/set_threshold"}
                    scaled={0.01}
                    min={0}
                    max={100}
                    disabled={this.state.disabled}
                    tooltip={"Sets detection confidence threshold"}
                    unit={"%"}
                  />
                  


          <Label title={"Max Rate Hz"}>    
          </Label>    

          <Columns>
          <Column>


              <Input id="max_rate_hz" 
                value={this.state.max_rate_hz} 
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"max_rate_hz")} 
                onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/set_max_rate")} 
              />

            </Column>
            <Column>

              <Input id="max_rate_hz" 
                disabled = {true}
                value={maxRateVal} 
              />

        </Column>
        </Columns>
      
        </div>

              <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
                       
              <Columns>
              <Column>
      

              <ButtonMenu style={{marginTop: "10px"}}>
              <Button onClick={() => sendTriggerMsg(appNamespace + "/reset_app")}>{"Reset Settings"}</Button>
            </ButtonMenu>
          

            </Column>
            <Column>
              
            <ButtonMenu style={{marginTop: "10px"}}>
              <Button onClick={() => sendTriggerMsg(appNamespace + "/save_config")}>{"Save Config"}</Button>
            </ButtonMenu>

          </Column>
          <Column>
          
            <ButtonMenu style={{marginTop: "10px"}}>
              <Button onClick={() => sendTriggerMsg(appNamespace + "/reset_config")}>{"Reset Config"}</Button>
            </ButtonMenu>

          </Column>
        </Columns>




    </Section>

      )
    }




  render() {
    const mgrNamespace = this.getMgrNamespace()

    //const lsxImageViewerElement = document.getElementById("lsxImageViewer")

    //const lsx_caps = lsxUnits[lsxNamespace]
    return (
      <React.Fragment>

        <Columns>
          <Column equalWidth = {false} >

                  <CameraViewer
                    id="ClassifierViewer"
                    imageTopic={mgrNamespace + '/detection_image'}
                    title={"Detection Image"}
                    hideQualitySelector={false}
                  />

          </Column>
          <Column>
          
          {this.renderAIActive()}
          {this.renderAISettings()}

          </Column>
        </Columns>

      </React.Fragment>
    )
  }
}




export default AiDetectorMgr
