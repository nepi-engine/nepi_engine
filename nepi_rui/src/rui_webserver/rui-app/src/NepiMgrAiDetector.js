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
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Input from "./Input"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import {SliderAdjustment} from "./AdjustmentWidgets"
import {ColoredTextIndicator, indicator_colors} from "./ColoredIndicator"
import Styles from "./Styles"
import Toggle from "react-toggle"


import { filterStrList, createShortValuesFromNamespaces, onChangeSwitchStateValue } from "./Utilities"
import { onUpdateSetStateValue, onEnterSendFloatValue} from "./Utilities"


@inject("ros")
@observer
class AiDetectorMgr extends Component {
  constructor(props) {

    super(props)
    //const classifier_running = ((this.props.ros.reportedClassifier) && (this.props.ros.reportedClassifier.classifier_state === "Running"))? true : false

    this.state = {
      appName: "ai_detector_mgr",
      filter_str_list: ['zed_node','app_ai','ai_detector_mgr'],
      last_reportedClassifier: null,
      selected_img_topic: (this.props.ros.reportedClassifier !== null && this.props.ros.reportedClassifier.selected_img_topic !== null) ? 
        this.props.ros.reportedClassifier.selected_img_topic : 'None',
      // Only set currentDisplayImgTopic when classifier is running -- this state transition is required for the CameraViewer to work properly
      selectedClassifier: (this.props.ros.reportedClassifier !== null && this.props.ros.reportedClassifier.selected_classifier !== null) ? 
        this.props.ros.reportedClassifier.selected_classifier : "None",
      detectionThreshold: (this.props.ros.reportedClassifier !== null && this.props.ros.reportedClassifier.detection_threshold !== null) ?
        this.props.ros.reportedClassifier.detection_threshold : 0.3,
      max_rate_hz: (this.props.ros.reportedClassifier !== null && this.props.ros.reportedClassifier.max_rate_hz !== null) ?
        this.props.ros.reportedClassifier.max_rate_hz : 1.0,

      showSettings: this.props.showSettings ? this.props.showSettings : false

    } 

    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.onImageTopicSelected = this.onImageTopicSelected.bind(this)
    this.onClassifierSelected = this.onClassifierSelected.bind(this)
    this.checkForClassifierRunning = this.checkForClassifierRunning.bind(this)
    this.onApplyButtonPressed = this.onApplyButtonPressed.bind(this)
    this.onStopButtonPressed = this.onStopButtonPressed.bind(this)
   
    this.checkForClassifierRunning()
  }

  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }

    return appNamespace
  }

  // Function for creating image topic options.
  createImageTopicsOptions() {
    const {imageFilterDetection} = this.props.ros
    const filter_str_list = this.state.filter_str_list
    var items = []
    items.push(<Option>{"None"}</Option>)
    const { imageTopics } = this.props.ros
    const imageTopicsFiltered = filterStrList(imageTopics,filter_str_list)
    var uniqueNames = createShortValuesFromNamespaces(imageTopicsFiltered)
    for (var i = 0; i < imageTopicsFiltered.length; i++) {
      // Run the filter
      if (imageFilterDetection && !(imageFilterDetection.test(imageTopicsFiltered[i]))) {
        continue
      }
        if ((this.props.ros.reportedClassifier !== null) && (imageTopicsFiltered[i] === this.props.ros.reportedClassifier.selected_img_topic)) {
          items.push(<Option selected="selected" value={imageTopicsFiltered[i]}>{uniqueNames[i]}</Option>)
        }
        else {
          items.push(<Option value={imageTopicsFiltered[i]}>{uniqueNames[i]}</Option>)
        }
    }
    return items
  }

  // Function for creating image classifier options.
  createImageClassifierOptions() {
    var items = []
    items.push(<Option>{"None"}</Option>)
    const { classifierLists } = this.props.ros
    if (classifierLists){
      const classifiers = classifierLists.models
      for (var i = 0; i < classifiers.length; i++) {
        if((this.props.ros.reportedClassifier !== null) && (classifiers[i] === this.props.ros.reportedClassifier.selected_classifier)) {
          items.push(<Option selected="selected" value={classifiers[i]}>{classifiers[i]}</Option>)
        }
        else {
          items.push(<Option value={classifiers[i]}>{classifiers[i]}</Option>)
        }
      }
    }
    return items
  }

  // Handler for when the image topic selection changes
  onImageTopicSelected(event) {
    var value = event.target.value
    this.setState({
      selected_img_topic: value,
    })
  }

  onClassifierSelected(event) {
    var value = event.target.value === "None" ? null : event.target.value

    this.setState({
      selectedClassifier: value
    })
  }


  async checkForClassifierRunning() {
    var displayImageTopic = 'None'
    var displayImageText = 'None'
    const {
      reportedClassifier,
    } = this.props.ros
    if ((reportedClassifier === null) || (reportedClassifier.classifier_state !== "Running")) {
      if (this.state.currentDisplayImgTopic !== this.state.selected_img_topic) {
        displayImageTopic = this.state.selected_img_topic
        displayImageText = this.state.selected_img_topic
      }
    }
    else {
        const detection_img_topic = "/" + this.props.ros.namespacePrefix + "/" + this.props.ros.deviceId + '/ai_detector_mgr/detection_image'
        displayImageTopic = detection_img_topic
        displayImageText = 'Detection Image'
    }
    this.props.ros.setClassifierImageInfo(displayImageTopic,displayImageText)

    // Run this method periodically forever
    setTimeout(this.checkForClassifierRunning, 250)
  }

   async onApplyButtonPressed() {
    const { startClassifier } = this.props.ros
    const classifier = this.state.selectedClassifier
    const selected_img_topic = this.state.selected_img_topic
    const threshold =  (this.props.ros.reportedClassifier !== null && this.props.ros.reportedClassifier.detection_threshold !== null) ? 
    this.props.ros.reportedClassifier.detection_threshold : 0

    if (selected_img_topic !== null && selected_img_topic !== 'None' && classifier !== null && classifier !== 'None'){
      startClassifier(selected_img_topic, classifier, threshold)
    }
  }

  onStopButtonPressed() {
    const {
      stopClassifier
    } = this.props.ros

    stopClassifier()
  }




  render() {
    const {reportedClassifier, sendTriggerMsg  } = this.props.ros
    const thresholdVal = reportedClassifier? reportedClassifier.detection_threshold : 0.3
    const maxRateVal = reportedClassifier? reportedClassifier.max_rate_hz : 1
    var status_text = reportedClassifier? reportedClassifier.classifier_state : "Unknown"
    
    if (reportedClassifier !== null && reportedClassifier !== this.state.last_reportedClassifier && reportedClassifier.selected_classifier !== "None"){
      if (reportedClassifier.classifier_state === "Running") {
        this.setState({
          selectedClassifier: reportedClassifier.selected_classifier,
          detectionThreshold: reportedClassifier.detection_threshold,
          last_reportedClassifier: reportedClassifier
        })
      }
    }
    
    var status_color = indicator_colors.grey
    if (status_text === "Stopped") {
      status_color = indicator_colors.red
    }
    else if (status_text === "Loading") {
      status_color = indicator_colors.orange
    }
    else if (status_text === "Running") {
      status_color = indicator_colors.green
    }   

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

      
}

export default AiDetectorMgr
