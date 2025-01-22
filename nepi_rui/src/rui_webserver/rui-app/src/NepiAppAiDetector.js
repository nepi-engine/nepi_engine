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

import { Columns, Column } from "./Columns"

import AiDetectorMgr from "./NepiMgrAiDetector"

import CameraViewer from "./CameraViewer"

import NepiIFSaveData from "./Nepi_IF_SaveData"

@inject("ros")
@observer
class AiDetectorApp extends Component {
  constructor(props) {
    super(props)

    this.state = {
      appName: "ai_detector_mgr",
    }
    this.getAppNamespace = this.getAppNamespace.bind(this)
  }

  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return appNamespace
  }

  render() {
    const appNamespace = this.getAppNamespace()
    //var imageTopic = this.props.ros.getClassifierImageTopic()
    //if (imageTopic === 'None'){
    //  imageTopic = null
    //}
    //const imageText = this.props.ros.getClassifierImageText()
    const classifier_running = ((this.props.ros.reportedClassifier) && (this.props.ros.reportedClassifier.classifier_state === "Running"))?
    true : false
    const detection_img_topic = "/" + this.props.ros.namespacePrefix + "/" + this.props.ros.deviceId + '/ai_detector_mgr/detection_image'
    var displayImageTopic = 'None'
    var displayImageText = 'None'
    if (detection_img_topic.indexOf('null') === -1 && classifier_running === true){
      displayImageTopic = detection_img_topic
      displayImageText = 'Detection Image'
    }
    else {
      displayImageTopic = null
      displayImageText = 'Waiting for detection image'
    }

    return (



      <Columns>
      <Column equalWidth={false}>


      <CameraViewer
        imageTopic={displayImageTopic}
        title={displayImageText}
        hideQualitySelector={false}
      />



      </Column>
      <Column>


      <AiDetectorMgr
              title={"Nepi_Mgr_AI_Detector"}
              showSettings = {true}
          />
      
      <div hidden={appNamespace === null}>
        <NepiIFSaveData
              saveNamespace={appNamespace}
              title={"Nepi_IF_SaveData"}
          />
      </div>

      </Column>
      </Columns>



      )
    }
  
}

export default AiDetectorApp
