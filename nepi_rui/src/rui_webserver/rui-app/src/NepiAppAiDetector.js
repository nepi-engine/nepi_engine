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
    this.getMgrNamespace = this.getMgrNamespace.bind(this)
  }

  getMgrNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var mgrNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      mgrNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return mgrNamespace
  }

  render() {
    const mgrNamespace = this.getMgrNamespace()
    const displayImageTopic = mgrNamespace + '/detection_image'
    return (



      <Columns>
      <Column equalWidth={false}>


      <CameraViewer
        imageTopic={displayImageTopic}
        title={'Detection Image'}
        hideQualitySelector={false}
      />

      <div hidden={mgrNamespace === null}>
        <NepiIFSaveData
              saveNamespace={mgrNamespace}
              title={"Nepi_IF_SaveData"}
          />
      </div>

      </Column>
      <Column>


      <AiDetectorMgr
              title={"Nepi_Mgr_AI_Detector"}
              showSettingsControl = {false}
              showSettings = {true}

          />
      


      </Column>
      </Columns>



      )
    }
  
}

export default AiDetectorApp
