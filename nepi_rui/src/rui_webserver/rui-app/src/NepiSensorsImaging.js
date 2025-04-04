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
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import CameraViewer from "./CameraViewer"

import NepiSensorsImagingControls from "./NepiSensorsImagingControls"

import NepiDeviceInfo from "./NepiDeviceInfo"
import NepiIFSettings from "./Nepi_IF_Settings"
import NepiIFSaveData from "./Nepi_IF_SaveData"

import {createShortUniqueValues} from "./Utilities"

@inject("ros")
@observer

// SensorIDX Application page
class NepiSensorsImaging extends Component {
  constructor(props) {
    super(props)

    this.onImageTopicSelected = this.onImageTopicSelected.bind(this)
    this.onTopicIDXSelected = this.onTopicIDXSelected.bind(this)
    this.clearTopicIDXSelection = this.clearTopicIDXSelection.bind(this)
    this.createTopicOptions = this.createTopicOptions.bind(this)
    this.createImageOptions = this.createImageOptions.bind(this)

    //const idxSensorNamespaces = Object.keys(props.ros.idxSensors)


    
    this.state = {

      show_controls: true,
      show_settings: true,
      show_save_data: true,
      
      // IDX Sensor topic to subscribe to and update
      currentIDXNamespace: null,
      currentIDXNamespaceText: "No sensor selected"
    }
  }


  // Function for creating topic options for Select input
  createTopicOptions(topics, filter) {
    var filteredTopics = topics
    var i
    if (filter) {
      filteredTopics = []
      for (i = 0; i < topics.length; i++) {
        // includes does a substring search
        if (topics[i].includes(filter)) {
          filteredTopics.push(topics[i])
        }
      }
    }

    var items = []
    items.push(<Option>{"None"}</Option>)
    var unique_names = createShortUniqueValues(filteredTopics)
    for (i = 0; i < filteredTopics.length; i++) {
      items.push(<Option value={filteredTopics[i]}>{unique_names[i]}</Option>)
    }
    // Check that our current selection hasn't disappeard as an available option
    const { currentIDXNamespace } = this.state
    if ((currentIDXNamespace != null) && (! filteredTopics.includes(currentIDXNamespace))) {
      this.clearTopicIDXSelection()
    }

    return items
  }

  createImageOptions(idxSensorNamespace) {
    var items = []
    items.push(<Option>{"None"}</Option>)

    const image_topics = this.props.ros.imageTopics
    var sensor_img_topics = []

    for (var i = 0; i < image_topics.length; i++) {
      const topic = image_topics[i]
      if (topic.startsWith(idxSensorNamespace) === false || image_topics[i].includes("idx") === false || image_topics[i].includes("depth_map")) {
        continue
      }
      sensor_img_topics.push(topic)
    }

    const sensor_img_topics_short = createShortUniqueValues(sensor_img_topics)
    for (i = 0; i < sensor_img_topics.length; i++) {
      items.push(<Option value={sensor_img_topics[i]}>{sensor_img_topics_short[i]}</Option>)
    }
    return items    
  }

  clearTopicIDXSelection() {
    this.setState({
      currentIDXNamespace: null,
      currentIDXNamespaceText: "No sensor selected",
      imageTopic_0: "None",
      imageText_0: "None"        
    })
  }

  // Handler for IDX Sensor topic selection
  onTopicIDXSelected(event) {
    var idx = event.nativeEvent.target.selectedIndex
    var text = event.nativeEvent.target[idx].text
    var value = event.target.value

    // Handle the "None" option -- always index 0
    if (idx === 0) {
      this.clearTopicIDXSelection()
      return
    }
    else{
      var autoSelectedImgTopic = null
      var autoSelectedImgTopicText = null
      const capabilities = this.props.ros.idxSensors[value]
      if (capabilities.has_color_2d_image) {
        autoSelectedImgTopic = value.concat("/idx/color_2d_image")
        autoSelectedImgTopicText = 'color_2d_image'
      }
      else if (capabilities.has_bw_2d_image) {
        autoSelectedImgTopic = value.concat("/idx/bw_2d_image")
        autoSelectedImgTopicText = 'bw_2d_image'      
      }

      this.setState({
        currentIDXNamespace: value,
        currentIDXNamespaceText: text,
        imageTopic_0: autoSelectedImgTopic,
        imageText_0: autoSelectedImgTopicText
      })
    }
  }

  // Handler for Image topic selection
  onImageTopicSelected(event) {
    var idx = event.nativeEvent.target.selectedIndex
    var text = event.nativeEvent.target[idx].text
    var value = event.target.value

    this.setState({
      imageTopic_0: value,
      imageText_0: text === "None" ? null : text
    })
  }

  renderSensorSelection() {
    const { idxSensors, sendTriggerMsg, saveConfigTriggered  } = this.props.ros
    const NoneOption = <Option>None</Option>
    const SensorSelected = (this.state.currentIDXNamespace != null)

    return (
      <React.Fragment>
        <Columns>
          <Column>
            <Section title={"Selection"}>

              <Columns>
              <Column>
              
                <Label title={"Sensor"}>
                  <Select
                    onChange={this.onTopicIDXSelected}
                    value={this.state.currentIDXNamespace}
                  >
                    {this.createTopicOptions(Object.keys(idxSensors))}
                  </Select>
                </Label>
               
                <div align={"left"} textAlign={"left"} hidden={!SensorSelected}>
                  <Label title={"Image"}>
                    <Select
                      id="topicSelect_0"
                      onChange={this.onImageTopicSelected}
                      value={this.state.imageTopic_0}
                    >
                      {this.state.currentIDXNamespace
                        ? this.createImageOptions(this.state.currentIDXNamespace)
                        : NoneOption}
                    </Select>
                  </Label>
                </div>

              </Column>
              <Column>
                <div align={"left"} textAlign={"left"} hidden={!SensorSelected}>
                    <ButtonMenu>
                      <Button onClick={() => saveConfigTriggered(this.state.currentIDXNamespace)}>{"Save Config"}</Button>
                    </ButtonMenu>
                    <ButtonMenu>
                      <Button onClick={() => sendTriggerMsg(this.state.currentIDXNamespace + "/idx/reset_factory")}>{"Factory Reset"}</Button>
                    </ButtonMenu>
                </div>
              </Column>
            </Columns>


            </Section>
          </Column>
        </Columns>
      </React.Fragment>
    )
  }

  renderImageViewer() {
    return (
      <React.Fragment>
        <Columns>
          <Column equalWidth={false}>
            <CameraViewer
              imageTopic={this.state.imageTopic_0}
              title={this.state.imageText_0}
              hideQualitySelector={false}
            />
          </Column>
        </Columns>
      </React.Fragment>
    )
  }


  render() {
    const SensorSelected = (this.state.currentIDXNamespace != null)
    const ImageName = this.state.imageText_0
    
    return (


    
      <div style={{ display: 'flex' }}>

          <div style={{ width: "65%" }}>

                    <div hidden={(!SensorSelected)}>
                      <NepiDeviceInfo
                            deviceNamespace={this.state.currentIDXNamespace}
                            status_topic={"/idx/status"}
                            status_msg_type={"nepi_ros_interfaces/IDXStatus"}
                            name_update_topic={"/idx/update_device_name"}
                            name_reset_topic={"/idx/reset_device_name"}
                            title={"NepiSensorsImagingInfo"}
                        />

                    </div>

                    {this.renderImageViewer()}

                    <div hidden={(!SensorSelected)}>

                      <NepiIFSaveData
                          saveNamespace={this.state.currentIDXNamespace}
                          title={"Nepi_IF_SaveData"}
                      />
                    </div>


          </div>




          <div style={{ width: '5%' }}>
                {}
          </div>



          <div style={{ width: "30%"}}>


                    {this.renderSensorSelection()}


                    <div hidden={(!SensorSelected && this.state.show_controls)}>
                      <NepiSensorsImagingControls
                          idxSensorNamespace={this.state.currentIDXNamespace}
                          idxImageName = {ImageName}
                          title={"NepiSensorsImagingControls"}
                      />
                    </div>


                    <div hidden={(!SensorSelected && this.state.show_settings)}>
                      <NepiIFSettings
                        settingsNamespace={this.state.currentIDXNamespace}
                        title={"Nepi_IF_Settings"}
                      />
                    </div>

          </div>



    </div>



    )
  }
}

export default NepiSensorsImaging
