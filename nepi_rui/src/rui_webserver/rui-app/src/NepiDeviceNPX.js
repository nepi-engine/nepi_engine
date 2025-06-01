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
import Styles from "./Styles"

import NepiDeviceNPXControls from "./NepiDeviceNPX-Controls"

import NepiDeviceInfo from "./Nepi_IF_DeviceInfo"
import ImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFSettings from "./Nepi_IF_Settings"
import NepiIFSaveData from "./Nepi_IF_SaveData"
import NavPoseDataViewer from "./Nepi_IF_NavPoseDataViewer"


import {createShortUniqueValues} from "./Utilities"

@inject("ros")
@observer

// NPX Application page
class NepiDeviceNPX extends Component {
  constructor(props) {
    super(props)


    //const namespaces = Object.keys(props.ros.npxDevices)

    this.state = {

      show_controls: true,
      show_settings: true,
      show_save_data: true,
      
      // NPX Device topic to subscribe to and update
      namespace: null,

      listener: null,

      disabled: false,
    }

    this.onImageTopicSelected = this.onImageTopicSelected.bind(this)
    this.ondeviceSelected = this.ondeviceSelected.bind(this)
    this.clearDeviceSelection = this.clearDeviceSelection.bind(this)
    this.createDeviceOptions = this.createDeviceOptions.bind(this)
    this.createImageOptions = this.createImageOptions.bind(this)


  }


  // Callback for handling ROS StatusNPX messages
  statusListener(message) {
    this.setState({
      navpose_data: message 
    })

  }

  // Function for configuring and subscribing to StatusNPX
  updateListener() {
    const namespace = this.state.currentNPXNamespace
    if (this.state.listener) {
      this.state.listener.unsubscribe()
    }
    var listener = this.props.ros.setupNPXStatusListener(
      namespace,
      this.statusListener
    )
    this.setState({ listener: listener, disabled: false })

  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.state.currentNPXNamespace
    if (prevState.namespace !== namespace){
      if (namespace != null) {
        this.updateListener()
      } else if (namespace == null){
        this.setState({ disabled: true })
      }
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to StatusNPX message
  componentWillUnmount() {
    if (this.state.listener) {
      this.state.listener.unsubscribe()
    }
  }


    // Function for creating topic options for Select input
    createDeviceOptions(topics) {

      var items = []
      items.push(<Option>{"None"}</Option>)
      //var unique_names = createShortUniqueValues(topics)
      var device_name = ""
      for (var i = 0; i < topics.length; i++) {
        device_name = topics[i].split('/idx')[0].split('/').pop()
        items.push(<Option value={topics[i]}>{device_name}</Option>)
      }
      // Check that our current selection hasn't disappeard as an available option
      const { namespace } = this.state
      if ((namespace != null) && (! topics.includes(namespace))) {
        this.clearDeviceSelection()
      }
  
      return items
    }
  
  
    clearDeviceSelection() {
      this.setState({
        namespace: null,
        namespaceText: "No sensor selected",
        imageTopic: "None",
        imageText: "None"        
      })
    }
  
    // Handler for IDX Sensor topic selection
    ondeviceSelected(event) {
      var index = event.nativeEvent.target.selectedIndex
      var text = event.nativeEvent.target[index].text
      var value = event.target.value
  
      // Handle the "None" option -- always index 0
      if (index === 0) {
        this.clearDeviceSelection()
        return
      }
      else{
        var autoSelectedImgTopic = null
        var autoSelectedImgTopicText = null
        const capabilities = this.props.ros.idxDevices[value]
        if (capabilities.has_color_image) {
          autoSelectedImgTopic = value + '/color_image'
          autoSelectedImgTopicText = 'color_image'
        }
  
        this.setState({
          namespace: value,
          namespaceText: text,
          imageTopic: autoSelectedImgTopic,
          imageText: autoSelectedImgTopicText
        })
      }
    }


  renderDeviceSelection() {
    const { npxDevices, sendTriggerMsg, saveConfigTriggered  } = this.props.ros
    const NoneOption = <Option>None</Option>
    const deviceSelected = (this.state.namespace != null)
    const namespace = this.state.namespace
    
    return (
      <React.Fragment>
        <Columns>
          <Column>
            <Section title={"Selection"}>

              <Columns>
              <Column>
              
                <Label title={"Device"}>
                  <Select
                    onChange={this.ondeviceSelected}
                    value={namespace}
                  >
                    {this.createDeviceOptions(Object.keys(npxDevices))}
                  </Select>
                </Label>
               
                <div align={"left"} textAlign={"left"} hidden={!deviceSelected}>
                  <Label title={"Image"}>
                    <Select
                      id="topicSelect_0"
                      onChange={this.onImageTopicSelected}
                      value={this.state.imageTopic_0}
                    >
                      {namespace
                        ? this.createImageOptions(namespace)
                        : NoneOption}
                    </Select>
                  </Label>
                </div>

              </Column>
              <Column>
 
              </Column>
            </Columns>

            <div align={"left"} textAlign={"left"} hidden={!deviceSelected}>

              <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                <Columns>
                  <Column>


                    <ButtonMenu>
                        <Button onClick={() => sendTriggerMsg(namespace + "/save_config")}>{"Save"}</Button>
                  </ButtonMenu>


                    </Column>
                  <Column>


                  <ButtonMenu>
                      <Button onClick={() => sendTriggerMsg( namespace + "/reset_config")}>{"Reset"}</Button>
                    </ButtonMenu>

                  </Column>
                  <Column>

                  <ButtonMenu>
                        <Button onClick={() => sendTriggerMsg( namespace + "/factory_reset_config")}>{"Factory Reset"}</Button>
                  </ButtonMenu>


                  </Column>
                </Columns>
          </div>


            </Section>
          </Column>
        </Columns>
      </React.Fragment>
    )
  }

  render() {
    const deviceSelected = (this.state.namespace != null)
    const namespace = this.state.namespace
    const navpose_data = this.state.navpose_data
    
    return (


    
      <div style={{ display: 'flex' }}>

          <div style={{ width: "65%" }}>

                    <div hidden={(!deviceSelected)}>
                      <NepiDeviceInfo
                            deviceNamespace={namespace}
                            status_topic={"/status"}
                            status_msg_type={"nepi_ros_interfaces/NPXStatus"}
                            name_update_topic={"/update_device_name"}
                            name_reset_topic={"/reset_device_name"}
                            title={"NepiDeviceNPXInfo"}
                        />

                    </div>

                    <NavPoseDataViewer
                      namespace={namespace}
                      navposeData={navpose_data}
                      title={"NepiDeviceNPXControls"}
                    />


                    <div hidden={(!deviceSelected)}>

                      <NepiIFSaveData
                          saveNamespace={namespace + ''}
                          title={"Nepi_IF_SaveData"}
                      />
                    </div>


          </div>




          <div style={{ width: '5%' }}>
                {}
          </div>



          <div style={{ width: "30%"}}>


                    {this.renderDeviceSelection()}


                    <div hidden={(!deviceSelected && this.state.show_controls)}>
                      <NepiDeviceNPXControls
                          namespace={namespace}
                          navposeData={navpose_data}
                          title={"NepiDeviceNPXControls"}
                      />
                    </div>


                    <div hidden={(!deviceSelected && this.state.show_settings)}>
                      <NepiIFSettings
                        settingsNamespace={namespace + ''}
                        title={"Nepi_IF_Settings"}
                      />
                    </div>

          </div>



    </div>



    )
  }
}

export default NepiDeviceNPX
