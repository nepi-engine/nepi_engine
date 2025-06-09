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
import Select, { Option } from "./Select"

//import EnableAdjustment from "./EnableAdjustment"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import RangeAdjustment from "./RangeAdjustment"
import {RadioButtonAdjustment, SliderAdjustment} from "./AdjustmentWidgets"
import Toggle from "react-toggle"
import Label from "./Label"
import Input from "./Input"
import { Column, Columns } from "./Columns"
import { round, onUpdateSetStateValue, onEnterSetStateFloatValue, createShortUniqueValues } from "./Utilities"

import NepiDeviceInfo from "./Nepi_IF_DeviceInfo"
import NepiIFSettings from "./Nepi_IF_Settings"
import NepiIFSaveData from "./Nepi_IF_SaveData"
import NavPoseDataViewer from "./Nepi_IF_NavPoseDataViewer"

@inject("ros")
@observer

// NPX Application page
class NavPoseMgr extends Component {
  constructor(props) {
    super(props)


    //const namespaces = Object.keys(props.ros.npxDevices)

    this.state = {


      mgrName: "navpose_mgr",
      namespace: null,
      base_namespace: null,

      show_controls: true,
      show_settings: true,
      show_save_data: true,
      
      // NPX Device topic to subscribe to and update
      namespace: null,

      message: null,

      listener: null,

      disabled: false,

      connected: false,
      statusListener: null,
      status_msg: null,

      navposeListener: null,
      data_msg: null,

      transfroms_msg: null,
      transformTX: 0,
      transformTY: 0,
      transformTZ: 0,
      transformRX: 0,
      transformRY: 0,
      transformRZ: 0,
      transformHO: 0,

      needs_update: true,
      nav_needs_update: true  
    }


    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getnamespace = this.getnamespace.bind(this)

    this.onTopicSelected = this.onTopicSelected.bind(this)
    this.createTopicOptions = this.createTopicOptions.bind(this)


    this.sendTransformUpdateMessage = this.sendTransformUpdateMessage.bind(this)
    this.sendTransformClearMessage = this.sendTransformClearMessage.bind(this)


    this.statusListener = this.statusListener.bind(this)
    this.navposeListener = this.navposeListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.updateNavposeListener = this.updateNavposeListener.bind(this)


  }

    
  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    }
    return baseNamespace
  }


  getnamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var namespace = null
    if (namespacePrefix !== null && deviceId !== null){
      namespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.mgrName
    }
    return namespace
  }





  // Callback for handling ROS StatusNPX messages
  statusListener(message) {
    this.setState({
      status_msg: message, 
      connected: true
    })

  }

  navposeListener(message) {
    
    // Transform the data to match what NavPoseDataViewer expects
    const navpose_data = {
      latitude: message.lat,
      longitude: message.long,
      altitude: message.altitude_m,
      heading: message.heading_deg,
      roll: message.roll_deg,
      pitch: message.pitch_deg,
      yaw: message.yaw_deg,
      x_m: message.x_m,
      y_m: message.y_m,
      z_m: message.z_m,
      frame_3d: message.frame_3d,
      frame_id: message.frame_id
    }
        
    this.setState({
      navpose_data: navpose_data, 
      connected: true
    })
  }

updateStatusListener() {
  const namespace = this.state.namespace

  var statusListener = this.props.ros.setupStatusListener(
    namespace + "/status",
    "nepi_sdk_interfaces/NavPoseMgrStatus",
    this.statusListener 
  )
  
  this.setState({ 
    statusListener: statusListener,
    needs_update: false 
  })
}

updateNavposeListener() {
  const namespace = this.state.base_namespace
  const navposeTopic = namespace + "/navpose"
  
  if (this.state.navposeListener) {
    this.state.navposeListener.unsubscribe()
  }
  
  var navposeListener = this.props.ros.setupStatusListener(
    navposeTopic,
    "nepi_sdk_interfaces/NavPoseData",
    this.navposeListener 
  )
  
  this.setState({ 
    navposeListener: navposeListener,
    nav_needs_update: false 
  })
}

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getMgrNamespace()
    const base_namespace = this.getBaseNamespace()
    if (prevState.namespace !== namespace){
      if (namespace != null) {
        this.setState({
          namespace: namespace,
          base_namespace: base_namespace
        })
        this.updateStatusListener()
        this.updateNavposeListener()
      } else if (namespace == null){
        this.setState({ disabled: true })
      }
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to StatusNPX message
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    if (this.state.navposeListener) {
      this.state.navposeListener.unsubscribe()
    }
  }


  // Function for creating topic options for Select input
  createTopicOptions(name) {
    const namespace = this.state.namespace
    const status_msg = this.state.status_msg

    var items = []
    items.push(<Option value={'None'}>{'None'}</Option>)
    if (status_msg != null){
  
      const comp_names = status_msg.comp_names
      const comp_infos = status_msg.comp_infos
      const index = comp_names.indexOf(name)
    
      if (index !== -1){
        const infos = comp_infos[index]
        const topics = infos.available_topics
        for (var i = 0; i < topics.length; i++) {
          items.push(<Option value={topics[i]}>{topics[i]}</Option>)
        }
      }
    }

    return items
  }


  // Handler for IDX Sensor topic selection
  onTopicSelected(event) {
    const {updateNavPoseTopic} = this.props.ros
    const name = event.target.id
    const topic = event.target.value
    const apply_tf = false
    const namespace = this.state.namespace + "/set_topic"
    updateNavPoseTopic(namespace, name, topic, apply_tf)
  }






  sendTransformUpdateMessage(){
    const {sendFrame3DTransformMsg} = this.props.ros
    const namespace = this.state.namespace + "/set_3d_transform"
    const TX = parseFloat(this.state.transformTX)
    const TY = parseFloat(this.state.transformTY)
    const TZ = parseFloat(this.state.transformTZ)
    const RX = parseFloat(this.state.transformRX)
    const RY = parseFloat(this.state.transformRY)
    const RZ = parseFloat(this.state.transformRZ)
    const HO = parseFloat(this.state.transformHO)
    const transformList = [TX,TY,TZ,RX,RY,RZ,HO]
    sendFrame3DTransformMsg(namespace,transformList)
  }


  sendTransformUZeroMessage(){
    this.setState({
      transformTX: 0,
      transformTY: 0,
      transformTZ: 0,
      transformRX: 0,
      transformRY: 0,
      transformRZ: 0,
      transformHO: 0,      
    })
    const {sendClearFrame3DTransformMsg} = this.props.ros
    const namespace = this.state.namespace + "/set_3d_transform"
    const transformList = [0,0,0,0,0,0,0]
    sendClearFrame3DTransformMsg(namespace,transformList)
  }


  sendTransformClearMessage(){
    const {sendTriggerMsg} = this.props.ros
    const namespace = this.state.namespace + "/clear_3d_transform"
    sendTriggerMsg(namespace)
  }

  

  renderMgrControls() {
    const {sendTriggerMsg, sendNavPoseDataMsg} = this.props.ros
    const namespace = this.state.namespace
    const status_msg = this.state.status_msg

    
    if (status_msg == null){
      return (
            <Columns>
            <Column>
            

            </Column>
          </Columns>
      )
    }
    else {
      const comp_names = status_msg.comp_names
      const comp_infos = status_msg.comp_infos

      for (var i = 0; i < comp_names.length; i++) {
        const comp_info = comp_infos[i]
        // Show for each
        const name = comp_info.name
        const name_text = name.toUpperCase()
        const fixed = comp_info.fixed
        const topics = comp_info.available_topics
        const msgs = comp_info.available_topic_msgs
        // Show the rest if fixed === false
          const topic = (comp_info.topic !== '') ? comp_info.topic : 'None'
          // Show the rest if topic !== 'None'
            const msg = comp_info.topic_msg
            const con = comp_info.connected
            // Show the rest if connected
              const rate = round(comp_info.avg_rate,2)
              const time = round(comp_info.last_time,2)
              const transform = comp_info.transform
        return (
          <Columns>
          <Column>
          <label style={{fontWeight: 'bold'}}>
            {name_text}
          </label>


          {/*
          <div align={"left"} textAlign={"left"} hidden={!device_selected}>
                  <Label title={"Select Source Topic"}>
                    <Select
                      id=name
                      onChange={() => this.onTopicSelected(name)}
                      value={topic}
                    >
                      {namespace
                        ? this.createTopicOptions(name)
                        : <Option value={'None'}>{'None'}</Option>}
                    </Select>
                  </Label>
                  

                  <Label title="Set Fixed Value">
                  <Toggle
                    checked={include_transform}
                    onClick={sendBoolMsg(this.props.npxNamespace + '/set_include_transform',!include_transform)}>
                  </Toggle>
                </Label>

                  <div align={"left"} textAlign={"left"} hidden={!fixed}>



                  </div>

                  <div align={"left"} textAlign={"left"} hidden={fixed}>



                  </div>

                  <div align={"left"} textAlign={"left"} hidden={fixed === True || topic === 'None' || topic === ''}>

                  <Label title={"Update Rate"}>
                        <Input
                          value={rate}
                          id="rate"
                          style={{ width: "100%" }}
                          disabled={true}
                        />
                      </Label>



                      <Columns>
                        <Column>

                        <Label title={"X (m)"}>
                          <Input
                            value={this.state.transformTX}
                            id="XTranslation"
                            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformTX")}
                            onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTX")}
                            style={{ width: "80%" }}
                          />
                        </Label>

                        <Label title={"Y (m)"}>
                          <Input
                            value={this.state.transformTY}
                            id="YTranslation"
                            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformTY")}
                            onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTY")}
                            style={{ width: "80%" }}
                          />
                        </Label>

                        <Label title={"Z (m)"}>
                          <Input
                            value={this.state.transformTZ}
                            id="ZTranslation"
                            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformTZ")}
                            onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTZ")}
                            style={{ width: "80%" }}
                          />
                        </Label>


                        <ButtonMenu>
                          <Button onClick={() => this.sendTransformUpdateMessage()}>{"Update Transform"}</Button>
                        </ButtonMenu>




                      </Column>
                      <Column>

                        <Label title={"Roll (deg)"}>
                          <Input
                            value={this.state.transformRX}
                            id="XRotation"
                            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformRX")}
                            onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRX")}
                            style={{ width: "80%" }}
                          />
                        </Label>

                        <Label title={"Pitch (deg)"}>
                          <Input
                            value={this.state.transformRY}
                            id="YRotation"
                            onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformRY")}
                            onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRY")}
                            style={{ width: "80%" }}
                          />
                        </Label>

                            <Label title={"Yaw (deg)"}>
                              <Input
                                value={this.state.transformRZ}
                                id="ZRotation"
                                onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformRZ")}
                                onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRZ")}
                                style={{ width: "80%" }}
                              />
                            </Label>


                                <ButtonMenu>
                                    <Button onClick={() => sendTriggerMsg( namespace + "/clear_3d_transform")}>{"Clear Transform"}</Button>
                                </ButtonMenu>


                          </Column>
                        </Columns>




                  </div>  





        </div>


        */}


              </Column>
            </Columns>
        )

      }    
    }
  }




  renderMgrSettings() {
    const {sendTriggerMsg} = this.props.ros
    const NoneOption = <Option>None</Option>
    const connected = (this.state.namespace != null)
    const namespace = this.state.namespace
    const status_msg = this.state.status_msg

    // If status_msg !== null, show rate_control 


    return (
      <React.Fragment>
        <Columns>
          <Column>


            <Section title={"NavPose Solution Controls"}>


            {this.renderMgrControls()}

            <div align={"left"} textAlign={"left"} hidden={!connected}>

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
    const namespace = this.state.namespace
    const navpose_data = this.state.navpose_data
    const status_msg = this.state.status_msg
    const connected = this.state.connected

    return (


    
      <div style={{ display: 'flex' }}>

          <div style={{ width: "65%" }}>

                    <NavPoseDataViewer
                      namespace={namespace}
                      navposeData={navpose_data}
                      title={"NavPose Data"}
                    />


                    <div hidden={(!connected)}>

                      <NepiIFSaveData
                        namespace={namespace}
                        title={"Nepi_IF_SaveData"}
                      />
                    </div>


          </div>




          <div style={{ width: '5%' }}>
                {}
          </div>



          <div style={{ width: "30%"}}>


                    {this.renderMgrSettings()}


                    <div hidden={(!connected && this.state.show_settings)}>
                      <NepiIFSettings
                        namespace={namespace ? namespace + '/npx' : null}
                        title={"Nepi_IF_Settings"}
                      />
                    </div>

          </div>



    </div>



    )
  }
}

export default NavPoseMgr
