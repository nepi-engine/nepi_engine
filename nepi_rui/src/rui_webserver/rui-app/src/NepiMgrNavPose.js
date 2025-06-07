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

import NepiDeviceInfo from "./Nepi_IF_DeviceInfo"
import NepiIFSettings from "./Nepi_IF_Settings"
import NepiIFSaveData from "./Nepi_IF_SaveData"
import NavPoseDataViewer from "./Nepi_IF_NavPoseDataViewer"


import {round, createShortUniqueValues} from "./Utilities"

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

      listener: null,

      disabled: false,

      connected: false,
      statusListener: null,
      status_msg: null,

      navposeListener: null,
      data_msg: null,

      needs_update: true,
      nav_needs_update: true
    }


    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getnamespace = this.getnamespace.bind(this)

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


  

  renderMgrControls() {
    const {sendTriggerMsg} = this.props.ros
    const namespace = this.state.namespace
    const connected = (this.state.namespace != null)
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
        const topics = comp_info.available_topics
        const msgs = comp_info.available_topic_msgs
        const fixed = comp_info.fixed
        // Show the rest if fixed === false
          const topic = comp_info.topic
          // Show the rest if topic !== 'None'
            const msg = comp_info.topic_msg
            const con = comp_info.connected
            // Show the rest if connected
            const rate = comp_info.avg_rate
            const time = round(comp_info.last_time,2)
            // Show the rest if show_transform === true
              const transform = comp_info.transform
        return (
          <Columns>
          <Column>
              
          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


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
