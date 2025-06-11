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

      message: null,

      listener: null,

      disabled: false,

      connected: false,
      statusListener: null,
      status_msg: null,

      navposeListener: null,
      navpose_msg: null,

      fixedDict: {
        location: {
          fixed: false,
          npData: this.props.ros.blankNavPoseData
        },
        heading:  {
          fixed: false,
          npData: this.props.ros.blankNavPoseData
        },
        orientation:  {
          fixed: false,
          npData: this.props.ros.blankNavPoseData
        },
        position:  {
          fixed: false,
          npData: this.props.ros.blankNavPoseData
        },
        altitude:  {
          fixed: false,
          npData: this.props.ros.blankNavPoseData
        },
        depth:  {
          fixed: false,
          npData: this.props.ros.blankNavPoseData
        }
      },




      showTransformsDict: {
        location: false,
        heading: false,
        orientation: false,
        position: false,
        altitude: false,
        depth: false
      },


      transformsDict: {
        location: { 
          transfroms_msg: null,
          transformTX: 0,
          transformTY: 0,
          transformTZ: 0,
          transformRX: 0,
          transformRY: 0,
          transformRZ: 0,
          transformHO: 0
          },
        heading: { 
          transfroms_msg: null,
          transformTX: 0,
          transformTY: 0,
          transformTZ: 0,
          transformRX: 0,
          transformRY: 0,
          transformRZ: 0,
          transformHO: 0
          },
        orientation: { 
          transfroms_msg: null,
          transformTX: 0,
          transformTY: 0,
          transformTZ: 0,
          transformRX: 0,
          transformRY: 0,
          transformRZ: 0,
          transformHO: 0
          },
        position: { 
          transfroms_msg: null,
          transformTX: 0,
          transformTY: 0,
          transformTZ: 0,
          transformRX: 0,
          transformRY: 0,
          transformRZ: 0,
          transformHO: 0
          },
        altitude: { 
          transfroms_msg: null,
          transformTX: 0,
          transformTY: 0,
          transformTZ: 0,
          transformRX: 0,
          transformRY: 0,
          transformRZ: 0,
          transformHO: 0
          },
        depth: { 
          transfroms_msg: null,
          transformTX: 0,
          transformTY: 0,
          transformTZ: 0,
          transformRX: 0,
          transformRY: 0,
          transformRZ: 0,
          transformHO: 0
          }
      },
     

      needs_update: true,
      nav_needs_update: true  
    }


    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getnamespace = this.getnamespace.bind(this)

    this.renderMgrFixedControls = this.renderMgrFixedControls.bind(this)
    this.renderMgrTopicControls = this.renderMgrTopicControls.bind(this)

    this.getShowTransform = this.getShowTransform.bind(this)
    this.setShowTransform = this.setShowTransform.bind(this)
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
    const last_status_msg = this.state.status_msg
    this.setState({
      status_msg: message, 
      connected: true
    })

    var has_changed = false
    const comp_infos = message.comp_infos
    const last_comp_infos = (last_status_msg != null) ?
                         last_status_msg.comp_infos : message.comp_infos
    for (var i = 0; i < comp_infos.length; i++) {
      const comp_info = comp_infos[i]
      const name = comp_info.name
      const last_comp_info = last_comp_infos[i]

      const fixed = comp_info.fixed
      const last_fixed = last_comp_info.fixed
      has_changed = (last_status_msg == null) ? true :
                            (last_fixed !== fixed)
      if (has_changed === true){
        this.fixedDict[name]['fixed'] = fixed
      }
      
      const transform_msg = comp_info.transform
      const last_transform_msg = last_comp_info.transform
      has_changed = (last_status_msg == null) ? true :
                           (last_transform_msg !== transform_msg)
      if (has_changed === true){
        this.state.transformsDict[name]['transform_msg'] = transform_msg,
        this.state.transformsDict[name]['transformTX'] = transform_msg.translate_vector.x,
        this.state.transformsDict[name]['transformTY'] = transform_msg.translate_vector.y,
        this.state.transformsDict[name]['transformTZ'] = transform_msg.translate_vector.z,
        this.state.transformsDict[name]['transformRX'] = transform_msg.rotate_vector.x,
        this.state.transformsDict[name]['transformRY'] = transform_msg.rotate_vector.y,
        this.state.transformsDict[name]['transformRZ'] = transform_msg.rotate_vector.z,
        this.state.transformsDict[name]['transformHO'] = transform_msg.heading_offset
      }
    }

  }

  navposeListener(message) {
    // Transform the data to match what NavPoseDataViewer expects
    const last_navpose_msg = this.state.navpose_msg
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
      navpose_msg: message,
      navpose_data: navpose_data, 
      connected: true
    })

    var name = ''
    var fixed = false
    var changed = false
    var navpose = null
    const navpose_msg = message


    name = 'location'
    fixed = this.state.fixedDict[name]['fixed']
    navpose = this.state.fixedDict[name]['npData']
    if (fixed === true){
      changed = last_navpose_msg == null ? true:
                ((navpose_msg.latitude !== last_navpose_msg.latitude) ||
                (navpose_msg.longitude !== last_navpose_msg.longitude))
      if (changed === true){
        this.state.fixedDict[name]['npData']['latitude'] = navpose_msg.latitude
        this.state.fixedDict[name]['npData']['latitude'] = navpose_msg.latitude
      }
    }

    name = 'heading'
    fixed = this.state.fixedDict[name]['fixed']
    navpose = this.state.fixedDict[name]['npData']
    if (fixed === true){
      changed = last_navpose_msg == null ? true:
                ((navpose_msg.heading_deg !== last_navpose_msg.heading_deg))
      if (changed === true){
        this.state.fixedDict[name]['npData']['heading_deg'] = navpose_msg.heading_deg
      }
    }

    name = 'orientation'
    fixed = this.state.fixedDict[name]['fixed']
    navpose = this.state.fixedDict[name]['npData']
    if (fixed === true){
      changed = last_navpose_msg == null ? true:
                ((navpose_msg.roll_deg !== last_navpose_msg.roll_deg) ||
                (navpose_msg.pitch_deg !== last_navpose_msg.pitch_deg)||
                (navpose_msg.yaw_deg !== last_navpose_msg.yaw_deg))
      if (changed === true){
        this.state.fixedDict[name]['npData']['roll_deg'] = navpose_msg.roll_deg
        this.state.fixedDict[name]['npData']['pitch_deg'] = navpose_msg.pitch_deg
        this.state.fixedDict[name]['npData']['yaw_deg'] = navpose_msg.yaw_deg
      }
    }

    name = 'position'
    fixed = this.state.fixedDict[name]['fixed']
    navpose = this.state.fixedDict[name]['npData']
    if (fixed === true){
      changed = last_navpose_msg == null ? true:
                ((navpose_msg.x_m !== last_navpose_msg.x_m) ||
                (navpose_msg.y_m !== last_navpose_msg.y_m)||
                (navpose_msg.z_m !== last_navpose_msg.z_m))
      if (changed === true){
        this.state.fixedDict[name]['npData']['x_m'] = navpose_msg.x_m
        this.state.fixedDict[name]['npData']['y_m'] = navpose_msg.y_m
        this.state.fixedDict[name]['npData']['z_m'] = navpose_msg.z_m
      }
    }


      name = 'altitude'
      fixed = this.state.fixedDict[name]['fixed']
      navpose = this.state.fixedDict[name]['npData']
      if (fixed === true){
        changed = last_navpose_msg == null ? true:
                 ((navpose_msg.altitude_m !== last_navpose_msg.altitude_m) )
        if (changed === true){
          this.state.fixedDict[name]['npData']['altitude_m'] = navpose_msg.altitude_m
        }
      }

      name = 'depth'
      fixed = this.state.fixedDict[name]['fixed']
      navpose = this.state.fixedDict[name]['npData']
      if (fixed === true){
        changed = last_navpose_msg == null ? true:
                  ((navpose_msg.depth_m !== last_navpose_msg.depth_m) )
        if (changed === true){
          this.state.fixedDict[name]['npData']['depth_m'] = navpose_msg.depth_m
        }
      }



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






  sendTransformUpdateMessage(name){
    const {sendFrame3DTransformUpdateMsg} = this.props.ros
    const namespace = this.state.namespace + "/set_3d_transform"
    const TX = parseFloat(this.state.transformsDict[name]['transformTX'])
    const TY = parseFloat(this.state.transformsDict[name]['transformTY'])
    const TZ = parseFloat(this.state.transformsDict[name]['transformTZ'])
    const RX = parseFloat(this.state.transformsDict[name]['transformRX'])
    const RY = parseFloat(this.state.transformsDict[name]['transformRY'])
    const RZ = parseFloat(this.state.transformsDict[name]['transformRZ'])
    const HO = parseFloat(this.state.transformsDict[name]['transformHO'])
    const transformList = [TX,TY,TZ,RX,RY,RZ,HO]
    sendFrame3DTransformUpdateMsg(namespace, name, transformList)
  }


  sendTransformClearMessage(name){
    const {sendStringMsg} = this.props.ros
    const namespace = this.state.namespace + "/clear_3d_transform"
    sendStringMsg(namespace,name)
  }

  

  renderMgrControls() {
    const {sendTriggerMsg, sendNavPoseDataMsg} = this.props.ros
    const namespace = this.state.namespace
    ////////////////
    const status_msg = false //this.state.status_msg
    //////////////// 
    
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
        return (
          <Columns>
          <Column>


                <Columns>
                  <Column>

                      <label style={{fontWeight: 'bold'}}>
                            {name_text}
                          </label>

                        <Label title="Set as Fixed">
                        <Toggle
                          checked={fixed === true}
                          onClick={(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + '.fixed')}>
                        </Toggle>
                      </Label>

                    </Column>
                    <Column>

                    </Column>
                  </Columns>




                  <div align={"left"} textAlign={"left"} hidden={!fixed}>

                  {this.renderMgrFixedControls(name, comp_info)}

                  </div>

                  <div align={"left"} textAlign={"left"} hidden={fixed}>

                  {this.renderMgrTopicControls(name, comp_info)}

                  </div>

 
              </Column>
            </Columns>
        )

      }    
    }
  }

  renderMgrFixedControls(name,comp_info) {
    const {sendTriggerMsg, sendNavPoseDataMsg} = this.props.ros
    const namespace = this.state.namespace
    ////////////////
    const status_msg = false //this.state.status_msg
    //////////////// 

    
    if (status_msg == null){
      return (
            <Columns>
            <Column>
            

            </Column>
          </Columns>
      )
    }
    else {
        const fixed = this.state.fixedDict[name]['fixed']
        const npData = this.state.fixedDict[name]['npData']

        if (name === 'location'){
          return (
            <Columns>
            <Column>
                  <Columns>
                    <Column>

                        <Label title={"Latitude"}>
                            <Input
                              value={round(npData['latitude'],6)}
                              id="latitude"
                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + ".npDict.latitude")}
                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'fixedDict.' + name + ".npDict.latitude")}
                              style={{ width: "80%" }}
                            />
                          </Label>

                      </Column>
                      <Column>

                      <Label title={"Longitude"}>
                            <Input
                              value={round(npData['longitude'],6)}
                              id="longitude"
                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + ".npDict.longidude")}
                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'fixedDict.' + name + ".npDict.longidude")}
                              style={{ width: "80%" }}
                            />
                          </Label>

                      </Column>
                    </Columns>


                  <Columns>
                  <Column>
                        <ButtonMenu>
                            <Button onClick={() => this.props.ros.sendNavPoseDataLocationMsg(namespace,this.state.fixedDict[name]['npData'])}>{"Update Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  <Column>
                        <ButtonMenu>
                          <Button onClick={() => this.props.ros.sendNavPoseDataLocationMsg(namespace,this.props.ros.blankNavPoseData)}>{"Clear Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  </Columns>
            </Column>
            </Columns>
          )
        }

        else if (name === 'heading'){
          return (
            <Columns>
            <Column>
                  <Columns>
                    <Column>

                        <Label title={"Heading (Deg)"}>
                            <Input
                              value={round(npData['heading_deg'],2)}
                              id="heading"
                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + ".npDict.heading_deg")}
                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'fixedDict.' + name + ".npDict.heading_deg")}
                              style={{ width: "80%" }}
                            />
                          </Label>

                      </Column>
                      <Column>

                      </Column>
                    </Columns>


                  <Columns>
                  <Column>
                        <ButtonMenu>
                            <Button onClick={() => this.props.ros.sendNavPoseDataHeadingMsg(namespace,this.state.fixedDict[name]['npData'])}>{"Update Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  <Column>
                        <ButtonMenu>
                          <Button onClick={() => this.props.ros.sendNavPoseDataHeadingMsg(namespace,this.props.ros.blankNavPoseData)}>{"Clear Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  </Columns>
            </Column>
            </Columns>
          )
        }


        if (name === 'orientation'){
          return (
            <Columns>
            <Column>
                  <Columns>
                    <Column>

                        <Label title={"Roll (Deg)"}>
                            <Input
                              value={round(npData['roll_deg'],2)}
                              id="roll"
                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + ".npDict.roll_deg")}
                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'fixedDict.' + name + ".npDict.roll_deg")}
                              style={{ width: "80%" }}
                            />
                          </Label>

                      </Column>
                      <Column>

                      <Label title={"Pitch (Deg)"}>
                            <Input
                              value={round(npData['pitch_deg'],2)}
                              id="pitch"
                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + ".npDict.pitch_deg")}
                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'fixedDict.' + name + ".npDict.pitch_deg")}
                              style={{ width: "80%" }}
                            />
                          </Label>

                      </Column>
                      <Column>

                      <Label title={"Yaw (Deg)"}>
                            <Input
                              value={round(npData['yaw_deg'],2)}
                              id="yaw"
                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + ".npDict.yaw_deg")}
                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'fixedDict.' + name + ".npDict.yaw_deg")}
                              style={{ width: "80%" }}
                            />
                          </Label>

                      </Column>
                    </Columns>


                  <Columns>
                  <Column>
                        <ButtonMenu>
                            <Button onClick={() => this.props.ros.sendNavPoseDataOrientationMsg(namespace,this.state.fixedDict[name]['npData'])}>{"Update Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  <Column>
                        <ButtonMenu>
                          <Button onClick={() => this.props.ros.sendNavPoseDataOrientationMsg(namespace,this.props.ros.blankNavPoseData)}>{"Clear Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  </Columns>
            </Column>
            </Columns>
          )
        }


        if (name === 'position'){
          return (
            <Columns>
            <Column>
                  <Columns>
                    <Column>

                        <Label title={"X (Meters)"}>
                            <Input
                              value={round(npData['x_m'],2)}
                              id="x"
                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + ".npDict.x_m")}
                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'fixedDict.' + name + ".npDict.x_m")}
                              style={{ width: "80%" }}
                            />
                          </Label>

                      </Column>
                      <Column>

                      <Label title={"Y (Meters)"}>
                            <Input
                              value={round(npData['y_m'],2)}
                              id="x"
                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + ".npDict.y_m")}
                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'fixedDict.' + name + ".npDict.y_m")}
                              style={{ width: "80%" }}
                            />
                          </Label>

                      </Column>
                      <Column>

                      <Label title={"Z (Meters)"}>
                            <Input
                              value={round(npData['z_m'],2)}
                              id="z"
                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + ".npDict.z_m")}
                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'fixedDict.' + name + ".npDict.z_m")}
                              style={{ width: "80%" }}
                            />
                          </Label>

                      </Column>
                    </Columns>


                  <Columns>
                  <Column>
                        <ButtonMenu>
                            <Button onClick={() => this.props.ros.sendNavPoseDataPositionMsg(namespace,this.state.fixedDict[name]['npData'])}>{"Update Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  <Column>
                        <ButtonMenu>
                          <Button onClick={() => this.props.ros.sendNavPoseDataPositionMsg(namespace,this.props.ros.blankNavPoseData)}>{"Clear Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  </Columns>
            </Column>
            </Columns>
          )
        }


        else if (name === 'altitude'){
          return (
            <Columns>
            <Column>
                  <Columns>
                    <Column>

                        <Label title={"Altitude (Meters)"}>
                            <Input
                              value={round(npData['altitude_m'],2)}
                              id="altitude"
                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + ".npDict.altitude_m")}
                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'fixedDict.' + name + ".npDict.altitude_m")}
                              style={{ width: "80%" }}
                            />
                          </Label>

                      </Column>
                      <Column>

                      </Column>
                    </Columns>


                  <Columns>
                  <Column>
                        <ButtonMenu>
                            <Button onClick={() => this.props.ros.sendNavPoseDataAltitudeMsg(namespace,this.state.fixedDict[name]['npData'])}>{"Update Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  <Column>
                        <ButtonMenu>
                          <Button onClick={() => this.props.ros.sendNavPoseDataAltitudeMsg(namespace,this.props.ros.blankNavPoseData)}>{"Clear Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  </Columns>
            </Column>
            </Columns>
          )
        }


        else if (name === 'depth'){
          return (
            <Columns>
            <Column>
                  <Columns>
                    <Column>

                        <Label title={"Depth (Meters)"}>
                            <Input
                              value={round(npData['depth_m'],2)}
                              id="depth"
                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'fixedDict.' + name + ".npDict.depth_m")}
                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'fixedDict.' + name + ".npDict.depth_m")}
                              style={{ width: "80%" }}
                            />
                          </Label>

                      </Column>
                      <Column>

                      </Column>
                    </Columns>


                  <Columns>
                  <Column>
                        <ButtonMenu>
                            <Button onClick={() => this.props.ros.sendNavPoseDataDepthMsg(namespace,this.state.fixedDict[name]['npData'])}>{"Update Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  <Column>
                        <ButtonMenu>
                          <Button onClick={() => this.props.ros.sendNavPoseDataDepthMsg(namespace,this.props.ros.blankNavPoseData)}>{"Clear Fix"}</Button>
                        </ButtonMenu>
                  </Column>
                  </Columns>
            </Column>
            </Columns>
          )
        }

        else {
          return (

                  <Columns>
                    <Column>

                  </Column>
                  </Columns>
          )
        }



    }
  }


  renderTopicControls(name, comp_info) {
    const {sendTriggerMsg, sendNavPoseDataMsg} = this.props.ros
    const namespace = this.state.namespace
    ////////////////
    const status_msg = false //this.state.status_msg
    //////////////// 
    
    if (status_msg == null){
      return (
            <Columns>
            <Column>
            

            </Column>
          </Columns>
      )
    }
    else {
  
        const name = comp_info.name
        const name_text = name.toUpperCase()
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
        const fixed = this.state.fixedDict[name]['fixed']
        const topic_selected = topic !== 'None' && topic !== ''
        const show_transform = this.state.showTransformsDict[name]
        const this_transform = this.state.transformsDict[name]
        return (
          <Columns>
          <Column>

                      <Label title={"Select Source Topic"}>
                        <Select
                          id={name}
                          onChange={() => this.onTopicSelected(name)}
                          value={topic}
                          >
                          {namespace
                            ? this.createTopicOptions(name)
                            : <Option value={'None'}>{'None'}</Option>}
                        </Select>
                      </Label>

                      <div align={"left"} textAlign={"left"} hidden={fixed === true || topic_selected === false}>

                            <Columns>
                              <Column>

                                  <pre style={{ height: "200px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                                    {("\nReceive Rate: " + rate + 
                                        "\nLast Pub Time Sec: " + time)}
                                  </pre>


                                  <Label title="Show 3D Transform">
                                    <Toggle
                                      checked={show_transform === true}
                                      onClick={(event) => onUpdateSetStateValue.bind(this)('showTransformsDict.' + name)}>
                                    </Toggle>
                                  </Label>


                                </Column>
                              <Column>

                              </Column>
                              </Columns>

              



                              <div align={"left"} textAlign={"left"} hidden={fixed === true || topic_selected === false || show_transform === false}>


                                    <Columns>
                                      <Column>

                                      <Label title={"X (m)"}>
                                        <Input
                                          value={this_transform.transformTX}
                                          id="XTranslation"
                                          onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'transformsDict.' + name + ".transformTX")}
                                          onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'transformsDict.' + name + ".transformTX")}
                                          style={{ width: "80%" }}
                                        />
                                      </Label>

                                      <Label title={"Y (m)"}>
                                        <Input
                                          value={this_transform.transformTY}
                                          id="YTranslation"
                                          onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'transformsDict.' + name + ".transformTY")}
                                          onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'transformsDict.' + name + ".transformTY")}
                                          style={{ width: "80%" }}
                                        />
                                      </Label>

                                      <Label title={"Z (m)"}>
                                        <Input
                                          value={this_transform.transformTZ}
                                          id="ZTranslation"
                                          onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'transformsDict.' + name + ".transformTZ")}
                                          onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'transformsDict.' + name + ".transformTZ")}
                                          style={{ width: "80%" }}
                                        />
                                      </Label>

                                    </Column>
                                    <Column>

                                      <Label title={"Roll (deg)"}>
                                        <Input
                                          value={this_transform.transformRX}
                                          id="XRotation"
                                          onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'transformsDict.' + name + ".transformRX")}
                                          onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'transformsDict.' + name + ".transformRX")}
                                          style={{ width: "80%" }}
                                        />
                                      </Label>

                                      <Label title={"Pitch (deg)"}>
                                        <Input
                                          value={this_transform.transformRY}
                                          id="YRotation"
                                          onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'transformsDict.' + name + ".transformRY")}
                                          onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'transformsDict.' + name + ".transformRY")}
                                          style={{ width: "80%" }}
                                        />
                                      </Label>

                                          <Label title={"Yaw (deg)"}>
                                            <Input
                                              value={this_transform.transformRZ}
                                              id="ZRotation"
                                              onChange= {(event) => onUpdateSetStateValue.bind(this)(event,'transformsDict.' + name + ".transformRZ")}
                                              onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,'transformsDict.' + name + ".transformRZ")}
                                              style={{ width: "80%" }}
                                            />
                                          </Label>

                                        </Column>
                                      </Columns>




                                      <Columns>
                                          <Column>

                                            <ButtonMenu>
                                              <Button onClick={() => this.sendTransformUpdateMessage(name)}>{"Update Transform"}</Button>
                                            </ButtonMenu>

                                          </Column>
                                          <Column>

                                                  <ButtonMenu>
                                                      <Button onClick={() => this.sendTransformClearMessage(name)}>{"Clear Transform"}</Button>
                                                  </ButtonMenu>

                                            </Column>
                                      </Columns>
                      
                                </div>  


                      </div>


              </Column>
            </Columns>
        )

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
