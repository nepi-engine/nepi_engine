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
import Select, { Option } from "./Select"
import Styles from "./Styles"

import DriversSelector from "./NepiDriversSelector"

import IDX from "./NepiSensorsImaging"
import PTX from "./NepiControlsPanTilt"
import LSX from "./NepiControlsLights"
import RBX from "./NepiControlsRobots"



@inject("ros")
@observer

class DevicesSelector extends Component {
  constructor(props) {
    super(props)

    this.state = {
      show_delete_drv: false,
      mgrName: "drivers_mgr",
      mgrNamespace: null,

      viewabledrvs: false,

      drvs_list: ['NONE'],
      last_drvs_list: [],
      drvs_active_list: [],
      drvs_active_group_list: [],
      drvs_install_path: null,
      drvs_install_list: [],
      selected_app: 'NONE',

      drvs_rui_list: null,
      drvs_group_list: [],

      drv_name: 'NONE',
      drv_description: null,
      drvs_path: null,
      drv_options_menu: null,
      active_state: null,

      backup_removed_drvs: true,

      connected: false,

      drvsListener: null,
      drvListener: null,

      selected_app_install_pkg: null,
      needs_update: true
    }


    this.getMgrNamespace = this.getMgrNamespace.bind(this)

    this.updatedrvsStatusListener = this.updatedrvsStatusListener.bind(this)
    this.drvsStatusListener = this.drvsStatusListener.bind(this)

    this.toggleViewabledrvs = this.toggleViewabledrvs.bind(this)  
    this.onToggledrvSelection = this.onToggledrvSelection.bind(this)  

    
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
  drvsStatusListener(message) {
    this.setState({
      drvs_path: message.drivers_path,
      drvs_list: message.drivers_ordered_list,
      drvs_group_list: message.drivers_group_list,
      drvs_active_list: message.drivers_active_list,
      drvs_active_group_list: message.drivers_active_group_list,
      connected: true
    })    

  }

  // Function for configuring and subscribing to Status
  updatedrvsStatusListener() {
    const statusNamespace = this.getMgrNamespace() + '/status'
    if (this.state.drvsListener) {
      this.state.drvsListener.unsubscribe()
    }
    var drvsListener = this.props.ros.setupStatusListener(
          statusNamespace,
          "nepi_ros_interfaces/DriversStatus",
          this.drvsStatusListener
        )
    this.setState({ drvsListener: drvsListener,
      needs_update: false})
  }




  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getMgrNamespace()
    const namespace_updated = (prevState.mgrNamespace !== namespace && namespace !== null)
    const needs_update = (this.state.needs_update && namespace !== null)
    if (namespace_updated || needs_update) {
      if (namespace.indexOf('null') === -1){
        this.setState({
          mgrNamespace: namespace,
        })
        this.updatedrvsStatusListener()
      } 
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.drvsListener) {
      this.state.drvsListener.unsubscribe()
    }
  }



  renderNoneApp() {
    return (
      <Columns>
        <Column>

      </Column>
      </Columns>
    )
  }


  
  renderDriversMgr() {
    return (
      <Columns>
        <Column>

        <DriversSelector
         title={"DriversMgr"}
         />

      </Column>
      </Columns>
    )
  }


  renderIdxDev() {
    return (
      <Columns>
        <Column>

        <IDX
         title={"IdxDevice"}
         />

      </Column>
      </Columns>
    )
  }

  renderPtxDev() {
    return (
      <Columns>
        <Column>

        <PTX
         title={"PtxDevice"}
         />

      </Column>
      </Columns>
    )
  }


  renderLsxDev() {
    return (
      <Columns>
        <Column>

        <LSX
         title={"LsxDevice"}
         />

      </Column>
      </Columns>
    )
  }

  renderRbxDev() {
    return (
      <Columns>
        <Column>

        <RBX
         title={"RbxDevice"}
         />

      </Column>
      </Columns>
    )
  }


  

  toggleViewabledrvs() {
    const viewable = !this.state.viewabledrvs
    this.setState({viewabledrvs: viewable})
  }


  onToggledrvSelection(event){
    const drv_name = event.target.innerText
    this.setState({selected_app: drv_name})
  }


  // Function for creating image topic options.
  getdrvOptions() {
    const groupList = this.state.drvs_active_group_list
    var items = []
    const connected = this.state.connected
    if (connected !== true){
      items.push(<Option value={'Connecting'}>{'Connecting'}</Option>)
    }
    else{
      if (groupList) {
        if (groupList.length > 0){
            if (groupList.indexOf("IDX") !== -1){
              items.push(<Option value={"Imaging"}>{"Imaging"}</Option>)
            }
            if (groupList.indexOf("PTX") !== -1){
              items.push(<Option value={"PanTilts"}>{"PanTilts"}</Option>)
            }
            if (groupList.indexOf("LSX") !== -1){
              items.push(<Option value={"Lights"}>{"Lights"}</Option>)
            }
            if (groupList.indexOf("RBX") !== -1){
              items.push(<Option value={"Robots"}>{"Robots"}</Option>)
            }
        }
      }
      items.push(<Option value={"Driver Mgr"}>{"Driver Mgr"}</Option>)
    }
    //items.push(<Option value={'TEST1'}>{'TEST1'}</Option>)
    //items.push(<Option value={'TEST2'}>{'TEST2'}</Option>)
    return items
  }


  renderSelection() {
    const drv_options = this.getdrvOptions()
    const hide_drv_list = !this.state.viewabledrvs && !this.state.connected

    return (
      <React.Fragment>

      <Columns>
        <Column>

        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Select Device App"}
         </label>
         

          <div onClick={this.toggleViewabledrvs} style={{backgroundColor: Styles.vars.colors.grey0}}>
            <Select style={{width: "10px"}}/>
          </div>
          <div hidden={hide_drv_list}>
          {drv_options.map((drv) =>
          <div onClick={this.onToggledrvSelection}
            style={{
              textAlign: "center",
              padding: `${Styles.vars.spacing.xs}`,
              color: Styles.vars.colors.black,
              backgroundColor: (drv.props.value === this.state.selected_app) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
              cursor: "pointer",
              }}>
              <body drv-topic ={drv} style={{color: Styles.vars.colors.black}}>{drv}</body>
          </div>
          )}
          </div>

      </Column>
      </Columns>

      </React.Fragment>
    )
  }


  renderdrvlication() {
    const sel_app = this.state.selected_app

    if (sel_app === "NONE"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
          {this.renderNoneApp()}    
        </React.Fragment>
      )
    }
    else if (sel_app === "Imaging"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
            </label>
            {this.renderIdxDev()}    
        </React.Fragment>
      )
    }
    else if (sel_app === "PanTilts"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
          {this.renderPtxDev()}    
        </React.Fragment>
      )
    }
    else if (sel_app === "Lights"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
          {this.renderLsxDev()}    
        </React.Fragment>
      )
    }
    else if (sel_app === "Robots"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
          {this.renderRbxDev()}    
        </React.Fragment>
      )
    }
    else if (sel_app === "Driver Mgr"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
          {this.renderDriversMgr()}    
        </React.Fragment>
      )
    }


  }



  render() {
    return (


      <div style={{ display: 'flex' }}>
        <div style={{ width: '10%' }}>
          {this.renderSelection()}
        </div>

        <div style={{ width: '5%' }}>
          {}
        </div>

        <div style={{ width: '85%' }}>
          {this.renderdrvlication()}
        </div>
      </div>

    )
  }

}

export default DevicesSelector
