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

import NepiDashboardData from "./NepiDashboardData"
import FilePubImgApp from "./NepiAppFilePubImg"
import FilePubVidApp from "./NepiAppFilePubVid"
import FilePubPcdApp from "./NepiAppFilePubPcd"
import PointcloudViewerApp from "./NepiAppPointcloudViewer"
import ImageViewerApp from "./NepiAppImageViewer"
//import ImageSequencer from "./NepiAppImageSequencer"


@inject("ros")
@observer

// Pointcloud Application page
class AppsDataSelector extends Component {
  constructor(props) {
    super(props)

    this.state = {
      show_delete_app: false,
      mgrName: "apps_mgr",
      mgrNamespace: null,

      viewableApps: false,

      apps_list: ['NONE'],
      last_apps_list: [],
      apps_active_list: [],
      apps_install_path: null,
      apps_install_list: [],
      selected_app: 'NONE',

      apps_rui_list: [],
      apps_group_list: [],

      app_name: 'NONE',
      app_description: null,
      apps_path: null,
      app_options_menu: null,
      active_state: null,

      backup_removed_apps: true,

      connected: false,

      appsListener: null,
      appListener: null,
      selected_app_install_pkg: null,
      needs_update: true
    }


    this.getMgrNamespace = this.getMgrNamespace.bind(this)

    this.updateAppsStatusListener = this.updateAppsStatusListener.bind(this)
    this.appsStatusListener = this.appsStatusListener.bind(this)

    this.toggleViewableApps = this.toggleViewableApps.bind(this)  
    this.onToggleAppSelection = this.onToggleAppSelection.bind(this)  

    
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
  appsStatusListener(message) {
    this.setState({
      apps_path: message.apps_path,
      apps_list: message.apps_ordered_list,
      apps_group_list: message.apps_group_list,
      apps_active_list: message.apps_active_list,
      apps_install_path: message.apps_install_path,
      apps_install_list: message.apps_install_list,
      backup_removed_apps: message.backup_removed_apps,
      apps_rui_list: message.apps_rui_list,
      connected: true
    })    

  }

  // Function for configuring and subscribing to Status
  updateAppsStatusListener() {
    const statusNamespace = this.getMgrNamespace() + '/status'
    if (this.state.appsListener) {
      this.state.appsListener.unsubscribe()
    }
    var appsListener = this.props.ros.setupStatusListener(
          statusNamespace,
          "nepi_ros_interfaces/AppsStatus",
          this.appsStatusListener
        )
    this.setState({ appsListener: appsListener,
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
        this.updateAppsStatusListener()
      } 
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.appsListener) {
      this.state.appsListener.unsubscribe()
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


  renderDataDashboard() {
    return (
      <Columns>
        <Column>

        <NepiDashboardData
         title={"Data Dashboard"}
         />

      </Column>
      </Columns>
    )
  }


  renderFilePubImgApp() {
    return (
      <Columns>
        <Column>

        <FilePubImgApp
         title={"Img_File_Publisher"}
         />

      </Column>
      </Columns>
    )
  }

  renderFilePubVidApp() {
    return (
      <Columns>
        <Column>

        <FilePubVidApp
         title={"Vid_File_Publisher"}
         />

      </Column>
      </Columns>
    )
  }

  renderFilePubPcdApp() {
    return (
      <Columns>
        <Column>

        <FilePubPcdApp
         title={"PCD_File_Publisher"}
         />

      </Column>
      </Columns>
    )
  }

  /*
  renderImageSequencerApp() {
    return (
      <Columns>
        <Column>

        <ImageSequencer
         title={"ImageSequencer"}
         />

      </Column>
      </Columns>
    )
  }
  */

  renderImageViewerApp() {
    return (
      <Columns>
        <Column>

        <ImageViewerApp
         title={"ImageViewerApp"}
         />

      </Column>
      </Columns>
    )
  }


  renderPointcloudViewerApp() {
    return (
      <Columns>
        <Column>

        <PointcloudViewerApp
         title={"PointcloudViewerApp"}
         />

      </Column>
      </Columns>
    )
  }

  toggleViewableApps() {
    const viewable = !this.state.viewableApps
    this.setState({viewableApps: viewable})
  }


  onToggleAppSelection(event){
    const app_name = event.target.innerText
    this.setState({selected_app: app_name})
  }


  // Function for creating image topic options.
  getAppOptions() {
    const appsList = this.state.apps_list
    const ruiList = this.state.apps_rui_list 
    const groupList = this.state.apps_group_list
    const activeList = this.state.apps_active_list
    var items = []
    const connected = this.state.connected
    if (connected !== true){
      items.push(<Option value={'Connecting'}>{'Connecting'}</Option>)
    }
    else {
      if (appsList.length > 0){
        for (var i = 0; i < ruiList.length; i++) {
          if (groupList[i] === "DATA" && ruiList[i] !== "None" && activeList.indexOf(appsList[i]) !== -1 ){
            items.push(<Option value={ruiList[i]}>{ruiList[i]}</Option>)
          }
        }
      }
      items.push(<Option value={'Data Dashboard'}>{'Data Dashboard'}</Option>)
    }
    return items
  }


  renderSelection() {
    const app_options = this.getAppOptions()
    const hide_app_list = !this.state.viewableApps && !this.state.connected

    return (
      <React.Fragment>

      <Columns>
        <Column>

        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Select Data App"}
         </label>
         

          <div onClick={this.toggleViewableApps} style={{backgroundColor: Styles.vars.colors.grey0}}>
            <Select style={{width: "10px"}}/>
          </div>
          <div hidden={hide_app_list}>
          {app_options.map((app) =>
          <div onClick={this.onToggleAppSelection}
            style={{
              textAlign: "center",
              padding: `${Styles.vars.spacing.xs}`,
              color: Styles.vars.colors.black,
              backgroundColor: (app.props.value === this.state.selected_app) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
              cursor: "pointer",
              }}>
              <body app-topic ={app} style={{color: Styles.vars.colors.black}}>{app}</body>
          </div>
          )}
          </div>

      </Column>
      </Columns>

      </React.Fragment>
    )
  }


  renderApplication() {
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
    else if (sel_app === "Data Dashboard"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
            </label>
            {this.renderDataDashboard()}    
        </React.Fragment>
      )
    }
    else if (sel_app === "Image File Publisher"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
          {this.renderFilePubImgApp()}    
        </React.Fragment>
      )
    }
    else if (sel_app === "Video File Publisher"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
          {this.renderFilePubVidApp()}    
        </React.Fragment>
      )
    }
    else if (sel_app === "Pointcloud File Publisher"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
          {this.renderFilePubPcdApp()}    
        </React.Fragment>
      )
    }
    else if (sel_app === "Image Viewer"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
            </label>
            {this.renderImageViewerApp()}    
        </React.Fragment>
      )
    }
    else if (sel_app === "Pointcloud Viewer"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
          {this.renderPointcloudViewerApp()}    
        </React.Fragment>
      )
    }
    else if (sel_app === "Image Sequencer"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
          {this.renderImageSequencerApp()}    
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
          {this.renderApplication()}
        </div>
      </div>

    )
  }

}

export default AppsDataSelector
