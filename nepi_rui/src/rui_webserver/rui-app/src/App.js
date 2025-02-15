/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
import { Route, Switch, withRouter } from "react-router-dom"
import { observer, inject } from "mobx-react"

import Page from "./Page"
import Nav from "./Nav"
import HorizontalDivider from "./HorizontalDivider"
//import PageLock from "./PageLock"


import Dashboard from "./NepiDashboard"

import DevicesSelector from "./NepiDevicesSelector"
import AppsAutoSelector from "./NepiAppsAutoSelector"
import AppsDataSelector from "./NepiAppsDataSelector"
import AppsNavPoseSelector from "./NepiAppsNavPoseSelector"
import AppsAiSelector from "./NepiAppsAiSelector"
import DriversSelector from "./NepiDriversSelector"


import DeviceMgr from "./NepiSystemDevice"
import NavPoseMgr from "./NepiMgrNavPose"
import SoftwareMgr from "./NepiSystemSoftware"
import AisMgr from "./NepiSystemAis"
import AppsMgr from "./NepiSystemApps"


//const IS_LOCAL = window.location.hostname === "localhost"

@inject("ros")
@withRouter
@observer
class App extends Component {

  componentDidMount() {
    this.props.ros.checkROSConnection()
  }

  render() {
    const { license_valid, license_server, license_type } = this.props.ros
    const unlicensed = (license_server !== null) && 
      (license_server.readyState === 1) && 
      (license_valid === false) 
      return (
      <Page>
        <Nav
          unlicensed={unlicensed}
          license_type={license_type}
          pages={[
            { path: "/", label: "Dashboard" },
            { path: "/devices_selector", label: "Devices"},
            { path: "/apps_data_selector", label: "Data"},
            { path: "/apps_navpose_selector", label: "NavPose"},
            { path: "/apps_ai_selector", label: "AI_System"},
            { path: "/apps_auto_selector", label: "Automation"},
            {
              path: "/system",
              label: "System",
              subItems: [
                { path: "/device_config", label: "Device" },
                { path: "/navPose", label: "NavPose" },
                { path: "/software_mgr", label: "Software" },
                { path: "/drivers_selector", label: "Drivers"},
                { path: "/apps_mgr", label: "Apps"},
                { path: "/ais_mgr", label: "AI Models"}
              ]
            },
            {
              path: "/help",
              label: "Help",
              subItems: [
                { path: "/docs", label: "Docs" },
                { path: "/tuts", label: "Tutorials" },
                { path: "/vids", label: "Videos" },
              ]
            }
          ]}
        />
        <HorizontalDivider />
        <Switch>
          <Route exact path="/" component={Dashboard} />

          <Route path="/apps_auto_selector" component={AppsAutoSelector} />
          <Route path="/apps_data_selector" component={AppsDataSelector} />
          <Route path="/apps_navpose_selector" component={AppsNavPoseSelector} />
          <Route path="/apps_ai_selector" component={AppsAiSelector} />
          <Route path="/drivers_selector" component={DriversSelector} />
          <Route path="/devices_selector" component={DevicesSelector} />

          <Route path="/navPose" component={NavPoseMgr} />
          <Route path="/device_config" component={DeviceMgr} />
          <Route path="/software_mgr" component={SoftwareMgr} />
          <Route path="/apps_mgr" component={AppsMgr} />
          <Route path="/ais_mgr" component={AisMgr} />



          <Route path='/docs' component={() => {
             window.location.href = 'https://nepi.com/documentation/';
             return null;
            }}/>
          <Route path='/tuts' component={() => {
             window.location.href = 'https://nepi.com/tutorials/';
             return null;
            }}/>
          <Route path='/vids' component={() => {
             window.location.href = 'https://nepi.com/videos/';
             return null;
            }}/>
          
        </Switch>
      </Page>
    )
  }
}

export default App
