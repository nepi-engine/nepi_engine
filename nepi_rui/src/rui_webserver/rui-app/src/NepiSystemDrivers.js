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
import Toggle from "react-toggle"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import Input from "./Input"
import BooleanIndicator from "./BooleanIndicator"

import NepiIFSettings from "./Nepi_IF_Settings"

import { onChangeSwitchStateValue,createMenuListFromStrList, onDropdownSelectedSetState } from "./Utilities"

  @inject("ros")
  @observer
  

  class DriversMgr extends Component {
    constructor(props) {
      super(props)
  
      this.state = {
        show_delete_driver: false,
        mgrName: "drivers_mgr",
        mgrNamespace: null,

      viewableDrivers: false,
      viewableGroups: false,

      drivers_list: [],
      last_drivers_list: [],
      drivers_active_list: [],
      drivers_install_path: null,
      drivers_install_list: [],
      selected_driver: null,

      driver_name: 'None',
      driver_description: null,
      drivers_path: null,
      group: null,
      group_name: "None",
      group_id: null,
      drivers_interfaces: null, 
      options_1_name: null, 
      options_1: [],
      set_option_1: null,
      options_2_name: null, 
      options_2: [],
      set_option_2: null,
      discovery: null,
      other_users_list: null,
      driver_options_menu: null,
      active_state: null,

      backup_removed_drivers: true,

      connected: false,

      driversListener: null,
      driverListener: null,
      selected_driver_install_pkg: null,
      group_list: ['IDX','LSX','PTX','RBX','NPX'],
      group_names: ['Imaging','Lights','PanTilts','Robots','NavPose'],
      drivers_group_list: [],
      selected_group: "All",
      needs_update: false
    }


    this.getMgrNamespace = this.getMgrNamespace.bind(this)

    this.sendDriverUpdateOrder = this.sendDriverUpdateOrder.bind(this)
    this.toggleViewableDrivers = this.toggleViewableDrivers.bind(this)
    this.toggleViewableGroups = this.toggleViewableGroups.bind(this)
    this.getDriverOptions = this.getDriverOptions.bind(this)
    this.getInstallOptions = this.getInstallOptions.bind(this)
    this.getGroupOptions = this.getGroupOptions.bind(this)
    this.onChangeGroupSelection = this.onChangeGroupSelection.bind(this)
    this.onToggleDriverSelection = this.onToggleDriverSelection.bind(this)

    this.updateDriversStatusListener = this.updateDriversStatusListener.bind(this)
    this.driversStatusListener = this.driversStatusListener.bind(this)

    this.updateDriverStatusListener = this.updateDriverStatusListener.bind(this)
    this.statusDriverListener = this.statusDriverListener.bind(this)
    this.statusDriverListener = this.statusDriverListener.bind(this)
    this.convertDriverStrInstallToStrList = this.convertDriverStrInstallToStrList.bind(this)

    this.getDisabledStr = this.getDisabledStr.bind(this)
    this.getActiveStr = this.getActiveStr.bind(this)
    this.getReadyStr = this.getReadyStr.bind(this)

    
  
    this.onDropdownSelectedSendDriverOption = this.onDropdownSelectedSendDriverOption.bind(this)
  
  
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
  driversStatusListener(message) {
    this.setState({
      drivers_path: message.drivers_path,
      drivers_list: message.drivers_ordered_list,
      drivers_group_list: message.drivers_group_list,
      drivers_active_list: message.drivers_active_list,
      drivers_install_path: message.drivers_install_path,
      drivers_install_list: message.drivers_install_list,
      backup_removed_drivers: message.backup_removed_drivers,
      selected_driver: message.selected_driver,
      connected: true
    })    

  }

  // Function for configuring and subscribing to Status
  updateDriversStatusListener() {
    const statusNamespace = this.getMgrNamespace() + '/status'
    if (this.state.driversListener) {
      this.state.driversListener.unsubscribe()
    }
    var driversListener = this.props.ros.setupStatusListener(
          statusNamespace,
          "nepi_ros_interfaces/DriversStatus",
          this.driversStatusListener
        )
    this.setState({ driversListener: driversListener})
  }


  statusDriverListener(message) {
    this.setState({
  
      driver_name: message.name,
      driver_description: message.description,
      drivers_path: message.path,
      group: message.group,
      group_id: message.group_id,
      drivers_interfaces: message.interfaces.join(','),
      options_1_name: message.options_1_name,
      options_1: message.options_1,
      set_option_1: message.set_option_1,
      options_2_name: message.options_2_name,
      options_2: message.options_2,
      set_option_2: message.set_option_2,
      discovery: message.discovery,
      other_users_list: message.other_users_list.join(','),
      active_state: message.active_state,
      order: message.order,
      msg_str: message.msg_str
    })
    const groups = this.state.group_list
    const group_index = groups.indexOf(message.group)
    const group_names = this.state.group_names
    if ( group_index !== -1){
      this.state.group_name = group_names[group_index]
    }
  }

    // Function for configuring and subscribing to Status
    updateDriverStatusListener() {
      const namespace = this.getMgrNamespace()
      const statusNamespace = namespace + '/status_driver'
      if (this.state.driverListener) {
        this.state.driverListener.unsubscribe()
      }
      var driverListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_ros_interfaces/DriverStatus",
            this.statusDriverListener
          )
      this.setState({ driverListener: driverListener})
    }

    componentDidMount(){
      this.setState({needs_update: true})
    }

    
  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getMgrNamespace()
    const namespace_updated = (prevState.mgrNamespace !== namespace && namespace !== null)
    if (namespace_updated) {
      if (namespace.indexOf('null') === -1){
        this.setState({
          mgrNamespace: namespace
        })
        this.updateDriversStatusListener()
        this.updateDriverStatusListener()
      } 
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.driversListener) {
      this.state.driversListener.unsubscribe()
      this.state.driverListener.unsubscribe()
    }
  }

  toggleViewableDrivers() {
    const set = !this.state.viewableDrivers
    this.setState({viewableDrivers: set})
  }

  toggleViewableGroups() {
    const set = !this.state.viewableGroups
    this.setState({viewableGroups: set})
  }

  // Function for creating image topic options.
  getDriverOptions() {
    const driversList = this.state.drivers_list  
    const groupsList = this.state.drivers_group_list
    const sel_group = this.state.selected_group
    var items = []
    if (driversList.length > 0){
      for (var i = 0; i < driversList.length; i++) {
          if (sel_group === "All" || groupsList[i] === sel_group){
            items.push(<Option value={driversList[i]}>{driversList[i]}</Option>)
          }
     }
    }
    else{
      items.push(<Option value={'None'}>{'None'}</Option>)
      //items.push(<Option value={'TEST1'}>{'TEST1'}</Option>)
      //items.push(<Option value={'TEST2'}>{'TEST2'}</Option>)
    }
    return items
  }

  // Function for creating group topic options.
  getGroupOptions() { 
    const groups = this.state.drivers_group_list
    const groupIds = this.state.group_list
    const groupNames = this.state.group_names
    var items = []
    items.push(<Option value={'None'}>{'None'}</Option>)
    items.push(<Option value={'All'}>{'All'}</Option>)
    if (groups.length > 0){
      for (var i = 0; i < groupIds.length; i++) {
        if (groups.indexOf(groupIds[i]) !== -1){
          items.push(<Option value={groupIds[i]}>{groupNames[i]}</Option>)
        }
      }
    }
    return items
  }
  


  onToggleDriverSelection(event){
    const {sendStringMsg} = this.props.ros
    const driver_name = event.target.value
    const selectNamespace = this.state.mgrNamespace + "/select_driver"
    sendStringMsg(selectNamespace,driver_name)
  }


  sendDriverUpdateOrder(){
    const {sendUpdateOrderMsg} = this.props.ros
    var namespace = this.state.mgrNamespace
    var driver_name = this.state.driver_name
    var move_cmd = this.state.move_cmd
    sendUpdateOrderMsg(namespace,driver_name,move_cmd)
  }
  convertDriverStrInstallToStrList(inputStr) {
    var strList = []
    if (inputStr != null){
      inputStr = inputStr.replaceAll("[","")
      inputStr = inputStr.replaceAll("]","")
      inputStr = inputStr.replaceAll(" '","")
      inputStr = inputStr.replaceAll("'","")
      strList = inputStr.split(",")
      
    }
    return strList
  }

  // Function for creating image topic options.
  getInstallOptions() {
    const driversList = this.state.drivers_install_list
    var items = []
    const connected = this.state.connected
    if (connected !== true){
      items.push(<Option value={'Connecting'}>{'Connecting'}</Option>)
    }
    else{
      if (driversList.length > 0){
        for (var i = 0; i < driversList.length; i++) {
            items.push(<Option value={driversList[i]}>{driversList[i]}</Option>)
        }
      }
      else{
        items.push(<Option value={'None'}>{'None'}</Option>)
        //items.push(<Option value={'TEST1'}>{'TEST1'}</Option>)
        //items.push(<Option value={'TEST2'}>{'TEST2'}</Option>)
      }
    }
    return items
  }


  onDropdownSelectedSendDriverOption(event) {
    const {sendUpdateOptionMsg} = this.props.ros
    const topic = event.target.id
    const namespace = this.state.mgrNamespace + "/" + topic
    const driver_name = this.state.driver_name
    const option_str = event.target.value
    sendUpdateOptionMsg(namespace, driver_name, option_str)
  }


  renderDriverConfigure() {
    const { sendStringMsg, sendUpdateOrderMsg, sendUpdateActiveStateMsg, } = this.props.ros
    const NoneOption = <Option>None</Option>
    const setting_namespace = this.state.mgrNamespace + '/' + this.state.driver_name

    return (
      <React.Fragment>

        <Section title={"Configure Driver"}>
        <Label title={"Turn off unused drivers for faster startup times"}> </Label>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <div hidden={(this.state.driver_name === 'None')}>

          <label style={{}} align={"left"} textAlign={"left"}>
            {this.state.driver_name}
          </label>
    

          <pre style={{ height: "50px", overflowY: "auto" }}>
          {this.state.driver_description}
          </pre>


      <Columns equalWidth={true}>
      <Column>

      <pre style={{ height: "50px", overflowY: "auto" }}>
          {"Type: " + this.state.group_name}
          </pre>

        <Label title="Enable/Disable Driver">
          <Toggle
            checked={this.state.active_state===true}
            onClick={() => sendUpdateActiveStateMsg(this.state.mgrNamespace + "/update_state", this.state.driver_name, !this.state.active_state)}>
          </Toggle>
          </Label>


      </Column>
      <Column>

      <pre style={{ height: "50px", overflowY: "auto" }}>
          {"Group: " + this.state.group}
          </pre>

      <Label title={"Driver Enabled"}>
          <BooleanIndicator value={(this.state.active_state !== null)? this.state.active_state : false} />
        </Label>
        
      </Column>
      <Column>

      <pre style={{ height: "50px", overflowY: "auto" }}>
          {"Subgroup: " + this.state.group_id}
          </pre>

      </Column>
      </Columns>


        <Columns equalWidth={true}>
          <Column>


      <Label title={"Name"}>
        <Input disabled value={this.state.driver_name} />
      </Label>


      <Label title={"group"}>
        <Input disabled value={this.state.group} />
      </Label>
      <Label title={"group_id"}>
        <Input disabled value={this.state.group_id} />
      </Label>

      <div hidden={!this.state.options_2_name==="None"}>
      <Label title={this.state.options_1_name}> 
        <Select
          id="update_option_1"
          onChange={this.onDropdownSelectedSendDriverOption}
          value={this.state.set_option_1}
        >
          {this.state.options_1
            ? createMenuListFromStrList(this.state.options_1, false, [],[],[])
            : NoneOption}
        </Select>
        </Label>
        </div>


      </Column>
      <Column>

      <Label title={"Interfaces"}>
        <Input disabled value={this.state.interfaces} />
      </Label>
      <Label title={"discovery"}>
        <Input disabled value={this.state.discovery} />
      </Label>
      <Label title={"other_users_list"}>
        <Input disabled value={this.state.other_users_list} />
      </Label>

      <div hidden={!this.state.options_2_name==="None"}>
      <Label title={this.state.options_2_name}> 
        <Select
          id="update_option_2"
          onChange={this.onDropdownSelectedSendDriverOption}
          value={this.state.set_option_2}
        >
          {this.state.options_2
            ? createMenuListFromStrList(this.state.options_2, false, [],[],[])
            : NoneOption}
        </Select>
        </Label>
        </div>

        <label style={{fontWeight: 'bold'}}>
            {"Discovery Options Settings"}
          </label>

        <NepiIFSettings
          settingsNamespace={setting_namespace}
          make_section = {false}
          title={"Nepi_IF_Settings"}
        />

      </Column>
      <Column>


        <Label title={"Driver Start Order"}>
          <Input disabled value={this.state.order} />
        </Label>


        <ButtonMenu>
        <Button onClick={() => sendUpdateOrderMsg(this.state.mgrNamespace + "/update_order", this.state.driver_name, "top")}>{"Move to Top"}</Button>
        </ButtonMenu>

        <ButtonMenu>
        <Button onClick={() => sendUpdateOrderMsg(this.state.mgrNamespace + "/update_order", this.state.driver_name, "up")}>{"Move    Up"}</Button>
        </ButtonMenu>

        <ButtonMenu>
          <Button onClick={() => sendUpdateOrderMsg(this.state.mgrNamespace + "/update_order", this.state.driver_name, "down")}>{"Move Down"}</Button>
        </ButtonMenu>

        <ButtonMenu>
          <Button onClick={() => sendUpdateOrderMsg(this.state.mgrNamespace + "/update_order", this.state.driver_name, "bottom")}>{"Move to Bottom"}</Button>
        </ButtonMenu>
        </Column>
        </Columns>


        <Columns equalWidth={true}>
          <Column>


          <Label title="Show Remove Driver">
                <Toggle
                checked={this.state.show_delete_driver===true}
                onClick={() => onChangeSwitchStateValue.bind(this)("show_delete_driver",this.state.show_delete_driver)}>
                </Toggle>
          </Label>


      </Column>
      <Column>


      </Column>
      <Column>

      </Column>
      <Column>

        </Column>
        </Columns>


      <div hidden={!this.state.show_delete_driver}>

        <Columns equalWidth={true}>
          <Column>


        <Label title="Backup on Remove">
                <Toggle
                checked={this.state.backup_removed_drivers===true}
                onClick={() => this.props.ros.sendBoolMsg(this.state.mgrNamespace + "/backup_on_remeove", this.state.backup_removed_drivers===false)}>
                </Toggle>
        </Label>


      </Column>
      <Column>

      <ButtonMenu>
        <Button onClick={() => sendStringMsg(this.state.mgrNamespace + "/remove_driver", this.state.selected_driver)}>{"Remove Driver"}</Button>
      </ButtonMenu>

      </Column>
      <Column>

        </Column>
        </Columns>

        </div>

        </div>

        </Section>
      
      </React.Fragment>
    )
  }

  
 
  renderDriverInstall() {
    const selected_install_pkg = this.state.selected_driver_install_pkg ? this.state.selected_driver_install_pkg : "None"
    const install_options = this.getInstallOptions()
    const {sendStringMsg} = this.props.ros
    return (
      <React.Fragment>

        <Section title={"Install/Update Drivers"}>

        <Label title="Install"> 
        <Select
          id="select_target"
          onChange={(event) => onDropdownSelectedSetState.bind(this)(event, "selected_driver_install_pkg")}
          value={this.state.selected_driver_install_pkg}
          >
          {install_options}
        </Select>
        </Label>

        <Label title="Show Install Driver">
                <Toggle
                checked={this.state.show_install_driver===true}
                onClick={() => onChangeSwitchStateValue.bind(this)("show_install_driver",this.state.show_install_driver)}>
                </Toggle>
          </Label>

          <div hidden={!this.state.show_install_driver}>
      <ButtonMenu>
        <Button onClick={() => sendStringMsg(this.state.mgrNamespace + "/install_driver_pkg", selected_install_pkg)}>{"Install Driver"}</Button>
      </ButtonMenu>
      </div>

          <Label title={"Install Folder"} >
          </Label>

          <pre style={{ height: "20Spx", overflowY: "auto" }}>
            {this.state.drivers_install_path}
          </pre>



        </Section>
        
      </React.Fragment>
    )
  }



  getActiveStr(){
    const active =  this.state.drivers_active_list
    var config_str_list = []
    for (var i = 0; i < active.length; i++) {
      config_str_list.push(active[i])
      config_str_list.push("\n")
    }
    const config_str =config_str_list.join("")
    return config_str
  }

  
  getDisabledStr(){
    const installed = this.state.drivers_list
    const active =  this.state.drivers_active_list
    var config_str_list = []
    for (var i = 0; i < installed.length; i++) {
      if (active.indexOf(installed[i]) === -1){
        config_str_list.push(installed[i])
        config_str_list.push("\n")
      }
    }
    const config_str =config_str_list.join("")
    return config_str
  }


  getReadyStr(){
    const ready = this.state.drivers_install_list
    var config_str_list = []
    for (var i = 0; i < ready.length; i++) {
      config_str_list.push(ready[i])
      config_str_list.push("\n")
    }
    const config_str =config_str_list.join("")
    return config_str
  }

  onChangeGroupSelection(event){
    var selected_group = event.target.value
    this.setState({selected_group: selected_group})
  }

  render() {
    if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    const selected_driver = this.state.selected_driver
    const driver_options = this.getDriverOptions()
    const active_driver_list = this.state.drivers_active_list
    const hide_driver_list = !this.state.viewableDrivers && !this.state.connected
    const drv_group_options = this.getGroupOptions()
    return (


    <Columns>
      <Column>


      <Columns equalWidth={true}>
        <Column>
        <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <label style={{fontWeight: 'bold'}}>
            {"Filter Driver List"}
          </label>

          <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


            <Select onChange={this.onChangeGroupSelection}
            id="DrvGroupSelector"
            value={this.state.selected_group}>
            {drv_group_options}
            </Select>

            <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Select Driver"}
         </label>

         <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

          <div onClick={this.toggleViewableDrivers} style={{backgroundColor: Styles.vars.colors.grey0}}>
            <Select style={{width: "10px"}}/>
          </div>
          <div hidden={hide_driver_list}>
          {driver_options.map((driver) =>
          <div onClick={this.onToggleDriverSelection}
            style={{
              textAlign: "center",
              padding: `${Styles.vars.spacing.xs}`,
              color: Styles.vars.colors.black,
              backgroundColor: (driver.props.value === selected_driver) ?
                Styles.vars.colors.green :
                (active_driver_list.includes(driver.props.value)) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
              cursor: "pointer",
              }}>
              <body driver-topic ={driver} style={{color: Styles.vars.colors.black}}>{driver}</body>
          </div>
          )}
          </div>

      </Column>
      <Column>

      
      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Active Drivers List "}
          </label>

      <pre style={{ height: "200px", overflowY: "auto" }} align={"center"} textAlign={"center"}>
        {this.getActiveStr()}
        </pre>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Disabled Drivers List "}
          </label>

        <pre style={{ height: "200px", overflowY: "auto" }} align={"center"} textAlign={"center"}>
        {this.getDisabledStr()}
        </pre>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Install Drivers List "}
          </label>

        <pre style={{ height: "200px", overflowY: "auto" }} align={"center"} textAlign={"center"}>
        {this.getReadyStr()}
        </pre>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

      </Column>
      <Column>

      <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/enable_all_drivers")}>{"Enable All"}</Button>
      </ButtonMenu>

      <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/disable_all_drivers")}>{"Disable All"}</Button>
      </ButtonMenu>

      <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/refresh_drivers")}>{"Refresh"}</Button>
      </ButtonMenu>

        <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/factory_reset")}>{"Factory Reset"}</Button>
      </ButtonMenu>

        </Column>
      </Columns>


      </Column>
      <Column>
      

      {this.renderDriverConfigure()}


       </Column>
     </Columns>
         

          

    )
  }

}

export default DriversMgr
