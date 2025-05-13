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
//import EnableAdjustment from "./EnableAdjustment"
import Button, { ButtonMenu } from "./Button"
import BooleanIndicator from "./BooleanIndicator"
import Toggle from "react-toggle"
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Styles from "./Styles"
import Select, { Option } from "./Select"
import Input from "./Input"


function roundWithSuffix(value, decimals, suffix) {
  return value && (value.toFixed(decimals) + " " + suffix)
}

@inject("ros")
@observer

// Component that contains the Save Data Controls
class NepiIFSaveData extends Component {
  constructor(props) {
    super(props)

    // these states track the values through the components Save Data Status messages
    this.state = {

      reset_save_data: true,
      saveRatesMsg: "",
      saveDataPrefix: "",
      saveUtcTz: false,
      exp_filename: "",
      saveDataEnabled: false,
      saveDataRate: "1.0",
      saveNavDataEnabled: true,
      saveNavDataRate: "10.0",
      saveNamesList: [],
      saveRatesList: [],
      selectedDataProducts: [],

      showSettings: this.props.showSettings ? this.props.showSettings : false,

      needs_update: true,
      saveStatusListener: null,
    }


    this.updateSaveStatusListener = this.updateSaveStatusListener.bind(this)
    this.saveStatusListener = this.saveStatusListener.bind(this)

    this.updateSaveLists = this.updateSaveLists.bind(this)
    this.getSaveNamesList = this.getSaveNamesList.bind(this)
    this.getSaveRatesList = this.getSaveRatesList.bind(this)
    this.getSaveConfigString = this.getSaveConfigString.bind(this)
    this.getSaveRateValue = this.getSaveRateValue.bind(this)
    this.getSaveRateData = this.getSaveRateData.bind(this)
    this.onClickToggleShowSettings = this.onClickToggleShowSettings.bind(this)


    this.getSaveDataValue = this.getSaveDataValue.bind(this)
    this.onChangeBoolSaveDataValue = this.onChangeBoolSaveDataValue.bind(this)
    this.onChangeBoolUtcTzValue = this.onChangeBoolUtcTzValue.bind(this)
    this.getSaveNavDataValue = this.getSaveNavDataValue.bind(this)
    this.onUpdateInputSaveDataRateValue = this.onUpdateInputSaveDataRateValue.bind(this)
    this.onKeySaveInputSaveDataRateValue = this.onKeySaveInputSaveDataRateValue.bind(this)
    this.onUpdateInputSaveDataPrefixValue = this.onUpdateInputSaveDataPrefixValue.bind(this)
    this.onKeySaveInputSaveDataPrefixValue = this.onKeySaveInputSaveDataPrefixValue.bind(this)
    this.onToggleDataProductSelection = this.onToggleDataProductSelection.bind(this)
    this.sendSaveRateUpdate = this.sendSaveRateUpdate.bind(this)
    

    this.getNavNamespace = this.getNavNamespace.bind(this)
    this.onChangeBoolSaveNavDataValue = this.onChangeBoolSaveNavDataValue.bind(this)
    this.onUpdateInputSaveNavDataRateValue = this.onUpdateInputSaveNavDataRateValue.bind(this)
    this.onKeySaveInputSaveNavDataRateValue = this.onKeySaveInputSaveNavDataRateValue.bind(this)
    this.sendSaveNavRateUpdate = this.sendSaveNavRateUpdate.bind(this)

    this.updateSelectedDataProducts = this.updateSelectedDataProducts.bind(this)
    this.getSelectedDataProducts = this.getSelectedDataProducts.bind(this)
    this.getDiskUsageRate = this.getDiskUsageRate.bind(this)

 

    this.convertStrListToMenuList = this.convertStrListToMenuList.bind(this)
    
    this.doNothing = this.doNothing.bind(this)

    this.onSnapshotTriggered = this.onSnapshotTriggered.bind(this)

  }

  // Callback for handling ROS Status messages
  saveStatusListener(message) {
    const saveDataRates = message.save_data_rates
    const saveDirPrefix = message.current_subfolder
    const saveNamePrefix = message.current_filename_prefix
    const saveUtcTz = message.save_data_utc
    const saveData = message.save_data
    const exp_filename = message.example_filename
    var saveDataPrefix = ""
    if (saveDirPrefix === ""){
      saveDataPrefix = saveNamePrefix
    } else {
      saveDataPrefix = saveDirPrefix + "/" + saveNamePrefix
    }
    this.setState({
      saveRatesMsg: saveDataRates,
      saveDataPrefix: saveDataPrefix,
      saveUtcTz: saveUtcTz,
      exp_filename: exp_filename,
      saveDataEnabled: saveData
    })
    this.updateSaveLists()
    this.updateSelectedDataProducts()
  }

  // Function for configuring and subscribing to Status
  updateSaveStatusListener() {
    const { saveNamespace } = this.props
    if (this.state.saveStatusListener) {
      this.state.saveStatusListener.unsubscribe()
    }
    var saveStatusListener = this.props.ros.setupSaveDataStatusListener(
          saveNamespace,
          this.saveStatusListener
        )
    this.setState({ saveStatusListener: saveStatusListener,
      needs_update: false})
  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.props.saveNamespace
    const namespace_updated = (prevState.saveNamespace !== namespace && namespace !== null)
    const needs_update = (this.state.needs_update && namespace !== null)
    if (namespace_updated || needs_update) {
      if (namespace.indexOf('null') === -1){
        this.setState({saveNamespace: namespace})
        this.updateSaveStatusListener()
      }
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.saveStatusListener) {
      this.state.saveStatusListener.unsubscribe()
    }
  }


  // Function for creating configs options list from capabilities
  updateSaveLists() {
    var saveRatesMsg = this.state.saveRatesMsg
    var NamesList = []
    var RatesList = []
    var name = ""
    var rate_int = 0
    var rate_str = ""
    var saveRateMsg = null
    // add None and All options to lists
    NamesList.push("NONE")
    RatesList.push("0.0")
    NamesList.push("ALL")
    RatesList.push(this.state.saveDataRate)
    for (let ind = 0; ind < saveRatesMsg.length; ind++){
        saveRateMsg =saveRatesMsg[ind]
        name = saveRateMsg.data_product
        rate_int = saveRateMsg.save_rate_hz
        rate_str = rate_int.toFixed(2)
        NamesList.push(name)
        RatesList.push(rate_str)
    }
    this.setState({saveNamesList:NamesList})
    this.setState({saveRatesList:RatesList})
  }

  onClickToggleShowSettings(){
    const currentVal = this.state.showSettings 
    this.setState({showSettings: !currentVal})
    this.render()
  }


  getSaveNamesList(){
    const list = this.state.saveNamesList
    return list
  }

  getSaveRatesList(){
    const list = this.state.saveRatesList
    return list
  }


  getSaveConfigString() {
    const namesList = this.state.saveNamesList
    const ratesList = this.state.saveRatesList
    var configsStr = ""
    var entryStr
    var configsStrList = [""]
    for (let ind = 0; ind < namesList.length; ind++) {
      if (namesList[ind] !== "NONE" && namesList[ind] !== "ALL" ){
        entryStr = namesList[ind] + " : " + ratesList[ind] +  " Hz\n"
        configsStrList.push(entryStr)
      }
    }
    var navSaveRate = "0.0"
    if (this.state.saveNavDataEnabled === true){
      navSaveRate = this.state.saveNavDataRate
    }
    entryStr = ("nav : " + navSaveRate + " Hz\n")
    configsStrList.push(entryStr)
    configsStr = configsStrList.join("")
    return configsStr
  }

  getSaveRateValue(dataName) {
    const configsStrList = this.getsaveRatesAsList()
    const configInd = configsStrList.indexOf(dataName)
    return configsStrList[configInd + 1]
  }

  getSaveRateData(dataName) {
    const configValue = this.getSaveRateValue(dataName)
    return parseFloat(configValue)
  }

  sendSaveRateUpdate(data_product,rate) {
    const {updateSaveDataRate}  = this.props.ros
    if (isNaN(rate) === false){
      updateSaveDataRate(this.props.saveNamespace,data_product,rate)
    }
  }

  sendSaveNavRateUpdate() {
    const {updateSaveDataRate}  = this.props.ros
    const navNamespace = this.getNavNamespace()
    const saveRateStr = this.state.saveNavDataRate
    const rate = parseFloat(saveRateStr)
    if (isNaN(rate) === false) {
      updateSaveDataRate(navNamespace,'nav_pose',rate)
    }
  }

  updateSelectedDataProducts() {
    const NamesList = this.state.saveNamesList
    const RatesList = this.state.saveRatesList
    var selectedList = []
    var name = ""
    for (let ind = 0; ind < NamesList.length; ind++){
      name = NamesList[ind]
      if (name !== "NONE" && name !== "ALL"){
        if (parseFloat(RatesList[ind]) > 0) {
          selectedList.push(NamesList[ind])
        }
      }
    }
    this.setState({selectedDataProducts:selectedList})
  }

  getSelectedDataProducts(){
    const list =  this.state.selectedDataProducts
    return list
  }

  doNothing(){
    var ret = false
    return ret
  }
 
  onToggleDataProductSelection(event){
    const data_product = event.target.getAttribute("data-product")
    var selectedList = this.getSelectedDataProducts()
    const saveDataRate = this.state.saveDataRate
    const rate = parseFloat(saveDataRate)
    const ind = selectedList.indexOf(data_product)
    if ( ind !== -1 ){
      delete selectedList[ind]
      this.sendSaveRateUpdate(data_product,0.0)
    } else {
      selectedList.push(data_product)
      this.sendSaveRateUpdate(data_product,rate)
    }
  }

  getSaveDataValue(){
    const saveData = (this.state.saveDataEnabled === true)
    return saveData
  }

  getNavNamespace()  {
    const nsStrList = this.props.saveNamespace.split("/")
    const navNamespace = "/" + nsStrList[1] + "/" + nsStrList[2] + "/nav_pose_pub"
    return navNamespace
  }

  onChangeBoolSaveDataValue(){
    const {sendBoolMsg}  = this.props.ros
    const enabled = (this.state.saveDataEnabled === false)
    const saveNavEnabled = (this.state.saveNavDataEnabled === true)
    const saveDataRate = this.state.saveDataRate
    const rate = parseFloat(saveDataRate)
    this.sendSaveRateUpdate('Active',rate)
    sendBoolMsg(this.props.saveNamespace + '/save_data',enabled)
    if (saveNavEnabled === true){
      const navNamespace = this.getNavNamespace()
      this.sendSaveNavRateUpdate()
      sendBoolMsg(navNamespace + '/save_data',enabled)
    }

  }

  onChangeBoolUtcTzValue(e){
    const {sendBoolMsg}  = this.props.ros
    const enabled = e.target.checked
    const saveNavEnabled = (this.state.saveNavDataEnabled === true)
    sendBoolMsg(this.props.saveNamespace + '/save_data_utc',enabled)
    if (saveNavEnabled === true){
      const navNamespace = this.getNavNamespace()
      this.sendSaveNavRateUpdate()
      sendBoolMsg(navNamespace + '/save_data_utc',enabled)
    }
  }

  getSaveNavDataValue(){
    const saveNavData = (this.state.saveNavDataEnabled === true)
    return saveNavData
  }

  onChangeBoolSaveNavDataValue(){
    const {updateSaveDataRate} = this.props.ros
    const navEnabled = (this.state.saveNavDataEnabled === false)
    const nav_rate_str = this.state.saveNavDataRate
    this.setState({saveNavDataEnabled:navEnabled})
    if (navEnabled){
      const nav_rate = parseFloat(nav_rate_str)
      if (isNaN(nav_rate) === false) {
        const navNamespace = this.getNavNamespace()
        updateSaveDataRate(navNamespace,"nav_pose",nav_rate)
      }
    }
  }



  onUpdateInputSaveDataRateValue(event) {
    this.setState({ saveDataRate: event.target.value })
    document.getElementById("input_rate").style.color = Styles.vars.colors.red
    this.render()
  }

  onKeySaveInputSaveDataRateValue(event) {
    const saveDataRate = this.state.saveDataRate
    const rate = parseFloat(saveDataRate)
    if(event.key === 'Enter'){
      const rate = parseFloat(event.target.value)
      if (!isNaN(rate)){
        this.setState({saveDataRate: event.target.value })
        this.sendSaveRateUpdate('Active',rate)
      }
      document.getElementById("input_rate").style.color = Styles.vars.colors.black
    }
  }


  onUpdateInputSaveNavDataRateValue(event) {
    this.setState({saveNavDataRate: event.target.value })
    document.getElementById("nav_rate").style.color = Styles.vars.colors.red
    this.render()
  }

  onKeySaveInputSaveNavDataRateValue(event) {
    if(event.key === 'Enter'){
      const rate = parseFloat(event.target.value)
      if (!isNaN(rate)){
        this.setState({saveNavDataRate: event.target.value })
        this.sendSaveNavRateUpdate()
      }
      document.getElementById("nav_rate").style.color = Styles.vars.colors.black
    }
  }

  onUpdateInputSaveDataPrefixValue(event) {
    this.setState({ saveDataPrefix: event.target.value })
    document.getElementById("input_prefix").style.color = Styles.vars.colors.red
    this.render()
  }

  onKeySaveInputSaveDataPrefixValue(event) {
    const { updateSaveDataPrefix} = this.props.ros
    const key = event.key
    const value = event.target.value
    const navEnabled = (this.state.saveNavDataEnabled === true)
    if(key === 'Enter'){
      document.getElementById("input_prefix").style.color = Styles.vars.colors.black
      updateSaveDataPrefix(this.props.saveNamespace,value)
      if ( navEnabled === true) {
        const navNamespace = this.getNavNamespace()
        updateSaveDataPrefix(navNamespace,value)
      }
    }
  }
  

  convertStrListToMenuList(strList){
    var menuList = []
    for (let ind = 0; ind < strList.length; ind++){
      menuList.push(<Option>{strList[ind]}</Option>)
    } 
    return menuList
  }

  getDiskUsageRate(){
    const {systemStatusDiskRate} = this.props.ros
    return systemStatusDiskRate
  }


  onSnapshotTriggered(){
    const { onSnapshotEventTriggered} = this.props.ros
    onSnapshotEventTriggered(this.props.saveNamespace)
    const navEnabled = (this.state.saveNavDataEnabled === true)
    if (navEnabled){
      const navNamespace = this.getNavNamespace()
      onSnapshotEventTriggered(navNamespace)
    }
  }

  render() {
    const { resetSaveDataTriggered} = this.props.ros
    const saveDataEnabled = this.getSaveDataValue()
    const saveNavDataEnabled = this.getSaveNavDataValue()
    const dataProdcutSources = this.getSaveNamesList()
    const selectedDataProducts = this.getSelectedDataProducts()
    const diskUsage = this.getDiskUsageRate()
    const saveUtcTz = this.state.saveUtcTz
    
    return (
      <Section title={"Save Data"}>

            <Columns>
            <Column>

              <ButtonMenu >
                  <Button onClick={this.onSnapshotTriggered}>{"Take Snapshot"}</Button>
                </ButtonMenu>

                <Label title={"Save Data"}>
                <Toggle
                  checked={ (saveDataEnabled === true) }
                  onClick={() => {this.onChangeBoolSaveDataValue()}}
                />
              </Label>

                </Column>
            <Column>



              </Column>
              <Column>


              <Label title="Show Settings">
                    <Toggle
                      checked={this.state.showSettings===true}
                      onClick={this.onClickToggleShowSettings}>
                    </Toggle>
                  </Label>

            </Column>
            </Columns>


            <div align={"left"} textAlign={"left"} hidden={this.state.showSettings === false}>

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        
          <Columns>
            <Column>
            <Label title={"Use UTC Time"}>
                <Toggle
                  checked={ (saveUtcTz) }
                  onClick={this.onChangeBoolUtcTzValue}
                />
              </Label>

              <Label title={"Set Save Rate (Hz)"}>
                <Input id="input_rate" 
                    value={this.state.saveDataRate} 
                    onChange={this.onUpdateInputSaveDataRateValue} 
                    onKeyDown= {this.onKeySaveInputSaveDataRateValue} />
            </Label>

            <Label title={"Set Save Prefix"}>
              <Input id="input_prefix" 
                  value={this.state.saveDataPrefix} 
                  onChange={this.onUpdateInputSaveDataPrefixValue} 
                  onKeyDown= {this.onKeySaveInputSaveDataPrefixValue} />
            </Label>



            <Label title="Selected Message Topics">
              <div onClick={this.doNothing} style={{backgroundColor: Styles.vars.colors.grey0}}>
                <Select style={{width: "10px"}}/>
              </div>
              <div hidden={false}>
              {dataProdcutSources.map((data_product) =>
              <div onClick={this.onToggleDataProductSelection}
                style={{
                  textAlign: "center",
                  padding: `${Styles.vars.spacing.xs}`,
                  color: Styles.vars.colors.black,
                  backgroundColor: (selectedDataProducts.includes(data_product))? Styles.vars.colors.blue : Styles.vars.colors.grey0,
                  cursor: "pointer",
                  }}>
                  <body data-product={data_product} style={{color: Styles.vars.colors.black}}>{data_product}</body>
              </div>
              )}
              </div>
            </Label>
            <Label title={"Nav Data Saving"}>
            </Label>
            <Label title={"Enable"}>
                <Toggle
                  checked={ (saveNavDataEnabled === true) }
                  onClick={this.onChangeBoolSaveNavDataValue}
                />
              </Label>
              <Label title={"Max Rate (Hz)"}>
                  <Input id="nav_rate" 
                      value={this.state.saveNavDataRate} 
                      onChange={this.onUpdateInputSaveNavDataRateValue} 
                      onKeyDown= {this.onKeySaveInputSaveNavDataRateValue} />
              </Label>


            </Column>
            <Column>

              <ButtonMenu>
                <Button onClick={() => resetSaveDataTriggered(this.props.saveNamespace)}>{"Reset Save Settings"}</Button>
              </ButtonMenu>
              <Label title={""}>
              </Label>
              <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

              <Label title={"Saving"}>
                <BooleanIndicator value={(this.getSaveDataValue() === true)} />
              </Label>

              <Label title={"Data Save Rate"}>
                <Input disabled value={roundWithSuffix(diskUsage, 3, "MB/s")} />
              </Label>

              <Label title={"Example Filename"}>
              </Label>

              <pre style={{ height: "100px", overflowY: "auto" }}>
                {this.state.exp_filename}
              </pre>

              <Label title={"Data Product Save Settings"}>
              </Label>

              <pre style={{ height: "400px", overflowY: "auto" }}>
                {this.getSaveConfigString()}
              </pre>

            </Column>
          </Columns>
      
        </div>


      </Section>
    )
  }

}
export default NepiIFSaveData
