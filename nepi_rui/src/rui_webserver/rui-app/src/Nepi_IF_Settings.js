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
import Toggle from "react-toggle"
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Styles from "./Styles"
import Select from "./Select"
import Input from "./Input"


import { createMenuListFromStrList, onChangeSwitchStateValue } from "./Utilities"

@inject("ros")
@observer

// Component that contains the Settings controls
class Nepi_IF_Settings extends Component {
  constructor(props) {
    super(props)

    // these states track the values through  Status messages
    this.state = {
      capSettingsTypes: ['Menu','Discrete','String','Bool','Int','Float'],
      capSettingsNamesList: [],
      capSettingsTypesList: [],
      capSettingsOptionsLists: [],
      settingsNamesList: [],
      settingsTypesList: [],
      settingsValuesList: [],
      settings: null,
      settingsCount: 0,
      selectedSettingInd: 0,
      selectedSettingName: "",
      selectedSettingType: "",
      selectedSettingValue: "",
      selectedSettingLowerLimit: "",
      selectedSettingUpperLimit: "",
      selectedSettingOptions: [],
      selectedSettingInput: "",

      settings_namespace: 'None',

      settingsListener: null,
    }

    this.updateSettingsListener = this.updateSettingsListener.bind(this)
    this.settingsStatusListener = this.settingsStatusListener.bind(this)

    this.updateCapSettingsLists = this.updateCapSettingsLists.bind(this)

    this.getSettingsAsString = this.getSettingsAsString.bind(this)
    this.getSettingValue = this.getSettingValue.bind(this)

    this.onChangeBoolSettingValue = this.onChangeBoolSettingValue.bind(this)
    this.onChangeDescreteSettingValue = this.onChangeDescreteSettingValue.bind(this)
    this.onUpdateInputSettingValue = this.onUpdateInputSettingValue.bind(this)
    this.onKeySaveInputSettingValue = this.onKeySaveInputSettingValue.bind(this)

    this.updateSelectedSettingInfo = this.updateSelectedSettingInfo.bind(this)
    this.getSelectedSettingInfo = this.getSelectedSettingInfo.bind(this)
    this.getSortedStrList = this.getSortedStrList.bind(this)

    this.renderSettings = this.renderSettings.bind(this)
  }

  // Callback for handling ROS Settings Status messages
  settingsStatusListener(message) {
    const settings = message.settings_list
    var namesList = []
    var typesList = []
    var valuesList = []
    for (let ind = 0; ind < settings.length; ind++){
      namesList.push(settings[ind].name_str)
      typesList.push(settings[ind].type_str)
      valuesList.push(settings[ind].value_str)
    }
    const count = namesList.length
    this.setState({settingsNamesList:namesList,
                   settingsTypesList:typesList,
                   settingsValuesList:valuesList,
                   settingsCount: count
    })

    this.updateCapSettingsLists() 
  }

  // Function for configuring and subscribing to Settings Status
  updateSettingsListener() {
    const { settingsNamespace } = this.props
    if (this.state.settingsListener) {
      this.state.settingsListener.unsubscribe()
    }
    var settingsListener = this.props.ros.setupSettingsStatusListener(
      settingsNamespace,
      this.settingsStatusListener
    )
    this.setState({ settingsListener: settingsListener})
  }


  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { settingsNamespace } = this.props
    if (prevProps.settingsNamespace !== settingsNamespace && settingsNamespace != null) {
      this.updateSettingsListener()
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Settings Status message
  componentWillUnmount() {
    if (this.state.settingsListener) {
      this.state.settingsListener.unsubscribe()
    }
  }


  // Function for creating settings options list from capabilities
  updateCapSettingsLists() {
    const {settingCaps} = this.props.ros
    const cur_namespace = this.props.settingsNamespace
    var namesList = []
    var typesList = []
    var optionsLists = []
    if (settingCaps && cur_namespace){
      if (settingCaps[this.props.settingsNamespace]){
        const set_namespace = this.state.settings_namespace
        if (set_namespace !== cur_namespace){

          const capabilities = settingCaps[this.props.settingsNamespace]
          const cap_settings = capabilities.setting_caps_list
          if (capabilities !== undefined){
            for (let ind = 0; ind < cap_settings.length; ind++){
              namesList.push(cap_settings[ind].name_str)
              typesList.push(cap_settings[ind].type_str)
              optionsLists.push(cap_settings[ind].options_list)
            }
            this.setState({
              capSettingsNamesList:namesList,      
              capSettingsTypesList:typesList,
              capSettingsOptionsLists:optionsLists
            })
          }
          else{
            this.setState({settings_namespace: cur_namespace,
              capSettingsNamesList:['None'],      
              capSettingsTypesList:['None'],
              capSettingsOptionsLists:['None']
            })

          }
          this.setState({settings_namespace: cur_namespace })
        }
      }
    } 
  }
  
  getSettingValue(settingName) {
    const namesList = this.state.settingsNamesList
    const valuesList = this.state.settingsValuesList
    const ind = namesList.indexOf(settingName)
    var value = -1
    if (ind !== -1){
      value = valuesList[ind]
    }
    return value
  }

  updateSelectedSettingInfo(event){
    const name_ind = event.nativeEvent.target.selectedIndex
    const name = event.nativeEvent.target[name_ind].text
    const ind = this.state.capSettingsNamesList.indexOf(name)
    if (ind !== -1){
      this.setState({selectedSettingInd : ind })
      this.setState({selectedSettingName  :  name })
      const type = this.state.capSettingsTypesList[ind]
      this.setState({selectedSettingType  :  type }) 
      const value = this.getSettingValue(name) 
      this.setState({selectedSettingValue  : value })
      const options = this.state.capSettingsOptionsLists[ind]
      this.setState({selectedSettingOptions  :  options })
      if (type === "Int" || type === "Float" ) {
        this.setState({selectedSettingLowerLimit  :  options.length > 0 ? options[0] : "" })
        this.setState({selectedSettingUpperLimit  :  options.length > 1 ? options[1] : "" })
      } else {
        this.setState({selectedSettingLowerLimit  :  "" })
        this.setState({selectedSettingUpperLimit  :  "" })
      }
      this.setState({selectedSettingInput  :  value })
      this.render()
    }
    else{
    this.setState({selectedSettingName  :  "NONE" }) 
    this.setState({selectedSettingType  :  "NONE" }) 
    }

  }

  getSelectedSettingInfo(){
    const info = [this.state.selectedSettingInd, this.state.selectedSettingType , this.state.selectedSettingName ,
       this.state.selectedSettingValue , this.state.selectedSettingLowerLimit , this.state.selectedSettingUpperLimit,
       this.state.selectedSettingOptions
       ]
    return info
  }


  onChangeBoolSettingValue(){
    const {updateSetting}  = this.props.ros
    const value = (this.getSettingValue(this.state.selectedSettingName) === "True") ? "False" : "True" 
    updateSetting(this.props.settingsNamespace,
      this.state.selectedSettingName,this.state.selectedSettingType,value)
  }

  onChangeDescreteSettingValue(event){
    const {updateSetting}  = this.props.ros
    const ind = event.nativeEvent.target.selectedIndex
    const value = event.nativeEvent.target[ind].text
    updateSetting(this.props.settingsNamespace,
      this.state.selectedSettingName,this.state.selectedSettingType,value)
  }

  onUpdateInputSettingValue(event) {
    this.setState({ selectedSettingInput: event.target.value })
    document.getElementById("input_setting").style.color = Styles.vars.colors.red
    this.render()
  }

  onKeySaveInputSettingValue(event) {
    const {updateSetting}  = this.props.ros
    if(event.key === 'Enter'){
      const value = this.state.selectedSettingInput
      updateSetting(this.props.settingsNamespace,
        this.state.selectedSettingName,this.state.selectedSettingType,value)
      document.getElementById("input_setting").style.color = Styles.vars.colors.black
      this.updateSelectedSettingInfo()
    }
  }




  getSettingsAsString() {
    var settingsStr = ""
    const namesList = this.state.settingsNamesList
    const valuesList = this.state.settingsValuesList
    var settingsStrList = []
    var sortedStrList = ["None"]
    var settingStr = ""
    for (let ind = 0; ind < namesList.length; ind++){
      settingStr = namesList[ind] + ": " +  valuesList[ind]
      settingsStrList.push(settingStr)
    }
    if (settingsStrList.length > 0){
      sortedStrList = settingsStrList.sort()
    }
    for (let ind = 0; ind < sortedStrList.length; ind++){
      const current_str = sortedStrList[ind]
      sortedStrList[ind] = current_str + "\n"
    }
    settingsStr =sortedStrList.join("")
    return settingsStr
  }

  getSortedStrList(strList){
    var copiedStrList = []
    var sortedStrList = []
    for (let ind = 0; ind < strList.length; ind++){
      copiedStrList.push(strList[ind])
    }
    if (copiedStrList.length > 0){
      sortedStrList = copiedStrList.sort()
    }
    return sortedStrList
  }

  renderSettings() {
    const { sendTriggerMsg} = this.props.ros
    const selSetInfo = this.getSelectedSettingInfo()
    const selSetType = selSetInfo[1]
    const selSetName = selSetInfo[2]
    const selSetMin = selSetInfo[4]
    const selSetMax = selSetInfo[5]
    const selSetOptions= selSetInfo[6]
    const capSettingNamesOrdered = this.getSortedStrList(this.state.capSettingsNamesList)
    const settingsHeight = this.state.settingsCount * 25
    const settingsHeightStr = settingsHeight.toString() + 'px'
    return (
      <Columns>
          <Column>

        <Columns>
          <Column>
            <Label title={"Select Setting"}>
              <Select
                id="selectedSettingName"
                onChange={this.updateSelectedSettingInfo}
                value={this.state.selectedSettingName}
              >
                {createMenuListFromStrList(capSettingNamesOrdered,false,[],['NONE'],[])}
              </Select>
            </Label>

            <div align={"left"} textAlign={"right"} hidden={selSetType !== "Bool" }>
              <Label title={selSetName}>
                <Toggle
                  checked={ (this.getSettingValue(selSetName) === "True")}
                  onClick={() => {this.onChangeBoolSettingValue()}}
                />
              </Label>
            </div>

              

              <div align={"left"} textAlign={"right"} hidden={selSetType !== "Menu" && selSetType !== "Discrete" }>
              <Label title={selSetName}>
                <Select
                  id="descreteSetting"
                  onChange={this.onChangeDescreteSettingValue}
                  value={this.getSettingValue(selSetName)}
                >
                  {createMenuListFromStrList(selSetOptions,false,[],["Select"],[])}
                </Select>
              </Label>
              </div>

            <div align={"left"} textAlign={"right"} 
              hidden={!(selSetType === "String" ||
              selSetType === "Int" ||
              selSetType === "Float")}
            >

                <div align={"left"} textAlign={"right"} hidden={selSetMin === ""}>
                    <Label title={"Lower Input Limit"}>
                      <Input disabled value={selSetMin} />
                    </Label>
                </div>

                <div align={"left"} textAlign={"right"} hidden={selSetMax === ""}>
                    <Label title={"Upper Input Limit"}>
                      <Input disabled value={selSetMax} />
                    </Label>
                </div>

                <Label title={selSetName}>
                  <Input id="input_setting" 
                    value={this.state.selectedSettingInput} 
                    onChange={this.onUpdateInputSettingValue} 
                    onKeyDown= {this.onKeySaveInputSettingValue} />
                </Label>
                <Label title={"* Some changes may require power cycle"}>
                </Label>
            </div>

          </Column>
          <Column>
          <div align={"left"} textAlign={"left"} >
              <ButtonMenu>
                <Button onClick={() => sendTriggerMsg(this.props.settingsNamespace + '/reset_settings')}>{"Reset Settings"}</Button>
              </ButtonMenu>
            </div>
          </Column>
        </Columns>

          
          <Label title={"Current Settings"} >
          </Label>
          <pre style={{ height: settingsHeightStr, overflowY: "auto" }}>
            {this.getSettingsAsString()}
          </pre>

 
          </Column>
        </Columns>
    )
  }

  render() {
    const make_section = (this.props.make_section === false )? this.props.make_section: true
    const cur_namespace = this.props.settingsNamespace
    if (cur_namespace !== "None" && make_section === true){
      return (
        <Section title={"Device Settings"}>
          {this.renderSettings()}
        </Section>
      )
    }
    else if (cur_namespace !== "None" && make_section === false) {
      return (
        <Columns>
          <Column>
          {this.renderSettings()}
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
export default Nepi_IF_Settings
