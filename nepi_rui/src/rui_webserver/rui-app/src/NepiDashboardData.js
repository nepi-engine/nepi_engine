/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
import Toggle from "react-toggle"
import { observer, inject } from "mobx-react"

import Input from "./Input"
import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Button, { ButtonMenu } from "./Button"
import { Option } from "./Select"
import Styles from "./Styles"

import NepiSaveSelector from "./NepiSaveSelector"



import {createShortUniqueValues} from "./Utilities"

// TODO: This is redundant with the one defined in APP.js
//const IS_LOCAL = window.location.hostname === "localhost"

function roundWithSuffix(value, decimals, suffix) {
  return value && (value.toFixed(decimals) + " " + suffix)
}

@inject("ros")
@observer
class NepiDashboardData extends Component {
  constructor(props) {
    super(props)

    this.state = {
      saveSettingsFilePrefix: "",
      currDeviceId: "",
      allowFileDeletion: false,
      saveFreq: this.props.ros.saveFreqHz,

      viewableSaves: false,
      selected_save: 'NONE'

    }


    this.renderSystemStatus = this.renderSystemStatus.bind(this)
    this.renderSaveData = this.renderSaveData.bind(this)
    this.renderDeleteData = this.renderDeleteData.bind(this)

    this.onUpdateSaveFreqText = this.onUpdateSaveFreqText.bind(this)
    this.onKeySaveFreqText = this.onKeySaveFreqText.bind(this);
    this.onUpdateSaveSettingFilePrefix = this.onUpdateSaveSettingFilePrefix.bind(this)
    this.onKeySaveSettingFilePrefix = this.onKeySaveSettingFilePrefix.bind(this)
    this.onToggleDataDeletion = this.onToggleDataDeletion.bind(this)
    this.onToggleSaveData = this.onToggleSaveData.bind(this)
    this.onToggleSaveUTC = this.onToggleSaveUTC.bind(this)

    
  }




  renderSystemStatus() {
    const {
      systemStatusDiskUsageMB,
      systemDefsDiskCapacityMB,
      diskUsagePercent
    } = this.props.ros

    return (
      <Section title={"Storage Status"}>
        <Label title={"Storage"}>
          <Input disabled value={diskUsagePercent} />
        </Label>

        <Label title={"Capacity"}>
          <Input disabled value={roundWithSuffix(systemDefsDiskCapacityMB / 1000.0, 1, "GB")} />
        </Label>

        <Label title={"Used"}>
          <Input disabled value={roundWithSuffix(systemStatusDiskUsageMB / 1000.0, 1, "GB")} />
        </Label>

      </Section>
    )
  }


  onUpdateSaveFreqText(e) {
    this.setState({saveFreq: e.target.value})
    document.getElementById(e.target.id).style.color = Styles.vars.colors.red
  }

  onKeySaveFreqText(e) {
    const {onChangeSaveFreq} = this.props.ros
    if(e.key === 'Enter'){
      onChangeSaveFreq(this.state.saveFreq)
      document.getElementById(e.target.id).style.color = Styles.vars.colors.black
    }
  }

  onUpdateSaveSettingFilePrefix(e) {
    this.setState({ saveSettingsFilePrefix: e.target.value })
    document.getElementById("file_prefix_input").style.color = Styles.vars.colors.red
  }

  onKeySaveSettingFilePrefix(e) {
    const {saveSettingsFilePrefix} = this.props.ros
    if(e.key === 'Enter'){
      saveSettingsFilePrefix({newFilePrefix: this.state.saveSettingsFilePrefix})
      document.getElementById("file_prefix_input").style.color = Styles.vars.colors.black
    }
  }

  onToggleDataDeletion(e) {
    this.setState({ allowFileDeletion: e.target.checked})
  }

  onToggleSaveData(e){
    const {onChangeSaveFreqAll, onToggleSaveDataAll} = this.props.ros
    const checked = e.target.checked
    onToggleSaveDataAll(checked)
  }

  onToggleSaveUTC(e){
    const {onToggleSaveUTCAll} = this.props.ros
    const checked = e.target.checked
    onToggleSaveUTCAll(checked)
  }

  renderSaveData() {
    const { systemStatusDiskRate } = this.props.ros
    return (
      <Section title={"Save Data"}>
        <Label title={"Save All Data (Save all enabled data products)"}>
          <Toggle id={"toggle_save_data"} onClick={this.onToggleSaveData} />
        </Label>
        <Label title={"Save Freq. (Hz)"}>
          <Input id="saveFreqInput" value={this.state.saveFreq} onChange={this.onUpdateSaveFreqText} onKeyDown= {this.onKeySaveFreqText} />
        </Label>
        <Label title={"Data Rate"}>
          <Input disabled value={roundWithSuffix(systemStatusDiskRate, 3, "MB/s")} />
        </Label>
        <Label title={"File Name Prefix"}>
          <Input
            id={"file_prefix_input"}
            value={this.state.saveSettingsFilePrefix}
            onChange={this.onUpdateSaveSettingFilePrefix}
            onKeyDown={this.onKeySaveSettingFilePrefix}
          />
        </Label>
        <Label title={"Use UTC Timezone"}>
          <Toggle id={"toggle_utc"} onClick={this.onToggleSaveUTC} />
        </Label>
      </Section>
    )
  }


  renderDeleteData() {
    const { deleteAllData } = this.props.ros
    return (
      <Section title={"Delete Data"}>
        <Label title={"Allow Data Deletion"}>
          <Toggle onClick={this.onToggleDataDeletion} />
        </Label>
        <ButtonMenu>
          <Button
            onClick={deleteAllData}
            hidden={!this.state.allowFileDeletion}>
            {"Delete All Data"}
          </Button>
        </ButtonMenu>

        <pre style={{ height: "34px", overflowY: "auto" }}>
            {""}
          </pre>

        <div hidden={(this.state.allowFileDeletion)}>
        <pre style={{ height: "19px", overflowY: "auto" }}>
            {""}
          </pre>
          </div>
      </Section>
    )
  }

  getAllNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var allNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      allNamespace = "/" + namespacePrefix + "/" + deviceId + "/save_data_status"
    }
    return allNamespace
  }

  createSaveDataOptions() {
    const allNamespace = this.getAllNamespace()
    var items = []
    //items.push(<Option value={"All"}>{"All"}</Option>)
    items.push(<Option value={"None"}>{"None"}</Option>)
    items.push(<Option value={allNamespace}>{'All'}</Option>)
    const saveData_topics = this.props.ros.saveDataTopics
    const shortnames = createShortUniqueValues(saveData_topics)
    var topic = ""
    for (var i = 0; i < saveData_topics.length; i++) {
      topic = saveData_topics[i]
      if (topic !== allNamespace && topic.indexOf("None") === -1) {
        items.push(<Option value={topic}>{shortnames[i]}</Option>)
      }
    }
    return items    
  }




  //disabled={document.getElementById("toggle_save_data").value}>

  render() {

    return (
      <React.Fragment>
      <Columns>
        <Column>
        {this.renderSystemStatus()}
        </Column>
        <Column>
          {this.renderSaveData()}
        </Column>
        <Column>
        {this.renderDeleteData()}
        </Column>
      </Columns>

      <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

      <NepiSaveSelector
        title={"NepiSaveSelector.js"}
        />

    </React.Fragment>


    )
  }
}

export default NepiDashboardData
