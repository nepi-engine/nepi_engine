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
import BooleanIndicator from "./BooleanIndicator"
import Styles from "./Styles"
import {createShortUniqueValues} from "./Utilities"


import Select, { Option } from "./Select"
import NepiMessagesSelector from "./NepiMessagesSelector"


// TODO: This is redundant with the one defined in APP.js
const IS_LOCAL = window.location.hostname === "localhost"

function roundWithSuffix(value, decimals, suffix) {
  return value && (value.toFixed(decimals) + " " + suffix)
}

@inject("ros")
@observer
class NepiDashboard extends Component {
  constructor(props) {
    super(props)

    this.state = {
      saveSettingsFilePrefix: "",
      currDeviceId: "",
      allowFileDeletion: false,
      saveFreq: this.props.ros.saveFreqHz,

            viewableMessages: false,
            selected_message: 'NONE'
    }

  
    this.renderDeviceInfo = this.renderDeviceInfo.bind(this)
    this.renderSystemClock = this.renderSystemClock.bind(this)
    this.renderSystemStatus = this.renderSystemStatus.bind(this)

    this.toggleViewableMessages = this.toggleViewableMessages.bind(this)
    this.onToggleMessagesSelection = this.onToggleMessagesSelection.bind(this)

  }

  getAllNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var allNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      allNamespace = "/" + namespacePrefix + "/" + deviceId + "/messages"
    }
    return allNamespace
  }

  createMessageOptions() {
    const allNamespace = this.getAllNamespace()
    var items = []
    //items.push(<Option value={"All"}>{"All"}</Option>)
    items.push(<Option value={"None"}>{"None"}</Option>)
    const Messages_topics = this.props.ros.messageTopics
    const shortnames = createShortUniqueValues(Messages_topics)
    var topic = ""
    for (var i = 0; i < Messages_topics.length; i++) {
      topic = Messages_topics[i]
      if (topic !== allNamespace && topic.indexOf("None") === -1) {
        items.push(<Option value={topic}>{shortnames[i]}</Option>)
      }
    }
    return items    
  }

  toggleViewableMessages() {
    const viewable = !this.state.viewableMessages
    this.setState({viewableMessages: viewable})
  }

  onToggleMessagesSelection(event){
    const selected_message = event.target.innerText
    this.setState({selected_message: selected_message})
  }

  renderSelectorMessages() {
    const messageTopics = this.createMessageOptions()
    const hide_messages_list = !this.state.viewableMessages && !this.state.connected
    return (
      <React.Fragment>
      <Columns>
      <Column>

      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Select message Topic"}
         </label>

      <div onClick={this.toggleViewableMessages} style={{backgroundColor: Styles.vars.colors.grey0}}>
      <Select style={{width: "10px"}}/>
    </div>
    <div hidden={hide_messages_list}>
    {messageTopics.map((message) =>
    <div onClick={this.onToggleMessagesSelection}
      style={{
        textAlign: "center",
        padding: `${Styles.vars.spacing.xs}`,
        color: Styles.vars.colors.black,
        backgroundColor: (message.props.value === this.state.selected_message) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
        cursor: "pointer",
        }}>
        <body message-topic ={message} style={{color: Styles.vars.colors.black}}>{message}</body>
    </div>
    )}
    </div>

    </Column>
      </Columns>

    </React.Fragment>
    )
  }

  renderDeviceInfo() {
    const {
      deviceType,
      deviceId,
      deviceSerial,
      systemDefsFirmwareVersion,
    } = this.props.ros
    return (
      <Section title={"Device Info"}>
        <Label title={"Type"}>
          <Input disabled value={deviceType} />
        </Label>
        <Label title={"Device ID"}>
          <Input disabled value={deviceId} />
        </Label>
        <Label title={"Serial Number"}>
          <Input disabled value={deviceSerial} />
        </Label>
        <Label title={"Firmware"}>
          <Input disabled value={systemDefsFirmwareVersion} />
        </Label>

        <pre style={{ height: "68px", overflowY: "auto" }}>
            {""}
          </pre>

      </Section>
    )
  }

  renderdeviceInfo() {
    const {
      deviceConnected,
      deviceType,
      deviceFirmwareVersion,
      eggFirmwareVersion,
    } = this.props.ros
    return (
      <Section title={"device Info"}>
        <Label title={"Connected"}>
          <BooleanIndicator value={deviceConnected} />
        </Label>
        <Label title={"Type"}>
          <Input disabled value={deviceType} />
        </Label>
        <Label title={"Firmware"}>
          <Input disabled value={deviceFirmwareVersion} />
        </Label>
        <Label title={"Sensor Firmware"}>
          <Input disabled value={eggFirmwareVersion} />
        </Label>
        
      </Section>
    )
  }

  renderSystemClock() {
    const {
      systemInContainer,
      systemStatusTime,
      clockUTCMode,
      clockTZ,
      onToggleClockUTCMode,
      clockNTP,
      onSyncUTCToDevice
    } = this.props.ros

    const time = systemStatusTime && systemStatusTime.format("h:mm:ss a")
    const date = systemStatusTime && systemStatusTime.format("l")

    return (
      <Section title={"System Clock"}>
        <Label title={"NTP"}>
          <BooleanIndicator value={clockNTP} />
        </Label>
        <Label title={"Time"}>
          <Input disabled value={time} />
        </Label>
        <Label title={"Date"}>
          <Input disabled value={date} />
        </Label>
        <Label title={"Timezone"}>
          <Input disabled value={clockTZ} />
        </Label>
        <Label title={"UTC"}>
          <Toggle checked={clockUTCMode} onClick={onToggleClockUTCMode} />
        </Label>
        <div hidden={systemInContainer === true}>
        {(IS_LOCAL === false) &&
        <ButtonMenu>
          <Button onClick={onSyncUTCToDevice}>{"Sync Clocks"}</Button>
        </ButtonMenu>}
        </div>




      </Section>
    )
  }

  renderSystemStatus() {
    const {
      heartbeat,
      systemStatusDiskUsageMB,
      systemStatusTempC,
      systemDefsDiskCapacityMB,
      diskUsagePercent
    } = this.props.ros

    return (
      <Section title={"System Status"}>
        <Label title={"Heartbeat"}>
          <BooleanIndicator value={heartbeat} />
        </Label>
        
        <Label title={"Temperature"}>
          <Input disabled value={roundWithSuffix(systemStatusTempC, 1, "\u00B0C")} />
        </Label>

        <Label title={"Storage"}>
          <Input disabled value={diskUsagePercent} />
        </Label>

        <Label title={"Capacity"}>
          <Input disabled value={roundWithSuffix(systemDefsDiskCapacityMB / 1000.0, 1, "GB")} />
        </Label>

        <Label title={"Used"}>
          <Input disabled value={roundWithSuffix(systemStatusDiskUsageMB / 1000.0, 1, "GB")} />
        </Label>

        <pre style={{ height: "8px", overflowY: "auto" }}>
            {""}
          </pre>

      </Section>
    )
  }


  render() {
    return (
    <React.Fragment>

      <Columns>
        <Column>
          {this.renderDeviceInfo()}



        </Column>
        <Column>
          {this.renderSystemStatus()}


        </Column>
        <Column>
        {this.renderSystemClock()}

        </Column>
      </Columns>

      <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

      <NepiMessagesSelector
        title={"NepiMessagesSelector.js"}
        />

    </React.Fragment>


    )
  }
}

export default NepiDashboard
