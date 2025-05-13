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
import {Link} from "react-router-dom"
import Toggle from "react-toggle"
import { displayNameFromNodeName, nodeNameFromDisplayName } from "./Store"
import Input from "./Input"
import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Button, { ButtonMenu } from "./Button"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import BooleanIndicator from "./BooleanIndicator"

function round(value, decimals = 0) {
  return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}

function styleTextEdited(text_box_element) {
  text_box_element.style.color = Styles.vars.colors.red
  text_box_element.style.fontWeight = "bold"
}

function styleTextUnedited(text_box_element) {
  text_box_element.style.color = Styles.vars.colors.black
  text_box_element.style.fontWeight = "normal"
}

const styles = Styles.Create({
  link_style: {
    color: Styles.vars.colors.blue,
    fontSize: Styles.vars.fontSize.medium,
    //lineHeight: Styles.vars.lineHeights.xl 
  }
})

@inject("ros")
@observer
class NepiSystemDevice extends Component {
  constructor(props) {
    super(props)

    this.state = {
      autoRate: this.props.ros.triggerAutoRateHz,
      autoRateUserEditing: false,
      ipAddrVal: "0.0.0.0/24",
      configSubsys: "All",
      advancedConfigDisabled: true,
      updatedDeviceId: "",
      selectedWifiNetwork: "",
      wifiClientSSIDEdited: false,
      wifiClientSSID: "",
      wifiClientPassphrase: "",
      wifiAPSSIDEdited: false,
      wifiAPSSID: "",
      wifiAPPassphrase: "",
      tx_bandwidth_limit: (this.props.ros.bandwidth_usage_query_response !== null)? this.props.ros.bandwidth_usage_query_response.tx_limit_mbps : -1,
      tx_bandwidth_user_editing: false
    }

    this.onUpdateAutoRateText = this.onUpdateAutoRateText.bind(this)
    this.onKeyAutoRateText = this.onKeyAutoRateText.bind(this)
    this.onUpdateTXRateLimitText = this.onUpdateTXRateLimitText.bind(this)
    this.onKeyTXRateLimitText = this.onKeyTXRateLimitText.bind(this)

    this.onIPAddrValChange = this.onIPAddrValChange.bind(this)
    this.onAddButtonPressed = this.onAddButtonPressed.bind(this)
    this.onRemoveButtonPressed = this.onRemoveButtonPressed.bind(this)
    this.onSaveCfg = this.onSaveCfg.bind(this)
    this.onUserReset = this.onUserReset.bind(this)
    this.onFactoryReset = this.onFactoryReset.bind(this)
    this.onConfigSubsysSelected = this.onConfigSubsysSelected.bind(this)
    this.onToggleAdvancedConfig = this.onToggleAdvancedConfig.bind(this)
    this.createConfigSubsysOptions = this.createConfigSubsysOptions.bind(this)
    this.createWifiNetworkOptions = this.createWifiNetworkOptions.bind(this)
    this.onWifiNetworkSelected = this.onWifiNetworkSelected.bind(this)
    this.onUpdateClientSSIDText = this.onUpdateClientSSIDText.bind(this)
    this.onUpdateClientPassphraseText = this.onUpdateClientPassphraseText.bind(this)
    this.onKeyClientWifi = this.onKeyClientWifi.bind(this)
    this.onUpdateAPSSIDText = this.onUpdateAPSSIDText.bind(this)
    this.onUpdateAPPassphraseText = this.onUpdateAPPassphraseText.bind(this)
    this.onKeyAPWifi = this.onKeyAPWifi.bind(this)

    this.onDeviceIdChange = this.onDeviceIdChange.bind(this)
    this.onDeviceIdKey = this.onDeviceIdKey.bind(this)

    this.renderDeviceSettings = this.renderDeviceSettings.bind(this)
    this.renderLicense = this.renderLicense.bind(this)
    this.renderLicenseRequestInfo = this.renderLicenseRequestInfo.bind(this)
    this.renderLicenseInfo = this.renderLicenseInfo.bind(this)
    this.renderNetworkInfo = this.renderNetworkInfo.bind(this)
    this.renderConfiguration = this.renderConfiguration.bind(this)
    this.renderTriggerSettings = this.renderTriggerSettings.bind(this)
  }

  onUpdateAutoRateText(e) {
    this.setState({autoRate: e.target.value});
    this.setState({autoRateUserEditing: true});
    styleTextEdited(document.getElementById(e.target.id))
  }

  onKeyAutoRateText(e) {
    const {onChangeTriggerRate, } = this.props.ros
    if(e.key === 'Enter'){
      this.setState({autoRateUserEditing: false});
      onChangeTriggerRate(this.state.autoRate)
      styleTextUnedited(document.getElementById(e.target.id))
    }
  }

  onUpdateClientSSIDText(e) {
    this.setState({wifiClientSSID: e.target.value, wifiClientSSIDEdited: true});
    var client_ssid_textbox = document.getElementById("wifi_client_ssid_textbox")
    styleTextEdited(client_ssid_textbox)
    
    var client_passphrase_textbox = document.getElementById("wifi_client_passphrase_textbox")
    styleTextEdited(client_passphrase_textbox)
  }

  onUpdateClientPassphraseText(e) {
    this.setState({wifiClientPassphrase: e.target.value, wifiClientSSIDEdited: true});
    var client_ssid_textbox = document.getElementById("wifi_client_ssid_textbox")
    styleTextEdited(client_ssid_textbox)
    var client_passphrase_textbox = document.getElementById("wifi_client_passphrase_textbox")
    styleTextEdited(client_passphrase_textbox)
  }

  onKeyClientWifi(e) {
    const {onUpdateWifiClientCredentials} = this.props.ros
    if(e.key === 'Enter'){
      this.setState({wifiClientSSIDEdited: false})
      onUpdateWifiClientCredentials(this.state.wifiClientSSID, this.state.wifiClientPassphrase)
      // Reset style
      var client_ssid_textbox = document.getElementById("wifi_client_ssid_textbox")
      styleTextUnedited(client_ssid_textbox)
      var client_passphrase_textbox = document.getElementById("wifi_client_passphrase_textbox")
      styleTextUnedited(client_passphrase_textbox)
    }
  }

  onUpdateAPSSIDText(e) {
    this.setState({wifiAPSSID: e.target.value, wifiAPSSIDEdited: true});
    var ap_ssid_textbox = document.getElementById("wifi_ap_ssid_textbox")
    styleTextEdited(ap_ssid_textbox)
    var ap_passphrase_textbox = document.getElementById("wifi_ap_passphrase_textbox")
    styleTextEdited(ap_passphrase_textbox)
  }

  onUpdateAPPassphraseText(e) {
    this.setState({wifiAPPassphrase: e.target.value, wifiAPSSIDEdited: true});
    var ap_ssid_textbox = document.getElementById("wifi_ap_ssid_textbox")
    styleTextEdited(ap_ssid_textbox)
    var ap_passphrase_textbox = document.getElementById("wifi_ap_passphrase_textbox")
    styleTextEdited(ap_passphrase_textbox)
  }

  onKeyAPWifi(e) {
    const {onUpdateWifiAPCredentials} = this.props.ros
    if(e.key === 'Enter'){
      this.setState({wifiAPSSIDEdited: false})
      onUpdateWifiAPCredentials(this.state.wifiAPSSID, this.state.wifiAPPassphrase)
      // Reset style
      var ap_ssid_textbox = document.getElementById("wifi_ap_ssid_textbox")
      styleTextUnedited(ap_ssid_textbox)
      var ap_passphrase_textbox = document.getElementById("wifi_ap_passphrase_textbox")
      styleTextUnedited(ap_passphrase_textbox)
    }
  }

  onUpdateTXRateLimitText(e) {
    this.setState({tx_bandwidth_limit: e.target.value});
    this.setState({tx_bandwidth_user_editing: true});
    var rate_limit_textbox = document.getElementById(e.target.id)
    styleTextEdited(rate_limit_textbox)
  }

  onKeyTXRateLimitText(e) {
    const {onChangeTXRateLimit} = this.props.ros
    if(e.key === 'Enter'){
      this.setState({tx_bandwidth_user_editing: false});
      onChangeTXRateLimit(this.state.tx_bandwidth_limit)
      var rate_limit_textbox = document.getElementById(e.target.id)
      styleTextUnedited(rate_limit_textbox)
    }
  }

  async onIPAddrValChange(e) {
    await this.setState({ipAddrVal: e.target.value})
  }

  async onAddButtonPressed() {
    const { addIPAddr } = this.props.ros
    const { ipAddrVal } = this.state

    addIPAddr(ipAddrVal)
  }

  async onRemoveButtonPressed() {
    const { removeIPAddr } = this.props.ros
    const { ipAddrVal } = this.state

    removeIPAddr(ipAddrVal)
  }

  async onDeviceIdChange(e) {
    this.setState({ updatedDeviceId: e.target.value })
    var device_id_textbox = document.getElementById(e.target.id)
    styleTextEdited(device_id_textbox)
  }

  async onDeviceIdKey(e) {
    const {setDeviceID} = this.props.ros
    if(e.key === 'Enter'){
      setDeviceID({newDeviceID: this.state.updatedDeviceId})
      var device_id_textbox = document.getElementById(e.target.id)
      styleTextUnedited(device_id_textbox)
    }
  }

  renderDeviceSettings() {
    const {deviceId} = this.props.ros
    return (
      <Section title={"Device"}>
        <Label title={this.state.advancedConfigDisabled? "Device ID" : "Updated Device ID"}>
          <Input
            id={"device_id_update_text"}
            value={this.state.advancedConfigDisabled? deviceId : this.state.updatedDeviceId }
            disabled={this.state.advancedConfigDisabled}
            onChange={this.onDeviceIdChange}
            onKeyDown={this.onDeviceIdKey}
          />
        </Label>
      </Section>
    )
  }

  renderLicense() {
    const {license_info} = this.props.ros

    const license_info_valid = license_info && ("licensed_components" in license_info) && ("nepi_base" in license_info["licensed_components"]) &&
      "commercial_license_type" in license_info["licensed_components"]["nepi_base"]

    const license_issue_date = license_info_valid && "issued_date" in license_info["licensed_components"]["nepi_base"]?
    license_info["licensed_components"]["nepi_base"]["issued_date"] : ""

    const license_issue_version = license_info_valid && "issued_version" in license_info["licensed_components"]["nepi_base"]?
      license_info["licensed_components"]["nepi_base"]["issued_version"] : ""

    const license_expiration_date = license_info_valid && "expiration_date" in license_info["licensed_components"]["nepi_base"]?
      license_info["licensed_components"]["nepi_base"]["expiration_date"] : null

    const license_expiration_version = license_info_valid && "expiration_version" in license_info["licensed_components"]["nepi_base"]?
      license_info["licensed_components"]["nepi_base"]["expiration_version"] : null
    
    return (
      <div>
        <Label title={"Issue Date"}>
          <Input value={license_issue_date} disabled={true}/>
        </Label>
        <Label title={"Issue Version"}>
          <Input value={license_issue_version} disabled={true}/>
        </Label>
        {license_expiration_date?
          <Label title={"Expiration Date"}>
            <Input value={license_expiration_date} disabled={true}/>
          </Label>
          : null
        }
        {license_expiration_version?
          <Label title={"Expiration Version"}>
            <Input value={license_expiration_version} disabled={true}/>
          </Label>
          : null
        }
      </div>
    )
  }

  renderLicenseRequestInfo() {
    const { license_request_info } = this.props.ros

    const license_request_info_valid = license_request_info && ('license_request' in license_request_info)
    const license_hw_key = (license_request_info_valid && ('hardware_key' in license_request_info['license_request']))?
      license_request_info['license_request']['hardware_key'] : 'Unknown'
    const license_request_date = (license_request_info_valid && ('date' in license_request_info['license_request']))?
      license_request_info['license_request']['date'] : 'Unknown'    
    const license_request_version = (license_request_info_valid && ('version' in license_request_info['license_request']))?
      license_request_info['license_request']['version'] : 'Unknown'    

    return (
      // TODO: A QR code or automatic API link would be nicer here.
      <div>
        <Label title={"H/W Key"}>
          <Input value={license_hw_key} disabled={true}/>
        </Label>
        <Label title={"Date"}>
          <Input value={license_request_date} disabled={true}/>
        </Label>
        <Label title={"Version"}>
          <Input value={license_request_version} disabled={true}/>
        </Label>
      </div>
    )
  }

  renderLicenseInfo() {
    const {license_info, commercial_licensed, license_request_mode, onGenerateLicenseRequest} = this.props.ros

    const license_info_valid = license_info && ("licensed_components" in license_info) && ("nepi_base" in license_info["licensed_components"]) &&
                               "commercial_license_type" in license_info["licensed_components"]["nepi_base"] &&
                               "status" in license_info["licensed_components"]["nepi_base"]
    
    var license_type = license_info_valid? license_info["licensed_components"]["nepi_base"]["commercial_license_type"] : "Unlicensed"
    var license_status = license_info_valid? license_info["licensed_components"]["nepi_base"]["status"] : ""
    if (license_request_mode === true) {
      license_type = "Request"
      license_status = "Pending"
    }

    return (
      <Section title={"NEPI License"}>
        <Label title={"Type"}>
          <Input value={license_type} disabled={true}/>
        </Label>

        <div hidden={license_type === "Unlicensed"}> 
        <pre style={{ height: "88px", overflowY: "auto" }}>
            {"No Commercial License Found. Valid for development purposes only"}
          </pre>
        </div>

        {license_info_valid?
          <Label title={"Status"} >
            <Input value={license_status} disabled={true}/>
          </Label>
          : null
        }

        {license_info_valid && license_request_mode?
          this.renderLicenseRequestInfo() : null
        }

        {license_info_valid && !license_request_mode?
          this.renderLicense() : null
        }
                         
        {(license_info_valid && !commercial_licensed)?
          <ButtonMenu>
            <Button onClick={onGenerateLicenseRequest}>{"License Request"}</Button>
          </ButtonMenu>
          : null
        }
                  
        {(license_info_valid && !commercial_licensed)?
            <div style={{textAlign: "center"}}>
              <Link to={{ pathname: "commercial_license_request_instructions.html" }} target="_blank" style={styles.link_style}>
                Open license request instructions
              </Link>
            </div>
            : null
        }
      </Section>
    )
  }

  renderTriggerSettings() {
    const {
      //triggerMask,
      onPressManualTrigger,
      //onToggleHWTriggerOutputEnabled,
      //onToggleHWTriggerInputEnabled,
      triggerAutoRateHz
    } = this.props.ros

    return (
      <Section title={"Trigger Settings"}>
        <Label title={"Auto Rate (Hz)"}>
          <Input
            id="autoRateInput"
            value={(this.state.autoRateUserEditing === true)? this.state.autoRate : triggerAutoRateHz}
            onChange={this.onUpdateAutoRateText} onKeyDown={this.onKeyAutoRateText}
          />
        </Label>
        <ButtonMenu>
          <Button onClick={onPressManualTrigger}>{"Manual Trigger"}</Button>
        </ButtonMenu>
        {/*
        <Label title={"Hardware Trigger Input Enable"}>
          <Toggle
            checked={false}
            disabled={true}
          />
        </Label>
        <Label title={"Hardware Trigger Output Enable"}>
          <Toggle
            checked={true}
            disabled={true}
          />
        </Label>
        */}
      </Section>
    )
  }

  createWifiNetworkOptions(wifiNetworks) {
    var network_options = []
    network_options.push(<Option>{""}</Option>)
    for (var i = 0; i < wifiNetworks.length; i++) {
      network_options.push(<Option>{wifiNetworks[i]}</Option>)
    }

    return network_options
  }

  async onWifiNetworkSelected(e) {
    var ssid_textbox = document.getElementById("wifi_client_ssid_textbox")
    var passphrase_textbox = document.getElementById("wifi_client_passphrase_textbox")
    if (e.target.value !== "") {
      ssid_textbox.style.color = Styles.vars.colors.red
      ssid_textbox.style.fontWeight = "bold"
      passphrase_textbox.style.color = Styles.vars.colors.red
      passphrase_textbox.style.fontWeight = "bold"
    }
    else {
      ssid_textbox.style.color = Styles.vars.colors.black
      ssid_textbox.style.fontWeight = "normal"
      passphrase_textbox.style.color = Styles.vars.colors.black
      passphrase_textbox.style.fontWeight = "normal"  
    }

    await this.setState({
      selectedWifiNetwork: e.target.value,
      wifiClientSSIDEdited: (e.target.value !== "")? true : false, 
      wifiClientSSID: e.target.value, 
      wifiClientPassphrase: ""
    })
  }

  renderNetworkInfo() {
    const { systemInContainer, sendTriggerMsg, ip_query_response, onToggleDHCPEnabled, bandwidth_usage_query_response } = this.props.ros
    const { ipAddrVal } = this.state
    
    return (
      <Section title={"Ethernet"}>

        <div hidden={systemInContainer === false}> 

          <pre style={{ height: "88px", overflowY: "auto" }}>
            {"NEPI Running in Container Mode.  Ethernet configuration set by host system"}
          </pre>
        </div>

        <div hidden={systemInContainer === true}>  

        <Label title={"Device IP Addresses"}>
          <pre style={{ height: "88px", overflowY: "auto" }}>
            {(ip_query_response !== null)? ip_query_response.ip_addrs.join('\n') : ""}
          </pre>
        </Label>
        <Label title={"DHCP Enabled"}>
          <Toggle
            checked={(ip_query_response !== null)? ip_query_response.dhcp_enabled : false}
            onClick= {onToggleDHCPEnabled}
          />
        </Label>


        <Label>
          <Input value={ipAddrVal} onChange={ this.onIPAddrValChange} />
        </Label>
        <ButtonMenu>
          <Button onClick={this.onAddButtonPressed}>{"Add"}</Button>
          <Button onClick={this.onRemoveButtonPressed}>{"Remove"}</Button>
        </ButtonMenu>



        <Label title={"TX Data Rate (Mbps)"}>
          <Input disabled value={(bandwidth_usage_query_response !== null)? round(bandwidth_usage_query_response.tx_rate_mbps, 2) : -1.0} />
        </Label>
        <Label title={"RX Data Rate (Mbps)"}>
          <Input disabled value={(bandwidth_usage_query_response !== null)? round(bandwidth_usage_query_response.rx_rate_mbps, 2) : -1.0} />
        </Label>
        <Label title={"TX Rate Limit (Mbps)"}>
          <Input
            id="txRateLimit"
            value={((this.state.tx_bandwidth_user_editing === true) || (bandwidth_usage_query_response === null))?
              this.state.tx_bandwidth_limit : bandwidth_usage_query_response.tx_limit_mbps}
            onChange={this.onUpdateTXRateLimitText}
            onKeyDown={this.onKeyTXRateLimitText}
          />
        </Label>


       </div>

      </Section>
    )
  }

  renderWifiInfo() {
    const { systemInContainer, wifi_query_response, onToggleWifiAPEnabled, onToggleWifiClientEnabled, onRefreshWifiNetworks } = this.props.ros
    const { wifiClientSSIDEdited, wifiClientSSID, wifiClientPassphrase,
            wifiAPSSIDEdited, wifiAPSSID, wifiAPPassphrase } = this.state
    const wifi_enabled = (wifi_query_response !== null)? wifi_query_response.wifi_client_enabled : false
    const client_ssid = (wifi_query_response !== null)? wifi_query_response.wifi_client_ssid : ""
    const client_passphrase = (wifi_query_response !== null)? wifi_query_response.wifi_client_passphrase : ""
    const ap_ssid = (wifi_query_response !== null)? wifi_query_response.wifi_ap_ssid : ""
    const ap_passphrase = (wifi_query_response !== null)? wifi_query_response.wifi_ap_passphrase : ""
    const available_networks = (wifi_query_response !== null)? wifi_query_response.available_networks : []
    
    return (
      <Section title={"WiFi"}>
        <div hidden={systemInContainer === false}> 

        <pre style={{ height: "88px", overflowY: "auto" }}>
          {"NEPI Running in Container Mode.  WiFi configuration set by host system"}
        </pre>
      </div>
      <div hidden={systemInContainer === true}> 
        <Columns>
          <Column>
            <Label title={"Client Enabled"} marginTop={Styles.vars.spacing.medium}>
              <Toggle
                checked={wifi_enabled}
                onClick= {onToggleWifiClientEnabled}
              />
            </Label>
          </Column>
          <Column>
            <Label title={"Connected"}>
              <BooleanIndicator value={(wifi_query_response !== null)? wifi_query_response.wifi_client_connected : false} />
            </Label>
          </Column>
        </Columns>
        <div hidden={!wifi_enabled}>
        <Columns>
          <Column>
            <Label title={"Network"} >
              <Input 
                id={"wifi_client_ssid_textbox"}
                value={(wifiClientSSIDEdited === true)? wifiClientSSID : client_ssid}
                onChange={this.onUpdateClientSSIDText} onKeyDown={this.onKeyClientWifi}
              />
            </Label>
            <Label title={"Avail. Networks"} >
              <Select
                onChange={this.onWifiNetworkSelected}
                value={this.state.selectedWifiNetwork}
              >
                {this.createWifiNetworkOptions(available_networks)}
              </Select>
            </Label>
          </Column>
          <Column>
            <Label title={"Passphrase"} >
              <Input 
                id={"wifi_client_passphrase_textbox"}
                type={"password"}
                value={(wifiClientSSIDEdited === true)? wifiClientPassphrase : client_passphrase}
                onChange={this.onUpdateClientPassphraseText} onKeyDown={this.onKeyClientWifi}
              />
            </Label>
            <ButtonMenu>
              <Button onClick={onRefreshWifiNetworks}>{"Refresh"}</Button>
            </ButtonMenu>
          </Column>
        </Columns>
        </div>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <Columns>
          <Column>
            <Label title={"Access Point Enabled"} >
              <Toggle
                checked={(wifi_query_response !== null)? wifi_query_response.wifi_ap_enabled : false}
                onClick= {onToggleWifiAPEnabled}
              />
            </Label>
          </Column>
          <Column/>
        </Columns>
        <Columns>
          <Column>
            <Label title={"Access Point"} >
              <Input
                id={"wifi_ap_ssid_textbox"} 
                value={(wifiAPSSIDEdited === true)? wifiAPSSID : ap_ssid}
                onChange={this.onUpdateAPSSIDText} onKeyDown={this.onKeyAPWifi}
              />
            </Label>
          </Column>
          <Column>
            <Label title={"Passphrase"} >
              <Input
              id={"wifi_ap_passphrase_textbox"}                 
                value={(wifiAPSSIDEdited === true)? wifiAPPassphrase : ap_passphrase}
                onChange={this.onUpdateAPPassphraseText} onKeyDown={this.onKeyAPWifi}
              />
            </Label>
          </Column>
        </Columns>
        </div>

      </Section>
    )
  }

  async onSaveCfg() {
    const { saveCfg } = this.props.ros
    var node_name = this.state.configSubsys
    if (node_name !== 'UNKNOWN_NODE') {
      saveCfg({baseTopic: node_name})
    }
  }

  async onUserReset() {
    const { systemReset } = this.props.ros
    var node_name = this.state.configSubsys
    if (node_name !== 'UNKNOWN_NODE') {
      systemReset(node_name, 0) // Value 1 per Reset.msg
    }
  }

  async onFactoryReset() {
    const { systemReset } = this.props.ros
    var node_name = this.state.configSubsys
    if (node_name !== 'UNKNOWN_NODE') {
      systemReset(node_name, 1) // Value 1 per Reset.msg
    }
  }

  async onSoftwareReset() {
    const { systemReset } = this.props.ros
    var node_name = this.state.configSubsys
    if (node_name !== 'UNKNOWN_NODE') {
      systemReset(node_name, 2) // Value 1 per Reset.msg
    }
  }

  async onHardwareReset() {
    const { systemReset } = this.props.ros
    var node_name = this.state.configSubsys
    if (node_name !== 'UNKNOWN_NODE') {
      systemReset(node_name, 3) // Value 1 per Reset.msg
    }
  }

  async onConfigSubsysSelected(e) {
    await this.setState({configSubsys: e.target.value})
  }

  async onToggleAdvancedConfig() {
    var disabled = this.state.advancedConfigDisabled
    this.setState({advancedConfigDisabled: !disabled})
  }

  createConfigSubsysOptions(resetTopics) {
    var subsys_options = []
    for (var i = 1; i < resetTopics.length; i++) { // Skip the first one -- it is global /numurus/dev_3dx/<s/n>
      var node_name = resetTopics[i].split("/").pop()
      subsys_options.push(<Option value={resetTopics[i]}>{node_name}</Option>)
    }
    return subsys_options
  }



  renderConfiguration() {
    const { resetTopics, onUserCfgRestore, onFactoryCfgRestore } = this.props.ros
    const { advancedConfigDisabled, configSubsys } = this.state

    return (
      <Section title={"Configuration"}>

        <Label title={"Subsystem"}>
          <Select
            onChange={this.onConfigSubsysSelected}
            value={configSubsys}
          >
            {this.createConfigSubsysOptions(resetTopics)}
          </Select>
        </Label>
          <ButtonMenu>
            <Button onClick={this.onSaveCfg}>{"Save"}</Button>
            <Button onClick={this.onUserReset}>{"Reset"}</Button>
            <Button onClick={this.onFactoryReset}>{"Factory Reset"}</Button>
            <Button onClick={this.onSoftwareReset}>{"Software Reset"}</Button>
            <Button onClick={this.onHardwareReset}>{"Hardware Reset"}</Button>
            <Button hidden={advancedConfigDisabled} onClick={this.onFactoryCfgRestore}>{"Full Factory Restore"}</Button>
            <Button hidden={advancedConfigDisabled} onClick={onUserCfgRestore}>{"Full User Restore"}</Button>
          </ButtonMenu>

          <Label title={"Show Advanced Settings"}>
            <Toggle
              onClick={this.onToggleAdvancedConfig}>
            </Toggle>
          </Label>
      </Section>
    )
  }

  render() {
    const { wifi_query_response } = this.props.ros
    const has_wifi = wifi_query_response? wifi_query_response.has_wifi : false

    return (
      <Columns>
        <Column>
          {this.renderDeviceSettings()}
          {this.renderLicenseInfo()}
          {/*this.renderTriggerSettings()*/}
          {this.renderConfiguration()}
        </Column>
        <Column>
          <Label title={"Internet Connected"}>
            <BooleanIndicator value={(wifi_query_response !== null)? wifi_query_response.internet_connected : false} />
          </Label>
          {this.renderNetworkInfo()}
          {has_wifi? this.renderWifiInfo(): null}
        </Column>
      </Columns>
    )
  }
}
export default NepiSystemDevice
