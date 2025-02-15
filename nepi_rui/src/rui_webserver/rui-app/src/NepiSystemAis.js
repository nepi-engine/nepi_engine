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


@inject("ros")
@observer

class AisMgr extends Component {
  constructor(props) {
    super(props)

    this.state = {

      mgrName: "ai_detector_mgr",
      mgrNamespace: null,

      viewable_frameworks: false,

      frameworks_list: [],
      last_frameworks_list: [],
      frameworks_active_list: [],
      framework_name: 'NONE',
      framework_active_state: false,
      selected_framework: 'NONE',
    
      viewable_models: false, 

      models_list: [],
      models_active_list: [],
      model_name: 'NONE',
      model_active_state: false,
      selected_model: 'NONE',

      connected: false,

      aiMgrListener: null,
      needs_update: false

    }


    this.getMgrNamespace = this.getMgrNamespace.bind(this)


    this.toggleViewableFrameworks = this.toggleViewableFrameworks.bind(this)
    this.getFrameworkOptions = this.getFrameworkOptions.bind(this)
    this.onToggleFrameworkSelection = this.onToggleFrameworkSelection.bind(this)
    this.getDisabledFrameworkStr = this.getDisabledFrameworkStr.bind(this)
    this.getActiveFrameworkStr = this.getActiveFrameworkStr.bind(this)

    this.toggleViewableModels = this.toggleViewableModels.bind(this)
    this.getModelOptions = this.getModelOptions.bind(this)
    this.onToggleModelSelection = this.onToggleModelSelection.bind(this)
    this.getDisabledModelStr = this.getDisabledModelStr.bind(this)
    this.getActiveModelStr = this.getActiveModelStr.bind(this)



    this.updateAiMgrStatusListener = this.updateAiMgrStatusListener.bind(this)
    this.aiMgrStatusListener = this.aiMgrStatusListener.bind(this)



  
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
  aiMgrStatusListener(message) {
    this.setState({
      frameworks_list: message.ai_frameworks,
      frameworks_active_list: message.active_ai_frameworks,
      models_list: message.ai_models,
      models_active_list: message.active_ai_models,
      classifier_namespace_list: message.classifier_namespace_list,
      active_classifier: message.active_classifier,
      connected: true
    })    
  }

  // Function for configuring and subscribing to Status
  updateAiMgrStatusListener() {
    const statusNamespace = this.getMgrNamespace() + '/status'
    if (this.state.aiMgrListener) {
      this.state.aiMgrListener.unsubscribe()
    }
    var aiMgrListener = this.props.ros.setupStatusListener(
          statusNamespace,
          "nepi_ros_interfaces/AiMgrStatus",
          this.aiMgrStatusListener
        )
    this.setState({ aiMgrListener: aiMgrListener,
      needs_update: false})
  }

  componentDidMount(){
    this.setState({needs_update: true})
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
          mgrNamespace: namespace
        })
        this.updateAiMgrStatusListener()
      } 
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.aiMgrListener) {
      this.state.aiMgrListener.unsubscribe()
    }
  }

  toggleViewableFrameworks() {
    const set = !this.state.viewable_frameworks
    this.setState({viewable_frameworks: set})
  }

  // Function for creating image topic options.
  getFrameworkOptions() {
    const frameworksList = this.state.frameworks_list  
    var items = []
    items.push(<Option>{"NONE"}</Option>) 
    if (frameworksList.length > 0){
      for (var i = 0; i < frameworksList.length; i++) {
          items.push(<Option value={frameworksList[i]}>{frameworksList[i]}</Option>)
     }
    }
    else{
      items.push(<Option value={'NONE'}>{'NONE'}</Option>)
      //items.push(<Option value={'TEST1'}>{'TEST1'}</Option>)
      //items.push(<Option value={'TEST2'}>{'TEST2'}</Option>)
    }
    return items
  }


  onToggleFrameworkSelection(event){
    const framework_name = event.target.value
    this.setState({selected_framework: framework_name})
  }

  getActiveFrameworkStr(){
    const active =  this.state.frameworks_active_list
    var config_str_list = []
    for (var i = 0; i < active.length; i++) {
      config_str_list.push(active[i])
      config_str_list.push("\n")
    }
    const config_str =config_str_list.join("")
    return config_str
  }

  
  getDisabledFrameworkStr(){
    const installed = this.state.frameworks_list
    const active =  this.state.frameworks_active_list
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

 


  renderFrameworkConfig() {
    const { sendUpdateActiveStateMsg} = this.props.ros
    const selected_framework = this.state.selected_framework
    const viewable_frameworks = this.state.viewable_frameworks
    const framework_options = this.getFrameworkOptions()
    const active_framework_list = this.state.frameworks_active_list
    const framework_state = this.state.frameworks_active_list.indexOf(this.state.selected_framework) !== -1 

    return (

      <Columns equalWidth={true}>
      <Column>

      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Select AI Framework"}
         </label>

          <div onClick={this.toggleViewableFrameworks} style={{backgroundColor: Styles.vars.colors.grey0}}>
            <Select style={{width: "10px"}}/>
          </div>
          <div hidden={!viewable_frameworks}>
          {framework_options.map((framework) =>
          <div onClick={this.onToggleFrameworkSelection}
            style={{
              textAlign: "center",
              padding: `${Styles.vars.spacing.xs}`,
              color: Styles.vars.colors.black,
              backgroundColor: (framework.props.value === selected_framework) ?
                Styles.vars.colors.green :
                (active_framework_list.includes(framework.props.value)) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
              cursor: "pointer",
              }}>
              <body framework-topic ={framework} style={{color: Styles.vars.colors.black}}>{framework}</body>
          </div>
          )}
          </div>


        </Column>
        <Column>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Active AI Frameworks List "}
          </label>

        <pre style={{ height: "200px", overflowY: "auto" }} align={"center"} textAlign={"center"}>
        {this.getActiveFrameworkStr()}
        </pre>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Disabled AI Frameworks List "}
          </label>

        <pre style={{ height: "200px", overflowY: "auto" }} align={"center"} textAlign={"center"}>
        {this.getDisabledFrameworkStr()}
        </pre>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


        </Column>
        <Column>



        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {this.state.framework_name}
          </label>

          <div hidden={(this.state.selected_framework === "NONE")}>

        <Label title="Enable AI Framework"> </Label>
          <Toggle
            checked={framework_state }
            onClick={() => sendUpdateActiveStateMsg(this.state.mgrNamespace + "/update_framework_state", this.state.selected_framework, !framework_state)}>
        </Toggle>

          </div>

        <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/enable_all_frameworks")}>{"Enable All"}</Button>
        </ButtonMenu>

        <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/disable_all_frameworks")}>{"Disable All"}</Button>
        </ButtonMenu>

        </Column>
        </Columns> 

    )
  }


  toggleViewableModels() {
    const set = !this.state.viewable_models
    this.setState({viewable_models: set})
  }

  // Function for creating image topic options.
  getModelOptions() {
    const modelsList = this.state.models_list  
    var items = []
    items.push(<Option>{"NONE"}</Option>) 
    if (modelsList.length > 0){
      for (var i = 0; i < modelsList.length; i++) {
          items.push(<Option value={modelsList[i]}>{modelsList[i]}</Option>)
     }
    }
    else{
      items.push(<Option value={'NONE'}>{'NONE'}</Option>)
      //items.push(<Option value={'TEST1'}>{'TEST1'}</Option>)
      //items.push(<Option value={'TEST2'}>{'TEST2'}</Option>)
    }
    return items
  }


  onToggleModelSelection(event){
    const model_name = event.target.value
    this.setState({selected_model: model_name})
  }


  getActiveModelStr(){
    const active =  this.state.models_active_list
    var config_str_list = []
    for (var i = 0; i < active.length; i++) {
      config_str_list.push(active[i])
      config_str_list.push("\n")
    }
    const config_str =config_str_list.join("")
    return config_str
  }

  
  getDisabledModelStr(){
    const installed = this.state.models_list
    const active =  this.state.models_active_list
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

 


  renderModelConfig() {
    const { sendUpdateActiveStateMsg} = this.props.ros
    const selected_model = this.state.selected_model
    const viewable_models = this.state.viewable_models
    const model_options = this.getModelOptions()
    const active_model_list = this.state.models_active_list
    const model_state = this.state.models_active_list.indexOf(this.state.selected_model) !== -1 



    return (

      <Columns equalWidth={true}>
      <Column>

      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Select AI Framework"}
         </label>

          <div onClick={this.toggleViewableModels} style={{backgroundColor: Styles.vars.colors.grey0}}>
            <Select style={{width: "10px"}}/>
          </div>
          <div hidden={!viewable_models}>
          {model_options.map((model) =>
          <div onClick={this.onToggleModelSelection}
            style={{
              textAlign: "center",
              padding: `${Styles.vars.spacing.xs}`,
              color: Styles.vars.colors.black,
              backgroundColor: (model.props.value === selected_model) ?
                Styles.vars.colors.green :
                (active_model_list.includes(model.props.value)) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
              cursor: "pointer",
              }}>
              <body model-topic ={model} style={{color: Styles.vars.colors.black}}>{model}</body>
          </div>
          )}
          </div>


        </Column>
        <Column>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Active Models List "}
          </label>

        <pre style={{ height: "200px", overflowY: "auto" }} align={"center"} textAlign={"center"}>
        {this.getActiveModelStr()}
        </pre>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Disabled Models List "}
          </label>

        <pre style={{ height: "200px", overflowY: "auto" }} align={"center"} textAlign={"center"}>
        {this.getDisabledModelStr()}
        </pre>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


        </Column>
        <Column>


        <div hidden={(this.state.selected_model === "NONE")}>

        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {this.state.model_name}
          </label>

        <Label title="Enable"> </Label>
          <Toggle
            checked={model_state===true}
            onClick={() => sendUpdateActiveStateMsg(this.state.mgrNamespace + "/update_model_state", this.state.selected_model, !model_state)}>
        </Toggle>

        </div>

        <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/enable_all_models")}>{"Enable All"}</Button>
        </ButtonMenu>

        <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/disable_all_models")}>{"Disable All"}</Button>
        </ButtonMenu>


        </Column>
        </Columns> 

    )
  }



render() {

    return (

    <Section title={"AI Framework and Model Settings"}>
       
       <Columns>
      <Column>

       <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/refresh_frameworks")}>{"Refresh"}</Button>
        </ButtonMenu>

        </Column>
        <Column>

        <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/factory_reset")}>{"Factory Reset"}</Button>
        </ButtonMenu>

        </Column>
     </Columns>

 
     <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

      <Columns>
      <Column>

      {this.renderFrameworkConfig()}
      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
      </Column>
        <Column>

      {this.renderModelConfig()}
      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

       </Column>
     </Columns>
         
    </Section>
          

    )
  }

}


export default AisMgr
