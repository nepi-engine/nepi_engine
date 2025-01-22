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

class AiFrameworksMgr extends Component {
  constructor(props) {
    super(props)

    this.state = {
      show_delete_ai: false,
      mgrName: "ai_detector_mgr",
      mgrNamespace: null,

      viewableAis: false,

      ais_list: [],
      last_ais_list: [],
      ais_active_list: [],
      ai_name: 'NONE',
      ai_active_state: false,
      selected_ai: 'NONE',
    
      viewableModels: false, 

      models_list: [],
      models_active_list: [],
      model_name: 'NONE',
      model_active_state: false,
      selected_model: 'NONE',

      connected: false,

      aisListener: null,
      needs_update: true

    }


    this.getMgrNamespace = this.getMgrNamespace.bind(this)


    this.toggleviewableAis = this.toggleviewableAis.bind(this)
    this.getAiOptions = this.getAiOptions.bind(this)
    this.onToggleAiSelection = this.onToggleAiSelection.bind(this)
    this.getDisabledAiStr = this.getDisabledAiStr.bind(this)
    this.getActiveAiStr = this.getActiveAiStr.bind(this)

    this.toggleviewableModels = this.toggleviewableModels.bind(this)
    this.getModelOptions = this.getModelOptions.bind(this)
    this.onToggleModelSelection = this.onToggleModelSelection.bind(this)
    this.getDisabledModelStr = this.getDisabledModelStr.bind(this)
    this.getActiveModelStr = this.getActiveModelStr.bind(this)



    this.updateAisStatusListener = this.updateAisStatusListener.bind(this)
    this.aisStatusListener = this.aisStatusListener.bind(this)



  
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
  aisStatusListener(message) {
    this.setState({
      ais_list: message.ai_frameworks,
      ais_active_list: message.active_ai_frameworks,
      models_list: message.ai_models,
      models_active_list: message.active_ai_models,
      connected: true
    })    

  }

  // Function for configuring and subscribing to Status
  updateAisStatusListener() {
    const statusNamespace = this.getMgrNamespace() + '/status'
    if (this.state.aisListener) {
      this.state.aisListener.unsubscribe()
    }
    var aisListener = this.props.ros.setupStatusListener(
          statusNamespace,
          "nepi_ros_interfaces/AiFrameworksStatus",
          this.aisStatusListener
        )
    this.setState({ aisListener: aisListener,
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
          mgrNamespace: namespace
        })
        this.updateAisStatusListener()
      } 
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.aisListener) {
      this.state.aisListener.unsubscribe()
    }
  }

  toggleviewableAis() {
    const set = !this.state.viewableAis
    this.setState({viewableAis: set})
  }

  // Function for creating image topic options.
  getAiOptions() {
    const aisList = this.state.ais_list  
    var items = []
    items.push(<Option>{"NONE"}</Option>) 
    if (aisList.length > 0){
      for (var i = 0; i < aisList.length; i++) {
          items.push(<Option value={aisList[i]}>{aisList[i]}</Option>)
     }
    }
    else{
      items.push(<Option value={'NONE'}>{'NONE'}</Option>)
      //items.push(<Option value={'TEST1'}>{'TEST1'}</Option>)
      //items.push(<Option value={'TEST2'}>{'TEST2'}</Option>)
    }
    return items
  }


  onToggleAiSelection(event){
    const ai_name = event.target.value
    this.setState({selected_ai: ai_name})
  }

  getActiveAiStr(){
    const active =  this.state.ais_active_list
    var config_str_list = []
    for (var i = 0; i < active.length; i++) {
      config_str_list.push(active[i])
      config_str_list.push("\n")
    }
    const config_str =config_str_list.join("")
    return config_str
  }

  
  getDisabledAiStr(){
    const installed = this.state.ais_list
    const active =  this.state.ais_active_list
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
    const selected_ai = this.state.selected_ai
    const viewableAis = this.state.viewableAis
    const ai_options = this.getAiOptions()
    const active_ai_list = this.state.ais_active_list
    const ai_state = this.state.ais_active_list.indexOf(this.state.selected_ai) !== -1 

    return (

      <Columns equalWidth={true}>
      <Column>

      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Select AI Framework"}
         </label>

          <div onClick={this.toggleviewableAis} style={{backgroundColor: Styles.vars.colors.grey0}}>
            <Select style={{width: "10px"}}/>
          </div>
          <div hidden={!viewableAis}>
          {ai_options.map((ai) =>
          <div onClick={this.onToggleAiSelection}
            style={{
              textAlign: "center",
              padding: `${Styles.vars.spacing.xs}`,
              color: Styles.vars.colors.black,
              backgroundColor: (ai.props.value === selected_ai) ?
                Styles.vars.colors.green :
                (active_ai_list.includes(ai.props.value)) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
              cursor: "pointer",
              }}>
              <body ai-topic ={ai} style={{color: Styles.vars.colors.black}}>{ai}</body>
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
        {this.getActiveAiStr()}
        </pre>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Disabled AI Frameworks List "}
          </label>

        <pre style={{ height: "200px", overflowY: "auto" }} align={"center"} textAlign={"center"}>
        {this.getDisabledAiStr()}
        </pre>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


        </Column>
        <Column>



        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {this.state.ai_name}
          </label>

          <div hidden={(this.state.selected_ai === "NONE")}>

        <Label title="Enable AI Framework"> </Label>
          <Toggle
            checked={ai_state }
            onClick={() => sendUpdateActiveStateMsg(this.state.mgrNamespace + "/update_ai_state", this.state.selected_ai, !ai_state)}>
        </Toggle>

          </div>

        <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/enable_all_ais")}>{"Enable All"}</Button>
        </ButtonMenu>

        <ButtonMenu>
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/disable_all_ais")}>{"Disable All"}</Button>
        </ButtonMenu>

        </Column>
        </Columns> 

    )
  }


  toggleviewableModels() {
    const set = !this.state.viewableModels
    this.setState({viewableModels: set})
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
    const viewableModels = this.state.viewableModels
    const model_options = this.getModelOptions()
    const active_model_list = this.state.models_active_list
    const model_state = this.state.models_active_list.indexOf(this.state.selected_model) !== -1 



    return (

      <Columns equalWidth={true}>
      <Column>

      <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Select AI Framework"}
         </label>

          <div onClick={this.toggleviewableModels} style={{backgroundColor: Styles.vars.colors.grey0}}>
            <Select style={{width: "10px"}}/>
          </div>
          <div hidden={!viewableModels}>
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
        <Button onClick={() => this.props.ros.sendTriggerMsg(this.state.mgrNamespace + "/refresh_ais")}>{"Refresh"}</Button>
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


export default AiFrameworksMgr
