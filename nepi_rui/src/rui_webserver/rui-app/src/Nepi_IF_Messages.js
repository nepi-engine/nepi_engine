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
import Toggle from "react-toggle"
import Label from "./Label"
import { Column, Columns } from "./Columns"

import {Queue} from "./Utilities"



@inject("ros")
@observer

class NepiSystemMessages extends Component {
  constructor(props) {
    super(props)

    // these states track the values through  Status messages
    this.state = {

      msg_queue_size: 50,
      status_msg: null,

      needs_update: true,

      paused: false,

      connected: false,

      messagesStatusListener: null,

    }

    this.msg_queue = new Queue()

    this.convertStrListToJoinedStr = this.convertStrListToJoinedStr.bind(this)
    this.onClickPause = this.onClickPause.bind(this)
    this.getAllNamespace = this.getAllNamespace.bind(this)

    this.messagesStatusListener = this.messagesStatusListener.bind(this)
    this.updateMessagesStatusListener = this.updateMessagesStatusListener.bind(this)
  }


  getAllNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var allNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      allNamespace = "/" + namespacePrefix + "/" + deviceId + "/messages"
    }
    return allNamespace
  }

    // Callback for handling ROS Status messages
    messagesStatusListener(message) {
      const msg_str = message.message
      const paused = this.state.paused
      const queue_size = this.state.queue_size
      var queue_length = this.msg_queue.getLength()
      while ( queue_length > queue_size ){
        this.msg_queue.pullItem()
        queue_length = this.msg_queue.getLength()
      }
      if (paused === false){
        this.msg_queue.pushItem(msg_str)
      }

      this.setState({
        connected: true
      })
    }

  // Function for configuring and subscribing to Status
  updateMessagesStatusListener() {
    const allNamespace = this.getAllNamespace()
    const { messagesNamespace } = this.props
    var namespace = messagesNamespace
    if (messagesNamespace === "All"){
      namespace = allNamespace
    }
    if (this.state.messagesStatusListener) {
      this.state.messagesStatusListener.unsubscribe()
    }
    var messagesStatusListener = this.props.ros.setupStatusListener(
      namespace,
      "nepi_sdk_interfaces/Message",
      this.messagesStatusListener
    )
    this.setState({ messagesStatusListener: messagesStatusListener,
      needs_update: false})
  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const {topicNames} = this.props.ros
    const namespace = this.props.messagesNamespace
    const namespace_updated = (prevState.messagesNamespace !== namespace && namespace !== null)
    const msg_namespace = namespace + "/messages"
    const message_publishing = topicNames.indexOf(msg_namespace) !== -1
    const needs_update = (this.state.needs_update && namespace !== null && message_publishing == true)
    if (namespace_updated || needs_update) {
      if (namespace.indexOf('null') === -1){
        this.setState({messagesNamespace: namespace})
        this.updateMessagesStatusListener()
      }
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.messagesStatusListener) {
      this.state.messagesStatusListener.unsubscribe()
    }
  }

  convertStrListToJoinedStr(str_list) {
    var mod_str_list = []
    for (var i = 0; i < str_list.length; ++i) {
      mod_str_list.push(str_list[i]+"\n")
    }
    const joined_str = mod_str_list.join("")
    return joined_str

  }

  onClickPause(){
    const currentVal = this.state.paused
    this.setState({paused: !currentVal})
  }
  
  render() {
    const show_debug = this.props.ros.systemDebugEnabled
    const connected = this.state.connected
    const msg_str_list = (connected === true && this.msg_queue.getLength() > 0) ? this.msg_queue.getItems() : ["Waiting for message to publish"]
    const msg_str = this.convertStrListToJoinedStr(msg_str_list.reverse())
    const paused = this.state.paused

    return (


      <Section title={"System Messages"}>

            <Columns>
            <Column>

            <Label title="Pause"> </Label>

            <Toggle
                      checked={paused===true}
                      onClick={this.onClickPause}>
            </Toggle>
              
            </Column>
            <Column>


            </Column>
            <Column>

            </Column>
            </Columns>


            <Columns>
            <Column>
           <div align={"left"} textAlign={"left"}> 
        <label style={{fontWeight: 'bold'}}>
          {"Messages"}
        </label>
        
          <pre style={{ height: "1000px", overflowY: "auto" }}>
            {msg_str ? msg_str : ""}
          </pre>
          </div>

          </Column>
          </Columns>

          <Columns>
            <Column>

            <Label title="Show Debug Messages"> </Label>
            <Toggle
            checked={show_debug}
            onClick={() => this.props.ros.sendBoolMsg("debug_mode_enable", !show_debug)}>
            </Toggle>
              
            </Column>
            <Column>


            </Column>
            <Column>

            </Column>
            </Columns>


      </Section>


    )
  }

}
export default NepiSystemMessages
