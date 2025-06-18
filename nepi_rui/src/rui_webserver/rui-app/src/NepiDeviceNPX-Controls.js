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
import Button, { ButtonMenu } from "./Button"
import RangeAdjustment from "./RangeAdjustment"
import {RadioButtonAdjustment, SliderAdjustment} from "./AdjustmentWidgets"
import Toggle from "react-toggle"
import Label from "./Label"
import Input from "./Input"
import { Column, Columns } from "./Columns"


import { round, onUpdateSetStateValue, onEnterSetStateFloatValue,  } from "./Utilities"


import NepiIF3DTransform from "./Nepi_IF_3DTransform"

@inject("ros")
@observer

// Component that contains the NPX Device controls
class NepiDeviceNPXControls extends Component {
  constructor(props) {
    super(props)

    // these states track the values through NPX Status messages
    this.state = {
      frame3D: null,
      age_filter_s: null
    }


  }


  renderControls() {
    const { npxDevices, sendBoolMsg, sendTriggerMsg, setIdxControlsEnable, setIdxAutoAdjust, setFrame3D } = this.props.ros
    const namespace = this.props.namespace ? this.props.namespace : null
    const message = this.props.status_msg ? this.props.status_msg : null
    const capabilities = npxDevices[namespace] ? npxDevices[namespace] : null


    if (namespace != null && capabilities != null && message != null){
      const update_rate = message.update_rate
      const frame_3d = message.frame_3d
      const frame_nav = message.frame_nav
      const frame_altitude = message.frame_altitude
      const frame_depth = message.frame_depth
      const has_loc = message.has_location
      const set_loc = message.set_as_location_source
      const has_head = message.has_heading
      const set_head = message.set_as_heading_source
      const has_orien = message.has_orientation
      const set_orien = message.set_as_orientation_source
      const has_pos = message.has_position
      const set_pos = message.set_as_position_source
      const has_alt = message.has_altitude
      const set_alt = message.set_as_altitude_source
      const has_depth = message.has_depth
      const set_depth = message.set_as_depth_source

      const has_transform = message.has_transform
    
      return (
        <Section title={"NavPose Controls"}>
          
          <div hidden={!has_loc}>    
            <Label title="Set as Location Source">
              <Toggle
                checked={set_loc===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_location_source",!set_loc)}>
              </Toggle>
            </Label>
          </div>

          <div hidden={!has_head}>    
            <Label title="Set as Heading Source">
              <Toggle
                checked={set_head===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_heading_source",!set_head)}>
              </Toggle>
            </Label>
          </div>

          <div hidden={!has_orien}>    
            <Label title="Set as Orientation Source">
              <Toggle
                checked={set_orien===true}
                onClick={() => {this.props.ros.sendBoolMsg(namespace + "/set_as_orientation_source", !set_orien);}}>
              </Toggle>
            </Label>
          </div>
          <div hidden={!has_pos}>    
            <Label title="Set as Position Source">
              <Toggle
                checked={set_pos===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_position_source",!set_pos)}>
              </Toggle>
            </Label>
          </div>

          <div hidden={!has_alt}>    
            <Label title="Set as Altitude Source">
              <Toggle
                checked={set_alt===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_altitude_source",!set_alt)}>
              </Toggle>
            </Label>
          </div>

          <div hidden={!has_depth}>    
            <Label title="Set as Depth Source">
              <Toggle
                checked={set_depth===true}
                onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_as_depth_source",!set_depth)}>
              </Toggle>
            </Label>
          </div>


          <Columns>
                  <Column>

                          <NepiIF3DTransform
                              namespace={namespace}
                              has_transform={has_transform}
                              title={"Nepi_IF_3DTransform"}
                          />

                  </Column>
            </Columns>



          <Columns>
            <Column>
            </Column>
            <Column>
              <ButtonMenu>
                <Button onClick={() => sendTriggerMsg(namespace + "/reset_controls")}>{"Reset Controls"}</Button>
              </ButtonMenu>
            </Column>
          </Columns>

        </Section>
      )
    }
  }

  render() {
    return (
      <Columns>
        <Column>
          {this.renderControls()}
        </Column>
      </Columns>
    )
  }
}

export default NepiDeviceNPXControls