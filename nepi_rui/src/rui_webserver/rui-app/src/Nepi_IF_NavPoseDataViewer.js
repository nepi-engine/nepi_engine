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
import Button, { ButtonMenu } from "./Button"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Styles from "./Styles"
import Select, { Option } from "./Select"
import {setElementStyleModified, clearElementStyleModified} from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}

@inject("ros")
@observer
class NavPoseDataViewer extends Component {
  constructor(props) {
    super(props)

  }


  render() {
    const navpose_data = this.props.navposeData ? this.props.navposeData : null
    const frame_id = navpose_data ? navpose_data.frame_id : null
    const frame_3d = navpose_data ? navpose_data.frame_3d : null
    const frame_alt = navpose_data ? navpose_data.frame_alt : null
    const lat = navpose_data ? navpose_data.latitude : null
    const long = navpose_data ? navpose_data.longitude : null
    const alt = navpose_data ? navpose_data.altitude : null
    const head = navpose_data ? navpose_data.heading : null
    const x_m = navpose_data ? navpose_data.x_m : null
    const y_m = navpose_data ? navpose_data.y_m : null
    const z_m = navpose_data ? navpose_data.z_m : null
    const roll = navpose_data ? navpose_data.roll : null
    const pitch = navpose_data ? navpose_data.pitch : null
    const yaw = navpose_data ? navpose_data.yaw : null
    return (
      <Section title={"Nav/Pose Output Frame (" + frame_id + ")"}>
        <Columns>
          <Column>
            <label style={{fontWeight: 'bold'}}>
              {"Location"}
            </label>
            <Label title={"Latitude"}>
              <Input
                disabled
                value={round(lat, 6)}
                style={{ width: "80%" }}
              />
            </Label>
            <Label title={"Longitude"}>
              <Input
                disabled
                value={round(long, 6)}
                style={{ width: "80%" }}
              />
            </Label>
            <Label title={"Altitude " + frame_alt + " (m)"}>
              <Input
                disabled
                value={round(alt, 2)}
                style={{ width: "80%" }}
              />
            </Label>

            <Label title={"Heading (deg)"}>
              <Input
                disabled
                style={{ width: "80%" }}
                value={round(head, 2)}
              />
            </Label>

          </Column>
          <Column>
            <div style={{ display: "flex", marginLeft: Styles.vars.spacing.regular }}>
              <label style={{fontWeight: 'bold', flex: 1, textAlign: "left"}}>
              {"Orientation Frame " + frame_3d}
              </label>
            </div>
            <Label title={"Roll Degs"}>
              <Input
                disabled
                style={{ width: "45%", float: "left" }}
                value={round(roll, 2)}
              />
            </Label>
            <Label title={"Pitch Degs"}>
              <Input
                disabled
                style={{ width: "45%", float: "left" }}
                value={round(pitch, 2)}
              />

            </Label>
            <Label title={"Yaw Degs"}>
              <Input
                disabled
                style={{ width: "45%", float: "left" }}
                value={round(yaw, 2)}
              />
            </Label>
{/*
            <Label title={"Ref. Frame"}>
              <Input
                disabled
                style={{ width: "80%" }}
                value={navPoseOrientationFrame}
              />
            </Label>
*/}
            </Column>
          <Column>
            <div style={{ display: "flex", marginLeft: Styles.vars.spacing.regular }}>
              <label style={{fontWeight: 'bold', flex: 1, textAlign: "left"}}>
              {"Position Frame " + frame_3d}
              </label>
            </div>
            <Label title={"X Meters"}>
              <Input
                disabled
                style={{ width: "45%", float: "left" }}
                value={round(x_m, 2)}
              />
            </Label>
            <Label title={"Y (North) Meters"}>
              <Input
                disabled
                style={{ width: "45%", float: "left" }}
                value={round(y_m, 2)}
              />
            </Label>
            <Label title={"Z (Up) Meters"}>
              <Input
                disabled
                style={{ width: "45%", float: "left" }}
                value={round(z_m, 2)}
              />
            </Label>


          </Column>
        </Columns>

      </Section>
    )
  }

}
export default NavPoseDataViewer
