/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
//import moment from "moment"
import { observer, inject } from "mobx-react"

import Section from "./Section"

import Styles from "./Styles"

import Toggle from "react-toggle"
import Label from "./Label"
import { Column, Columns } from "./Columns"

const styles = Styles.Create({
  canvas: {
    width: "100%",
    height: "auto",
    transform: "scale(1)"
  }
})

const COMPRESSION_HIGH_QUALITY = 95
const COMPRESSION_MED_QUALITY = 50
const COMPRESSION_LOW_QUALITY = 10

const PORT = 9091
const ROS_WEBCAM_URL_BASE = `http://${
  window.location.hostname
}:${PORT}/stream?topic=`

@inject("ros")
@observer
class CameraViewer extends Component {
  constructor(props) {
    super(props)

    this.state = {
      hasInitialized: false,
      shouldUpdate: true,
      //clockTime: moment(),
      streamWidth: null,
      streamHeight: null,
      currentStreamingImageQuality: COMPRESSION_HIGH_QUALITY,
      hideQualitySelector: this.props.hideQualitySelector
    }
    this.updateFrame = this.updateFrame.bind(this)
    this.onCanvasRef = this.onCanvasRef.bind(this)
    this.updateImageSource = this.updateImageSource.bind(this)
    this.onChangeImageQuality = this.onChangeImageQuality.bind(this)
  }

  updateFrame() {
    const {
      shouldUpdate,
      hasInitialized,
      streamWidth,
      streamHeight
    } = this.state

    const width_changed = this.canvas.width !== streamWidth
    const height_changed = this.canvas.height !== streamHeight
    const size_changed = (width_changed === true || height_changed === true)
    const updateCheck = ( shouldUpdate || size_changed)
    //const { imageRecognitions } = this.props.ros
    if (shouldUpdate && hasInitialized && this.canvas) {
      if (width_changed === true) {
        this.canvas.width = streamWidth
      }

      if (height_changed === true) {
        this.canvas.height = streamHeight
      }

      const context = this.canvas.getContext("2d")
      context.fillStyle = "red"
      context.textAlign = "center"
      context.font = "50px Arial"
      context.clearRect(0, 0, streamWidth, streamHeight)
      context.drawImage(this.image, 0, 0, streamWidth, streamHeight)

      /* -- TODO: Restore imageRecognitions when we have a need and are ready to use BoundingBoxes data type
      imageRecognitions.forEach(
        ({ label, roi: { x_offset, y_offset, width, height } }) => {
          context.beginPath()
          context.lineWidth = "10"
          context.strokeStyle = "red"
          context.rect(x_offset, y_offset, width, height)
          context.stroke()
          context.fillText(
            label,
            x_offset + width / 2,
            y_offset + 70,
            width - 30
          )
        }
      )
      */

      //this.setState({ clockTime: moment() })
      requestAnimationFrame(this.updateFrame)
    }
  }

  onCanvasRef(ref) {
    this.canvas = ref
    //this.updateImageSource()
  }

  updateImageSource() {
    if (this.props.imageTopic) {
      if (!this.image) {
        this.image = new Image() // EXPERIMENT -- Only create a new Image when strictly required
      }
      this.image.crossOrigin = "Anonymous"
      this.image.onload = () => {
        const { width, height } = this.image
        this.setState(
          {
            hasInitialized: true,
            streamWidth: width,
            streamHeight: height
          },
          () => {
            requestAnimationFrame(this.updateFrame)
          }
        )
      }
    }
    if (this.image) {
      const { streamingImageQuality } = this.props.ros
      this.image.src = ROS_WEBCAM_URL_BASE + this.props.imageTopic + '&quality=' + streamingImageQuality
    }
  }

  // Lifecycle method called when the props change.
  // Used to track changes in the image topic value
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { imageTopic } = this.props
    if (prevProps.imageTopic !== imageTopic || prevState.currentStreamingImageQuality !== this.state.currentStreamingImageQuality){
      this.updateImageSource()
    }
  }

  componentWillUnmount() {
    this.setState({ shouldUpdate: false })
    if (this.image) {
      this.image.src = null
    }
  }

  componentDidMount() {
    this.updateImageSource()
  }

  onChangeImageQuality(quality) {
    this.props.ros.onChangeStreamingImageQuality(quality)
    this.setState({currentStreamingImageQuality: quality})
    this.updateImageSource()
  }

  render() {
    const {
      streamingImageQuality
    } = this.props.ros

    if (streamingImageQuality !== this.state.currentStreamingImageQuality)
    {
      this.setState({currentStreamingImageQuality: streamingImageQuality})
    }

    return (
      <Section title={this.props.title}>
        <canvas style={styles.canvas} ref={this.onCanvasRef} />
        <Columns>
          <Column>
          <div align={"left"} textAlign={"left"}>
            { this.state.hideQualitySelector ?
              null :
              <Label title={"Compression Level"} />
            }
          </div>
          </Column>
          <Column>
          <div align={"left"} textAlign={"left"}>
            { this.state.hideQualitySelector ?
              null :
              <div>
                <Label title={"Low"} />
                <Toggle
                  checked={streamingImageQuality >= COMPRESSION_HIGH_QUALITY}
                  onClick={() => {this.onChangeImageQuality(COMPRESSION_HIGH_QUALITY)}}
                />
              </div>
            }
          </div>
          </Column>
          <Column>
          <div align={"left"} textAlign={"left"}>
            { this.state.hideQualitySelector ?
              null :
              <div>
                <Label title={"Medium"} />
                <Toggle
                  checked={streamingImageQuality >= COMPRESSION_MED_QUALITY && streamingImageQuality < COMPRESSION_HIGH_QUALITY}
                  onClick={() => {this.onChangeImageQuality(COMPRESSION_MED_QUALITY)}}
                />
              </div>
            }
          </div>
          </Column>
          <Column>
          <div align={"left"} textAlign={"left"}>
            { this.state.hideQualitySelector ?
              null :
              <div>
                <Label title={"High"} />
                <Toggle
                  checked={streamingImageQuality <= COMPRESSION_LOW_QUALITY}
                  onClick={() => {this.onChangeImageQuality(COMPRESSION_LOW_QUALITY)}}
                />
              </div>
            }
          </div>
          </Column>
        </Columns>
      </Section>
    )
  }
}

CameraViewer.defaultProps = {
  imageRecognitions: [
    // {
    //   label: "foobar",
    //   roi: { x_offset: 500, y_offset: 100, width: 300, height: 400 }
    // }
  ]
}

export default CameraViewer
