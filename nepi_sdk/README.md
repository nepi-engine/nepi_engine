<!--
Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.

This file is part of nepi-engine
(see https://github.com/nepi-engine).

License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
-->
# nepi_sdk
This repository contains foundational NEPI Engine ROS and support modules. In particular, abstract base classes, ROS-independent "drivers", full system and device command and control nodes, and anything that doesn't serve a very specialized purpose in the NEPI edge ecosystem is included in this repository. Along with _nepi_ros_interfaces_, the present repository can be considered the foundation of the NEPI edge ecosystem, with most other submodules in the ecosystem relying heavily on this one.

### Code Organization ###
Code is organized within this repo according to standard ROS directory structures.

Worth noting is that supporting Python classes and functions intended to be imported in other modules generally are kept in `src/nepi_sdk` rather than the `scripts` subdirectory that hosts Python-implemented nodes. This allows a single _nepi_sdk_ python module to be generated for convenient and consistent import of these supporting components.

### Build and Install ###
This repository is typically built as part of the [nepi_engine_ws](https://github.com/nepi-engine/nepi_engine_ws) catkin workspace. Refer to that project's top-level README for build and install instructions.

The repository may be included in other custom catkin workspaces or built using standard CMake commands (with appropriate variable definitions provided), though it should be noted that the executables here work in concert with a complete NEPI Engine installation, so operability and functionality may be limited when building outside of the [nepi_engine_ws](https://github.com/nepi-engine/nepi_engine_ws) source tree.

### Branching and Tagging Strategy ###
In general branching, merging, tagging are all limited to the [nepi_engine_ws](https://github.com/nepi-engine/nepi_engine_ws) container repository, with the present submodule repository kept as simple and linear as possible.

### Contribution guidelines ###
Bug reports, feature requests, and source code contributions (in the form of pull requests) are gladly accepted!

### Who do I talk to? ###
At present, all user contributions and requests are vetted by Numurus technical staff, so you can use any convenient mechanism to contact us.