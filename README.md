# slg_msgs

![ROS2](https://img.shields.io/badge/ros2-jazzy-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/github/license/ajtudela/slg_msgs)
[![Build](https://github.com/ajtudela/slg_msgs/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/ajtudela/slg_msgs/actions/workflows/build.yml)
[![codecov](https://codecov.io/gh/ajtudela/slg_msgs/graph/badge.svg?token=R48HZO62SQ)](https://codecov.io/gh/ajtudela/slg_msgs)

## Overview
This package provides classes and messages to interact with laser related geometry, simple laser geometry.

## simple_laser_geometry c++ API
* [point2D.hpp](include/slg_msgs/point2D.hpp): Faster 2D point class based on PCL but with unnecesary 3D functions.
* [polygon.hpp](include/slg_msgs/polygon.hpp): Definitions and functionality relating to polygons.
* [segment2D.hpp](include/slg_msgs/segment2D.hpp): Definitions and functionality relating to segments of laserscan.

## Messages (.msg)
* [Segment](msg/Segment.msg): Describes a laserscan splitted in segment.
* [SegmentArray](msg/SegmentArray.msg): An array of Segment messages.

## Installation

### Binaries

On Ubuntu 24.04 you can install the latest version of this package using the following command

```bash
sudo apt-get update
sudo apt-get install ros-jazzy-slg-msgs
```

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/jazzy/) (middleware for robotics),

#### Building

To build from source, clone the latest version from the main repository into your colcon workspace and compile the package using

```bash
cd colcon_workspace/src
git clone https://github.com/ajtudela/slg_msgs.git -b jazzy
cd ../
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install
```