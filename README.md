# simple_laser_geometry

![ROS2](https://img.shields.io/badge/ros2-galactic-purple?logo=ros&logoColor=white)
![License](https://img.shields.io/badge/license-MIT-green)
[![Compiling Test](https://github.com/ajtudela/simple_laser_geometry/actions/workflows/build.yml/badge.svg?branch=galactic)](https://github.com/ajtudela/simple_laser_geometry/actions/workflows/build.yml)

## Overview
This package provides classes and messages to interact with laser related geometry.

## simple_laser_geometry c++ API
* [point2D.hpp](include/sensor_msgs/point2D.hpp): Faster 2D point class based on PCL but with unnecesary 3D functions.
* [polygon.hpp](include/sensor_msgs/polygon.hpp): Definitions and functionality relating to polygons.
* [segment2D.hpp](include/sensor_msgs/segment2D.hpp): Definitions and functionality relating to segments of laserscan.

## Messages (.msg)
* [Segment](msg/Segment.msg): Describes a laserscan splitted in segment.
* [SegmentArray](msg/BatteryState.msg): An array of Segment messages.
