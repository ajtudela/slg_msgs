# slg_msgs

![ROS2](https://img.shields.io/badge/ros2-galactic-purple?logo=ros&logoColor=white)
![License](https://img.shields.io/badge/license-MIT-green)
[![Build](https://github.com/ajtudela/slg_msgs/actions/workflows/build.yml/badge.svg?branch=galactic)](https://github.com/ajtudela/slg_msgs/actions/workflows/build.yml)

## Overview
This package provides classes and messages to interact with laser related geometry, simple laser geometry.

## simple_laser_geometry c++ API
* [point2D.hpp](include/slg_msgs/point2D.hpp): Faster 2D point class based on PCL but with unnecesary 3D functions.
* [polygon.hpp](include/slg_msgs/polygon.hpp): Definitions and functionality relating to polygons.
* [segment2D.hpp](include/slg_msgs/segment2D.hpp): Definitions and functionality relating to segments of laserscan.

## Messages (.msg)
* [Segment](msg/Segment.msg): Describes a laserscan splitted in segment.
* [SegmentArray](msg/BatteryState.msg): An array of Segment messages.
