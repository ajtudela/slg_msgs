# simple_laser_geometry

![ROS2](https://img.shields.io/badge/ros2-galactic-purple?logo=ros&logoColor=white)
![License](https://img.shields.io/badge/license-MIT-green)
[![Compiling Test](https://github.com/ajtudela/simple_laser_geometry/actions/workflows/build.yml/badge.svg?branch=galactic)](https://github.com/ajtudela/simple_laser_geometry/actions/workflows/build.yml)

## Overview
ROS package that contains classes and messages to interact with laser related geometry and functions to convert them to ROS standard.
Useful to use with LaserScan. Contains three classes:

 * **`Point2D`**

	Faster point class based on PCL but with unnecesary 3D functions.

 * **`Segment2D`**

	Extension of class Point to handle segments.

 * **`Polygon`**

	Struct that defines a polygon and functions related.

And three custom messages:
 * **`Segment`**

	Message based on the class mention before.

 * **`SegmentStamped`**

	Segment message with header.

 * **`SegmentArray`**

	An array of Segment message with header.
