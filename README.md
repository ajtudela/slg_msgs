# simple_laser_geometry

## Overview
ROS package that contains classes and messages to interact with laser related geometry. Useful to use with LaserScan. Contains three classes:

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
