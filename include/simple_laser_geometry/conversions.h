/*
 * CONVERSIONS
 *
 * Copyright (c) 2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of simple_laser_geometry.
 * 
 * All rights reserved.
 *
 */

#ifndef CONVERSIONS_H
#define CONVERSIONS_H

// C++
#include <vector>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ROS
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>

#include <simple_laser_geometry/polygon.h>
#include <simple_laser_geometry/segment2D.h>
#include <simple_laser_geometry/Segment.h>
#include <simple_laser_geometry/SegmentStamped.h>
#include <simple_laser_geometry/SegmentArray.h>

// Typedef for easier readability
typedef pcl::PointCloud<pcl::PointXYZRGB> pcloud;

namespace slg{
geometry_msgs::Point 	point2DToGeometryPoint(slg::Point2D point);
slg::Point2D  				geometryPointToPoint2D(geometry_msgs::Point gPoint);

geometry_msgs::Pose 	segment2DToPose(slg::Segment2D segment);
pcloud::Ptr 			segment2DToPointCloud(slg::Segment2D segment, std_msgs::Header segHeader);

Segment2D 								segmentMsgToSegment2D(simple_laser_geometry::Segment segmentMsg);
simple_laser_geometry::Segment 			segment2DToSegmentMsg(slg::Segment2D segment);
simple_laser_geometry::SegmentStamped 	segment2DToSegmentStampedMsg(std_msgs::Header header, slg::Segment2D segment);

std::vector<slg::Segment2D> 			segmentArrayMsgToSegmentVector(simple_laser_geometry::SegmentArray segmentArrayMsg);
simple_laser_geometry::SegmentArray 	segmentVectorToSegmentArray(std_msgs::Header header, std::vector<slg::Segment2D> segments);

geometry_msgs::Polygon 				polygonToGeometryPolygon(slg::Polygon polygon);
slg::Polygon						geometryPolygonToPolygon(geometry_msgs::Polygon gPolygon);
}
#endif
