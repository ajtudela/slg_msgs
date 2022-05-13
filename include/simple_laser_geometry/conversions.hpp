/*
 * CONVERSIONS
 *
 * Copyright (c) 2021-2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
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
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>

#include <simple_laser_geometry/polygon.hpp>
#include <simple_laser_geometry/segment2D.hpp>
#include <simple_laser_geometry/msg/segment.hpp>
#include <simple_laser_geometry/msg/segment_stamped.hpp>
#include <simple_laser_geometry/msg/segment_array.hpp>

// Typedef for easier readability
typedef pcl::PointCloud<pcl::PointXYZRGB> pcloud;

namespace slg{
geometry_msgs::msg::Point 						point2DToGeometryPoint(slg::Point2D point);
slg::Point2D  									geometryPointToPoint2D(geometry_msgs::msg::Point gPoint);

geometry_msgs::msg::Pose 						segment2DToPose(slg::Segment2D segment);
pcloud::Ptr 									segment2DToPointCloud(slg::Segment2D segment, std_msgs::msg::Header segHeader);

Segment2D 										segmentMsgToSegment2D(simple_laser_geometry::msg::Segment segmentMsg);
simple_laser_geometry::msg::Segment 			segment2DToSegmentMsg(slg::Segment2D segment);
simple_laser_geometry::msg::SegmentStamped 		segment2DToSegmentStampedMsg(std_msgs::msg::Header header, slg::Segment2D segment);

std::vector<slg::Segment2D> 					segmentArrayMsgToSegmentVector(simple_laser_geometry::msg::SegmentArray segmentArrayMsg);
simple_laser_geometry::msg::SegmentArray 		segmentVectorToSegmentArray(std_msgs::msg::Header header, std::vector<slg::Segment2D> segments);

geometry_msgs::msg::Polygon 					polygonToGeometryPolygon(slg::Polygon polygon);
slg::Polygon									geometryPolygonToPolygon(geometry_msgs::msg::Polygon gPolygon);
}
#endif
