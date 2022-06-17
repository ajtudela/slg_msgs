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

#ifndef SIMPLE_LASER_GEOMETRY__CONVERSIONS_HPP_
#define SIMPLE_LASER_GEOMETRY__CONVERSIONS_HPP_

// C++
#include <vector>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ROS
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>

#include <simple_laser_geometry/segment2D.hpp>
#include <simple_laser_geometry/msg/segment.hpp>
#include <simple_laser_geometry/msg/segment_array.hpp>

// Typedef for easier readability
typedef pcl::PointCloud<pcl::PointXYZRGB> pcloud;

namespace slg{
geometry_msgs::msg::Pose segment2D_to_pose(slg::Segment2D segment);
pcloud::Ptr segment2D_to_pcl(slg::Segment2D segment, std_msgs::msg::Header segHeader);

std::vector<slg::Segment2D> segment_array_msg_to_segment_vector(simple_laser_geometry::msg::SegmentArray segmentArrayMsg);
simple_laser_geometry::msg::SegmentArray segment_vector_to_segment_array(std_msgs::msg::Header header, std::vector<slg::Segment2D> segments);
}  // namespace slg

#endif  // SIMPLE_LASER_GEOMETRY__CONVERSIONS_HPP_
