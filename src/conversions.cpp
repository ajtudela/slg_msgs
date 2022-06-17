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

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "simple_laser_geometry/conversions.hpp"

/* Convert segment to Pose */
geometry_msgs::msg::Pose slg::segment2D_to_pose(slg::Segment2D segment){
	// Convert segment to tf2 transform
	tf2::Transform pose;
	pose.setOrigin({segment.centroid().x, segment.centroid().y, 0.0});
	// Create quaternion
	tf2::Quaternion q;
	q.setRPY(0, 0, segment.orientation());
	pose.setRotation(q);
	// Convert tf pose to geometry pose
	geometry_msgs::msg::Pose poseMsg;
	tf2::toMsg(pose, poseMsg);

	return poseMsg;
}

/* Segment2D to PointCloud */
pcloud::Ptr slg::segment2D_to_pcl(slg::Segment2D segment, std_msgs::msg::Header segHeader){
	pcloud::Ptr segmentCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (const auto& p : segment.get_points()){
		pcl::PointXYZRGB point;
		point.x = p.x;
		point.y = p.y;
		point.z = 0.0;
		point.rgb = 0;
		segmentCloudPtr->push_back(point);
	}
	segmentCloudPtr->header.frame_id = segHeader.frame_id;
	return segmentCloudPtr;
}

/* Convert a segment array message to a vector of segments */
std::vector<slg::Segment2D> slg::segment_array_msg_to_segment_vector(simple_laser_geometry::msg::SegmentArray segmentArrayMsg){
	std::vector<slg::Segment2D> segments;
	segments.insert(segments.begin(), std::begin(segmentArrayMsg.segments), std::end(segmentArrayMsg.segments));
	return segments;
}

/* Convert a vector of segments to a segment array message */
simple_laser_geometry::msg::SegmentArray slg::segment_vector_to_segment_array(std_msgs::msg::Header header, std::vector<slg::Segment2D> segments){
	simple_laser_geometry::msg::SegmentArray segmentArrayMsg;
	segmentArrayMsg.header = header;
	segmentArrayMsg.segments.insert(segmentArrayMsg.segments.begin(), std::begin(segments), std::end(segments));
	return segmentArrayMsg;
}