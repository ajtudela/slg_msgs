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

/* Convert Point2D to geometry_msgs::Point */
geometry_msgs::msg::Point slg::point2DToGeometryPoint(slg::Point2D point){
	geometry_msgs::msg::Point gPoint;
	gPoint.x = point.x;
	gPoint.y = point.y;
	return gPoint;
}

/* Convert geometry_msgs::Point to Point2D */
slg::Point2D slg::geometryPointToPoint2D(geometry_msgs::msg::Point gPoint){
	return slg::Point2D(gPoint.x, gPoint.y);
}

/* Convert segment to Pose */
geometry_msgs::msg::Pose slg::segment2DToPose(slg::Segment2D segment){
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
pcloud::Ptr slg::segment2DToPointCloud(slg::Segment2D segment, std_msgs::msg::Header segHeader){
	pcloud::Ptr segmentCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

	std::vector<slg::Point2D> points = segment.getPoints();
	for (const auto& p: points){
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

/* Convert a segment message to a segment 2D */
slg::Segment2D slg::segmentMsgToSegment2D(simple_laser_geometry::msg::Segment segmentMsg){
	slg::Point2D lastPointPriorSeg(segmentMsg.last_point_prior_segment.x, segmentMsg.last_point_prior_segment.y);
	slg::Point2D firstPointNextSeg(segmentMsg.first_point_next_segment.x, segmentMsg.first_point_next_segment.y);

	slg::Segment2D currSegment;

	// Read the points
	for (const auto& point: segmentMsg.points){
		currSegment.addPoint({point.x, point.y});
	}

	currSegment.setId(segmentMsg.id);
	currSegment.setLabel((slg::Label)segmentMsg.label);
	currSegment.setAngularDistanceToClosestBoundary(segmentMsg.angular_distance);
	currSegment.setPriorSegment(lastPointPriorSeg);
	currSegment.setNextSegment(firstPointNextSeg);

	return currSegment;
}

/* Convert a segment 2D to a segment message */
simple_laser_geometry::msg::Segment slg::segment2DToSegmentMsg(slg::Segment2D segment){
	simple_laser_geometry::msg::Segment segmentMsg;

	// Transform the segment in message
	segmentMsg.id = segment.getId();
	segmentMsg.label = segment.getLabel();
	segmentMsg.angular_distance = segment.getAngularDistanceToClosestBoundary();
	segmentMsg.last_point_prior_segment.x = segment.getPriorSegment().x;
	segmentMsg.last_point_prior_segment.y = segment.getPriorSegment().y;
	segmentMsg.first_point_next_segment.x = segment.getNextSegment().x;
	segmentMsg.first_point_next_segment.y = segment.getNextSegment().y;

	for (const auto& point: segment.getPoints()){
		geometry_msgs::msg::Point gPoint;
		gPoint.x = point.x;
		gPoint.y = point.y;
		segmentMsg.points.push_back(gPoint);
	}

	return segmentMsg;
}

/* Convert a segment 2D to a segment stamped message */
simple_laser_geometry::msg::SegmentStamped slg::segment2DToSegmentStampedMsg(std_msgs::msg::Header header, slg::Segment2D segment){
	simple_laser_geometry::msg::SegmentStamped segmentMsg;

	// Header of the message
	segmentMsg.header = header;

	// Transform the segment in message
	segmentMsg.id = segment.getId();
	segmentMsg.label = segment.getLabel();
	segmentMsg.angular_distance = segment.getAngularDistanceToClosestBoundary();
	segmentMsg.last_point_prior_segment.x = segment.getPriorSegment().x;
	segmentMsg.last_point_prior_segment.y = segment.getPriorSegment().y;
	segmentMsg.first_point_next_segment.x = segment.getNextSegment().x;
	segmentMsg.first_point_next_segment.y = segment.getNextSegment().y;

	for (const auto& point: segment.getPoints()){
		geometry_msgs::msg::Point gPoint;
		gPoint.x = point.x;
		gPoint.y = point.y;
		segmentMsg.points.push_back(gPoint);
	}

	return segmentMsg;
}

/* Convert a segment array message to a vector of segments */
std::vector<slg::Segment2D> slg::segmentArrayMsgToSegmentVector(simple_laser_geometry::msg::SegmentArray segmentArrayMsg){
	std::vector<slg::Segment2D> segments;

	// Read segments
	for (const auto& segment: segmentArrayMsg.segments){
		segments.push_back(segmentMsgToSegment2D(segment));
	}

	return segments;
}

/* Convert a vector of segments to a segment array message */
simple_laser_geometry::msg::SegmentArray slg::segmentVectorToSegmentArray(std_msgs::msg::Header header, std::vector<slg::Segment2D> segments){
	simple_laser_geometry::msg::SegmentArray segmentArrayMsg;

	// Header of the message
	segmentArrayMsg.header = header;

	// Transform the segment in message
	for (const auto& segment: segments){
		segmentArrayMsg.segments.push_back(segment2DToSegmentMsg(segment));
	}

	return segmentArrayMsg;
}

/* Convert a polygon to a geometry polygon */
geometry_msgs::msg::Polygon slg::polygonToGeometryPolygon(slg::Polygon polygon){
	geometry_msgs::msg::Polygon gPolygon;

	for (const auto& edge: polygon.getEdges()){
		slg::Point2D p = edge.a;
		geometry_msgs::msg::Point32 gPoint;
		gPoint.x = p.x;
		gPoint.y = p.y;
		gPoint.z = 0.0;
		gPolygon.points.push_back(gPoint);
	}

	return gPolygon;
}

/* Convert a geometry polygon to a polygon */
slg::Polygon slg::geometryPolygonToPolygon(geometry_msgs::msg::Polygon gPolygon){
	slg::Polygon polygon;

	// Read n-1 points
	for (unsigned int i = 0; i < gPolygon.points.size()-1; i++){
		geometry_msgs::msg::Point32 currPoint = gPolygon.points[i];
		geometry_msgs::msg::Point32 nextPoint = gPolygon.points[i+1];

		slg::Point2D a(currPoint.x, currPoint.y);
		slg::Point2D b(nextPoint.x, nextPoint.y);
		polygon.addEdge({a,b});
	}
	// Add the last edge
	geometry_msgs::msg::Point32 firstPoint = gPolygon.points[0];
	geometry_msgs::msg::Point32 lastPoint = gPolygon.points[gPolygon.points.size()-1];

	slg::Point2D a(lastPoint.x, lastPoint.y);
	slg::Point2D b(firstPoint.x, firstPoint.y);
	polygon.addEdge({a,b});

	return polygon;
}
