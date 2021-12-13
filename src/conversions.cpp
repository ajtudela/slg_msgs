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

#include <tf/tf.h>

#include "simple_laser_geometry/conversions.h"

/* Convert Point2D to geometry_msgs::Point */
geometry_msgs::Point slg::point2DToGeometryPoint(Point2D point){
	geometry_msgs::Point gPoint;
	gPoint.x = point.x;
	gPoint.y = point.y;
	return gPoint;
}

/* Convert geometry_msgs::Point to Point2D */
Point2D slg::geometryPointToPoint2D(geometry_msgs::Point gpoint){
	return Point2D(gpoint.x, gpoint.y);
}

/* Convert segment to Pose */
geometry_msgs::Pose slg::segment2DToPose(Segment2D segment){
	// Convert segment to tf pose
	tf::Pose pose;
	pose.setOrigin({segment.centroid().x, segment.centroid().y, 0.0});
	pose.setRotation(tf::createQuaternionFromYaw(segment.orientation()));
	// Convert tf pose to geometry pose
	geometry_msgs::Pose poseMsg;
	tf::poseTFToMsg(pose, poseMsg);

	return poseMsg;
}

/* Segment2D to PointCloud */
pcloud::Ptr slg::segment2DToPointCloud(Segment2D segment, std_msgs::Header segHeader){
	pcloud::Ptr segmentCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

	std::vector<Point2D> points = segment.getPoints();
	for(auto p: points){
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
Segment2D slg::segmentMsgToSegment2D(simple_laser_geometry::Segment segmentMsg){
	Point2D lastPointPriorSeg(segmentMsg.lastPointPriorSeg.x, segmentMsg.lastPointPriorSeg.y);
	Point2D firstPointNextSeg(segmentMsg.firstPointNextSeg.x, segmentMsg.firstPointNextSeg.y);

	Segment2D currSegment;

	// Read the points
	for(geometry_msgs::Point point: segmentMsg.points){
		currSegment.addPoint({point.x, point.y});
	}

	currSegment.setId(segmentMsg.id);
	currSegment.setLabel((Label)segmentMsg.label);
	currSegment.setAngularDistanceToClosestBoundary(segmentMsg.angleClose);
	currSegment.setPriorSegment(lastPointPriorSeg);
	currSegment.setNextSegment(firstPointNextSeg);

	return currSegment;
}

/* Convert a segment 2D to a segment message */
simple_laser_geometry::Segment slg::segment2DToSegmentMsg(Segment2D segment){
	simple_laser_geometry::Segment segmentMsg;

	// Transform the segment in message
	segmentMsg.id = segment.getId();
	segmentMsg.label = segment.getLabel();
	segmentMsg.angleClose = segment.getAngularDistanceToClosestBoundary();
	segmentMsg.lastPointPriorSeg.x = segment.getPriorSegment().x;
	segmentMsg.lastPointPriorSeg.y = segment.getPriorSegment().y;
	segmentMsg.firstPointNextSeg.x = segment.getNextSegment().x;
	segmentMsg.firstPointNextSeg.y = segment.getNextSegment().y;

	for(Point2D point: segment.getPoints()){
		geometry_msgs::Point gPoint;
		gPoint.x = point.x;
		gPoint.y = point.y;
		segmentMsg.points.push_back(gPoint);
	}

	return segmentMsg;
}

/* Convert a segment 2D to a segment stamped message */
simple_laser_geometry::SegmentStamped slg::segment2DToSegmentStampedMsg(std_msgs::Header header, Segment2D segment){
	simple_laser_geometry::SegmentStamped segmentMsg;

	// Header of the message
	segmentMsg.header = header;

	// Transform the segment in message
	segmentMsg.id = segment.getId();
	segmentMsg.label = segment.getLabel();
	segmentMsg.angleClose = segment.getAngularDistanceToClosestBoundary();
	segmentMsg.lastPointPriorSeg.x = segment.getPriorSegment().x;
	segmentMsg.lastPointPriorSeg.y = segment.getPriorSegment().y;
	segmentMsg.firstPointNextSeg.x = segment.getNextSegment().x;
	segmentMsg.firstPointNextSeg.y = segment.getNextSegment().y;

	for(Point2D point: segment.getPoints()){
		geometry_msgs::Point gPoint;
		gPoint.x = point.x;
		gPoint.y = point.y;
		segmentMsg.points.push_back(gPoint);
	}

	return segmentMsg;
}

/* Convert a segment array message to a vecctor of segments */
std::vector<Segment2D> slg::segmentArrayMsgToSegmentVector(simple_laser_geometry::SegmentArray segmentArrayMsg){
	std::vector<Segment2D> segments;

	// Read segments
	for(simple_laser_geometry::Segment segment: segmentArrayMsg.segments){
		segments.push_back(segmentMsgToSegment2D(segment));
	}

	return segments;
}

/* Convert a vecctor of segments to a segment array message */
simple_laser_geometry::SegmentArray slg::segmentVectorToSegmentArray(std_msgs::Header header, std::vector<Segment2D> segments){
	simple_laser_geometry::SegmentArray segmentArrayMsg;

	// Header of the message
	segmentArrayMsg.header = header;

	// Transform the segment in message
	for(Segment2D segment: segments){
		segmentArrayMsg.segments.push_back(segment2DToSegmentMsg(segment));
	}

	return segmentArrayMsg;
}
