#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ros/time.h>

#include <laser_geometry/laser_geometry.h>

#include <rviz/default_plugin/point_cloud_common.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/int_property.h>
#include <rviz/validate_floats.h>

#include "segmentArrayDisplay.h"

namespace simple_laser_geometry{
segmentArrayDisplay::segmentArrayDisplay(): point_cloud_common_(new rviz::PointCloudCommon(this)), 
											projector_(new laser_geometry::LaserProjection()){
}

segmentArrayDisplay::~segmentArrayDisplay(){
	delete point_cloud_common_;
	delete projector_;
}

void segmentArrayDisplay::onInitialize(){
	// Use the threaded queue for processing of incoming messages
	update_nh_.setCallbackQueue(context_->getThreadedQueue());

	MFDClass::onInitialize();
	point_cloud_common_->initialize(context_, scene_node_);
}

void segmentArrayDisplay::processMessage(const sensor_msgs::LaserScanConstPtr& scan){
	sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);

	// Compute tolerance necessary for this scan
	ros::Duration tolerance(scan->time_increment * scan->ranges.size());
	if (tolerance > filter_tolerance_){
		filter_tolerance_ = tolerance;
		tf_filter_->setTolerance(filter_tolerance_);
	}

	try{
		auto tf = context_->getTF2BufferPtr();

		projector_->transformLaserScanToPointCloud(fixed_frame_.toStdString(), *scan, *cloud, *tf, -1.0,
											   laser_geometry::channel_option::Intensity);
	}catch (tf2::TransformException& e){
		ROS_DEBUG("LaserScan [%s]: failed to transform scan: %s.  This message should not repeat (tolerance "
				  "should now be set on our tf2::MessageFilter).",
				  qPrintable(getName()), e.what());
		return;
	}

	point_cloud_common_->addMessage(cloud);
}

void segmentArrayDisplay::update(float wall_dt, float ros_dt){
	point_cloud_common_->update(wall_dt, ros_dt);
}

void segmentArrayDisplay::reset(){
	MFDClass::reset();
	point_cloud_common_->reset();
}

} // namespace simple_laser_geometry

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(simple_laser_geometry::segmentArrayDisplay, rviz::Display)
