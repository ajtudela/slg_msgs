#ifndef SEGMENT_ARRAY_DISPLAY_H
#define SEGMENT_ARRAY_DISPLAY_H

#include <sensor_msgs/LaserScan.h>
#include <rviz/message_filter_display.h>

namespace Ogre{
	class SceneNode;
}

namespace laser_geometry{
	class LaserProjection;
}

namespace rviz{
	class IntProperty;
	class PointCloudCommon;
}

namespace simple_laser_geometry{
/** @brief Visualizes a laser scan, received as a sensor_msgs::LaserScan. */
class segmentArrayDisplay: public rviz::MessageFilterDisplay<sensor_msgs::LaserScan>{
Q_OBJECT
	public:
		segmentArrayDisplay();
		~segmentArrayDisplay() override;
		void reset() override;
		void update(float wall_dt, float ros_dt) override;

	protected:
		/** @brief Do initialization. Overridden from MessageFilterDisplay. */
		void onInitialize() override;

		/** @brief Process a single message.  Overridden from MessageFilterDisplay. */
		void processMessage(const sensor_msgs::LaserScanConstPtr& scan) override;

		rviz::PointCloudCommon* point_cloud_common_;

		laser_geometry::LaserProjection* projector_;
			ros::Duration filter_tolerance_;
		};

} // namespace simple_laser_geometry

#endif
