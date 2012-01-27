#ifndef __TransformPublisher__
#define __TransformPublisher__

#include <ros/ros.h>

#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>
#include <tf/transform_broadcaster.h>

#include "aruco/aruco.h"

class TransformPublisher
{
	private: tf::TransformBroadcaster transformBroadcaster;

	public: void publishTransform(std_msgs::Header imageHeader, aruco::Marker marker, btVector3 position, btQuaternion orientation);
};

#endif
