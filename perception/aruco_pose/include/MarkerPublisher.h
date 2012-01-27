#ifndef __MarkerPublisher__
#define __MarkerPublisher__

#include <ros/ros.h>

#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>

#include <visualization_msgs/Marker.h>

#include "aruco/aruco.h"

class MarkerPublisher
{
	private: ros::Publisher visualizationPublisher;
	private: double markerSize;

	public: MarkerPublisher(ros::NodeHandle nodeHandle, double markerSize)
	{
		this->visualizationPublisher = nodeHandle.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
		this->markerSize = markerSize;
	}

	public: void publishMarker(std_msgs::Header imageHeader, aruco::Marker marker, btVector3 position, btQuaternion orientation);
};

#endif
