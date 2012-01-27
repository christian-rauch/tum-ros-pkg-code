#include "MarkerPublisher.h"

using namespace ros;
using namespace visualization_msgs;
using namespace aruco;

void MarkerPublisher::publishMarker(std_msgs::Header imageHeader, aruco::Marker marker, btVector3 position, btQuaternion orientation)
{
	visualization_msgs::Marker visualizationMarker;

	visualizationMarker.header = imageHeader;

	visualizationMarker.ns = "aruco_pose";
	visualizationMarker.id = marker.id;
	visualizationMarker.type = visualization_msgs::Marker::CUBE;
	visualizationMarker.action = visualization_msgs::Marker::ADD;

	visualizationMarker.pose.position.x = position.x();
	visualizationMarker.pose.position.y = position.y();
	visualizationMarker.pose.position.z = position.z();

	visualizationMarker.pose.orientation.x = orientation.x();
	visualizationMarker.pose.orientation.y = orientation.y();
	visualizationMarker.pose.orientation.z = orientation.z();
	visualizationMarker.pose.orientation.w = orientation.w();

	visualizationMarker.scale.x = 1.00 * markerSize;
	visualizationMarker.scale.y = 1.00 * markerSize;
	visualizationMarker.scale.z = 0.25 * markerSize;

	visualizationMarker.color.r = 0.0;
	visualizationMarker.color.g = 0.0;
	visualizationMarker.color.b = 1.0;
	visualizationMarker.color.a = 1.0;

	visualizationMarker.lifetime = Duration(30.0);

	visualizationPublisher.publish(visualizationMarker);
}
