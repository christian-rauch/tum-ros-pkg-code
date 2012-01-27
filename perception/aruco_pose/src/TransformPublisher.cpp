#include "TransformPublisher.h"

#include <sstream>

using namespace std;
using namespace ros;
using namespace tf;
using namespace aruco;

void TransformPublisher::publishTransform(std_msgs::Header imageHeader, Marker marker, btVector3 position, btQuaternion orientation)
{
	stringstream targetFrame;
	targetFrame << "/marker_" << marker.id;

	transformBroadcaster.sendTransform(StampedTransform(Transform(orientation, position), imageHeader.stamp, imageHeader.frame_id, targetFrame.str()));
}

