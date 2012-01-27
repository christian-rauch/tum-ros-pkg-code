#ifndef __PoseEstimator__
#define __PoseEstimator__

#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>

#include "aruco/aruco.h"

class PoseEstimator
{
	private: double markerSize;

	public: PoseEstimator(double markerSize)
	{
		this->markerSize = markerSize;
	}

	public: bool estimatePose(aruco::Marker marker, double intrinsicCameraMatrix[], btVector3& position, btQuaternion& orientation);
};

#endif
