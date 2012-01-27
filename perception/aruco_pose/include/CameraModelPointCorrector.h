#ifndef __CameraModelPointCorrector__
#define __CameraModelPointCorrector__

#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/CameraInfo.h>

#include "EdgeDetection/PointCorrector.h"
#include "EdgeDetection/Vector2.h"

class CameraModelPointCorrector : public EdgeDetection::PointCorrector
{
	private: image_geometry::PinholeCameraModel cameraModel;

	public: EdgeDetection::Vector2 CorrectPoint(EdgeDetection::Vector2 point)
	{
		Point2d rawPoint(point.GetX(), point.GetY());
		Point2d correctedPoint = cameraModel.rectifyPoint(rawPoint);

		return EdgeDetection::Vector2(correctedPoint.x, correctedPoint.y);
	}
	public: void updateCameraModel(const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		cameraModel.fromCameraInfo(cameraInfo);
	}
	public: void updateCameraModel(sensor_msgs::CameraInfo& cameraInfo)
	{
		cameraModel.fromCameraInfo(cameraInfo);
	}
};

#endif
