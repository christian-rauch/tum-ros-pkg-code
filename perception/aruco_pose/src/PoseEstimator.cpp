#include "PoseEstimator.h"

#include <LinearMath/btTransform.h>

#include "rpp/librpp.h"

bool PoseEstimator::estimatePose(aruco::Marker marker, double intrinsicCameraMatrix[], btVector3& position, btQuaternion& orientation)
{
	double low = 0 * markerSize;
	double high = 1 * markerSize;

	// Output
	rpp_float error;
	rpp_mat rotation;
	rpp_vec translation;

	// Input
	rpp_float cameraPrincipalPoint[2] = { intrinsicCameraMatrix[0 * 3 + 2 * 1], intrinsicCameraMatrix[1 * 3 + 2 * 1] };
	rpp_float cameraFocalLength[2] = { intrinsicCameraMatrix[0 * 3 + 0 * 1], intrinsicCameraMatrix[1 * 3 + 1 * 1] };

	rpp_vec modelPoints[4];
	modelPoints[0][0] = low; modelPoints[0][1] = low; modelPoints[0][2] = 0;
	modelPoints[1][0] = high; modelPoints[1][1] = low; modelPoints[1][2] = 0;
	modelPoints[2][0] = high; modelPoints[2][1] = high; modelPoints[2][2] = 0;
	modelPoints[3][0] = low; modelPoints[3][1] = high; modelPoints[3][2] = 0;
	rpp_vec imagePoints[4];
	imagePoints[0][0] = marker[0].x; imagePoints[0][1] = marker[0].y; imagePoints[0][2] = 1;
	imagePoints[1][0] = marker[1].x; imagePoints[1][1] = marker[1].y; imagePoints[1][2] = 1;
	imagePoints[2][0] = marker[2].x; imagePoints[2][1] = marker[2].y; imagePoints[2][2] = 1;
	imagePoints[3][0] = marker[3].x; imagePoints[3][1] = marker[3].y; imagePoints[3][2] = 1;

	robustPlanarPose(error, rotation, translation, cameraPrincipalPoint, cameraFocalLength, modelPoints, imagePoints, 4, 0, true, 1E-8, 1E-5, 0);

	position = btVector3(translation[0], translation[1], translation[2]);

	btMatrix3x3
	(
		rotation[0][0], rotation[0][1], rotation[0][2],
		rotation[1][0], rotation[1][1], rotation[1][2],
		rotation[2][0], rotation[2][1], rotation[2][2]
	)
	.getRotation(orientation);

	return error < 1E10;
}
