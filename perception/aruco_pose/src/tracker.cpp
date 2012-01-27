#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "aruco/aruco.h"

#include <iostream>
#include <XmlRpcValue.h>

#include "PoseEstimator.h"
#include "CameraModelPointCorrector.h"
#include "MarkerRefiner.h"
#include "MarkerPublisher.h"
#include "TransformPublisher.h"

using namespace std;
using namespace XmlRpc;
using namespace ros;
using namespace image_transport;
using namespace sensor_msgs;
using namespace aruco;
using namespace EdgeDetection;

PoseEstimator* poseEstimator;
CameraModelPointCorrector* cameraModelPointCorrector;
MarkerRefiner* markerRefiner;
MarkerPublisher* markerPublisher;
TransformPublisher* transformPublisher;

void handleImage(const ImageConstPtr& image, const CameraInfoConstPtr& cameraInfo)
{
	CvBridge bridge;
	IplImage* iplImage = bridge.imgMsgToCv(image, "rgb8");
	Mat matrixImage = iplImage;

	cameraModelPointCorrector->updateCameraModel(cameraInfo);

	double intrinsicCameraMatrix[9];
	for (int index = 0; index < 9; index++) intrinsicCameraMatrix[index] = cameraInfo->K[index];

	vector<aruco::Marker> markers;

	MarkerDetector markerDetector;
	markerDetector.detect(matrixImage, markers, CameraParameters());

	for (vector<aruco::Marker>::iterator marker = markers.begin(); marker < markers.end(); marker++)
	{
		aruco::Marker refinedMarker = markerRefiner->refineMarker(*marker, matrixImage);

		btVector3 position;
		btQuaternion orientation;
		if (poseEstimator->estimatePose(refinedMarker, intrinsicCameraMatrix, position, orientation))
		{
			if (markerPublisher != 0) markerPublisher->publishMarker(image->header, refinedMarker, position, orientation);
			if (transformPublisher != 0) transformPublisher->publishTransform(image->header, refinedMarker, position, orientation);
		}
	}
}

int main(int argc, char** argv)
{
	init(argc, argv, "aruco_pose");

	NodeHandle nodeHandle("~");

	string correctionTablePath;
	nodeHandle.param("correction_table_path", correctionTablePath, string("MicroEdgeCorrectionTable.bin"));
	double markerSize;
	nodeHandle.param("marker_size", markerSize, 1.0);
	bool publishMarkers;
	nodeHandle.param("publish_markers", publishMarkers, false);
	bool publishTransforms;
	nodeHandle.param("publish_transforms", publishTransforms, true);

	ImageTransport imageTransport(nodeHandle);
	imageTransport.subscribeCamera("/image", 1, handleImage);

	poseEstimator = new PoseEstimator(markerSize);
	cameraModelPointCorrector = new CameraModelPointCorrector();
	markerRefiner = new MarkerRefiner(correctionTablePath, cameraModelPointCorrector);
	if (publishMarkers) markerPublisher = new MarkerPublisher(nodeHandle, markerSize);
	if (publishTransforms) transformPublisher = new TransformPublisher();

	spin();

	delete transformPublisher;
	delete markerPublisher;
	delete markerRefiner;
	delete cameraModelPointCorrector;
	delete poseEstimator;
}
