/*
 * Copyright (C) 2010, Dejan Pangercic <dejan.pangercic@cs.tum.edu>, Alexis Maldonado <maldonado@tum.de>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ace/config.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
using namespace std;
//openCV
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

//yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <camera_calibration_parsers/parse_ini.h>



//namespaces
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;

class YARPToROSImage {

public:
    string port_name_, output_image_topic_;
    // Initialize network
    Network yarp_;
    // Make a port for reading and writing images
    BufferedPort<ImageOf<PixelRgb> > port_;
    IplImage *cvImage_;
    ImageOf<PixelRgb> *yarpImage;
    sensor_msgs::CameraInfo camera_info;
    std::string camera_parameters_file;
    std::string camera_frame_id;


    bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp) {

      camera_info = req.camera_info;
      ROS_INFO("New camera info received and accepted");

      if (camera_calibration_parsers::writeCalibrationIni(camera_parameters_file.c_str(), "yarp_to_ros_image", camera_info)) {
        ROS_INFO( "Camera information written to the camera parameters file: %s", camera_parameters_file.c_str() );
        return true;
      }
      else {
        ROS_ERROR( "Could not write the camera parameters file: %s", camera_parameters_file.c_str() );
        return false;
      }
    }
    
    // Constructor
    YARPToROSImage(ros::NodeHandle &n) :
    n_(n), it_(n_)
    {
        n_.param("port_name", port_name_, std::string("/yarp_to_ros_image"));
        n_.param("output_image_topic", output_image_topic_, std::string("image"));
        n_.param("camera_parameters_file", camera_parameters_file, std::string("camera_parameters.txt"));
        n_.param("camera_frame_id", camera_frame_id, std::string("/r_eye3"));

        std::string camera_name;
        if (camera_calibration_parsers::readCalibrationIni(camera_parameters_file.c_str(), camera_name, camera_info)) {
              ROS_INFO("Successfully read camera calibration: \"%s\"  Rerun camera calibrator if it is incorrect.", camera_parameters_file.c_str());
        } else {
            ROS_ERROR("No camera parameters file found.  Use default file if no other is available. (This process will publish invalid camera calibration data).");
        }

        ConstString yarp_port_name_(port_name_.c_str());
        yarp_.init();
        port_.open(yarp_port_name_);
        pub_ = it_.advertiseCamera(output_image_topic_, 1);
        set_camera_info = n_.advertiseService("set_camera_info", &YARPToROSImage::setCameraInfo, this);
    }
  

    ~YARPToROSImage()
    {
        yarp_.fini();
    }
    

    // Spin (!)
    bool spin ()
    {
        yarpImage = port_.read();
        cvImage_ = cvCreateImage(cvSize(yarpImage->width(),yarpImage->height()), 
                                 IPL_DEPTH_8U, 3 );
        while (n_.ok ())
        {     
            // read an image from the port
            yarpImage = port_.read();
            ros::Time now = ros::Time::now();

            if (yarpImage==NULL) continue;
            
            cvCvtColor((IplImage*)yarpImage->getIplImage(), cvImage_, CV_RGB2BGR);
            camera_info.header.stamp.sec = now.sec;
            camera_info.header.stamp.nsec = now.nsec;
            //The other fields of camera_info are already there: loaded from file
            // or set by the service call
            
            try
            {
                sensor_msgs::Image::Ptr pimg = bridge_.cvToImgMsg(cvImage_, "bgr8");
                pimg->header.stamp.sec = now.sec;
                pimg->header.stamp.nsec = now.nsec;
                pimg->header.frame_id = camera_frame_id.c_str();
                pub_.publish(*pimg, camera_info);
            }
            catch (sensor_msgs::CvBridgeException error)
            {
                ROS_ERROR("CvBridgeException error");
            }

            ros::spinOnce ();
        }
        cvReleaseImage(&cvImage_);
        return (true);
    }
    
protected:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  image_transport::CameraPublisher pub_;
  ros::ServiceServer set_camera_info;

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "yarp_to_ros_image");
    ros::NodeHandle n("~");
    YARPToROSImage ytri(n);
    ytri.spin();
    return 0;
}
