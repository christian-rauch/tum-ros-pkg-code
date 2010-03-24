/*
 * Copyright (C) 2010 by Dejan Pangercic <dejan.pangercic@cs.tum.edu>
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

 
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"

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
    bool display_;
    int ct_;
    char key_;
    IplImage *cvImage_;
    ImageOf<PixelRgb> *yarpImage;
    YARPToROSImage(ros::NodeHandle &n) :
    n_(n), it_(n_)
    {
        n_.param("port_name", port_name_, std::string("/yarp_to_ros_image"));
        n_.param("output_image_topic", output_image_topic_, std::string("yarp_to_ros_image"));
        n_.param("display", display_, true);
        ConstString yarp_port_name_(port_name_.c_str());
        yarp_.init();
        ct_ = 0;
        key_ = ' ';
        port_.open(yarp_port_name_);
        image_pub_ = it_.advertise(output_image_topic_, 1);
    }
  

    ~YARPToROSImage()
    {
        yarp_.fini();
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
        //double interval = rate_ * 1e+6;
        yarpImage = port_.read();
        cvImage_ = cvCreateImage(cvSize(yarpImage->width(),yarpImage->height()), 
                                 IPL_DEPTH_8U, 3 );
        while (n_.ok ())
        {     
            // read an image from the port
            yarpImage = port_.read();
            if (yarpImage==NULL) continue;
            
            // add a blue circle
            if (display_)
            {
                PixelRgb blue(0,0,255);
                addCircle(*yarpImage,blue,ct_,50,10);
                ct_ = (ct_+5)%yarpImage->width();
            }
            ROS_INFO("Copying YARP image to an OpenCV/IPL image");
            cvCvtColor((IplImage*)yarpImage->getIplImage(), cvImage_, CV_RGB2BGR);
            
            if (display_)
            {
                ROS_INFO("Showing OpenCV/IPL image");
                cvNamedWindow("yarp_to_ros_image",1);
                cvShowImage("yarp_to_ros_image",cvImage_);
            }
            try
            {
                ROS_INFO ("Publishing data on topic %s.", n_.resolveName ("yarp_to_ros_image").c_str ());
                image_pub_.publish(bridge_.cvToImgMsg(cvImage_, "passthrough"));
            }
            catch (sensor_msgs::CvBridgeException error)
            {
                ROS_ERROR("error");
            }
            if (display_)
                key_ = cvWaitKey(100);
            ros::spinOnce ();
        }
        return (true);
        cvReleaseImage(&cvImage_);
        if(display_)
            cvDestroyWindow("yarp_to_ros_image");
    }
    
protected:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  image_transport::Publisher image_pub_;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "yarp_to_ros_image");
    ros::NodeHandle n("~");
    YARPToROSImage ytri(n);
    ytri.spin();
    return 0;
}
