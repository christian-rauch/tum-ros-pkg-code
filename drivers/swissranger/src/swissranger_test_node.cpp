/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* author: Radu Bogdan Rusu <rusu@cs.tum.edu> */

/**

@mainpage

@htmlinclude manifest.html

@b swissranger_test_node is a test for the SwissRanger SRx000 driver.

<hr>

@section information Information

The driver depends on the libusbsr library to compile correctly.

<hr>

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name / type):
- @b "cloud_sr"/PointCloud  : Point cloud data from the SwissRanger.
- @b "images_sr"/ImageArray : Distance, intensity, and confidence camera images from the SwissRanger

<hr>

@section parameters ROS parameters

Reads the following parameters from the parameter server

- @b "sr_auto_illumination"     : @b [int] the angle of the first range measurement in degrees (Default: -90.0)
- @b "sr_integration_time"      : @b [int] the angle of the last range measurement in degrees (Default: 90.0)
- @b "sr_modulation_freq"       : @b [int] the number of adjascent range measurements to cluster into a single reading (Default: 1)
- @b "sr_amp_threshold"         : @b [int] the number of scans to skip between each measured scan (Default: 1)

 **/

// ROS core
#include <ros/node.h>
#include <ros/time.h>
#include <ros/common.h>

#include <robot_msgs/PointCloud.h>
#include <deprecated_msgs/ImageArray.h>

// OpenCV  + point_cloud_mapping (for dumping data to disk)
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <point_cloud_mapping/cloud_io.h>

#include <fstream>

// Drivers used
#include "swissranger.h"

#define DEFAULT_INT_VALUE INT_MIN
#define DEFAULT_DBL_VALUE DBL_MIN

using namespace std;
using namespace robot_msgs;
using namespace deprecated_msgs;

class SwissRangerTestNode
{
  protected:
    ros::Node& node_;
  public:

    int sr_auto_illumination_, sr_integration_time_, sr_modulation_freq_, sr_amp_threshold_;
    int sr_auto_illumination_prev_, sr_integration_time_prev_, sr_modulation_freq_prev_, sr_amp_threshold_prev_;

    // ROS messages
    PointCloud sr_msg_cloud_;
    ImageArray sr_msg_images_;

    swissranger::SwissRanger sr_;
    
    bool dump_to_disk_;

    SwissRangerTestNode (ros::Node& anode) : node_ (anode), dump_to_disk_ (false)
    {
      // Initialize internal parameters
      node_.param ("~sr_auto_illumination", sr_auto_illumination_, DEFAULT_INT_VALUE);
      node_.param ("~sr_integration_time", sr_integration_time_, DEFAULT_INT_VALUE);
      node_.param ("~sr_modulation_freq", sr_modulation_freq_, DEFAULT_INT_VALUE);
      node_.param ("~sr_amp_threshold", sr_amp_threshold_, DEFAULT_INT_VALUE);

      node_.param ("~sr_dump_to_disk", dump_to_disk_, false);
      
      sr_auto_illumination_prev_ = sr_integration_time_prev_ = sr_modulation_freq_prev_ = sr_amp_threshold_prev_ = DEFAULT_INT_VALUE;
      
      // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
      node_.advertise<PointCloud>("cloud_sr", 1);
      node_.advertise<ImageArray>("images_sr", 1);
    }

    ~SwissRangerTestNode ()
    {
      stop ();
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Start
    int
      start ()
    {
      // Open the SwissRanger device
      try
      {
        if (sr_.open () == 0)
        {
          ROS_INFO ("[SwissRangerTestNode::SwissRanger] Connected to device with ID: %s", sr_.device_id_.c_str ());
          ROS_INFO ("[SwissRangerTestNode::SwissRanger] libusbSR version: %s", sr_.lib_version_.c_str ());
        }
      } catch (swissranger::Exception& e) {
        ROS_ERROR("[SwissRangerTestNode::SwissRanger] Exception thrown while connecting to the sensor.\n%s", e.what ());
        return (-1);
      }

//      sr_.setAutoIllumination (1);
//      sr_.setAmplitudeThreshold (100.0);
      sr_.setModulationFrequency (MF_30MHz);			// For the love of God, please do not change this.
      ROS_INFO ("[SwissRangerTestNode::SwissRanger] Modulation frequency is: %d", sr_.getModulationFrequency ());
      ROS_INFO ("[SwissRangerTestNode::SwissRanger] Amplitude threshold is: %d", sr_.getAmplitudeThreshold ());

      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Stop
    int
      stop ()
    {
      // Close the SwissRanger device
      if (sr_.close () == 0)
        ROS_INFO ("[SwissRangerTestNode::SwissRanger] Driver shut down successfully");
      else
        return (-1);

      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Obtain a set of interesting parameters from the parameter server
    void
      getParametersFromServer ()
    {
      // Swissranger related parameters
      node_.getParam ("~sr_auto_illumination", sr_auto_illumination_);
      node_.getParam ("~sr_integration_time", sr_integration_time_);
      node_.getParam ("~sr_modulation_freq", sr_modulation_freq_);
      node_.getParam ("~sr_amp_threshold", sr_amp_threshold_);
      node_.getParam ("~sr_dump_to_disk", dump_to_disk_);
   }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Activate the parameters which changed (and do it only once)
    void
      setInternalParameters ()
    {
      // Swissranger related parameters
      if (sr_auto_illumination_ != sr_auto_illumination_prev_)
      {
        sr_auto_illumination_prev_ = sr_auto_illumination_;
        ROS_INFO ("[SwissRangerTestNode::setInternalParameters::Swissranger] Auto illumination settings changed to %d", sr_auto_illumination_);
        sr_.setAutoIllumination (sr_auto_illumination_);
      }

      if (sr_integration_time_ != sr_integration_time_prev_)
      {
        sr_integration_time_prev_ = sr_integration_time_;
        ROS_INFO ("[SwissRangerTestNode::setInternalParameters::Swissranger] Integration time changed to %d", sr_integration_time_);
        sr_.setIntegrationTime (sr_integration_time_);
      }

      if (sr_modulation_freq_ != sr_modulation_freq_prev_)
      {
        sr_modulation_freq_prev_ = sr_modulation_freq_;
        ROS_INFO ("[SwissRangerTestNode::setInternalParameters::Swissranger] Modulation frequency changed to %d", sr_modulation_freq_);
        sr_.setModulationFrequency (sr_modulation_freq_);
      }

      if (sr_amp_threshold_ != sr_amp_threshold_prev_)
      {
        sr_amp_threshold_prev_ = sr_amp_threshold_;
        ROS_INFO ("[SwissRangerTestNode::setInternalParameters::Swissranger] Amplitude threshold changed to %d", sr_amp_threshold_);
        sr_.setAmplitudeThreshold (sr_amp_threshold_);
      }
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Save Swissranger images to disk
    void
      saveSRImages (deprecated_msgs::ImageArray sr_msg_images_, int img_count)
    {
      char fn[80];
      CvSize sr_size = cvSize (sr_msg_images_.images[0].width, sr_msg_images_.images[0].height);
      IplImage *image_sr = cvCreateImage (sr_size, IPL_DEPTH_16U, 1);

      image_sr->imageData = (char*)&(sr_msg_images_.images[0].data[0]);
      sprintf (fn, "%04i-sr4k-distance.png", img_count);
      cvSaveImage (fn, image_sr);

      image_sr->imageData = (char*)&(sr_msg_images_.images[1].data[0]);
      sprintf (fn, "%04i-sr4k-intensity.png", img_count);
      cvSaveImage (fn, image_sr);

      image_sr->imageData = (char*)&(sr_msg_images_.images[2].data[0]);
      sprintf (fn, "%04i-sr4k-confidence.png", img_count);
      cvSaveImage (fn, image_sr);

      cvReleaseImage (&image_sr);
    }

   ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
      char fn[80];
      int img_count = 1;

      while (1)
      {
//        usleep (100000);
        
        // Change certain parameters in the cameras, based on values from the parameter server
        getParametersFromServer ();
        setInternalParameters ();
        
        try
        {
          // Read data from the SwissRanger
          sr_.readData (sr_msg_cloud_, sr_msg_images_);
        } catch (swissranger::Exception& e) {
          ROS_WARN("[SwissRangerTestNode] Exception thrown while trying to read data.\n%s", e.what ());
          continue;
        }
        
        if (dump_to_disk_)
        {
          ROS_INFO ("Saving data to disk, frame number %i", img_count);
          saveSRImages (sr_msg_images_, img_count);
          sprintf (fn, "%04i-sr4k.pcd", img_count);
          cloud_io::savePCDFileBinary (fn, sr_msg_cloud_);
        } // dump_to_disk
          
        // Publish it
        sr_msg_cloud_.header.frame_id = "base_link";
        node_.publish ("cloud_sr", sr_msg_cloud_);
        node_.publish ("images_sr", sr_msg_images_);

        // Bump the frame count by 1
        img_count++;
      }

      return (true);
    }

  
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);
  ros::Node ros_node ("swissranger_test_node");

  SwissRangerTestNode c (ros_node);

  if (c.start () == 0)
    c.spin ();

  return (0);
}
/* ]--- */
