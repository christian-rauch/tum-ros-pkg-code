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
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/common.h>

#include <robot_msgs/PointCloud.h>
#include <deprecated_msgs/ImageArray.h>
#include <swissranger_srv/SRDumpToggle.h>

// libpng + point_cloud_mapping (for dumping data to disk)
#include <png.h>
#include <point_cloud_mapping/cloud_io.h>

// Drivers used
#include "swissranger.h"

#include <boost/thread/mutex.hpp>

#define DEFAULT_INT_VALUE INT_MIN
#define DEFAULT_DBL_VALUE DBL_MIN

using namespace std;
using namespace ros;
using namespace robot_msgs;
using namespace deprecated_msgs;
using namespace swissranger_srv;

class SwissRangerTestNode
{
  protected:
    ros::NodeHandle nh_;
  public:

    int sr_auto_illumination_, sr_integration_time_, sr_modulation_freq_, sr_amp_threshold_;
    int sr_auto_illumination_prev_, sr_integration_time_prev_, sr_modulation_freq_prev_, sr_amp_threshold_prev_;

    // ROS messages
    PointCloud sr_msg_cloud_;
    ImageArray sr_msg_images_;

    swissranger::SwissRanger sr_;
    
    Publisher cloud_pub_, images_pub_;
    ServiceServer snapshot_service_;
    
    boost::mutex m_lock_;

    bool dump_to_disk_;
    int img_count_, snap_count_;

    SwissRangerTestNode () : dump_to_disk_ (false), img_count_ (1), snap_count_ (1)
    {
      // Initialize internal parameters
      nh_.param ("~sr_auto_illumination", sr_auto_illumination_, DEFAULT_INT_VALUE);
      nh_.param ("~sr_integration_time", sr_integration_time_, DEFAULT_INT_VALUE);
      nh_.param ("~sr_modulation_freq", sr_modulation_freq_, DEFAULT_INT_VALUE);
      nh_.param ("~sr_amp_threshold", sr_amp_threshold_, DEFAULT_INT_VALUE);

      nh_.param ("~sr_dump_to_disk", dump_to_disk_, false);
      
      sr_auto_illumination_prev_ = sr_integration_time_prev_ = sr_modulation_freq_prev_ = sr_amp_threshold_prev_ = DEFAULT_INT_VALUE;
      
      // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
      cloud_pub_  = nh_.advertise<PointCloud>("cloud_sr", 1);
      images_pub_ = nh_.advertise<ImageArray>("images_sr", 1);
      snapshot_service_ = nh_.advertiseService("acquire_snapshot_sr", &SwissRangerTestNode::snapshot, this);
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
      nh_.getParam ("~sr_auto_illumination", sr_auto_illumination_);
      nh_.getParam ("~sr_integration_time", sr_integration_time_);
      nh_.getParam ("~sr_modulation_freq", sr_modulation_freq_);
      nh_.getParam ("~sr_amp_threshold", sr_amp_threshold_);
      nh_.getParam ("~sr_dump_to_disk", dump_to_disk_);
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
    // Assemble a PNG file using libpng
    bool
      writePNG (const char* file_name, const Image &img) 
    {
      // Create the output file
      FILE *fp = fopen (file_name, "wb");
      if (!fp)
        return (false);
      //  Create and initialize the png_struct
      png_structp png_ptr = png_create_write_struct (PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
      if (!png_ptr)
        return (false);
        
      // Allocate/initialize the image information data
      png_infop info_ptr = png_create_info_struct (png_ptr);
      if (!info_ptr)
        return (false);
      
      if (setjmp (png_jmpbuf (png_ptr)))
        return (false);
       
      // Set up the output control
      png_init_io (png_ptr, fp);
      
      // Set filtering/compression parameters
      png_set_filter (png_ptr, PNG_FILTER_TYPE_BASE, PNG_FILTER_SUB);
      png_set_compression_level (png_ptr, Z_BEST_SPEED);
      png_set_compression_strategy (png_ptr, Z_HUFFMAN_ONLY);
      
      // Set up the header
      int depth = 16, bpp = 2;
      png_set_IHDR (png_ptr, info_ptr, img.width, img.height, depth, 
                    PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

      // Write the header
      png_write_info (png_ptr, info_ptr);

      png_set_swap (png_ptr);
      
      png_bytep *row_pointers = new png_bytep[img.height];
      if (!row_pointers)
        return (false);

      for (unsigned int i = 0; i < img.height; i++)
        row_pointers[i] = (png_bytep)(unsigned char*)&img.data[0] + (i * img.width * bpp);
      
      // Write the actual data
      png_write_image (png_ptr, row_pointers);
      
      // Close, delete, and return      
      png_write_end (png_ptr, info_ptr);
      delete [] row_pointers;
      png_destroy_write_struct (&png_ptr, &info_ptr);
      fclose (fp);
              
      return (true);
    }

   ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
      char fn[80];

      while (nh_.ok ())
      {
        // Change certain parameters in the cameras, based on values from the parameter server
        getParametersFromServer ();
        setInternalParameters ();
        
        try
        {
          // Read data from the SwissRanger
          m_lock_.lock ();
          sr_.readData (sr_msg_cloud_, sr_msg_images_);
          m_lock_.unlock ();
        } catch (swissranger::Exception& e) {
          ROS_WARN("[SwissRangerTestNode] Exception thrown while trying to read data.\n%s", e.what ());
          continue;
        }
        
        if (dump_to_disk_)
        {
          ROS_INFO ("Saving data to disk, frame number %i", img_count_);
          
          sprintf (fn, "/tmp/%04i-sr4k.pcd", img_count_);
          cloud_io::savePCDFileBinary (fn, sr_msg_cloud_);

          sprintf (fn, "/tmp/%04i-sr4k-distance.png", img_count_);
          writePNG (fn, sr_msg_images_.images[0]);
          sprintf (fn, "/tmp/%04i-sr4k-intensity.png", img_count_);
          writePNG (fn, sr_msg_images_.images[1]);
          sprintf (fn, "/tmp/%04i-sr4k-confidence.png", img_count_);
          writePNG (fn, sr_msg_images_.images[2]);
          
          img_count_++;
        } // dump_to_disk
          
        // Publish it
        sr_msg_cloud_.header.frame_id = "base_link";
        cloud_pub_.publish (sr_msg_cloud_);
        images_pub_.publish (sr_msg_images_);
        
        ros::spinOnce ();
      }

      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      snapshot (SRDumpToggle::Request &req, SRDumpToggle::Response &resp)
    {
      char fn[80];
      
      sprintf (fn, "/tmp/snapshot-%04i-sr4k.pcd", snap_count_);
      ROS_INFO ("Snapshot enabled... saving data to disk: %s", fn);

      m_lock_.lock ();
      cloud_io::savePCDFileBinary (fn, sr_msg_cloud_);
      
      sprintf (fn, "/tmp/snapshot-%04i-sr4k-distance.png", snap_count_);
      writePNG (fn, sr_msg_images_.images[0]);
      sprintf (fn, "/tmp/snapshot-%04i-sr4k-intensity.png", snap_count_);
      writePNG (fn, sr_msg_images_.images[1]);
      sprintf (fn, "/tmp/snapshot-%04i-sr4k-confidence.png", snap_count_);
      writePNG (fn, sr_msg_images_.images[2]);
      m_lock_.unlock ();
      
      snap_count_++;
      resp.counter = snap_count_;

      return (true);
    }

};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "swissranger_test_node");

  SwissRangerTestNode c;

  if (c.start () == 0)
    c.spin ();

  return (0);
}
/* ]--- */
