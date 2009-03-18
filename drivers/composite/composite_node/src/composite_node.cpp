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

@b composite_node is a driver connecting to three different sensors: a Videre
Design STOC (Stereo-On-A-Chip), a SwissRanger SRx000, and a FLIR Thermal camera.

<hr>

@section information Information

The driver depends on a list of 3rd party libraries to compile correctly:
- stoc_driver depends on libsvs (Small Vision System - commercial license)
- flir_driver depends on libunicap (GPL)
- swissranger_driver depends on libusbsr (commercial license ?)

<hr>

@section usage Usage
@verbatim
$ composite_node [standard ROS args]
@endverbatim

@par Example

@verbatim
$ composite_node
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name / type):
- @b "cloud_sr"/<a href="../../std_msgs/html/classstd__msgs_1_1PointCloud.html">PointCloud</a> : Point cloud data from the SwissRanger.
- @b "cloud_stoc"/<a href="../../std_msgs/html/classstd__msgs_1_1PointCloud.html">PointCloud</a> : Point cloud data from the STOC.
- @b "images_sr"/<a href="../../std_msgs/html/classstd__msgs_1_1ImageArray.html">ImageArray</a> : Distance and intensity camera images from the SwissRanger
- @b "images_stoc"/<a href="../../std_msgs/html/classstd__msgs_1_1ImageArray.html">ImageArray</a> : Left, right, and disparity camera images from the STOC.
- @b "image_flir"/<a href="../../std_msgs/html/classstd__msgs_1_1Image.html">Image</a> : Thermal camera image from the FLIR.

<hr>

@section parameters ROS parameters

Reads the following parameters from the parameter server

- @b "sr_auto_illumination"     : @b [int] the angle of the first range measurement in degrees (Default: -90.0)
- @b "sr_integration_time"      : @b [int] the angle of the last range measurement in degrees (Default: 90.0)
- @b "sr_modulation_freq"       : @b [int] the number of adjascent range measurements to cluster into a single reading (Default: 1)
- @b "sr_amp_threshold"         : @b [int] the number of scans to skip between each measured scan (Default: 1)

 **/

// ROS core
#include "ros/node.h"
#include "ros/time.h"
#include "ros/common.h"

#include "std_msgs/PointCloud.h"
#include "std_msgs/ImageArray.h"
#include "std_msgs/Image.h"

// OpenCV (for dumping data to disk)
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <fstream>

// Drivers used
#include "stoc.h"
#include "swissranger.h"
#include "flir.h"

#include "tf/tf.h"

#define DEFAULT_INT_VALUE INT_MIN
#define DEFAULT_DBL_VALUE DBL_MIN

using namespace std;

class CompositeNode: public ros::node
{
  public:

    int sr_auto_illumination_, sr_integration_time_, sr_modulation_freq_, sr_amp_threshold_;
    int stoc_speckle_diff_, stoc_speckle_size_, stoc_horopter_, stoc_corrsize_, stoc_unique_, stoc_tex_thresh_, stoc_ndisp_;
    bool stoc_multiproc_;
    double stoc_zmax_;

    int sr_auto_illumination_prev_, sr_integration_time_prev_, sr_modulation_freq_prev_, sr_amp_threshold_prev_;
    int stoc_speckle_diff_prev_, stoc_speckle_size_prev_, stoc_horopter_prev_, stoc_corrsize_prev_, stoc_unique_prev_, stoc_tex_thresh_prev_, stoc_ndisp_prev_;
    bool stoc_multiproc_prev_;
    double stoc_zmax_prev_;

    // ROS messages
    std_msgs::PointCloud sr_msg_cloud_, stoc_msg_cloud_;
    std_msgs::ImageArray sr_msg_images_, stoc_msg_images_;
    std_msgs::Image flir_msg_image_, stoc_msg_left_image_;

    swissranger::SwissRanger sr_;
    stoc::STOC stoc_;
    unicap::FLIR flir_;

    // Save data to disk ?
    bool dump_to_disk_, sr_enable_, stoc_enable_, flir_enable_, stoc_just_left_;
    
    int composite_snapshot_mode_, composite_snapshot_mode_prev_;

    CompositeNode () : ros::node ("composite"), dump_to_disk_(false), 
                                                sr_enable_(true), stoc_enable_(true), flir_enable_(true), 
                                                stoc_just_left_(false)
    {
      // Initialize internal parameters
      param ("~sr_auto_illumination", sr_auto_illumination_, DEFAULT_INT_VALUE);
      param ("~sr_integration_time", sr_integration_time_, DEFAULT_INT_VALUE);
      param ("~sr_modulation_freq", sr_modulation_freq_, DEFAULT_INT_VALUE);
      param ("~sr_amp_threshold", sr_amp_threshold_, DEFAULT_INT_VALUE);
      
      sr_auto_illumination_prev_ = sr_integration_time_prev_ = sr_modulation_freq_prev_ = sr_amp_threshold_prev_ = DEFAULT_INT_VALUE;
      
      param ("~stoc_speckle_diff", stoc_speckle_diff_, DEFAULT_INT_VALUE);
      param ("~stoc_speckle_size", stoc_speckle_size_, DEFAULT_INT_VALUE);
      param ("~stoc_horopter", stoc_horopter_, DEFAULT_INT_VALUE);
      param ("~stoc_corrsize", stoc_corrsize_, DEFAULT_INT_VALUE);
      param ("~stoc_unique", stoc_unique_, DEFAULT_INT_VALUE);
      param ("~stoc_tex_thresh", stoc_tex_thresh_, DEFAULT_INT_VALUE);
      param ("~stoc_ndisp", stoc_ndisp_, DEFAULT_INT_VALUE);

      stoc_speckle_diff_prev_ = stoc_speckle_size_prev_ = stoc_horopter_prev_ = stoc_corrsize_prev_ = stoc_unique_prev_ = stoc_tex_thresh_prev_ = stoc_ndisp_prev_ = DEFAULT_INT_VALUE;
      
//      param ("~stoc_multiproc", stoc_multiproc_, true);
      param ("~stoc_zmax", stoc_zmax_, DEFAULT_DBL_VALUE);
      
      stoc_zmax_prev_ = DEFAULT_DBL_VALUE;
      
      composite_snapshot_mode_ = composite_snapshot_mode_prev_ = 0;

      // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
      advertise<std_msgs::PointCloud>("cloud_sr", 1);
      advertise<std_msgs::PointCloud>("cloud_stoc", 1);

      advertise<std_msgs::ImageArray>("images_sr", 1);
      if (stoc_just_left_)
        advertise<std_msgs::Image>("image_stoc_left", 1);
      else
        advertise<std_msgs::ImageArray>("images_stoc", 1);
        
      advertise<std_msgs::Image>("image_flir", 1);
    }

    ~CompositeNode ()
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
          ROS_INFO ("[CompositeNode::SwissRanger] Connected to device with ID: %s", sr_.device_id_.c_str ());
          ROS_INFO ("[CompositeNode::SwissRanger] libusbSR version: %s", sr_.lib_version_.c_str ());
        }
      } catch (swissranger::Exception& e) {
        ROS_ERROR("[CompositeNode::SwissRanger] Exception thrown while connecting to the sensor.\n%s", e.what ());
        if (sr_enable_)
          return (-1);
      }

//      sr_.setAutoIllumination (1);
//      sr_.setAmplitudeThreshold (100.0);
      sr_.setModulationFrequency (MF_30MHz);			// For the love of God, please do not change this.
      ROS_INFO ("[CompositeNode::SwissRanger] Modulation frequency is: %d", sr_.getModulationFrequency ());
      ROS_INFO ("[CompositeNode::SwissRanger] Amplitude threshold is: %d", sr_.getAmplitudeThreshold ());

      // Open the STOC device
      try
      {
        if (stoc_.open () == 0)
          ROS_INFO ("[CompositeNode::STOC] Connected to device with ID: %s", stoc_.device_id_.c_str ());
      } catch (stoc::Exception& e) {
        ROS_ERROR("[CompositeNode::STOC] Exception thrown while connecting to the sensor.\n%s", e.what ());
        if (stoc_enable_)
          return (-1);
      }

      // Open the FLIR device
      try
      {
        if (flir_.open () == 0)
          ROS_INFO ("[CompositeNode::FLIR] Connected to device at %i: %s", flir_.device_id_, flir_.device_.identifier);
      } catch (unicap::Exception& e) {
        ROS_ERROR("[CompositeNode::FLIR] Exception thrown while connecting to the sensor.\n%s", e.what ());
        if (flir_enable_)
          return (-1);
      }

      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Stop
    int
      stop ()
    {
      // Close the SwissRanger device
      if (sr_.close () == 0)
        ROS_INFO ("[CompositeNode::SwissRanger] Driver shut down successfully");
      else
        return (-1);

      // Close the STOC device
      if (stoc_.close () == 0)
        ROS_INFO ("[CompositeNode::STOC] Driver shut down successfully");
      else
        return (-1);

      // Close the FLIR device
      if (flir_.close () == 0)
        ROS_INFO ("[CompositeNode::FLIR] Driver shut down successfully");
      else
        return (-1);

      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Dump a point cloud to disk
    void
      SaveCloud (char *fn, std_msgs::PointCloud cloud)
    {
      std::ofstream fs;
      fs.precision (5);
      fs.open (fn);

      int nr_pts = cloud.get_pts_size ();
      int dim    = cloud.get_chan_size ();
      fs << "COLUMNS x y z i pid";
      for (int d = 0; d < dim; d++)
        fs << " " << cloud.chan[d].name;
      fs << endl;
      fs << "POINTS " << nr_pts << endl;
      fs << "DATA ascii" << endl;
      
      for (int i = 0; i < nr_pts; i++)
      {
        fs << cloud.pts[i].x << " " << cloud.pts[i].y << " " << cloud.pts[i].z;
        for (int d = 0; d < dim; d++)
          fs << " " << cloud.chan[d].vals[i];
        fs << endl;
      }
      fs.close ();
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Obtain a set of interesting parameters from the parameter server
    void
      getParametersFromServer ()
    {
      // Composite related parameters
      if (has_param ("~composite_snapshot"))
        get_param ("~composite_snapshot", composite_snapshot_mode_);
      
      // Swissranger related parameters
      if (has_param ("~sr_auto_illumination"))
        get_param ("~sr_auto_illumination", sr_auto_illumination_);
      if (has_param ("~sr_integration_time"))
        get_param ("~sr_integration_time", sr_integration_time_);
      if (has_param ("~sr_modulation_freq"))
        get_param ("~sr_modulation_freq", sr_modulation_freq_);
      if (has_param ("~sr_amp_threshold"))
        get_param ("~sr_amp_threshold", sr_amp_threshold_);
        
      // Stereo related parameters (disparity calculation parameters)
      if (has_param ("~stoc_speckle_diff"))
        get_param ("~stoc_speckle_diff", stoc_speckle_diff_);
      if (has_param ("~stoc_speckle_size"))
        get_param ("~stoc_speckle_size", stoc_speckle_size_);
      if (has_param ("~stoc_horopter"))
        get_param ("~stoc_horopter", stoc_horopter_);
      if (has_param ("~stoc_corrsize")) 
        get_param ("~stoc_corrsize", stoc_corrsize_);
      if (has_param ("~stoc_unique"))
        get_param ("~stoc_unique", stoc_unique_);
      if (has_param ("~stoc_tex_thresh"))
        get_param ("~stoc_tex_thresh", stoc_tex_thresh_);
      if (has_param ("~stoc_ndisp"))
        get_param ("~stoc_ndisp", stoc_ndisp_);

//      if (has_param ("~stoc_multiproc"))
//        get_param ("~stoc_multiproc", stoc_multiproc_);
      if (has_param ("~stoc_zmax"))
        get_param ("~stoc_zmax", stoc_zmax_);
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
        ROS_INFO ("[CompositeNode::setInternalParameters::Swissranger] Auto illumination settings changed to %d", sr_auto_illumination_);
        sr_.setAutoIllumination (sr_auto_illumination_);
      }

      if (sr_integration_time_ != sr_integration_time_prev_)
      {
        sr_integration_time_prev_ = sr_integration_time_;
        ROS_INFO ("[CompositeNode::setInternalParameters::Swissranger] Integration time changed to %d", sr_integration_time_);
        sr_.setIntegrationTime (sr_integration_time_);
      }

      if (sr_modulation_freq_ != sr_modulation_freq_prev_)
      {
        sr_modulation_freq_prev_ = sr_modulation_freq_;
        ROS_INFO ("[CompositeNode::setInternalParameters::Swissranger] Modulation frequency changed to %d", sr_modulation_freq_);
        sr_.setModulationFrequency (sr_modulation_freq_);
      }

      if (sr_amp_threshold_ != sr_amp_threshold_prev_)
      {
        sr_amp_threshold_prev_ = sr_amp_threshold_;
        ROS_INFO ("[CompositeNode::setInternalParameters::Swissranger] Amplitude threshold changed to %d", sr_amp_threshold_);
        sr_.setAmplitudeThreshold (sr_amp_threshold_);
      }
      
      // Stereo related parameters
      if (stoc_zmax_ != stoc_zmax_prev_) 
      {
        stoc_zmax_prev_ = stoc_zmax_;
        ROS_INFO ("[CompositeNode::setInternalParameters::STOC] Z-max cutoff value changed to %g", stoc_zmax_);
        stoc_.z_max_     = stoc_zmax_;
      }

      if (stoc_ndisp_ != stoc_ndisp_prev_) 
      {
        stoc_ndisp_prev_ = stoc_ndisp_;
        ROS_INFO ("[CompositeNode::setInternalParameters::STOC] Number of disparities changed to %d", stoc_ndisp_);
        stoc_.setNDisp (stoc_ndisp_);
      }

      if (stoc_tex_thresh_ != stoc_tex_thresh_prev_) 
      {
        stoc_tex_thresh_prev_ = stoc_tex_thresh_;
        ROS_INFO ("[CompositeNode::setInternalParameters::STOC] Texture filter threshold changed to %d", stoc_tex_thresh_);
        stoc_.setTexThresh (stoc_tex_thresh_);
      }

      if (stoc_unique_ != stoc_unique_prev_) 
      {
        stoc_unique_prev_ = stoc_unique_;
        ROS_INFO ("[CompositeNode::setInternalParameters::STOC] Uniqueness filter threshold changed to %d", stoc_unique_);
        stoc_.setUnique (stoc_unique_);
      }

      if (stoc_corrsize_ != stoc_corrsize_prev_) 
      {
        stoc_corrsize_prev_ = stoc_corrsize_;
        ROS_INFO ("[CompositeNode::setInternalParameters::STOC] Correlation window size changed to %d", stoc_corrsize_);
        stoc_.setCorrSize (stoc_corrsize_);
      }

      if (stoc_horopter_ != stoc_horopter_prev_) 
      {
        stoc_horopter_prev_ = stoc_horopter_;
        ROS_INFO ("[CompositeNode::setInternalParameters::STOC] Horopter (X Offset) changed to %d", stoc_horopter_);
        stoc_.setHoropter (stoc_horopter_);
      }

      if (stoc_speckle_size_ != stoc_speckle_size_prev_) 
      {
        stoc_speckle_size_prev_ = stoc_speckle_size_;
        ROS_INFO ("[CompositeNode::setInternalParameters::STOC] Minimum disparity region size changed to %d", stoc_speckle_size_);
        stoc_.setSpeckleSize (stoc_speckle_size_);
      }

      if (stoc_speckle_diff_ != stoc_speckle_diff_prev_) 
      {
        stoc_speckle_diff_prev_ = stoc_speckle_diff_;
        ROS_INFO ("[CompositeNode::setInternalParameters::STOC] Disparity region neighbor difference changed to %d", stoc_speckle_diff_);
        stoc_.setSpeckleDiff (stoc_speckle_diff_);
      }
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Save Swissranger images to disk
    void
      saveSRImages (std_msgs::ImageArray sr_msg_images_, int img_count)
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
    // Save the STOC's left channel image to disk
    void
      saveSTOCLeftImage (std_msgs::Image stoc_msg_left_image_, int img_count)
    {
      char fn[80];
      CvSize stoc_size = cvSize (stoc_msg_left_image_.width, stoc_msg_left_image_.height);
      IplImage *image_stoc;

      // Left channel
      if (stoc_msg_left_image_.colorspace == "rgb24")
        image_stoc = cvCreateImage (stoc_size, IPL_DEPTH_8U, 3);
      else
        image_stoc = cvCreateImage (stoc_size, IPL_DEPTH_8U, 1);

      image_stoc->imageData = (char*)&(stoc_msg_left_image_.data[0]);
      sprintf (fn, "%04i-stoc-left.png", img_count);
      
      cvSaveImage (fn, image_stoc);

      cvReleaseImage (&image_stoc);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Save STOC images to disk (left + right only)
    void
      saveSTOCImages (std_msgs::ImageArray stoc_msg_images_, int img_count)
    {
      char fn[80];
      CvSize stoc_size = cvSize (stoc_msg_images_.images[0].width, stoc_msg_images_.images[0].height);
      IplImage *image_stoc;

      // Left channel
      if (stoc_msg_images_.images[0].colorspace == "rgb24")
        image_stoc = cvCreateImage (stoc_size, IPL_DEPTH_8U, 3);
      else
        image_stoc = cvCreateImage (stoc_size, IPL_DEPTH_8U, 1);

      image_stoc->imageData = (char*)&(stoc_msg_images_.images[0].data[0]);
      sprintf (fn, "%04i-stoc-left.png", img_count);
      
      cvSaveImage (fn, image_stoc);

      cvReleaseImage (&image_stoc);  // assume right != left, size-wise

      // Right channel
      if (stoc_msg_images_.images[1].colorspace == "rgb24")
        image_stoc = cvCreateImage (stoc_size, IPL_DEPTH_8U, 3);
      else
        image_stoc = cvCreateImage (stoc_size, IPL_DEPTH_8U, 1);

      image_stoc->imageData = (char*)&(stoc_msg_images_.images[1].data[0]);
      sprintf (fn, "%04i-stoc-right.png", img_count);
      
      cvSaveImage (fn, image_stoc);

      cvReleaseImage (&image_stoc);
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Rotate an image with 180 in place
    void
      rotateImage180_16bit (std_msgs::Image &image)
    {
      // Create a temporary data array
      vector<uint8_t> imgdata (image.get_data_size ());
      memcpy (&(imgdata[0]), &(image.data[0]), image.get_data_size ());
      
      for (unsigned int u = 0; u < image.height; u++)
      {
        int nu_ = (image.height - u - 1) * image.width;
        int u_  =                 u      * image.width;
        for (unsigned int v = 0; v < image.width; v++)
        {
          int nv = image.width - v - 1;
          image.data[(nu_ + nv) * 2 + 0] = imgdata[(u_ + v) * 2 + 0];
          image.data[(nu_ + nv) * 2 + 1] = imgdata[(u_ + v) * 2 + 1];
        }
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Rotate an image with 180 in place
    void
      rotateImage180_16bit (std_msgs::ImageArray &images)
    {
      // Create a temporary data array
      for (unsigned int cp = 0; cp < images.get_images_size (); cp++)
      {
        vector<uint8_t> imgdata (images.images[cp].get_data_size ());
        memcpy (&(imgdata[0]), &(images.images[cp].data[0]), images.images[cp].get_data_size ());
        
        for (unsigned int u = 0; u < images.images[cp].height; u++)
        {
          int nu_ = (images.images[cp].height - u - 1) * images.images[cp].width;
          int u_  =                             u      * images.images[cp].width;
          for (unsigned int v = 0; v < images.images[cp].width; v++)
          {
            int nv = images.images[cp].width - v - 1;
            images.images[cp].data[(nu_ + nv) * 2 + 0] = imgdata[(u_ + v) * 2 + 0];
            images.images[cp].data[(nu_ + nv) * 2 + 1] = imgdata[(u_ + v) * 2 + 1];
          }
        }
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
      char fn[80];
      int img_count = 1;

      while (ok ())
      {
//        usleep (100000);
        
        // Change certain parameters in the cameras, based on values from the parameter server
        getParametersFromServer ();
        setInternalParameters ();
        
        try
        {
          // Read data from the SwissRanger
          if (sr_enable_)
            sr_.readData (sr_msg_cloud_, sr_msg_images_);
          // Read data from the SwissRanger
          if (stoc_enable_)
          {
            if (stoc_just_left_)
              stoc_.readDataLeft (stoc_msg_cloud_, stoc_msg_left_image_);
            else
              stoc_.readData (stoc_msg_cloud_, stoc_msg_images_);
          }
          // Read data from the FLIR Thermal
          if (flir_enable_)
            flir_.readData (flir_msg_image_);
        } catch (swissranger::Exception& e) {
          ROS_WARN("[CompositeNode] Exception thrown while trying to read data.\n%s", e.what ());
          continue;
        } catch (stoc::Exception& e) {
          ROS_WARN("[CompositeNode] Exception thrown while trying to read data.\n%s", e.what ());
          continue;
        } catch (unicap::Exception& e) {
          ROS_WARN("[CompositeNode] Exception thrown while trying to read data.\n%s", e.what ());
          continue;
        }
        
        // If composite snapshot enabled
        if (composite_snapshot_mode_ != composite_snapshot_mode_prev_)
        {
          composite_snapshot_mode_prev_ = composite_snapshot_mode_;
          ROS_INFO ("[CompositeNode] Snapshot request received!");
          
          if (sr_enable_)
            saveSRImages (sr_msg_images_, img_count);
          if (stoc_enable_)
          {
            if (stoc_just_left_)
              saveSTOCLeftImage (stoc_msg_left_image_, img_count);
            else               // save all 3 (left, right, disparity)
              saveSTOCImages (stoc_msg_images_, img_count);
          }
        }
        
        // SwissRanger enabled ?
        if (sr_enable_)
        {
          if (dump_to_disk_)
          {
            saveSRImages (sr_msg_images_, img_count);
            sprintf (fn, "%04i-sr4k.pcd", img_count);
            SaveCloud (fn, sr_msg_cloud_);
          } // dump_to_disk
          
          // Publish it
          publish ("cloud_sr", sr_msg_cloud_);
//          rotateImage180_16bit (sr_msg_images_.images[0]);
//          rotateImage180_16bit (sr_msg_images_.images[1]);
//          rotateImage180_16bit (sr_msg_images_.images[2]);
          publish ("images_sr", sr_msg_images_);
        } // SR

        // Videre STOC enable d?
        if (stoc_enable_)
        {
          if (dump_to_disk_)
          {
            if (stoc_just_left_)
              saveSTOCLeftImage (stoc_msg_left_image_, img_count);
            else               // save all 3 (left, right, disparity)
              saveSTOCImages (stoc_msg_images_, img_count);

//            sprintf (fn, "%04i-stoc.pcd", img_count);
//            SaveCloud (fn, stoc_msg_cloud_);
          } // dump_to_disk

          // Publish it
          publish ("cloud_stoc", stoc_msg_cloud_);
          if (stoc_just_left_)
            publish ("image_stoc_left", stoc_msg_left_image_);
          else
            publish ("images_stoc", stoc_msg_images_);
        } // STOC


        // Thermal FLIR enabled ?
        if (flir_enable_)
        {
          if (dump_to_disk_)
          {
            CvSize flir_size = cvSize (flir_msg_image_.width, flir_msg_image_.height);
            IplImage *image_flir = cvCreateImage (flir_size, IPL_DEPTH_16U, 1);

            image_flir->imageData = (char*)&(flir_msg_image_.data[0]);
            sprintf (fn, "%04i-thermal.png", img_count);
            cvSaveImage (fn, image_flir);
            cvReleaseImage (&image_flir);
          } // dump_to_disk

          // Publish it
          publish ("image_flir", flir_msg_image_);
        } // FLIR

        // Bump the frame count by 1
        img_count++;
      }

      return true;
    }

  
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  CompositeNode c;
  c.dump_to_disk_ = false;
  c.sr_enable_    = false;
  c.flir_enable_  = false;
  c.stoc_enable_  = false; //c.stoc_just_left_ = true;

  if (c.start () == 0)
    c.spin ();

  ros::fini ();

  return (0);
}
/* ]--- */
