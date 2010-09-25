/* 
 * Copyright (c) 2010, Dejan Pangercic <dejan.pangercic@cs.tum.edu>, 
 Zoltan-Csaba Marton <marton@cs.tum.edu>, Nico Blodow <blodow@cs.tum.edu>
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef IMAGE_ALGOS_COLOR_FIND_HSV_H
#define IMAGE_ALGOS_COLOR_FIND_HSV_H
#include <fstream>

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <image_algos/image_algos.h>
#include <ias_table_msgs/TableObject.h>
//in image_algos.h:
//#include "sensor_msgs/Image.h"

#include "yaml-cpp/yaml.h"
#include "cv.h"  
#include "cv_bridge/CvBridge.h"
#include "image_transport/image_transport.h"

namespace image_algos
{

class ColorFindHSV : public ImageAlgo
{
public:
  ColorFindHSV () 
  { 
    colors_yaml_file_ = std::string ("data/colors.yaml");
  };
  typedef sensor_msgs::Image InputType;
  typedef ias_table_msgs::TableObject OutputType;

  static std::string default_input_topic ()
    {return std::string ("image");}

  static std::string default_output_topic ()
    {return std::string ("color");};

  static std::string default_node_name () 
    {return std::string ("find_color_hsv_node");};

  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>);
  boost::shared_ptr<const OutputType> output ();

protected:
  ros::NodeHandle nh_;
  struct threshold_values
  {
    //saturation threshold
    int sthreshold_min, sthreshold_max;  
    //hue threshold
    double hupper, hlower;
    //how many pixels of this color
    int pixel_frequency;
  } threshold_values_;
  
  std::map<std::string, threshold_values> color_values_map_;
  //output
  boost::shared_ptr<OutputType> color_;
  //learnt colors
  std::string colors_yaml_file_;
  //ros Image to IplImage conversion
  sensor_msgs::CvBridge bridge_;
  //result image publisher (if debug on)
  ros::Publisher result_image_pub_;
  //image_transport::ImageTransport it_;
};

}//namespace image_algos
#endif


