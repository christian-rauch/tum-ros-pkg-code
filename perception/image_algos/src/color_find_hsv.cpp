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


#include <image_algos/color_find_hsv.h>
 #include "highgui.h"  

using namespace image_algos;

void ColorFindHSV::init (ros::NodeHandle& nh)
{
  nh_ = nh;
  nh_.param("colors_yaml_file", colors_yaml_file_, colors_yaml_file_);
  result_image_pub_ = nh_.advertise<sensor_msgs::Image>("result", 0);
  if (colors_yaml_file_ == "")
  {
    ROS_ERROR("You have to provide file with colors");
    exit(0);
  }
  std::ifstream fin(colors_yaml_file_.c_str());
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  threshold_values tv;
  for(unsigned i=0;i<doc.size();i++) 
  {
    
    doc[i]["sthreshold_min"] >> tv.sthreshold_min;
    doc[i]["sthreshold_max"] >> tv.sthreshold_max;
    doc[i]["hlower"] >> tv.hlower;
    doc[i]["hupper"] >> tv.hupper;
    color_values_map_[doc[i]["color"]] = tv;
  }
  ROS_INFO("[ColorFindHSV:] Map size: %ld", color_values_map_.size());
}


void ColorFindHSV::pre  ()
{
  color_ = boost::shared_ptr<OutputType> (new OutputType);
}


void ColorFindHSV::post ()
{
  //Nothing TODO
}


std::vector<std::string> ColorFindHSV::requires ()
{
  return std::vector<std::string>();
}


std::vector<std::string> ColorFindHSV::provides ()
{
  return std::vector<std::string>();
}


std::string ColorFindHSV::process (const boost::shared_ptr<const InputType> input)
{
  ROS_INFO ("[ColorFindHSV:] Image message received on %s with width %d, height %d", default_input_topic().c_str (), input->width, input->height);
  int sthreshold_max, sthreshold_min;  
  double hupper, hlower;  
  int i, j;
  int height,width,step,channels;
  int stepmono,channelsmono;
  IplImage *frame = NULL;
  try
  {
    frame = bridge_.imgMsgToCv(input, "bgr8");
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("Error converting top openCV image");
  }
  

  IplImage *imageHSV = cvCloneImage(frame);
  IplImage *result = cvCreateImage(cvGetSize(frame), 8, 1);
  /*Converting the color space of the image....*/  
  cvCvtColor(frame,imageHSV,CV_BGR2HSV);
  //cvNamedWindow("image", CV_WINDOW_AUTOSIZE);  
  //cvShowImage("image", frame);  
  //cvWaitKey(3000);
  
  uchar *data, *datamono;
  data = (uchar *)imageHSV->imageData;  
  height = frame->height;  
  width = frame->width;  
  step = frame->widthStep;  
  channels = frame->nChannels;    
  datamono = (uchar *) result->imageData;
  stepmono = result->widthStep;
  channelsmono = result->nChannels;
  cvZero(result);
  std::map<std::string, threshold_values>::iterator it;
  for ( it=color_values_map_.begin(); it != color_values_map_.end(); it++)
  {
    cvZero(result);
    //set intensities to 0 
    i=j=0;
    (*it).second.pixel_frequency = 0;
    sthreshold_min = (*it).second.sthreshold_min;
    sthreshold_max = (*it).second.sthreshold_max;
    hupper = (*it).second.hupper;
    hlower = (*it).second.hlower;
    //iterate over the image
    for (i = 0; i < height; i++) 
    {
      for (j = 0; j < width; j++) 
      {
        if( 
           (data[(i) * step + j * channels] <= hupper) &&
           (data[(i) * step + j * channels] >= hlower) //&&
           //(data[(i) * step + j * (channels) + 1] > sthreshold_min)  &&
           //(data[(i) * step + j * (channels) + 1] < sthreshold_max)
            )
        {
          //datamono[(i) * stepmono + j * channelsmono] = 255;
          datamono[(i) * stepmono + j * channelsmono] = 255;
        }
      }
    }
    cvErode(result,result,0,1);
    cvDilate( result,result,0,1); 
    i=j=0;
    for (i = 0; i < height; i++) 
    {
      for (j = 0; j < width; j++) 
      {
        if(datamono[(i) * stepmono + j * channelsmono] == 255)

        {
          (*it).second.pixel_frequency++;
        }
      }
    }
    ROS_INFO("[ColorFindHSV:] Color: %s, Nr. of pixels: %d", (*it).first.c_str(), (*it).second.pixel_frequency);
  }

  int max_freq = 0;
  std::string color("");
  for ( it=color_values_map_.begin(); it != color_values_map_.end(); it++)
    if ((*it).second.pixel_frequency > max_freq)
    {
      max_freq = (*it).second.pixel_frequency;
      color = (*it).first;
    }
  ROS_INFO("[ColorFindHSV:] %s", color.c_str());
  if (color != "")
  {
    color_->object_color = color;
    color_->perception_method = ros::this_node::getName();;
    color_->roi = *input;
  }
  try
  {
    result_image_pub_.publish(bridge_.cvToImgMsg(result, "passthrough"));
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("Error converting to ROS Image");
  }

  cvReleaseImage(&imageHSV);
  cvReleaseImage(&result);
  return std::string ("ok");
}


boost::shared_ptr<const ColorFindHSV::OutputType> ColorFindHSV::output () 
{
  return color_;
};


#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <ColorFindHSV> (argc, argv);
}
#endif

