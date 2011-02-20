/* 
 * Copyright (c) 2010, Dejan Pangercic <dejan.pangercic@cs.tum.edu>
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

#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
#include <boost/thread.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include "boost/filesystem.hpp"

using namespace std;
using namespace boost::filesystem;


class ImageConverter {

public:
  string image_file_;
  IplImage *cv_image_;
  double rate_;
  boost::thread spin_thread_;
  std::vector<std::string> fileNames_;
  ImageConverter(ros::NodeHandle &n) :
    n_(n), it_(n_)
  {
    image_pub_ = it_.advertise("image_topic_2",10);
    spin_thread_ = boost::thread (boost::bind (&ros::spin));
  }
  
  ~ImageConverter()
  {
  }

  void get_log_files ( const path & directory, vector <string> &file_list, string suffix=".pcd",
		  bool recurse_into_subdirs = false )
   {
     if( exists( directory ) )
     {
       directory_iterator end ;
       for( directory_iterator iter(directory) ; iter != end ; ++iter )
         if ( is_directory( *iter ) )
         {
           if( recurse_into_subdirs ) get_log_files(*iter,file_list,suffix,recurse_into_subdirs) ;
         }
         else
         {
           int len = iter->string().length();
           int npos =  iter->string().rfind(suffix);
           if((unsigned long)(len - npos) == suffix.length())
             file_list.push_back(iter->string());
         }
     }
   }


  ////////////////////////////////////////////////////////////////////////////////
  // Spin (!)
  bool spin ()
  {
    ros::Rate loop_rate(1/rate_);
    while (n_.ok () && fileNames_.size()!= 0)
      {     
        ROS_INFO ("Publishing data on topic %s.", n_.resolveName ("image_topic_2").c_str ());

        ROS_INFO ("Loading file %s...", fileNames_.back().c_str ());
        cv_image_ = cvLoadImage(fileNames_.back().c_str());
        fileNames_.pop_back();
        try
          {
            image_pub_.publish(bridge_.cvToImgMsg(cv_image_));
          }
        catch (sensor_msgs::CvBridgeException error)
          {
            ROS_ERROR("error");
          }
        
        if (rate_ == 0)  
          break;
        loop_rate.sleep();

      }
    return (true);
  }

protected:

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  image_transport::Publisher image_pub_;

};
int main(int argc, char** argv)
{
  if (argc < 3)
    {
      ROS_ERROR ("Syntax is: %s <path to directory> [time beetween 2 images (in sec)]", argv[0]);
      return (-1);
    }
  ros::init(argc, argv, "openCv_to_ros");
  ros::NodeHandle n;
  ImageConverter ic(n);
  ic.get_log_files(argv[1],ic.fileNames_,".png",true);

  for(unsigned int i=0;i<ic.fileNames_.size();i++){

	  ROS_INFO ("fileNames %s..", ic.fileNames_[i].c_str());
  }
  ic.rate_ = atof (argv[2]);

  ic.spin();

  return 0;
}
