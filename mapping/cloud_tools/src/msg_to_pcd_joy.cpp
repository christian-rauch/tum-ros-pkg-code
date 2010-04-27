/*
 * Copyright (c) 2010 Nico Blodow <blodow -=- cs.tum.edu>
 *
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
 *
 *
 */

/** 
@file

@brief msg_to_pcd_joy subscribes to both, topic with PointCloud message
and topic with Joy message, and saves PointCloud as pcd file upon pressing 
button B on the Wii joystick device. 


@par Advertises

@par Subscribes
- \b topic with PointCloud message
- \b topic with Joy message
@par Parameters
- \b string input_cloud_topic_
- \b string input_joy_topic_
*/

// #include <unistd.h>

#include <ctime>
#include <joy/Joy.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>
#include <boost/thread/mutex.hpp>

class MsgToPCDJoy
{
  protected:
    ros::NodeHandle nh_;
    std::string input_cloud_topic_;
    std::string input_joy_topic_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber joy_sub_;
    int counter_;
    boost::mutex lock;
    sensor_msgs::PointCloudConstPtr cloud_in_;

  public:
    MsgToPCDJoy () : counter_(0)
    {
      nh_.param ("input_cloud_topic", input_cloud_topic_, std::string("shoulder_cloud"));
      nh_.param ("input_joy_topic", input_joy_topic_, std::string("/wii/wiimote"));
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &MsgToPCDJoy::cloud_cb, this);
      joy_sub_ = nh_.subscribe (input_joy_topic_, 1, &MsgToPCDJoy::joy_cb, this);
      ROS_INFO ("subscribed to both topics.");
    }
  
  /*
   * \brief joy_cb checks if button B is pressed and if we received the PointCloud message
   * and writes it to disk.
   * \param j input Joy message
   */
    void
      joy_cb (const joy::Joy::ConstPtr& j)
    {
      if (j->buttons[3] == 1 && counter_ > 0)
      {
        lock.lock ();
        std::ostringstream filename;
        filename << "cloud_" << time (NULL) << "_" << getpid () << ".pcd";
        ROS_INFO ("Joystick message received on %s. Saving to %s", input_joy_topic_.c_str (), filename.str ().c_str ());
        cloud_io::savePCDFile (filename.str ().c_str (), *cloud_in_, true);
        lock.unlock ();
      }
      else
      {
        for (unsigned int i = 0; i < j->buttons.size(); i++)
          if (j->buttons[i] == 1)
            ROS_INFO ("Joystick button Nr. %i pressed.", i);
      }
    }
  
  /*
   * \brief cloud_cb PointCloud callback
   * \param cloud input PointCloud message
   */
    void
      cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
    {
      ROS_INFO ("PointCloud message received on %s with %d points", input_cloud_topic_.c_str (), (int)cloud->points.size());
      lock.lock ();
      cloud_in_ = cloud;
      counter_ ++;
      lock.unlock ();
    }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "msg_to_pcd");

  MsgToPCDJoy n;
  ros::spin ();

  return (0);
}

