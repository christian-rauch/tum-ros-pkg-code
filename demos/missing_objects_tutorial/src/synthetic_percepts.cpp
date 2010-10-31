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
//#include "cv_bridge/CvBridge.h"
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <string.h>
#include <vision_msgs/cop_answer.h>

using namespace std;

class SyntheticPercepts {

public:
  double rate_;
  int object_id_;
  std::vector<std::string> objects_;
  std::string missing_object_;
  SyntheticPercepts(ros::NodeHandle &n) :
    n_(n)
  {
    tabletop_pub_ = n_.advertise<vision_msgs::cop_answer>("tabletop_percepts",10);
    n_.param ("object_id", object_id_, 700000);
    n_.param("missing_object", missing_object_, string(""));
    objects_.push_back("BreakfastCereal"), objects_.push_back("CowsMilk-Product"), objects_.push_back("Bowl-Eating");
    for (unsigned int i = 0; i < objects_.size(); i++)
    {
      if (objects_[i] == missing_object_)
      {
        objects_.erase (objects_.begin()+i);
      }
    }
  }
  

  ~SyntheticPercepts()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Spin (!)
  bool spin ()
  {
    ros::Rate loop_rate(1);
    vision_msgs::cop_answer msg;
    vision_msgs::cop_descriptor cop_descriptor;
    vision_msgs::aposteriori_position aposteriori_position;
    vision_msgs::cop_descriptor cop_descriptor2;
    vision_msgs::aposteriori_position aposteriori_position2;
    msg.found_poses.push_back(aposteriori_position);
    msg.found_poses.push_back(aposteriori_position2);

    msg.found_poses[0].models.push_back(cop_descriptor);
    msg.found_poses[1].models.push_back(cop_descriptor2);
    
    msg.found_poses[0].objectId = 0;
    msg.found_poses[1].objectId = 1;

    while (n_.ok ())
    {     
      ROS_INFO ("Publishing data on topic %s.", n_.resolveName ("tabletop_percepts").c_str ());
      msg.found_poses[0].models[0].type = "ODUFinder";
      //      msg.found_poses[0].models[0].sem_class = "CowsMilk-Product";
      msg.found_poses[0].models[0].sem_class = objects_[0];
      msg.found_poses[0].models[0].object_id = ++object_id_;
      msg.found_poses[0].position = 0;
      
      msg.found_poses[1].models[0].type = "ODUFinder";
      //      msg.found_poses[1].models[0].sem_class = "BreakfastCereal";
      msg.found_poses[1].models[0].sem_class = objects_[1];
      msg.found_poses[1].models[0].object_id = ++object_id_;
      msg.found_poses[1].position = 0;
      
      tabletop_pub_.publish(msg);
      loop_rate.sleep();
      ros::spinOnce ();
    }
    return (true);
  }
  
protected:

  ros::NodeHandle n_;
  ros::Publisher tabletop_pub_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "synthetic_percepts");
  ros::NodeHandle n("~");
  SyntheticPercepts sp(n);
  sp.spin();
  return 0;
}
