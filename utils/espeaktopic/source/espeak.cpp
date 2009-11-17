/* 
 * Copyright (c) 2009, U. Klank   klank@in.tum.de
                       Piotr Esden-Tempski
               
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
#include <std_msgs/String.h>
#include <vision_msgs/cop_answer.h>
#include <nav_pcontroller/nav_actionActionGoal.h>
#include <boost/thread.hpp>
//#include <cogman_msgs/ArmHandActionGoal.h>
boost::mutex mutex;

// Main procedure 
void speakCallback(const std_msgs::String::ConstPtr &st)
{
  
  char test[4096];
  if((*st).data.length() > 4096)
     return;
  printf("Saying: %s\n", (*st).data.c_str());
  sprintf(test, "say \"%s\"", (*st).data.c_str());
  mutex.lock();
  system(test);
  mutex.unlock();
}
void speakCopCallback(const vision_msgs::cop_answer::ConstPtr &answer)
{
  std::string st;
  if((*answer).error.length() > 0)
  {
    return;
     st = (*answer).error;
  }
  else
  {
      st = "I Found an object, it is a ";
      printf("Looking for C: %i\n", (int)(*answer).found_poses[0].classes[0].compare("Cluster"));
      if((*answer).found_poses[0].classes[0] == "Cluster"){
	  printf("[Ignoring Cluster]\n");
	  return;
      }
      st = st.append((*answer).found_poses[0].classes[0]).append(" ");
      if((*answer).found_poses[0].classes.size() > 1)
      {
	if((*answer).found_poses[0].classes[1] == "Cluster "){
	  printf("[Ignoring Cluster]\n");
	  return;
	}
        st = st.append((*answer).found_poses[0].classes[1]);
      }
  } 
  
   char test[4096];
  if(st.length() > 4096)
     return;
  printf("Saying: '%s'\n", st.c_str());
  sprintf(test, "say \"%s\"", st.c_str());
  mutex.lock();
  system(test);
  mutex.unlock();
}

void speakNavCallback(const nav_pcontroller::nav_actionActionGoal::ConstPtr &answer)
{
  std::string st;
  if((*answer).goal.target_lo.data==101)
     st = "  I am Going to the Table.      ";
  else if((*answer).goal.target_lo.data==100)
  {
      st = " I am  Going to the second Table.    ";
  
  }
  else
  {
    return;
  }
  
   char test[4096];
  if(st.length() > 4096)
     return;
  printf("Saying: %s\n", st.c_str());
  sprintf(test, "say \"%s\"", st.c_str());
   mutex.lock();
  system(test);
  mutex.unlock();
}
/*
void speakArmCallback(const cogman_msgs::ArmHandActionGoal::ConstPtr &answer)
{
  std::string st;
  if((*answer).goal.target_lo.data==101)
     st = "    Going to Table.";
  else
  {
      st = "    Going to Counter.";
  
  } 
  
   char test[4096];
  if(st.length() > 4096)
     return;
  printf("Saying: %s\n", st.c_str());
  sprintf(test, "say \"%s\"", st.c_str());
   mutex.lock();
  system(test);
  mutex.unlock();
}

*/
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "espeak") ;
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/espeak/wordstosay", 1, &speakCallback);
  ros::Subscriber sub2 = n.subscribe("/kipla/cop_reply", 1, &speakCopCallback);
  ros::Subscriber sub3 = n.subscribe("/nav_pcontroller/nav_action/goal", 1, &speakNavCallback);
  /*ros::Subscriber sub3 = n.subscribe  "/rigth_arm/result", 1, &speakArmCallback);*/
  ros::spin();
  return 0;
}





