/* 
 * Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>
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

#ifndef __PRESSURE_H__
#define __PRESSURE_H__

// roslaunch arm_ik.launch
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <pr2_msgs/PressureState.h>
#include <boost/thread/mutex.hpp>

class Pressure {
private:
  ros::Subscriber sub_;
  ros::NodeHandle n_;

  long zero_cnt_;

  float r_center[2];
  float l_center[2];
  float r_sum[22];
  float l_sum[22];
  float r_zero[22];
  float l_zero[22];

  float r_curr[22];
  float l_curr[22];

  void calcCenter(float pressure[], double &xcenter, double &ycenter);

  boost::mutex pressure_mutex;

  void pressureCallback(const pr2_msgs::PressureState::ConstPtr& msg);

  Pressure(int side);

  ~Pressure();

  static Pressure *instance_[];

  bool initialized;

public:

 static Pressure *getInstance(int side=0);

 void reset();

 void getCenter(float *r, float *l);

 void getCurrent(float r[], float l[], bool zero = true);

 //get sum of pressure on the inside pads
 void getInside(float &r, float &l, bool zero = true);
 //get sum of pressure on the front pads
 void getFront(float &r, float &l, bool zero = true);

};



#endif
