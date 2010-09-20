/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <kdl/tree.hpp>
#include <ros/ros.h>
#include "robot_state_chain_publisher/robot_state_publisher.h"
#include "robot_state_chain_publisher/joint_state_listener.h"

using namespace std;
using namespace ros;
using namespace KDL;
using namespace robot_state_chain_publisher;


JointStateListener::JointStateListener(const KDL::Tree& tree)
  : n_tilde_("~"), state_publisher_(tree)
{
  // set publish frequency
  double publish_freq, min_publish_freq;
  n_tilde_.param("publish_frequency", publish_freq, 50.0);
  n_tilde_.param("min_publish_frequency", min_publish_freq, 1.0);
  publish_delay_ = Duration(1/publish_freq);
  max_publish_delay_ = Duration(1/min_publish_freq);
  //Time::waitForValid(); // new method which was introduced later?
  last_publish_time_ = Time::now();
  // subscribe to mechanism state
  joint_state_sub_ = n_.subscribe("joint_states", 10, &JointStateListener::callbackJointState, this);

  timer_ = n_tilde_.createTimer(max_publish_delay_, &JointStateListener::timerCallback, this);
};


JointStateListener::~JointStateListener()
{};


void JointStateListener::timerCallback(const ros::TimerEvent& e)
{
  Time now = Time::now();
  if( now >= last_publish_time_ + max_publish_delay_*0.9 )
  {
    // time to publish something, with or without joint states
    state_publisher_.publishTransforms(joint_positions, now, (now - max_publish_delay_));

    if( last_publish_time_ - now > max_publish_delay_*2 || last_publish_time_ - now < max_publish_delay_*2)
      last_publish_time_ = now;
    else
      last_publish_time_ += max_publish_delay_;
  }
}

void JointStateListener::callbackJointState(const JointStateConstPtr& state)
{
  if (state->name.size() == 0){
    ROS_ERROR("Robot state publisher received an empty joint state vector");
    return;
  }

  if (state->name.size() != state->position.size()){
    ROS_ERROR("Robot state publisher received an invalid joint state vector");
    return;
  }

  // early-simtime bug - if we compute a negative time, ros throws an exception
  if(last_publish_time_.toSec() <= max_publish_delay_.toSec()) {
    last_publish_time_ = Time::now();
    return;
  }

  // get joint positions from state message
  for (unsigned int i=0; i<state->name.size(); i++) {
    RobotStatePublisher::JointState s;
    s.pos = state->position[i];
    s.time = state->header.stamp;
    s.published = false;
    joint_positions[state->name[i]] = s;

  }

  Time now = Time::now();
  if( now >= last_publish_time_ + publish_delay_ )
  {
    state_publisher_.publishTransforms(joint_positions, now, (now - max_publish_delay_));

    if( last_publish_time_ - now > publish_delay_*2 || last_publish_time_ - now < publish_delay_*2)
      last_publish_time_ = now;
    else
      last_publish_time_ += publish_delay_;
  }
}
