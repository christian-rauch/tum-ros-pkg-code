/*
 * Copyright (c) 2009, Ingo Kresse <kresse@in.tum.de>
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


/**

@mainpage

@htmlinclude manifest.html

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "tf_message" tf/tfMessage: robot's pose in the "map" frame
- @b "goal" geometry_msgs/PoseStamped : goal for the robot.

Publishes to (name / type):
- @b "cmd_vel" geometry_msgs/Twist : velocity commands to robot
- @b "state" nav_robot_actions/MoveBaseState : current planner state (e.g., goal reached, no path)

@section parameters ROS parameters
  - @b "xy_tolerance" (double) : Goal distance tolerance (how close the robot must be to the goal before stopping), default: 0.05 m
  - @b "th_tolerance" (double) : Goal rotation tolerance (how close the robot must be to the goal before stopping), default: 0.05 rad
  - @b "vel_lin_max" (double) : maximum linear velocity, default: 0.2 m/s
  - @b "vel_ang_max" (double) : maximum angular velocity, default: 0.2 rad/s
  - @b "acc_lin_max" (double) : maximum linear acceleration, default: 0.1 m/s^2
  - @b "acc_ang_max" (double) : maximum angular acceleration, default: 0.1 rad/s^2
  - @b "loop_rate" (int) : rate at which the control loop runs, default: 30 s^-1
  - @b "p" (double) : P controller value, default: 1.0

*/

#include <unistd.h>
#include <math.h>

#include "ros/ros.h"


#include "tf/transform_listener.h"


// The messages that we'll use
//#include <nav_robot_actions/MoveBaseState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <sensor_msgs/LaserScan.h>

#include "BaseDistance.h"

class SpeedFilter {
private:
  BaseDistance dist_control_;

  ros::NodeHandle n_;
  ros::Publisher  pub_vel_;
  ros::Subscriber sub_vel_;

  void input_vel(const geometry_msgs::Twist::ConstPtr& msg);
  
public:
  SpeedFilter();
};

void SpeedFilter::input_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
  geometry_msgs::Twist t = *msg;

  dist_control_.compute_distance_keeping(&(t.linear.x), &(t.linear.y), &(t.angular.z));
  pub_vel_.publish(t);
}


SpeedFilter::SpeedFilter() : n_("~")
{
  sub_vel_ =  n_.subscribe("/input_vel", 1, &SpeedFilter::input_vel, this);
  pub_vel_ =  n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  //CHANGED
  //  dist_control_.setFootprint(0.42, -0.3075, 0.3075, -0.42, 0.0);
  double front, rear, left, right, tolerance;
  n_.param("footprint/left", left, 0.309);
  n_.param("footprint/right", right, -0.309);
  n_.param("footprint/front", front, 0.43);
  n_.param("footprint/rear", rear, -0.43);
  n_.param("footprint/tolerance", tolerance, 0.0);
  dist_control_.setFootprint(front, rear, left, right, tolerance);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "speed_filter");

  SpeedFilter sf;

  ros::spin();

  return 0;
}
