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


#ifndef __ROBOT_DRIVER_H__
#define __ROBOT_DRIVER_H__

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_common_action_msgs/ArmMoveIKAction.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <boost/thread/mutex.hpp>


class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;
  //! We need laser scans avoiding obstacles
  ros::Subscriber subScan_;
  laser_geometry::LaserProjection *projector_;
  boost::mutex scan_mutex;
  bool weHaveScan;
  float scanPoints[5000][2];
  size_t numScanPoints;

  //! ROS node initialization
  RobotDriver();

  static RobotDriver *instance;

  void QuaternionToEuler(const btQuaternion &TQuat, btVector3 &TEuler);

public:

  static RobotDriver *getInstance();

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);

  //!checks wheter an envisioned movement of the base would collide the robot considering base_scan, currently supports only movements in x and y without rotation
  bool checkCollision(float relativePose[]);

  void getRobotPose(tf::Stamped<tf::Pose> &marker);


  bool driveInMap(tf::Stamped<tf::Pose> targetPose,bool exitWhenStuck = false);

  //! Drive forward a specified distance based on odometry information
  bool driveInMap(const float targetPose[],bool exitWhenStuck = false);

  bool driveInOdom(const float targetPose[], bool exitWhenStuck = false);

  void moveBase(const float pose[], bool useNavigation = false);
  void moveBaseP(float x, float y, float oz, float ow, bool useNavigation = false);

  void driveToMatch(std::vector<tf::Stamped<tf::Pose> > targetPose, std::vector<std::string> frame_ids);

};

#endif
