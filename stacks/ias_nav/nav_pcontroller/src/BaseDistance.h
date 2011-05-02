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

#ifndef BASE_DISTANCE_H
#define BASE_DISTANCE_H

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <math.h>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

class BaseDistance {
public:
  // helper class
  class Vector2
  {
  public:
    double x, y;
    Vector2() {}
    Vector2(double x_, double y_) : x(x_), y(y_) {}

    double len2() { return x*x + y*y; }
    double len() {return sqrt(len2()); }
  };

  BaseDistance();

  bool compute_pose2d(const char* from, const char* to, const ros::Time time, double *x, double *y, double *th);
  void setFootprint(double front, double rear, double left, double right, double tolerance);

  void setSafetyLimits(double safety_dist, double slowdown_near, double slowdown_far, double rate);

  void compute_distance_keeping(double *vx, double *vy, double *vth);

  double distance(std::vector<Vector2> &points, Vector2 *nearest, double dx=0, double dy=0, double dth=0);
private:

  Vector2 transform(const Vector2 &v, double x, double y, double th);

  double brake(std::vector<Vector2> &points, double *vx, double *vy, double *vth);
  double grad(std::vector<Vector2> &points, double *gx, double *gy, double *gth);
  double project(std::vector<Vector2> &points, double *vx, double *vy, double *vth);

  double marker_size_;
  double early_reject_distance_;
  double front_, rear_, left_, right_;
  double tolerance_;
  double d_, safety_dist_, slowdown_near_, slowdown_far_, repelling_dist_, repelling_gain_, repelling_gain_max_;

  std::string odom_frame_, base_link_frame_;
  int n_lasers_;
  bool complete_blind_spots_;
  double blind_spot_threshold_;

  ros::NodeHandle n_;

  std::vector<Vector2> blind_spots_;
  boost::shared_ptr<std::vector<Vector2> > laser_points_[2];

  //boost::shared_ptr<std::vector<Vector2> > current_points_;  //!< should replace the points argument of distance(), project(), grad() and brake().
  double rob_x_, rob_y_, rob_th_;
  double vx_last_, vy_last_, vth_last_;
  Vector2 nearest_;
  int mode_;

  ros::Subscriber laser_subscriptions_[2];
  
  ros::Publisher marker_pub_;
  ros::Publisher laser_points_pub_;
  ros::Publisher debug_pub_;
  tf::TransformListener tf_;
  
  //double reading_blind_front_, reading_blind_left_, reading_blind_right_, reading_blind_rear_;
  //double laser_x_front_, laser_y_front_, laser_x_rear_, laser_y_rear_;

  boost::mutex lock;

  /**
   * Interpolates n points between pt1 and pt2 and adds them to
   * blind_spots_
   */
  bool interpolateBlindPoints(int n, const Vector2 &pt1, const Vector2 &pt2);
  void laserCallback(int index, const sensor_msgs::LaserScan::ConstPtr& scan);
  void publishLaserMarker(const Vector2 &pt, const std::string &ns, int id=0);
  void publishNearestPoint();
  void publishBaseMarker();
  void publishPoints(const std::vector<Vector2> &points);

  void calculateEarlyRejectDistance();

  //void swap_point_buffers();
};

#endif
