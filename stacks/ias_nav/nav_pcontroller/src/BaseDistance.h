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

class Vector2 {
public:
  Vector2() {}
  Vector2(double x_, double y_)
    : x(x_), y(y_) {}
  double x, y;
};


class BaseDistance {
public:
  BaseDistance();

  bool compute_pose2d(const char* from, const char* to, double *x, double *y, double *th);
  void setFootprint(double top, double left, double right, double bottom, double tolerance);

  void setSafetyLimits(double safety_dist, double slowdown_near, double slowdown_far, double rate);

  double distance(std::vector<Vector2> &points, Vector2 *nearest, double dx=0, double dy=0, double dth=0);
  double brake(std::vector<Vector2> &points, double *vx, double *vy, double *vth);
  double grad(std::vector<Vector2> &points, double *gx, double *gy, double *gth);
  double project(std::vector<Vector2> &points, double *vx, double *vy, double *vth);
  void compute_distance_keeping(double *vx, double *vy, double *vth);
private:
  double top_, left_, right_, bottom_;
  double tolerance_;
  double d_, safety_dist_, slowdown_near_, slowdown_far_, repelling_dist_, repelling_gain_, repelling_gain_max_;

  int n_lasers_;
  bool complete_blind_spots_;
  double blind_spot_threshold_;

  ros::NodeHandle n_;

  std::vector<Vector2> blind_spots_;
  boost::shared_ptr<std::vector<Vector2> > laser_points_[2];

  ros::Subscriber laser_subscriptions_[2];
  
  ros::Publisher marker_pub_;
  ros::Publisher marker_laser_pub_;
  ros::Publisher interpolated_points_pub_;
  tf::TransformListener tf_;
  
  //double reading_blind_front_, reading_blind_left_, reading_blind_right_, reading_blind_rear_;
  //double laser_x_front_, laser_y_front_, laser_x_rear_, laser_y_rear_;

  boost::mutex lock;

  /**
   * Interpolates n points between pt1 and pt2 and adds them to
   * blind_spots_
   */
  void interpolateBlindPoints(int n, const Vector2 &pt1, const Vector2 &pt2);
  void laserCallback(int index, const sensor_msgs::LaserScan::ConstPtr& scan);
  void publishLaserMarker(const Vector2 &pt, const std::string &ns, int id=0);
  void publishPoints(const std::vector<Vector2> &points);

  //void swap_point_buffers();
};

#endif
