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


#include <vector>
#include <math.h>

#include <stdio.h>

#include <algorithm>
#include <iterator>

#include <boost/bind.hpp>

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>

#include "BaseDistance.h"

/* \todo: avoid unnecessary computation

  write standalone laser_bumper

  integrate
  test
  debug print about activity

  there are a lot of 'new' statements. -> performance problem?
*/


#define MODE_FREE 0
#define MODE_PROJECTING 1
#define MODE_HARD_PROJECTING 2
#define MODE_BRAKING 3
#define MODE_REPELLING 4
#define MODE_PROJECTION_MASK 3


BaseDistance::BaseDistance()
  : n_("~")
{
  d_ = 1 / 10.0;
  rob_x_ = rob_y_ = rob_th_ = 0.0;
  vx_last_ = vy_last_ = vth_last_ = 0.0;

  n_.param("marker_size", marker_size_, 0.1);
  n_.param("safety_dist", safety_dist_, 0.10);
  n_.param("slowdown_far", slowdown_far_, 0.30);
  n_.param("slowdown_near", slowdown_near_, 0.15);
  n_.param("repelling_dist", repelling_dist_, 0.20);
  n_.param("repelling_gain", repelling_gain_, 0.5);
  n_.param("repelling_gain_max", repelling_gain_max_, 0.015);

  n_.param<std::string>("odom_frame", odom_frame_, "/odom");
  n_.param<std::string>("base_link_frame", base_link_frame_, "/base_link");

  ROS_INFO("## safe=%f slow=%f..%f repell=%f repell_gain=%f\n", safety_dist_, slowdown_near_, slowdown_far_, repelling_dist_, repelling_gain_);

  n_.param("n_lasers", n_lasers_, 2);
  if(n_lasers_ < 1 || n_lasers_ > 2)
    ROS_FATAL("Only one or two lasers are supported. %d lasers specified.", n_lasers_);

  n_.param("complete_blind_spots", complete_blind_spots_, true);
  n_.param("blind_spot_threshold", blind_spot_threshold_, 0.85);
  
  marker_pub_ = n_.advertise<visualization_msgs::Marker>("/visualization_marker", 0);
  laser_points_pub_ = n_.advertise<sensor_msgs::PointCloud>("laser_points", 0);
  debug_pub_ = n_.advertise<std_msgs::Float64MultiArray>("debug_channels", 0);

  laser_subscriptions_[0] = n_.subscribe<sensor_msgs::LaserScan>
    ("laser_1", 2, boost::bind(&BaseDistance::laserCallback, this, 0, _1));
  if(n_lasers_ > 1)
    laser_subscriptions_[1] = n_.subscribe<sensor_msgs::LaserScan>
      ("laser_2", 2, boost::bind(&BaseDistance::laserCallback, this, 1, _1));

  calculateEarlyRejectDistance();
}



BaseDistance::Vector2 BaseDistance::transform(const Vector2 &v, double x, double y, double th)
{
  return Vector2(v.x*cos(th) - v.y*sin(th) + x,
                 v.x*sin(th) + v.y*cos(th) + y);
}


void BaseDistance::setFootprint(double front, double rear, double left, double right, double tolerance)
{
  ROS_INFO("setting footprint. x : %f .. %f     y : %f .. %f", rear, front, right, left);

  rear_ = rear;
  front_ = front;
  left_ = left;
  right_ = right;
  tolerance_ = tolerance;

  calculateEarlyRejectDistance();
}

void BaseDistance::calculateEarlyRejectDistance()
{
  double movement_tolerance = 0.2;
  double diameter = sqrt((front_ - rear_)*(front_ - rear_) + (right_ - left_)*(right_ - left_));
  early_reject_distance_ = diameter + slowdown_far_ + tolerance_ + movement_tolerance;
}


bool BaseDistance::compute_pose2d(const char* from, const char* to, const ros::Time time, double *x, double *y, double *th)
{
  tf::StampedTransform pose;
  try
  {
    tf_.lookupTransform(from, to, time, pose);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("no localization information yet (%s -> %s)", from, to);
    return false;
  }

  *x = pose.getOrigin().x();
  *y = pose.getOrigin().y();

  btMatrix3x3 m = pose.getBasis();
  *th = atan2(m[1][0], m[0][0]);

  return true;
}

void BaseDistance::laserCallback(int index, const sensor_msgs::LaserScan::ConstPtr &scan)
{
  boost::shared_ptr<std::vector<Vector2> > points(new std::vector<Vector2>);

  points->reserve(scan->ranges.size());

  double x, y, th;
  compute_pose2d(odom_frame_.c_str(), scan->header.frame_id.c_str(), ros::Time(0), &x, &y, &th);

  double angle;
  int i, first_scan = 0, last_scan = 0;

  // find last valid scan range
  for(int i=scan->ranges.size()-1; i >= 0; i--)
  {
    if(!(scan->ranges[i] <= scan->range_min ||
       scan->ranges[i] >= scan->range_max))
    {
      last_scan = i;
      break;
    }
  }


  for(angle=scan->angle_min, i=0;
      i < (int) scan->ranges.size();
      i++, angle+=scan->angle_increment)
  {
    // reject invalid ranges
    if(scan->ranges[i] <= scan->range_min ||
       scan->ranges[i] >= scan->range_max)
    {
      first_scan++;
      continue;
    }

    // reject readings from far away
    if(scan->ranges[i] >= early_reject_distance_ && i != first_scan && i != last_scan)
      continue;

    first_scan = -1;

    double xl = scan->ranges[i] * cos(angle);
    double yl = scan->ranges[i] * sin(angle);
    points->push_back(transform(Vector2(xl, yl), x, y, th));
  }

  boost::mutex::scoped_lock mutex(lock);
  laser_points_[index] = points;

  if(complete_blind_spots_ && laser_points_[0] && laser_points_[1])
  {
    blind_spots_.clear();

    if(interpolateBlindPoints(10, laser_points_[0]->front(), laser_points_[1]->back()))
    {
      publishLaserMarker(laser_points_[0]->front(), "blind_spots", 0);
      publishLaserMarker(laser_points_[1]->back(),  "blind_spots", 1);
    }

    if(interpolateBlindPoints(10, laser_points_[0]->back(), laser_points_[1]->front()))
    {
      publishLaserMarker(laser_points_[0]->back(),  "blind_spots", 2);
      publishLaserMarker(laser_points_[1]->front(), "blind_spots", 3);
    }
    //// publishPoints(blind_spots_);
  }
}

bool BaseDistance::interpolateBlindPoints(int n, const Vector2 &pt1, const Vector2 &pt2)
{
  Vector2 p1_rob = transform(pt1, rob_x_, rob_y_, rob_th_);
  Vector2 p2_rob = transform(pt2, rob_x_, rob_y_, rob_th_);

  if( p1_rob.len2() < blind_spot_threshold_ * blind_spot_threshold_ ||
      p2_rob.len2() < blind_spot_threshold_ * blind_spot_threshold_)
  {
    for(int i=0; i < n; i++)
    {
      double t = (i/(n-1.0));
      blind_spots_.push_back(Vector2(t*pt1.x + (1.0-t)*pt2.x,
                                     t*pt1.y + (1.0-t)*pt2.y));
    }
    return true;
  }
  else
  {
    return false;
  }
}

void BaseDistance::publishLaserMarker(const Vector2 &pt, const std::string &ns, int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = odom_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.z = 0.55;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker_size_;
  marker.scale.y = marker_size_;
  marker.scale.z = marker_size_;
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.2);
  marker.id = id;
  marker.pose.position.x = pt.x;
  marker.pose.position.y = pt.y;
  marker.pose.orientation.w = 1.0;
  marker_pub_.publish(marker);
}

void BaseDistance::publishNearestPoint()
{
  // visualize closest point
  visualization_msgs::Marker marker;
  marker.header.frame_id = base_link_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "nearest_point";
  marker.id = 0;
   marker.type = visualization_msgs::Marker::CUBE;

  if(mode_ & MODE_REPELLING)
    marker.type = visualization_msgs::Marker::SPHERE;
  else
    marker.type = visualization_msgs::Marker::CUBE;

  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = nearest_.x;
  marker.pose.position.y = nearest_.y;
  marker.pose.position.z = 0.55;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker_size_*2;
  marker.scale.y = marker_size_*2;
  marker.scale.z = marker_size_*2;

  switch(mode_ & MODE_PROJECTION_MASK)
  {
    case(MODE_FREE):
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      break;
    case(MODE_PROJECTING):
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      break;
    case(MODE_HARD_PROJECTING):
      marker.color.r = 1.0f;
      marker.color.g = 0.5f;
      break;
    case(MODE_BRAKING):
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      break;
  }
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(0.1);
  marker_pub_.publish(marker);
}

void BaseDistance::publishBaseMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = base_link_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "base_footprint";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = (front_ + rear_) * 0.5;
  marker.pose.position.y = (right_ + left_) * 0.5;
  marker.pose.position.z = 0.3;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = front_ - rear_;
  marker.scale.y = right_ - left_;
  marker.scale.z = 0.6;
  marker.color.r = 0.5f;
  marker.color.g = 0.5f;
  marker.color.b = 1.0f;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration();
  marker_pub_.publish(marker);
}

void BaseDistance::publishPoints(const std::vector<Vector2> &points)
{
  sensor_msgs::PointCloud cloud;

  cloud.points.resize(points.size());

  std::vector<Vector2>::const_iterator src_it;
  sensor_msgs::PointCloud::_points_type::iterator dest_it;

  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = odom_frame_;
  for(src_it=points.begin(), dest_it=cloud.points.begin();
      src_it != points.end();
      src_it++, dest_it++)
  {
    dest_it->x = src_it->x;
    dest_it->y = src_it->y;
    dest_it->z = 0.3;
  }

  // add one dummy point to clear the display
  if(points.size() == 0)
  {
    cloud.points.resize(1);
    cloud.points[0].x = 0.0;
    cloud.points[0].y = 0.0;
    cloud.points[0].z = 0.0;
  }

  laser_points_pub_.publish(cloud);
}

#define CLAMP(x, min, max) (x < min) ? min : ((x > max) ? max : x)

double BaseDistance::grad(std::vector<Vector2> &points, double *gx, double *gy, double *gth)
{
  if(!gx || !gy || !gth)
    return 0;

  double d0, dx_p, dy_p, dth_p, dx_n, dy_n, dth_n;

  double v_len = sqrt(vx_last_*vx_last_ + vy_last_*vy_last_ + vth_last_*vth_last_);
  double dd = CLAMP(v_len*d_*2, 0.005, 0.15);

  d0   = distance(points, &nearest_, 0, 0, 0);

  dx_p  = distance(points, 0,  dd,  0,  0);
  dx_n  = distance(points, 0, -dd,  0,  0);

  dy_p  = distance(points, 0,  0,  dd,  0);
  dy_n  = distance(points, 0,  0, -dd,  0);

  dth_p = distance(points, 0,  0,  0,  dd);
  dth_n = distance(points, 0,  0,  0, -dd);

  *gx  = (dx_p  - dx_n)  / (2.0 * dd);
  *gy  = (dy_p  - dy_n)  / (2.0 * dd);
  *gth = (dth_p - dth_n) / (2.0 * dd);

  // compute second derivatives for finding saddle points
  double g2x = (dx_p  + dx_n - 2*d0) / (dd*dd);
  double g2y = (dy_p  + dy_n - 2*d0) / (dd*dd);
  double g2th = (dth_p  + dth_n - 2*d0) / (dd*dd);

  //ROS_DEBUG("ddx=%f, ddy=%f, ddth=%f\n", ddx, ddy, ddth);
  //ROS_DEBUG("gx: %f (+- %f)  gy: %f (+-%f)  gth: %f (+- %f)\n", *gx, g2x, *gy, g2y, *gth, g2th);

  // "dampen" if second derivative is big
  double g2_scaledown = 2.0;
  double gx_damp = (fabs(g2x) < g2_scaledown) ? 1.0 : g2_scaledown / fabs(g2x);
  double gy_damp = (fabs(g2y) < g2_scaledown) ? 1.0 : g2_scaledown / fabs(g2y);
  double gth_damp = (fabs(g2th) < g2_scaledown) ? 1.0 : g2_scaledown / fabs(g2th);

  *gx = *gx * gx_damp;
  *gy = *gy * gy_damp;
  *gth = *gth * gth_damp;

  std_msgs::Float64MultiArray msg;
  msg.data.resize(7);
  msg.data[0] = dd;

  msg.data[1] = *gx;
  msg.data[2] = *gy;
  msg.data[3] = *gth;

  msg.data[4] = g2x;
  msg.data[5] = g2y;
  msg.data[6] = g2th;

  debug_pub_.publish(msg);  

  return d0;
}


void BaseDistance::setSafetyLimits(double safety_dist, double slowdown_near, double slowdown_far, double rate)
{
  d_ = 1.0 / rate;
  safety_dist_ = safety_dist;
  slowdown_near_ = slowdown_near;
  slowdown_far_ = slowdown_far;
}


double BaseDistance::brake(std::vector<Vector2> &points, double *vx, double *vy, double *vth)
{
  double factor = 1.5;
  double d = distance(points, 0, -*vx*d_*factor, -*vy*d_*factor, -*vth*d_*factor);
  if(d < safety_dist_) { // if we are stuck in the NEXT timestep...
    mode_ |= MODE_BRAKING;
    *vx = 0.0;
    *vy = 0.0;
    *vth = 0.0;
  }
  return d;
}

double BaseDistance::project(std::vector<Vector2> &points, double *vx, double *vy, double *vth)
{
  double d, gx, gy, gth, factor=0;
  d = grad(points, &gx, &gy, &gth);

  // normalize gradient
  double l_grad = 1/sqrt(gx*gx + gy*gy + gth*gth);
  gx*=l_grad;
  gy*=l_grad;
  gth*=l_grad;

  if(d < slowdown_far_) {
    // project (vx,vy,vth) onto (gx,gy,gth)
    double dp = *vx*gx + *vy*gy + *vth*gth;
    if(dp > 0) { // we are moving towards the obstacle
      if(d < slowdown_near_)
      {
        mode_ |= MODE_HARD_PROJECTING;
        factor = 1;
      }
      else
      {
        mode_ |= MODE_PROJECTING;
        factor = (d - slowdown_far_)/(slowdown_near_ - slowdown_far_);
      }

      *vx  -= gx *dp*factor;
      *vy  -= gy *dp*factor;
      *vth -= gth*dp*factor;
    }
  }

  if(d < repelling_dist_) {
    mode_ |= MODE_REPELLING;
    double l_grad_2d = 1/sqrt(gx*gx + gy*gy);
    double a = (1.0/d - 1.0/repelling_dist_)
             * (1.0/d - 1.0/repelling_dist_)
             * 0.5 * repelling_gain_;

    if(a > repelling_gain_max_)
      a = repelling_gain_max_;
    *vx -= gx * l_grad_2d * a;
    *vy -= gy * l_grad_2d * a;
  }

  return d;
}

void BaseDistance::compute_distance_keeping(double *vx, double *vy, double *vth)
{

  // compute last known robot position in odom frame
  compute_pose2d(base_link_frame_.c_str(), odom_frame_.c_str(), ros::Time(0), &rob_x_, &rob_y_, &rob_th_);

  // collect all relevant obstacle points
  std::vector<Vector2> current_points;

  {
    boost::mutex::scoped_lock current_lock(lock);

    current_points.reserve((laser_points_[0] ? laser_points_[0]->size() : 0) +
                           (laser_points_[1] ? laser_points_[1]->size() : 0) +
                           blind_spots_.size());

    if(laser_points_[0])
    {
      std::copy(laser_points_[0]->begin(), laser_points_[0]->end(),
                std::back_insert_iterator<std::vector<Vector2> >(current_points));
    }
    if(laser_points_[1])
    {
      std::copy(laser_points_[1]->begin(), laser_points_[1]->end(),
                std::back_insert_iterator<std::vector<Vector2> >(current_points));
    }
    std::copy(blind_spots_.begin(), blind_spots_.end(),
              std::back_insert_iterator<std::vector<Vector2> >(current_points));
  }

  mode_ = MODE_FREE;

  // modify velocity vector vector
  project(current_points, vx, vy, vth);

  // if necessary, do braking
  brake(current_points, vx, vy, vth);

  vx_last_ = *vx;
  vy_last_ = *vy;
  vth_last_ = *vth;

  publishNearestPoint();
  publishBaseMarker();
  publishPoints(current_points);
}

#define LEFT 1
#define RIGHT 2
#define REAR 4
#define FRONT 8

double BaseDistance::distance(std::vector<Vector2> &points, Vector2 *nearest, double dx, double dy, double dth)
{
  Vector2 rnearest(0, 0), lnearest(0, 0);
  double rqdistance=INFINITY, ldistance=INFINITY;

  int rarea, larea;

  if(points.size() == 0)
    ROS_WARN("No points received. Maybe the laser topic is wrong?");

  // transform from odom to base_link
  btMatrix3x3 rob(cos(rob_th_), -sin(rob_th_), rob_x_,
                  sin(rob_th_),  cos(rob_th_), rob_y_,
                  0,             0,            1);

  // this transform adds some adjustment
  btMatrix3x3 adj(cos(dth), -sin(dth), dx,
                  sin(dth),  cos(dth), dy,
                  0,         0,        1);

  // create transformation matrix
  btMatrix3x3 m = adj*rob;

  for(unsigned int i=0; i < points.size(); i++) {
    double px = points[i].x*m[0][0] + points[i].y*m[0][1] + m[0][2];
    double py = points[i].x*m[1][0] + points[i].y*m[1][1] + m[1][2];

    double dright = -py + right_;
    double dleft  =  py - left_;
    double drear  = -px + rear_;
    double dfront =  px - front_;

    int area=0;
    area |= RIGHT * (dright > -tolerance_);
    area |= LEFT  * (dleft  > -tolerance_);
    area |= REAR  * (drear  > -tolerance_);
    area |= FRONT * (dfront > -tolerance_);

    double rqdist=INFINITY, ldist=INFINITY;
    switch(area) {
    case(RIGHT): ldist = dright; break;
    case(LEFT):  ldist = dleft; break;
    case(REAR):  ldist = drear; break;
    case(FRONT): ldist = dfront; break;

    case(LEFT  | REAR):  rqdist = dleft*dleft + drear*drear; break;
    case(LEFT  | FRONT): rqdist = dleft*dleft + dfront*dfront; break;
    case(RIGHT | REAR):  rqdist = dright*dright + drear*drear; break;
    case(RIGHT | FRONT): rqdist = dright*dright + dfront*dfront; break;
    }

    if(ldist < ldistance) {
      ldistance = ldist;
      lnearest = Vector2(px, py);
      larea = area;
    }

    if(rqdist < rqdistance) {
      rqdistance = rqdist;
      rnearest = Vector2(px, py);
      rarea = area;
    }
  }


  double rdistance = sqrt(rqdistance);
  if(rdistance < ldistance) {

    //ROS_DEBUG("area=%d\n", rarea);

    if(nearest)
      *nearest = rnearest;
    return rdistance;
  } else {

    //ROS_DEBUG("area=%d\n", larea);

    if(nearest)
      *nearest = lnearest;
    return ldistance;
  }
}
