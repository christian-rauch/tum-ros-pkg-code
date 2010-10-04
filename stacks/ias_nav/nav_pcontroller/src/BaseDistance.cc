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

#include "BaseDistance.h"

#define MARKER_SCALE 0.13

/* \todo: avoid unnecessary computation

  write standalone laser_bumper

  integrate
  test
  debug print about activity

*/

#define TOP 1
#define BOTTOM 2
#define LEFT 4
#define RIGHT 8


//!! evil hack
BaseDistance::BaseDistance()
  : n_("~")
{
  d_ = 1 / 10.0;

  n_.param("safety_dist", safety_dist_, 0.10);
  n_.param("slowdown_far", slowdown_far_, 0.30);
  n_.param("slowdown_near", slowdown_near_, 0.15);
  n_.param("repelling_dist", repelling_dist_, 0.20);
  n_.param("repelling_gain", repelling_gain_, 0.5);
  n_.param("repelling_gain_max", repelling_gain_max_, 0.015);

  ROS_INFO("## safe=%f slow=%f..%f repell=%f repell_gain=%f\n", safety_dist_, slowdown_near_, slowdown_far_, repelling_dist_, repelling_gain_);

  n_.param("n_lasers", n_lasers_, 2);
  if(n_lasers_ < 1 || n_lasers_ > 2)
    ROS_FATAL("Only one or two lasers are supported. %d lasers specified.", n_lasers_);

  n_.param("complete_blind_spots", complete_blind_spots_, true);
  n_.param("blind_spot_threshold", blind_spot_threshold_, 0.85);
  
  // Use only one marker topic plus namespaces
  marker_pub_ = n_.advertise<visualization_msgs::Marker>("closest_point", 0);
  marker_laser_pub_ = n_.advertise<visualization_msgs::Marker>("base_footprint", 0);

  interpolated_points_pub_ = n_.advertise<sensor_msgs::PointCloud>("interpolated_points", 0);

  laser_subscriptions_[0] = n_.subscribe<sensor_msgs::LaserScan>
    ("laser_1", 2, boost::bind(&BaseDistance::laserCallback, this, 0, _1));
  if(n_lasers_ > 1)
    laser_subscriptions_[1] = n_.subscribe<sensor_msgs::LaserScan>
      ("laser_2", 2, boost::bind(&BaseDistance::laserCallback, this, 1, _1)); 
}

void BaseDistance::setFootprint(double top, double left, double right, double bottom, double tolerance)
{
  top_ = top;
  left_ = left;
  right_ = right;
  bottom_ = bottom;
  tolerance_ = tolerance;
}


bool BaseDistance::compute_pose2d(const char* from, const char* to, double *x, double *y, double *th)
{
  tf::Stamped<tf::Pose> global_pose;
  tf::Stamped<tf::Pose> robotPose;
  robotPose.setIdentity();
  robotPose.frame_id_ = to;
  robotPose.stamp_ = ros::Time(0);
  try
  {
    tf_.transformPose(from, robotPose, global_pose);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("no localization information yet (%s -> %s)", from, to);
    return false;
  }

  // find out where we are now
  *x = global_pose.getOrigin().x();
  *y = global_pose.getOrigin().y();
  
  double unused_pitch, unused_roll;
  btMatrix3x3 mat =  global_pose.getBasis();
  mat.getEulerYPR(*th, unused_pitch, unused_roll);

  return true;
}

void BaseDistance::laserCallback(int index, const sensor_msgs::LaserScan::ConstPtr &scan)
{
  boost::shared_ptr<std::vector<Vector2> > points(new std::vector<Vector2>);

  points->reserve(scan->ranges.size());

  double x, y, th;
  compute_pose2d("/base_link", scan->header.frame_id.c_str(), &x, &y, &th);

  double angle, i;
  
  for(angle=scan->angle_min, i=0;
      i<scan->ranges.size();
      i++, angle+=scan->angle_increment)
  {
    if(scan->ranges[i] <= scan->range_min ||
       scan->ranges[i] >= scan->range_max)
      continue;

    double xl = scan->ranges[i] * cos(angle);
    double yl = scan->ranges[i] * sin(angle);
    points->push_back(Vector2(xl*cos(th) - yl*sin(th) + x,
                              xl*sin(th) + yl*cos(th) + y));
  }

  boost::mutex::scoped_lock mutex(lock);
  laser_points_[index] = points;

  if(complete_blind_spots_ && laser_points_[0] && laser_points_[1])
  {
    blind_spots_.clear();
    interpolateBlindPoints(10, laser_points_[0]->front(), laser_points_[1]->back());
    interpolateBlindPoints(10, laser_points_[0]->back(), laser_points_[1]->front());
    publishLaserMarker(laser_points_[0]->front(), "laser_1", 0);
    publishLaserMarker(laser_points_[0]->back(), "laser_1", 1);
    publishLaserMarker(laser_points_[1]->front(), "laser_2", 0);
    publishLaserMarker(laser_points_[1]->back(), "laser_2", 1);

    publishPoints(blind_spots_);
  }
}

void BaseDistance::interpolateBlindPoints(int n, const Vector2 &pt1, const Vector2 &pt2)
{
  if( pt1.x * pt1.x + pt1.y * pt1.y < blind_spot_threshold_ * blind_spot_threshold_ ||
      pt2.x * pt2.x + pt2.y * pt2.y < blind_spot_threshold_ * blind_spot_threshold_)
  {
    for(int i=0; i < n; i++)
    {
      double t = (i/(n-1.0));
      blind_spots_.push_back(Vector2(t*pt1.x + (1.0-t)*pt2.x,
                                     t*pt1.y + (1.0-t)*pt2.y));
    }
  }
}

void BaseDistance::publishLaserMarker(const Vector2 &pt, const std::string &ns, int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.z = 0.55;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = MARKER_SCALE;
  marker.scale.y = MARKER_SCALE;
  marker.scale.z = MARKER_SCALE;
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

void BaseDistance::publishPoints(const std::vector<Vector2> &points)
{
  sensor_msgs::PointCloud cloud;

  cloud.points.resize(points.size());

  std::vector<Vector2>::const_iterator src_it;
  sensor_msgs::PointCloud::_points_type::iterator dest_it;

  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = "/base_link";
  for(src_it=points.begin(), dest_it=cloud.points.begin();
      src_it != points.end();
      src_it++, dest_it++)
  {
    dest_it->x = src_it->x;
    dest_it->y = src_it->y;
    dest_it->z = 0.3;
  }

  interpolated_points_pub_.publish(cloud);
}

// void BaseDistance::laser_points(const sensor_msgs::LaserScan &scan, std::vector<Vector2> &points)
// {
//   int nScans = scan.ranges.size();
//   int firstPoint = points.size();
//   double inc = (scan.angle_max - scan.angle_min) / (nScans-1.0);
//   points.resize(points.size() + nScans);

//   double x, y, th;
//   compute_pose2d("/base_link", scan.header.frame_id.c_str(), &x, &y, &th);
//   //printf("frame %s is (%f %f %f)\n", scan.header.frame_id.c_str(), x, y, th);

//   if(x < 0)
//     scans_received_ |= 1;
//   if(x > 0)
//     scans_received_ |= 2;

//   int i0=0;

//   // TODO: clean up this method
//   static Vector2 vleft_;
//   // static Vector2 vright_;
//   static Vector2 vfront_;
//   // static Vector2 vrear_;


//   for(int i=0, j=firstPoint; i < nScans; i++, j++) {
//     double a,r, xl,yl, xr,yr;
//     a = scan.angle_min + i*inc;
//     r = scan.ranges[i];

//     if(r <= scan.range_min || r >= scan.range_max)
//       continue;

//     // convert polar -> cartesian
//     xl = r*cos(a);
//     yl = r*sin(a);
//     // convert from laser frame to robot frame
//     xr = xl*cos(th) - yl*sin(th) + x;
//     yr = xl*sin(th) + yl*cos(th) + y;
//     // store point
//     points[j] = Vector2(xr, yr);


//     // if(x > 0)
//     {
//       vleft_ = Vector2(xr, yr);
//       if(i0 == 0)
//         vfront_ = Vector2(xr, yr);
//     }

//     // if(x < 0) {
//     //   vright_ = Vector2(xr, yr);
//     //   if(i0 == 0)
//     //     vrear_ = Vector2(xr, yr);
//     // }

//     if(i0==0) i0 = i; 

//   }

  
//   // deal with "blind spots"

//   double threshold_short_ = 0.6;
//   double threshold_long_ = 0.7;

//   // store readings
//   if(x > 0) {  // front laser
// //    reading_blind_front_ = scan.ranges[i0];
// //    reading_blind_left_ = scan.ranges[i1];

// //    laser_x_front_ = x;
// //    laser_y_front_ = y;
//   } else {  // rear laser
// //    reading_blind_right_ = scan.ranges[i0];
// //    reading_blind_rear_ = scan.ranges[i1];

// //    laser_x_rear_ = x;
// //    laser_y_rear_ = y;
//   }


//   // create intermediate points

//   // create front points if necessary
//   if(x > 0 &&
//        (vfront_.y > -threshold_short_ ||
//         vright_.x <  threshold_long_)) {

// /*
//     double x_front = laser_x_front_ + reading_blind_front_ * sin(15.0*M_PI/180.0);
//     double y_front = laser_y_front_ - reading_blind_front_ * cos(15.0*M_PI/180.0);

//     double x_right = laser_x_rear_ + reading_blind_right_ * sin(75.0*M_PI/180.0);
//     double y_right = laser_y_rear_ - reading_blind_right_ * cos(75.0*M_PI/180.0);

//     Vector2 pp = points[points.size()-nScans];
//     printf("points.size()=%d\n", points.size());
//     printf("laser: (%f %f), additional (%f %f)\n", pp.x, pp.y, x_front, y_front);
//     pp = points[points.size()-1];
//     printf("laser: (%f %f), additional (%f %f) d=%f\n", pp.x, pp.y, x_right, y_right, reading_blind_right_);
// */



//     // create 10 intermediate points
//     int n=10;
//     for(int i=0; i < n; i++) {
//       double t = (i/(n-1.0));
//       double px = t*vfront_.x + (1.0-t)*vright_.x;
//       double py = t*vfront_.y + (1.0-t)*vright_.y;
//       points.push_back(Vector2(px, py));
//     }
//   }

//   // create rear points if necessary
//   if(x < 0 &&
//        (vrear_.y <  threshold_short_ ||
//         vleft_.x > -threshold_long_)) {

// /*    double x_rear = laser_x_rear_ - reading_blind_rear_ * sin(15*M_PI/180);
//     double y_rear = laser_y_rear_ + reading_blind_rear_ * cos(15*M_PI/180);

//     double x_left = laser_x_front_ - reading_blind_left_ * cos(15*M_PI/180);
//     double y_left = laser_y_front_ + reading_blind_left_ * sin(15*M_PI/180);
// */
//     //    Vector2 pp = points[points.size()-nScans];
// /*    Vector2 pp = points[points.size()-nScans];
//     printf("laser: (%f %f), additional (%f %f) d=%f\n", pp.x, pp.y, x_rear, y_rear, reading_blind_rear_);
//     pp = points[points.size()-1];
//     printf("laser: (%f %f), additional (%f %f) d=%f\n", pp.x, pp.y, x_left, y_left, reading_blind_left_);
// */

//     // create 10 intermediate points
//     int n=10;
//     for(int i=0; i < n; i++) {
//       double t = (i/(n-1.0));
//       double px = t*vrear_.x + (1.0-t)*vleft_.x;
//       double py = t*vrear_.y + (1.0-t)*vleft_.y;
//       points.push_back(Vector2(px, py));
//     }
//   }
  


//   visualization_msgs::Marker marker;
//   marker.header.frame_id = "/base_link";
//   marker.header.stamp = ros::Time::now();
//   marker.ns = "front";
//   marker.type = visualization_msgs::Marker::CUBE;
//   marker.action = visualization_msgs::Marker::ADD;
//   marker.pose.position.z = 0.55;
//   marker.pose.orientation.x = 0.0;
//   marker.pose.orientation.y = 0.0;
//   marker.pose.orientation.z = 0.0;
//   marker.pose.orientation.w = 1.0;
//   marker.scale.x = MARKER_SCALE;
//   marker.scale.y = MARKER_SCALE;
//   marker.scale.z = MARKER_SCALE;
//   marker.color.r = 1;
//   marker.color.g = 0;
//   marker.color.b = 0;
//   marker.color.a = 1.0;
//   marker.lifetime = ros::Duration(0.2);

//   marker.id = 0;
//   marker.pose.position.x = vfront_.x;
//   marker.pose.position.y = vfront_.y;
//   // if(vfront_.y > -threshold_short_)
//     marker_pub_.publish(marker);

//   // marker.id = 1;
//   // marker.pose.position.x = vright_.x;
//   // marker.pose.position.y = vright_.y;
//   // if(vright_.x <  threshold_long_)
//   //   marker_pub_.publish(marker);
 
//   // marker.id = 2;
//   // marker.pose.position.x = vrear_.x;
//   // marker.pose.position.y = vrear_.y;
//   // if(vrear_.y < threshold_short_)
//   //   marker_pub_.publish(marker);

//   marker.id = 3;
//   marker.pose.position.x = vleft_.x;
//   marker.pose.position.y = vleft_.y;
//   // if(vleft_.x > -threshold_long_)
//     marker_pub_.publish(marker);
// }

// void BaseDistance::swap_point_buffers()
// {
//   //printf("swapping\n");
//   boost::mutex::scoped_lock curr_lock(lock);
  
//   current_points = receiving_points;
//   receiving_points = (&points1 == current_points) ? &points2 : &points1;
//   receiving_points->clear();
// }


// void BaseDistance::laser_collect(const sensor_msgs::LaserScan::ConstPtr& scan)
// {

//   //printf("laser data\n");

//   laser_points(*scan, *receiving_points);

//   // if(scans_received_ == 3)
//   {
//     swap_point_buffers();
//     scans_received_ = 0;
//   }
// }

double BaseDistance::grad(std::vector<Vector2> &points, double *gx, double *gy, double *gth)
{
  if(!gx || !gy || !gth)
    return 0;

  double d=0.005;
  double d0, dx, dy, dth;

  Vector2 nearest;

  d0  = distance(points, &nearest, 0, 0, 0);
  dx  = distance(points, 0, d, 0, 0);
  dy  = distance(points, 0, 0, d, 0);
  dth = distance(points, 0, 0, 0, d);

  *gx  = dx  - d0;
  *gy  = dy  - d0;
  *gth = dth - d0;

/*
  visualization_msgs::MarkerArray laserMarkers;
  for(int i=0; i < points.size(); i++) {
//  static int i=0;
//  i = (i + 1) % points.size();
//  {  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "laser_points";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = points[i].x;
  marker.pose.position.y = points[i].y;
  marker.pose.position.z = 0.5;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.r = (i%2 == 0)? 1 : 0;
  marker.color.g = (i%2 == 1)? 1 : 0;
  marker.color.b = (i%3 == 2)? 1 : 0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.1);
//  marker_pub_.publish(marker);
  laserMarkers.markers.push_back(marker);
  }
*/
//  marker_laser_pub_.publish(laserMarkers);

  // visualize closest point
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "nearest_point";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = nearest.x;
  marker.pose.position.y = nearest.y;
  marker.pose.position.z = 0.55;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = MARKER_SCALE*2;
  marker.scale.y = MARKER_SCALE*2;
  marker.scale.z = MARKER_SCALE*2;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.1);
  marker_pub_.publish(marker);



  //visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "base_footprint";
  marker.id = 10;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0.3;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 2*left_;
  marker.scale.y = 2*top_;
  marker.scale.z = 0.6;
  marker.color.r = 0.5f;
  marker.color.g = 0.5f;
  marker.color.b = 1.0f;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration();
  marker_laser_pub_.publish(marker);

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
  double factor = 3;
  double d = distance(points, 0, *vx*d_*factor, *vy*d_*factor, *vth*d_*factor);
  if(d < safety_dist_) {
    *vx = 0.0;
    *vy = 0.0;
    *vth = 0.0;
  }
  return d;
}

double BaseDistance::project(std::vector<Vector2> &points, double *vx, double *vy, double *vth)
{
  double d, gx, gy, gth;
  d = grad(points, &gx, &gy, &gth);

  double l_grad = 1/sqrt(gx*gx + gy*gy + gth*gth);

  gx*=l_grad;
  gy*=l_grad;
  gth*=l_grad;

  if(d < slowdown_far_) {
    // project (vx,vy,vth) onto (gx,gy,gth)
    double dp = *vx*gx + *vy*gy + *vth*gth;
    if(dp > 0) { // we are moving towards the obstacle
      double factor = (d - slowdown_far_)/(slowdown_near_ - slowdown_far_);
      if(d < slowdown_near_) {factor=1; printf("hard "); }
      printf("projecting... (%f %f %f) ", gx *dp*factor, gy *dp*factor, gth*dp*factor);
      *vx  -= gx *dp*factor;
      *vy  -= gy *dp*factor;
      *vth -= gth*dp*factor;
    }
  }

  if(d < repelling_dist_) {
    printf("repelling\n");
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
    
              
  //printf("ld: ");

  double d = project(current_points, vx, vy, vth);
  //double d = brake(*current_points, vx, vy, vth);

  printf("d=%f  ", d);

  double dx = distance(current_points, 0, -*vx*d_, -*vy*d_, -*vth*d_);

  if(dx < safety_dist_) {  // if we are stuck in the NEXT timestep...
    printf("[braking %f < %f]", dx, safety_dist_);
    *vx = 0.0;
    *vy = 0.0;
    *vth = 0.0;
  }

  printf("\n");
}


double BaseDistance::distance(std::vector<Vector2> &points, Vector2 *nearest, double dx, double dy, double dth)
{
  Vector2 rnearest(0, 0), lnearest(0, 0);
  double rqdistance=INFINITY, ldistance=INFINITY;

  int rarea, larea;

  if(points.size() == 0)
    ROS_WARN("No points received. Maybe the laser topic is wrong?");

  for(unsigned int i=0; i < points.size(); i++) {
    double px = points[i].x*cos(dth) - points[i].y*sin(dth) + dx;
    double py = points[i].x*sin(dth) + points[i].y*cos(dth) + dy;

    double dbottom  = -py + bottom_;
    double dtop     =  py - top_;
    double dleft    = -px + left_;
    double dright   =  px - right_;

    int area=0;
    area |= BOTTOM * (dbottom > -tolerance_);
    area |= TOP    * (dtop    > -tolerance_);
    area |= LEFT   * (dleft   > -tolerance_);
    area |= RIGHT  * (dright  > -tolerance_);

    double rqdist=INFINITY, ldist=INFINITY;
    switch(area) {
    case(TOP):     ldist = dtop; break;
    case(BOTTOM):  ldist = dbottom; break;
    case(LEFT):    ldist = dleft; break;
    case(RIGHT):   ldist = dright; break;

    case(TOP | LEFT):     rqdist = dtop*dtop + dleft*dleft; break;
    case(TOP | RIGHT):    rqdist = dtop*dtop + dright*dright; break;
    case(BOTTOM | LEFT):  rqdist = dbottom*dbottom + dleft*dleft; break;
    case(BOTTOM | RIGHT): rqdist = dbottom*dbottom + dright*dright; break;
    }

    if(ldist < ldistance) {
      ldistance = ldist;
      lnearest = points[i];
      larea = area;
    }

    if(rqdist < rqdistance) {
      rqdistance = rqdist;
      rnearest = points[i];
      rarea = area;
    }
  }

  double rdistance = sqrt(rqdistance);
  if(rdistance < ldistance) {

    //printf("area=%d\n", rarea);

    if(nearest)
      *nearest = rnearest;
    return rdistance;
  } else {

    //printf("area=%d\n", larea);

    if(nearest)
      *nearest = lnearest;
    return ldistance;
  }
}
