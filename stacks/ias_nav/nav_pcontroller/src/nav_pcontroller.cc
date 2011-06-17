/*
 * Copyright (c) 2009, Ingo Kresse <kresse@in.tum.de>, U. Klank
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
#include <stdio.h>

// For min/max
#include <algorithm>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include "tf/transform_listener.h"


// The messages that we'll use
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>

#ifdef JLO_BASE_CONTROL
#include <std_msgs/UInt64.h>
#include <vision_srvs/srvjlo.h>
using namespace vision_srvs;
#define JLO_IDQUERY "idquery"
#define JLO_FRAMEQUERY "framequery"
#define JLO_NAMEQUERY "namequery"
#define JLO_DELETE "del"
#define JLO_UPDATE "update"

#include <navp_action/nav_actionAction.h>
#endif

#include "BaseDistance.h"

class BasePController {
private:

  double xy_tolerance_, th_tolerance_;
  ros::Duration fail_timeout_;
  double fail_velocity_;
  double vel_ang_max_, vel_lin_max_, acc_ang_max_, acc_lin_max_, p_;
  int loop_rate_;

  double vx_, vy_, vth_;
  double x_goal_, y_goal_, th_goal_;
  double x_now_, y_now_, th_now_;
  bool goal_set_, keep_distance_;

  std::string global_frame_;
  std::string base_link_frame_;

  BaseDistance dist_control_;

  ros::NodeHandle n_;
  tf::TransformListener tf_;
  ros::Subscriber sub_goal_;
  ros::Publisher pub_vel_;

  ros::Time low_speed_time_;

  boost::mutex lock;

  void newGoal(const geometry_msgs::PoseStamped &msg);
  void newGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);

  //  void laserData(const sensor_msgs::LaserScan::ConstPtr& msg);

  void cycle();
  void stopRobot();
  void sendVelCmd(double vx, double vy, double vth);
  bool comparePoses(double x1, double y1, double a1,
                    double x2, double y2, double a2);

  bool retrieve_pose();
  double p_control(double x, double p, double limit);
  double limit_acc(double x, double x_old, double limit);
  void compute_p_control();

  void parseParams();

#ifdef JLO_BASE_CONTROL
  bool jlo_enabled_;
  unsigned long map_jlo_id_;
  ros::ServiceClient client_;
  actionlib::SimpleActionServer<navp_action::nav_actionAction> *jlo_actionserver_;

  unsigned long namequery_jlo(std::string name);
  bool query_jlo(unsigned long id, double &x, double &y, double &theta);

  void newJloActionGoal();
  void preemptJloActionGoal();
#endif

  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> move_base_actionserver_;

  void newMoveBaseGoal();
  void preemptMoveBaseGoal();

public:
  BasePController();
  ~BasePController();
  void main();
};

double BasePController::p_control(double x, double p, double limit)
{
  return (x > 0) ? std::min(p*x, limit) : std::max(p*x, -limit);
}


double BasePController::limit_acc(double x, double x_old, double limit)
{
  x = std::min(x, x_old + limit);
  x = std::max(x, x_old - limit);
  return x;
}

/*
void BasePController::laserData(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  dist_control_.laser_collect(msg);
}
*/

BasePController::BasePController()
  : n_("~"),
    move_base_actionserver_(n_, "move_base")
{
  parseParams();

  move_base_actionserver_.registerGoalCallback(boost::bind(&BasePController::newMoveBaseGoal, this));
  move_base_actionserver_.registerPreemptCallback(boost::bind(&BasePController::preemptMoveBaseGoal, this));
  
#ifdef JLO_BASE_CONTROL
  if(jlo_enabled_)
  {
    jlo_actionserver_ = new actionlib::SimpleActionServer<navp_action::nav_actionAction>(n_, "nav_action");

    jlo_actionserver_->registerGoalCallback(boost::bind(&BasePController::newJloActionGoal, this));
    jlo_actionserver_->registerPreemptCallback(boost::bind(&BasePController::preemptJloActionGoal, this));

    ros::service::waitForService("/located_object");
    client_ = n_.serviceClient<srvjlo>("/located_object", true);
    map_jlo_id_ = namequery_jlo(global_frame_);
  }
  else
    jlo_actionserver_ = 0;
#endif

  sub_goal_ = n_.subscribe("/goal", 1, &BasePController::newGoal, this);
  pub_vel_ =  n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  vx_=0; vy_=0; vth_=0;  // \todo: get from base driver

  goal_set_ = false;
}

BasePController::~BasePController()
{
  #ifdef JLO_BASE_CONTROL
  delete jlo_actionserver_;
  #endif
}

void BasePController::main()
{
  ros::Rate loop(loop_rate_);
  ros::AsyncSpinner spinner(1);

  spinner.start();
  while(n_.ok()) {
    if(goal_set_)
      cycle();
    loop.sleep();
  }
}

void BasePController::parseParams()
{
  n_.param<double>("xy_tolerance", xy_tolerance_, 0.005);
  n_.param<double>("th_tolerance", th_tolerance_, 0.005);

  double tmp_fail_timeout;
  n_.param<double>("fail_timeout", tmp_fail_timeout, 5.0);
  fail_timeout_ = ros::Duration(tmp_fail_timeout);

  n_.param<double>("fail_velocity", fail_velocity_, 0.02);
  n_.param<double>("vel_ang_max", vel_ang_max_, 0.2);
  n_.param<double>("vel_lin_max", vel_lin_max_, 0.2);
  n_.param<double>("acc_ang_max", acc_ang_max_, 0.4);
  n_.param<double>("acc_lin_max", acc_lin_max_, 0.4);
  n_.param<int>("loop_rate", loop_rate_, 30);
  n_.param<double>("p", p_, 1.2);
  n_.param<bool>("keep_distance", keep_distance_, true);

#ifdef JLO_BASE_CONTROL
  n_.param<bool>("enable_jlo", jlo_enabled_, true);
#endif

  n_.param<std::string>("global_frame", global_frame_, "/map");
  n_.param<std::string>("base_link_frame", base_link_frame_, "/base_link");

  //CHANGED
  //  dist_control_.setFootprint(0.42, -0.3075, 0.3075, -0.42, 0.0);
  /*(0.309, -0.43, 0.43, -0.309, 0.0);*/
  double front, rear, left, right, tolerance;
  std::string stName;
  n_.param("speed_filter_name", stName, std::string("/speed_filter"));
  n_.param(stName + "/footprint/left", left, 0.309);
  n_.param(stName + "/footprint/right", right, -0.309);
  n_.param(stName + "/footprint/front", front, 0.43);
  n_.param(stName + "/footprint/rear", rear, -0.43);
  n_.param(stName + "/footprint/tolerance", tolerance, 0.0);
  
  dist_control_.setFootprint(front, rear, left, right, tolerance);
}

#ifdef JLO_BASE_CONTROL
unsigned long BasePController::namequery_jlo(std::string name)
{
  srvjlo msg;
  msg.request.query.name = name;
  msg.request.command = JLO_NAMEQUERY;

  if(!client_.isValid())
  {
    ros::service::waitForService("/located_object");
    client_ = n_.serviceClient<srvjlo>("/located_object", true);
  }
  if (!client_.call(msg))
  {
    printf("Error asking jlo namequery %s!\n", name.c_str());
    return 1;
  }
  else if (msg.response.error.length() > 0)
  {
    printf("Error from jlo: %s!\n", msg.response.error.c_str());
    return 1;
  }
  return msg.response.answer.id;
}

bool BasePController::query_jlo(unsigned long id, double &x, double &y, double &theta)
{
  srvjlo msg;
  msg.request.query.id = id;
  msg.request.query.parent_id = map_jlo_id_;
  msg.request.command = JLO_FRAMEQUERY;
  if(id == 1)
     msg.request.command = JLO_IDQUERY;

  if(!client_.isValid())
  {
    ros::service::waitForService("/located_object");
    client_ = n_.serviceClient<srvjlo>("/located_object", true);
  }
  if (!client_.call(msg))
  {

    printf("Error from jlo: %s!\n", msg.response.error.c_str());
    return false;
  }
  else if (msg.response.error.length() > 0)
  {
    printf("Error from jlo: %s!\n", msg.response.error.c_str());
    return false;
  }
  vision_msgs::partial_lo::_pose_type &vectmp = msg.response.answer.pose;
  x = vectmp[3];
  y = vectmp[7];
  printf("Rotationmatrix\n%f %f %f\n %f %f %f\n %f %f %f\n", vectmp[0],
  vectmp[1],vectmp[2],
  vectmp[4],vectmp[5],vectmp[6],
  vectmp[8],vectmp[9],vectmp[10]);

  theta = atan2(vectmp[4], vectmp[0]);

  printf("=> %f theta\n", theta);

  return true;
}

void BasePController::newJloActionGoal()
{
  if(move_base_actionserver_.isActive())
    preemptMoveBaseGoal();
      
  navp_action::nav_actionGoal::ConstPtr msg = jlo_actionserver_->acceptNewGoal();

  // To be able to reconfigure the base controller on the fly, whe read the parameters whenever we receive a goal
  parseParams();

  // Jlo might block for quite some time, so we do not want to hold
  // the lock while calling jlo. Rather, we first store the goal in
  // temporary variables and copy after the jlo call.
  double x_goal, y_goal, th_goal;
  if(query_jlo(msg->target_lo.data, x_goal, y_goal, th_goal))
  {
    boost::mutex::scoped_lock curr_lock(lock);

    x_goal_ = x_goal;
    y_goal_ = y_goal;
    th_goal_ = th_goal;

    low_speed_time_ = ros::Time::now();

    ROS_INFO("received goal: %f %f %f", x_goal_, y_goal_, th_goal_);
    goal_set_ = true;
  }
  else
  {
    ROS_WARN("Jlo query for goal %ld failed.", msg->target_lo.data);
    jlo_actionserver_->setAborted();
  }
}

void BasePController::preemptJloActionGoal()
{
  boost::mutex::scoped_lock curr_lock(lock);

  goal_set_ = false;
  stopRobot();
  jlo_actionserver_->setPreempted();
}
#endif

void BasePController::newMoveBaseGoal()
{
  #ifdef JLO_BASE_CONTROL
  if(jlo_actionserver_ && jlo_actionserver_->isActive())
    preemptJloActionGoal();
  #endif

  move_base_msgs::MoveBaseGoal::ConstPtr msg = move_base_actionserver_.acceptNewGoal();

  // To be able to reconfigure the base controller on the fly, whe read the parameters whenever we receive a goal
  //parseParams();

  newGoal(msg->target_pose);
  ROS_INFO("received goal: %f %f %f", x_goal_, y_goal_, th_goal_);
}

void BasePController::preemptMoveBaseGoal()
{
  boost::mutex::scoped_lock curr_lock(lock);

  goal_set_ = false;
  stopRobot();
  move_base_actionserver_.setPreempted();
}

void BasePController::newGoal(const geometry_msgs::PoseStamped &msg)
{
  boost::mutex::scoped_lock curr_lock(lock);

  geometry_msgs::PoseStamped goal;


  try
  {
    tf_.transformPose(global_frame_, msg, goal);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("no localization information yet %s",ex.what());
    return;
  }

  x_goal_ = goal.pose.position.x;
  y_goal_ = goal.pose.position.y;

  // th = atan2(r21/2,r11/2)
  const geometry_msgs::Quaternion &q = goal.pose.orientation;
  th_goal_ = atan2(q.x*q.y + q.w*q.z, 0.5 - q.y*q.y -q.z*q.z);

  ROS_INFO("got goal: %f %f %f", x_goal_, y_goal_, th_goal_);
  
  low_speed_time_ = ros::Time::now();

  goal_set_ = true;
}

void BasePController::newGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
#ifdef JLO_BASE_CONTROL
  if(jlo_actionserver_ && jlo_actionserver_->isActive())
    preemptJloActionGoal();
#endif
  if(move_base_actionserver_.isActive())
    preemptMoveBaseGoal();

  newGoal(*msg);
}

//! retrieves tf pose and updates (x_now_, y_now_, th_now_)
bool BasePController::retrieve_pose()
{
  tf::StampedTransform global_pose;
  try
  {
    tf_.lookupTransform(global_frame_, base_link_frame_, ros::Time(0), global_pose);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("no localization information yet %s",ex.what());
    return false;
  }

  // find out where we are now
  x_now_ = global_pose.getOrigin().x();
  y_now_ = global_pose.getOrigin().y();

  // th = atan2(r_21/2, r_11/2)
  const btQuaternion &q = global_pose.getRotation();
  th_now_ = atan2(q.x()*q.y() + q.w()*q.z(), 0.5 - q.y()*q.y() - q.z()*q.z());

  return true;
}

void BasePController::compute_p_control()
{
  //compute differences (world space)
  double x_diff = x_goal_ - x_now_;
  double y_diff = y_goal_ - y_now_;

  // \todo: clean this ugly code
  double th_diff = fmod(th_goal_ - th_now_, 2*M_PI);
  if(th_diff > M_PI) th_diff = th_diff - 2*M_PI;
  if(th_diff < -M_PI) th_diff = th_diff + 2*M_PI;

  // transform to robot space
  double dx =  x_diff*cos(th_now_) + y_diff*sin(th_now_);
  double dy = -x_diff*sin(th_now_) + y_diff*cos(th_now_);
  double dth = th_diff;

  // do p-controller with limit (robot space)
  double vel_x =  p_control(dx, p_, vel_lin_max_);
  double vel_y =  p_control(dy, p_, vel_lin_max_);
  double vel_th = p_control(dth, p_, vel_ang_max_);

  // limit acceleration (robot space)
  vel_x  = limit_acc(vel_x, vx_, acc_lin_max_/loop_rate_);
  vel_y  = limit_acc(vel_y, vy_, acc_lin_max_/loop_rate_);
  vel_th = limit_acc(vel_th, vth_, acc_ang_max_/loop_rate_);

  // store resulting velocity
  vx_  = vel_x;
  vy_  = vel_y;
  vth_ = vel_th;
}

#ifndef max
#define max(A,B) (A) > (B) ? (A) : (B)
#endif
void BasePController::cycle()
{
  boost::mutex::scoped_lock curr_lock(lock);

  if(!retrieve_pose()) {
    stopRobot();
    return;
  }

  compute_p_control();
  
  if (keep_distance_)
    dist_control_.compute_distance_keeping(&vx_, &vy_, &vth_);

  sendVelCmd(vx_, vy_, vth_);

  if(comparePoses(x_goal_, y_goal_, th_goal_, x_now_, y_now_, th_now_)) {
#ifdef JLO_BASE_CONTROL
    if(jlo_actionserver_ && jlo_actionserver_->isActive())
    {
      navp_action::nav_actionResult result;
      result.distance.data = sqrt((x_goal_-x_now_)*(x_goal_-x_now_) + (y_goal_-y_now_)*(y_goal_-y_now_));
      ROS_INFO("nav_pcontroller: Reached goal %f (%f rot reached %f commanded)\n",
        result.distance.data, th_now_, th_goal_);
      jlo_actionserver_->setSucceeded(result);
    }
#endif
    if(move_base_actionserver_.isActive())
      move_base_actionserver_.setSucceeded();
    goal_set_ = false;
    stopRobot();
  }
  // Sort of a bad hack. It might be a bad idea to 'unify' angular
  // and linear velocity and just take te maximum.
  double velocity = max(sqrt(vx_ * vx_+ vy_ * vy_) , vth_);

  if( velocity < fail_velocity_ )
  {
    if( ros::Time::now() - low_speed_time_ > fail_timeout_ )
    {
      goal_set_ = false;
      stopRobot();
#ifdef JLO_BASE_CONTROL
      if(jlo_actionserver_ && jlo_actionserver_->isActive())
      {
        navp_action::nav_actionResult result;
        result.distance.data = sqrt((x_goal_-x_now_)*(x_goal_-x_now_) + (y_goal_-y_now_)*(y_goal_-y_now_));
        jlo_actionserver_->setAborted(result);
      }
#endif
      if(move_base_actionserver_.isActive())
        move_base_actionserver_.setAborted();
      return;
    }
  }
  else
    low_speed_time_ = ros::Time::now();

#ifdef JLO_BASE_CONTROL
  if(jlo_actionserver_ && jlo_actionserver_->isActive())
  {
    navp_action::nav_actionFeedback feedback;
    feedback.speed.data = velocity;
    feedback.distance.data = sqrt((x_goal_-x_now_)*(x_goal_-x_now_) + (y_goal_-y_now_)*(y_goal_-y_now_));
    jlo_actionserver_->publishFeedback(feedback);
  }
#endif
  if(move_base_actionserver_.isActive())
  {
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position.header.stamp = ros::Time::now();
    feedback.base_position.header.frame_id = global_frame_;
    feedback.base_position.pose.position.x = x_now_;
    feedback.base_position.pose.position.y = y_now_;
    feedback.base_position.pose.position.z = 0.0;
    feedback.base_position.pose.orientation = tf::createQuaternionMsgFromYaw(th_now_);
    move_base_actionserver_.publishFeedback(feedback);
  }
}

#define ANG_NORM(a) atan2(sin((a)),cos((a)))

void BasePController::stopRobot()
{
  sendVelCmd(0.0,0.0,0.0);
}

void BasePController::sendVelCmd(double vx, double vy, double vth)
{
  geometry_msgs::Twist cmdvel;

  cmdvel.linear.x = vx;
  cmdvel.linear.y = vy;
  cmdvel.angular.z = vth;

  pub_vel_.publish(cmdvel);
}


bool BasePController::comparePoses(double x1, double y1, double a1,
                                   double x2, double y2, double a2)
{
  bool res;
  if((fabs(x2-x1) <= xy_tolerance_) &&
     (fabs(y2-y1) <= xy_tolerance_) &&
     (fabs(ANG_NORM(ANG_NORM(a2)-ANG_NORM(a1))) <= th_tolerance_))
    res = true;
  else
    res = false;
  return(res);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nav_pcontroller");

  BasePController pc;
  pc.main();
  return 0;
}
