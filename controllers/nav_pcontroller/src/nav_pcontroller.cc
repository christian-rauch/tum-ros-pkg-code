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

#include "ros/ros.h"


#include <roslib/Time.h>
#include "tf/transform_listener.h"


// The messages that we'll use
//#include <nav_robot_actions/MoveBaseState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

#ifdef JLO_BASE_CONTROL
#include <std_msgs/UInt64.h>
#include <vision_srvs/srvjlo.h>
using namespace vision_srvs;
#define JLO_IDQUERY "idquery"
#define JLO_FRAMEQUERY "framequery"
#define JLO_NAMEQUERY "namequery"
#define JLO_DELETE "del"
#define JLO_UPDATE "update"

#include <nav_pcontroller/nav_actionAction.h>
#include <actionlib/server/simple_action_server.h>
#endif

#include "BaseDistance.h"

class BasePController {
private:
  double xy_tolerance_, th_tolerance_;
  double vel_ang_max_, vel_lin_max_, acc_ang_max_, acc_lin_max_, p_;
  int loop_rate_;

  double vx_, vy_, vth_;
  double x_goal_, y_goal_, th_goal_;
  double x_now_, y_now_, th_now_;
  bool goal_set_;

  BaseDistance dist_control_;

  ros::NodeHandle n_;
  tf::TransformListener tf_;
  ros::Subscriber sub_goal_;
  ros::Publisher pub_vel_;
  ros::Publisher pub_fin_;

#ifdef JLO_BASE_CONTROL
  void newGoal(const nav_pcontroller::nav_actionGoal::ConstPtr& msg);
#else
  void newGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);
#endif
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

#ifdef JLO_BASE_CONTROL
  unsigned long namequery_jlo(std::string name);
  bool query_jlo(unsigned long id, double &x, double &y, double &theta);

  unsigned long map_jlo_id_;
  ros::ServiceClient client_;
  actionlib::SimpleActionServer<nav_pcontroller::nav_actionAction> actionserver_;

#endif
public:
  BasePController();
  void main();


};

#define MIN(a,b) ((a < b) ? (a) : (b))
#define MAX(a,b) ((a > b) ? (a) : (b))

double BasePController::p_control(double x, double p, double limit)
{
  return (x > 0) ? MIN(p*x, limit) : MAX(p*x, -limit);
}


double BasePController::limit_acc(double x, double x_old, double limit)
{
  x = MIN(x, x_old + limit);
  x = MAX(x, x_old - limit);
  return x;
}

/*
void BasePController::laserData(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  dist_control_.laser_collect(msg);
}
*/
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
  std::vector<double>& vectmp = msg.response.answer.pose;
  x = vectmp[3];
  y = vectmp[7];
  printf("Rotationmatrix\n%f %f %f\n %f %f %f\n %f %f %f\n", vectmp[0],
  vectmp[1],vectmp[2],
  vectmp[4],vectmp[5],vectmp[6],
  vectmp[8],vectmp[9],vectmp[10]);
   if (vectmp[10]==1) /*Row 2 Column 0 */
   {
      theta = acos(vectmp[0]);
      if(vectmp[4] < 0)
        theta = -theta;
      /*rot_y = -M_PI/2;*/
      /*theta = 0.0;*/
   } else if (vectmp[10]==-1)
   {
      theta = atan2(vectmp[1],vectmp[2]);
      /*rot_y = M_PI/2;*/
      /*theta = 0.0;*/
   } else {
      theta = atan2(vectmp[9], vectmp[10]);
      /*rot_y = atan2(vectemp[8], sqrt(vectemp[0]*vectemp[0] + vectemp[4]*vectemp[4]));*/
      /*theta = atan2(vectmp[5], vectmp[0]);*/
   }
   printf("=> %f theta\n", theta);
  return true;
}

void BasePController::newGoal(const nav_pcontroller::nav_actionGoal::ConstPtr& msg)
{
  printf("received new goal: %lld\n", msg->target_lo.data);
  if(query_jlo(msg->target_lo.data, x_goal_, y_goal_, th_goal_ ))
  {
   // shamelessly copied from the ogre implementation
   ROS_INFO("got goal: %f %f %f", x_goal_, y_goal_, th_goal_);
   goal_set_ = true;

   ros::Rate loop(loop_rate_);
    while(n_.ok() && goal_set_)
    {
     if(actionserver_.isPreemptRequested())
     {
       goal_set_ = false;
       stopRobot();
       actionserver_.setPreempted();
       break;
     }
     cycle();
     loop.sleep();
    }
  }

}
#else
void BasePController::newGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  x_goal_ = msg->pose.position.x;
  y_goal_ = msg->pose.position.y;

  // shamelessly copied from the ogre implementation
  const geometry_msgs::Quaternion &q = msg->pose.orientation;
  th_goal_ = atan2(2.0*q.x*q.y + 2.0*q.w*q.z, 1.0-(2.0*q.y*q.y + 2.0*q.z*q.z));

  ROS_INFO("got goal: %f %f %f", x_goal_, y_goal_, th_goal_);
  goal_set_ = true;
}
#endif

BasePController::BasePController() : n_("~")
#ifdef JLO_BASE_CONTROL
  ,
  actionserver_(n_, "nav_action", boost::bind(&BasePController::newGoal, this, _1))
#endif
{
#ifdef JLO_BASE_CONTROL
  ros::service::waitForService("/located_object");
  client_ = n_.serviceClient<srvjlo>("/located_object", true);
  map_jlo_id_ = namequery_jlo("/map");
  pub_vel_ =  n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
#else
  sub_goal_ = n_.subscribe("/goal", 1, &BasePController::newGoal, this);
  pub_vel_ =  n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  pub_fin_ =  n_.advertise<std_msgs::String>("/base_finished", 1);
#endif
  vx_=0; vy_=0; vth_=0;  // \todo: get from base driver

  n_.param<double>("xy_tolerance", xy_tolerance_, 0.005);
  n_.param<double>("th_tolerance", th_tolerance_, 0.005);
  n_.param<double>("vel_ang_max", vel_ang_max_, 0.2);
  n_.param<double>("vel_lin_max", vel_lin_max_, 0.2);
  n_.param<double>("acc_ang_max", acc_ang_max_, 0.4);
  n_.param<double>("acc_lin_max", acc_lin_max_, 0.4);
  n_.param<int>("loop_rate", loop_rate_, 30);
  n_.param<double>("p", p_, 1.2);

  goal_set_ = false;

  //CHANGED
  //  dist_control_.setFootprint(0.42, -0.3075, 0.3075, -0.42, 0.0);
  dist_control_.setFootprint(0.309, -0.43, 0.43, -0.309, 0.0);
}


void BasePController::main()
{
  ros::Rate loop(loop_rate_);

  while(n_.ok()) {
    ros::spinOnce();
    if(goal_set_)
      cycle();
    loop.sleep();
  }
}


//! retrieves tf pose and updates (x_now_, y_now_, th_now_)
bool BasePController::retrieve_pose()
{
  tf::Stamped<tf::Pose> global_pose;
  tf::Stamped<tf::Pose> robotPose;
  robotPose.setIdentity();
  robotPose.frame_id_ = "base_link";
  robotPose.stamp_ = ros::Time();
  try
  {
    tf_.transformPose("/map", robotPose, global_pose);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("no localization information yet");
    return false;
  }

  // find out where we are now
  x_now_ = global_pose.getOrigin().x();
  y_now_ = global_pose.getOrigin().y();

  double unused_pitch, unused_roll;
  btMatrix3x3 mat =  global_pose.getBasis();
  mat.getEulerYPR(th_now_, unused_pitch, unused_roll);

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
  if(!retrieve_pose()) {
    stopRobot();
    return;
  }

  compute_p_control();

  dist_control_.compute_distance_keeping(&vx_, &vy_, &vth_);

  sendVelCmd(vx_, vy_, vth_);

  if(comparePoses(x_goal_, y_goal_, th_goal_, x_now_, y_now_, th_now_)) {
#ifdef JLO_BASE_CONTROL
    nav_pcontroller::nav_actionResult result;
    result.distance.data = sqrt((x_goal_-x_now_)*(x_goal_-x_now_) + (y_goal_-y_now_)*(y_goal_-y_now_));
    char tmp [512];
    sprintf(tmp, "nav_pcontroller: Reached goal %f (%f rot reached %f commanded)\n", result.distance.data, th_now_, th_goal_);
    ROS_INFO(tmp);
    actionserver_.setSucceeded(result);
    goal_set_ = false;
#else
    std_msgs::String s;
    s.data = "finished";
    pub_fin_.publish(s);
    goal_set_ = false;
#endif
  }
#ifdef JLO_BASE_CONTROL
  else{
    nav_pcontroller::nav_actionFeedback feedback;
    feedback.speed.data = max(sqrt(vx_ * vx_+ vy_ * vy_) , vth_);
    feedback.distance.data = sqrt((x_goal_-x_now_)*(x_goal_-x_now_) + (y_goal_-y_now_)*(y_goal_-y_now_));
    actionserver_.publishFeedback(feedback);
  }
#endif
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
#ifdef JLO_BASE_CONTROL
  ros::spin();
#else
  pc.main();
#endif
  return 0;
}
