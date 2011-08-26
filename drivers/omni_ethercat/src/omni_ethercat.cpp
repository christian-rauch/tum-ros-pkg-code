/*
 * Copyright (C) 2009 by Ingo Kresse <kresse@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <unistd.h>
#include <sys/time.h>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <rosie_msgs/PowerState.h>


extern "C" {
#include "omnilib/omnilib.h"
}

#define LIMIT(x, l) ( (x>l) ? l : (x<-l) ? -l : x )

class Omnidrive
{
private:
  ros::NodeHandle n_;
  diagnostic_updater::Updater diagnostic_;
  ros::Publisher power_pub_;
  ros::Subscriber power_sub_;
  ros::Time watchdog_time_;
  double drive_[3], drive_last_[3];
  bool runstop_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string power_name_;
  void cmdArrived(const geometry_msgs::Twist::ConstPtr& msg);
  void stateUpdate(diagnostic_updater::DiagnosticStatusWrapper &s);
  void powerCommand(const rosie_msgs::PowerState::ConstPtr& msg);
  void runstopReceiver(const std_msgs::BoolConstPtr &msg);
public:
  Omnidrive();
  void main();
};


Omnidrive::Omnidrive() : n_("omnidrive"), diagnostic_(), runstop_(false)
{
  diagnostic_.setHardwareID("omnidrive");
  diagnostic_.add("Base", this, &Omnidrive::stateUpdate);
  n_.param("frame_id", frame_id_, std::string("/odom"));
  n_.param("child_frame_id", child_frame_id_, std::string("/base_link"));
  n_.param("power_name", power_name_, std::string("Wheels"));

  power_pub_ = n_.advertise<rosie_msgs::PowerState>("/power_state", 1);
  power_sub_ = n_.subscribe<rosie_msgs::PowerState>("/power_command", 16, &Omnidrive::powerCommand, this);


  for(int i=0; i < 3; i++) {
    drive_last_[i] = 0;
    drive_[i] = 0;
  }

  watchdog_time_ = ros::Time::now();
}

void Omnidrive::cmdArrived(const geometry_msgs::Twist::ConstPtr& msg)
{
  // NOTE: No need for synchronization since this is called inside spinOnce() in the main loop

  drive_[0] = msg->linear.x;
  drive_[1] = msg->linear.y;
  drive_[2] = msg->angular.z;

  if(msg->linear.z == -1) {
    // emergency brake!
    for(int i=0; i < 3; i++) {
      drive_last_[i] = 0;
      drive_[i] = 0;
    }
  }

  watchdog_time_ = ros::Time::now();
}


void Omnidrive::runstopReceiver(const std_msgs::BoolConstPtr &msg)
{
  runstop_ = msg->data;
}

void Omnidrive::stateUpdate(diagnostic_updater::DiagnosticStatusWrapper &s)
{
  int estop;
  char drive[4];
  omnidrive_status(&drive[0], &drive[1], &drive[2], &drive[3], &estop);

  bool operational = (drive[0] == '4' &&
                      drive[1] == '4' &&
                      drive[2] == '4' &&
                      drive[3] == '4' &&
                      estop == 0);

  if(operational)
    s.summary(0, "Operational");
  else
    s.summary(1, "Down");
  
  for(int i=0; i < 4; i++)
    s.add(std::string("status drive ")+(char) ('1' + i),
          std::string("")+drive[i]);
  s.add("Emergency Stop", (estop) ? "== pressed ==" : "released");

  commstatus_t comm = omnidrive_commstatus();

  for(int i=0; i < 4; i++)
    s.addf(std::string("comm status drive ")+(char) ('1' + i),
//    s.addf("cstatus drive",
           "0x%02X, %s, %s operational",
             comm.slave_state[i],
             comm.slave_online[i] ? "online" : "offline",
             comm.slave_operational[i] ? "" : "not");

  s.addf("master state", "Link is %s, %d slaves, AL states: 0x%02X",
           comm.master_link ? "up" : "down",
           comm.master_slaves_responding,
           comm.master_al_states);

  s.addf("working counter", "%d, %s",
           comm.working_counter,
           comm.working_counter_state == 2 ? "complete" : "incomplete");

  rosie_msgs::PowerState power;
  power.name = power_name_;
  power.enabled = operational;

  power_pub_.publish(power);
}

void Omnidrive::powerCommand(const rosie_msgs::PowerState::ConstPtr& msg)
{
  if(msg->name == power_name_)
  {
    int estop;
    char drive[4];
    omnidrive_status(&drive[0], &drive[1], &drive[2], &drive[3], &estop);

    bool power_state = (drive[0] == '4' &&
                        drive[1] == '4' &&
                        drive[2] == '4' &&
                        drive[3] == '4' &&
                        estop == 0);

    if(msg->enabled == true && power_state == false)
    {
      omnidrive_recover();
      omnidrive_poweron();
    }

    if(msg->enabled == false && power_state == true)
    {
      omnidrive_poweroff();
    }
    
  }
}

void Omnidrive::main()
{
  double speed, acc_max, t, radius, drift;
  int tf_frequency, runstop_frequency;
  const int loop_frequency = 250; // 250Hz update frequency

  n_.param("speed", speed, 0.1);
  // default acc: brake from max. speed to 0 within 1.5cm
  n_.param("acceleration", acc_max, 0.5*speed*speed/0.015);
  // radius of the robot
  n_.param("radius", radius, 0.6);
  n_.param("tf_frequency", tf_frequency, 50);
  n_.param("runstop_frequency", runstop_frequency, 10);
  n_.param("watchdog_period", t, 0.5);
  ros::Duration watchdog_period(t);
  n_.param("odometry_correction", drift, 1.0);

  // set acceleration to correct scale
  acc_max /= loop_frequency;

  if(omnidrive_init() != 0) {
    ROS_ERROR("failed to initialize omnidrive");
    ROS_ERROR("check dmesg and try \"sudo /etc/init.d/ethercat restart\"");
    return;
  }
  omnidrive_set_correction(drift);

  tf::TransformBroadcaster transforms;

  ros::Subscriber sub = n_.subscribe("/cmd_vel", 10, &Omnidrive::cmdArrived, this);
  ros::Subscriber runstop_sub = n_.subscribe("/soft_runstop", 1, &Omnidrive::runstopReceiver, this);
  ros::Publisher hard_runstop_pub = n_.advertise<std_msgs::Bool>("/hard_runstop", 1);

  double x=0, y=0, a=0;

  int tf_publish_counter=0;
  int tf_send_rate = loop_frequency / tf_frequency;

  int runstop_publish_counter=0;
  int runstop_send_rate = loop_frequency / runstop_frequency;

  ros::Rate r(loop_frequency);

  while(n_.ok()) {

    omnidrive_odometry(&x, &y, &a);

    if(ros::Time::now() - watchdog_time_ > watchdog_period || runstop_) {
      // emergency brake!
      omnidrive_drive(0, 0, 0);
      
      if((drive_[0] != 0 || drive_[1] != 0 || drive_[2] !=0)
         && !runstop_)
        ROS_WARN("engaged watchdog!");

      for(int i=0; i < 3; i++) {
        drive_last_[i] = 0;
        drive_[i] = 0;
      }

      watchdog_time_ = ros::Time::now();
    }

    for(int i=0; i < 3; i++) {
      // acceleration limiting
      double acc = drive_[i] - drive_last_[i];
      double fac_rot = (i == 2) ? 1.0/radius : 1.0;
      acc = LIMIT(acc, acc_max*fac_rot*fac_rot);
      drive_[i] = drive_last_[i] + acc;

      // velocity limiting
      drive_[i] = LIMIT(drive_[i], speed*fac_rot);

      drive_last_[i] = drive_[i];
    }

    omnidrive_drive(drive_[0], drive_[1], drive_[2]);

    // publish odometry readings
    if(++tf_publish_counter == tf_send_rate) {
      tf::Quaternion q;
      q.setRPY(0, 0, a);
      tf::Transform pose(q, tf::Point(x, y, 0.0));
      transforms.sendTransform(tf::StampedTransform(pose, ros::Time::now(), frame_id_, child_frame_id_));
      tf_publish_counter = 0;
    }

    // publish hard runstop state
    if(++runstop_publish_counter == runstop_send_rate) {
      int runstop=0;
      omnidrive_status(0,0,0,0,&runstop);
      std_msgs::Bool msg;
      msg.data = (runstop != 0);
      hard_runstop_pub.publish(msg);
      runstop_publish_counter = 0;
    }

    // process incoming messages
    ros::spinOnce();

    diagnostic_.update();
    r.sleep();
  }

  omnidrive_shutdown();
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "omni_ethercat");

  Omnidrive drive;
  drive.main();

  return 0;
}
