/*
 * Copyright (c) 2009 Dejan Pangercic <pangercic -=- cs.tum.edu>
 *
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
 *
 * $Id: sweep_laser_scan_cli_client.cpp 121 2009-08-28 12:46:27Z dejanpan $
 *
 */

/**
   @mainpage

   @htmlinclude manifest.html

   \author Dejan Pangercic

   @b This Player client encapsulated into a ROS node controls (rotates)
   PowerCube modules and thus helps acquiring laser point cloud data. It 
   has been used with David system (http://www.david-laserscanner.com/)
   and Sick LMS400 laser range scanner. In latter case it also enables 
   logging of data.
**/


///////////////////////////////////////////////////////////////////////////////////////
//ros includes
///////////////////////////////////////////////////////////////////////////////////////
#include "ros/ros.h"
#include <mapping_srvs/TriggerSweep.h>
using namespace mapping_srvs;

///////////////////////////////////////////////////////////////////////////////////////
//player includes
///////////////////////////////////////////////////////////////////////////////////////
#include <ncurses.h>
#include <playerc.h>
#include <string>
#include <math.h>
#include <stdio.h>
#include <cstdlib>
#include <getopt.h>
#include <iostream>
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////
//player defines
///////////////////////////////////////////////////////////////////////////////////////
// Convert radians to degrees
#define RAD2DEG(r) (float)((r) * 180 / M_PI)
// Convert degrees to radians
#define DEG2RAD(d) (float)((d) * M_PI / 180)
#define HOSTNAME "localhost"
// what diference between angles is considered 0?
#define EPS 0.01


class Amtec_Sweep_service{

private:
  ros::NodeHandle nh_;
public:
  ros::ServiceServer service_;
  //set options to some impossible values
  string object_;
  double angle_filename_, start_angle_, end_angle_;
  int interface_, rot_joint_, is_lms400_;
  double rot_speed_;
  bool debug_output_;
  ///////////////////////////////////////////////////////////////////////////////////////
  //Contructor
  ///////////////////////////////////////////////////////////////////////////////////////
  Amtec_Sweep_service()
  {
    nh_.param("~start_angle", start_angle_, 0.0);
    nh_.param("~end_angle", end_angle_, 10.0);
    nh_.param("~interface", interface_, 1);
    //are we scanning using lms400 and player logging
    nh_.param("~is_lms400_", is_lms400_, 0);
    nh_.param("~rot_joint", rot_joint_, 4);
    nh_.param("~rot_speed", rot_speed_, 0.1);
    service_ = nh_.advertiseService("amtec_sweep", &Amtec_Sweep_service::amtec_sweep_main, this);  
    debug_output_ = false;
    if(debug_output_)
      ROS_INFO("Amtec_Sweep_service has been initialized");  
  }

  ///////////////////////////////////////////////////////////////////////////////////////
  //Destructor
  ///////////////////////////////////////////////////////////////////////////////////////
  virtual ~Amtec_Sweep_service () { }

  ///////////////////////////////////////////////////////////////////////////////////////
  //service function, used to be main as player client
  ///////////////////////////////////////////////////////////////////////////////////////

  bool
  amtec_sweep_main (TriggerSweep::Request &req, TriggerSweep::Response &resp)
  {
    //update parameters on the fly
    update_parameters_from_server();

    if(debug_output_)
      ROS_INFO("In amtec_sweep_main");
    angle_filename_ = req.angle_filename;
    object_ = req.object;
        
    playerc_client_t *client;
    void *rdevice;
    int port = 6665;
    playerc_actarray_t *device_actarray;
    playerc_laser_t *device_laser;
    playerc_log_t *device_log;
    client = playerc_client_create (NULL, HOSTNAME, port);
    playerc_client_connect (client);
    
    //return from go_to_pose
    float next_pos;
    
    device_actarray  = playerc_actarray_create (client, interface_);
    device_laser  = playerc_laser_create (client, 0);
    if(is_lms400_)
      device_log = playerc_log_create      (client, 0);
    
    playerc_actarray_subscribe (device_actarray, PLAYER_OPEN_MODE);
    if(is_lms400_)
      playerc_log_subscribe      (device_log, PLAYER_OPEN_MODE);
    playerc_laser_subscribe      (device_laser, PLAYER_OPEN_MODE);
    
    ////playerc_log_set_filename (device_log, g_options.log_filename.c_str());
    
    for (int t = 0; t < 20; t++)
      rdevice = playerc_client_read (client);
    
    int current_actuator = device_actarray->actuators_count - 1;
    
    rdevice = playerc_client_read (client);
    //print_data (device_actarray);
    
    //enable logging
    if(is_lms400_)
      {
        char angle_tmp[100];
        int angle_int = round(req.angle_filename);
        sprintf (angle_tmp, "%d",  angle_int);
        string logFileName = req.object + "_" +  string(angle_tmp) + "_" + ".log";
        playerc_log_set_filename (device_log, logFileName.c_str());
        playerc_log_set_write_state (device_log, 1);
        if(debug_output_)
          ROS_INFO("Logging : enabled"); 
      }
    //sweeping
    next_pos = go_to_pose(client, device_actarray, rot_joint_, start_angle_, end_angle_);
    if(debug_output_)
      ROS_INFO("Sweeping to next_pos: %f", next_pos);
    sweep (client, device_actarray, rot_joint_, next_pos, rot_speed_);

    //wait until end angle reached
    while (fabs(next_pos  -   RAD2DEG (device_actarray->actuators_data[rot_joint_].position)) > EPS)
      {
        playerc_client_read (client);
        //printf ("Angle  %f\n", fabs(next_pos  -   RAD2DEG (device_actarray->actuators_data[rot_joint_].position)));
      }
    
    //logging disabled
    if(is_lms400_)
      {
        playerc_log_set_write_state (device_log, 0);
        if(debug_output_)
          ROS_INFO ("Logging : disabled"); 
      }
    //clean up player
    if(debug_output_)
      ROS_INFO("unsubscribing\n");
    playerc_actarray_unsubscribe (device_actarray);
    playerc_actarray_destroy (device_actarray);
    
    if(is_lms400_)
      {
        playerc_log_unsubscribe (device_log);
        playerc_log_destroy (device_log);
      }

    playerc_laser_unsubscribe (device_laser);
    playerc_laser_destroy (device_laser);
    
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////
  // sweep one cube
  //////////////////////////////////////////////////////////////////////////////////
  void
  sweep (playerc_client_t *client, playerc_actarray_t *device, 
         int actuator, float next_pos, float speed)
  {
    // Set the speed to move with
    playerc_actarray_speed_config (device, actuator, speed);
    if(debug_output_)
      ROS_INFO ("Sweeping. Next position is %f \n", next_pos);
    playerc_actarray_position_cmd (device, actuator, DEG2RAD (next_pos));
    
  }

  //////////////////////////////////////////////////////////////////////////////////
  // find the start_angle
  //////////////////////////////////////////////////////////////////////////////////
  float
  go_to_pose (playerc_client_t *client, playerc_actarray_t *device, 
              int actuator, float min_angle, float max_angle)
  {
    float next_pos;
    
    // Find the current position
    float current_pos = RAD2DEG (device->actuators_data[actuator].position);
    
    // Go to either min_angle or max_angle depending which one is closer
    if (current_pos < min_angle)
      next_pos = min_angle;
    
    if (current_pos > max_angle)
      next_pos = max_angle;
    
    if (fabs (current_pos - min_angle) > fabs (current_pos - max_angle))
      next_pos = max_angle;
    else
      next_pos = min_angle;
    if(debug_output_)
      ROS_INFO ("Init sweeping to: %f (dist to min: %f, dist to max:  %f) \n", 
                next_pos, fabs (current_pos - min_angle), fabs (current_pos - max_angle));
    
    playerc_actarray_position_cmd (device, actuator, DEG2RAD (next_pos));
    // Loop until the actuator is idle-ing or break-ed.
    do {
      playerc_client_read (client);
      //print_data (device);
      current_pos = RAD2DEG (device->actuators_data[actuator].position);
    }  
    while ( (fabs (current_pos - next_pos) > EPS) || (
                                                      (device->actuators_data[actuator].state != PLAYER_ACTARRAY_ACTSTATE_IDLE) &&
                                                      (device->actuators_data[actuator].state != PLAYER_ACTARRAY_ACTSTATE_BRAKED))
            );
    
    if (next_pos == max_angle)
      next_pos = min_angle;
    else
      next_pos = max_angle;
    
    return next_pos;
  }

  //////////////////////////////////////////////////////////////////////////////////
  // //print data to stdio
  //////////////////////////////////////////////////////////////////////////////////
  void
  print_data (playerc_actarray_t *device)
  {
    int i;
    for (i = 0; i < device->actuators_count; i++)
      {
        printf ("X%d> pos, speed, accel, cur, state  : [%f, %f, %f, %f, %d]\n", (i+1), 
                RAD2DEG (device->actuators_data[i].position), 
                device->actuators_data[i].speed, 
                device->actuators_data[i].acceleration, 
                device->actuators_data[i].current, 
                device->actuators_data[i].state);
      }
  }

  //////////////////////////////////////////////////////////////////////////////////
  // //update parameters on the fly
  //////////////////////////////////////////////////////////////////////////////////
  void
  update_parameters_from_server()
  {
    if (nh_.hasParam("~rot_speed"))
      nh_.getParam("~rot_speed", rot_speed_);
    if (nh_.hasParam("~rot_joint_"))
      nh_.getParam("~rot_joint_", rot_joint_);
  }
    
  //class Amtec_Sweep_service ends here  
};

  

int main(int argc, char **argv)
{
  //ROS
  ros::init(argc, argv, "amtec_sweep_service_server");
  Amtec_Sweep_service p;    
  ros::spin();
  return 0;
}

