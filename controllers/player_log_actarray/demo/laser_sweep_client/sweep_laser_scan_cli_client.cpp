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
  double angle_filename_;
  double start_angle_;
  double end_angle_;
  int interface_;
  int rot_joint_;
  double rot_speed_;
  ///////////////////////////////////////////////////////////////////////////////////////
  //Contructor
  ///////////////////////////////////////////////////////////////////////////////////////
  Amtec_Sweep_service()
  {
    nh_.param("~start_angle", start_angle_, 0.0);
    nh_.param("~end_angle", end_angle_, 10.0);
    nh_.param("~interface", interface_, 1);
    nh_.param("~rot_joint", rot_joint_, 4);
    nh_.param("~rot_speed", rot_speed_, 0.1);
    service_ = nh_.advertiseService("amtec_sweep", &Amtec_Sweep_service::amtec_sweep_main, this);	
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
    ROS_INFO("in amtec_sweep_main");
    ROS_INFO("start_angle %lf", start_angle_);
    ROS_INFO("end_angle %lf", end_angle_);
    angle_filename_ = req.angle_filename;
    object_ = req.object;
    
    //  if (g_options.interface == -1000)
    //    {
    //      cerr << "Error: You have to specify interface (-i)" << endl;
    //      exit(0);
    //    }
    //  if (g_options.start_angle  == -1000)
    //    {
    //      cerr << "Error: You have to specify start_angle (-s)" << endl;
    //      exit(0);
    //    }
    //  if (g_options.end_angle  == -1000)
    //    {
    //      cerr << "Error: You have to specify end_angle (-e)" << endl;
    //      exit(0);
    //    }
    //  if (g_options.rot_joint == -1000)
    //    {
    //      cerr << "Error: You have to specify rotational joint (-j)" << endl;
    //      exit(0);
    //    }
    //   if (g_options.angle_filename == -1000)
    //    {
    //      cerr << "Error: You have to specify ptu angle (-a)" << endl;
    //      exit(0);
    //    }
    //  if (g_options.rot_speed == -1000)
    //    {
    //    cerr << "Warning: Setting rot speed to 0.1 rad/s" << endl;
    //    g_options.rot_speed = 0.05;
    //    }
    //  if(g_options.log_filename == "")
    //    {
    //      cerr << "Warning: Using auto-generated log filenames!" << endl; 
    //      log_filename_from_time = true;
    //    }
    
    int delta_rot = 10;
    char logFileName[80];
    struct timeval t1;
    playerc_client_t *client;
    int t, i, j;
    void *rdevice;
    int port = 6665;
    playerc_actarray_t *device_actarray;
    playerc_laser_t *device_laser;
    ////playerc_log_t *device_log;
    client = playerc_client_create (NULL, HOSTNAME, port);
    playerc_client_connect (client);
    
    //return from go_to_pose
    float next_pos;
    
    device_actarray  = playerc_actarray_create (client, interface_);
    device_laser  = playerc_laser_create (client, 0);
    ////device_log       = playerc_log_create      (client, 0);
    
    playerc_actarray_subscribe (device_actarray, PLAYER_OPEN_MODE);
    ////playerc_log_subscribe      (device_log, PLAYER_OPEN_MODE);
    playerc_laser_subscribe      (device_laser, PLAYER_OPEN_MODE);
    
    ////playerc_log_set_filename (device_log, g_options.log_filename.c_str());
    
    for (t = 0; t < 20; t++)
      rdevice = playerc_client_read (client);
    
    int current_actuator = device_actarray->actuators_count - 1;
    
    rdevice = playerc_client_read (client);
    //print_data (device_actarray);
    
    //enable logging
    ////gettimeofday(&t1, NULL);
    ////sprintf (logFileName, "%d.log", t1.tv_sec);
    ////playerc_log_set_filename (device_log, logFileName);
    ////playerc_log_set_write_state (device_log, 1);
    ////printf ("Logging : enabled\n"); 
    
    //sweeping
    next_pos = go_to_pose(client, device_actarray, rot_joint_, start_angle_, end_angle_);
    ROS_INFO("Sweeping to next_pos: %f", next_pos);
    sweep (client, device_actarray, rot_joint_, next_pos, rot_speed_);
    //printf ("Sweeping...........\n"); 
    //wait until end angle reached
    while (fabs(next_pos  -   RAD2DEG (device_actarray->actuators_data[rot_joint_].position)) > EPS)
      {
	playerc_client_read (client);
	//printf ("Angle  %f\n", fabs(next_pos  -   RAD2DEG (device_actarray->actuators_data[rot_joint_].position)));
      }
    
    //logging disabled
    ////playerc_log_set_write_state (device_log, 0);
    ////printf ("Logging : disabled\n"); 
    
    //clean up player
    printf ("unsubscribing\n");
    playerc_actarray_unsubscribe (device_actarray);
    playerc_actarray_destroy (device_actarray);
    
    ////playerc_log_unsubscribe (device_log);
    ////playerc_log_destroy (device_log);
    
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
    
    printf ("Sweeping. Next position is %f \n", next_pos);
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
    int key;
    
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
    
    printf ("Init sweeping to: %f (dist to min: %f, dist to max:  %f) \n", 
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

