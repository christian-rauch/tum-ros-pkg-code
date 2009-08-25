#include <ncurses.h>
#include <playerc.h>
#include <string>
#include <math.h>
#include <stdio.h>
#include <cstdlib>
#include <getopt.h>
#include <iostream>
using namespace std;
//#define M_PI        3.14159265358979323846
// Convert radians to degrees
#define RAD2DEG(r) (float)((r) * 180 / M_PI)
// Convert degrees to radians
#define DEG2RAD(d) (float)((d) * M_PI / 180)

#define HOSTNAME "localhost"

// what diference between angles is considered 0?
#define EPS 0.01

void home (playerc_client_t *client, playerc_actarray_t *device, int joint);
void homeall (playerc_client_t *client, playerc_actarray_t *device);
void sweep (playerc_client_t *client, playerc_actarray_t *device, 
	    int actuator, float next_pos,  float speed);
float go_to_pose (playerc_client_t *client, playerc_actarray_t *device, 
		  int actuator, float min_angle, float max_angle);
void rotate (playerc_client_t *client, playerc_actarray_t *device, 
	   int actuator, float angle, float speed);
void print_data (playerc_actarray_t *device);
void laser_power_on_off(playerc_client_t *client, playerc_laser_t *device_laser, bool sw);
void usage_ ();
//public struct for command line arguments
static struct
{
  char *program;
  std::string log_filename;	
  float start_angle;
  float end_angle;
  int interface;
  int rot_joint;
  //in rad/s
  float rot_speed;
  int loop;   
} g_options;


int main (int argc, char  **argv)
{

  //////////////////////////////////////////////////////////////////////////////////
  // Parse options
  //////////////////////////////////////////////////////////////////////////////////
  g_options.program = argv[0];
 //set options to some impossible values
 g_options.log_filename = " ";
 g_options.start_angle = -1000;
 g_options.end_angle = -1000;
 g_options.interface = -1000;
 g_options.rot_joint = -1000;
 g_options.rot_speed = -1000;
 g_options.loop = 0;
 //use custom given log filename
 bool log_filename_from_time = false;

 while (1)
   {
     //for some reason specifying "optional_argument" results in empty valued option, 
     //thus all of them come with "required_argument" despite being optional  
     static struct option long_options[] = {
       {"help", no_argument, 0, 'h'},
       {"start_angle", required_argument, 0, 's'},
       {"end_angle", required_argument, 0, 'e'},
       {"interface", required_argument, 0, 'i'},
       {"rot_joint", required_argument, 0, 'j'},
       {"rot_speed", required_argument, 0, 'v'},
       {"log_filename", required_argument, 0, 'f'},
       {"loop", required_argument, 0, 'l'},
     };
     int option_index = 0;
     int c = getopt_long(argc, argv, "s:e:i:j:v:f:hl:", long_options, &option_index);
     if (c == -1) 
       {
	 //usage_();
	 break;
       }
     switch (c)
       {
       case 'h':
	 usage_();
	 break;
       case 's':
	 g_options.start_angle = atof(optarg);
	 break;
       case 'e':
	 g_options.end_angle = atof(optarg);
	 break;
       case 'i':
	 g_options.interface = atoi(optarg);
	 break;
       case 'j':
	 g_options.rot_joint = atoi(optarg);
	 break;
       case 'v':
	 g_options.rot_speed = atof(optarg);
	 break;
       case 'l':
	 g_options.loop = atoi(optarg);
	 break;
       case 'f':
	  g_options.log_filename.assign(optarg);
	  cerr << "log: " << g_options.log_filename << endl;
	  break;
       }
   }

 if (g_options.interface == -1000)
   {
     cerr << "Error: You have to specify interface (-i)" << endl;
     exit(0);
   }
 if (g_options.start_angle  == -1000)
   {
     cerr << "Error: You have to specify start_angle (-s)" << endl;
     exit(0);
   }
 if (g_options.end_angle  == -1000)
   {
     cerr << "Error: You have to specify end_angle (-e)" << endl;
     exit(0);
   }
 if (g_options.rot_joint == -1000)
   {
     cerr << "Error: You have to specify rotational joint (-j)" << endl;
     exit(0);
   }
 if (g_options.rot_speed == -1000)
   {
   cerr << "Warning: Setting rot speed to 0.1 rad/s" << endl;
   g_options.rot_speed = 0.05;
   }
 if(g_options.log_filename == "")
   {
     cerr << "Warning: Using auto-generated log filenames!" << endl; 
     log_filename_from_time = true;
   }


 //////////////////////////////////////////////////////////////////////////////////
 // main begins
 //////////////////////////////////////////////////////////////////////////////////
 // exit(0);

 // Set the current actuator as the gripper (default)
  int delta_rot = 10;
  
  char logFileName[80];
  struct timeval t1;
  playerc_client_t *client;
  int t, i, j;
  void *rdevice;
  int port = 6665;
  playerc_actarray_t *device_actarray;
  playerc_laser_t *device_laser;
  playerc_log_t *device_log;
  client = playerc_client_create (NULL, HOSTNAME, port);
  playerc_client_connect (client);

  //return from go_to_pose
  float next_pos;
  
  device_actarray  = playerc_actarray_create (client, g_options.interface);
  device_laser  = playerc_laser_create (client, 0);
  device_log       = playerc_log_create      (client, 0);

  playerc_actarray_subscribe (device_actarray, PLAYER_OPEN_MODE);
  playerc_log_subscribe      (device_log, PLAYER_OPEN_MODE);
  playerc_laser_subscribe      (device_laser, PLAYER_OPEN_MODE);

  playerc_log_set_filename (device_log, g_options.log_filename.c_str());
  initscr ();
  refresh ();
  cbreak ();
  
  for (t = 0; t < 20; t++)
    rdevice = playerc_client_read (client);

  int current_actuator = device_actarray->actuators_count - 1;
  
  nodelay (stdscr, TRUE);
  keypad (stdscr, TRUE);
  
  printw ("Legend: 'p' - pause logging, 'r' - start/resume logging into from time generated log files, 'h' - home all cubes\n");
  printw ("        'b' - start/resume logging into cli passed log files\n");
  printw ("        's' - sweep a cube (selected using '1'-'%d')\n", device_actarray->actuators_count - 1);
  printw ("        'o' - home one cube, 'x' - exit\n"); refresh ();
  
  mvprintw (20, 0, "Logging : disabled\n"); refresh ();
  
  int key;
 
  if(g_options.loop == 1)
    {
      cerr << "loop   " << g_options.loop << endl;
      //exit(0);
      while (key != 'x')
	{
	  laser_power_on_off(client, device_laser, false);
	  rdevice = playerc_client_read (client);
	  print_data (device_actarray);
	  key = getch ();
	  mvprintw (4, 0, "Keystroke: %d\n", key); refresh ();
	  next_pos = go_to_pose(client, device_actarray, g_options.rot_joint, g_options.start_angle, g_options.end_angle);
	  sweep (client, device_actarray, g_options.rot_joint, next_pos, g_options.rot_speed);
	  if (next_pos == g_options.end_angle) 
	    laser_power_on_off(client, device_laser, true);
	  while (fabs(next_pos  -   RAD2DEG (device_actarray->actuators_data[g_options.rot_joint].position)) > EPS)
	    {
	      playerc_client_read (client);
	      mvprintw (22, 0,  "Angle  %f\n", fabs(next_pos  -   RAD2DEG (device_actarray->actuators_data[g_options.rot_joint].position))); refresh();
	      //sleep(1);
	    }
	}
    }
 
  while (key != 'x')
  {
    rdevice = playerc_client_read (client);

    print_data (device_actarray);
    key = getch ();
    mvprintw (4, 0, "Keystroke: %d\n", key); refresh ();
     
    switch (key)
      {
      case '1':
	current_actuator = 0;
	break;
      case '2':
	current_actuator = 1;
	break;
      case '3':
	current_actuator = 2;
	break;
      case '4':
	current_actuator = 3;
	break;
      case '5':
	current_actuator = 4;
	break;
      case '6':
	current_actuator = 5;
	break;
      case '7':
	current_actuator = 6;
	break;
	
	// Left arrow
      case 260:
	delta_rot--;
	mvprintw (22, 0, "Delta : %d\n", delta_rot); refresh ();
	break;
	
	// Right arrow
      case 261:
	delta_rot++;
	mvprintw (22, 0, "Delta : %d\n", delta_rot); refresh ();
	break;
	
      case '-':
	// move - 10
	// rotate (client, device_actarray, current_actuator, -delta_rot, 0.1);
	mvprintw (21, 0, "Moving - with %d\n", delta_rot); refresh ();
	break;
      case '+':
	// move + 10
	//rotate (client, device_actarray, current_actuator, delta_rot, 0.1);
	mvprintw (21, 0, "Moving + with %d\n", delta_rot); refresh ();
	break;
	
      case 'p':
	// pause logging
	playerc_log_set_write_state (device_log, 0);
	mvprintw (20, 0, "Logging : disabled\n"); refresh ();
	break;
      case 'r':
	// resume logging
	if(log_filename_from_time)
	  {
	    gettimeofday(&t1, NULL);
	    sprintf (logFileName, "%d.log", t1.tv_sec);
	    playerc_log_set_filename (device_log, logFileName);
	    playerc_log_set_write_state (device_log, 1);
	    mvprintw (20, 0, "Logging : enabled\n"); refresh ();
	  }
	else
	  {
	      mvprintw(20, 0, "This option not possible, use __\"b\"__ key to resume logging\n"); refresh();
	  }
	break;
      case 'b':
        // resume logging, override custom log file
	// from g_options.log_filename;
	if(!log_filename_from_time)
	  {
	    //playerc_log_set_filename (device_log, g_options.log_filename.c_str());
	    playerc_log_set_write_state (device_log, 1);
	    mvprintw (20, 0, "Logging : enabled\n"); refresh ();
	  }
	else
	  {
	    mvprintw(20, 0, "This option not possible, use __\"r\"__ key to resume logging\n"); refresh();
	  }
	break;
      case 'h':
	// home
	homeall (client, device_actarray);
	break;
      case 'o':
	// home
	home (client, device_actarray, current_actuator);
	break;
      case 's':
	next_pos = go_to_pose(client, device_actarray, g_options.rot_joint, g_options.start_angle, g_options.end_angle);
	sweep (client, device_actarray, g_options.rot_joint, next_pos, g_options.rot_speed);
	break;
      }
    mvprintw (23, 0, "Current actuator: %d\n", (current_actuator + 1)); refresh ();
  }
 
  printw ("unsubscribing\n");refresh ();
  playerc_actarray_unsubscribe (device_actarray);
  playerc_actarray_destroy (device_actarray);
  
  playerc_log_unsubscribe (device_log);
  playerc_log_destroy (device_log);

  playerc_laser_unsubscribe (device_laser);
  playerc_laser_destroy (device_laser);
  
  endwin ();
  return 0;
}

// ---[ Home one cube
void
    home (playerc_client_t *client, playerc_actarray_t *device, int joint)
{
    int i;

    playerc_actarray_home_cmd (device, joint);
    do {
        playerc_client_read (client);
        print_data (device);
    }	
    while ((device->actuators_data[joint].state != PLAYER_ACTARRAY_ACTSTATE_IDLE) &&
           (device->actuators_data[joint].state != PLAYER_ACTARRAY_ACTSTATE_BRAKED));
}

// ---[ Home all cubes
void
    homeall (playerc_client_t *client, playerc_actarray_t *device)
{
    int i;
    playerc_actarray_power (device, 1);
    
    for (i = 6; i > -1; i--)
    {
	playerc_actarray_home_cmd (device, i);
	mvprintw (21, 0, "Homing %d\n", i); refresh ();
	do {
	    playerc_client_read (client);
	    print_data (device);
	}	
        while ((device->actuators_data[i].state != PLAYER_ACTARRAY_ACTSTATE_IDLE) &&
	       (device->actuators_data[i].state != PLAYER_ACTARRAY_ACTSTATE_BRAKED));
    }
}

// ---[ Sweep a cube
void
    sweep (playerc_client_t *client, playerc_actarray_t *device, 
	    int actuator, float next_pos, float speed)
{
  // Set the speed to move with
  playerc_actarray_speed_config (device, actuator, speed);
  
  mvprintw (21, 0, "Sweeping. Next position is %f \n", next_pos); refresh ();
  playerc_actarray_position_cmd (device, actuator, DEG2RAD (next_pos));

}

//find the start_angle
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
  
  mvprintw (21, 0, "Init sweeping to: %f (dist to min: %f, dist to max:  %f) \n", next_pos, fabs (current_pos - min_angle), fabs (current_pos - max_angle)); refresh ();
  
  playerc_actarray_position_cmd (device, actuator, DEG2RAD (next_pos));
  // Loop until the actuator is idle-ing or break-ed.
  do {
    playerc_client_read (client);
    print_data (device);
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

// ---[ Move a cube
void
    rotate (playerc_client_t *client, playerc_actarray_t *device, 
	    int actuator, float angle, float speed)
{
    float next_pos;

    // Set the speed to move with
    //playerc_actarray_speed_config (device, actuator, speed);
    
    // Find the current position
    float current_pos = RAD2DEG (device->actuators_data[actuator].position);
    next_pos = current_pos + angle;
    
    playerc_actarray_position_cmd (device, actuator, DEG2RAD (next_pos));
    // Loop until the actuator is idle-ing or break-ed.
    do {
        playerc_client_read (client);
	print_data (device);
    }
    while ((device->actuators_data[actuator].state != PLAYER_ACTARRAY_ACTSTATE_IDLE) &&
           (device->actuators_data[actuator].state != PLAYER_ACTARRAY_ACTSTATE_BRAKED));
}

//print data to stdio
void
    print_data (playerc_actarray_t *device)
{
    int i;
    for (i = 0; i < device->actuators_count; i++)
    {
      mvprintw (10+i, 0, "X%d> pos, speed, accel, cur, state  : [%f, %f, %f, %f, %d]\n", (i+1), 
	    RAD2DEG (device->actuators_data[i].position), 
	    device->actuators_data[i].speed, 
	    device->actuators_data[i].acceleration, 
	    device->actuators_data[i].current, 
	    device->actuators_data[i].state); refresh ();
    }
}

void 
laser_power_on_off(playerc_client_t *client, playerc_laser_t *device_laser, bool sw)
{
  //turn laser power on and off
  player_laser_power_config_t power_rq;
  power_rq.state = sw;
  void * dummy; 
  playerc_client_request(client, &device_laser->info, PLAYER_LASER_REQ_POWER,
			 reinterpret_cast <void *>(&power_rq), 
			 NULL);
 }

//print CLI usage
void usage_ ()
 {
   fprintf(stderr, "Usage: %s [options]\n", g_options.program);
   fprintf(stderr, "  Available options\n");
   fprintf(stderr, "    -i, --interface <0-right|1-left>\n");
   fprintf(stderr, "    -s, --start_angle\n");
   fprintf(stderr, "    -e, --end_angle\n");
   fprintf(stderr, "    -j, --rot_joint <0-5>\n");
   fprintf(stderr, "    -v, --rot_speed <rad/s>\n");
   fprintf(stderr, "    -f, --log_filename \n");
   fprintf(stderr, "    -h, --help Print this message and exit\n");
   exit(0);
 }
