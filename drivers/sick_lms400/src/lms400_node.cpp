/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id$
 *
 */


/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b lms400_node is a node providing access to a SICK LMS400 laser sensor via the sick_lms400 library.
The code is based on the LGPL-ed LMS400 Player driver.
**/

/**- 
- filter (integer)
  - Default: 0 (disabled)
  - Filter settings. Valid values are:
    - 0 (disabled)
    - 1 (enable median filter)
    - 2 (enable edge filter)
    - 4 (enable range filter)
    - 8 (enable mean filter)
  - Notes :
    - 1) You can combine the filters as required. If several filters are
        active, then the filters act one after the other on the result of the
        previous filter. The processing in this case follows the following
        sequence: edge filter, median filter, range filter, mean filter.
    - 2) You can use PLAYER_LASER_REQ_SET_FILTER to enable/disable the filters
        as well as set their parameters from the client. The parameters of the
        filters are stored in a float array in the sequence mentioned above.
        Since the current LMS400 firmware version (1.20) supports setting the
        parameters for the range and mean filters, the order of the parameters
        in the (player_laser_set_filter_config_t - float parameters[]) array,
        provided that both range and mean are enabled is:
        [BottomLimit TopLimit Mean]

- range_filter_parameters (float tuple)
  - Default: [700.0 3000.0] (BottomLimit TopLimit)
  - Define a specific range within which measured values are valid and are
    output. Possible values: [+700.0...+3000.0 <bottom limit>...+3000.0]

- mean_filter_parameter (integer)
  - Default: 2.
  - Define the number of means for the mean filter. Possible values: 2..200.

- angular_resolution (float)
  - Default: 0.25 degrees
  - Angular resolution. Valid values are: 0.1 ..1 (with 0.1 degree increments)

- scanning_frequency (float)
  - Default: 500 Hz.
  - Scanning frequency. Valid values are:
    - 200..500Hz (on the LMS400-0000)
    - 360..500Hz (on the LMS400-1000)

- min_angle (float)
  - Default: 55 degrees.
  - Defines the minimum angle of the laser unit (where the scan should start).
    Valid values: 55-124 degrees.

- max_angle (float)
  - Default: 125 degrees.
  - Defines the maximum angle of the laser unit (where the scan should end).
    Valid values: [min_angle]-125 degrees.

- pose (length tuple)
  - Default: [0.0 0.0 0.0]
  - Pose (x,y,theta) of the laser, relative to its parent object (e.g.,
    the robot to which the laser is attached).

- size (length tuple)
  - Default: [0.15 0.15]
  - Footprint (x,y) of the laser.

- enable_eRIS (integer)
  - Default: 1
  - Enable extended RIS detectivity. If you want to measure objects with
    remission values < 10%, you can extend the so-called Remission
    Information System (RIS) on the LMS4000.

- password (string)
  - Default: servicelevel/81BE23AA
  - Service (userlevel 3) password. Used for enabling/disabling and/or setting
    the filter parameters.

- debug (int)
  - Default: 0
  - Enable debugging mode (read/writes to the device are printed on screen).
    Valid values: 0 (disabled), 1 (enabled for standard messages), 2 (enabled
    for all messages including measurements - warning: this slows down your
    throughoutput date!).

@par Example

@verbatim
driver
(
  name "sicklms400"
  provides ["laser:0"]

  # Enable median, range and mean filter
  filter 13
  # Set the range filter parameters to 800...2000mm
  range_filter_parameters [800.0 2900.0]
  # Set the mean filter parameter
  mean_filter_parameter 3

  # Set the angular resolution (0.1 degrees) and scanning frequency (360Hz)
  angular_resolution 10
  scanning_frequency 360

)
*/

#include <sick_lms400/sick_lms400.h>
#include <ros/ros.h>
#include <math.h>

#define DEFAULT_LMS400_L3_PASSWORD "NULL"
#define DEFAULT_LMS400_FREQUENCY    360
#define DEFAULT_LMS400_ANGULAR_RES  0.25
#define DEFAULT_LMS400_MINANGLE     55.0
#define DEFAULT_LMS400_MAXANGLE     125.0

/*// The SICK LMS 400 laser device class.
////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
void
SickLMS400::MainQuit ()
{
 
  // Stop the measurement process
  lms400->StopMeasurement ();
  
  // Set back to userlevel 0 
  lms400->TerminateConfiguration ();
  
  // Disconnect from the laser unit
  lms400->Disconnect ();
  
  PLAYER_MSG0 (1, "> SICK LMS400 driver shutting down... [done]");
}

////////////////////////////////////////////////////////////////////////////////
// ProcessMessage
int
  SickLMS400::ProcessMessage (QueuePointer &resp_queue,
                              player_msghdr* hdr,
                              void* data)
{
  assert (hdr);

  // ---[ Get geometry
  if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_GET_GEOM, device_addr))
  {
    player_laser_geom_t geom;
    memset(&geom, 0, sizeof(geom));
    geom.pose.px = pose[0];
    geom.pose.py = pose[1];
    geom.pose.pyaw = pose[2];
    geom.size.sl = size[0];
    geom.size.sw = size[1];

    Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
             PLAYER_LASER_REQ_GET_GEOM, (void*)&geom, sizeof(geom), NULL);
    return (0);
  }
  // --]

  // ---[ Set power
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_POWER, device_addr))
  {
    player_laser_power_config_t* config =
      reinterpret_cast<player_laser_power_config_t *> (data);

    if (config->state == 0)
    {
      lms400->StopMeasurement ();
      laser_enabled = false;
    }
    else
    {
      lms400->StartMeasurement (intensity);
      laser_enabled = true;
    }

    Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
             hdr->subtype);
    return (0);
  }
  // --]

  // ---[ Set configuration
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_SET_CONFIG, device_addr))
  {
    player_laser_config_t* config =
      reinterpret_cast<player_laser_config_t *> (data);

    // Set intensity locally, ignore range_max and range_res (unused for LMS400)
    intensity = config->intensity;
    // Setting {min, max}_angle, resolution (angular), and scanning_frequency
    angular_resolution = RTOD (config->resolution);
    min_angle  = RTOD (config->min_angle);
    max_angle  = RTOD (config->max_angle);
    scanning_frequency = config->scanning_frequency;

    // Stop the measurement process
    lms400->StopMeasurement ();

    // Change userlevel to 3
    if (lms400->SetUserLevel (4, password) != 0)
      PLAYER_WARN1 ("> Unable to change userlevel to 'Service' using %s", password);
    else
      // Set the angular resolution and frequency
      if (lms400->SetResolutionAndFrequency (
           scanning_frequency,
 	   angular_resolution,
	   min_angle,
	   max_angle - min_angle) == 0)
      {
        // Re-start the measurement process
        if (laser_enabled) lms400->StartMeasurement (intensity);

        // Configuration succeeded; send an ACK
        Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
                 hdr->subtype);
        return (0);
      }

    // Re-start the measurement process
    if (laser_enabled) lms400->StartMeasurement (intensity);

    // Configuration failed; send a NACK
    Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK,
             hdr->subtype);
    return (-1);
  }

  // ---[ Get configuration
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_GET_CONFIG, device_addr))
  {
    // Get min_angle, max_angle, resolution and scanning_frequency
    player_laser_config_t config = lms400->GetConfiguration ();
    config.max_range = 3; // maximum measurable distance
    config.range_res = 1; // bogus value
    config.intensity = intensity;

    Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
             PLAYER_LASER_REQ_GET_CONFIG, (void*)&config, sizeof (config), NULL);
    return (0);
  }

  // ---[ Get IDentification information
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_GET_ID, device_addr))
  {
    char *macaddress = new char[40];
    char *hexaddress = new char[13];

    // Stop the measurement process
    lms400->StopMeasurement ();
    // Get the MAC address
    if (lms400->GetMACAddress (&macaddress) != 0)
    {
      // Re-start the measurement process
      if (laser_enabled) lms400->StartMeasurement (intensity);
      // Configuration failed; send a NACK
      Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK,
               hdr->subtype);
      return (-1);
    }
    // Re-start the measurement process
    if (laser_enabled) lms400->StartMeasurement (intensity);

    // Tokenize: remove the ":"s and convert to integer
    char *token = strtok (macaddress, ":");
    int i = 0;
    while (1)
    {
      strcpy (hexaddress + i, token);
      i += 2;
      token = strtok (NULL, ":");
      if (token == NULL)
        break;
    }

    char lasthexaddress[7];
    memcpy (lasthexaddress, hexaddress+6, 6);
    lasthexaddress[6] = 0;

    player_laser_get_id_config_t idconfig;
    idconfig.serial_number = strtol (lasthexaddress, NULL, 16);

    delete macaddress;
    delete hexaddress;

    Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
             PLAYER_LASER_REQ_GET_ID, (void*)&idconfig, sizeof (idconfig), NULL);
    return (0);
  }
  // --]

  // ---[ Set filter settings
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_SET_FILTER, device_addr))
  {
    if (!loggedin)
    {
      PLAYER_WARN ("> Userlevel 3 password not given or invalid. Filter(s) disabled!");
      // Configuration failed; send a NACK
      Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK,
               hdr->subtype);
      return (-1);
    }

    player_laser_set_filter_config_t* config =
      reinterpret_cast<player_laser_set_filter_config_t *> (data);

    // Stop the measurement process
    lms400->StopMeasurement ();
    // Check filter parameter settings
    switch (config->parameters_count)
    {
      case 3:		// Presume both range and mean filter are enabled
      {
        // Check the range filter parameters
        float r_params[2];
        r_params[0] = config->parameters[0];
        r_params[1] = config->parameters[1];

        if ((r_params[0] >= 700) &&
            (r_params[1] > r_params[0]) &&
            (r_params[1] <= 3000))
        lms400->SetRangeFilterParameters (r_params);

        if (mean_filter_params != 0)
          lms400->SetMeanFilterParameters (mean_filter_params);

        // Check the mean filter parameters
        int m_params = static_cast<int> (round (config->parameters[2]));
        if ((m_params >= 2) && (m_params <= 200))
          lms400->SetMeanFilterParameters (m_params);

        break;
      }
      case 2:		// Presume only the range filter is enabled
      {
        // Check the range filter parameters
        float r_params[2];
        r_params[0] = config->parameters[0];
        r_params[1] = config->parameters[1];

        if ((r_params[0] >= 700) &&
            (r_params[1] > r_params[0]) &&
            (r_params[1] <= 3000))
        lms400->SetRangeFilterParameters (r_params);

        if (mean_filter_params != 0)
          lms400->SetMeanFilterParameters (mean_filter_params);

        break;
      }
      case 1:		// Presume only the mean filter is enabled
      {
        // Check the mean filter parameters
        int m_params = static_cast<int> (round (config->parameters[0]));
        if ((m_params >= 2) && (m_params <= 200))
          lms400->SetMeanFilterParameters (m_params);
        break;
      }
      case 0:
        break;
      default:
      {
        PLAYER_WARN ("> LMS400 doesn't support more than 3 filter parameters.");
        break;
      }
    }

    if (lms400->EnableFilters (config->filter_type) > 0)
    {
      // Re-start the measurement process
      if (laser_enabled) lms400->StartMeasurement (intensity);
      // Configuration succeeded; send an ACK
      Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
               hdr->subtype);
      return (0);
    }
    else
    {
      // Re-start the measurement process
      if (laser_enabled) lms400->StartMeasurement (intensity);
      // Configuration failed; send a NACK
      Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK,
               hdr->subtype);
      return (-1);
    }
  }
  else
    return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void
  SickLMS400::Main ()
{
  // Start Continous measurements
  lms400->StartMeasurement (intensity);

  while (true)
  {
    // test if we are supposed to cancel
    pthread_testcancel ();

    // Request/replies handler
    ProcessMessages ();

    // Refresh data only if laser power is on
    if (laser_enabled)
    {
      player_laser_data_t data = lms400->ReadMeasurement ();

      // Make data available
      if (data.ranges_count != (unsigned int)-1)
        Publish (device_addr, PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCAN, &data);
      player_laser_data_t_cleanup(&data);
    }
  }
}*/

using namespace std;
using namespace sick_lms400;

class LMS400Node
{
  protected:
    ros::NodeHandle nh_;
  public:

    SickLMS400 lms_;

    // Laser pose in robot cs.
    double pose[3];
    double size[2];

    // TCP/IP connection parameters
    string hostname_, password_;
    int         port_number_;

    // Filter settings
    int filter_;
    int mean_filter_params_;
    float range_filter_params_[2];

    // Turn intensity data on/off
    bool intensity_;

    // Turn laser on/off
    bool laser_enabled_;

    // Basic measurement parameters
    double angular_resolution_, scanning_frequency_;
    double min_angle_, max_angle_;
    int eRIS_;

    // Password for changing to userlevel 3 (service)
    bool loggedin_;
    int debug_;

    ////////////////////////////////////////////////////////////////////////////////
    LMS400Node () : debug_ (0)
    {
      nh_.param ("~hostname", hostname_, "192.168.0.1");
      nh_.param ("~password", password_, "81BE23AA"); // Userlevel 3 password (hashed). Default: servicelevel/81BE23AA
      nh_.param ("~port", port_, 2111);

      nh_.param ("~leaf_width_x", pose_.x, 0.0);
      nh_.param ("~leaf_width_y", pose_.y, 0.0);
      nh_.param ("~leaf_width_z", pose_.z, 0.0);




/*
  // Laser geometry.
  pose[0] = cf->ReadTupleLength(section, "pose", 0, 0.0);
  pose[1] = cf->ReadTupleLength(section, "pose", 1, 0.0);;
  pose[2] = cf->ReadTupleLength(section, "pose", 2, 0.0);;
  size[0] = 0.15;
  size[1] = 0.15;

  intensity = true;

  // Read TCP/IP connection settings
  port_number = cf->ReadInt (section, "port", DEFAULT_LMS400_PORT);

  // Read filter settings
  filter              = cf->ReadInt (section, "filter", 0);
  mean_filter_params  = cf->ReadInt (section, "mean_filter_parameter", 0);

  range_filter_params[0] =
    cf->ReadTupleFloat (section, "range_filter_parameters", 0, 0);
  range_filter_params[1] =
    cf->ReadTupleFloat (section, "range_filter_parameters", 1, 0);

  // Basic measurement parameters (defaults to 1 degree and 500Hz)
  angular_resolution =
    cf->ReadFloat (section, "angular_resolution", DEFAULT_LMS400_ANGULAR_RES);
  scanning_frequency =
    cf->ReadFloat (section, "scanning_frequency", DEFAULT_LMS400_FREQUENCY);

  min_angle = cf->ReadFloat (section, "min_angle", DEFAULT_LMS400_MINANGLE);
  max_angle = cf->ReadFloat (section, "max_angle", DEFAULT_LMS400_MAXANGLE);

  eRIS  = cf->ReadInt (section, "enable_eRIS", 1);

  // Password for changing to userlevel 3 (service)
  password = cf->ReadString (section, "password", DEFAULT_LMS400_L3_PASSWORD);
  loggedin = false;

  laser_enabled = true;

  debug = cf->ReadInt (section, "debug", 0);*/      
    }



    ////////////////////////////////////////////////////////////////////////////////
    // Start
    int
      start ()
    {
      // Open the LMS400 device
      lms_ = SickLMS400 (hostname_, port_number_, debug_);

      // Attempt to connect to the laser unit
      if (lms_.Connect () != 0)
      {
        ROS_ERROR ("> [SickLMS400] Connecting to SICK LMS400 on [%s:%d]...[failed!]", hostname_, port_number_);
        return (-1);
      }
      ROS_INFO ("> [SickLMS400] Connecting to SICK LMS400 on [%s:%d]... [done]", hostname_, port_number_);

      // Stop the measurement process (in case it's running from another instance)
      lms_.StopMeasurement ();

      if (strncmp (password_, "NULL", 4) != 0)
      {
        // Login to userlevel 3
        if (lms_.SetUserLevel (4, password_) != 0)
          ROS_WARN ("> [SickLMS400] Unable to change userlevel to 'Service' using %s", password);
        else
        {
          loggedin_ = true;
          // Enable or disable filters
          if ((mean_filter_params >= 2) && (mean_filter_params <= 200))
            lms_.SetMeanFilterParameters (mean_filter_params);

          if ((range_filter_params[0] >= 700) && (range_filter_params[1] > range_filter_params[0]) && (range_filter_params[1] <= 3600))
            lms_.SetRangeFilterParameters (range_filter_params);

          lms_.EnableFilters (filter);

          ROS_INFO ("> [SickLMS400] Enabling selected filters... [done]");
        }
      }
      else
        ROS_WARN ("> [SickLMS400] Userlevel 3 password not given. Filter(s) disabled!");

      // Enable extended RIS detectivity
      if (eRIS_)
      {
        lms_.EnableRIS (1);
        ROS_INFO ("> [SickLMS400] Enabling extended RIS detectivity... [done]");
      }

      // Set scanning parameters
      if (lms_.SetResolutionAndFrequency (scanning_frequency_, angular_resolution_, min_angle_, max_angle_ - min_angle_) != 0)
        ROS_ERROR ("> [SickLMS400] Couldn't set values for resolution, frequency, and min/max angle. Using previously set values.");
      else
        ROS_INFO ("> [SickLMS400] Enabling user values for resolution, frequency and min/max angle... [done]");

      return (0);
    }


    ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
      
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "lms400_node");

  LMS400Node c;

  if (c.start () == 0)
    c.spin ();

  return (0);
}
/* ]--- */

