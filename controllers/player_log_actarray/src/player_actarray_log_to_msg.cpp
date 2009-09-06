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

@b player_actarray_log_to_msg converts Player (http://player.sf.net) actarray logs to valid ROS messages.

 **/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <player_log_actarray/PlayerActarray.h>

#include <fstream>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace ros;
using namespace player_log_actarray;

class PlayerLogToMsg
{
  protected:
    string tf_frame_;
    NodeHandle nh_;

  public:
    // ROS messages
    PlayerActarray msg_act_;

    string file_name_, msg_topic_;
    Publisher act_pub_;

    int joints_to_publish_;
    ifstream logfile_stream_;
    bool is_file_, spin_, multifile_;
    int save_pcd_actarray_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PlayerLogToMsg () : tf_frame_ ("laser_tilt_mount_link"),
                        is_file_ (true), multifile_(false)
    {
      nh_.param ("~joints_to_publish", joints_to_publish_, -1);     // publish only the first <joints_to_publish_> joints
      msg_topic_ = "/player_actarray";
      act_pub_   = nh_.advertise<PlayerActarray> (msg_topic_.c_str (), 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~PlayerLogToMsg () { }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //initialize args needed for multi log files
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void init()
  {
    save_pcd_actarray_ = 0;
    nh_.setParam("/save_pcd_actarray", 0);
    spin_ = true;
  }
  

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Start
    int
      start ()
    {
      // Open file
      logfile_stream_.open (file_name_.c_str ());
      long int nr_lines = count (istreambuf_iterator<char>(logfile_stream_), istreambuf_iterator<char> (), '\n');

      if (nr_lines == 0)
        ROS_WARN ("No lines found in %s", file_name_.c_str ());
      logfile_stream_.seekg (0, ios::beg);
      ROS_INFO ("Extracting poses from %s ...", file_name_.c_str ());
      msg_act_.header.frame_id = tf_frame_;
      return (0);
    }

  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Start overridden
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int
      start2 (string file_name)
  {
      // Open file
      logfile_stream_.open (file_name.c_str ());
      long int nr_lines = count (istreambuf_iterator<char>(logfile_stream_), istreambuf_iterator<char> (), '\n');
      
      if (nr_lines == 0)
        ROS_WARN ("No lines found in %s", file_name.c_str ());
      logfile_stream_.seekg (0, ios::beg);
      ROS_INFO ("Extracting poses from %s ...", file_name.c_str ());
      msg_act_.header.frame_id = tf_frame_;
      return (0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool
      spin ()
    {
      double ti, tj = 0, tdif = 0;
      int total_nr_poses = 0;
      string line;
      vector<string> st;

      while (nh_.ok ())
      {
        getline (logfile_stream_, line);
        // Do we assume that the input is a file? If so, and EOF, break
        if (logfile_stream_.eof () && is_file_)
          break;
        // If any bad/eof/fail flags are set, continue
        if (!logfile_stream_.good ())
        {
          usleep (500);
          logfile_stream_.clear ();
          continue;
        }
        // If the line is empty, continue
        if (line == "")
          continue;

        // Split a line into tokens
        boost::trim (line);
        boost::split (st, line, boost::is_any_of (" "), boost::token_compress_on);

        string line_type = st.at (0);
        if (line_type.substr (0, 1) == "#")
          continue;

        // Get the interface name
        string interface = st.at (3);

        // ---[ actarray
        if (interface.substr (0, 8) != "actarray")
          continue;

        ti = atof (line_type.c_str ());

        tdif = fabs (ti - tj);                    // Just in case time decides to go backwards :)

        msg_act_.header.stamp = Time (ti);

        // index = st.at (4)
        int type = atoi (st.at (5).c_str ());
        if (type != 1)                            // we only process DATA packets
          continue;

        int nr_joints = atoi (st.at (7).c_str ());
        if (joints_to_publish_ != -1)
          nr_joints = min (nr_joints, joints_to_publish_);

        msg_act_.joints.resize (nr_joints);
        if (msg_act_.joints.size () == 0)         // If no joints found, continue to the next packet
          continue;

        for (unsigned int i = 0; i < msg_act_.joints.size (); i++)
          msg_act_.joints[i] = atof (st.at (8 + 5 * i).c_str ());
        total_nr_poses++;

        if (is_file_)
          ROS_DEBUG ("Publishing data (%d joint positions) on topic %s in frame %s.",
                     (int)msg_act_.joints.size (), nh_.resolveName (msg_topic_).c_str (), msg_act_.header.frame_id.c_str ());
        act_pub_.publish (msg_act_);

        // Sleep for a certain number of seconds (tdif)
        if (tj != 0 && is_file_)
        {
          Duration tictoc (tdif);
          tictoc.sleep ();
        }

        spinOnce ();
        tj = ti;

        logfile_stream_.clear ();
      }

      // Close the file and finish the movie
      logfile_stream_.close ();
      ROS_INFO ("[done : %d poses extracted]", total_nr_poses);

      return (true);
    }

  ////////////////////////////////////////////////////////////////////////////////
  // Spin for multiple files (!)
  ////////////////////////////////////////////////////////////////////////////////
  bool
  spin2 ()
  {
    double ti, tj = 0, tdif = 0;
    int total_nr_poses = 0;
    string line;
    vector<string> st;
    ros::Rate loop_rate(0.2);
    
    while (nh_.ok ())
      {
	update_parameters_from_server();
	if(spin_)
	  {
	    getline (logfile_stream_, line);
	    // Do we assume that the input is a file? If so, and EOF, signal and wait for next file
	    if (logfile_stream_.eof () && is_file_ && file_name_ != "nothing")
	      {
		ROS_INFO("EOF actarray!!!");
		logfile_stream_.close();
		spin_ = false;
		continue;
	      }
	    //break;
	    // If any bad/eof/fail flags are set, continue
	    if (!logfile_stream_.good ())
	      {
		usleep (500);
		logfile_stream_.clear ();
		continue;
	      }
	    // If the line is empty, continue
	    if (line == "")
	      continue;
	    
	    // Split a line into tokens
	    boost::trim (line);
	    boost::split (st, line, boost::is_any_of (" "), boost::token_compress_on);
	    
	    string line_type = st.at (0);
	    if (line_type.substr (0, 1) == "#")
	      continue;
	    
	    // Get the interface name
	    string interface = st.at (3);
	    
	    // ---[ actarray
	    if (interface.substr (0, 8) != "actarray")
	      continue;
	    
	    ti = atof (line_type.c_str ());
	    
	    tdif = fabs (ti - tj);                    // Just in case time decides to go backwards :)
	    
	    msg_act_.header.stamp = Time (ti);
	    
	    // index = st.at (4)
	    int type = atoi (st.at (5).c_str ());
	    if (type != 1)                            // we only process DATA packets
	      continue;
	    
	    int nr_joints = atoi (st.at (7).c_str ());
	    if (joints_to_publish_ != -1)
	      nr_joints = min (nr_joints, joints_to_publish_);
	    
	    msg_act_.joints.resize (nr_joints);
	    if (msg_act_.joints.size () == 0)         // If no joints found, continue to the next packet
	      continue;
	    
	    for (unsigned int i = 0; i < msg_act_.joints.size (); i++)
	      msg_act_.joints[i] = atof (st.at (8 + 5 * i).c_str ());
	    total_nr_poses++;
	    
	    if (is_file_)
	      ROS_DEBUG ("Publishing data (%d joint positions) on topic %s in frame %s.",
			 (int)msg_act_.joints.size (), nh_.resolveName (msg_topic_).c_str (), msg_act_.header.frame_id.c_str ());
	    act_pub_.publish (msg_act_);
	    
	    // Sleep for a certain number of seconds (tdif)
	    if (tj != 0 && is_file_)
	      {
		Duration tictoc (tdif);
		tictoc.sleep ();
	      }
	     nh_.getParam("/save_pcd_actarray", save_pcd_actarray_);
	     if(save_pcd_actarray_ == 1 || save_pcd_actarray_ == 2)
	       nh_.setParam("/save_pcd_actarray", 0);
	    spinOnce ();
	    tj = ti;
	    
	    logfile_stream_.clear ();
	  }
	
	if(!spin_)
	  {
	    loop_rate.sleep();
	    tj = 0, tdif = 0;
	    total_nr_poses = 0;
	    line = "";
	    st.clear();
	    nh_.getParam("/save_pcd_actarray", save_pcd_actarray_);
	    if(save_pcd_actarray_ == 0)
	      nh_.setParam("/save_pcd_actarray", 1);
	    ROS_WARN("player actarray looping!!!!");
	  }
	
      }
    // Close the file and finish the movie
    if(logfile_stream_.is_open())
      logfile_stream_.close ();
    ROS_INFO ("[done : %d poses extracted]", total_nr_poses);
    
    return (true);
  }
    
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Update parameters from server
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
  update_parameters_from_server()
  {
    string log_filename_tmp = file_name_;
    nh_.getParam("/log_filename", file_name_);
    if (log_filename_tmp != file_name_ && file_name_ != "nothing")
      {
	ROS_INFO("Actarray filename set to:  %s", file_name_.c_str());
	start2(file_name_);
	spin_ = true;
      }
  }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 3)
  {
    ROS_WARN ("Syntax is: %s <file.log> [bool is_file (0/1)]", argv[0]);
    ROS_INFO ("           [note: set 'is_file' to 0 if the input .log file is a stream (i.e., data is still written to it during the execution of %s)]", argv[0]);
    ROS_INFO ("Usage example: %s 1190378714.log 0", argv[0]);
    return (-1);
  }

  init (argc, argv, "player_actarray_log_to_msg");

  PlayerLogToMsg p;
  p.file_name_ = string (argv[1]);
  p.is_file_   = atoi (argv[2]);

  //do we pass files in directory?
  if (atoi(argv[2]) != 2)
    {
      if (p.start () == -1)
	{
	  ROS_ERROR ("Error processing %s. Exiting.", argv[1]);
	  return (-1);
	}
      p.spin ();
    }
  else
    {
      p.init();
      p.file_name_ = "nothing";
      p.spin2();
     
    }

  return (0);
}
/* ]--- */
