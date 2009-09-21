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
 * $Id: rotating_unit.cpp 90 2009-08-24 09:44:53Z veedee $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b rotating_dp_ptu controls a PTU D47 Directed Perception unit and a SICK LMS400 to acquire dense 3D point cloud data automatically.

 **/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include <point_cloud_mapping/geometry/nearest.h>

#include <mapping_srvs/RotatePTU.h>
#include <mapping_srvs/TriggerSweep.h>
#include <perception_srvs/David.h>
using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;

#define ANGLE -180.0
#define S_ANGLE -180.0

class RotatingDPPTU
{
  protected:
    NodeHandle nh_;
    boost::mutex s_lock_, a_lock_;
    bool david_connect_, spin_;
    int total_laser_scans_;
  public:
    // ROS messages
    list<LaserScanConstPtr> scans_;
    double first_act_stamp_;

    PointCloud cloud_;

    Publisher cloud_pub_;

    ServiceClient ptu_serv_, scan_serv_, david_scan_, save_images_serv_;

    // Parameters
    double min_distance_, max_distance_, laser_min_angle_, laser_max_angle_;
    double angle_step_;
    string object_;
    bool left_arm_;
    //do we use david
    int is_david_;
    //do we want to rotate with PTU
    int is_ptu_;
    //do we want to take snapshots
    int is_image_;
    //do we sweep with laser (e.g. LMS400)
    int is_laser_;
    //how many times do we sweep the laser
    int sweeps_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    RotatingDPPTU () : total_laser_scans_ (0),
                       left_arm_ (true)
    {
      david_connect_ = true, spin_ = true;
      nh_.param ("~min_distance", min_distance_, .7);     // minimum distance range to be considered
      nh_.param ("~max_distance", max_distance_, 3.01);   // maximum distance range to be considered
      nh_.param ("~angle_step", angle_step_, 30.0);     // ptu rotating angle
      nh_.param ("~object", object_, string("mug"));   // name of object to be scanned
      nh_.param ("~sweeps", sweeps_, 1); 		// number of sweeps of the laser arm per scan
      nh_.param ("~is_ptu", is_ptu_, 0);
      nh_.param ("~is_image", is_image_, 0);
      nh_.param ("~is_laser", is_laser_, 0);
      nh_.param("~is_david", is_david_, 0);

      cloud_pub_ = nh_.advertise<PointCloud> ("/tilt_laser_cloud", 1);

      ptu_serv_  = nh_.serviceClient<mapping_srvs::RotatePTU>("get_angle_service");
      scan_serv_ = nh_.serviceClient<mapping_srvs::TriggerSweep>("amtec_sweep");
      david_scan_ = nh_.serviceClient<perception_srvs::David>("david");
      save_images_serv_ = nh_.serviceClient<mapping_srvs::TriggerSweep>("save_images_service");
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~RotatingDPPTU () { }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix3f
      assembleRotationMatrixZ (float angle)
    {
      double cz = cos (angle), sz = sin (angle);
      Eigen::Matrix3f rot_mat;
      rot_mat << 
        cz, -sz, 0, 
        sz,  cz, 0,
        0 ,   0, 1;
      return (rot_mat);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Rotate a PointCloud relative to the angle of the turning table
    void
      rotateCloudRelative (float relative_angle, const PointCloud &cloud_in, PointCloud &cloud_out)
    {
      cloud_out.header   = cloud_in.header;
      cloud_out.channels = cloud_in.channels;
      cloud_out.points.resize (cloud_in.points.size ());

      // Demean the point cloud 
      Point32 centroid;
      cloud_geometry::nearest::computeCentroid (cloud_in, centroid); 
      
      // Rotate it around Z with the given angle
      Eigen::Matrix3f rot_mat = assembleRotationMatrixZ (relative_angle);
      
      for (unsigned int i = 0; i < cloud_in.points.size (); i++)
      {
        double cx = cloud_in.points[i].x - centroid.x; 
        double cy = cloud_in.points[i].y - centroid.y; 
        double cz = cloud_in.points[i].z - centroid.z; 

        cloud_out.points[i].x = rot_mat (0, 0) * cx + rot_mat (0, 1) * cy + rot_mat (0, 2) * cz;
        cloud_out.points[i].y = rot_mat (1, 0) * cx + rot_mat (1, 1) * cy + rot_mat (1, 2) * cz;
        cloud_out.points[i].x = rot_mat (2, 0) * cx + rot_mat (2, 1) * cy + rot_mat (2, 2) * cz;
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      spin ()
    {
      //update parameters on the fly
      
      
      float s_angle = S_ANGLE, angle = ANGLE; //roslaunch takes double only
      mapping_srvs::RotatePTU p_s;
      mapping_srvs::TriggerSweep s_s;
      mapping_srvs::TriggerSweep i_s;
      perception_srvs::David d_s;
      ros::Duration tictoc (1, 0);
      ros::Duration wait_grab_texture (3, 0);
      ros::Duration david_wait (0.1);
      ros::Rate loop_rate(5);
      PointCloud cloud_r;
      bool home;
      home = true;

      while (nh_.ok ())
      {
	update_parameters_from_server();
	ROS_INFO("is_image %d", is_image_);
	ROS_INFO("is_laser %d", is_laser_);
	if(spin_)
	  {
	    ROS_INFO("New Spin----------------------------------------------");
	    ROS_INFO("------------------------------------------------------");
	    ROS_INFO("------------------------------------------------------");
	    //send PTU to 0 position
// 	    if (home)
// 	      {
// 		p_s.request.angle = 30.0;
// 		ptu_serv_.call (p_s);
// 		ROS_INFO ("Homing ____PTU______ ");
// 		tictoc.sleep ();
// 		home = false;
// 	      }
	    // Send a request to the PTU to move
	    if (is_ptu_)
	      {
		p_s.request.angle = angle;
		ROS_INFO ("Setting ____PTU______ angle to %f. Sleeping for %f seconds.", angle, tictoc.toSec ());
		ptu_serv_.call (p_s);
		tictoc.sleep ();
	      }
	    
	    if (is_david_)
	      {
	        //ROS_WARN("In is_david");
		// Start david scanning system
		if(david_connect_){
		  ROS_WARN("Connecting to DAVID server");
		  //connect only once per node cycle
		  d_s.request.david_method = "connect";
		  david_connect_ = false;
		}
		david_scan_.call(d_s);
		david_wait.sleep();
		d_s.request.david_method = "erase";
		david_scan_.call(d_s);
		david_wait.sleep();
		d_s.request.david_method = "eraseTexture";
		david_scan_.call(d_s);
		david_wait.sleep();
		d_s.request.david_method = "start";
		david_scan_.call(d_s);
		ROS_INFO ("David started. Sleeping for %f seconds.", tictoc.toSec ());
		tictoc.sleep();
	      }
	    // Trigger the LMS400 to sweep
	    // or
	    // only rotate one joint if scanning with David system
	    if (is_laser_)
	      {
		tictoc.sleep ();
		for (int i = 0; i < sweeps_; i++)
		  {
		    s_s.request.object = object_;
		    s_s.request.angle_filename = angle;
		    scan_serv_.call (s_s);
		    ROS_INFO ("___Sweeping___ %d times. Setting angle to %f, object to %s. ", i, angle, object_.c_str());
		  }
	      }
	    // Rotate the point cloud and publish it
	    //rotateCloudRelative (angle - s_angle, s_s.response.cloud, cloud_r);
	    
	    //         if (cloud_r.points.size () > 0)
	    //         {
	    //           cloud_pub_.publish (cloud_r);
	    //           ROS_INFO ("Publishing cloud with %d points and %d channels on topic %s.", 
	    //  (int)cloud_r.points.size (), (int)cloud_r.channels.size (), cloud_r.header.frame_id.c_str ());
	    //         }
	    
	    // Stop David system
	    if (is_david_)
	      {
		d_s.request.david_method = "stop";
		david_scan_.call(d_s);
		david_wait.sleep();
		d_s.request.david_method = "grabTexture";
		david_scan_.call(d_s);
		david_wait.sleep();
		//david save name
		char angle_tmp[100];
		int angle_int = round(angle);
		sprintf (angle_tmp, "%d",  angle_int);
		string david_save = "save_" + object_ + "_" +  string(angle_tmp) + "_" + ".obj";
		ROS_INFO("Saving David scan to %s", david_save.c_str());
		d_s.request.david_method = david_save;
		david_scan_.call(d_s);
		ROS_INFO("Sleeping for %f seconds.", wait_grab_texture.toSec ());
		wait_grab_texture.sleep();
		//added temporarily to avoid save being called twice
		d_s.request.david_method = "erase";
		david_scan_.call(d_s);
		ROS_INFO ("David stopped. Sleeping for %f seconds.", tictoc.toSec ());
		tictoc.sleep();
	      }

	    if (is_image_)
	      {
		i_s.request.object = object_;
		i_s.request.angle_filename = angle;
		ROS_INFO ("Capturing Images!");
		save_images_serv_.call (i_s);
		tictoc.sleep ();
	      }
	    // Increase angle and repeat
	    angle += angle_step_;
	    if (angle > 180.0)
	      {
		spin_ = false;
		s_angle = S_ANGLE, angle = ANGLE;
		home = true;
		//break;
	      }
	  }
	ROS_WARN("Looping!!!!");
        ros::spinOnce ();
	if(!spin_)
	  loop_rate.sleep();  
      }
 
       ROS_INFO ("Another Scanning completed.");
       return (true);
    }
  
 //////////////////////////////////////////////////////////////////////////////////
  // //update parameters on the fly
  //////////////////////////////////////////////////////////////////////////////////
  void
  update_parameters_from_server()
  {
    if (nh_.hasParam("~object"))
      {
	string object_tmp_ = object_;
	nh_.getParam("~object", object_);
	if (object_tmp_ != object_)
	  {
	    spin_ = true;
	  }
	ROS_INFO("Updating parameters from server");
      }
  }

};

/* ---[ */
int
  main (int argc, char** argv)
{
  init (argc, argv, "rotating_dp_ptu");

  RotatingDPPTU p;
  p.spin ();

  return (0);
}
/* ]--- */
