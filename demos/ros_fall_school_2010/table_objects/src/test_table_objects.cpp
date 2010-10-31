/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: table_objects.cpp 30899 2010-07-16 04:56:51Z rusu $
 *
 */

/**
\author Zoltan-Csaba Marton
 **/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>

#include <table_objects/GetObjects.h>

ros::ServiceClient client;
ros::Publisher pub;
ros::Publisher pub_marker;

void callback (sensor_msgs::PointCloud2 cloud)
{
  table_objects::GetObjects srv;
  srv.request.data = cloud;
  if (client.call(srv))
  {
    //for (std::vector<mapping_msgs::CollisionObject>::iterator it = srv.response.table_objects.begin (); it != srv.response.table_objects.end (); it++)
    for (size_t i = 0; i < srv.response.table_objects.size (); i++)
    {
      mapping_msgs::CollisionObject obj = srv.response.table_objects[i];
      //obj.header.stamp += ros::Duration (i * 0.001); // TODO is this hack of Radu's needed?
      pub.publish (obj);

      // publishing marker for checking
      visualization_msgs::Marker marker;
      marker.header = obj.header;
      marker.ns = "CollisionObjects";
      marker.id = i;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position = obj.poses[0].position;
      marker.pose.orientation = obj.poses[0].orientation;
      marker.scale.x = obj.shapes[0].dimensions[0];
      marker.scale.y = obj.shapes[0].dimensions[1];
      marker.scale.z = obj.shapes[0].dimensions[2];
      marker.color.a = 0.5;
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      pub_marker.publish (marker);
    }
    ROS_INFO("Published %zu objects", srv.response.table_objects.size ());
  }
  else
    ROS_ERROR("Failed to call service!");
}

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "test_table_objects");

  ros::NodeHandle nh;

  client = nh.serviceClient<table_objects::GetObjects>("get_table_objects");
  
  pub = nh.advertise<mapping_msgs::CollisionObject> ("collison_objects", 50);
  pub_marker = nh.advertise<visualization_msgs::Marker> ("collison_object_markers", 50);

  ros::Subscriber sub = nh.subscribe("/extract_objects_indices/output", 5, callback);

  ros::spin ();

  return (0);
}
