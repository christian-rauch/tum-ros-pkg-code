/*
 * Copyright (c) 2010 Nico Blodow <blodow -=- cs.tum.edu>
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
 *
 */

/** 
@file
@brief table_msg_split subscribes to topic with TableWithObjects message, extracts
table Polygon and object condidates (as PointClouds) and publishes them on the
separate topic. 

@par Advertises
- \b topic with PointCloud message
- \b topic with Polygon message
@par Subscribes
- \b topic with TableWithObjects message
@par Parameters
- \b string input_table_topic_
- \b string output_pcds_topic_
- \b string output_polygon_topic_
*/

// #include <unistd.h>

#include <ctime>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>
#include <ias_table_msgs/TableWithObjects.h>
#include <geometry_msgs/PolygonStamped.h>

class TableMsgSplit
{
  protected:
    ros::NodeHandle nh_;
    std::string input_table_topic_;
    std::string output_polygon_topic_;
    std::string output_pcds_topic_;
    ros::Subscriber table_sub_;

    ros::Publisher polygon_pub_;
    ros::Publisher clusters_pub_;

  public:
    TableMsgSplit (ros::NodeHandle &anode) : nh_(anode)
    {
      nh_.param ("input_table_topic", input_table_topic_, std::string("table_with_objects"));
      nh_.param ("output_pcds_topic", output_pcds_topic_, std::string("table_pcds"));
      nh_.param ("output_polygon_topic", output_polygon_topic_, std::string("table_polygon"));
      
      table_sub_ = nh_.subscribe (input_table_topic_, 1, &TableMsgSplit::table_cb, this);
      polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped> (output_polygon_topic_, 1);
      clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud> (output_pcds_topic_, 1);
    }
    
    void
      table_cb (const ias_table_msgs::TableWithObjects::ConstPtr& table)
    {
      geometry_msgs::PolygonStamped p;
      p.header = table->header;
      p.polygon = table->table;
      polygon_pub_.publish (p);
      
      for (unsigned int i = 0; i < table->objects.size (); i++)
      {
        sensor_msgs::PointCloud pc;
        pc.header = table->header;
	      pc.points = table->objects[i].points.points;
        clusters_pub_.publish (pc);
      }
    }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "table_msg_split");

  ros::NodeHandle nh("~");
  TableMsgSplit n (nh);
  ros::spin ();

  return (0);
}


