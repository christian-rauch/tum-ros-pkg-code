/* 
 * Copyright (c) 2010, Nico Blodow <blodow@cs.tum.edu>
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

// #include <unistd.h>

#include <ctime>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
//later: cut polygon instead of AABB
#include <geometry_msgs/PolygonStamped.h>

/** 
@file

@brief  box_filter cuts out the box part of the pointcloud.

@par Advertises
 - \b box_filter_node/cloud_box_clip topic

@par Subscribes
 - \b /cloud_pcd topic


@par Parameters
- \b input_cloud_topic
- \b output_cloud_topic
- \b box_min_x
- \b box_max_x
- \b box_min_y
- \b box_max_y
- \b box_min_z
- \b box_max_z
*/

class BoxFilter
{
  protected:
    ros::NodeHandle nh_;
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;

    double box_min_x_;
    double box_max_x_;
    double box_min_y_;
    double box_max_y_;
    double box_min_z_;
    double box_max_z_;

  public:
    BoxFilter (ros::NodeHandle &anode) : nh_(anode)
    {
      nh_.param ("input_cloud_topic", input_cloud_topic_, std::string("cloud_pcd"));
      nh_.param ("output_cloud_topic", output_cloud_topic_, std::string("cloud_box_clipped"));
      
      nh_.param ("box_min_x", box_min_x_, -4.0);
      nh_.param ("box_max_x", box_max_x_,  3.0);
      nh_.param ("box_min_y", box_min_y_, -3.0);
      nh_.param ("box_max_y", box_max_y_,  2.5);
      nh_.param ("box_min_z", box_min_z_,  3.0);
      nh_.param ("box_max_z", box_max_z_,  3.0);
      
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &BoxFilter::cloud_cb, this);
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud> (output_cloud_topic_, 1);
    }
  /**
   * \brief cloud callback and the core filtering function at the same time 
   * \param cloud input point cloud to be cut off
   */
    void
      cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
    {
      // copy header over from input cloud
      sensor_msgs::PointCloud cloud_out;
      cloud_out.header = cloud->header;

      // resize channels and copy their names
      cloud_out.channels.resize (cloud->channels.size());
      for (unsigned int i = 0; i < cloud->channels.size(); i++)
        cloud_out.channels[i].name = cloud->channels[i].name;

      // go through all points 
      for (unsigned int i = 0; i < cloud->points.size(); i++)
      {
        // select all points that are within in the specified points
        if (cloud->points[i].x > box_min_x_ && cloud->points[i].x < box_max_x_
           && cloud->points[i].y > box_min_y_ && cloud->points[i].y < box_max_y_
           && cloud->points[i].z > box_min_z_ && cloud->points[i].z < box_max_z_)
        {
          // copy point over
          cloud_out.points.push_back (cloud->points[i]);

          // copy channel values over
          for (unsigned int j = 0; j < cloud->channels.size(); j++)
            cloud_out.channels[j].values.push_back (cloud->channels[j].values[i]);
        }
      }
      cloud_pub_.publish (cloud_out); 
    }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "box_filter");

  ros::NodeHandle nh("~");
  BoxFilter n (nh);
  ros::spin ();

  return (0);
}



