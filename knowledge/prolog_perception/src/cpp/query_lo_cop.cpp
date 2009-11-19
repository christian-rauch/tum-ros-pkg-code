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
 * $Id: query_lo_cop.cpp 17089 2009-06-15 18:52:12Z pangercic $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

@query_lo_cop is an example client to obtain a unique 
lo(https://tum-ros-pkg.svn.sourceforge.net/svnroot/tum-ros-pkg/locations/lo/README)
id for a data structure and trigger a 
cop(https://tum-ros-pkg.svn.sourceforge.net/svnroot/tum-ros-pkg/locations/cop)
query with it. In following a location query for clusters in SWISSRANGER 
PointCloud is demonstrated.
**/


#include <ros/ros.h>
#include <vision_srvs/srvjlo.h>
#include <vision_srvs/cop_call.h>
#include <vision_msgs/partial_lo.h>
#include <vision_msgs/cop_answer.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/geometry/statistics.h> 
using namespace vision_srvs; 
using namespace vision_msgs;
using namespace geometry_msgs;
using namespace std;
using namespace sensor_msgs;

#define JLO_IDQUERY "idquery"
#define JLO_FRAMEQUERY "framequery"
#define JLO_DELETE "del"
#define JLO_UPDATE "update"
#define ID_WORLD 1

bool exit_cb = false;
vector<Point32> cluster_centers_g, cluster_covs_g;
void cop_cb(const boost::shared_ptr<const cop_answer> &msg)
{
  ROS_DEBUG("got answer from cop! (Errors: %s)\n", msg->error.c_str());
  for(unsigned int i = 0; i < msg->found_poses.size(); i++)
    {
      const aposteriori_position &pos =  msg->found_poses[i];
      ROS_DEBUG("Foub Obj nr %d with prob %f at pos %d\n", (int)pos.objectId, pos.probability, (int)pos.position);
    }
  ROS_DEBUG("End!\n");
  exit_cb = true;
}

//Assuming to get clusters one by one on the topic
//TODO: Implementation for the case when we get an annotated point cloud with 
//multiple clusters
void clusters_cb(const PointCloudConstPtr& cloud)
{
  Point32 minP;
  Point32 maxP;
  Point32 center, extents;
  cloud_geometry::statistics::getMinMax (*cloud, minP, maxP);
  center.x = minP.x + (maxP.x - minP.x) * 0.5;                                                                                         
  center.y = minP.y + (maxP.y - minP.y) * 0.5;                                                                                         
  center.z = minP.z + (maxP.z - minP.z) * 0.5;   
  cluster_centers_g.push_back(center);
  extents.x = center.x - minP.x;                                                                                                                 
  extents.y = center.y - minP.y;                                                                                                                 
  extents.z = center.z - minP.z; 
  cluster_covs_g.push_back(extents);
}

vector<unsigned int> update_jlo(vector<Point32> cluster_centers, vector<Point32> cluster_covs, ros::ServiceClient client)
{
  vector<unsigned int> lo_ids;
  for (unsigned int cluster = 0; cluster < cluster_centers.size();  cluster++)
    {
      srvjlo call;
      call.request.command = JLO_UPDATE;
      //world frame
      call.request.query.parent_id = 1;
      call.request.query.id = 0;
      
      //fill in pose
      int width = 4;
      for(int r = 0; r < width; r++)
        {
          for(int c = 0; c < width; c++)
            {
              if(r == c)
                call.request.query.pose[r * width + c] = 1;
              else
                call.request.query.pose[r * width + c] = 0;
            }
        }
      call.request.query.pose[3] = cluster_centers[cluster].x;
      call.request.query.pose[7] =  cluster_centers[cluster].y;
      call.request.query.pose[11] =  cluster_centers[cluster].z;
      
      //fill in covariance matrix: 0.9 * 1/2*max(cluster)-min(cluster) [m]
      width = 6;
      for(int r = 0; r < width; r++)
        {
          for(int c = 0; c < width; c++)
            {
              if(r == c)
                call.request.query.cov[r * width + c] = 0.0;
              else
                call.request.query.cov[r * width + c] = 0;
            }
        }
      call.request.query.cov[0] = cluster_covs[cluster].x;
      call.request.query.cov[7] = cluster_covs[cluster].y;
      call.request.query.cov[13] = cluster_covs[cluster].z;
      if (!client.call(call))
        {
          ROS_ERROR("Error in ROSjloComm: Update of pose information not possible!\n");
        }
      else if (call.response.error.length() > 0)
        {
          ROS_ERROR("Error from jlo: %s!\n", call.response.error.c_str());
        } 
      ROS_DEBUG("New Id: %ld (parent %ld)\n", call.response.answer.id, call.response.answer.parent_id);
      width = 4;
      for(int r = 0; r < width; r++)
        {
          for(int c = 0; c < width; c++)
            {
              ROS_DEBUG("%f", call.response.answer.pose[r * width + c]);
            }
        }
      lo_ids.push_back(call.response.answer.id);
    }
  return lo_ids;
}

bool call_cop(vector<unsigned int> pos_ids, std::string stTopicName, ros::ServiceClient client)
{ 
  vector<string> colors;
  colors.push_back("black"), colors.push_back("white");
  colors.push_back("red"), colors.push_back("green");
  colors.push_back("blue");
  for (unsigned int col = 0; col < colors.size(); col ++)
    {
      cop_call call;
      call.request.outputtopic = stTopicName;
      call.request.object_classes.push_back(colors[col]);
      call.request.action_type = 0;
      call.request.number_of_objects = 1;
      for (unsigned int obj = 0; pos_ids.size(); obj++)
        {
          apriori_position pos;
          pos.probability = 1.0;
          pos.positionId = pos_ids[obj];
          call.request.list_of_poses.push_back(pos);
        }
      if(!client.call(call))
        {
          ROS_DEBUG("Error calling cop\n");
          return false;
        }
      else
        {
          ROS_DEBUG("Called cop \n");
        }
    }
  return true;
}

// Main
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "query_lo_cop") ;
  ros::NodeHandle n;
  //check if service servers are there
  if(!ros::service::waitForService("/located_object") || !ros::service::waitForService("/tracking/in"))
    return 0;
  ros::ServiceClient client_lo = n.serviceClient<srvjlo>("/located_object", true);
  ros::ServiceClient client_cop = n.serviceClient<cop_call>("/tracking/in", true);
  /**Subscribe for cop answer*/
  std::string cop_topic_name = "/tracking/out";
  ros::Subscriber read_cop_answer = n.subscribe<cop_answer>(cop_topic_name, 1000, &cop_cb);
  //subscribe to get clusters
  std::string clusters_topic_name = "/clusters";
  ros::Subscriber read_clusters = n.subscribe(clusters_topic_name, 1000, &clusters_cb);
  vector<unsigned int> lo_ids;

  lo_ids = update_jlo(cluster_centers_g, cluster_covs_g, client_lo);
  call_cop(lo_ids, cop_topic_name, client_cop);
  ros::Rate r(100);
  while(n.ok() && !exit_cb)
    {
      ros::spinOnce();
      r.sleep();
    }
  return 0;
}



