/*
 * Copyright (c) 2010 Dejan Pangercic <pangercic -=- cs.tum.edu>
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

@brief triangular_mesh_to_vtk concatenates n [triangle_mesh/TriangleMesh] 
messages and writes them to a VTK (http://www.vtk.org/VTK/img/file-formats.pdf) compliant format.
NOTE: You have to specify the list of nodes publishing TriangleMesh(es) that you 
would like to concatenate and save to .vtk format. See \b subscribed_to_nodes parameter
in triangular_mesh_to_vtk.launch file.

@par Advertises
- \b topic with concatenated triangle_mesh/TriangleMesh messages
@par Subscribes
- \b topic with triangle_mesh/TriangleMesh message
@par Parameters
-  std::string input_mesh_topic, output_vtk_file
-  int file_name_counter_;
-  std::map <std::string, bool> subscribed_to_nodes_;
*/

// ROS core
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <point_cloud_mapping/cloud_io.h>
#include <triangle_mesh/TriangleMesh.h>

#include <math.h>

using namespace std;

class TriangleMeshToVTK
{
protected:
  //Subscribers/Publishers
  ros::NodeHandle nh_;
  ros::Subscriber mesh_sub_;
  ros::Publisher mesh_pub_;
  //Parameters
  std::string input_mesh_topic_, output_vtk_file_;
  int file_name_counter_;
  std::map <std::string, bool> subscribed_to_nodes_;
  std::map <std::string, bool>::iterator it_;
  
public:  
  // ROS messages
  triangle_mesh::TriangleMesh mesh_; 


  TriangleMeshToVTK (ros::NodeHandle &n) : nh_(n)
  {
    nh_.param("input_mesh_topic", input_mesh_topic_, std::string("mesh_input"));
    nh_.param("output_vtk_file", output_vtk_file_, std::string("mesh.vtk"));
    XmlRpc::XmlRpcValue v;
    nh_.param("subscribed_to_nodes", v, v);
    for(int i =0; i < v.size(); i++)
    {
      subscribed_to_nodes_[std::string(v[i])] = false;
    }
    for (it_=subscribed_to_nodes_.begin() ; it_ != subscribed_to_nodes_.end(); it_++)
      ROS_INFO("node_names: %s, toggle values: %d", (*it_).first.c_str(), (*it_).second);

    mesh_sub_ = nh_.subscribe(input_mesh_topic_, 10, &TriangleMeshToVTK::mesh_cb, this);
    mesh_pub_ = nh_.advertise<triangle_mesh::TriangleMesh> ("mesh_output", 1);
    mesh_.points.resize(0);
    mesh_.triangles.resize(0);
    file_name_counter_ = 0;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief mesh callback
   * \param mesh mesh messages constituting our to-be-reconstructed object
   */
  void mesh_cb(const triangle_mesh::TriangleMeshConstPtr& mesh)
  {
    mesh_.header = mesh->header;
    ROS_INFO("Sending node name: %s", mesh->sending_node.c_str());
    unsigned long size_points = mesh_.points.size();
    unsigned long size_triangles = mesh_.triangles.size();
    for (unsigned long i = 0; i < mesh->points.size(); i++)
      mesh_.points.push_back(mesh->points[i]);

    for (unsigned long j = 0; j < mesh->triangles.size(); j++)
    {
      mesh_.triangles.push_back(mesh->triangles[j]);
      mesh_.triangles[size_triangles + j].i += size_points;
      mesh_.triangles[size_triangles + j].j += size_points;
      mesh_.triangles[size_triangles + j].k += size_points;
    }
    
    if (subscribed_to_nodes_.find(mesh->sending_node) != subscribed_to_nodes_.end())
      subscribed_to_nodes_[mesh->sending_node] = true;

    //check if we received messages from all nodes subscribing to this one
    if (all_meshes_received(subscribed_to_nodes_))
    {
      mesh_pub_.publish(mesh_);
      write_vtk_file(output_vtk_file_, mesh_);
      mesh_.points.resize(0);
      mesh_.triangles.resize(0);
      for (it_=subscribed_to_nodes_.begin() ; it_ != subscribed_to_nodes_.end(); it_++)
        (*it_).second = false;
    }
  }  
  
  ////////////////////////////////////////////////////////////////////////////////
   /**
    * \brief check if we received all meshes for one model
    * \param subscribed_to_nodes map of node_name <=> toggle value indicating
    * whether message from that node has been received or not
    */
  bool all_meshes_received(std::map <std::string, bool> &subscribed_to_nodes)
  {
    for (it_=subscribed_to_nodes.begin() ; it_ != subscribed_to_nodes.end(); it_++)
    {
      if ((*it_).second == false)
        return false;
    }
    return true;
  }
  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief write TriangleMesh to vtk file
   * \param output vtk output file
   * \param mesh_ input mesh message
   */
  void write_vtk_file(std::string output, triangle_mesh::TriangleMesh &mesh_)
  {
    /* writing VTK file */
    ROS_WARN("Writting to vtk file");
    FILE *f;
    std::string s;
    file_name_counter_++;
    char file_name_counter[100];
    sprintf (file_name_counter, "%04d",  file_name_counter_);
    output = std::string(file_name_counter) + "_" + output;
    
    f = fopen(output.c_str(),"w");
    fprintf (f, "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS %ld float\n",mesh_.points.size());
    unsigned long i;
    
    for (i=0; i<mesh_.points.size(); i++)
    {
      fprintf (f,"%f %f %f ", mesh_.points[i].x, mesh_.points[i].y, mesh_.points[i].z);
      fprintf (f,"\n");
    }
    
    fprintf(f,"\nPOLYGONS %ld %ld\n", mesh_.triangles.size(), 4*mesh_.triangles.size());
    for (unsigned long i=0; i< mesh_.triangles.size(); i++)
    {
      if ((unsigned long)mesh_.triangles[i].i  >= mesh_.points.size() || mesh_.triangles[i].i < 0 ||  isnan(mesh_.triangles[i].i))
        ;
      else if  ((unsigned long)mesh_.triangles[i].j  >= mesh_.points.size() || mesh_.triangles[i].j < 0 ||  isnan(mesh_.triangles[i].j))
        ;
      else if  ((unsigned long)mesh_.triangles[i].k  >= mesh_.points.size() || mesh_.triangles[i].k < 0 ||  isnan(mesh_.triangles[i].k))
        ;
      else if ( mesh_.triangles[i].i == mesh_.triangles[i].j || mesh_.triangles[i].i == mesh_.triangles[i].k ||
                mesh_.triangles[i].j == mesh_.triangles[i].k)
        ;
      else
        fprintf(f,"3 %d %d %d\n",mesh_.triangles[i].i, mesh_.triangles[i].k, mesh_.triangles[i].j);
    }
    ROS_WARN("Writting to vtk %s file DONE", output.c_str());
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // \brief spin function
  bool spin ()
  {
    ros::Rate loop_rate(1);
    while(nh_.ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }    
    return (true);
  }
};

/* ---[ */
int main (int argc, char** argv)
{
  ros::init (argc, argv, "triangular_mesh_to_vtk");
  ros::NodeHandle n("~");
  TriangleMeshToVTK c(n);
  c.spin ();
  
  return (0);
}
/* ]--- */
