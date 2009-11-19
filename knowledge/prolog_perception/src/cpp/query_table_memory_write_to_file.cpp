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
 * $Id: query_table_memory_write_to_file.cpp 17089 2009-06-15 18:52:12Z pangercic $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

@query_table_memory_write_to_file queries table_memmory node
(https://tum-ros-pkg.svn.sourceforge.net/svnroot/tum-ros-pkg/mapping/cloud_tools/src/table_memory.cpp)
for found tables and on them located clusters in PointCloud data. As oppose to query_table_memory
it simply writes a return to a file on a disk (data/table_memory.dat).
Return type:
[[TableId, TimeStamp1, Tx1, Ty1, Tz1, Obj1, ObjID1, Ox1, Oy1, Oz1], ...]
**/


#include <ros/ros.h>
#include <ias_table_srvs/ias_table_clusters_service.h>
//prolog wrapper
#include <cstdlib>
#include <ctype.h>
#include <vector>
#include <fstream>
#include <ctime>
#include<iomanip>
using std::ofstream;
using namespace ias_table_srvs;

int record_to_file(std::vector<std::string> data)
{
  ofstream outdata; // outdata is like cin    
  outdata.open("data/table_memory.dat", std::ios::app); // opens the file
  if( !outdata ) 
    { 
      // file couldn't be opened
      ROS_ERROR("Error: file could not be opened");
      exit(1);
    }
  for (unsigned int i =0; i < data.size(); i++)
    outdata << data[i] << std::endl;
  outdata.close();
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "query_table_memmory_write_to_file");
  ias_table_clusters_service call;
  ros::NodeHandle n;
  std::string table_memory_srvs("/table_memory/table_memory_clusters_service");
  /** Subscribe to service */
  ros::service::waitForService(table_memory_srvs);
  ros::ServiceClient client = n.serviceClient<ias_table_clusters_service>(table_memory_srvs, true);
    
  ros::Duration s(10.0);
  //  while(n.ok())
  //{
  //  ROS_DEBUG("Sleeping");
  //  s.sleep();
  //  ros::spinOnce();
      if(!client.call(call))
	{
	  ROS_ERROR("ERROR Calling %s", table_memory_srvs.c_str());
	}
      else
	{
	  ROS_DEBUG("Called service %s", table_memory_srvs.c_str());
	}
      std::vector<std::string> data_vector_;
      std::ostringstream data_;
      for (unsigned int i = 0; i < call.response.prolog_return.size(); i++)
	{
	  std::ostringstream data_;	  
	  ROS_DEBUG("table_id: %ld stamp: %.10lf", call.response.prolog_return[i].table_id,  call.response.prolog_return[i].stamp.toSec());
	  ROS_DEBUG("table center: %f %f %f", call.response.prolog_return[i].table_center.x, call.response.prolog_return[i].table_center.y,
		   call.response.prolog_return[i].table_center.z);
	  data_ << call.response.prolog_return[i].table_id << " " << 
	    call.response.prolog_return[i].table_center.x << " " << 
	    call.response.prolog_return[i].table_center.y << " " << 
	    call.response.prolog_return[i].table_center.z << " " <<
	    std::setprecision(20) << 
	    call.response.prolog_return[i].stamp.toSec() << " " <<  std::setprecision(5);
	  ROS_DEBUG("cluster center: %f %f %f", call.response.prolog_return[i].cluster_center.x, call.response.prolog_return[i].cluster_center.y,
		   call.response.prolog_return[i].cluster_center.z);
	  data_ << call.response.prolog_return[i].cluster_center.x << " " << 
	    call.response.prolog_return[i].cluster_center.y << " " << 
	    call.response.prolog_return[i].cluster_center.z << " ";
	  for (unsigned int j = 0; j < call.response.prolog_return[i].cluster_colors.size(); j++)
	    {
	      ROS_DEBUG("cluster colors %s", call.response.prolog_return[i].cluster_colors[j].c_str());
	      data_ << call.response.prolog_return[i].cluster_colors[j] << "\n";
	    }					
	  ROS_DEBUG("Pushing facts %s", data_.str().c_str());
	  data_vector_.push_back(data_.str());
	}
      record_to_file(data_vector_);	
      // }
}
