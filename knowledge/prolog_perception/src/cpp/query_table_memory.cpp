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
 * $Id: query_table_memory.cpp 17089 2009-06-15 18:52:12Z pangercic $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

@query_table_memory is a middleware between (ros)prolog and ros nodes.
Compilled into a shared library it
in general:
a)accepts prolog foreign function calls and
b)invokes ros ServiceClient calls

and in particular:
c)queries table_memmory node
(https://tum-ros-pkg.svn.sourceforge.net/svnroot/tum-ros-pkg/mapping/cloud_tools/src/table_memory.cpp)
for found tables and on them located clusters in PointCloud data.
Return type:
[[TableId, TimeStamp1, Tx1, Ty1, Tz1, Obj1, ObjID1, Ox1, Oy1, Oz1], ...]
**/

#include <ros/ros.h>

#include <ias_table_srvs/ias_table_clusters_service.h>
//prolog wrapper
#include <cstdlib>
#include <ctype.h>
#include <SWI-Prolog.h>
#include <vector>
//#include <ias_table_msgs/PrologReturn.h>

using namespace ias_table_srvs;

extern "C"
{ 
  
  using namespace std;

  foreign_t
  pl_getTableMemoryObjects(term_t l)    
  {
    int argc;
    argc=1;
    char **argv;
    argv = (char **)malloc(sizeof(char*));
    argv[0] = (char *)malloc(sizeof(char) *2);
    argv[0][0] = 'd';
    argv[0][1] = '\0';

    term_t tmp = PL_new_term_ref();
    
    ros::init(argc, argv, "testclient") ;
    ias_table_clusters_service srv;
    ros::NodeHandle n;
    std::string table_memory_srvs("/table_memory/table_memory_clusters_service");
    /** Subscribe to service */
    ros::service::waitForService(table_memory_srvs);
    ros::ServiceClient client = n.serviceClient<ias_table_clusters_service>(table_memory_srvs, true);
    float cluster_center[3];
    float table_center[3];
    double old_stamp = 0.0;
    int old_stamp_diff = 0;
    if(!client.call(srv))
      {
        ROS_ERROR("ERROR Calling %s", table_memory_srvs.c_str());
	return 0;
      }
    else
      {
        ROS_DEBUG("Called %s", table_memory_srvs.c_str());
      }
    ROS_DEBUG("Calling %s, prolog_return size %ld", table_memory_srvs.c_str(),  srv.response.prolog_return.size());
    if (srv.response.prolog_return.size() != 0)
      old_stamp = srv.response.prolog_return[0].stamp.toSec();
    for (unsigned int i = 0; i < srv.response.prolog_return.size(); i++)
	  { 
	    //add response variables of choice
	    //table id
	    if (!PL_unify_list(l, tmp, l) ||
		!PL_unify_integer(tmp, srv.response.prolog_return[i].table_id))
	      PL_fail;
	    ROS_DEBUG("table_id");

	    //check if the timestamp changed
	    ROS_DEBUG("old_stamp_diff: %lf", srv.response.prolog_return[i].stamp.toSec() - old_stamp);
	    if ((srv.response.prolog_return[i].stamp.toSec() - old_stamp) > 0.0)
	      {
		ROS_DEBUG("old_stamp_diff: %f, %d", srv.response.prolog_return[i].stamp.toSec() - old_stamp, old_stamp_diff);
		old_stamp_diff++;
	    	old_stamp = srv.response.prolog_return[i].stamp.toSec();
	      }

	    //time stamp as an integer
	    if (!PL_unify_list(l, tmp, l) ||
		!PL_unify_integer(tmp, old_stamp_diff))
	      PL_fail;
	    ROS_DEBUG("stamp: %f", srv.response.prolog_return[i].stamp.toSec());

	    //table center
	    table_center[0] = srv.response.prolog_return[i].table_center.x;
	    table_center[1] = srv.response.prolog_return[i].table_center.y;
	    table_center[2] = srv.response.prolog_return[i].table_center.z;
	    for (int ii = 0; ii < 3; ii++)
              {
		if(!PL_unify_list(l, tmp, l) ||  !PL_unify_float(tmp, table_center[ii]))
		  {
		    PL_fail;
		    break;
		  }
	      }
	    ROS_DEBUG("table_center");
	    

	    //ROS_DEBUG("semantic types %s",  srv.response.prolog_return[i].cluster_semantic_types[0].c_str());
	    //cluster semantic type
	     if (srv.response.prolog_return[i].cluster_semantic_types.size() != 0)
	       {
		 if (!PL_unify_list(l, tmp, l) ||
		     !PL_unify_string_chars(tmp, srv.response.prolog_return[i].cluster_semantic_types[0].c_str()))
		   PL_fail;
		 ROS_DEBUG("semantic types %s",  srv.response.prolog_return[i].cluster_semantic_types[0].c_str());
	       }
	     else
	       {
		 if (!PL_unify_list(l, tmp, l) ||
		     !PL_unify_string_chars(tmp, "nn"))
		   PL_fail;
		 ROS_DEBUG("semantic types nn");
	       }

	     
	     //cluster id
	     if (!PL_unify_list(l, tmp, l) ||
		 !PL_unify_integer(tmp, srv.response.prolog_return[i].cluster_id))
	       PL_fail;
	     ROS_DEBUG("cluster_id");
	     
	    //cluster center
	    cluster_center[0] = srv.response.prolog_return[i].cluster_center.x;
	    cluster_center[1] = srv.response.prolog_return[i].cluster_center.y;
	    cluster_center[2] = srv.response.prolog_return[i].cluster_center.z;
	    for (int ii = 0; ii < 3; ii++)
              {
		if(!PL_unify_list(l, tmp, l) ||  !PL_unify_float(tmp, cluster_center[ii]))
		  {
		    PL_fail;
		    break;
		  }
	      }
	    ROS_DEBUG("cluster_center");
	  }
    free(argv[0]);
    free(argv);
    return PL_unify_nil(l);
  }

  /////////////////////////////////////////////////////////////////////////////
  // register foreign functions
  ////////////////////////////////////////////////////////////////////////////
  install_t
  install()
  { 
    PL_register_foreign("getTableMemoryObjects", 1, (void *)pl_getTableMemoryObjects, 0);
  }
}
