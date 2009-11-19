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
 * $Id: query_tabletop_missing_objects.cpp 17089 2009-06-15 18:52:12Z pangercic $
 *
 */

/**
@mainpage

@htmlinclude manifest.html
@query_tabletop_missing_objects is a middleware between (ros)prolog and ros nodes.
Compilled into a shared library it
in general:
a)accepts prolog foreign function calls and
b)invokes ros ServiceClient calls

and in particular:
c)queries table_memmory node
(https://tum-ros-pkg.svn.sourceforge.net/svnroot/tum-ros-pkg/mapping/cloud_tools/src/table_memory.cpp)
for found tables and on them located clusters in PointCloud data.
Return type:
[[TableId, Tx1, Ty1, Tz1, Obj1, Ox1, Oy1, Oz1], ...]
The data is used in another pipeline with knowledge processing 
and reasoning in order to infer the missing objects on the tabletop.
**/

#include "ros/ros.h"
#include "ros/node_handle.h"
#include <ias_table_srvs/ias_table_clusters_service.h>
#include <cstdlib>
#include <ctype.h>
#include <SWI-Prolog.h>
extern "C"
{
  using namespace  ias_table_srvs;    

  foreign_t
  pl_getFoundObjectsROS(term_t l)    
  {
    int argc;
    argc=1;
    char **argv;
    argv = (char **)malloc(sizeof(char*));
    argv[0] = (char *)malloc(sizeof(char) *2);
    argv[0][0] = 'e';
    argv[0][1] = '\0';
    ros::init(argc, argv, "query_tabletop_missing_objects");
    ros::NodeHandle n;
    std::string tabletop_missing_objects_srvs("/tabletop_missing_objects_server/tabletop_missing_objects_service");
    ros::service::waitForService(tabletop_missing_objects_srvs);
    ros::ServiceClient client = n.serviceClient<ias_table_clusters_service>(tabletop_missing_objects_srvs);
    ias_table_clusters_service srv;
    term_t tmp = PL_new_term_ref();
    float cluster_center[3];
    float table_center[3];
    ROS_DEBUG(" srv.response.table.header.seq");
    if (client.call(srv))
      {
	ROS_DEBUG("Calling %s, prolog_return size %ld", tabletop_missing_objects_srvs.c_str(),  srv.response.prolog_return.size());
	for (unsigned int i = 0; i < srv.response.prolog_return.size(); i++)
	  { 
	    //add response variables of choice
	    if (!PL_unify_list(l, tmp, l) ||
		!PL_unify_integer(tmp, srv.response.prolog_return[i].table_id))
	      PL_fail;
	    ROS_DEBUG("table_id");
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
	    
	    if (!PL_unify_list(l, tmp, l) ||
		!PL_unify_string_chars(tmp, srv.response.prolog_return[i].cluster_semantic_types[0].c_str()))
	      PL_fail;
	    ROS_DEBUG("semantic types %s",  srv.response.prolog_return[i].cluster_semantic_types[0].c_str());

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
	return PL_unify_nil(l);
	ROS_DEBUG("exiting if");
      }
    else
      {
        ROS_ERROR("Failed to call service %s",  tabletop_missing_objects_srvs.c_str());
        return -1;
      }
    free(argv[0]);
    free(argv);
    return 0;
  }
  
  /////////////////////////////////////////////////////////////////////////////
  // register foreign functions
  ////////////////////////////////////////////////////////////////////////////
  install_t
  install()
  { 
    PL_register_foreign("getFoundObjectsROS", 1, (void *)pl_getFoundObjectsROS, 0);
  }
}
