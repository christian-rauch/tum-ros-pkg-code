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
 * $Id: query_service_prolog.cpp 17089 2009-06-15 18:52:12Z pangercic $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

@query_plane is a middleware between (ros)prolog and ros nodes.
Compilled into a shared library it 
in general:
a)accepts prolog foreign function calls and
b)invokes ros ServiceClient calls

and in particular:
c)queries for and returns planes with their Ids and equation's coefficients.
**/



#include "ros/ros.h"
#include "ros/node_handle.h"
#include <mapping_srvs/GetPlaneClusters.h>
#include <cstdlib>
#include <ctype.h>
#include <SWI-Prolog.h>
extern "C"
{
  using namespace mapping_srvs;    

  foreign_t
  pl_getPlaneROS(term_t l)    
  {
    int argc;
    argc=1;
    char **argv;
    argv = (char **)malloc(sizeof(char*));
    argv[0] = (char *)malloc(sizeof(char) *2);
    argv[0][0] = 'b';
    argv[0][1] = '\0';
    ros::init(argc, argv, "query_service");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<GetPlaneClusters>("get_detect_planes_service");
    GetPlaneClusters srv;
    term_t tmp = PL_new_term_ref();
    if (client.call(srv))
      {
        //add response variables of choice
        if (!PL_unify_list(l, tmp, l) ||
            !PL_unify_integer(tmp, srv.response.planeId))
          PL_fail;
        ROS_DEBUG("Coeff a: %f", srv.response.a);
        if (!PL_unify_list(l, tmp, l) ||
            !PL_unify_float(tmp, srv.response.a))
          PL_fail;
        ROS_DEBUG("Coeff b: %f", srv.response.b);
        if (!PL_unify_list(l, tmp, l) ||
            !PL_unify_float(tmp, srv.response.b))
          PL_fail;
	ROS_DEBUG("Coeff c: %f", srv.response.c);
        if (!PL_unify_list(l, tmp, l) ||
            !PL_unify_float(tmp, srv.response.c))
          PL_fail;
        ROS_DEBUG("Coeff d: %f", srv.response.d);
        if (!PL_unify_list(l, tmp, l) ||
            !PL_unify_float(tmp, srv.response.d))
          PL_fail;
      
        return PL_unify_nil(l);
      }
    else
      {
        ROS_ERROR("Failed to call service get_detect_planes_service");
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
    PL_register_foreign("getPlaneROS", 1, (void *)pl_getPlaneROS, 0);
  }
}
