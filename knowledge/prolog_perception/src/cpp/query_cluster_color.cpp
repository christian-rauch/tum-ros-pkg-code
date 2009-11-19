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
 * $Id: query_clusters_colors.cpp 17089 2009-06-15 18:52:12Z pangercic $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

@query_cluster_color is a middleware between (ros)prolog and ros nodes.
Compilled into a shared library it
in general:
a)accepts prolog foreign function calls and
b)invokes ros ServiceClient calls

and in particular:
c)queries COP (https://tum-ros-pkg.svn.sourceforge.net/svnroot/tum-ros-pkg/perception/cop/)
to locate clusters in PointCloud data and checks them against given colors (to the time
RGB and black and white are supported)
**/

#include <ros/ros.h>
#include <vision_srvs/cop_call.h>
#include <vision_msgs/cop_answer.h>

//prolog wrapper
#include <cstdlib>
#include <ctype.h>
#include <SWI-Prolog.h>
#include <vector>

//callback bind
#include <boost/bind.hpp>

extern "C"
{ 
  using namespace vision_srvs;
  using namespace vision_msgs;
  using namespace std;

  //cop answer format
  struct answer_st
  {
    int objId;
    double prob;
    int loId;
  };
  //storage for cop answers
  vector<answer_st> answer_vec;

  //if exit_cb == 1 -> query for clusters finished
  //if exit_cb == 2 -> query for colors finished
  int exit_cb = 0;

  void callback(vector<answer_st> *prob, const boost::shared_ptr<const cop_answer> &msg)
  {
    answer_st answer_tmp;
    prob->clear();
    ROS_DEBUG("got answer from cop! (Errors: %s)", msg->error.c_str());
    ROS_DEBUG("msg size %ld",  msg->found_poses.size());
    for(unsigned int i = 0; i < msg->found_poses.size(); i++)
      {
        const aposteriori_position &pos =  msg->found_poses[i];
        ROS_DEBUG("Found Obj (callback) nr %d with prob %f at pos %d, exit: %d", (int)pos.objectId, pos.probability, (int)pos.position, exit_cb);
        answer_tmp.objId =  (int)pos.objectId;
        answer_tmp.prob =  pos.probability;
        answer_tmp.loId =  (int)pos.position;
        prob->push_back(answer_tmp);
      }
    exit_cb++;
    ROS_DEBUG("End!");
  }

  foreign_t
  pl_getCopClustersColorROS(term_t l)    
  {
    int argc;
    argc=1;
    char **argv;
    argv = (char **)malloc(sizeof(char*));
    argv[0] = (char *)malloc(sizeof(char) *2);
    argv[0][0] = 'c';
    argv[0][1] = '\0';

    term_t tmp = PL_new_term_ref();
    
    ros::init(argc, argv, "testclient") ;
    cop_call call;
    cop_answer answer;
    ros::NodeHandle n;

    /** Advertise the topic? should be subscibed already by cop*/
    std::string stTopicName = "/tracking/out";
    /** Create the cop_call msg*/
    call.request.outputtopic = stTopicName;
    call.request.object_classes.push_back("Cluster");
    call.request.action_type = 0;
    call.request.number_of_objects = 5;
    apriori_position pos;
    pos.probability = 1.0;
   
    ros::Subscriber read = n.subscribe<cop_answer>(stTopicName, 1000, boost::bind(&callback, &answer_vec, _1));

    /** Publish */
    ros::service::waitForService("/tracking/in");
    ros::ServiceClient client = n.serviceClient<cop_call>("/tracking/in", true);
    //call for clusters
    if(!client.call(call))
      {
        ROS_DEBUG("Error calling cop\n");
      }
    else
      {
        ROS_DEBUG("Called cop \n");
      }
    
    ros::Rate r(100);
    while(n.ok() && exit_cb < 1)
    {
      ros::spinOnce();
      r.sleep();
    }
    //call for color
    call.request.object_classes.pop_back();
    //call.request.object_classes.push_back("red");
    call.request.object_classes.push_back("green");
    //call.request.object_classes.push_back("blue");
    //call.request.object_classes.push_back("white");
    //call.request.object_classes.push_back("black");
    ROS_DEBUG("vector size %ld",  answer_vec.size());

    for (unsigned int i =0; i < answer_vec.size(); i++)
      {
        pos.positionId = answer_vec[i].loId;
        call.request.list_of_poses.push_back(pos);
      }
    if(!client.call(call))
      {
        ROS_DEBUG("Error calling cop\n");
      }
    else
      {
        ROS_DEBUG("Called cop \n");
      }

     while(n.ok() && exit_cb < 2)
    {
      ros::spinOnce();
      r.sleep();
    }
     
     for (unsigned int i =0; i < answer_vec.size(); i++)
       {
         if(answer_vec[i].prob > 0.5)
           {
             ROS_DEBUG("Unifying object %d with prob %f at pos %d", answer_vec[i].objId,  answer_vec[i].prob,  answer_vec[i].loId);
             if (!PL_unify_list(l, tmp, l) ||
                 !PL_unify_float(tmp, answer_vec[i].prob))
               PL_fail;
           }
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
    PL_register_foreign("getCopClustersColorROS", 1, (void *)pl_getCopClustersColorROS, 0);
  }
}
