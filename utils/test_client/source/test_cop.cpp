/* 
 * Copyright (c) 2009, U. Klank   klank@in.tum.de
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

#include <ros/ros.h>
#include <vision_srvs/cop_call.h>
#include <vision_srvs/srvjlo.h>
#include <vision_msgs/cop_answer.h>
#include <stdio.h>

using namespace vision_srvs;
using namespace vision_msgs;

bool breaker = false;
unsigned long vision_primitive = 0;

 void callback(const boost::shared_ptr<const cop_answer> &msg)
 {
  printf("got answer from cop! (Errors: %s)\n", msg->error.c_str());
  if(vision_primitive == msg->perception_primitive)
  {
  for(size_t i = 0; i < msg->found_poses.size(); i++)
  {
    const aposteriori_position &pos =  msg->found_poses[i];
    printf("Found Obj nr %d with prob %f at pos %d\n", (int)pos.objectId, pos.probability, (int)pos.position);
    for(size_t j = 0; j <  pos.models.size(); j++)
    {
      printf("  %s", pos.models[j].sem_class.c_str());
    }
    if(pos.models.size() > 0)
      printf("\n");
  }
  printf("End!\n");
  breaker = true;
  }
  else
  {
    printf("wrong vision primitive, continue waiting\n");
  }
 }

int main(int argc, char* argv[])
{
  if(argc == 1)
  {
    printf("Usage: \n%s ClassName LoID NumObjects Command\n\nClassName can be Cluster, Mug, IceTea, black, white, AssamBlend, ...\nLoID is a search space (use querylo to resolve tf-names to LoIDs)\nNumObjects specifies the maximum number of objects returned\nCommand Defaul is 0:=Locate 256:=Track 768:=Refine 2048:=StopTrack 25600:=LookUp, Show\n", argv[0]);
    return 0;
  }
  ros::init(argc, argv, "cop_testclient", ros::init_options::AnonymousName) ;
  cop_call call;
  cop_answer answer;
  ros::NodeHandle n;

 /** Advertise the topic? should be subscibed already by cop*/
  std::string stTopicName = "/tracking/cop_handler";
  /** Create the cop_call msg*/
  call.request.outputtopic = stTopicName;
  int index_of_args = 1;
  char name[75];
  if(argc > index_of_args)
  {
    if(strcmp("--name", argv[1]) == 0)
    {
       strcpy(name, argv[2]);
       index_of_args += 2;
    }
    else
    {
      strcpy(name, "cop");
    }
    if(atoi(argv[index_of_args]) == 0)
      call.request.object_classes.push_back(argv[index_of_args]);
    else
      call.request.object_ids.push_back(atoi(argv[index_of_args]));
  }
  else
    call.request.object_classes.push_back( "Mug");
  printf("Index of args %ld\n",index_of_args);
  call.request.action_type = argc > index_of_args+3 ? atoi(argv[index_of_args + 3]) : 0;
  if(argc > index_of_args+3 && call.request.action_type == 0 && strlen(argv[index_of_args + 3]) > 3)
  {
    if(strcmp("Refine", argv[index_of_args+3]) == 0)
    {
       call.request.action_type = 768;
    }
    else if(strcmp("Track", argv[index_of_args+3]) == 0)
    {
       call.request.action_type = 256;
    }
    else if(strcmp("StopTrack", argv[index_of_args+3]) == 0)
    {
       call.request.action_type = 2048;
    }
    else if(strcmp("Show", argv[index_of_args+3]) == 0 || strcmp("LookUp", argv[index_of_args+3]) == 0) 
    {
       call.request.action_type = 25600;
    }
  }
  if( argc > index_of_args+4)
  {
    call.request.object_classes.push_back(argv[index_of_args+4]);
  }
  call.request.number_of_objects = argc > index_of_args+2 ? atoi(argv[index_of_args+2]): 1;
  apriori_position pos;
  pos.probability = 1.0;
  if(argc > index_of_args+1)
  {
    pos.positionId = atoi(argv[index_of_args+1]);
    if( pos.positionId==0 && strlen(argv[index_of_args+1]) > 1)
    {
      vision_srvs::srvjlo msg;
      msg.request.command = "namequery";
      msg.request.query.name = argv[index_of_args+1];
      ros::ServiceClient client = n.serviceClient<srvjlo>("/located_object", true);
      if (!client.call(msg))
      {
         printf("Error calling jlo!\n");
      }  
      if (msg.response.error.length() > 0)
      {
         printf("Error from jlo: %s!\n", msg.response.error.c_str());
         return 0;
      }
       pos.positionId = msg.response.answer.id;
    }
    printf("Sending position ID : %ld\n",  pos.positionId );
    call.request.list_of_poses.push_back(pos);
    if(argc > index_of_args+5)
    {
      if(atoi(argv[index_of_args+3]) == 768)
      {
        for(int i = index_of_args+5; i < argc; i++)
        {
          call.request.object_classes.push_back(argv[i]);
        }
      }
      else
      {
        for(int i = index_of_args+5; i < argc; i++)
        {
          pos.positionId = atoi(argv[i]);
          call.request.list_of_poses.push_back(pos);
        }
      }
    }
  }
  /** subscribe to the topic cop should publish the results*/
  ros::Subscriber read = n.subscribe<cop_answer>(stTopicName, 1000, &callback);
  /** Publish */
  char srvname[80];
  sprintf(srvname, "/%s/in", name);
  ros::service::waitForService(srvname);
  ros::ServiceClient client = n.serviceClient<cop_call>(srvname, true);
  if(!client.call(call))
  {
      printf("Error calling cop\n");
      return 0;
  }
  else
      printf("Called cop \n");

  vision_primitive =  call.response.perception_primitive;

  ros::Rate r(100);
  while(n.ok() && !breaker)
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
