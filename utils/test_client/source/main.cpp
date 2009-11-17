/* 
 * Copyright (c) 2009, U.
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
#include <vision_msgs/cop_answer.h>
#include <stdio.h>

using namespace vision_srvs;
using namespace vision_msgs;

bool breaker = false;

 void callback(const boost::shared_ptr<const cop_answer> &msg)
 {
  printf("got answer from cop! (Errors: %s)\n", msg->error.c_str());
  for(int i = 0; i < msg->found_poses.size(); i++)
  {
    const aposteriori_position &pos =  msg->found_poses[i];
    printf("Foub Obj nr %d with prob %f at pos %d\n", (int)pos.objectId, pos.probability, (int)pos.position);
  }
  printf("End!\n");
  breaker = true;
 }

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "testclient") ;
  cop_call call;
  cop_answer answer;
  ros::NodeHandle n;

 /** Advertise the topic? should be subscibed already by cop*/
  std::string stTopicName = "/tracking/out";
  /** Create the cop_call msg*/
  call.request.outputtopic = stTopicName;
  call.request.object_classes.push_back(argc > 1 ? argv[1] : "Mug");
  call.request.action_type = argc > 4 ? atoi(argv[4]) : 0;
  call.request.number_of_objects = argc > 3 ? atoi(argv[3]): 1;
  apriori_position pos;
  pos.probability = 1.0;
  if(argc > 2)
  {
    pos.positionId = atoi(argv[2]);
    call.request.list_of_poses.push_back(pos);
  }
  /** subscribe to the topic cop should publish the results*/
  ros::Subscriber read = n.subscribe<cop_answer>(stTopicName, 1000, &callback);
  /** Publish */
  ros::service::waitForService("/tracking/in");
  ros::ServiceClient client = n.serviceClient<cop_call>("/tracking/in", true);
  if(!client.call(call))
  {
      printf("Error calling cop\n");
  }
  else
      printf("Called cop \n");
  ros::Rate r(100);
  while(n.ok() && !breaker)
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
