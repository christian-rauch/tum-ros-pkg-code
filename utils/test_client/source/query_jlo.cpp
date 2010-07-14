/*
 * Copyright (c) 2009, U. Klank klank@in.tum.de
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
#include <vision_srvs/srvjlo.h>
#include <stdio.h>
using namespace vision_srvs;

#define JLO_IDQUERY "idquery"
#define JLO_FRAMEQUERY "framequery"
#define JLO_DELETE "del"
#define JLO_UPDATE "update"
#define JLO_NAMEQUERY "namequery"
#define ID_WORLD 1

unsigned long ResolveName(ros::ServiceClient &client, const char* name_or_id)
{
  if (atoi(name_or_id) == 0 && strlen(name_or_id) > 1)
  {
    srvjlo msg;
    printf("Resolving name: %s ->", name_or_id);
    msg.request.command = "namequery";
    msg.request.query.name = name_or_id;
    if (!client.call(msg))
    {
       printf("Error calling jlo!\n");
    }
    if (msg.response.error.length() > 0)
    {
       printf("Error from jlo: %s!\n", msg.response.error.c_str());
       return 0;
    }
    printf("%ld\n", msg.response.answer.id);
    return msg.response.answer.id;
  }
  else
      return atoi(name_or_id);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "queryjlo") ;
  srvjlo msg;
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<srvjlo>("/located_object", true);

  msg.request.command = JLO_IDQUERY;
  if (argc > 3)
  {
     msg.request.command = "coutlo";
      msg.request.query.id = ResolveName(client, argv[1]);
      msg.request.query.parent_id = ResolveName(client, argv[2]);
  }
  else if(argc > 2)
  {
     if(strcmp(argv[1], "del") == 0)
     {
       msg.request.command = JLO_DELETE;
       msg.request.query.id = atoi(argv[2]);
     }
     else
     {
      msg.request.command = JLO_FRAMEQUERY;
      msg.request.query.id = ResolveName(client, argv[1]);
      msg.request.query.parent_id = ResolveName(client, argv[2]);
    }
  }
  else if(argc > 1)
  {
    if(atoi(argv[1]) == 0)
    {
       msg.request.command = JLO_NAMEQUERY;
       msg.request.query.name = argv[1];
    }
    else
      msg.request.query.id = ResolveName(client, argv[1]);

  }
  else
  {
    printf("Usage: ./query_jlo (ID [PARENT_ID]  |  NAME)\n");
    msg.request.query.id = ID_WORLD;
  }
  //printf("Asking command=%s with id=%ld, parent=%ld and name=%s\n", msg.request.command.c_str(), msg.request.query.id, msg.request.query.parent_id, msg.request.query.name.c_str());

  if (!client.call(msg))
  {
    printf("Error calling jlo!\n");
  }
  if (msg.response.error.length() > 0)
  {
    printf("Error from jlo: %s!\n", msg.response.error.c_str());
    return 0;
  }
  int width2 = 4;
  printf("Showing PosId %d (%s) with parent %d:\n", (int)msg.response.answer.id, msg.response.answer.name.c_str(), (int)msg.response.answer.parent_id);

  for(int r = 0; r < width2; r++)
  {
    for(int c = 0; c < width2; c++)
    {
        printf( "%f ", msg.response.answer.pose[r * width2 + c]);

    }
    printf("\n");
  }
  printf("Cov:\n");
  width2 = 6;
  for(int r = 0; r < width2; r++)
  {
    for(int c = 0; c < width2; c++)
    {
        printf( "%f ", msg.response.answer.cov[r * width2 + c]);

    }
    printf("\n");
  }
  printf("Type: %d\n", msg.response.answer.type);
  printf("pos dist: %f\n", sqrt(msg.response.answer.pose[3] * msg.response.answer.pose[3] + msg.response.answer.pose[7]*msg.response.answer.pose[7]+msg.response.answer.pose[11]*msg.response.answer.pose[11]) );
  return 0;
}
