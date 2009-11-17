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
#include <vision_srvs/srvjlo.h>
#include <stdio.h>

using namespace vision_srvs;


#define JLO_IDQUERY "idquery"
#define JLO_FRAMEQUERY "framequery"
#define JLO_DELETE "del"
#define JLO_UPDATE "update"
#define ID_WORLD 1

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "testclient") ;
   srvjlo msg;

  msg.request.command = JLO_UPDATE;
  if(argc > 2)
    msg.request.query.parent_id = atoi(argv[2]);
  else
    msg.request.query.parent_id = 1;
  if(argc > 1)
    msg.request.query.id = atoi(argv[1]);
  else
    msg.request.query.id = 864;
  if(argc > 15)
    msg.request.query.name = argv[15];
  msg.request.query.type = 0;  
  int width = 4;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
      if(r == c)
        msg.request.query.pose[r * width + c] = 1;
      else
        msg.request.query.pose[r * width + c] = 0;
    }
  }
/*  msg.request.query.pose[3] = 0.0;
  msg.request.query.pose[7] = 0.0;
  msg.request.query.pose[11] = 0.4;*/


/*-0.874767 0.480757 -0.060465 0.050978
-0.296898 -0.433193 0.850997 0.188964
0.382929 0.762375 0.521679 0.777142
0.000000 0.000000 0.000000 1.000000*/
if(argc < 15)
{
  msg.request.query.pose[0] =  0.874767;
  msg.request.query.pose[1] =  0.480757;
  msg.request.query.pose[2] =  0.060465;
  msg.request.query.pose[3] =  0.050978;
  msg.request.query.pose[4] =  0.296898;
  msg.request.query.pose[5] =   -0.433193;
  msg.request.query.pose[6] =  -0.850997;
  msg.request.query.pose[7] =  0.188964; 
  msg.request.query.pose[8] =  -0.382929;
  msg.request.query.pose[9] =  0.762375;
  msg.request.query.pose[10] = -0.521679;
  msg.request.query.pose[11] =  0.777142;
}
else
{
   int i = 0;
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //1
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //2
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++;  //3
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //4  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //5  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //6  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //7  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //8  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //9   
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //10  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++;  //11
   msg.request.query.pose[i] = atof(argv[i + 3]);
}

  printf("Showing Query with PosId %d with parent %d:\n", (int)msg.request.query.id, (int)msg.request.query.parent_id);
  
    for(int r = 0; r < width; r++)
    {
       for(int c = 0; c < width; c++)
       {
         printf( "%f ", msg.request.query.pose[r * width + c]);
       }
       printf("\n");
    }

  width = 6;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
      if(r == c)
      {
       if(r == 5)
       {
         msg.request.query.cov[r * width + c] = 6.0;
        }
        else
        msg.request.query.cov[r * width + c] = 0.10;
      }
      else
        msg.request.query.cov[r * width + c] = 0;
    }
  }

  /*msg.request.command = JLO_IDQUERY;
  if(argc > 2)
    msg.request.query.parent_id = atoi(argv[2]);
  if(argc > 1)
    msg.request.query.id = atoi(argv[1]);
  else msg.request.query.id = ID_WORLD;*/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<srvjlo>("/located_object", true);
  if (!client.call(msg))
  {
    printf("Error in ROSjloComm: Update of pose information not psossible!\n");
    return NULL;
  }
  else if (msg.response.error.length() > 0)
  {
    printf("Error from jlo: %s!\n", msg.response.error.c_str());
  }
  int width2 = 4;
  printf("Showing PosId %d with parent %d:\n", (int)msg.response.answer.id, (int)msg.response.answer.parent_id);
  
  for(int r = 0; r < width2; r++)
  {
    for(int c = 0; c < width2; c++)
    {
        printf( "%f ", msg.response.answer.pose[r * width2 + c]);

    }
    printf("\n");
  }
  printf("Type: %d\n", msg.response.answer.type);

  return 0;
}
