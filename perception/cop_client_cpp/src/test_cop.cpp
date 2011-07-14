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
#include <cop_client_cpp/cop_client.h>

#include <sstream>

int main(int argc, char* argv[])
{
  ros::init( argc, argv, "test_cop");
  ros::NodeHandle nh;
  CopClient cop(nh);
  ros::Rate r(100);
  bool refine = false;
  bool grasp = true;
  bool arm = true;

  unsigned long id_startpos = cop.LONameQuery("/base_link");
  id_startpos = cop.LOFrameQuery(id_startpos, 1);

  if(argc > 1)
    refine = true;
    
  unsigned long id_searchspace = cop.LONameQuery("/openni_rgb_optical_frame");
  if(id_searchspace == 0)
    id_searchspace = cop.LONameQuery("/narrow_stereo_optical_frame");
   std::string object = "Cluster";
  while(true)
  {
    long vision_primitive =  cop.CallCop(object, id_searchspace, 10);


  size_t num_results_expected = 1;


  std::vector<vision_msgs::cop_answer> results;
  while(nh.ok())
  {
    if(cop.HasResult(vision_primitive) >= num_results_expected)
      break;
    ros::spinOnce();
    r.sleep();
  }
  std::vector<vision_msgs::cop_answer> result = cop.GetResult (vision_primitive);
  int j = 0; 
  int i = 0;
  while(refine && result.size() > 0)
  {
    system("reset");
    printf("============================\n      Refine\n============================\n\n");
    for(j = 0; j < result[i].found_poses.size(); j++)
    {
      printf("%d: Refine %s (%ld at %ld)\n", j, result[i].found_poses[j].models[0].sem_class.c_str(),
                                                result[i].found_poses[j].objectId,
                                                result[i].found_poses[j].position);
     }
      
    printf("%d: %s\n Select cluster to refine: ", j, grasp ? "Continue to grasping": "End");
    int select = result.size();
    int test = scanf("%d", &select);
    printf("Selected %d\n", select);
    if((unsigned)select < result[i].found_poses.size())
    {
      printf("Refine of obj %ld , pos %ld\n", result[i].found_poses[select].objectId, result[i].found_poses[select].position);
      long vision_primitive =  cop.CallCop("",  result[i].found_poses[select].position, 1, result[i].found_poses[select].objectId, 256);
      printf("Wait for refine\n");
      while(nh.ok())
      {
        if(cop.HasResult(vision_primitive) >= num_results_expected)
          break;
        ros::spinOnce();
        r.sleep();
      }    
    }
    else
      break;

    system("reset");
    printf("============================\n      Redetect\n============================\n\n");
    long vision_primitive_ss =  cop.CallCop("", result[i].found_poses[select].position, 1, result[i].found_poses[select].objectId);
    size_t num_results_expected_ss = 1;


      std::vector<vision_msgs::cop_answer> results_ss;
      while(nh.ok())
      {
        if(cop.HasResult(vision_primitive_ss) >= num_results_expected_ss)
          break;
        ros::spinOnce();
        r.sleep();
      }
      results_ss = cop.GetResult (vision_primitive);
      for(j = 0; j < result[i].found_poses.size(); j++)
      {
        printf("%d: Found %s (%ld at %ld)\n", j, results_ss[0].found_poses[j].models[0].sem_class.c_str(),
                                                  results_ss[0].found_poses[j].objectId,
                                                  results_ss[0].found_poses[j].position);
       }
       int test10 = scanf("%d", &select);

  }
  int result_sys_im = 0;
  if(grasp)
  {
    std::string  arm_string = "right";
    if(!arm)
      arm_string = "left";
    system("reset");
    printf("============================\n      Grasp\n============================\n\n");
    for(j = 0; j < result[i].found_poses.size(); j++)
    {
      printf("%d: Grasp %s (%ld at %ld)\n", j, result[i].found_poses[j].models[0].sem_class.c_str(),
                                                result[i].found_poses[j].objectId,
                                                result[i].found_poses[j].position);
    }
    
    printf("%d: End\n Select cluster to grab: ", j);
    int select = result.size();
    int test = scanf("%d", &select);
    printf("Selected %d\n", select);
    if((unsigned)select < result[i].found_poses.size())
    {
        std::ostringstream os;
        os << "\"";
        for(int k = 0; k < result[i].found_poses.size(); k++)
        {
           if(k != select)
           {
             os << result[i].found_poses[k].position << " ";   
           }
                
                
        }
        os <<  "\"";
        
        char cmd[1024];
        if( result[i].found_poses.size() > 1)
          sprintf(cmd, "rosrun demo_scripts armhand -s %s -r %ld -o %s", arm_string.c_str(), result[i].found_poses[select].position, os.str().c_str());
        else
          sprintf(cmd, "rosrun demo_scripts armhand -s %s -r %ld", arm_string.c_str(), result[i].found_poses[select].position);         
        printf("Calling %s\n", cmd);
        result_sys_im = system(cmd);
        printf("\n\nSystem result: %d\n\n", result_sys_im);
        sleep(0.5);
        int result_sys;
        if(result_sys_im == 0 || 2560 == result_sys_im)
        {
          sprintf(cmd, "rosrun demo_scripts armhand -s %s -l 0.2", arm_string.c_str());
          result_sys = system(cmd);
          sleep(1.5);
         
          sprintf(cmd, "rosrun demo_scripts armhand -s %s -l -0.2", arm_string.c_str());
          result_sys = system(cmd);
          sleep(1.0);
          sprintf(cmd, "rosrun demo_scripts armhand -s %s -g open_relax", arm_string.c_str());
          result_sys = system(cmd);
          sleep(0.5);
          sprintf(cmd, "rosrun demo_scripts armhand -s %s -l 0.3", arm_string.c_str());
          result_sys = system(cmd);
          sleep(1.0);
        }
        sprintf(cmd, "rosrun demo_scripts armhand -s %s -p open", arm_string.c_str());
        result_sys = system(cmd);
        sleep(0.1);
    }
  }
  while(true)
  {
    system("reset");
    printf("============================\n      END\n============================\n\n");
    printf("\n\nResult of grasp was %d\n\n", result_sys_im);

    printf("(0) Home & continue\n(1) END \n(2) Toggle arm\n(3) Toggle refine\n(4) Toggle grasp\n(5) Set search space\n(6) Set search term\n(7)continue w/o move");
    
     int select2;
     int test = scanf("%d", &select2);
     printf("Selected %d\n", select2);
     if(select2 == 0)
     {
       char cmddrive[256];
       sprintf(cmddrive, "rosrun drive_at drive_at.py -l %ld", id_startpos);
       system(cmddrive);
       break;
     }
     else if(select2 == 1)
       exit(0);
     else if (select2 == 2)
        arm = !arm;
     else if(select2 == 3)
        refine = !refine;
     else if(select2 == 4)    
        grasp = !grasp;
     else if(select2 == 5)
     {
         char crash[200];
         test = scanf("%s", &crash);
         if(atoi(crash) != 0)
           id_searchspace = atoi(crash);
        else
        {
          unsigned long test = cop.LONameQuery(crash);
          if(test != 0)
            id_searchspace = test;
        }
     }
     else if(select2 == 6)
     {
       char crash[200];
       test = scanf("%s", &crash);
       object = crash;
     }
     else if (select2 == 7)
     {
      break;
     }
   }
  }
  return 0;
}
