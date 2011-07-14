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
#include <std_msgs/String.h>
#include <sstream>

ros::Subscriber sr;

std::map<long, std::string> mapping;

void cb_logging(const std_msgs::StringConstPtr& str)
{
  long test = atoi((*str).data.c_str());
  if(test != 0)
  {
    mapping[test] = (*str).data;
  }
}

int main(int argc, char* argv[])
{
  ros::init( argc, argv, "test_cop");
  ros::NodeHandle nh;
  CopClient cop(nh);
  sr = nh.subscribe("/cop/logging", 1, cb_logging);
  

  ros::Rate r(100);
  bool refine = false;
  bool grasp = true;
  bool arm = true;
  bool cheap = true;
  double xoff = 0.0, yoff = 0.0, zoff = 0.0;

  unsigned long id_base_link = cop.LONameQuery("/base_link");
  unsigned long id_startpos = cop.LOFrameQuery(id_base_link, 1);
  unsigned long id_cam = cop.LONameQuery("/RightEyeCalc");
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
    if(result.size() > 0)
    {
      system("reset");
      printf("============================\n      Refine\n============================\n\n");
      for(j = 0; j < result[i].found_poses.size(); j++)
      {
        printf("%d: Refine %s (%ld at %ld)\n", j, result[i].found_poses[j].models[0].sem_class.c_str(),
                                                result[i].found_poses[j].objectId,
                                                result[i].found_poses[j].position);
        long vision_primitive =  cop.CallCop("",  result[i].found_poses[j].position, 1, result[i].found_poses[j].objectId, 256);
        printf("Wait for refine\n");
        while(nh.ok())
        {
          if(cop.HasResult(vision_primitive) >= num_results_expected)
            break;
          ros::spinOnce();
          r.sleep();
        }    
        std::vector<vision_msgs::cop_answer> result_tmp = cop.GetResult (vision_primitive);
        
        result.push_back(result_tmp[0]);
      }
    }
    for(int bla = 1 ; bla < 2 ; bla++)
    for (int trials = 0; trials < 20; trials++)
    {
     system("reset");
     printf("============================\n      Redetect \n============================\n\n");
     int i = 0;
     long vision_primitive_ss =  cop.CallCop(object, id_searchspace, 10);
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
      std::vector<unsigned long> vec ;
      
      for(i = 0; i < results_ss[0].found_poses.size(); i++)
      {
        vec.push_back(results_ss[0].found_poses[i].position);
      }
        
     for(i = 1; i < result.size(); i++)
     {
       for(j = 0; j < result[i].found_poses.size(); j++)
       {
         printf("%d: Redetect %s (%ld at %ld)\n", i, result[i].found_poses[j].models[result[i].found_poses[j].models.size() - 1].sem_class.c_str(),
                                                 result[i].found_poses[j].objectId,
                                                 result[i].found_poses[j].position);
                                               
       }
       int select = i;
       
       if(result.size() > select && result[select].found_poses.size() > 0)
       {
         long vision_primitive =  cop.CallCop("", vec, 1, result[select].found_poses[0].objectId, 0);
         printf("Wait for relocate\n");
         while(nh.ok())
         {
           if(cop.HasResult(vision_primitive) >= num_results_expected)
             break;
           ros::spinOnce();
           r.sleep();
         }    
         std::vector<vision_msgs::cop_answer> result_tmp = cop.GetResult (vision_primitive);
         char txt[512];
         bool succes = false;
         double dist = 0.0, dist_cam = 0.0;
         if(result_tmp.size() > 0 && result_tmp[0].found_poses.size() > 0)
         {
           dist = cop.LODistanceQuery(result[select].found_poses[0].position, result_tmp[0].found_poses[0].position);
           dist_cam = cop.LODistanceQuery(result[select].found_poses[0].position, id_cam);
           
           succes = true;
         }
         if(bla > 0)
         {
          if(dist < dist_cam*0.15)
          {
            cop.CopFeedBack(vision_primitive, 1.0, 0);           
          }
          else
          {
            cop.CopFeedBack(vision_primitive, 0.1, 2048);           
          }
         }
         std::string text;
         if(mapping.find(vision_primitive) != mapping.end())
          text = mapping[vision_primitive];
         sprintf(txt, "%ld %d %f %f %f %f %f %d %s\n", result[select].found_poses[0].objectId, succes ? 1 : 0, dist, xoff, yoff, zoff, dist_cam, bla, text.c_str());

         FILE *file = fopen("results.txt", "a");
         fwrite(txt, sizeof(*txt), strlen(txt), file);
         fclose(file);
       }
      }
/*   if(cheap)
        cheap = !cheap;*/
      while(true)
      {
        /* Move */ 
        xoff = 0.5 * ((double)rand() / RAND_MAX) - 0.25;
        yoff = 0.5 * ((double)rand() / RAND_MAX) - 0.25;
        
        if(xoff > 0.25)
          xoff = 0.25;
         if(xoff < -0.25)
          xoff = -0.25;  
         
        
        if(yoff > 0.25)
            yoff = 0.25; 
        if(yoff < -0.25)
           yoff = -0.25;        
        unsigned long newpos = cop.LOOffset(0, id_base_link, xoff, yoff, zoff);
        
        printf("GENERATED LO ID %ld\n\n\n", newpos);
        double dist = cop.LODistanceQuery(newpos, id_startpos);
        if(dist < 0.5)        
        {
          char cmddrive[256];
          sprintf(cmddrive, "rosrun drive_at drive_at.py -l %ld", newpos);
          printf(cmddrive);
          system(cmddrive);
          break;
        }
        else
        {
          printf("Its FAAAR away (dist = %f), ignore\n", dist);
        }
        cop.LOFreeID(newpos);
      }
         
    }

    while(true)
    {
    system("reset");
    printf("============================\n      END\n============================\n\n");

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
     else if(select2 == 2)
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
