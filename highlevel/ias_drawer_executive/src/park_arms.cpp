/*
 * Copyright (c) 2010, Dejan Pangercic <pangercic@cs.tum.edu>
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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <ias_drawer_executive/OpenContainerAction.h>
//#include <ias_drawer_executive/CloseContainerAction.h>
#include <ias_drawer_executive/OperateHandleController.h>
// #include <ias_drawer_executive/RobotDriver.h>
// #include <ias_drawer_executive/RobotArm.h>
// #include <ias_drawer_executive/Torso.h>
// #include <ias_drawer_executive/Perception3d.h>

//for trajectory publishing
//#include <visualization_msgs/MarkerArray.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "park_arms");
  if (argc != 2)
    {
      ROS_ERROR("Usage %s <[l|h] (park arms l=low, h=high)>", argv[0]);
      exit(0);
    }    

  if (std::string(argv[1]) == "h")
    OperateHandleController::plateAttackPose();
  else if (std::string(argv[1]) == "l")
    OperateHandleController::plateTuckPose();
  else
    ROS_WARN("[park_arms:] Unknown option %s.", argv[1]);
  return 0;
}


/*
*/
