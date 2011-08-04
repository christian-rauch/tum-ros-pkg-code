/*
 * Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>
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

#ifndef __OPERATEHANDLECONTROLLER_H__
#define __OPERATEHANDLECONTROLLER_H__

#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>


class RobotArm;
class Pressure;

class OperateHandleController {

  public:

  static tf::Stamped<tf::Pose> getBowlPose();
  static btVector3 getPlatePose();
  static btVector3 getTabletPose();

  //returns a handle that can be used to close the thing again,
  static int operateHandle(int arm, tf::Stamped<tf::Pose> aM, int numretry = 0);

  static int graspHandle(int arm_, tf::Stamped<tf::Pose> aM);


  static tf::Stamped<tf::Pose> getHandlePoseFromMarker(int arm_, int pos);

  // starting from a nice initial pose
  static void close(int side_c, int handle_);

  static void pickPlate(btVector3 plate, float width = 0.34);

  static void getPlate(int object, float zHint = 0);

  static void singleSidedPick(int side,tf::Stamped<tf::Pose> start, tf::Stamped<tf::Pose> end);

  static int maxHandle;

  static std::vector<std::vector<tf::Stamped<tf::Pose> *> > openingTraj;

  //static void spinnerL(float l, float back);
  static void spinnerL(float x, float y, float z);

  static void openGrippers();

  static void plateCarryPose();

  static void plateTuckPose();
  static void plateTuckPoseLeft();
  static void plateTuckPoseRight();

  static void plateAttackPose();
  static void plateAttackPoseLeft();
  static void plateAttackPoseRight();

};




#endif
