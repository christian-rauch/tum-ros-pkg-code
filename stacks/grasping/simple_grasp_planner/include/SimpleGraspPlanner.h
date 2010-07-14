/*
 * Copyright (c) 2008, U. Klank
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
 *
 */
#ifndef SIMPLE_GRASP_PLANNER
#define SIMPLE_GRASP_PLANNER

 #ifdef __cplusplus
 extern "C" {
 #endif


typedef struct
{
  double x;
  double y;
  double z;
}
Point3D;

typedef struct
{
  double sx; double sxy; double sxz;
  double syx; double sy; double syz;
  double szx; double szy; double sz;
}
CovariancePoint;

typedef struct
{
  double alpha;
  double beta;
  double delta_max;
}
HandConfig;

#define MAX_TESTED_POSES 1000

typedef struct 
{
  int length;
  int min_score_index;
  double     scores[MAX_TESTED_POSES];
  HandConfig configs[MAX_TESTED_POSES]; 
}
HandConfList;
int InitSGP(Point3D* hand_points, int num, double table_z);

void setTableHeight(double table_z);
double getTableHeight();

Point3D TransformPoint(Point3D hand, Point3D target, double alpha, double beta, double delta);

HandConfig GetGraspLM(Point3D* obstacle_list, CovariancePoint* obstacle_covs, int num,
                      double offset_rot_z_side, double offset_ros_z_top);

HandConfList GetGraspList(Point3D* obstacle_list, CovariancePoint* obstacle_covs, int num,
                      double offset_rot_z_side, double offset_ros_z_top);


 #ifdef __cplusplus
 }
 #endif
#endif