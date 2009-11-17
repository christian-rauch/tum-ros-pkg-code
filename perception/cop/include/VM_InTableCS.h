/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 
#ifndef VM_INTABLECS_H
#define VM_INTABLECS_H
#include "cpp/HalconCpp.h"
#include "cpp/HIOStream.h"
#include "SearchParams3d.h"

//#include "iostream"
//#include "math.h"
#include <cmath>
#include <lo/lo.h>
using namespace Halcon;

void obj_pose_cam2ref(const HTuple RefCSHMInCam, const HTuple ObjHMInCam, HTuple *ObjHMInRefCS);

void obj_pose_ref2cam(const HTuple RefCSPoseInCam, const HTuple ObjPoseInRefCS, HTuple *pObjPoseInCamCS);

void caltab2table_pose_incam(const HTuple &CaltabPose,HTuple *pTableCSHomMat);

HTuple get_estd_pose_and_covmatrix(HTuple TableCSPose, HTuple * pMeanPoseInCam,
HTuple InputMeanPoseInTable, HTuple RegionSize);

ColumnVector TransformPose(ColumnVector ExtremePoseInCS1, ColumnVector MeanPoseInCS1,HTuple CS1InCS2);

void tuple2mat(HTuple TupleIN,int type,Matrix * MatrixOUT);
void mat2tuple(Matrix MatrixIN,int type,HTuple * TupleOUT);

ReturnMatrix CreateCov (Matrix ExPoses,ColumnVector MeanPose);

ReturnMatrix Trans12Poses(Matrix EPInCS1,HTuple PoseCS1inCS2);

ReturnMatrix TableOffsetsToObjectOffsets(Matrix EposesInTable,HTuple MeanPoseInTable);

#endif //VM_INTABLECS_H
