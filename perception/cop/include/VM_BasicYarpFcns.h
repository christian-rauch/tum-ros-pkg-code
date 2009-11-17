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

 
//using namespace Halcon;
#ifndef VM_BASICYARPFCNS_H
#define VM_BASICYARPFCNS_H

#include <lo/lo.h>
#include <yarp/os/all.h>
#include "cpp/HalconCpp.h"
#include "cpp/HIOStream.h"
#include "cpp/HObjectModel3D.h"
#include "RelPoseFactory.h"

void AddMatrixToABottle(int width, yarp::os::Bottle& lo, Matrix m);

void PutPoseIntoABottle(yarp::os::Bottle &lo, jlo::LocatedObject* pose);

void PutPoseIntoABottle_rok(yarp::os::Bottle & lo, jlo::LocatedObject * pose);

void ExtractBottle(yarp::os::Bottle bottle, Halcon::HTuple *MeanMatTuple, string * ImageFileName);

std::pair<RelPose*, double> CreatePoseFromBottle(yarp::os::Bottle* lo);

#endif //VM_BASICYARPFCNS_H
