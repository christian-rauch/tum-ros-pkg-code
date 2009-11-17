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

 
#ifndef VM_INTERFACEFCNS_H
#define VM_INTERFACEFCNS_H

#include "cpp/HalconCpp.h"
#include "cpp/HIOStream.h"
#include <lo/lo.h>
#include <yarp/os/all.h>
#include "VM_RPLCommand.h"

extern bool g_calibTAB;

void SendPoseToProlog(Halcon::HTuple ObjectPose, Halcon::HTuple Object, Halcon::HTuple DomAxis, yarp::os::BufferedPort < yarp::os::Bottle > & to_prolog, std::string & ImgFName, yarp::os::Bottle &CopsOrigAns);

yarp::os::Bottle talkToCop(yarp::os::Bottle * lo_query, char *Object, int NumOfObjects);

std::vector<RPLCommand> GetCommand(yarp::os::BufferedPort <yarp::os::Bottle> & RPL_in);

#endif // VM_INTERFACEFCNS_H
