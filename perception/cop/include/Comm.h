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

 
#ifndef COMM_H
#define COMM_H


#define STD_LO_RPC_PORT_INTERNAL "/located_object"
#define XML_PROPERTY_LO_RPC_PORT "LO_SERVICE_PORT"


class RelPose;
class Matrix;

class Comm
{
public:
  virtual void NotifyPoseUpdate(RelPose* /*pose*/, bool /*sendObjectRelation = true*/)=0;/*{throw "Comm NotifyPoseUpdate: Not implemented";}*/
  virtual RelPose* CreateNewPose(RelPose* /*pose*/, Matrix* /*mat*/, Matrix* /*cov*/){throw "Comm CreateNewPose: Not implemented";}
  virtual RelPose* GetPose(int /*poseId*/){throw "Comm GetPose: Not implemented";}
  virtual RelPose* GetPoseRelative(int /*poseId*/, int /*parentPoseId*/){throw "Comm GetPoseRelative: Not implemented";}
  virtual void FreePose(int /*poseID*/){}
};

#endif /**COMM_H*/

