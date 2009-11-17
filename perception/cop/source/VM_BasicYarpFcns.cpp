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

 
#include "VM_BasicYarpFcns.h"

using namespace Halcon;
/********************************************************************
*     CreatePoseFromBottle                                         */
/********************************************************************
*   @brief Creates a lo from a bottle
*********************************************************************/
std::pair<RelPose*, double> CreatePoseFromBottle(yarp::os::Bottle* lo)
{
    std::pair<RelPose*, double> result;

    result.first = NULL;
    if(!lo->get(0).isDouble())
      return result;
    result.second = lo->get(0).asDouble();
    if(!lo->get(1).isInt())
      return result;
    /** Containing a parent id,*/
    int id = lo->get(1).asInt();
    RelPose* pose = RelPoseFactory::FRelPose(id);
    /** And two matrices */
    result.first = pose;
    return result; 
}

/********************************************************************
*     AddMatrixToABottle                                         */
/********************************************************************
*   @brief Adds a bottle containing a matrix
*********************************************************************/
void AddMatrixToABottle(int width, yarp::os::Bottle& lo, Matrix m)
{
  yarp::os::Bottle &mat_b = lo.addList();;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
        mat_b.addDouble(m.element(r,c));
    }
  }
}
/********************************************************************
*     PutPoseIntoABottle                                         */
/********************************************************************
*   @brief Creates a lo from a bottle
*********************************************************************/
void PutPoseIntoABottle(yarp::os::Bottle &lo, jlo::LocatedObject* pose)
{
  lo.addInt(pose->m_uniqueID);
  if(pose->m_uniqueID != ID_WORLD)
    lo.addInt(pose->m_parentID);
  else
    lo.addInt(1);
  Matrix m = pose->GetMatrix();
  /* Bottle per matrix*/
  int width = 4;
  AddMatrixToABottle(width, lo, m);
  width = 6;
  Matrix cov = pose->GetCovarianceMatrix();
  AddMatrixToABottle(width,lo,cov);
}

void PutPoseIntoABottle_rok(yarp::os::Bottle & lo, jlo::LocatedObject * pose)
{
    lo.addInt(pose->m_parentID);
    Matrix m = pose->GetMatrix();
    Matrix cov = pose->GetCovarianceMatrix();
    /* Bottle per matrix */
    int width = 4;
    yarp::os::Bottle & mat_b = lo.addList();;
    for (int r = 0; r < width; r++) {
	for (int c = 0; c < width; c++) {
	    mat_b.addDouble(m.element(r, c));
	}
    }

    width = 6;
    yarp::os::Bottle & cov_b = lo.addList();;
    for (int r = 0; r < width; r++) {
	for (int c = 0; c < width; c++) {
	    cov_b.addDouble(cov.element(r, c));
	}
	}
}

void ExtractBottle(yarp::os::Bottle bottle, HTuple *MeanMatTuple, string * ImageFileName)//,HTuple *CovMatTuple)

{
yarp::os::Bottle *covmat, *meanmat;
int ID=bottle.get(2).asInt();

RelPose* Pose=RelPoseFactory::FRelPose(ID);
Pose->GetHommat(MeanMatTuple,-1);
        if(bottle.size() > 2)
                *ImageFileName=bottle.get(3).asString();
        else 
                *ImageFileName="";
}
