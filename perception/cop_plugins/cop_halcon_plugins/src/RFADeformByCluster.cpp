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

#include "RFADeformByCluster.h"
#include "Camera.h"
#include "RegionOI.h"
#include <algorithm>
#include "DeformShapeModel.h"

using namespace cop;

RFADeformByCluster::RFADeformByCluster(void)
{
}

RFADeformByCluster::RFADeformByCluster(XMLTag* tag)
{
}


RFADeformByCluster::~RFADeformByCluster(void)
{
}

Descriptor* RFADeformByCluster::Perform(std::vector<Sensor*> cam_vec, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
	DeformShapeModel* deformshape  = NULL;
  Camera *cam = NULL;


  for(size_t i = 0; i < cam_vec.size(); i++)
  {
    if(cam_vec[i] != NULL && cam_vec[i]->IsCamera())
    {
      cam = (Camera*)cam_vec[i];
      try
      {
        if(deformshape == NULL)
        {
          Class *cl = new Class();
          std::stringstream st;
          st << "Texture_" << cl->m_ID;
          cl->SetName(st.str());
          deformshape = new DeformShapeModel(cl);
        }
        Image* img = (Image*)cam->GetReading(-1);
        RegionOI region(pose, img->GetPose()->m_uniqueID, &(cam->m_calibration));
        try
        {
          qualityMeasure = deformshape->DefineDeformShapeModel(img, &(region.m_reg), cam, pose);
          deformshape->Evaluate(qualityMeasure, 100.0);
        }
        catch(char const* ex)
        {
          printf("Learning of Descriptorbased model Failed: %s\n", ex);
          delete deformshape;
          deformshape = NULL;
        }
        img->Free();
       }
       catch(char const* ex)
       {
         printf("Learning of Descriptorbased model Failed: %s\n", ex);
         if(deformshape != NULL)
         {
           delete deformshape;
         }
         deformshape = NULL;
       }
    }
  }
  return deformshape;
}

double RFADeformByCluster::CheckSignature(const Signature& sig, const std::vector<Sensor*> &sensors)
{

	if(sig.GetElement(0, DESCRIPTOR_DEFORMSHAPE) == NULL)
	{
			return 1.0;
	}
	return 0.0;
}

XMLTag* RFADeformByCluster::Save()
{
	XMLTag* tag = new XMLTag(GetName());

	//TODO: parameter?
	return tag;
}
