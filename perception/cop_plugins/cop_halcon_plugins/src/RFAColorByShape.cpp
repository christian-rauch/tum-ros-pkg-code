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

#include "RFAColorByShape.h"
#include "ColorClass.h"
/*#include "ShapeModel.h"*/
#include "CheckColorClass.h"
#include "Camera.h"

using namespace cop;
RFAColorByShape::RFAColorByShape()
{
}

void RFAColorByShape::SetData(XMLTag* tag)
{
  XMLTag* child = tag->GetChild(XML_NODE_CHECKCOLORCLASS);
  if(child == NULL)
    throw "Error loading RFAColorByShape";
  m_checkColor = (CheckColorClass*)LocateAlgorithm::LocAlgFactory(child);

}
RFAColorByShape::RFAColorByShape(CheckColorClass* checkColor) :
  m_checkColor(checkColor)
{
}


RFAColorByShape::~RFAColorByShape(void)
{
}

Descriptor* RFAColorByShape::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
	ColorClass* cm = NULL;
	Camera* cam = Camera::GetFirstCamera(sensors);
	if(cam != NULL)
	{
    if(cam->CanSee(*pose))
    {
      Image* img = cam->GetImage(-1);
      std::map<std::string, double> hist;
      RegionOI* region = new RegionOI(pose, cam->m_relPose->m_uniqueID, &(cam->m_calibration));
      std::string stColor;
	  /*(Hobject *img, Hobject *region, std::string &color, double& qualityMeasure)*/
      m_checkColor->Inner(img->GetHImage() , &region->GetRegion(), stColor, hist, qualityMeasure);
      if(qualityMeasure > 0.2)
        cm = new ColorClass(new Class(stColor, Elem::m_LastID), stColor, hist);
      else
        cm = new ColorClass(new Class("MultiColor", Elem::m_LastID), stColor, hist);
     /* Halcon::Hobject xld = sm->GetContour(*sig->m_relPose);
      int num = 0;
      Halcon::count_obj(xld, (Hlong*)&num);

      Halcon::Hobject region;*/
      /* Generate region for descriptor extraction*/
      /*Halcon::gen_empty_region(&region);
      for(int i = 0; i < num; i++)
      {
        Halcon::Hobject obj;
        Halcon::gen_region_contour_xld(xld, &obj ,"filled");
        Halcon::union2(obj, region, &region);
      }
      Halcon::connection(region, &region);
      Halcon::fill_up(region, &region);
      Halcon::Hobject* obj = img->GetHImage()
      CheckColorClass
      std::string color
      Inner(img->GetHImage(), &region, color, qualityMeasure);
      cm = new ColorClass(color);*/
    }

	}
	return cm;

}

double RFAColorByShape::CheckSignature(const Signature& sig, const  std::vector<Sensor*> &sens)
{
  if(sig.GetElement(0,DESCRIPTOR_COLORCLASS) == NULL)
    return 1.0;
  else
    return -0.0;
}

XMLTag* RFAColorByShape::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_COLORBYSHAPE);

	//TODO: parameter?
	return tag;
}

