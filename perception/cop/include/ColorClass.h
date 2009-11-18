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


/************************************************************************
ColorClass.h - Copyright klank
**************************************************************************/




#ifndef COLORCLASS_H
#define COLORCLASS_H
#include "Descriptor.h"
#include "XMLTag.h"
#include "RegionOI.h"
#include "SearchParams3d.h"
#include "Camera.h"

#define XML_NODE_COLORCLASS "ColorClass"
#define XML_ATTRIBUTE_COLORNAME "ColorName"
namespace cop
{
  class ColorClass : public Descriptor
  {
  public:
    ColorClass(Class* cl, std::string colorName) : Descriptor(cl), m_stColorName(colorName)
    {};
    ColorClass(XMLTag* tag) : Descriptor(tag)
    {
        m_stColorName = tag->GetProperty(XML_ATTRIBUTE_COLORNAME);
    };


    static RegionOI* GetRegion(RelPose* pose, int cam_pose_id, Calibration* calib)
    {
      RegionOI* region = new RegionOI();
  #ifdef HALCONIMG
      Halcon::HTuple pose_s(7,0.0), extents;
      bool cov = true;
      Matrix m,ExtremePoses;
      double gravPoint[3];
      gravPoint[0] = 0.0;
      gravPoint[1] = 0.0;
      gravPoint[2] = 0.0;
  #ifdef _DEBUG
      printf("ColorClass:Calculating new Search Ranges\n");
  #endif /*_DEBUG*/
      try
      {
        if(pose->m_uniqueID != ID_WORLD)
        {
          Matrix dings = pose->GetMatrix(cam_pose_id);
          if(dings.element(0,0) >= 0.9999 &&
             dings.element(1,1) >= 0.9999 &&
             dings.element(2,2) >= 0.9999 &&
             fabs(dings.element(0,3)) <= 0.00001 &&
             fabs(dings.element(1,3)) <= 0.00001 &&
             fabs(dings.element(2,3)) <= 0.00001)
          {
            printf("Identity transform, no sense to calc search spaces.\n");
            cov = false;
          }
          else
          {
            m = pose->GetCovarianceMatrix();
            double d = m.trace();
            if (d == 0.0)
            {
              printf("ColorClass::GetRegion: Ignoring covariances with trace = 0\n");
              return false;
            }
            else
            {
              m = pose->GetCovarianceMatrix();
            }
          }
          }
          else
            cov = false;
       }
       catch(...)
       {
         cov = false;
       }
       if(cov == true)
       {
         try
         {
           pose->GetPose(&pose_s, cam_pose_id);
           ExtremePoses = GetExtremePoses(m);
           extents=GetExtents(ExtremePoses.t(), pose_s, gravPoint, calib, region);
           printf("Assigned new region of size: %d\n", region->GetSize());
         }
         catch(...)
         {
           printf("Error in ColorClass GetRegion!\n");

         }
       }
  #endif /*HALCONIMG*/
       return region;
    }

    bool IsClass(std::string colorname)
    {
       printf("Compare %s with %s\n", m_stColorName.c_str(), colorname.c_str());
       return (m_stColorName.compare(colorname) == 0);
    }
    void SaveTo(XMLTag* tag)
    {
      Descriptor::SaveTo(tag);
      tag->AddProperty(XML_ATTRIBUTE_COLORNAME, m_stColorName);
    }

     virtual void Show(RelPose* pose, Camera* cam)
     {
      if(pose != NULL && cam != NULL)
      {
  #ifdef HALCONIMG
        Halcon::HWindow* hwin = cam->GetWindow();
        hwin->SetColor("green");
        Halcon::HTuple pose_ht;
        pose->GetPose(&pose_ht);
        Halcon::HTuple camparam = cam->m_calibration.CamParam();
        Halcon::HTuple r,c;
        Halcon::project_3d_point(pose_ht[0],pose_ht[1],pose_ht[2], camparam, &r,&c);
        Halcon::disp_cross(hwin->WindowHandle(), r, c, 15, 0);
  #endif /*HALCONIMG*/
      }
    }

     virtual std::string GetNodeName() const{return XML_NODE_COLORCLASS;}
     virtual int GetType() const{return DESCRIPTOR_COLORCLASS;}
  private:
    std::string m_stColorName;
  };
}
#endif /*COLORCLASS_H*/
