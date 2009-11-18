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


#include "CheckColorClass.h"
#include "ColorClass.h"



#define XML_NODE_CHECKCOLORCLASS "CheckColorClass"
#define XML_ATTRIBUTE_MLPPATH "mlpPath"


#ifdef HALCONIMG

Halcon::HTuple Colors;
#include <cpp/HalconCpp.h>


using namespace cop;

// Procedure declarations
// Local procedures
void classify_colors_score (Halcon::Hobject Image, Halcon::Hobject Region, Halcon::Hobject *ClassRegions,
    Halcon::HTuple MLPHandle, Halcon::HTuple Colors, Halcon::HTuple *Color, Halcon::HTuple *Score);
void read_color_mlp (Halcon::HTuple Path, Halcon::HTuple *MLPHandle, Halcon::HTuple *Colors);
#endif

CheckColorClass::CheckColorClass(std::string path) :
  m_stPath(path),
  m_MLPHandle(-1)
{
#ifdef HALCONIMG
  printf("CheckColorClass\n");
  Halcon::HTuple mlp;
  try
  {
  read_color_mlp(m_stPath.c_str(), &mlp, &Colors);
  }
  catch(Halcon::HException ex)
  {
    printf("Error in loading mlp: %s\n", ex.message);
    throw "Error loading mlp";
  }
  printf("MLPHandle: %d\n", mlp[0].I());
  m_MLPHandle = mlp[0].I();
  #endif
}

#ifdef HALCONIMG

void classify_colors_score (Halcon::Hobject Image, Halcon::Hobject Region, Halcon::Hobject *ClassRegions,
    Halcon::HTuple MLPHandle, Halcon::HTuple Colors, Halcon::HTuple *Color, Halcon::HTuple *Score)
{
  using namespace Halcon;

  // Local iconic variables
  Hobject  ImageReduced, RegionTemp;

  // Local control variables
  HTuple  Number, Area, Row, Column, Indices, Area1;
  HTuple  Row1, Column1, Sum, WindowHandle;
  area_center(Region, &Area1, &Row, &Column);
  erosion_circle(Region, &RegionTemp, sqrt((double)Area1[0].I()) / 3);
  reduce_domain(Image, RegionTemp, &ImageReduced);

/*    open_window(200, 200, 400, 400, 0, "visible", "", &WindowHandle);
      set_part(WindowHandle, 0, 0, 1600, 1200);
      disp_obj(ImageReduced, WindowHandle);*/
  classify_image_class_mlp(ImageReduced, &(*ClassRegions), MLPHandle, 0.2);
  count_obj((*ClassRegions), &Number);
  area_center((*ClassRegions), &Area, &Row, &Column);
  for(int i = 0; i < Area.Num(); i++)
    printf("%d/%d ",  Area[i].I(), Area1[0].I());
  printf("\n");
//  cout <<"AreaTupleCheckColor: "<< Area.ToString("04d") << std::endl;
  tuple_sort_index(Area, &Indices);
  tuple_sum(Area, &Sum);
  if(Sum[0].I() == 0)
    (*Score) = 0.0;
  else
    (*Score) = (HTuple(Area[HTuple(Indices[Number-1])]).Real())/(Sum.Real());
  (*Color) = Colors[HTuple(Indices[Number-1])];
  return;
}

void read_color_mlp (Halcon::HTuple Path, Halcon::HTuple *MLPHandle, Halcon::HTuple *Colors)
{
  using namespace Halcon;
  read_class_mlp(Path+"color_classification_mlp.gmc", &(*MLPHandle));
  read_tuple(Path+"color_classification_mlp_colors.dat", &(*Colors));
  return;
}
#endif
CheckColorClass::CheckColorClass(XMLTag* tag)
{
  printf("Loading Algorithm CheckColorClass\n");
  m_stPath = tag->GetProperty(XML_ATTRIBUTE_MLPPATH);
#ifdef HALCONIMG
  Halcon::HTuple Colors, mlp;

  try
  {
    printf("\n\nLoading  %s \n\n", m_stPath.c_str());
    read_color_mlp(m_stPath.c_str(), &mlp, &Colors);
  }
  catch(Halcon::HException ex)
  {
    printf("Error in loading mlp: %s\n", ex.message);
    throw "Error loading mlp";
  }
  printf("Ready\n");
  m_MLPHandle = mlp[0].I();
#endif
}


CheckColorClass::~CheckColorClass(void)
{
#ifdef HALCONIMG

  using namespace Halcon;
  clear_all_class_mlp();
#endif
}


XMLTag* CheckColorClass::Save()
{
  XMLTag* tag = new XMLTag(XML_NODE_CHECKCOLORCLASS);
  tag->AddProperty(XML_ATTRIBUTE_MLPPATH, m_stPath);
  return tag;
}
#ifdef HALCONIMG
void CheckColorClass::Inner(Hobject *img, Hobject *region, std::string &color, double& qualityMeasure)
{
      Halcon::Hobject ClassRegions;
      Halcon::HTuple Color, Score;
      printf("Colors[0]:=%s\n", Colors[0].S());
      classify_colors_score(*img, *region, &ClassRegions, m_MLPHandle, Colors, &Color, &Score);
      qualityMeasure = Score[0].D();
      color = Color[0].S();
      printf("Class result %s\n", Color[0].S());
}
#endif
// Public attribute accessor methods
//
std::vector<RelPose*> CheckColorClass::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
#ifdef HALCONIMG
  int i = 0;
  std::vector<ColorClass*> possibleColors;
  std::vector<RelPose*> results;
  numOfObjects = 0;
  qualityMeasure = 0;
  Camera* cam = Camera::GetFirstCamera(sensors);
  while (true)
  {
    Elem *cl = object.GetElement(i++,DESCRIPTOR_COLORCLASS);
    if(cl != NULL)
    {
      possibleColors.push_back((ColorClass*)cl);
    }
    else
      break;
  }
  if(possibleColors.size() > 0 && cam != NULL)
  {
    Halcon::HTuple Color;
    Halcon::HTuple Score;
    Image* img = cam->GetImage(-1);
    RegionOI* region = ColorClass::GetRegion(pose, cam->m_relPose->m_uniqueID, &(cam->m_calibration));
    if(img != NULL && region != NULL && img->GetType() == HALCONIMAGE)
    {
      Halcon::Hobject* obj = img->GetHImage();
      Halcon::Hobject& reg = region->GetRegion();
      std::string st;
      Inner(obj, &reg, st, qualityMeasure);

      for(unsigned int iterc = 0; iterc < possibleColors.size(); iterc++)
      {
        printf(".\n");
        ColorClass* cl = possibleColors[iterc];
        printf("Color %s with %f\n", st.c_str(), qualityMeasure);
        if(cl->IsClass(st))
        {
          numOfObjects = 1;
          results.push_back(pose);
          break;
        }
      }
    }
    else
    {
      printf("Error: %p %p\n",img, region);
    }
    delete region;
  }

  return results;
#endif
}

double CheckColorClass::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
    if(object.GetElement(0,DESCRIPTOR_COLORCLASS) != NULL)
      return 1.0;
    else
      return 0.0;
}
