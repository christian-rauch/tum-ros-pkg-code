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


/*********************************************************************
*  Camera.cpp Copyright klank
*********************************************************************/


#include "CameraDriver.h"
#include "SimulatedCamera.h"

#ifdef USE_YARP_COMM
#include "NetworkCamera.h"
#else
#include "ROSCamera.h"
#endif /*USE_YARP _COMM*/


#include "XMLTag.h"
#ifdef HALCONIMG
#include "cpp/HalconCpp.h"

using namespace Halcon;
#endif

#ifdef BOOST_THREAD
#include <boost/thread/mutex.hpp>
#define BOOST(A) A
#else
#define BOOST(A) ;
#endif

using namespace cop;


// Constructors/Destructors
//

Camera* Camera::GetFirstCamera(const std::vector<Sensor*> &sensors)
{
  printf("Enter (%ld sensors)\n", sensors.size());
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    printf("Checking %s for beeing Camera = %s\n", (*it)->GetName().c_str(), (*it)->IsCamera() ? "true" : "false");
    if((*it)->IsCamera())
    {
      return (Camera*)(*it);
    }
  }
  return NULL;
}



Calibration::~Calibration()
{
#ifdef HALCONIMG
  if(m_radialDistortionHandling)
  {
    delete m_radialDistMap;
    delete m_camdist;
  }
#endif
}

#ifdef HALCONIMG
HTuple Calibration::CamParam() const
{
  if(m_radialDistortionHandling)
    return *m_camdist;
  else
  {
    HTuple par;
    tuple_gen_const(8, 0, &par);
    if(m_calibrationmatrix.size() < 6)
      throw "No Camera Matrix available for Projection\n";
    for(int i = 0; i < 6; i++)
      par[i] = m_calibrationmatrix[i];
    par[6] = m_width;
    par[7] = m_height;
    return par;
  }
}

HTuple Calibration::CamParam(int width, int height)
{
  if(m_radialDistortionHandling)
  {
    HTuple tup = *m_camdist;
    tup[4] = tup[4].D() * width / m_width;
    tup[5] = tup[5].D() * height / m_height;
    tup[6] = width;
    tup[7] = height;
    //TODO support width height
    return tup;
  }
  else
  {
    HTuple par;
    tuple_gen_const(8, 0, &par);
    if(m_calibrationmatrix.size() < 6)
      throw "No Camera Matrix available for Projection\n";
    for(int i = 0; i < 4; i++)
      par[i] = m_calibrationmatrix[i];
    par[4] = m_calibrationmatrix[4] * width / m_width;
    par[5] = m_calibrationmatrix[5] * height / m_height;
    par[6] = width;
    par[7] = height;
    return par;
  }
}

HTuple Calibration::CamParam(double scale_image)
{
  if(m_radialDistortionHandling)
  {
    HTuple tup = *m_camdist;
    tup[2] = tup[2].D() / scale_image;
    tup[3] = tup[3].D() / scale_image;
    tup[4] = tup[4].D() * scale_image;
    tup[5] = tup[5].D() * scale_image;
    tup[6] = tup[6].D() * scale_image;
    tup[7] = tup[7].D() * scale_image;
    //TODO support width height
    return tup;
  }
  else
  {
    HTuple par;
    tuple_gen_const(8, 0, &par);
    if(m_calibrationmatrix.size() < 6)
      throw "No Camera Matrix available for Projection\n";
    par[0] = m_calibrationmatrix[0];
    par[1] = m_calibrationmatrix[1];
    par[2] = m_calibrationmatrix[2] / scale_image;
    par[3] = m_calibrationmatrix[3] / scale_image;
    par[4] = m_calibrationmatrix[4] * scale_image;
    par[5] = m_calibrationmatrix[5] * scale_image;
    par[6] = m_width  * scale_image;
    par[7] = m_height * scale_image;
    return par;
  }
}


HTuple Calibration::CamMatrix()
{
  if(m_radialDistortionHandling || m_calibrationmatrix[1] == 0.0)
  {
    HTuple param = CamParam();
    HTuple matrix, height, width;
    Halcon::cam_par_to_cam_mat(param, &matrix, &height, &width);
    return matrix;
  }
  throw "Not yet implemented";
}
#endif
Calibration::Calibration(XMLTag* tag) :
  m_radialDistortionHandling(false)
{
  if(tag != NULL)
  {
    XMLTag* tagName = tag->GetChild(XML_NODE_CALIBRATION);
    if(tagName != NULL)
      m_calibrationmatrix = XMLTag::Load(tagName, &m_calibrationmatrix);
    tagName = tag->GetChild(XML_NODE_WIDTH);
    if(tagName != NULL)
      m_width = XMLTag::Load(tagName, &m_width);
    tagName = tag->GetChild(XML_NODE_HEIGHT);
    if(tagName != NULL)
      m_height = XMLTag::Load(tagName, &m_height);
    if(m_calibrationmatrix.size() != 0)
    {
    if(m_calibrationmatrix[1] != 0.0)
    {
      printf("Radial Distortion Handling necessary!\n");
#ifdef HALCONIMG
      Halcon::HTuple param = this->CamParam();
      m_radialDistortionHandling = true;
      m_radialDistMap = new Hobject();
      m_camdist = new HTuple();
      Halcon::change_radial_distortion_cam_par("fixed", param, 0.0, m_camdist);
      Halcon::gen_radial_distortion_map(m_radialDistMap, param, *m_camdist, "bilinear");
                        m_radialDistortionHandling = true;
#endif
    }
    }
  }
}

#ifdef HALCONIMG
void Calibration::SetCamParam(HTuple& param)
{
  m_calibrationmatrix.clear();
  if(param.Num() > 1 && param[1].D() != 0.0)
  {
    m_camdist = new HTuple();
    Halcon::change_radial_distortion_cam_par("fixed", param, 0.0, m_camdist);
    m_radialDistMap = new Hobject();
    Halcon::gen_radial_distortion_map(m_radialDistMap, param, *m_camdist, "bilinear");
                m_radialDistortionHandling = true;

  }
  for(int i = 0; i < param.Num() - 2; i++)
  {
    m_calibrationmatrix.push_back(param[i].D());
  }
  m_width = param[param.Num() - 2].I();
  m_height = param[param.Num() - 1].I();
}
#endif

void Calibration::SaveTo(XMLTag* tag)
{
  tag->AddChild(XMLTag::Tag(m_calibrationmatrix, XML_NODE_CALIBRATION));
  tag->AddChild(XMLTag::Tag(m_width, XML_NODE_WIDTH));
  tag->AddChild(XMLTag::Tag(m_height, XML_NODE_HEIGHT));
}


Camera::~Camera()
{
#ifdef HALCONIMG
  delete m_win;
#endif /*HALCONIMG*/
}


bool Camera::DeleteImg()
{
  bool ret = false;
//  static int count_occ = 0;
  BOOST(m_mutexImageList.lock());

  if((*m_images.begin())->m_usageCount == 0)
  {
    delete (*m_images.begin());
    m_images.erase(m_images.begin());
    m_deletedOffset++;
    ret = true;
  }
  else
  {
    m_temp_images.push_back(*m_images.begin());
    m_images.erase(m_images.begin());
    if((*m_images.begin())->m_usageCount == 0)
    {
      delete (*m_images.begin());
      m_images.erase(m_images.begin());
      m_deletedOffset++;
      ret = true;
    }
  }

  BOOST(m_mutexImageList.unlock());

  return ret;
}


void Camera::ReadCamParam(std::string filename)
{
#ifdef HALCONIMG
  HTuple t;
  try
  {
    Halcon::read_cam_par(filename.c_str(), &t);
    m_calibration.SetCamParam(t);
  }
  catch(Halcon::HException ex)
  {
    printf("Error reading Camera Param File: %s\n", ex.message);
  }
#endif
}

void Camera::SaveTo(XMLTag* tag)
{
  m_calibration.SaveTo(tag);
  Sensor::SaveTo(tag);
}

Camera::Camera()
{
}

void Camera::SetData(XMLTag* tag)
{
  Sensor::SetData(tag);
  m_calibration = Calibration(tag);
#ifdef HALCONIMG
  m_win = (NULL);
#endif
  if(m_relPose== NULL)
    m_relPose = RelPoseFactory::FRelPoseWorld();
};


#ifdef HALCONIMG
Halcon::HWindow* Camera::GetWindow()
{
  if(m_win == NULL)
  {
#ifdef _DEBUG
    printf("Creating a new window\n");
#endif
    m_win = new Halcon::HWindow(10,10,650,490, 0, "", "");
  }
  //m_win->SetShape("rectangle1");
  return m_win;
}
void Camera::DeleteWindow()
{
  delete m_win; m_win = NULL;
}
#endif

void Camera::WriteToFile(std::string fileName, const long& Frame)
{
#ifdef HALCONIMG
  printf("Write to %s\n", fileName.c_str());
  Image* img = GetImage(Frame);
  if(img != NULL)
  {
    Halcon::Hobject* obj = img->GetHImage();
    if(img->GetType() == YUV_IMAGE)
    {
      Halcon::Hobject img1, img2, img3, imgr, imgg, imgb;
      Halcon::decompose3(*obj, &img1, &img2, &img3);
      Halcon::trans_to_rgb(img1, img2, img3, &imgr, &imgg, &imgb, "yuv");
      Halcon::compose3(imgr, imgg, imgb, &img1);
      Halcon::write_image(img1, "jpg", 0, fileName.c_str());
    }
    else
      Halcon::write_image(*obj, "jpg", 0, fileName.c_str());
    img->Free();
  }
#endif
}

void Camera::Show(const long frame)
{
#ifdef HALCONIMG
  try
  {
    if(m_win == NULL)
      GetWindow();
    Image* img = GetImage(frame);
    if(img != NULL)
    {
      Halcon::Hobject* obj = img->GetHImage();
      Halcon::HTuple p,t,height, width, chan;
      Halcon::get_image_pointer1(*obj, &p,&t,&width, &height);

      //printf("Showing with: height %d width %d\n", height[0].I(), width[0].I());//
      m_win->SetWindowExtents(20,10,(width[0].I()/4 ) + 10, (height[0].I()/4 ) + 20);
      m_win->SetPart(0,0,height, width);
      //TODO Check if this is not colored!
      if(img->GetType() == YUV_IMAGE)
      {
        Halcon::Hobject img1, img2, img3, imgr, imgg, imgb;
        Halcon::decompose3(*obj, &img1, &img2, &img3);
        Halcon::trans_to_rgb(img1, img2, img3, &imgr, &imgg, &imgb, "yuv");
        Halcon::compose3(imgr, imgg, imgb, &img1);
        Halcon::disp_obj(img1, m_win->WindowHandle());
      }
      else if(img->GetType() == GRAY_DISPARITY_IMAGE)
      {
        Halcon::Hobject img1, img2, img3;
        Halcon::decompose3(*obj, &img1, &img2, &img3);
        Halcon::disp_obj(img1, m_win->WindowHandle());
      }
      else
        Halcon::disp_obj(*obj, m_win->WindowHandle());
      img->Free();
    }
    else{
      printf("Image from camera == NULL (%s)\n", GetName().c_str());
    }
  }
  catch(Halcon::HException ex)
  {
    printf("Showing not possible: %s \n", ex.message);
  }
  catch(...)
  {
     printf("Showing not possible! Unknown Exception\n");
  }
#endif
}


//#endif
