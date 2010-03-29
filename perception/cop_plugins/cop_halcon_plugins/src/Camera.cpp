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
#include "cpp/HalconCpp.h"

using namespace Halcon;

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
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->IsCamera())
    {
      return (Camera*)(*it);
    }
  }
  return NULL;
}



Calibration::~Calibration()
{
  if(m_radialDistortionHandling)
  {
    delete m_radialDistMap;
    delete m_camdist;
  }
}

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
      Halcon::HTuple param = this->CamParam();
      m_radialDistortionHandling = true;
      m_radialDistMap = new Hobject();
      m_camdist = new HTuple();
      Halcon::change_radial_distortion_cam_par("fixed", param, 0.0, m_camdist);
      Halcon::gen_radial_distortion_map(m_radialDistMap, param, *m_camdist, "bilinear");
                        m_radialDistortionHandling = true;
    }
    }
  }
}

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

void Calibration::SaveTo(XMLTag* tag)
{
  tag->AddChild(XMLTag::Tag(m_calibrationmatrix, XML_NODE_CALIBRATION));
  tag->AddChild(XMLTag::Tag(m_width, XML_NODE_WIDTH));
  tag->AddChild(XMLTag::Tag(m_height, XML_NODE_HEIGHT));
}


Camera::~Camera()
{
  delete m_win;
}


void Camera::ReadCamParam(std::string filename)
{
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
  m_win = (NULL);
  if(m_relPose== NULL)
    m_relPose = RelPoseFactory::FRelPoseWorld();
};


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

void Camera::WriteToFile(std::string fileName, const long& Frame)
{
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
}

std::pair<std::string, std::vector<double> > Camera::GetUnformatedCalibrationValues()
{
  std::pair<std::string, std::vector<double> > ret;
  ret.first = "RECTHALCONCALIB";
  Halcon::HTuple tup = m_calibration.CamParam();
  ret.second.push_back( tup[0].D());
  ret.second.push_back( tup[2].D());
  ret.second.push_back( tup[3].D());
  ret.second.push_back( tup[4].D());
  ret.second.push_back( tup[5].D());
  return ret;
}


bool  Camera::CanSee (RelPose &pose) const
{
  if(m_relPose == NULL)
     return false;
#ifdef _DEBUG
  printf("Can a camera %s at Pose %ld See %ld\n",GetSensorID().c_str(),  m_relPose->m_uniqueID, pose.m_uniqueID);
#endif
  if(pose.m_uniqueID == m_relPose->m_uniqueID) /*lazy people just search in front of the camera, allow it*/
    return true;
  RelPose* pose_rel = RelPoseFactory::GetRelPose(pose.m_uniqueID, m_relPose->m_uniqueID);
  Matrix m = pose_rel->GetMatrix();
  RelPoseFactory::FreeRelPose(pose_rel);
  double x = m.element(0,3);
  double y = m.element(1,3);
  double z = m.element(2,3);
  if(z > 0.0)
  {

    try
    {
      Halcon::HTuple R, C;
      Halcon::project_3d_point(x,y,z, m_calibration.CamParam(), &R , &C);
      if(R >= 0 && R < m_calibration.m_height
        && C >= 0 && C < m_calibration.m_width)
        return true;
    }
    catch(Halcon::HException ex)
    {
      printf("Error: %s\n", ex.message);
    }
  }
  //TODO what when the position can be in the image, but not in the center?
#ifdef _DEBUG
  printf("A Camera can not see pose (rel to camera): %f %f %f\n", x,y,z);
#endif
  return false;
}

void Camera::Show(const long frame)
{
  try
  {
    printf("Try to show Camera %s\n", GetSensorID().c_str());
    if(m_win == NULL)
    {
      GetWindow();
      printf("Opened Window\n");
    }
    Image* img = GetImage(frame);
    if(img != NULL)
    {
      printf("Got image\n");

      Halcon::Hobject* obj = img->GetHImage();
      Halcon::HTuple p,t,height, width, chan;
      Halcon::get_image_pointer1(*obj, &p,&t,&width, &height);

      //printf("Showing with: height %d width %d\n", height[0].I(), width[0].I());//
      m_win->SetWindowExtents(20,10,(width[0].I() ) + 10, (height[0].I()) + 20);
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
      {
        printf("Try to show normal image\n");
        Halcon::disp_obj(*obj, m_win->WindowHandle());
      }
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
}


//#endif
