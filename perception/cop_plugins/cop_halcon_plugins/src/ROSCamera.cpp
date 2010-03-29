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


#include "ROSCamera.h"
#include "XMLTag.h"


#include "cpp/HalconCpp.h"



#include "boost/bind.hpp"
#define BOOST(A) A


#define XML_ATTRIBUTE_CALIBFILE       "CalibFileName"
#define XML_ATTRIBUTE_TOPICNAME  "TopicName"
using namespace cop;

ROSCOPCamera::ROSCOPCamera()
{
}

void ROSCOPCamera::SetData( XMLTag* ConfigFile)
{
  Camera::SetData(ConfigFile);
  m_grabbing = (false);

  printf("Creating a ROSCOPCAMERA\n");
  try
  {
    if(ConfigFile != NULL)
    {
      try
      {
        XMLTag* tag = ConfigFile->GetChild(XML_NODE_RELPOSE);
        if(tag != NULL)
        {
          m_relPose = RelPoseFactory::FRelPose(tag);
          if(m_relPose != NULL)
          {
#ifdef _DEBUG
          printf("RC: Camera is localized at position %ld\n", m_relPose->m_uniqueID);
#endif
          }
#ifdef _DEBUG
          else
            printf("RC: Camera has no location\n");
#endif
        }
        else
        {
#ifdef _DEBUG
          printf("RC: Camera has no location\n");
#endif
        }
      }
      catch(...)
      {
        printf("RC: error Reading RelPose\n");
      }
      m_stImageTopic = ConfigFile->GetProperty(XML_ATTRIBUTE_TOPICNAME, "image");
      m_stCalibName = ConfigFile->GetProperty(XML_ATTRIBUTE_CALIBFILE);
      if(m_stCalibName.length() > 0)
      {
        ReadCamParam(m_stCalibName);
      }
    }
  }
  catch(char const* text)
  {
    printf("Error Loading CameraDriver: %s\n", text);
  }
}

ROSCOPCamera::~ROSCOPCamera()
{
}

bool ROSCOPCamera::Start()
{
  ros::NodeHandle node;
  printf("Subscribing sensor %s at topic %s\n", GetSensorID().c_str(), m_stImageTopic.c_str());
  m_subsciber = node.subscribe(m_stImageTopic, 1, &ROSCOPCamera::ImageCallback, this);
  m_grabbing = true;
  return true;
}

bool ROSCOPCamera::Stop()
{
  printf("Sleeping a while and ");
  boost::xtime t;
  boost::xtime_get(&t, boost::TIME_UTC);
  t.sec += 1;
  boost::thread::sleep(t);
  m_grabbing = false;
  return true;
}

//
// Methods
//
Reading* ROSCOPCamera::GetReading(const long &Frame)
{
  printf("ROSCOPCamera::GetImage (Grabbing %s, Images %ld )\n", m_grabbing ? "true" : "false", m_images.size());
  if((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0)
  {
    if(m_grabbing)
    {
      while(m_grabbing && ((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0))
      {
        BOOST(printf("Sleeping a while and "));
        BOOST(boost::xtime t);
        BOOST(boost::xtime_get(&t, boost::TIME_UTC));
        BOOST(t.sec += 1);
        BOOST(boost::thread::sleep(t));

        printf("waiting for %s to start grabbing(Grabbing: %s, NumImages: %ld)\n", GetSensorID().c_str(), m_grabbing ? "true" : false, m_images.size());
      }
      printf("Got a new image: %d\n", (int)m_images.size());
    }
    if((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0)
    {
      printf("ROSCOPCamera: unexspected error\n");
      throw "Asking for images from a camera that has no images";
    }
  }
  if(Frame == -1 || (Frame - m_deletedOffset < 0 && (unsigned)(Frame - m_deletedOffset) >= m_images.size()))
  {
    return GetReading_Lock(m_images.size() -1);
    /*return m_images[m_images.size() -1];*/
  }
  return GetReading_Lock(Frame - m_deletedOffset);
  /*return m_images[Frame - m_deletedOffset];*/
}

XMLTag* ROSCOPCamera::Save()
{
  XMLTag* tag = new XMLTag(GetName());

  Camera::SaveTo(tag);
  if(tag != NULL)
  {
    tag->AddProperty(XML_ATTRIBUTE_CALIBFILE, m_stCalibName);
    tag->AddProperty(XML_ATTRIBUTE_TOPICNAME, m_stImageTopic);
  }

  return tag;
}

void ReadImage(const sensor_msgs::ImageConstPtr& m, Halcon::Hobject* obj)
{
  int height = m->height;
  int width = m->width;
  int channels = m->encoding.substr(0,4).compare("Mono") == 0 ? 1 : 3;
  Halcon::Hobject r, g, b;
  if(channels == 3)
    Halcon::gen_image_interleaved(obj, (Hlong)&(m->data[0]), "rgb", width, height, 1, "byte", width, height, 0,0,-1,0);
  else
  {
  for(int c = 0; c < channels; c++)
  {
    try
    {

    Halcon::HTuple pointer, emp;
    switch(c)
    {
      case 0:
        Halcon::gen_image_const(&r, "byte", width, height);
        Halcon::get_image_pointer1(r, &pointer,  &emp, &emp, &emp);
        break;
      case 1:
        Halcon::gen_image_const(&g, "byte", width, height);
        Halcon::get_image_pointer1(g, &pointer,  &emp, &emp, &emp);
        break;
      default:
        Halcon::gen_image_const(&b, "byte", width, height);
        Halcon::get_image_pointer1(b, &pointer,  &emp, &emp, &emp);
        break;
    }
    char* img_ptr = (char*)pointer[0].L();
    for(int row = 0; row < height; row++)
    {
      for(int col = 0; col < width; col++)
      {
        img_ptr[row * width + col] = m->data[row * width * channels + col * channels + c];
      }
    }
    }
    catch(...)
    {
      printf("Big error readiong from roscamera\n");
    }
  }
  if(channels == 3)
     Halcon::compose3(r, g, b, obj);
  }

}


  void ROSCOPCamera::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {

    Halcon::Hobject *obj = new Halcon::Hobject();
    printf("In callback of sensor %s\n", GetSensorID().c_str());
    ReadImage(msg, obj);
    if(false && m_calibration.m_radialDistortionHandling)
    {

      Halcon::HTuple width, height, p, t;
      Halcon::get_image_pointer1(*obj,&p, &t, &width, &height);
      if(width[0].I() == m_calibration.m_width)
        Halcon::map_image(*obj, *m_calibration.m_radialDistMap, obj);
    }
    Image* img = new Image(obj, RGB_IMAGE);
    PushBack(img);

    while(m_images.size() > 3)
    {
      if(DeleteReading())
        continue;
      else
      {
        printf("Ros Camera: Could not delete an image!\n");
        break;
      }
    }
  }


