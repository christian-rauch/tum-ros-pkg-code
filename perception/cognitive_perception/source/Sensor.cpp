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



#include "XMLTag.h"
#include "Sensor.h"

#include "XMLTag.h"

#include <pluginlib/class_loader.h>

#ifdef BOOST_THREAD
#include "boost/bind.hpp"
#define BOOST(A) A
#else
#define BOOST(A)
#endif


using namespace cop;

Sensor::~Sensor()
{
  for(std::vector<Reading*>::iterator iter = m_images.begin(); iter != m_images.end(); iter++)
  {
    delete (*iter);
  }
  m_images.clear();
}

pluginlib::ClassLoader<Sensor> s_sens_loader("cognitive_perception", "Sensor");

Sensor* Sensor::SensorFactory(XMLTag* tag)
{
  Sensor* sens = NULL;

  try
  {
    sens = s_sens_loader.createClassInstance(tag->GetName());
    sens->SetData(tag);
    if(sens->Start())
      printf("Sensor of type %s started\n", sens->GetName().c_str());
    else
      printf("Error: Sensor of type %s did NOT start\n", sens->GetName().c_str());

  }
  catch(pluginlib::PluginlibException& ex)
  {
  //handle the class failing to load
    sens = NULL;
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    printf("Tag failed: %s\n", tag->GetName().c_str());

  }
	return sens;
}


bool Sensor::DeleteReading()
{
  bool ret = false;
//  static int count_occ = 0;
  BOOST(m_mutexImageList.lock());
  if(m_images.size() == 0)
    return true;
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


void Sensor::SetData(XMLTag* tag)
{
  printf("Entering set data\n");
  if(tag != NULL)
  {
    m_stSensorName = tag->GetProperty(XML_PROPERTY_SENSORNAME, "");
    XMLTag* pose_elem = tag->GetChild(XML_NODE_RELPOSE);
    if(pose_elem != NULL)
    {
      m_relPose = RelPoseFactory::FRelPose(pose_elem);
    }
    else
    {
      m_relPose = RelPoseFactory::FRelPoseWorld();
    }
  }
}


void Sensor::SaveTo(XMLTag* tag)
{
  if(m_relPose != NULL)
    tag->AddChild(m_relPose->Save());
}

Reading* Sensor::GetReading_Lock(size_t index)
{
  BOOST(m_mutexImageList.lock());

  if(index >= m_images.size())
  {
    printf("Camera skew asked %ldth frame which does not exist, will return %ld\n", index, m_images.size() - 1 );
    index = m_images.size() -1;
  }
  m_images[index]->Hold();
  m_images[index]->SetPose(m_relPose);
  BOOST(m_mutexImageList.unlock());

  return m_images[index];
}

void Sensor::PushBack(Reading* img)
{
  BOOST(m_mutexImageList.lock());
  m_images.push_back(img);
  m_FrameCount++;
  BOOST(m_mutexImageList.unlock());
}



//#endif
