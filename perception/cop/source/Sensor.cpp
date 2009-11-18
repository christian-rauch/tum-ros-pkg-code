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
#include "Camera.h"
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

Sensor* Sensor::SensorFactory(XMLTag* tag)
{
  Sensor* sens = Camera::CamFactory(tag);
  sens->SetData(tag);
  if(sens == NULL)
	{
    pluginlib::ClassLoader<Sensor> alg_loader("cop", "Sensor");

    try
    {
      sens = alg_loader.createClassInstance(tag->GetName());
      sens->SetData(tag);
    }
    catch(pluginlib::PluginlibException& ex)
    {
    //handle the class failing to load
      printf("The plugin failed to load for some reason. Error: %s", ex.what());
    }
	}
	return sens;

  /** TODO: plugin*/
}


bool Sensor::DeleteReading()
{
  bool ret = false;
//  static int count_occ = 0;
#ifdef BOOST_1_35
  BOOST(m_mutexImageList.lock());
#else
  BOOST(locker::lock(m_mutexImageList));
#endif

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
#ifdef BOOST_1_35
  BOOST(m_mutexImageList.unlock());
#else
  BOOST(locker::unlock(m_mutexImageList));
#endif

  return ret;
}


void Sensor::SetData(XMLTag* tag)
{
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


void Sensor::SaveTo(XMLTag* tag)
{
  if(m_relPose != NULL)
    tag->AddChild(m_relPose->Save());
}

Reading* Sensor::GetReading_Lock(size_t index)
{
#ifdef BOOST_1_35
  BOOST(m_mutexImageList.lock());
#else
  BOOST(locker::lock(m_mutexImageList));
#endif

  if(index >= m_images.size())
  {
    printf("Camera skew asked %ldth frame which does not exist, will return %ld\n", index, m_images.size() - 1 );
    index = m_images.size() -1;
  }
  m_images[index]->Hold();
  m_images[index]->SetPose(m_relPose);
#ifdef BOOST_1_35
  BOOST(m_mutexImageList.unlock());
#else
  BOOST(locker::unlock(m_mutexImageList));
#endif

  return m_images[index];
}

void Sensor::PushBack(Reading* img)
{
#ifdef BOOST_1_35
  BOOST(m_mutexImageList.lock());
#else
  BOOST(locker::lock(m_mutexImageList));
#endif

  m_images.push_back(img);
  m_FrameCount++;
#ifdef BOOST_1_35
  BOOST(m_mutexImageList.unlock());
#else
  BOOST(locker::unlock(m_mutexImageList));
#endif

}



//#endif
