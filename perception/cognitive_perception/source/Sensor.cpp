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
    if(sens != NULL)
    {
      printf("Calling sens (%p)->SetData(%s)\n", sens, sens->GetName().c_str());
      sens->SetData(tag);
      if(sens->Start())
        printf("Sensor of type %s started\n", sens->GetName().c_str());
      else
        printf("Error: Sensor of type %s did NOT start\n", sens->GetName().c_str());
    }
    else
    {
      printf("A sensor was not loaded :(\n");
    }
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

void Sensor::WaitForNewData()
{
 lock lk(m_mutexImageList);
 printf("Waiting for new data");
 m_newDataArrived.wait(lk);
 printf("Got new data\n");
}


bool Sensor::DeleteReading()
{
  bool ret = false;
//  static int count_occ = 0;
  lock lk(m_mutexImageList);
  try
  {
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
  }
  catch(...)
  {
    printf("Error deleting Reading in Sensor::DeleteReading\n");
  }
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
      if(m_relPose == NULL)
        printf("Error loading RelPose of Camera\n");
      else
        printf("Camera Located at %ld (%s)\n", m_relPose->m_uniqueID, m_relPose->m_mapstring.c_str());
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
  Reading* ret = NULL;
  RelPose* temp = RelPoseFactory::GetRelPose(m_relPose->m_mapstring);
  if(temp!= NULL && temp->m_uniqueID != m_relPose->m_uniqueID)
  {
    ROS_ERROR("jlo tree was corrupted: id of sensor changed: %s (%ld <- %ld)", m_relPose->m_mapstring.c_str(), temp->m_uniqueID, m_relPose->m_uniqueID);
    RelPoseFactory::FreeRelPose(m_relPose);
    m_relPose = temp;
  }
  else
  {
     RelPoseFactory::FreeRelPose(temp);
  }
  lock lk(m_mutexImageList);
  if(index >= m_images.size())
  {
    printf("Camera skew asked %ldth frame which does not exist, will return %ld\n", index, m_images.size() - 1 );
    index = m_images.size() -1;
    if( m_images.size() == 0)
    {
      throw "Sensor broken\n";
    }
  }
  ret = m_images[index];
  ret->Hold();
  return ret;
}

void Sensor::ProjectPoint3DToSensor(const double &x, const double &y, const double &z, double &row, double &column)
{
  MinimalCalibration(GetUnformatedCalibrationValues()).Project3DPoint(x,y,z,row, column) ;
}

void Sensor::PushBack(Reading* img)
{
  lock lk(m_mutexImageList);
  m_images.push_back(img);
  img->SetPose(m_relPose);
  m_FrameCount++;
  m_newDataArrived.notify_all();
}
#ifdef USE_YARP_COMM
/*
template <class SensorType, class MessageType>
void SensorNetworkRelay<SensorType, MessageType>::SetData(XMLTag* tag)
{
  SensorType::SetData(tag);
  m_stTopic = tag->GetProperty(XML_ATTRIBUTE_TOPICNAME);
  m_rate = tag->GetPropertyInt(XML_ATTRIBUTE_RATE);
  ros::NodeHandle nh;
  m_pub = nh.advertise<MessageType>(m_stTopic, 5);
}
*/
/**
*  Save all necessry Properties

template <class SensorType, class MessageType>
XMLTag* SensorNetworkRelay<SensorType, MessageType>::Save()
{
  XMLTag* tag = SensorType::Save();
  tag->SetName(GetName());
  tag->AddProperty(XML_ATTRIBUTE_TOPICNAME, m_stTopic);
  tag->AddProperty(XML_ATTRIBUTE_RATE, m_rate);
  return tag;
}
*/
#endif
