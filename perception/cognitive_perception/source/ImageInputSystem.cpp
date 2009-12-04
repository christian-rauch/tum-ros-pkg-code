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
                        ImageInputSystem.cpp - Copyright klank

**************************************************************************/

#include "ImageInputSystem.h"
#include "XMLTag.h"

#ifdef HALCONIMG
#include <cpp/HalconCpp.h>
#endif
#ifdef BOOST_THREAD
    /*printf("Sleeping\n");*/
void Sleeping(long ms)
 {
#ifdef BOOST_1_35
  BOOST(boost::system_time t);
#else
  boost::xtime t;
#endif

#ifdef BOOST_1_35
  BOOST(t = get_system_time());
  BOOST(t += boost::posix_time::seconds((double)ms / 1000));
#else
  boost::xtime_get(&t, boost::TIME_UTC);
  t.nsec += ms * 1000000;  //TODO Check
#endif
    boost::thread::sleep(t);
 }
#else
void Sleeping(long ms)
{
}
#endif
using namespace cop;


// Constructors/Destructors
//

ImageInputSystem::ImageInputSystem (XMLTag* configFile)
{
  if(configFile != NULL && configFile->CountChildren() > 0)
  {
#ifdef HALCONIMG
    Halcon::close_all_framegrabbers();
#endif
    m_cameras = XMLTag::Load(configFile->GetChild(0), &m_cameras);
    for(std::vector<Sensor*>::const_iterator it = m_cameras.begin();
      it != m_cameras.end(); it++)
     {
       if((*it) != NULL && (*it)->RequiresSensorList())
       {
         (*it)->SetSensorList(m_cameras);
       }
     }
    printf("Loaded %ld Cameras\n", m_cameras.size());
  }
}

ImageInputSystem::~ImageInputSystem ( ) { }


void ImageInputSystem::AddSensor(Sensor* sensor)
{
  m_cameras.push_back(sensor);

   for(std::vector<Sensor*>::const_iterator it = m_cameras.begin();
       it != m_cameras.end(); it++)
   {
     if((*it)->RequiresSensorList())
     {
        (*it)->SetSensorList(m_cameras);
     }
   }
}


// Accessor methods
//
XMLTag* ImageInputSystem::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_IMAGEINPUTSYSTEM);
	tag->AddChild(XMLTag::Tag(m_cameras));
	return tag;
}


/**
 * GetBestCamera
 *  @brief Selected depening on the position that should be observed a camera and returns it.
 *	@return Camera
 *	@param  pose the pose that should be observed
 *   @throws char* with an error message in case of failure
 */
std::vector<Sensor*> ImageInputSystem::GetBestSensor (RelPose &pose)
{
  printf("ImageInputSystem::GetBestSensor (%ld)\n", m_cameras.size());
	size_t nSize = m_cameras.size();
  std::vector<Sensor*> sensors_seeing;
	for(unsigned int i = 0; i < nSize; i++)
	{
	  printf("Can Camera %d(%p) see pose?\n", i, m_cameras[i]);
	  if(m_cameras[i] == NULL)
      continue;
		if(m_cameras[i]->CanSee(pose))//TODO: choose best
		{
			sensors_seeing.push_back(m_cameras[i]);
			printf("Yes\n");
		}
		else
      printf("No\n");
	}
	return sensors_seeing;
}

