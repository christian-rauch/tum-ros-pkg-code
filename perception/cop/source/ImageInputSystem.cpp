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
	}
}

ImageInputSystem::~ImageInputSystem ( ) { }

//
// Methods
//
void ImageInputSystem::TurnTo(RelPose& targetPose, unsigned int camera)
{
	if(m_cameras[camera] != NULL && m_cameras[camera]->CanSee(targetPose))
	{
		m_cameras[camera]->LookAt(targetPose);
	}
}

void ImageInputSystem::AddCamera(Camera* cam)
{
	m_cameras.push_back(cam);
}

Calibration& ImageInputSystem::GetCalibration(unsigned int camera)
{
	if(m_cameras.size() > camera && m_cameras[camera] != NULL)
	{
		return m_cameras[camera]->m_calibration;
	}
	throw "NoCamera";
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
 *	@return Image
 *	@param frameNumber
 *	@param camera
 */
Image* ImageInputSystem::GetImage (const int &frameNumber, const unsigned int& camera ) {
	return m_cameras[camera]->GetImage(frameNumber);
}

/**
 * GetBestCamera
 *  @brief Selected depening on the position that should be observed a camera and returns it.
 *	@return Camera
 *	@param  pose the pose that should be observed
 *  @param camera index will be returned
 *  @param offset specify this if the first camera is not good
 *	@param  camera retrieves the selected camera
 *   @throws char* with an error message in case of failure
 */
Camera* ImageInputSystem::GetBestCamera (RelPose &pose, unsigned int& camera, int offset)
{
  printf("ImageInputSystem::GetBestCamera\n");
	size_t nSize = m_cameras.size();
	int missed = 0;
	camera = 0;

	for(unsigned int i = 0; i < nSize; i++)
	{
	  if(m_cameras[i] == NULL)
      continue;
		if(m_cameras[i]->CanSee(pose))//TODO: choose best
		{
			if(offset == missed)
			{
				//TODO: Wait?
				camera = i;
				return	m_cameras[i];
			}
			else
				missed ++;
		}
		else
		{
				m_cameras[i]->LookAt(pose);
		}
	}
	if(offset > 0)
		return NULL;
	/* If no camera is available that sees the position, take the first*/
	if(nSize > 0)
		return m_cameras[0];
	throw "No Camera Instantiated";
}

