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
                        Descriptor.cpp - Copyright klank


**************************************************************************/

#include "Descriptor.h"
#include "XMLTag.h"


#define XML_ATTRIBUTE_QUALITY "DescriptorQuality"

// Constructors/Destructors
//
Descriptor::Descriptor (Class *classref ) :
	m_class(classref),
	m_imgLastMatchImage(NULL),
	m_poseLastMatchImage(NULL),
	m_qualityMeasure(1.0)
{
}

Descriptor::Descriptor ( XMLTag* tag) :
	Elem(tag),
	m_imgLastMatchImage(NULL),
	m_poseLastMatchImage(NULL)
{
	m_class = Class::ClassFactory(tag->GetChild(0));
	m_qualityMeasure = tag->GetPropertyDouble(XML_ATTRIBUTE_QUALITY, -1.0);

	XMLTag* img = tag->GetChild(XML_NODE_IMAGEFILE);
	if(img != NULL)
	{
		try
		{
			try
      {
			  m_imgLastMatchImage = new Image(img);
      }
      catch(...)
      {
          printf("Error reading XML: Last matched image of a descriptor could not be found\n");
      }
			XMLTag* tagPose = tag->GetChild(XML_NODE_RELPOSE);
			if(tagPose != NULL)
				m_poseLastMatchImage = RelPoseFactory::FRelPose(tagPose);
			else
			{
				delete m_imgLastMatchImage;
				m_imgLastMatchImage = NULL;
			}

		}
		catch(char const* ch)
		{
			delete m_imgLastMatchImage;
			m_imgLastMatchImage = NULL;
			printf("Error reading XML: %s\n", ch);
		}
	}
}

Class* Descriptor::GetClass()
{
	return m_class;
}

Descriptor::~Descriptor ( )
{
	if(m_imgLastMatchImage != NULL)
	{
		m_imgLastMatchImage->Free();
		if(m_imgLastMatchImage->m_usageCount < 0)
			delete m_imgLastMatchImage;
		//RelPoseFactory::FreeRelPose(m_poseLastMatchImage);
    delete m_poseLastMatchImage;
	}
}

//
// Methods
//
void Descriptor::SaveTo(XMLTag* tag)
{
	tag->AddProperty(XML_ATTRIBUTE_QUALITY, m_qualityMeasure);
	if(m_imgLastMatchImage != NULL)
	{
		tag->AddChild(m_imgLastMatchImage->Save());
		if(m_poseLastMatchImage != NULL)
			tag->AddChild(m_poseLastMatchImage->Save());
	}
	if(m_class != NULL)
		tag->AddChild(m_class->Save(), 0);
	else
		printf("Trying to save a broken Descriptor\n");
}

void Descriptor::SetLastMatchedImage(Image* img, RelPose* pose)
{
	m_imgLastMatchImage = img->Clone();
	delete m_poseLastMatchImage;
  try
  {
	   m_poseLastMatchImage = RelPoseFactory::CloneRelPose(pose);
  }
  catch(...)
  {
    printf("Tying to copy a singular position\n");
    m_poseLastMatchImage = RelPoseFactory::FRelPoseWorld();
  }
	this->Touch();}


