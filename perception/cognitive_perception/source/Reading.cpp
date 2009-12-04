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


/*****************************************************************
                        Reading.cpp - Copyright klank

**************************************************************************/

#include "Reading.h"
#ifdef HALCONIMG
#include "cpp/HalconCpp.h"
#endif
#include "XMLTag.h"
#include "Image.h"
#include <sstream>

#include <pluginlib/class_loader.h>

using namespace cop;


Reading::~Reading()
{
  RelPoseFactory::FreeRelPose(m_relPose);
}

pluginlib::ClassLoader<Reading> s_reading_loader("cognitive_perception", "Reading");

Reading* Reading::ReadingFactory( XMLTag* tag)
{
  std::string name = tag->GetName();
  Reading* reading = NULL;

  try
  {
    reading = s_reading_loader.createClassInstance(name);
    reading->SetData(tag);
  }
  catch(pluginlib::PluginlibException& ex)
  {
  //handle the class failing to load
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    printf("Tag failed: %s\n", tag->GetName().c_str());

  }
  return reading;
}

void Reading::SetPose(RelPose* parent)
{
    m_relPose = RelPoseFactory::FRelPoseIdentityChild(parent);
}
