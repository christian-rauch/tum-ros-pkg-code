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


#include "SegmentPrototype.h"
#include "XMLTag.h"


#define XML_ATTRIBUTE_RELFRAME   "RelFrame"
#define XML_ATTRIBUTE_PARALLEL "ParallelSearch"
#define XML_ATTRIBUTE_COVX       "CovX"
#define XML_ATTRIBUTE_COVY       "CovY"
#define XML_ATTRIBUTE_COVZ       "CovZ"

using namespace cop;


SegmentPrototype::SegmentPrototype() :
  m_relFrame("/base_link"),
  m_relPoseOfRefFrame(NULL),
  m_covRotX(0.2),
  m_covRotY(0.2),
  m_covRotZ(0.8),
  m_parallel(true)

{
}


void SegmentPrototype::SetData(XMLTag* tag)
{
  Descriptor::SetData(tag);
  m_relFrame =   tag->GetProperty(XML_ATTRIBUTE_RELFRAME, m_relFrame);
  m_parallel =   tag->GetPropertyInt(XML_ATTRIBUTE_PARALLEL, 0) == 0;
  UpdateRefFrame();
  m_covRotX =    tag->GetPropertyDouble(XML_ATTRIBUTE_COVX, m_covRotX);
  m_covRotY =    tag->GetPropertyDouble(XML_ATTRIBUTE_COVY, m_covRotY);
  m_covRotZ =    tag->GetPropertyDouble(XML_ATTRIBUTE_COVZ, m_covRotZ);

}

unsigned long SegmentPrototype::GetFrameId()
{
  UpdateRefFrame();
  return m_frameID;
}

void SegmentPrototype::UpdateRefFrame()
{
  RelPose* temp = m_relPoseOfRefFrame;
  m_relPoseOfRefFrame  = RelPoseFactory::GetRelPose(m_relFrame);
  if(temp  != NULL)
  {
    RelPoseFactory::FreeRelPose(temp );
  }
  m_frameID = m_relPoseOfRefFrame->m_uniqueID;
}

SegmentPrototype::~SegmentPrototype(void)
{
  RelPoseFactory::FreeRelPose(m_relPoseOfRefFrame);
}


void SegmentPrototype::SaveTo(XMLTag* tag)
{
  Descriptor::SaveTo(tag);
  tag->AddProperty(XML_ATTRIBUTE_RELFRAME, m_relFrame);
  tag->AddProperty(XML_ATTRIBUTE_PARALLEL, m_parallel);
  tag->AddProperty(XML_ATTRIBUTE_COVX, m_covRotX);
  tag->AddProperty(XML_ATTRIBUTE_COVY, m_covRotY);
  tag->AddProperty(XML_ATTRIBUTE_COVZ, m_covRotZ);
}


void SegmentPrototype::Show(RelPose* pose, Sensor* camin)
{
  printf("in SegmentPrototype::Show(RelPose* pose, Sensor* camin)\n");
  if(camin != NULL && pose != NULL && camin->GetRelPose() != NULL)
  {
     Matrix m = pose->GetMatrix(camin->GetRelPose()->m_uniqueID);
     printf("Pose %ld in %ld\n", pose->m_uniqueID, camin->GetRelPose()->m_uniqueID);
     cout << m;
     double row, column;
     camin->ProjectPoint3DToSensor(m.element(0, 3), m.element(1,3), m.element(2,3), row, column);
     printf("projected to %f, %f\n", row, column);
   }
}

