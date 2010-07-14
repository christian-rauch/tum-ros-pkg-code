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
                        RelPose.cpp - Copyright klank
**************************************************************************/

#include "RelPose.h"
#include "XMLTag.h"
#include "Object.h"

// Constructors/Destructors
//

using namespace cop;

#ifndef NO_LOLIBADDED



RelPose::RelPose(jlo::LazyLocatedObjectLoader* loader) :
jlo::LocatedObject(loader),
m_qualityMeasure(0.0)
{

}
RelPose::RelPose(jlo::LazyLocatedObjectLoader* loader, unsigned long id, unsigned long parentID, Matrix m, Matrix cov, std::string name) :
jlo::LocatedObject(loader, id, parentID, m, cov),
m_qualityMeasure(0.0)
{
  m_mapstring = name;
}

RelPose::RelPose(jlo::LocatedObject& pose) :
jlo::LocatedObject(pose),
m_qualityMeasure(0.0)
{
}



/*RelPose::RelPose(RelPose* pose, Matrix m, Matrix cov) :
jlo::LocatedObject(pose),
m_qualityMeasure(0.0)
{
  Set(m, cov);
}*/

/*RelPose::RelPose(jlo::LocatedObject* pose, Matrix m, Matrix cov) :
jlo::LocatedObject(pose),
m_qualityMeasure(0.0)
{
  Set(m, cov);
}*/

/*void RelPose::TransformPointLocally(const double& x_in, const double& y_in, const double& z_in, double& x_out, double& y_out, double& z_out, const double& scale)
{
//	ColumnVector cm = GetRotation();
//	cout << cm << "\n";

  Matrix m = GetMatrix();
  ColumnVector d(4);
  d << x_in / scale << y_in  / scale << z_in / scale<< 1;
  ColumnVector f = m * d;
//	cout << m << " \n* \n" << d << "\n=\n" << f;
  x_out = f.element(0);
  y_out = f.element(1);
  z_out = f.element(2);
  //cout << x_out << ", " << y_out << ", " << z_out << "\n";
}
*/
XMLTag* RelPose::Save()
{
  XMLTag* ret = new XMLTag(XML_NODE_RELPOSE);
  if(m_mapstring.length() > 0)
    ret->AddProperty(XML_ATTRIBUTE_LOID, m_mapstring);
  else
    ret->AddProperty(XML_ATTRIBUTE_LOID, m_uniqueID);

  return ret;
}

#ifdef NO_LO_SERVICE_AVAILABLE
XMLTag* RelPose::SaveComplete()
{
  XMLTag* ret = new XMLTag(XML_NODE_RELPOSE);

  ret->AddProperty(XML_ATTRIBUTE_LOID, m_uniqueID);
  if(m_uniqueID != ID_WORLD)
    ret->AddProperty(XML_ATTRIBUTE_LOIDFATHER, m_relation == NULL ? ID_WORLD : m_relation->m_uniqueID);
  ret->AddChild(XMLTag::Tag(GetMatrix(0), XML_NODE_MATRIX));
  ret->AddChild(XMLTag::Tag(GetCovarianceMatrix(0), XML_NODE_COVARIANCE));

  return ret;
}
#else
XMLTag* RelPose::SaveComplete()
{
  XMLTag* ret = new XMLTag(XML_NODE_RELPOSE);

  if(m_mapstring.length() > 0)
    ret->AddProperty(XML_ATTRIBUTE_LOID, m_mapstring);
  else
    ret->AddProperty(XML_ATTRIBUTE_LOID, m_uniqueID);

  if(m_uniqueID != ID_WORLD)
    ret->AddProperty(XML_ATTRIBUTE_LOIDFATHER, m_parentID);
  ret->AddChild(XMLTag::Tag(GetMatrix(0), XML_NODE_MATRIX));
  ret->AddChild(XMLTag::Tag(GetCovarianceMatrix(0), XML_NODE_COVARIANCE));

  return ret;
}
#endif


#else
RelPose::RelPose ( ) :
  m_qualityMeasure(0.0)
{

initAttributes();
}

RelPose::RelPose (Pose pose, Object* relation) :
  m_pose(pose),
  m_relation(relation),
  m_qualityMeasure(0.0)
{

}

RelPose::RelPose (XMLTag* tag) :
  m_qualityMeasure(0.0)
{
  if(tag != NULL)
  {
    if(tag->GetName().compare(XML_NODE_RELPOSE) == 0)
    {
      m_pose = Pose(tag->GetChild(0));
      if(tag->CountChildren() > 1)
        m_relation = (Object*)Elem::ElemFactory(tag->GetChild(1));
      else
        m_relation = NULL;
    }
    else
      throw "WRONG NODE";
  }
}

RelPose::~RelPose ( ) { }

//
// Methods
//


// Accessor methods
//
#define MAX_DEPTH 5

inline bool CheckCircles(Object* relation, int depth)
{
  if(depth <= 0)
    return false;
  if(relation != NULL && relation->m_relPose != NULL)
  {
    return CheckCircles(relation->m_relPose->m_relation, depth - 1);
  }
  return true;
}
XMLTag* RelPose::Save()
{
  XMLTag* ret = new XMLTag(XML_NODE_RELPOSE);
  ret->AddChild(m_pose.Save());
  if(m_relation != NULL)
  {
    if ( CheckCircles ( m_relation, MAX_DEPTH) )
      ret->AddChild(m_relation->Save());
  }
  return ret;
}

RelPose& RelPose::operator +=( const Pose &p)
{
  this->m_pose += p;
  return *this;
}

RelPose RelPose::operator + (const Pose &p) const
{
  RelPose rel (this->m_pose, this->m_relation);
  rel.m_pose += p;
  return rel;
}


RelPose RelPose::operator - ( const int &levels ) const
{
  RelPose rel(this->m_pose, this->m_relation);
  if(m_relation != NULL)
  {
    if(levels > 0)
    {
      if(m_relation->m_relPose != NULL)
        rel = (*m_relation->m_relPose) + m_pose;
      else
        rel.m_relation = NULL;
      if(levels > 1)
        rel = rel - (levels - 1);
    }
  }
  return rel;
}




// Public attribute accessor methods
//

// Protected static attribute accessor methods
//


// Protected attribute accessor methods
//


// Private static attribute accessor methods
//


// Private attribute accessor methods
//


// Other methods
//

void RelPose::initAttributes ( ) {
}
#endif


Matrix RelPose::GetMatrix(unsigned long id)
{
  if(id == 0 || m_parentID == id)
    return LocatedObject::GetMatrix();
  else
  {
    RelPose* pose = RelPoseFactory::GetRelPose(m_uniqueID, id);
    Matrix m =  pose->GetMatrix(0);
    RelPoseFactory::FreeRelPose(pose);
    return m;
  }
}

