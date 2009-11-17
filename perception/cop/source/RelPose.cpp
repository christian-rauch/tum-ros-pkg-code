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
#ifdef HALCONIMG
#include "cpp/HalconCpp.h"
#endif
// Constructors/Destructors
//
#ifndef NO_LOLIBADDED



RelPose::RelPose(jlo::LazyLocatedObjectLoader* loader) :
jlo::LocatedObject(loader),
m_qualityMeasure(0.0)
{

}
RelPose::RelPose(jlo::LazyLocatedObjectLoader* loader, int id, int parentID, Matrix m, Matrix cov) :
jlo::LocatedObject(loader, id, parentID, m, cov),
m_qualityMeasure(0.0)
{

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
#ifdef HALCONIMG

/*RelPose::RelPose(Halcon::HTuple& relPose, Halcon::HTuple& covariance, RelPose* pose) :
jlo::LocatedObject(pose),
m_qualityMeasure(0.0)
{
  Update(relPose, covariance, pose);
  //cout << "RelPoseConstructor Matrix:\n" << GetMatrix() <<std::endl;
}*/
bool RelPose::TupleToMat(Halcon::HTuple& poseDesc, Halcon::HTuple&  covariance, Matrix &m, Matrix &d)
{
      if(poseDesc.Num() > 6)
      {
        Halcon::HTuple hommat;
        if(poseDesc.Num() == 12)
            hommat = poseDesc;
        else
           Halcon::pose_to_hom_mat3d(poseDesc, &hommat);
        m << hommat[0].D() << hommat[1].D() << hommat[2].D() << hommat[3].D()
          << hommat[4].D() << hommat[5].D() << hommat[6].D() << hommat[7].D()
          << hommat[8].D() << hommat[9].D() <<hommat[10].D() <<hommat[11 ].D()
          << 0 << 0 <<0 << 1;
        d.element(0,0)=(covariance.Num() > 0 ? covariance[0].D() : 0.001);
        d.element(1,1)=(covariance.Num() > 1 ? covariance[1].D() : 0.001);
        d.element(2,2)=(covariance.Num() > 2 ? covariance[2].D() : 0.001);
        d.element(3,3)=(covariance.Num() > 3 ? ((covariance[3].D() / 180) * M_PI) : 0.001);
        d.element(4,4)=(covariance.Num() > 4 ? ((covariance[4].D() / 180) * M_PI) : 0.001);
        d.element(5,5)=(covariance.Num() > 5 ? ((covariance[5].D() / 180) * M_PI) : 0.001);
        d.element(0,1) = 0.0; d.element(0,2) = 0.0;d.element(0,3) = 0.0; d.element(0,4) = 0.0;d.element(0,5) = 0.0;
        d.element(1,0) = 0.0; d.element(1,2) = 0.0;d.element(1,3) = 0.0; d.element(1,4) = 0.0;d.element(1,5) = 0.0;
        d.element(2,0) = 0.0; d.element(2,1) = 0.0;d.element(2,3) = 0.0; d.element(2,4) = 0.0;d.element(2,5) = 0.0;
        d.element(3,0) = 0.0; d.element(3,1) = 0.0;d.element(3,2) = 0.0; d.element(3,4) = 0.0;d.element(3,5) = 0.0;
        d.element(4,0) = 0.0; d.element(4,1) = 0.0;d.element(4,2) = 0.0; d.element(4,3) = 0.0;d.element(4,5) = 0.0;
        d.element(5,0) = 0.0; d.element(5,1) = 0.0;d.element(5,2) = 0.0; d.element(5,3) = 0.0;d.element(5,4) = 0.0;
        return true;
      }
      return false;
}

void RelPose::Update(Halcon::HTuple& poseDesc, Halcon::HTuple& covariance, RelPose* relation)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  m_relation = relation;
#endif /*NO_LO_SERVICE_AVAILABLE*/
  Halcon::HTuple hommat;
  if(poseDesc.Num() > 6)
  {
    Matrix m(4,4);
    Matrix d(6,6);
    TupleToMat(poseDesc, covariance, m, d);
    Set(m, d);
  }
}
#endif
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

  ret->AddProperty(XML_ATTRIBUTE_LOID, m_uniqueID);
  if(m_uniqueID != ID_WORLD)
    ret->AddProperty(XML_ATTRIBUTE_LOIDFATHER, m_parentID);
  ret->AddChild(XMLTag::Tag(GetMatrix(0), XML_NODE_MATRIX));
  ret->AddChild(XMLTag::Tag(GetCovarianceMatrix(0), XML_NODE_COVARIANCE));

  return ret;
}
#endif

#ifdef HALCONIMG
#ifdef _DEBUG
void RelPose::Print()
{
  Halcon::HTuple pose;
  printf("Printing Pose id: %ld parent id %ld\n", m_uniqueID, m_parentID);
  GetPose(&pose);
  for(int i = 0; i < pose.Num() -1; i++)
  {
    printf("%f, ", pose[i].D());
  }
  printf("%d\n", pose[pose.Num() -1].I());

}
#endif
void RelPose::GetHommat(Halcon::HTuple* hommat, int poseRel)
{
  Matrix m(4,4);
  if(poseRel > 0 && (unsigned)poseRel != m_parentID)
  {
      RelPose* pose_rel = RelPoseFactory::GetRelPose(m_uniqueID, poseRel);
      m = pose_rel->GetMatrix();
      RelPoseFactory::FreeRelPose(pose_rel);
  }
  else
    m = GetMatrix();
  Halcon::tuple_gen_const(12,0,hommat);
  (*hommat)[0] = m.element(0,0);
  (*hommat)[1] = m.element(0,1);
  (*hommat)[2] = m.element(0,2);
  (*hommat)[3] = m.element(0,3);
  (*hommat)[4] = m.element(1,0);
  (*hommat)[5] = m.element(1,1);
  (*hommat)[6] = m.element(1,2);
  (*hommat)[7] = m.element(1,3);
  (*hommat)[8] = m.element(2,0);
  (*hommat)[9] = m.element(2,1);
  (*hommat)[10] = m.element(2,2);
  (*hommat)[11] = m.element(2,3);
}
void RelPose::GetPose(Halcon::HTuple* poses, int poseRel)
{
  Halcon::HTuple hommat;
  GetHommat(&hommat, poseRel);
  Halcon::hom_mat3d_to_pose(hommat,poses);
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
