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
                        Signature.cpp - Copyright klank


**************************************************************************/

#include "Signature.h"
#include "XMLTag.h"
#include "Camera.h"
#ifdef _DEBUG
#include "ShapeModel.h"
#include "CalTab.h"
#endif
#ifdef HALCONIMG
#include "cpp/HalconCpp.h"
#endif

#define XML_NODE_CLASSES "Classes"

#ifdef BOOST_THREAD
#ifdef BOOST_1_35
#include <boost/thread/mutex.hpp>
#else
#include <boost/thread/detail/lock.hpp>
typedef boost::detail::thread::lock_ops<boost::mutex> locker;
#endif
#define BOOST(A) A
#else
#define BOOST(A)
#endif
using namespace cop;


// Constructors/Destructors
//

Signature::Signature ( ) {
initAttributes();
}

Signature::Signature ( XMLTag* tag ) :
  Object(tag)
{
  std::string stName = tag->GetName();
  std::string stDefault = GetNodeName();
  unsigned int i;
  if(stName.compare(stDefault) != 0)
  {
    printf("Wrong node detected opening a XML-File for reading a signature\n");
    throw "WRONG NODE";
  }
  //Call Elem Factory
  //Call Class Factory
  try
  {
    XMLTag* classes = tag->GetChild(XML_NODE_CLASSES);
    if(classes  != NULL)
    {
      for(i = 0; i < classes->CountChildren(); i++)
      {
        SetClass(Class::ClassFactory(classes->GetChild(i)));
      }
    }
    else
    {
      printf("Signature: XML node missing: Classes\n");
    }
  }
  catch(...)
  {
     printf("Error creating Classes\n");
  }
  XMLTag* describingElems = tag->GetChild(0);
  if(describingElems != NULL)
  {
    for(i = 0; i < describingElems->CountChildren(); i++)
    {
      XMLTag* tagdescChild = describingElems->GetChild(i);
      if(tagdescChild != NULL)
      {
        try
        {
          SetElem(Elem::ElemFactory(tagdescChild));
        }
        catch(...)
        {
          printf("Error creating Descriptor:  index %d: name %s\n", i, tagdescChild->GetName().c_str());
          printf("Content: %s \n", tagdescChild->WriteToString());
          tagdescChild->FreeAfterWriteToString();
        }
      }
    }
  }
  else
  {
    printf("Signature: XML node missing (descriptors)\n");
  }
}

Signature::~Signature ( )
{
  printf("\n\nDeleting Signature\n\n");
#ifdef BOOST_1_35
  BOOST(m_mutexElems.lock());
#else
  BOOST(locker::lock(m_mutexElems));
#endif

  for(std::vector<Elem*>::const_iterator iter = m_elems.begin();
    iter != m_elems.end(); iter++)
  {
    delete (*iter);
  }
  m_elems.clear();
#ifdef BOOST_1_35
  BOOST(m_mutexElems.unlock());
#else
  BOOST(locker::unlock(m_mutexElems));
#endif

#ifdef BOOST_1_35
  BOOST(m_mutexClasses.lock());
#else
  BOOST(locker::lock(m_mutexClasses));
#endif

  for(std::vector<Class*>::const_iterator iter = m_class.begin();
    iter != m_class.end(); iter++)
  {
    delete (*iter);
  }
  m_class.clear();
#ifdef BOOST_1_35
  BOOST(m_mutexClasses.unlock());
#else
  BOOST(locker::unlock(m_mutexClasses));
#endif

}

//
// Methods
//

Class* Signature::GetClass(int index)
{
  printf("GetClass: %p\n", this);
#ifdef BOOST_1_35
  BOOST(m_mutexClasses.lock());
#else
  BOOST(locker::lock(m_mutexClasses));
#endif
  Class* ret  = NULL;
  if((signed)m_class.size() > index)
    ret = m_class[index];
#ifdef BOOST_1_35
  BOOST(m_mutexClasses.unlock());
#else
  BOOST(locker::unlock(m_mutexClasses));
#endif
  return ret;
}

bool Signature::HasClass(Class* classToSet)
{
  size_t size = CountClasses();
  for (unsigned int i = 0 ; i < size; i++)
  {
    if(m_class[i]->m_ID == classToSet->m_ID)
      return true;
  }
  return false;
}
#ifdef HALCONIMG
void disp_3d_coord_system (Halcon::HTuple WindowHandle, Halcon::HTuple CamParam,
    Halcon::HTuple Pose, Halcon::HTuple CoordAxesLength);
#endif

using namespace Halcon;
void Signature::Show(Camera* cam)
{
  printf("\n\nEntering Showing of signature %d\n\n", m_ID);
  if(cam != NULL && m_relPose != NULL && cam->m_relPose != NULL)
  {
#ifdef HALCONIMG
    HWindow* hwin = cam->GetWindow();
    HTuple po, row, col;
    m_relPose->GetPose(&po, cam->m_relPose->m_uniqueID);
    try
    {
        project_3d_point(po[0],po[1],po[2],cam->m_calibration.CamParam(),&row,&col);
        set_tposition(hwin->WindowHandle(),row,col);
        hwin->SetColor("blue");
#ifdef WIN32
        set_font(hwin->WindowHandle(), "-Arial-14-");
#else /*WIN32*/
        set_font(hwin->WindowHandle(), "-*-courier-bold-r-normal--18-*-*-*-*-*-iso8859-1");
#endif /*WIN32*/
        write_string(hwin->WindowHandle(),m_ID);
        disp_3d_coord_system(hwin->WindowHandle(), cam->m_calibration.CamParam(), po, 0.1);
    }
    catch(Halcon::HException ex)
    {
      printf("Showing not possible: %s\n", ex.message);
      printf("Pose was id %ld\n", m_relPose->m_uniqueID);
    }
  }
  for(unsigned int i = 0; i < CountElems(); i++)
  {
    try
    {
      ((Descriptor*)GetElement(i,0))->Show(m_relPose, cam);
    }
    catch(Halcon::HException ex)
    {
      printf("Showing not possible (Elem %d):  %s\n",i, ex.message);
    }
    catch(...)
    {
      printf("Showing of elem %d failed ... \n", i);
    }
#endif
  }
}

void Signature::SaveTo(XMLTag* tag)
{
  XMLTag* delimiter = new XMLTag("DescibingElems");
  for(std::vector<Elem*>::const_iterator iter = m_elems.begin();
    iter != m_elems.end(); iter++)
  {
    delimiter->AddChild((*iter)->Save(m_fullPose));
  }
  tag->AddChild(delimiter);
  delimiter = new XMLTag("Classes");
  for(std::vector<Class*>::const_iterator iter = m_class.begin();
    iter != m_class.end(); iter++)
  {
    delimiter->AddChild((*iter)->Save());
  }
  tag->AddChild(delimiter);
  Object::SaveTo(tag);

  //TODO
}


// Private static attribute accessor methods
//


// Private attribute accessor methods
//


// Other methods
//


/**
 * @return Elem--
 * @param  index
 * @param  type
 */
 Elem* Signature::GetElement (const int &index, const int &type ) const
 {
  Elem* elem = NULL;

  if(type == 0)
  {
    if(index >= 0 && (unsigned)index < m_elems.size())
      elem = m_elems[index];
  }
  else
  {
    int count = index;
    for(std::vector<Elem*>::const_iterator iter = m_elems.begin();
      iter != m_elems.end(); iter++)
    {
      if((*iter)->GetType() == type)
      {
        if(count == 0)
        {
          elem = (*iter);
          break;
        }
        else
          count--;
      }
    }
  }
  return elem;
}


/**
 * @return int
 * @param  elemToSet
 */
long Signature::SetElem (Elem* elemToSet )
{
  if(elemToSet != NULL)
  {
#ifdef BOOST_1_35
  BOOST(m_mutexElems.lock());
#else
  BOOST(locker::lock(m_mutexElems));
#endif

    m_elems.push_back(elemToSet);
#ifdef BOOST_1_35
  BOOST(m_mutexElems.unlock());
#else
  BOOST(locker::unlock(m_mutexElems));
#endif

    int type = elemToSet->GetType();
    if(type > ELEM && type < SIGNATURE)
    {
      SetClass(((Descriptor*)elemToSet)->GetClass());
    }

    return m_elems.size() - 1;
  }
  return -1;
}

long Signature::SetClass (Class* classToSet )
{
  if(classToSet != NULL)
  {
    if(!HasClass(classToSet))
    {
#ifdef BOOST_1_35
  BOOST(m_mutexClasses.lock());
#else
  BOOST(locker::lock(m_mutexClasses));
#endif

      m_class.push_back(classToSet);
#ifdef BOOST_1_35
  BOOST(m_mutexClasses.unlock());
#else
  BOOST(locker::unlock(m_mutexClasses));
#endif

      return m_class.size() - 1;
    }
  }
  return -1;
}


void Signature::initAttributes ( ) {
}

