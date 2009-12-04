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
#include "Sensor.h"

#define XML_NODE_CLASSES "Classes"

#include "BoostUtils.h"

#ifdef BOOST_THREAD
#include <boost/thread/mutex.hpp>
#define BOOST(A) A
#else
#define BOOST(A)
#endif

#ifdef _DEBUG
#define DEBUG(A) A
#else
#define DEBUG(A) ;
#endif

using namespace cop;


// Constructors/Destructors
//

Signature::Signature ( ) {
initAttributes();
}

void Signature::SetData ( XMLTag* tag )
{
  Object::SetData(tag);
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
        SetClass((Class*)Elem::ElemFactory(classes->GetChild(i)));
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
  BOOST(m_mutexElems.lock());

  for(std::vector<Elem*>::const_iterator iter = m_elems.begin();
    iter != m_elems.end(); iter++)
  {
    delete (*iter);
  }
  m_elems.clear();
  BOOST(m_mutexElems.unlock());

  BOOST(m_mutexClasses.lock());
  for(std::vector<Class*>::const_iterator iter = m_class.begin();
    iter != m_class.end(); iter++)
  {
    delete (*iter);
  }
  m_class.clear();
  BOOST(m_mutexClasses.unlock());

}

//
// Methods
//

Class* Signature::GetClass(int index)
{
  DEBUG(printf("GetClass: %p\n", this));
  BOOST(m_mutexClasses.lock());
  Class* ret  = NULL;
  if((signed)m_class.size() > index)
    ret = m_class[index];
  BOOST(m_mutexClasses.unlock());
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

void Signature::Show(Sensor* cam)
{
#ifdef BOOST_THREAD
  Sleeping(0.001);
#endif
  try
  {
    if(cam!= NULL)
      cam->Show();
  }
  catch(const char* text)
  {
    printf("Display of sensor data failed: %s\n", text);
  }
  catch (...)
  {
    printf("Display of sensor data failed\n");
  }
  DEBUG(printf("Entering Showing of signature %d\n", m_ID));
  for(unsigned int i = 0; i < CountElems(); i++)
  {
    try
    {
      ((Descriptor*)GetElement(i,0))->Show(m_relPose, cam);
    }
    catch(...)
    {
      printf("Showing of elem %d failed ... \n", i);
    }
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
  BOOST(m_mutexElems.lock());

    m_elems.push_back(elemToSet);
  BOOST(m_mutexElems.unlock());

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
      BOOST(m_mutexClasses.lock());
      m_class.push_back(classToSet);
      BOOST(m_mutexClasses.unlock());

      return m_class.size() - 1;
    }
  }
  return -1;
}


void Signature::initAttributes ( ) {
}

