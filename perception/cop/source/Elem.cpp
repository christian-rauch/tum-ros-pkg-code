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
                        Elem.cpp - Copyright klank


**************************************************************************/

#include "Elem.h"
#include "XMLTag.h"
#include "Signature.h"
#include "ShapeModel.h"
#include "TexturedModel.h"
#include "ColorModel.h"
#include "CalTab.h"
#include "Blob.h"
#ifdef DESCRIPTOR_AVAILABLE
#include "PointDescrModel.h"
#endif
#ifdef DEFORMSHAPE_AVAILABLE
#include "DeformShapeModel.h"
#endif
#include "Object.h"
#include "Descriptor.h"
#include "Class.h"
#include "Action.h"
#include "ColorClass.h"
#include "SupportingPlanerDescriptor.h"
#include <time.h>

int Elem::m_LastID = 0;


// Constructors/Destructors
//

Elem::Elem ( ) :
    m_ID(m_LastID),
    m_timestamp((unsigned long)time(NULL))
{
    m_LastID++;

}

Elem::Elem ( int id ) :
    m_ID(id),
    m_timestamp((unsigned long)time(NULL))
{
        if(m_LastID < m_ID)
            m_LastID = m_ID + 1;
}

Elem::Elem ( XMLTag* creator) :
    m_timestamp((unsigned long)time(NULL))
{
    if(creator != NULL)
    {
        m_ID = creator->GetPropertyInt(XML_PROPERTY_ELEMID);
        if(m_LastID < m_ID)
            m_LastID = m_ID + 1;

        m_timestamp = creator->date();
        m_timestamp -= 2;
    }
}

bool StringEquals(std::string name1, std::string name2)
{
    return name1.compare(name2) == 0;
}

Elem* Elem::ElemFactory ( XMLTag* tag)
{
    if(tag == NULL)
        throw "WRONG NODE";
    std::string name = tag->GetName();
    Elem* elem = NULL;
    if(StringEquals(name, XML_NODE_CLASS))
    {
        elem = new Class(tag);
    }
    else if(StringEquals(name, XML_NODE_ACTION))
    {
        elem = new Action(tag);
    }
    else if(StringEquals(name, XML_NODE_DESCRIPTOR))
    {
        elem = new Descriptor(tag);
    }
    else if(StringEquals(name, XML_NODE_ELEMENT))
    {
        elem = new Elem(tag);
    }
    else if(StringEquals(name, XML_NODE_OBJECT))
    {
        elem = new Object(tag);
    }
    else if(StringEquals(name, XML_NODE_SHAPEMODEL))
    {
        elem = new ShapeModel(tag);
    }
    else if(StringEquals(name, XML_NODE_SIGNATURE))
    {
        elem = new Signature(tag);
    }
    else if(StringEquals(name, XML_NODE_TEXMODEL))
    {
        elem = new TexturedModel(tag);
    }
    else if(StringEquals(name, XML_NODE_COLORMODEL))
    {
        elem = new ColorModel(tag);
    }
    else if(StringEquals(name, XML_NODE_CALTAB))
    {
        elem = new CalTab(tag);
    }
    else if(StringEquals(name, XML_NODE_BLOB))
    {
        elem = new Blob(tag);
    }
    else if(StringEquals(name, XML_NODE_COLORCLASS))
    {
        elem = new ColorClass(tag);
    }
    else if(StringEquals(name, XML_NODE_PLANE_DESCR))
    {
        elem = new SupportingPlanerDescriptor(tag);
    }
#ifdef DESCRIPTOR_AVAILABLE
    else if(StringEquals(name, XML_NODE_POINTDESCRMODEL))
    {
        elem = new PointDescrModel(tag);
    }
#endif
#ifdef DEFORMSHAPE_AVAILABLE
    else if(StringEquals(name, XML_NODE_DEFORMSHAPEMODEL))
    {
        elem = new DeformShapeModel(tag);
    }
#endif
    return elem;
}

Elem* Elem::Duplicate(bool bStaticCopy)
{
  XMLTag* tag = this->Save();
  if(!bStaticCopy)
  {
    tag->AddProperty(XML_PROPERTY_ELEMID, m_LastID++);
  }
    return Elem::ElemFactory(tag);
}

void Elem::Touch()
{
    m_timestamp = (unsigned long)time(NULL);
}



Elem::~Elem ( ) { }

//
// Methods
//
XMLTag* Elem::Save(bool full_pose)
{
  m_fullPose = full_pose;
    XMLTag* ret = new XMLTag(GetNodeName());
    ret->AddProperty(XML_PROPERTY_ELEMID, m_ID);
    SaveTo(ret);
    return ret;
}

// Accessor methods
//


// Other methods
//


