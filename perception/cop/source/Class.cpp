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
                        Class.cpp - Copyright klank


**************************************************************************/

#include "Class.h"
#include "XMLTag.h"

// Constructors/Destructors
//

Class::Class ( ) :
Elem()
{
  printf("Creating Empty Class\n");
}

Class::Class(std::string name, int id) :
  Elem(id),
  m_name(name)
{
  if(name.length() == 0)
  {
    printf("Creating Empty Class with empty name\n");
    throw "Error Building Classes";
  }

}


Class::Class ( XMLTag* tag) :
	Elem(tag)
{
  if(tag == NULL)
  {
    throw "Error loading class";
  }
  m_name = tag->GetProperty(XML_ATTRIBUTE_CLASSNAME);
  if(m_name.length() == 0)
  {
    printf("Creating Empty Class from xml\n");
   throw "Error Building Classes";
  }

}


Class::~Class ( ) { }

//
// Methods
//

void Class::SaveTo(XMLTag* tag)
{
	tag->AddProperty(XML_ATTRIBUTE_CLASSNAME, m_name);
}

Class* Class::ClassFactory(XMLTag* tag)
{
	return new Class(tag);
}


void Class::SetName(std::string name)
{
	m_name = name;
}
// Accessor methods
//


// Other methods
//


