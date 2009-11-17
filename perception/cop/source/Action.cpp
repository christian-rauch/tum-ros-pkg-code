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
                        Action.cpp - Copyright klank


**************************************************************************/

#include "Action.h"
#include "XMLTag.h"
// Constructors/Destructors
//  

Action::Action ( ) {
initAttributes();
}

Action::Action ( XMLTag* tag ) :
	Elem(tag)
{
	if(tag != NULL  || tag->GetName().compare(XML_NODE_ACTION) != 0)
	{
		m_objectS = (Object*)ElemFactory(tag->GetChild(0));
	}
}

Action::~Action ( ) { }

//  
// Methods
//  

void Action::SaveTo(XMLTag* tag)
{
	tag->AddChild(m_objectS->Save());
}

// Accessor methods
//  


// Public static attribute accessor methods
//  


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

void Action::initAttributes ( ) {
}

