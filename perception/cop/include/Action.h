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
                        Action.h - Copyright klank


**************************************************************************/


#ifndef ACTION_H
#define ACTION_H
#include "Object.h"

#define XML_NODE_ACTION "Action"
/**
  * class Action
  *   @brief Action represents recognizable actions in the database Elem hirarchy
  */
class Action : public Elem
{
public:

  // Constructors/Destructors
  //  


	/**
	* Empty Constructor
	*/
	Action ( );
	/**
	*	Constructor from a saved configuration
	*/
	Action ( XMLTag* tag );

	/**
	* Empty Destructor
	*/
	virtual ~Action ( );

	Object* m_objectS;

	// Public attribute accessor methods
	//  
	std::string GetNodeName(){return XML_NODE_ACTION;}
	void SaveTo(XMLTag* tag);

private:


  void initAttributes ( ) ;

};

#endif // ACTION_H
