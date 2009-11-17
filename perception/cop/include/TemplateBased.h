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
                        TemplateBased.h - Copyright klank

lateBased.h
**************************************************************************/


#ifndef TEMPLATEBASED_H
#define TEMPLATEBASED_H
#include "LocateAlgorithm.h"


#define XML_NODE_TEMPLATEBASED "TemplateBased"
/**
  * class TemplateBased
  */

class TemplateBased : public LocateAlgorithm
{
public:

	// Constructors/Destructors
	//  


	/**
	* Empty Constructor
	*/
	TemplateBased ( );
	TemplateBased ( XMLTag* tag );

	/**
	* Empty Destructor
	*/
	virtual ~TemplateBased ( );

	//
	// Methods
	//  
	XMLTag* Save();
	// Public attributes
	//  


	// Public attribute accessor methods
	//  


	// Public attribute accessor methods
	//  


protected:

  // Static Protected attributes
  //  

  // Protected attributes
  //  


  // Protected attribute accessor methods
  //  


  // Protected attribute accessor methods
  //  


private:

  // Static Private attributes
  //  

  // Private attributes
  //  


  // Private attribute accessor methods
  //  


  // Private attribute accessor methods
  //  



};

#endif // TEMPLATEBASED_H
