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
                        Object.h - Copyright klank


**************************************************************************/


#ifndef OBJECT_H
#define OBJECT_H
#include "Elem.h"
#include "Comm.h"
#include "TexturedModel.h"
#include "RelPoseFactory.h"

#define XML_NODE_OBJECT "Object"
/**
  * class Object
  * @brief representation of an object in the SignatureDB, part of the Elem hirarchy
  */
class Object : public Elem
{
public:

	// Constructors/Destructors
	//  


	/**
	* Empty Constructor
	*/
	Object ( );
	Object ( XMLTag* tag );
	/**
	* Empty Destructor
	*/
	virtual ~Object ( );

	// Static Public attributes
	//  
	virtual std::string GetNodeName() const{return XML_NODE_OBJECT;}

	virtual void SetPose(RelPose* pose);

  virtual void SetCommCallBack(Comm* com);
	/**
	*	Last know relative position
	*/
	RelPose* m_relPose;
  bool m_bCommunicationCallBack;
  Comm* m_com;



protected:
	virtual void SaveTo(XMLTag* tag);

private:


  void initAttributes ( ) ;

};

#endif // OBJECT_H
