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
                        Signature.h - Copyright klank


**************************************************************************/


#ifndef SIGNATURE_H
#define SIGNATURE_H


#include "Class.h"
#include "Object.h"
class RelPose;
class Camera;
#ifndef XML_NODE_SIGNATURE
#define XML_NODE_SIGNATURE "Signature"
#define XML_NODE_SIGNATURE_VEC "SignatureVector"
#endif

#ifdef BOOST_THREAD
#include <boost/thread.hpp>
#else
#endif

/**
  * class Signature
  *	@brief contains the information about an object in the SignatureDB, part of the Elem hirarchy
  */
class Signature : public Object
{
public:

  // Constructors/Destructors
  //
  /**
  * Empty Constructor
  */
  Signature ( );
  /**
  *   Constructor
    *   @param tag
    *   @throws char* with an error message in case of failure
    */
  Signature ( XMLTag* tag );

  virtual ~Signature ( );

  // Static Public attributes
  //

  // Public attributes
  //

  // Public attribute accessor methods
  //


  // Public attribute accessor methods
  //



  /**
  * @return Elem
  * @param  index
  * @param  type
  */
  Elem* GetElement (int id, int type );

  size_t CountClasses(){return m_class.size();}
  size_t CountElems(){return m_elems.size();}
  Class* GetClass(int index);
  bool HasClass(Class* classToSet);

  virtual std::string GetNodeName() const{return XML_NODE_SIGNATURE;}
  /**
  * @return int
  * @param  elemToSet
  */
  long SetElem (Elem* elemToSet );
  long SetClass ( Class* classToSet);
  void Show(Camera* cam);
    virtual int GetType(){return SIGNATURE;}
protected:
  virtual void SaveTo(XMLTag* tag);

  // Protected attribute accessor methods
  //


private:

  std::vector<Elem*> m_elems;
  std::vector<Class*> m_class;
  void initAttributes ( ) ;
#ifdef BOOST_THREAD
  boost::mutex m_mutexElems;
  boost::mutex m_mutexClasses;
#endif
};

#endif // SIGNATURE_H
