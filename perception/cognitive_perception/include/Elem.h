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
                        Elem.h - Copyright klank


**************************************************************************/


#ifndef ELEM_H
#define ELEM_H

#include <string>
#include <vector>
#include "ElemTypes.h"


#define XML_PROPERTY_ELEMID "ElemID"
#define XML_NODE_ELEMENT "Elem"

namespace cop
{

  class XMLTag;
  class Sensor;

  /**
    * class Elem
    * @brief basic class for all entries in the SignatureDB
    */
  class Elem
  {
  public:
    /**
     * Constructor that creates a new element
     */
    Elem( );
    /**
    * Constructor that restores an certain element
    */
    Elem ( int id );

    /**
     * Empty Destructor
     */
    virtual ~Elem( );

    // Static Public attributes
    //
    static int m_LastID;
    // Public attributes
    //
    int m_ID;
    /**
    *   Save the current element, the function will call SaveTo().
    *   @param full_pose if set to true poses will not be saved as jlo reference but as a matrix
    */
    XMLTag* Save(bool full_pose = false);
    /**
    *	Can Creates most of the possible elements, returns null if the node name was not known
    *	@param tag xmltag that has an name of an elem, see ElemType.h for the possible elements
    *	@returns a inheritation of elem.
      *   @throws char* with an error message in case of failure
    */
    static Elem* ElemFactory(XMLTag* tag);
    /**
    *	Duplicates an elem (only possible for elems that can be constructed with ElemFactory or that have overriden this member)
    */
    virtual Elem* Duplicate(bool bStaticCopy = true);
    /**
    *   returns the name that a will be used in the xml. This should be replaced by any derivative of Elem
    */
    virtual std::string GetNodeName() const {return XML_NODE_ELEMENT;}

    /**
    *   Returns the type which is used for checkign for certain descriptor types, should be overwritten by any derivative of Elem
    */
    virtual int GetType() const {return ELEM;}
    /**
    *   Shows the current Element relative to the given Camera
    */
    virtual void Show(Sensor* ){};

    /**
    *	Age of the newest information carried by this element
    */
    virtual unsigned long date() const
    {
      return m_timestamp;
    }
    void Touch();

  protected:
    /**
    * Overwrite to save additional data, will be called during saving
    */
    virtual void SaveTo(XMLTag* )      {}
    /**
    * Overwrite to set additional data on creation, will be called directly after creation
    */
    virtual void SetData(XMLTag* data);

    bool m_fullPose;
  private:

   unsigned long m_timestamp;

  };
}
#endif // ELEM___H
