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
                        SignatureDB.h - Copyright klank


**************************************************************************/


#ifndef SIGNATUREDB_H
#define SIGNATUREDB_H

#include <string>
#include <vector>

#include "XMLTag.h"
#ifndef XML_NODE_SIGNATURE
#define XML_NODE_SIGNATURE "Signature"
#endif

#define XML_NODE_SIGNATUREDB "SignatureDB"

namespace cop
{
  class ShapeModelDownloader;
  /**
    * class SignatureDB
    * @the database that contains all object and class relations and models of the vision system
    */
  class SignatureDB
  {
  public:

    // Constructors/Destructors
    //


    /**
    * Empty Constructor
      *   @throws char* with an error message in case of failure
    */
    SignatureDB ( XMLTag* config );

    /**
    * Empty Destructor
    */
    virtual ~SignatureDB ( );

    // Methods
    //
    void UpdateNodes(Signature* sig, int index);
    /**
    * Adds a signature to the database, an index will be created for all related classes
    */
    int AddSignature(Signature* sig);
  #ifdef BOOST_THREAD
    void AddSignatureThreaded(Signature* sig);
  #endif
    /**
    * Free Active Signature
    * Marks a signature for later delete
    */
    void FreeActiveSignature(Signature* sig);
      /***
      *   Adds a class to the database
      */
    void AddClass(std::string stname, int id);
    /**
    *	Direct request for an element
    */
    Signature* GetSignatureByID(int ElemID);

    /*******************************************************************************
    *   GetClassByID                                                              */
    /*******************************************************************************
    *
    * @brief Direct Request for a class
    * @remarks throws on error a char* Exception
    *
    *******************************************************************************/
    Class* GetClassByID(int id);
    /**
    *	Direct request for an class
    */
    Signature* GetSignatureByClass(int ClassID, int index = 0);
    Signature* GetSignatureByClass(std::string className, int index = 0){return GetSignatureByClass(CheckClass(className), index);}

    /*******************************************************************************
    *   GetSignature                                                              */
    /*******************************************************************************
    *
    * @param class_ids  a list of ids that specify classes in the database
    *
    * @return           returns the Signature of a new instance of an object
    *                   having all class labels, returns NULL if the classes
    *                   do not exist and could not be derived
    * @remarks throws char* on error
    *******************************************************************************/
    Signature* GetSignature(std::vector<int> class_ids);

    /*******************************************************************************
    *   FindCreateDescriptor                                                      */
    /*******************************************************************************
    *
    * @param class_id   an id that specify an class
    * @return           returns an Elem describing the
    *                   class class_id, returns NULL if the class
    *                   does not exist and could not be derived
    * @remarks throws char* on error
    *******************************************************************************/
    Elem* FindCreateDescriptor(int class_id);

    // Public attributes
    //

    std::string CheckClass(int id);
    int CheckClass(std::string name);

    // Public attribute accessor methods
    //
    /**
    *	Answers any queryx to the cop model database, pass class name or id as string
    */
    XMLTag* Query(std::string stQueryString);

    /**
    *	Check if this ID is already in the DB
    */
    bool Check(int sigID, int& error) const;


    XMLTag* Save();
  protected:

    // Static Protected attributes
    //

    // Protected attributes
    //
    void UpdateIDList();

    Signature* GetSignatureByIndex(unsigned int index);

    /**
    * CleanUpActiveSignatureList
    * deletes all old signatures in the active Signature List
    */
    void CleanUpActiveSignatureList();
    /**
    * AddSignatureToActiveList
    * Adds a signature to the list of currenty active signatures and maps it to the index
    */
    void AddSignatureToActiveList(Signature* sig, int index);
  private:
    // XML data
    //
    XMLTag* m_dbStarter;
    XMLTag* m_index;


    std::vector<int> m_ids;
    std::vector<std::pair<std::string, int> > m_classes;

    std::vector<std::pair<Signature*, int> > m_currentlyActiveSignatures;
    std::map<int, int> m_activeMap;
    /**
    *	Helping function to get the first element connected with a class
    */
    int GetElemIdByClass(int ClassID, int index = 0);

    ShapeModelDownloader* s_shapeDownloader;

  };
}
#endif // SIGNATUREDB_H
