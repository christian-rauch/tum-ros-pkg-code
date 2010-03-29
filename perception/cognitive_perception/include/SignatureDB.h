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
#include "PerceptionPrimitive.h"
#ifndef XML_NODE_SIGNATURE
#define XML_NODE_SIGNATURE "Signature"
#endif

#define XML_NODE_SIGNATUREDB "SignatureDB"

#define PROP_DECAY       0.01
#define STARTING_WEIGHT  0.25

namespace cop
{
  /**
    * class SignatureDB
    * @brief the database that contains all object and class relations and models of the vision system
    */
  class SignatureDB
  {
  public:

    // Constructors/Destructors
    //


    /**
    * Empty Constructor
      *   @throw char* with an error message in case of failure
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
    void AddAndShowSignatureAsync(Signature* sig, Sensor* sens);
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
    Class* GetClassByID(ObjectID_t id);
    /**
    *	Direct request for an class
    */
    Signature* GetSignatureByClass(ObjectID_t ClassID, int index = 0);
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
    * @throw char* on error
    *******************************************************************************/
    Signature* GetSignature(std::vector<ObjectID_t> class_ids);

    /*******************************************************************************
    *   CreateNewPerceptionPrimitive                                                              */
    /*******************************************************************************
    *
    * @param sig       a signature the will be changed by the signature
    *
    * @return           creates a new PerceptionPrimitive changinf the signature
    * @throw char* on error
    *******************************************************************************/
    PerceptionPrimitive& CreateNewPerceptionPrimitive(Signature* sig);
    /*******************************************************************************
    *   EvaluatePerceptionPrimitive                                                              */
    /*******************************************************************************
    *
    * @param id       a signature the will be changed by the signature
    * @param value    The value that should be back propagated
    * @throw char* on error
    *******************************************************************************/
    void EvaluatePerceptionPrimitive(PerceptionPrimitiveID_t id, double value, double weight = STARTING_WEIGHT);
    /*******************************************************************************
    *   CompleteSignature                                                              */
    /*******************************************************************************
    * @brief            Add all classes (if they are already existing
                        descriptors for those classes) to a signature
    * @param sig_max    the signature that receives all classes.
    * @param class_ids  a list of ids that specify classes in the database
    *
    * @throw char* on error
    *******************************************************************************/
    void CompleteSignature(Signature* sig_max, std::vector<ObjectID_t> class_ids);
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
    Elem* FindCreateDescriptor(ObjectID_t class_id);

    // Public attributes
    //

    std::string CheckClass(ObjectID_t id);
    ObjectID_t CheckClass(std::string name);

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


    std::vector<ObjectID_t> m_ids;
    std::vector<std::pair<std::string, ObjectID_t> > m_classes;

    std::vector<std::pair<Signature*, int> > m_currentlyActiveSignatures;
    std::map<int, int> m_activeMap;
    std::map<PerceptionPrimitiveID_t, PerceptionPrimitive*> m_ppMap;
    /**
    *	Helping function to get the first element connected with a class
    */
    int GetElemIdByClass(ObjectID_t ClassID, int index = 0);

  };
}
#endif // SIGNATUREDB_H
