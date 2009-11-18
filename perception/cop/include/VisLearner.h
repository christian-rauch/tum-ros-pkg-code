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
                        VisLearner.h - Copyright klank

**************************************************************************/


#ifndef VISLEARNER_H
#define VISLEARNER_H

#include <string>
#include <vector>
#include "SignatureDB.h"
#include "ImageInputSystem.h"
#include "Statistics.h"
#include "RefineAlgorithm.h"
#include "AlgorithmSelector.h"
#ifdef BOOST_THREAD
#include "boost/thread.hpp"
#else
#endif



#define XML_NODE_VISLEARNER "VisLearner"
namespace cop
{
  typedef int TaskID;

  /**
   * class VisLearner
   *  @brief Basic class for improving model knowledge
    */
  class VisLearner
  {
  public:

    // Constructors/Destructors
    //

    /**
    * Constructor
    */
    VisLearner ( XMLTag* tag, SignatureDB& sigDB, ImageInputSystem& imgSys,
  #ifdef LOGFILE
         LogFile& log,
  #endif /*LOGFILE*/
          bool bLearning = true);


    /**
    * Empty Destructor
    */
    virtual ~VisLearner ( );
  private:
    VisLearner();

    // Methods
    //
  public:
    XMLTag* Save();
    /**
    * @return double
    * @param  ObjectWith3dShapeModel
    */
    double RefineObject (Signature& ObjectWith3dShapeModel);


    /**
    * @return lastKnownPoses
    * @param  ObjectWith3dShapeModel
    */
    SignatureLocations_t RefineObject (PossibleLocations_t* lastKnownPoses, Signature& object, int &numOfObjects);



    void AddAlgorithm(Algorithm<Descriptor*>*);
    void AddAlgorithm(Algorithm<double>*);

    /**
     * @return Signature*
     * @param  index
     */
    Signature* GetObjectSignature (int index );


    void threadfunc();
  private:

    // Private attributes
    //
    SignatureDB&		m_signatureDB;
    ImageInputSystem&	m_imageSys;
    Statistics			m_stats;


      std::vector<std::pair<TaskID, Signature*> > m_taskList;
    AlgorithmSelector<Descriptor*>	m_refinements;
    AlgorithmSelector<double>		m_checks;
  #ifdef BOOST_THREAD
    boost::thread* m_learningThread;
  #else
  #endif
    static bool s_Learning;
    static bool s_Running;
  private:
    VisLearner& operator=(VisLearner&){throw "Error";}

  };
}
#endif // VISLEARNER_H
