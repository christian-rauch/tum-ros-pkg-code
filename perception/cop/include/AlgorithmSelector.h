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
                        AlgorithmSelector.h - Copyright klank


**************************************************************************/


#ifndef ALGORITHMSELECTOR_H
#define ALGORITHMSELECTOR_H

#include <string>
#include <vector>

#include "AlgorithmEval.h"
#ifdef LOGFILE
#include "LogFile.h"
#define STD_LOGFILENAME "AlgActions.log"
#endif /*LOGFILE*/

/**
*	typedefs:
*/
typedef double Probability_1D_t; /* < Type for storing Probability of a location to occur*/
typedef std::vector<std::pair<RelPose*, Probability_1D_t> > PossibleLocations_t;/* < Type for a list of relposes with their a-priori probability */
typedef std::vector<std::pair<RelPose*, Signature*> > SignatureLocations_t;

/***/

#define XML_NODE_ALGORITHMSELECTOR "AlgorithmSelector"

/**
  * class AlgorithmSelector
  *	@brief Provides an interface for selecting an optmial algorithm
  */
template<typename T>
class AlgorithmSelector
{
public:
  // Constructors/Destructors
  //
  /**
   * Constructor
   */
    AlgorithmSelector(
#ifdef LOGFILE
        LogFile& log
        ) : m_logfile(log)
#else /*LOGFILE*/
        )
#endif /*LOGFILE*/
        {}
	AlgorithmSelector (XMLTag* node
#ifdef LOGFILE
        , LogFile& log
#endif /*LOGFILE*/
        );
  /**
   * Empty Destructor
   */
  ~AlgorithmSelector ( ){}
	/**
	*	AddAlgorithm
	*	@brief
	*	@param alg	A pointer to a algorithm class
	*	@param nType	the algorithm type (e.g. )
	*	@param dEval
	*	@return the index in the algorithm list
	*/
	int AddAlgorithm(Algorithm<T>* alg, int nType, double dEval, double dTime);


	/**
	*	Save
	*	@return a XMLTag* that saves the evaluation of the algorithms and the algorithms themselves
	*	@param name sets the name of the node returned
	*/
	XMLTag* Save(std::string name = "");
	/**
	*	BestAlgorithm
	*	@brief Selects from a list the best algorithm
	*	@param type specifies the type of algorithm that should be searched, normally a multiple of 0x100 modulo a special case
	*	@param sig specifies the signature that has to be searched
	*/
	Algorithm<T>* BestAlgorithm(int type, Signature& sig);
	/**
	*	Sets the evaluation for an algorithm
	*/
	void EvalAlgorithm(Algorithm<T>* alg, double eval, double time, Elem* relatedElemg);
  /**
  *	Return num of Algorithms in the visual finder
  */
  int CountAlgorithms(){return m_algorithmlist.size();}
    /**
    *   SetCaller information for logginf info
    */
    std::string GetName(){return caller_name;}
    void SetName(std::string name){caller_name = name;}



protected:
	/**
	*	Insets in the list of a special type of algorithms
	**/
	int InsertInList(AlgorithmEval<T> eval);
	/**
	*	Get the correct algorithm list dependand on the type t
	*/
	std::vector<AlgorithmEval<T> > GetAlgorithmList(std::vector<AlgorithmEval<T> >*) ;

	/**
	*	test compatibility of an algorithm with a task
	*/
	bool CheckTypeCompatibility(int listedType, int askedType);


	/**
	* Get the value of m_algorithmlist
	* @return the value of m_algorithmlist
	*/
	Algorithm<T>* getAlgorithm(int index);

private:
    /**
    *   List of Algorithms, including their evaluation
    */
	std::vector<AlgorithmEval<T> > m_algorithmlist;

    /**
    *   LogFile
    */
#ifdef LOGFILE
    LogFile& m_logfile;
#endif /*LOGFILE*/
    std::string caller_name;
};

#endif // ALGORITHMSELECTOR_H
