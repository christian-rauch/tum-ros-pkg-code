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


#ifndef ALGORITHMEVAL_H
#define ALGORITHMEVAL_H

#include "Algorithm.h"
#include "LocateAlgorithm.h"
#include "RefineAlgorithm.h"
#include "ProveAlgorithm.h"

#define XML_NODE_ALGORITHMEVAL "AlgorithmEval"
#define XML_NODE_EVAL "Eval"
#define XML_NODE_ALGTYPE "AlgType"
#define XML_NODE_AVGTIME "AvgTime"


#define ALGORITHMTYPE_LOCATE            0x000
#define ALGORITHMTYPE_TRACK             0x100
#define ALGORITHMTYPE_2OBJS             0x200

#define ALGORITHMTYPE_REFINE            0x300
#define ALGORITHMTYPE_RPOVE             0x400

#define ALGORITHMTYPE_STOPTRACK         0x800

#define ALGORITHMSPEC_ONETARGET         0
#define ALGORITHMSPEC_SEVERALTARGET     1

namespace cop
{
  /**
  * template<typename T> class AlgorithmEval
  * @brief Template for saving the evaluation of an algorithm for the algorithm selector, the type refers to the used algorithm
  */
  template<typename T> class AlgorithmEval
  {
  public:
    AlgorithmEval(Algorithm<T>* alg, int type, double eval, double m_avgRunTime) :
      m_algorithm(alg),
      m_algorithmType(type),
      m_eval(eval),
      m_avgRunTime(m_avgRunTime)
    {
      if(alg == NULL)
        printf("Empty Algorithm\n");
    }
      /**
      * Constructor Algorithm Eval
      * @param tag Contains data to load the Evaluation Algorithm.
      * @throws char* with an error message in case of failure
      */
    AlgorithmEval(XMLTag* tag);

    XMLTag* Save(std::string name = "");

    Algorithm<T>*	m_algorithm;
    int				m_algorithmType;
    double			m_eval;
    double			m_avgRunTime;
  };
}
#endif /*ALGORITHMEVAL_H*/
