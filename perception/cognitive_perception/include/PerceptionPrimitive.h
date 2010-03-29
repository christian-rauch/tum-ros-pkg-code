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
                        PerceptionPrimitive.h - Copyright klank


**************************************************************************/

#ifndef PERCEPTIONPRIMITIVE_H
#define PERCEPTIONPRIMITIVE_H


#include "Signature.h"
#include "AlgorithmEval.h"
#include "Sensor.h"

namespace cop
{
  enum PerceptionPrimitiveState
  {
    PP_STARTED,
    PP_TERMINATED,
    PP_EVALUATED,
    PP_DELETABLE
  };
  /************************************************************************
  *  class PerceptionPrimitive
  *  @brief Describes the entity created by each call of cop in order
  *         to answer a query and enable later evaluation
  **************************************************************************/
  class PerceptionPrimitive
  {
  public:
    PerceptionPrimitive(Signature* sig) :
      m_evaluation(0),
      m_timing(0),
      m_count(0),
      m_uniqueID(m_lastID++),
      m_currState(PP_STARTED)
    {
      m_signatures.push_back(sig);
    }

    Signature* GetSignature(size_t index = 0){return m_signatures[index];}
    PerceptionPrimitiveID_t GetID(){return m_uniqueID;}

    void AddResult(ObjectID_t id, double quality =  1.0, unsigned long calctime = 0)
    {
      m_results.push_back(id);
      m_evaluation += quality;
      m_timing += calctime;
      m_count++;
    }

    void SetTerminated()
    {
      if(m_currState == PP_EVALUATED)
        m_currState = PP_DELETABLE;
      else if (m_currState == PP_TERMINATED)
        printf("State transition from PP_TERMINATED to PP_TERMINATED\n");
      else
        m_currState = PP_TERMINATED;
    }

    void SetEvaluated()
    {
       if(m_currState == PP_TERMINATED)
        m_currState = PP_DELETABLE;
      else
        m_currState = PP_EVALUATED;
    }

    PerceptionPrimitiveState GetCurrState(){return m_currState;}
    std::vector<Signature*> m_signatures;
    std::vector<unsigned long> m_AlgorithmIDs;
    std::vector<Sensor*> m_sensors;

    std::vector<ObjectID_t> m_results;
    double        m_evaluation;
    unsigned long m_timing;
    unsigned long m_count;
  private:
    PerceptionPrimitiveID_t m_uniqueID;
    static PerceptionPrimitiveID_t m_lastID;
    PerceptionPrimitiveState m_currState;
  };
}
#endif /* PERCEPTIONPRIMITIVE_H*/
