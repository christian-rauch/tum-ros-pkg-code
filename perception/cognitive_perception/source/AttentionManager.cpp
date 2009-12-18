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
                        AttentionManager.cpp - Copyright klank


**************************************************************************/

#include "AttentionManager.h"
#include "AttentionAlgorithm.h"
#include "ImageInputSystem.h"
#include "SignatureDB.h"
#include "XMLTag.h"
#include "BoostUtils.h"
#ifdef BOOST_THREAD
#include "boost/bind.hpp"
#define BOOST(A) A
#else
#define BOOST(A)
#endif

#define XML_PROPERTY_ATTENDING "IsAttending"
#define XML_NODE_ATTENTIONALGORITHMS "Attendants"


using namespace cop;
extern volatile bool g_stopall;
// Constructors/Destructors
//

AttentionManager::AttentionManager ( XMLTag* config , SignatureDB& sig_db, ImageInputSystem& imginsys
#ifdef LOGFILE
                                    , LogFile& log
#endif /*LOGFILE*/
                                    ) :
	m_imginsys(imginsys),
#ifdef LOGFILE
		m_attendants(config != NULL ? config->GetChild(XML_NODE_ATTENTIONALGORITHMS) : NULL, log),
    m_logFile(log),
#else  /*LOGFILE*/
    m_attendants(config != NULL ? config->GetChild(XML_NODE_ATTENTIONALGORITHMS) : NULL),
#endif /*LOGFILE*/
    m_sigDB(sig_db)
{
#ifdef BOOST_THREAD
	if(m_Attending)
		m_learningThread = new boost::thread( boost::bind(&AttentionManager::threadfunc, this) ) ;
	else
		m_learningThread = NULL;
#else
#endif

  m_attendants.SetName(XML_NODE_ATTENTIONMANAGER);
}

AttentionManager::~AttentionManager ( )
{
	m_Attending = false;
#ifdef BOOST_THREAD
	if(m_learningThread != NULL)
		m_learningThread->join();
	delete m_learningThread;
#else
#endif

  for(std::vector<AttendedObjects>::iterator it = m_attendedObjectPrototypes.begin();
    it != m_attendedObjectPrototypes.end(); )
  {
    it = m_attendedObjectPrototypes.erase(it);
  }
}

//
// Methods
//


void AttentionManager::SetObjectToAttend (Signature* prototype, RelPose* pointOfInterest, Comm* comm)
{
  AttendedObjects obj;
  obj.comm = comm;
  obj.pose = pointOfInterest;
  obj.sig = prototype;
  m_attendedObjectPrototypes.push_back(obj);
}

void AttentionManager::StopAttend(Comm* comm)
{
  unsigned long actionToStop = comm->GetCommID();
  for(std::vector<AttendedObjects>::iterator it = m_attendedObjectPrototypes.begin();
    it != m_attendedObjectPrototypes.end(); it++)
  {
    if((*it).comm->GetCommID() == actionToStop)
    {
      m_attendedObjectPrototypes.erase(it);
      break;
    }
  }
}

void AttentionManager::threadfunc()
{
	while(m_Attending && !g_stopall)
	{

    for(size_t i = 0 ; i < m_attendedObjectPrototypes .size(); i++)
    {
      Signature* sig = m_attendedObjectPrototypes[i].sig;
      RelPose* area = m_attendedObjectPrototypes[i].pose;
      std::vector<Sensor*> sensors;
      if(area != NULL)
        sensors = m_imginsys.GetBestSensor(*area);
      else
        sensors = m_imginsys.GetAllSensors();


      AttentionAlgorithm* refalg = (AttentionAlgorithm*)m_attendants.BestAlgorithm(0, *sig, sensors);

      if(refalg != NULL)
      {
          printf("Alg selected: %s\n", refalg->GetName().c_str());
          try
          {
            int numOfObjects = 0;
            double qualityMeasure = 0.0;
            std::vector<Signature*> results = refalg->Perform(sensors, area, *sig, numOfObjects, qualityMeasure);
            for(size_t j = 0; j < results.size(); j++)
            {
               m_attendedObjectPrototypes[i].comm->NotifyNewObject(results[j], results[j]->m_relPose);
#ifdef BOOST_THREAD
               m_sigDB.AddAndShowSignatureAsync(results[j], sensors[0]);
#else
               m_sigDB.AddSignature(results[j]);
#endif
            }
          }
          catch(char const* error_text)
          {
            printf("Error in Attention System: %s\n", error_text);

          }
      }
    }
    Sleeping(10);
  }
}




XMLTag* AttentionManager::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_ATTENTIONMANAGER);
  tag->AddChild(m_attendants.Save(XML_NODE_ATTENTIONALGORITHMS));
	return tag;
}


#ifndef WIN32
#include "AlgorithmSelector.hpp"
template class AlgorithmSelector<std::vector<Signature*> >;
#else
#include "AlgorithmSelector.hpp"
template AlgorithmSelector<std::vector<Signature*> >;
#endif



