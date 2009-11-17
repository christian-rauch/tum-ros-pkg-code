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
                        VisLearner.cpp - Copyright klank

**************************************************************************/

#include "VisLearner.h"
#include "ShapeModel.h"
#include "ColorModel.h"

extern volatile bool g_stopall;

#ifdef BOOST_THREAD
#include "boost/bind.hpp"
#define BOOST(A) A
#else
#define BOOST(A)
#endif


#define XML_NODE_REFINEMENTALGORITHMS "RefineAlgs"
#define XML_NODE_PROVINGALGORITHMS "ProveAlgs"

bool VisLearner::s_Learning = false;
bool VisLearner::s_Running = true;
// Constructors/Destructors
//

VisLearner::VisLearner ( XMLTag* tag, SignatureDB& sigDB, ImageInputSystem& imgSys
#ifdef LOGFILE
            ,LogFile& log
#endif /*LOGFILE*/
            , bool bLearning) :
		m_signatureDB(sigDB),
		m_imageSys(imgSys),
		m_stats(tag != NULL ? tag->GetChild(XML_NODE_STATISTICS) : NULL)
#ifdef LOGFILE
		,m_refinements(tag != NULL ? tag->GetChild(XML_NODE_REFINEMENTALGORITHMS) : NULL, log)
		,m_checks(tag != NULL ? tag->GetChild(XML_NODE_PROVINGALGORITHMS): NULL, log)
#else  /*LOGFILE*/
    ,m_refinements(tag != NULL ? tag->GetChild(XML_NODE_REFINEMENTALGORITHMS) : NULL)
		,m_checks(tag != NULL ? tag->GetChild(XML_NODE_PROVINGALGORITHMS): NULL)
#endif /*LOGFILE*/
{
#ifdef BOOST_THREAD
  bLearning = false;
	if(bLearning)
		m_learningThread = new boost::thread( boost::bind(&VisLearner::threadfunc, this) ) ;
	else
		m_learningThread = NULL;
#else
#endif
       m_refinements.SetName(XML_NODE_VISLEARNER);
       m_checks.SetName(XML_NODE_VISLEARNER);
}

VisLearner::~VisLearner ( )
{
	s_Running = false;
#ifdef BOOST_THREAD
	if(m_learningThread != NULL)
		m_learningThread->join();
	delete m_learningThread;
#else
#endif
}

//
// Methods
//
 void VisLearner::threadfunc()
{

	while(VisLearner::s_Running && !g_stopall)
	{
		size_t n = m_taskList.size();
		if(n > 0)
		{
			s_Learning = true;
			TaskID type = m_taskList[0].first;
			Signature* sig = m_taskList[0].second;
			std::vector<Camera*> cam;
			int numOfObjects = 1;
			double qualityMeasure = 0.0;
#ifdef _DEBUG
			printf("New Learning Task\n");
#endif /*_DEBUG*/
			RefineAlgorithm* refalg = (RefineAlgorithm*)m_refinements.BestAlgorithm(type, *sig);

			if(refalg != NULL)
			{
          printf("Alg selected: %s\n", refalg->GetName().c_str());
          try
          {
				    Descriptor* d = refalg->Perform(cam, sig->m_relPose, *sig, numOfObjects, qualityMeasure);
				    sig->SetElem(d);
                }
                catch(char const* error_text)
                {
                  printf("Refinement failed due to: %s\n", error_text);
                }
            }
			else
			{
				ProveAlgorithm* provalg = (ProveAlgorithm*)m_checks.BestAlgorithm(type, *sig);
				if(provalg != NULL)
				{
          printf("Alg selected: %s\n", refalg->GetName().c_str());
          try
          {
					  double d = provalg->Perform(cam, sig->m_relPose, *sig, numOfObjects, qualityMeasure);
#ifdef _DEBUG
					  printf("Evaluation results in: %f\n", d);
#endif /*_DEBUG*/
				  	if(d > 0.5)
					  {
#ifdef _DEBUG
						  printf("This is good!\n");
#endif /*_DEBUG*/
					  }
           }
           catch(char const* error_text)
           {
#ifdef _DEBUG
             printf("Evaluation failed due to: %s\n", error_text);
#endif /*_DEBUG*/
           }
				}
			}
			m_taskList.erase(m_taskList.begin());
			s_Learning = false;
		}
		else
		{
#ifdef BOOST_THREAD
			//printf("Learning Thread Sleeps \n\n\n");
			#ifdef BOOST_1_35
  BOOST(boost::system_time t);
#else
  boost::xtime t;
#endif

#ifdef BOOST_1_35
      BOOST(t = get_system_time());
      t += boost::posix_time::seconds(1);
#else
      boost::xtime_get(&t, boost::TIME_UTC);
			t.sec += 1;
#endif
  cout << ",";
			boost::thread::sleep(t);
#else
			return;
#endif
		}
	}
}

void VisLearner::AddAlgorithm(Algorithm<Descriptor*>* alg)
{
	m_refinements.AddAlgorithm(alg, ELEM, 1.0, 0.0);
}
void VisLearner::AddAlgorithm(Algorithm<double>* alg)
{
	m_checks.AddAlgorithm(alg, ELEM, 1.0, 0.0);
}

/**
 * @return SignatureLocations_t
 * @param  lastKnownPoses
 * @param  object
 * @param  numOfObjects
 */
SignatureLocations_t VisLearner::RefineObject (PossibleLocations_t* lastKnownPoses, Signature& sig, int &numOfObjects)
{
  SignatureLocations_t ret_vec;
  TaskID type = 0x100;
  RefineAlgorithm* refalg = (RefineAlgorithm*)m_refinements.BestAlgorithm(type, sig);
  PossibleLocations_t::const_iterator it = (*lastKnownPoses).begin();
  for(;it!=(*lastKnownPoses).end(); it++)
  {
    RelPose* lastKnownPose = (*it).first;
    std::vector<Camera*> cameras;
    unsigned int nCamera;
    try
    {
      int offset = 0;
      while(true)
      {
          Camera* cam = m_imageSys.GetBestCamera(*lastKnownPose, nCamera, offset);
          if(cam == NULL)
              break;
          cameras.push_back(cam);
          offset++;
      }
    }
    catch(const char* text)
    {
      printf ("Error selecting a camera: %s\n", text);
      continue;
    }
    if(refalg != NULL)
    {
        printf("Alg selected: %s\n", refalg->GetName().c_str());
        try
        {
          double qualityMeasure;
          Descriptor* d = refalg->Perform(cameras, lastKnownPose, sig, numOfObjects, qualityMeasure);
          sig.SetElem(d);
          lastKnownPose->m_qualityMeasure = qualityMeasure;
          ret_vec.push_back(std::pair<RelPose*, Signature*>(lastKnownPose, &sig));
        }
        catch(char const* error_text)
        {
          printf("Refinement failed due to: %s\n", error_text);
        }
    }
    else
    {
      ProveAlgorithm* provalg = (ProveAlgorithm*)m_checks.BestAlgorithm(type, sig);
      if(provalg != NULL)
      {
        printf("Alg selected: %s\n", refalg->GetName().c_str());
        try
        {
          double qualityMeasure;
          double d = provalg->Perform(cameras, lastKnownPose, sig, numOfObjects, qualityMeasure);
  #ifdef _DEBUG
          printf("Evaluation results in: %f\n", d);
  #endif /*_DEBUG*/
          if(d > 0.5)
          {
  #ifdef _DEBUG
            printf("This is good!\n");
  #endif /*_DEBUG*/
          }
         }
         catch(char const* error_text)
         {
  #ifdef _DEBUG
           printf("Evaluation failed due to: %s\n", error_text);
  #endif /*_DEBUG*/
         }
      }
    }
  }
	return ret_vec;
}


/**
 * @return double
 * @param  ObjectWith3dShapeModel
 * @param  EstimatedPose
 * @param  index
 */
double VisLearner::RefineObject (Signature& sig)
{
	if(s_Learning == true)
		return 0.0;
	TaskID id = ELEM;

	m_taskList.push_back(std::pair<TaskID, Signature*>
			(id, &sig));

#ifndef BOOST_THREAD
	threadfunc();
#endif
	return m_taskList.size();
}



XMLTag* VisLearner::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_VISLEARNER);
	tag->AddChild(m_stats.Save());
	tag->AddChild(m_refinements.Save(XML_NODE_REFINEMENTALGORITHMS));
	tag->AddChild(m_checks.Save(XML_NODE_PROVINGALGORITHMS));
	return tag;
}

/**
 * @return Signature
 * @param  index
 */
Signature* VisLearner::GetObjectSignature (int index )
{
	return NULL;
}

#ifndef WIN32
#include "AlgorithmSelector.hpp"
template class AlgorithmSelector<Descriptor*>;
template class AlgorithmSelector<double>;
#else
#include "AlgorithmSelector.hpp"
template class AlgorithmSelector<Descriptor*>;
template AlgorithmSelector<double>;
#endif
