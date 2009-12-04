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
                        TrackAlgorithm.cpp - Copyright klank


**************************************************************************/

#include "TrackAlgorithm.h"
#ifdef BOOST_THREAD
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#endif
extern volatile bool g_stopall;

#ifdef BOOST_THREAD
//#include <boost/placeholders.hpp>
#define BOOST(A) A
#else
#define BOOST(A)
#endif

using namespace cop;


// Constructors/Destructors
//

TrackAlgorithm::TrackAlgorithm (Signature& sig, Algorithm<std::vector<RelPose*> > *alg,
                               ImageInputSystem& imageSys) :
    m_Running(true),
    m_alg(alg),
    m_curObject(sig),
    m_imageSys( imageSys)
{
#ifdef BOOST_THREAD
    m_trackingThread = new boost::thread( boost::bind(&TrackAlgorithm::threadfunc, this));
#else
  TrackAlgorithm::threadfunc();
#endif
}


TrackAlgorithm::~TrackAlgorithm ( )
{
    /* Stop Thread:*/
    m_Running = false;
#ifdef BOOST_THREAD
  if(m_trackingThread != NULL)
    m_trackingThread->join();
    delete m_trackingThread;
#endif
    /* Wait for termination and continue destroying*/
}

//
// Methods
//
//
// Methods
//
 void TrackAlgorithm::threadfunc()
{
#ifdef BOOST_THREAD
#ifdef BOOST_1_35
    boost::system_time t1, t2;
#else
    boost::xtime t1, t2;
#endif
#endif
  while(m_Running && !g_stopall)
  {
#ifdef BOOST_THREAD
#ifdef BOOST_1_35
  BOOST(t1 = get_system_time());
#else
  boost::xtime_get(&t1, boost::TIME_UTC);
#endif
#endif
    RelPose* lastKnownPose = m_curObject.m_relPose;
    int numOfObjects = 1;
    try
    {
      std::vector<Sensor*> sensors = m_imageSys.GetBestSensor(*lastKnownPose);
#ifdef _DEBUG
      printf("Algorithm for track: %s (%p)\n",m_alg != NULL ? m_alg->GetName().c_str() : "NoName", m_alg );
#endif
      if(m_alg != NULL)
      {
        double qualityMeasure = 0.0;
#ifdef BOOST_THREAD
#ifdef BOOST_1_35
         boost::system_time t0, t1;
#else
         boost::xtime t0, t1;
#endif
#ifdef BOOST_1_35
         BOOST(t0 = get_system_time());
#else
         boost::xtime_get(&t0, boost::TIME_UTC);
#endif
#endif

         std::vector<RelPose*> pose = m_alg->Perform(sensors, lastKnownPose, m_curObject, numOfObjects, qualityMeasure);
#ifdef BOOST_THREAD
#ifdef BOOST_1_35
         BOOST(t1 = get_system_time());
#else
         boost::xtime_get(&t1, boost::TIME_UTC);
#endif

#ifdef _DEBUG
#ifdef BOOST_1_35
      /*std::cout << "Calc time: " << t1 - t0 << std::endl;*/
#else
        printf("Calc time: %ld ns\n", ((1000000000 * (t1.sec - t0.sec))+(t1.nsec - t0.nsec)));
#endif
#endif
#endif
        if(pose.size() > 0)
        {
          pose[0]->m_qualityMeasure = qualityMeasure;
          m_curObject.SetPose(pose[0]);
        }
        else
        {
           m_curObject.SetPose(NULL);
        }
      }
    }
    catch(char const* message)
    {
      printf("Error in Locate: %s\n", message);
    }
#ifdef BOOST_THREAD
    //printf("Learning Thread Sleeps \n\n\n");
#ifdef BOOST_1_35
  BOOST(t2 = boost::get_system_time());
  t2 += boost::posix_time::millisec(1);
#else
  boost::xtime_get(&t2, boost::TIME_UTC);
  t2.nsec += 1000;
#endif
    boost::thread::sleep(t2);
#else
    return;
#endif
  }
}

XMLTag* TrackAlgorithm::Save()
{
    return NULL;
}

