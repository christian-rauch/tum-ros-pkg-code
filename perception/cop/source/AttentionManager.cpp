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
#include "ImageInputSystem.h"
#include "XMLTag.h"

#ifdef BOOST_THREAD
#include "boost/bind.hpp"
#define BOOST(A) A
#else
#define BOOST(A) 
#endif

#define XML_PROPERTY_ATTENDING "IsAttending"

extern volatile bool g_stopall;
// Constructors/Destructors
//  

AttentionManager::AttentionManager ( XMLTag* /*config*/ , ImageInputSystem& imginsys
#ifdef LOGFILE
                                    , LogFile& log
#endif /*LOGFILE*/
                                    ) :
	m_imginsys(imginsys)
#ifdef LOGFILE
    , m_logFile(log)
#endif /*LOGFILE*/
{
#ifdef BOOST_THREAD
	if(m_Attending)
		m_learningThread = new boost::thread( boost::bind(&AttentionManager::threadfunc, this) ) ;
	else
		m_learningThread = NULL;
#else
#endif

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
}

//  
// Methods
//  


// Accessor methods
//  


// Other methods
//  
void AttentionManager::threadfunc()
{
	while(m_Attending && !g_stopall)
	{
		{
			Camera* cam = m_imginsys.GetCamara(0);
			if(cam != NULL)
			{
				Image* img = cam->GetImage(-1);
				if(img != NULL)
				{
					img->Free();
				}
			}
#ifdef BOOST_THREAD
			//printf("Learning Thread Sleeps \n\n\n");
#ifdef BOOST_1_35 
                      BOOST(boost::system_time t);
#else
                      boost::xtime t;
#endif

#ifdef BOOST_1_35 
                     BOOST(t = get_system_time());
                     BOOST(t += boost::posix_time::seconds(10));
#else
                     boost::xtime_get(&t, boost::TIME_UTC);
                     t.sec += 10;
#endif
                     boost::thread::sleep(t);
#else
      return;
#endif
    }
  }
}


/**
 * @param  pose
 * @param  RoISize
 * @param  eval
 */
void AttentionManager::SetDistractionPoint (RelPose /*pose*/, double /*RoISize*/, double /*eval*/ ) 
{

}


/**
 * @param  attentionLevel
 */
void AttentionManager::SetAttentionLevel (double /*attentionLevel*/, VisFinder* /*visfinder*/ ) {

}

XMLTag* AttentionManager::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_ATTENTIONMANAGER);

	return tag;
}



