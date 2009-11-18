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
                        AttentionManager.h - Copyright klank


**************************************************************************/


#ifndef ATTENTIONMANAGER_H
#define ATTENTIONMANAGER_H

#include <string>
#include <vector>
#include "XMLTag.h"
#include "LogFile.h"

#ifdef BOOST_THREAD
#include "boost/thread.hpp"
#else
#endif
#define XML_NODE_ATTENTIONMANAGER "AttentionManager"
namespace cop
{
  class VisFinder;
  class ImageInputSystem;
  /**
    * class AttentionManager
    *	* @brief manages the attention of the system: if their is distraction or not
    */
  class AttentionManager
  {
  public:
    /**
     * Constructor
     */
    AttentionManager ( XMLTag* config , ImageInputSystem& imginsys
  #ifdef LOGFILE
          , LogFile& log
  #endif /*LOGFILE*/
          );

    /**
     * Destructor
     */
    virtual ~AttentionManager ( );

    /**
     * @param  pose
     * @param  RoISize
     * @param  eval
     */
    void SetDistractionPoint (RelPose pose, double RoISize, double eval );

    XMLTag* Save();
    /**
     * @param  attentionLevel
     */
    void SetAttentionLevel (double attentionLevel, VisFinder* visfinder);


      void threadfunc();
  #ifdef BOOST_THREAD
    boost::thread* m_learningThread;
  #else
  #endif
    bool m_Attending;
      ImageInputSystem&   m_imginsys;
  #ifdef LOGFILE
      LogFile&            m_logFile;
  #endif /*LOGFILE*/

  private:
    AttentionManager& operator=(AttentionManager&){throw "Error";}
  };
}
#endif // ATTENTIONMANAGER_H
