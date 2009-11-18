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


#ifndef USE_YARP_COMM /*Only this or Yarp*/

#ifndef ROSCOMM_H
#define ROSCOMM_H

#include "RelPose.h"
#include "VisFinder.h"
#include "Signature.h"
#include "Comm.h"


#include <map>

#include <vision_srvs/cop_call.h>
#include <ros/ros.h>

#define STD_COP_OUTPUT_PORT "/tracking/out"

#ifdef BOOST_THREAD
#include <boost/thread.hpp>
#include <boost/bind.hpp>
using namespace boost;
#endif
namespace cop
{
  /*****************************************************************************************
  *  class ROSComm                                                                        */
  /*****************************************************************************************
  *   This class implements a ros service that answers cop_querys
  ******************************************************************************************/
  class ROSComm : public Comm
  {
  public:
    ROSComm(VisFinder& visFinder, PossibleLocations_t* pose, Signature& sig, ros::Publisher * pub, int numOfObjects, int actionType) :
      m_visFinder(visFinder),
      m_pose(pose),
      m_sig(sig),
      m_publisher(pub),
      m_numOfObjects(numOfObjects),
      m_actionType(actionType)
    {
    }

    ~ROSComm();

    /*****************************************************************************************
    *  NotifyPoseUpdate                                                                        */
    /*****************************************************************************************
    *  Call back that is called whenever a new pose is set for a certain model
    *  This callback must be told the signature that is tracked
    ******************************************************************************************/
    virtual void NotifyPoseUpdate(RelPose* pose, bool sendObjectRelation = true);

    /*****************************************************************************************
    *  Start                                                                                */
    /*****************************************************************************************
    *  Calls the yarp threadfunc, with a new thread if this is possible
    ******************************************************************************************/
    void Start();

    /*****************************************************************************************
    *  threadfunc                                                                           */
    /*****************************************************************************************
    *  Action: calls visual finder, and writes the results on a topic
    ******************************************************************************************/
    void threadfunc();

    /*yarp::os::BufferedPort<yarp::os::Bottle>* m_port;*/
    VisFinder& m_visFinder;
    PossibleLocations_t* m_pose;
    Signature& m_sig;
    ros::Publisher* m_publisher;
    int m_numOfObjects;
    int m_actionType;
  };

  class ROSTopicManager
  {
  public:
    ROSTopicManager(VisFinder* s_visFinder, SignatureDB *s_sigDb);
    ~ROSTopicManager();

    void CloseROSTopic(std::string name);

    void Listen(std::string name, volatile bool &g_stopall, ros::NodeHandle* node);
    bool ListenCallBack(vision_srvs::cop_call::Request& request, vision_srvs::cop_call::Response&  answer);
    /*void ListenCallBack(const boost::shared_ptr<const cop::cop_call> &msg);*/
    bool OpenCommOnROSTopic(std::string st);

    std::map<std::string, ros::Publisher*> m_openTopics;
    VisFinder& m_visFinder;
    SignatureDB& m_sig;
  };
}
#endif /* ROSCOMM_H */

#endif /*USE_YARP_COMM */
