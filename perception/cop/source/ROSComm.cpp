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

 
#ifndef USE_YARP_COMM


#include <vision_msgs/cop_answer.h>
#include <vision_srvs/cop_call.h>
using namespace vision_msgs;
using namespace vision_srvs;

#include <ros/ros.h>

#include "ROSComm.h"

#ifdef BOOST_THREAD
#include <boost/thread/mutex.hpp>
boost::mutex s_mutexAnswer;
#define BOOST(A) A
#else
#define BOOST (A) ;
#endif

typedef double Probability_1D_t;

/********************************************************************
*     CreatePoseFromMessage                                         */
/********************************************************************
*   @brief Creates a lo from a bottle
*********************************************************************/
std::pair<RelPose*, Probability_1D_t> CreatePoseFromMessage(const apriori_position& call)
{
    std::pair<RelPose*, Probability_1D_t> result;
    result.second = call.probability;
    result.first = RelPoseFactory::FRelPose(call.positionId);
    if(result.first == NULL)
    {
      printf("ROSComm: Major Error in Message: got an undefined pose\n");
      throw "Wrong Pose";
    }
    return result;
}


void PutPoseIntoAMessage(cop_answer &answer, SignatureLocations_t new_location)
{
    for(unsigned int i = 0; i < new_location.size(); i++) /**TODO, replace loicate and ...*/
    {
      aposteriori_position apo_pose;
      apo_pose.objectId = new_location[i].second->m_ID;
      apo_pose.probability = new_location[i].first->m_qualityMeasure;
      apo_pose.position = new_location[i].first->m_uniqueID;
      int classes_c = new_location[i].second->CountClasses();
      for(int j = 0; j < classes_c; j++)
      {
        printf("added Class %d\n", j);
        Class* cl = new_location[i].second->GetClass(j);
        std::string st =  cl->GetName();
        printf("%s\n", st.c_str());
        if(cl != NULL)
          apo_pose.classes.push_back(st);
        printf("?\n");
      }
      printf("..\n");
      answer.found_poses.push_back(apo_pose);
    }

}

void ROSComm::NotifyPoseUpdate(RelPose* pose, bool sendObjectRelation)
{
  BOOST(s_mutexAnswer.lock());
  cop_answer answer;
  if(pose != NULL)
  {
    aposteriori_position ap_pose;
    ap_pose.objectId = m_sig.m_ID;
    ap_pose.probability = pose->m_qualityMeasure;
    ap_pose.position = pose->m_uniqueID;
    answer.found_poses.push_back(ap_pose);
  }
  else
  {
      answer.error = "Object lost";
  }
  m_publisher->publish(answer);
#ifdef _DEBUG
  /*printf("Writing a bottle in Pose Update Notification:  %s\n", new_loc_bottle.toString().c_str());*/
#endif /*_DEBUG*/
  BOOST(s_mutexAnswer.unlock());
}


ROSComm::~ROSComm()
{
  /*for(int i = 0; i < m_pose->size(); i++)
    RelPoseFactory::FreeRelPose((*m_pose)[i].first);*/
  delete m_pose;
}

void ROSComm::Start()
{
#ifdef BOOST_THREAD
  boost::thread( boost::bind(&ROSComm::threadfunc, this));
#else
  ROSComm::threadfunc(this);
#endif /*BOOST_THREAD*/
}

void ROSComm::threadfunc()
{
#ifdef _DEBUG
    printf("Answer Thread started for object %d command %s\n", m_sig.m_ID, ((m_actionType == ALGORITHMTYPE_LOCATE) ? "Locate" : ((m_actionType == ALGORITHMTYPE_TRACK) ? "Track" : "StopTrack or unknown action")));
#endif
    bool bFinished = false;
    cop_answer answer;

    switch(m_actionType)
    {
    case ALGORITHMTYPE_REFINE:
        try
        {
          const SignatureLocations_t &new_location = m_visFinder.m_visLearner.RefineObject(m_pose, m_sig, m_numOfObjects);
          BOOST(s_mutexAnswer.lock());
          if(m_numOfObjects > 0 && new_location.size() > 0)
          {
                  PutPoseIntoAMessage(answer, new_location);
          }
          else
          {
            answer.error = "No Object Found!";
          }
          m_publisher->publish(answer);
          BOOST(s_mutexAnswer.unlock());
        }
        catch(...)
        {
          BOOST(s_mutexAnswer.lock());
          answer.error = "Locate failed";
          m_publisher->publish(answer);
          BOOST(s_mutexAnswer.unlock());
        }
        break;
    case ALGORITHMTYPE_TRACK:
       m_sig.SetCommCallBack(this);
        if(m_pose->size() > 0)
        {
          PossibleLocations_t::const_iterator it = m_pose->begin();
          m_visFinder.StartTrack(m_sig, (*it).first);
        }
        else
          m_visFinder.StartTrack(m_sig, NULL);
       break;
    case ALGORITHMTYPE_STOPTRACK:
        m_sig.SetCommCallBack(NULL);
        m_visFinder.StopTrack(m_sig);
        bFinished = true;
        break;
    case ALGORITHMTYPE_LOCATE:
        {
        try
        {
          const SignatureLocations_t &new_location = m_visFinder.Locate(m_pose, m_sig, m_numOfObjects);
        BOOST(s_mutexAnswer.lock());
          if(m_numOfObjects > 0 && new_location.size() > 0)
          {
                  PutPoseIntoAMessage(answer, new_location);
          }
          else
          {
            answer.error = "No Object Found!";
          }
          m_publisher->publish(answer);
          BOOST(s_mutexAnswer.unlock());

#ifdef WIN32
          BOOST(boost::system_time t);

          BOOST(t = get_system_time());
          t +=  boost::posix_time::seconds(1);  //TODO Check

          boost::thread::sleep(t);
          delete m_visFinder.m_imageSys.GetCamara(0)->m_win;
          m_visFinder.m_imageSys.GetCamara(0)->m_win = NULL;
#endif
        }
        catch(char const* text)
        {
          printf("Locate called by ros failed: %s\n", text);
          BOOST(s_mutexAnswer.lock());
          answer.error = "Locate failed";
          m_publisher->publish(answer);
          BOOST(s_mutexAnswer.unlock());
          return;
        }
        catch(...)
        {
          BOOST(s_mutexAnswer.lock());
          answer.error = "Locate failed";
          m_publisher->publish(answer);
          BOOST(s_mutexAnswer.unlock());
         break;
       }
       bFinished = true;
      }
      break;
    default:
          BOOST(s_mutexAnswer.lock());
          answer.error = "Locate failed";
          m_publisher->publish(answer);
          BOOST(s_mutexAnswer.unlock());
       bFinished = true;
      break;    }
#ifdef _DEBUG
    printf("Finished  with action of type \"%s\".\nReturning to listen loop.\n", ((m_actionType == ALGORITHMTYPE_LOCATE) ? "Locate" : ((m_actionType == ALGORITHMTYPE_TRACK) ? "Track" : "StopTrack or unknown action")) );
#endif
    /*TODO delete this*/
    if(bFinished)
      delete this;

}


ROSTopicManager::ROSTopicManager(VisFinder* visFinder, SignatureDB *sigDb) :
  m_visFinder(*visFinder),
  m_sig(*sigDb)
{
}

ROSTopicManager:: ~ROSTopicManager()
{
  for(std::map<std::string, ros::Publisher*>::iterator it = m_openTopics.begin(); it != m_openTopics.end(); it++)
  {
    delete (*it).second;
  }
}

void ROSTopicManager::CloseROSTopic(std::string name)
{
}

/*void ROSTopicManager::ListenCallBack(const boost::shared_ptr<const cop_call> &msg)*/
bool ROSTopicManager::ListenCallBack(cop_call::Request& msg, cop_call::Response&  answer)
{
#ifdef _DEBUG
  printf("Entering ROSTopicManager::ListenCallBack with find %p sigdb %p\n", &(m_visFinder), &(m_sig));
  printf("Got Message: noo %ld \n", msg.number_of_objects);
#endif
  std::vector<int> class_id;
  std::string topicname = msg.outputtopic.length() == 0 ? STD_COP_OUTPUT_PORT : msg.outputtopic;
  Signature* sig = NULL;
  PossibleLocations_t* poses = new PossibleLocations_t();

  for(unsigned int i = 0; i < msg.object_classes.size(); i++)
  {
    try
    {
      int class_from_string = m_sig.CheckClass(msg.object_classes[i]);
#ifdef _DEBUG
      printf("  Class as string: %s -> %d\n", msg.object_classes[i].c_str(), class_from_string);
#endif
      if(class_from_string != -1)
        class_id.push_back(class_from_string);
      else
      {
        Class cl;
        cl.SetName(msg.object_classes[i]);
        m_sig.AddClass(cl.GetName(), cl.m_ID);
        int class_from_string = m_sig.CheckClass(msg.object_classes[i]);
        if(class_from_string != -1)
        {
          class_id.push_back(class_from_string);
          printf("Added Class %s as %ldth element\n", msg.object_classes[i].c_str(), class_id.size());
        }

      }
    }
    catch(...)
    {
        printf("ROSComm: Problems reading classes\n");
        return false;
    }
  }
  for(unsigned int j = 0; j < msg.object_ids.size(); j++)
  {
    try
    {
        int index;
        int id = msg.object_ids[j];
        if(m_sig.CheckClass(id).length() > 0)
        {
          class_id.push_back(id);
        }
        else if(m_sig.Check(id, index))
        {
          sig = m_sig.GetSignatureByID(id);
        }
        else
        {
          printf("Received unknown Element id: %d\n", id);
        }
    }
    catch(...)
    {
        printf("ROSComm: Problems reading classes\n");
        return false;
    }
  }
  if(sig == NULL)
  {
     try
     {
        sig = m_sig.GetSignature(class_id);
        if(sig == NULL)
        {
          printf("ROSComm: Could not generate signature for the requested description\n");
        return false;
        }
     }
     catch(char const* text)
     {
       printf("Error Creating signature: %s\n", text);
        return false;
     }
     catch(...)
     {
       printf("Error Creating signature\n");
        return false;
     }
  }
  #ifdef _DEBUG
  printf("Created Signature with all classes (%d)\n", sig->m_ID);
  #endif
  for(unsigned int k = 0; k < msg.list_of_poses.size(); k++)
  {
    try
    {
      poses->push_back(CreatePoseFromMessage(msg.list_of_poses[k]));
    }
    catch(char* txt)
    {
      continue;
    }
  }

  if(m_openTopics.find(topicname) == m_openTopics.end())
  {
    ros::NodeHandle n;
    printf("Publisher pub = n.advertise<ccop_answer>(%s, 1000)\n", topicname.c_str());
    ros::Publisher* pub= new ros::Publisher();
    *pub = n.advertise<cop_answer>(topicname, 5);
/*     ros::Rate r(1);
     r.sleep();
    cop_answer answer;
     answer.error = "No Object Found!";
     pub->publish(answer);
     printf("Published?\n");*/
     m_openTopics[topicname] = pub;
  }

  ROSComm* comm = new ROSComm(m_visFinder, poses, *sig, m_openTopics[topicname], (int)msg.number_of_objects, (int)msg.action_type);
  comm->Start();
  return true;
}


void ROSTopicManager::Listen(std::string name, volatile bool &g_stopall, ros::NodeHandle* node)
{
  /**
   * Now we subscribe using the normal method, to demonstrate the difference.
   */
  /*ros::Publisher pub = n.advertise<cop_call>(name, 1000);*/

#ifdef ROS_ACTIONLIB
#else
  printf("advertiseService<cop_call> (%s, ...)\n", name.c_str());
  ros::ServiceServer srv = node->advertiseService("/tracking/in", &ROSTopicManager::ListenCallBack, this);
#endif
  /*ros::Subscriber sub2 = n.subscribe<cop_call>(name, 1000, &ROSTopicManager::ListenCallBack, this_p);
  printf("Topic check: %s\n", sub2.getTopic().c_str());*/
  /**
   * Now do a custom spin, to demonstrate the difference.
  */
  /*ros::spin();*/
  ros::Rate r(10 );
  while (node->ok() && !g_stopall)
  {
    printf("Call ros spin \n");
    try
    {
      ros::spin();
    }
    catch(char const * text)
    {
      printf("Error while spinning: %s\n", text);
    }
    catch(...)
    {
      printf("Unknown Error while spinning\n");
    }
    r.sleep();
  }
  printf("Returning from ros spinning\n");
  return;
}

bool ROSTopicManager::OpenCommOnROSTopic(std::string st)
{
  return true;
}



#endif /*USE_YARP_COMM*/
