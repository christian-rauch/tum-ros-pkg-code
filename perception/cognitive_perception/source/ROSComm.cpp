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




#include "ROSComm.h"
using namespace vision_msgs;
using namespace vision_srvs;

#include "BoostUtils.h"

#ifdef BOOST_THREAD
#include <boost/thread/mutex.hpp>
boost::mutex s_mutexAnswer;
#endif

using namespace cop;

unsigned long Comm::s_lastCommID = 0;

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
  for(unsigned int i = 0; i < new_location.size(); i++)
  {
    aposteriori_position apo_pose;
    apo_pose.objectId = new_location[i].second->m_ID;
    apo_pose.probability = new_location[i].first->m_qualityMeasure;
    apo_pose.position = new_location[i].first->m_uniqueID;
    int elems_c = new_location[i].second->CountElems();
    for(int j = 0; j < elems_c; j++)
    {
      Descriptor* descriptor = (Descriptor*)new_location[i].second->GetElement(j, ELEM);
      if(descriptor != NULL)
      {
        vision_msgs::cop_descriptor descr;
        descr.sem_class = descriptor->GetClass()->GetName();
        descr.object_id = descriptor->m_ID;
        descr.type      = descriptor->GetNodeName();
        descr.quality   = descriptor->GetQuality();
        apo_pose.models.push_back(descr);
      }
    }
    answer.found_poses.push_back(apo_pose);
  }
}


void PutPoseIntoAMessage(cop_answer &answer, Signature* sig)
{
  aposteriori_position apo_pose;
  apo_pose.objectId = sig->m_ID;
  int elems_c = sig->CountElems();
  RelPose* pose = NULL;
  unsigned long max_timestamp = 0;
  double pose_qual = 0.0;
  for(int j = 0; j < elems_c; j++)
  {
    Descriptor* descriptor = (Descriptor*)sig->GetElement(j, ELEM);
    if(descriptor != NULL)
    {
      vision_msgs::cop_descriptor descr;
      descr.sem_class = descriptor->GetClass()->GetName();
      descr.object_id = descriptor->m_ID;
      descr.type      = descriptor->GetNodeName();
      descr.quality   = descriptor->GetQuality();
      if(descriptor->date() > max_timestamp)
      {
        pose = descriptor->GetLastMatchedPose();
        if(pose != NULL)
        {
          max_timestamp = descriptor->date();
          pose_qual = pose->m_qualityMeasure;
        }
      }
      apo_pose.models.push_back(descr);
    }
  }
  apo_pose.probability = pose_qual;
  if(pose != NULL)
    apo_pose.position = pose->m_uniqueID;
  answer.found_poses.push_back(apo_pose);
}

void ROSComm::NotifyPoseUpdate(RelPose* pose, bool sendObjectRelation)
{
  BOOST(s_mutexAnswer.lock());
  try
  {
    cop_answer answer;
    if(pose != NULL)
    {
      aposteriori_position ap_pose;
      ap_pose.objectId = m_visPrim.GetSignature()->m_ID;
      ap_pose.probability = pose->m_qualityMeasure;
      ap_pose.position = pose->m_uniqueID;
      answer.found_poses.push_back(ap_pose);
      answer.perception_primitive = m_visPrim.GetID();
    }
    else
    {
        answer.error = "Object lost";
    }
    m_publisher->publish(answer);
  }
  catch(...)
  {
    printf("Problems publishing answer in ROSComm::NotifyPoseUpdate\n");
  }
#ifdef _DEBUG
  /*printf("Writing a bottle in Pose Update Notification:  %s\n", new_loc_bottle.toString().c_str());*/
#endif /*_DEBUG*/
  BOOST(s_mutexAnswer.unlock());
}

void ROSComm::NotifyNewObject(Signature* sig, RelPose* pose)
{
  BOOST(s_mutexAnswer.lock());
  try
  {
    cop_answer answer;
    if(pose != NULL)
    {
      aposteriori_position ap_pose;
      ap_pose.objectId = sig->m_ID;
      ap_pose.probability = pose->m_qualityMeasure;
      ap_pose.position = pose->m_uniqueID;
      answer.found_poses.push_back(ap_pose);
      answer.perception_primitive = m_visPrim.GetID();
    }
    else
    {
        answer.error = "Object lost";
    }
    m_publisher->publish(answer);
  }
  catch(...)
  {
    printf("Problems publishing answer in ROSComm::NotifyNewObject\n");
  }
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
  boost::thread( boost::bind(&ROSComm::ProcessCall, this));
#else
  ROSComm::ProcessCall(this);
#endif /*BOOST_THREAD*/
}

void ROSComm::PublishAnswer(cop_answer &answer)
{
  int timeout = 0;
  while(m_publisher->getNumSubscribers() < 1 && timeout < 100)
  {
     printf("No subscribers, waiting\n");
     Sleeping(10);
     timeout++;
  }
  BOOST(s_mutexAnswer.lock());
  try
  {
    answer.perception_primitive = m_visPrim.GetID();
    m_publisher->publish(answer);
  }
  catch(...)
  {
     printf("Problems publishing answer in ROSComm::PublishAnswer\n");
  }
  BOOST(s_mutexAnswer.unlock());
}


void ROSComm::ProcessCall()
{
#ifdef _DEBUG
    printf("Answer Thread started for object %ld command %s\n", m_visPrim.GetSignature()->m_ID, ((m_actionType == ALGORITHMTYPE_LOCATE) ? "Locate" : ((m_actionType == ALGORITHMTYPE_TRACK) ? "Track" : "StopTrack or unknown action")));
#endif
    bool bFinished = false;
    cop_answer answer;

    switch(m_actionType)
    {
    case ALGORITHMTYPE_LOOKUP:
        PutPoseIntoAMessage(answer, m_visPrim.GetSignature());
        PublishAnswer(answer);
        break;
    case ALGORITHMTYPE_REFINE:
        try
        {
          const SignatureLocations_t &new_location = m_visFinder.m_visLearner.RefineObject(m_pose, m_visPrim, m_numOfObjects);
          if(m_numOfObjects > 0 && new_location.size() > 0)
          {
                  PutPoseIntoAMessage(answer, new_location);
          }
          else
          {
            answer.error = "No Object Found!";
          }
          PublishAnswer(answer);
        }
        catch(...)
        {
          answer.error = "Locate failed";
          PublishAnswer(answer);
        }
        break;
    case ALGORITHMTYPE_TRACK:
       m_visPrim.GetSignature()->SetCommCallBack(this);
        if(m_pose->size() > 0)
        {
          PossibleLocations_t::const_iterator it = m_pose->begin();
          m_visFinder.StartTrack(m_visPrim, (*it).first);
        }
        else
          m_visFinder.StartTrack(m_visPrim, NULL);
       break;
    case ALGORITHMTYPE_STOPTRACK:
        m_visPrim.GetSignature()->SetCommCallBack(NULL);
        m_visFinder.StopTrack(*m_visPrim.GetSignature());
        bFinished = true;
        break;
    case ALGORITHMTYPE_STARTATTEND:
    {
        m_visFinder.m_attentionMan.SetObjectToAttend(&m_visPrim,m_pose, this);
        bFinished = false;
        break;
    }

    case ALGORITHMTYPE_STOPATTEND:
        m_visFinder.m_attentionMan.StopAttend(this);
        bFinished = true;
        break;

    case ALGORITHMTYPE_LOCATE:
      try
      {
        const SignatureLocations_t &new_location = m_visFinder.Locate(m_pose, m_visPrim, m_numOfObjects);
        if(m_numOfObjects > 0 && new_location.size() > 0)
        {
           PutPoseIntoAMessage(answer, new_location);
        }
        else
        {
          answer.error = "No Object Found!";
        }
        PublishAnswer(answer);
      }
      catch(char const* text)
      {
        printf("Locate called by ros failed: %s\n", text);
        answer.error = "Locate failed";
        PublishAnswer(answer);
      }
      catch(...)
      {
        answer.error = "Locate failed";
        PublishAnswer(answer);
      }
      bFinished = true;
      break;
    default:
       answer.error = "Locate failed";
       PublishAnswer(answer);
       bFinished = true;
       break;
    }
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
  std::vector<ObjectID_t> class_id;
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
        sig = NULL;
    }
  }
  for(unsigned int j = 0; j < msg.object_ids.size(); j++)
  {
    try
    {
        int index;
        ObjectID_t id = msg.object_ids[j];
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
          printf("Received unknown Element id: %ld\n", id);
        }
    }
    catch(...)
    {
        printf("ROSComm: Problems reading classes\n");
        sig = NULL;
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
       sig = NULL;
     }
     catch(...)
     {
       printf("Error Creating signature\n");
       sig = NULL;
     }
  }
  else
  {
    m_sig.CompleteSignature(sig, class_id);
  }
#ifdef _DEBUG
  if(sig != NULL)
  {
    printf("Created Signature with all classes (%ld)\n", sig->m_ID);
  }
#endif
  if(sig == NULL)
    return false;
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
    printf("Publisher pub = n.advertise<cop_answer>(%s, 1000)\n", topicname.c_str());
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
  PerceptionPrimitive& vis = m_sig.CreateNewPerceptionPrimitive(sig);
  ROSComm* comm = new ROSComm(m_visFinder, poses, vis, m_openTopics[topicname], (int)msg.number_of_objects, (int)msg.action_type);
  comm->Start();
  answer.perception_primitive = vis.GetID();
  return true;
}

void ROSTopicManager::NewSignatureCallBack(std_msgs::String::ConstPtr xmlFilename)
{
  try
  {
    XMLTag* signature = XMLTag::ReadFromFile(xmlFilename->data);
    Signature* sig = (Signature*)Elem::ElemFactory(signature);
    m_sig.AddSignature(sig);

    printf("Signature created remotely. num descriptors: %ld\n", sig->CountElems());

  }
  catch(const char* text)
  {
    printf("Error loading incoming Signature file: %s\n", text);
  }
}

void ROSTopicManager::FeedbackCallBack(vision_msgs::cop_feedback::ConstPtr feedback)
{
  m_sig.EvaluatePerceptionPrimitive(feedback->perception_primitive, feedback->evaluation);
}

void ROSTopicManager::Listen(std::string name, volatile bool &g_stopall, ros::NodeHandle* node)
{
  /**
   * Now we subscribe using the normal method, to demonstrate the difference.
   */
  /*ros::Publisher pub = n.advertise<cop_call>(name, 1000);*/

  printf("advertiseService<cop_call> (%s, ...)\n", name.c_str());
  ros::ServiceServer srv = node->advertiseService(node->resolveName(name), &ROSTopicManager::ListenCallBack, this);


  ros::Subscriber sub_add_sig =
        node->subscribe<std_msgs::String>(
                 node->resolveName("new_signatures"), 1000,
                 boost::bind(&ROSTopicManager::NewSignatureCallBack, this, _1));

  ros::Subscriber sub_feedback =
        node->subscribe<vision_msgs::cop_feedback>(
                 node->resolveName("feedback"), 1000,
                 boost::bind(&ROSTopicManager::FeedbackCallBack, this, _1));

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
