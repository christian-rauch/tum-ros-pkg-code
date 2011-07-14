/*
 * Copyright (c) 2009, U. Klank   klank@in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cop_client_cpp/cop_client.h>
#include <vision_srvs/cop_call.h>
#include <vision_msgs/cop_feedback.h>
#include <vision_srvs/srvjlo.h>
using namespace vision_srvs;
using namespace vision_msgs;

bool breaker = false;
unsigned long vision_primitive = 0;
bool found_trans_obj = false;
unsigned long tobj_pos = 0;

CopClient::CopClient(ros::NodeHandle &nh, std::string stTopicName, std::string stCopName)
{
 /** subscribe to the topic cop should publish the results*/
 m_answerTopic = stTopicName;
 m_readAnswer = nh.subscribe<cop_answer>(stTopicName, 1000, &CopClient::callback, this);
 /** Publish */
 char srvname[512];
 sprintf(srvname, "/%s/in", stCopName.c_str());
 ros::service::waitForService(srvname);
 m_client = nh.serviceClient<cop_call>(srvname, true);

 ros::service::waitForService("/located_object");
 m_jlo_client = nh.serviceClient<srvjlo>("/located_object", true);

 m_pubFeedBack = nh.advertise<vision_msgs::cop_feedback>("/cop/feedback", 1000);


 ros::spinOnce();

}

unsigned long CopClient::LOFrameQuery(unsigned long child, unsigned long parent)
{
  srvjlo msg;
  msg.request.command = "framequery";
  msg.request.query.id = child;
  msg.request.query.parent_id = parent;
  m_jlo_client.call(msg);

  return msg.response.answer.id;
}

void CopClient::LOFreeID(unsigned long id)
{
  srvjlo msg;
  msg.request.command = "del";
  msg.request.query.id = id;
  m_jlo_client.call(msg);
}

double CopClient::LODistanceQuery(unsigned long child, unsigned long parent)
{
  srvjlo msg;
  if(child == parent)
    return 0.0;
  msg.request.command = "framequery";
  msg.request.query.id = child;
  msg.request.query.parent_id = parent;
  m_jlo_client.call(msg);
  LOFreeID(msg.response.answer.id);
  return sqrt(msg.response.answer.pose[3]*msg.response.answer.pose[3] + msg.response.answer.pose[7]*msg.response.answer.pose[7]+msg.response.answer.pose[11]*msg.response.answer.pose[11]);
}


void CopClient::LOPointQuery(unsigned long child, float *vector)
{
  srvjlo msg;
  msg.request.command = "framequery";
  msg.request.query.id = child;
  msg.request.query.parent_id = 1;
  m_jlo_client.call(msg);

  vector[0] = msg.response.answer.pose[3];
  vector[1] = msg.response.answer.pose[7];
  vector[2] = msg.response.answer.pose[11];
  LOFreeID(msg.response.answer.id);

  //return msg.response.answer.pose[11];
}


tf::Stamped<tf::Pose> CopClient::LOPoseQuery(unsigned long child)
{
  srvjlo msg;
  msg.request.command = "framequery";
  msg.request.query.id = child;
  msg.request.query.parent_id = 1; //map
  m_jlo_client.call(msg);

  tf::Stamped<tf::Pose> ret;
  ret.frame_id_ = "/map";
  ret.setOrigin(btVector3(msg.response.answer.pose[3],msg.response.answer.pose[7],msg.response.answer.pose[11]));
  btMatrix3x3 mat(msg.response.answer.pose[0],msg.response.answer.pose[1],msg.response.answer.pose[2],
                  msg.response.answer.pose[4],msg.response.answer.pose[5],msg.response.answer.pose[6],
                  msg.response.answer.pose[8],msg.response.answer.pose[9],msg.response.answer.pose[10]);
  btQuaternion qat;
  mat.getRotation(qat);
  ret.setRotation(qat);

  LOFreeID(msg.response.answer.id);

  //return msg.response.answer.pose[11];
  return ret;
}


unsigned long CopClient::LOOffset(unsigned long id, unsigned long parent, float x, float y, float z)
{
  srvjlo msg;
  msg.request.command = "update";
  msg.request.query.id = id;
  msg.request.query.parent_id = parent;

  msg.request.query.pose[0] = 1.0;
  msg.request.query.pose[5] = 1.0;
  msg.request.query.pose[10] = 1.0;
  msg.request.query.pose[15] = 1.0;
  msg.request.query.pose[3] = x;
  msg.request.query.pose[7] = y;
  msg.request.query.pose[11] = z;

  m_jlo_client.call(msg);

  return msg.response.answer.id;
}



unsigned long CopClient::LONameQuery(std::string name)
{
  vision_srvs::srvjlo msg;
  msg.request.command = "namequery";
  msg.request.query.name = name;
  if (!m_jlo_client.call(msg))
  {
     printf("Error calling jlo!\n");
  }
  if (msg.response.error.length() > 0)
  {
     printf("Error from jlo: %s!\n", msg.response.error.c_str());
     return 0;
  }
   return msg.response.answer.id;
}


void CopClient::callback(const boost::shared_ptr<const cop_answer> &msg)
{
 found_trans_obj = false;
 printf("got answer from cop! (Errors: %s)\n", msg->error.c_str());
 m_msg_cluster[msg->perception_primitive].push_back(*msg);
 for(size_t i = 0; i < msg->found_poses.size(); i++)
 {
   const aposteriori_position &pos =  msg->found_poses[i];
   printf("Found Obj nr %d with prob %f at pos %d\n", (int)pos.objectId, pos.probability, (int)pos.position);
   for(size_t j = 0; j <  pos.models.size(); j++)
   {
     printf("  %s", pos.models[j].sem_class.c_str());
   }

   if(pos.models.size() > 0)
     printf("\n");
 }
 printf("End!\n");
}


long CopClient::CallCop(std::string object_class, unsigned long position_id, int num_of_objects, unsigned long alterantive_id, unsigned long action_type)
{
  cop_call call;
  call.request.outputtopic = m_answerTopic;

  if(alterantive_id == 0 && object_class.length() > 0)
    call.request.object_classes.push_back(object_class);
  else
     call.request.object_ids.push_back(alterantive_id);

  call.request.action_type = action_type;

  call.request.number_of_objects =  num_of_objects;

  apriori_position pos;
  pos.probability = 1.0;
  pos.positionId = position_id;
  call.request.list_of_poses.push_back(pos);


  if(!m_client.call(call))
  {
      printf("Error calling cop\n");
      return -1;
  }
  else
      printf("Called cop \n");

  vision_primitive =  call.response.perception_primitive;
  return (signed)vision_primitive;
}


long CopClient::CallCop(std::string object_class, std::vector<unsigned long> position_ids, int num_of_objects, unsigned long alterantive_id, unsigned long action_type)
{
  cop_call call;
  call.request.outputtopic = m_answerTopic;

  if(alterantive_id == 0 && object_class.length() > 0)
    call.request.object_classes.push_back(object_class);
  else
     call.request.object_ids.push_back(alterantive_id);

  call.request.action_type = action_type;

  call.request.number_of_objects =  num_of_objects;
  for(size_t i = 0 ; i < position_ids.size(); i++)
  {
    apriori_position pos;
    pos.probability = 1.0;
    pos.positionId = position_ids[i];
    call.request.list_of_poses.push_back(pos);
  }


  if(!m_client.call(call))
  {
      printf("Error calling cop\n");
      return -1;
  }
  else
      printf("Called cop \n");

  vision_primitive =  call.response.perception_primitive;
  return (signed)vision_primitive;
}



void CopClient::CopFeedBack(long primitive, double  evaluation, unsigned long error_id)
{
  vision_msgs::cop_feedback msg;
  msg.evaluation = evaluation;
  msg.perception_primitive = primitive;
  if(error_id > 0)
  {
    vision_msgs::system_error err;
    err.error_id = error_id;
    msg.error.push_back(err);
  }
  m_pubFeedBack.publish(msg);

}
