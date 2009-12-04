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


#include <vision_srvs/srvjlo.h>
#include "ROSjloComm.h"

using namespace vision_srvs;
using namespace vision_msgs;

#define JLO_IDQUERY "idquery"
#define JLO_FRAMEQUERY "framequery"
#define JLO_DELETE "del"
#define JLO_UPDATE "update"


using namespace cop;

void PutPoseIntoAMessage (partial_lo& plo, RelPose* pose)
{
  Matrix m = pose->GetMatrix();
  int width = 4;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
        plo.pose[r * width + c] = m.element(r,c);
    }
  }
  m = pose->GetCovarianceMatrix();
  width = 6;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
        plo.cov[r * width + c] = m.element(r,c);
    }
  }
  plo.id = pose->m_uniqueID;
  plo.parent_id = pose->m_parentID;
  plo.type = 0;
}

RelPose* GetPoseFromMessage(partial_lo& msg, jlo::LazyLocatedObjectLoader* loader)
{
  Matrix m(4,4);
  m << msg.pose[ 0] << msg.pose[ 1] << msg.pose[ 2] << msg.pose[ 3]
    << msg.pose[ 4] << msg.pose[ 5] << msg.pose[ 6] << msg.pose[ 7]
    << msg.pose[ 8] << msg.pose[ 9] << msg.pose[10] << msg.pose[11]
    << msg.pose[12] << msg.pose[13] << msg.pose[14] << msg.pose[15];
  Matrix cov(6,6);
  cov << msg.cov[ 0] << msg.cov[ 1] << msg.cov[ 2] << msg.cov[ 3] << msg.cov[ 4] << msg.cov[ 5]
      << msg.cov[ 6] << msg.cov[ 7] << msg.cov[ 8] << msg.cov[ 9] << msg.cov[10] << msg.cov[11]
      << msg.cov[12] << msg.cov[13] << msg.cov[14] << msg.cov[15] << msg.cov[16] << msg.cov[17]
      << msg.cov[18] << msg.cov[19] << msg.cov[20] << msg.cov[21] << msg.cov[22] << msg.cov[23]
      << msg.cov[24] << msg.cov[25] << msg.cov[25] << msg.cov[26] << msg.cov[27] << msg.cov[29]
      << msg.cov[30] << msg.cov[31] << msg.cov[32] << msg.cov[33] << msg.cov[34] << msg.cov[35];
  return new RelPose(loader,msg.id, msg.parent_id, m, cov);
}

extern volatile bool g_stopall;

ROSjloComm::ROSjloComm(std::string nodeName) :
  m_service(nodeName)
{
  ros::NodeHandle node;
  while(!g_stopall)
  {
    if(ros::service::waitForService(m_service, 1000))
      break;
    else 
    {
      printf("Waiting for Jlo to startup at %s\n", m_service.c_str());
      g_stopall = g_stopall || !node.ok();
    }
  }
  m_client = node.serviceClient<srvjlo>(this->m_service, true);
  printf("Jlo at %s\n", m_service.c_str());

}

ROSjloComm::~ROSjloComm()
{
}

ros::ServiceClient ROSjloComm::GetJloServiceClient()
{
  ros::NodeHandle node;
  while(!m_client.isValid())
  {
    if(g_stopall)
      throw "Interupted while waiting for jlo";
    ros::service::waitForService(m_service, 0);
    m_client = node.serviceClient<srvjlo>(this->m_service, true);
  }
  return m_client;
}

void ROSjloComm::NotifyPoseUpdate(RelPose* pose, bool sendObjectRelation)
{
  srvjlo msg;
  msg.request.command = JLO_UPDATE;
  PutPoseIntoAMessage(msg.request.query, pose);
  if (!GetJloServiceClient().call(msg))
  {
    printf("Notify Pose Update: Error in ROSjloComm: Update of pose information not psossible !\n");
  }
  else if (msg.response.error.length() > 0)
  {
    printf("Error from jlo: %s!\n", msg.response.error.c_str());
  }
  return;
}

RelPose* ROSjloComm::CreateNewPose(RelPose* pose, Matrix* mat, Matrix* cov)
{
  srvjlo msg;
  msg.request.command = JLO_UPDATE;
  msg.request.query.id = 0;
  if(pose != NULL)
    msg.request.query.parent_id = pose->m_uniqueID;
  else
    msg.request.query.parent_id = ID_WORLD;
#ifdef _DEBUG
/*   printf("Parent ID before query: %d\n", (int )  msg.request.query.parent_id);*/
#endif
  int width = 4;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
      msg.request.query.pose[r * width + c] = mat->element(r,c);
    }
  }
  width = 6;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
        msg.request.query.cov[r * width + c] = cov->element(r,c);
    }
  }
  if (!GetJloServiceClient().call(msg))
  {
    printf("Create New Pose: Error in ROSjloComm: Update of pose information not psossible!\n");
    printf("Debug out of srvjl::request :\n");
    printf("command:  %s\n", msg.request.command.c_str());
    printf("query.id %d query.parent_id %d type %d\n", (int)msg.request.query.id, (int)msg.request.query.parent_id, msg.request.query.type);
    printf("query.pose: \n%f %f %f %f \n %f %f %f %f \n%f %f %f %f\n%f %f %f %f\n",  msg.request.query.pose[0] ,
    msg.request.query.pose[1] ,
    msg.request.query.pose[2] ,
    msg.request.query.pose[3] ,
    msg.request.query.pose[4] ,
    msg.request.query.pose[5] ,
    msg.request.query.pose[6],
    msg.request.query.pose[7] ,
    msg.request.query.pose[8] ,
    msg.request.query.pose[9] ,
    msg.request.query.pose[10] ,
    msg.request.query.pose[11] ,
    msg.request.query.pose[12] ,
    msg.request.query.pose[13] ,
    msg.request.query.pose[14] ,
    msg.request.query.pose[15] );
    return NULL;
  }
  else if (msg.response.error.length() > 0)
  {
    printf("Message from jlo: %s!\n", msg.response.error.c_str());
  }
  return GetPoseFromMessage(msg.response.answer, this);
}

RelPose* ROSjloComm::GetPose(int poseId)
{
   srvjlo msg;
   msg.request.command = JLO_IDQUERY;
   msg.request.query.id = poseId;
  if (!GetJloServiceClient().call(msg))
  {
    printf("Error in ROSjloComm: Reading of pose information not possible (task: %s with %d)!\n", JLO_IDQUERY, poseId);
    return NULL;
  }
  else if (msg.response.error.length() > 0)
  {
    printf("Error from jlo: %s!\n", msg.response.error.c_str());
    return NULL;
  }
  else
    return GetPoseFromMessage(msg.response.answer, this);
}

jlo::LocatedObject* ROSjloComm::GetParent(const jlo::LocatedObject& child)
{
  if(child.m_parentID == 0)
    return NULL;
  return GetPose(child.m_parentID);
}

RelPose* ROSjloComm::GetPoseRelative(int poseId, int parentPoseId)
{
  srvjlo msg;
  msg.request.command  = JLO_FRAMEQUERY;
  msg.request.query.id = poseId;
  msg.request.query.parent_id = parentPoseId;
  if (!GetJloServiceClient().call(msg))
  {
    printf("Error in ROSjloComm: Reading of pose information not possible!\n");
    return NULL;
  }
  else if (msg.response.error.length() > 0)
  {
    printf("Error from jlo: %s!\n", msg.response.error.c_str());
    return NULL;
  }
  return GetPoseFromMessage(msg.response.answer, this);
}


void ROSjloComm::FreePose(int poseId)
{
  srvjlo msg;
  msg.request.command = JLO_DELETE;
  msg.request.query.type = 0;
  msg.request.query.id = poseId;
  if(poseId == ID_WORLD)
    return;
  if (!GetJloServiceClient().call(msg))
  {
    printf("Error in ROSjloComm: Deleting of pose information not psossible!\n");
  }
  else if (msg.response.error.length() > 0)
  {
    if(msg.response.error.compare("Object deleted.") != 0)
      printf("Message from jlo: %s!\n", msg.response.error.c_str());
  }
}
#endif /*USE_YARP_COMM*/
