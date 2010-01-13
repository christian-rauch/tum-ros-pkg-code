/*
 * Copyright (c) 2008, U. Klank
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
 *
 *
 */

#include "CopAnswerDisplay.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/common.h"

#include <ogre_tools/axes.h>
#include <ogre_tools/movable_text.h>

#include <tf/transform_listener.h>
#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <ogre_tools/point_cloud.h>

#include <algorithm>

using namespace vision_srvs;
namespace rviz_shows_cop
{

CopAnswerDisplay::CopAnswerDisplay(const std::string & name,
                                         rviz::VisualizationManager * manager)
: Display(name, manager)
, color_(0.1f, 1.0f, 0.0f)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  inited_jlo = false;
  static int count = 0;
  std::stringstream ss;
  ss << "Cop Answer" << count++;
  manual_object_ = scene_manager_->createManualObject(ss.str());
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
  setAlpha (1.0f);
}

CopAnswerDisplay::~CopAnswerDisplay()
{
  unsubscribe();
  clear();
  scene_manager_->destroyManualObject(manual_object_);

}

void CopAnswerDisplay::clear()
{
  /*manual_object_->clear ();*/

  scene_node_->removeAndDestroyAllChildren();

  /*manual_object_ = scene_manager_->createManualObject("Cop Answer");
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);*/

}

void CopAnswerDisplay::setTopic(const std::string & topic)
{
  unsubscribe();
  topic_ = topic;
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}


void CopAnswerDisplay::setAlpha(float alpha)
{
  alpha_ = alpha;

  propertyChanged(alpha_property_);
  processMessage(current_message_);
  causeRender();
}

void CopAnswerDisplay::subscribe()
{

  if (!isEnabled() || topic_.length() < 2)
    return;

  cop_subscriber = update_nh_.subscribe<vision_msgs::cop_answer>(topic_,1, boost::bind(&CopAnswerDisplay::incomingMessage, this, _1));
}

void CopAnswerDisplay::unsubscribe()
{
  cop_subscriber.shutdown();
}

void CopAnswerDisplay::onEnable()
{
  scene_node_->setVisible(true);
  subscribe();
}

void CopAnswerDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible(false);
}

void CopAnswerDisplay::fixedFrameChanged()
{
  clear();
}

void CopAnswerDisplay::update(float wall_dt, float ros_dt)
{
}



bool CopAnswerDisplay::GetJlo(unsigned long id, unsigned long parent_id,
    vision_msgs::partial_lo::_pose_type &mat,  vision_msgs::partial_lo::_cov_type &cov)
{
  srvjlo msg;
  msg.request.command = "framequery";
  msg.request.query.parent_id = parent_id;
  msg.request.query.id = id;
  if(!inited_jlo || !jlo_client.isValid())
  {
    inited_jlo = true;
    jlo_client = update_nh_.serviceClient<srvjlo>("/located_object", true);
  }
  if (!jlo_client.call(msg))
  {
    printf("Error in ROSjloComm: Update of pose information not psossible!\n");
    return false;
  }
  else if (msg.response.error.length() > 0)
  {
    printf("Error from jlo: %s!\n", msg.response.error.c_str());
    return false;
  }
  mat = msg.response.answer.pose;
  cov = msg.response.answer.cov;
  return true;
}

template<typename T>
void CopAnswerDisplay::setSceneNodePose(Ogre::SceneNode* scene_node, T mat, Ogre::Quaternion &orientation)
{
   tf::Stamped<tf::Pose> pose_w(btTransform(btMatrix3x3(mat[0],mat[1],mat[2],
                                                       mat[4],mat[5],mat[6],
                                                       mat[8],mat[9],mat[10]),
  btVector3(mat[3], mat[7], mat[11])), ros::Time(),
      "/map");

  Ogre::Vector3 position;
  try
  {
    if(!vis_manager_->getFrameManager()->getTransform("/map", ros::Time(), position, orientation, false))
        ROS_ERROR("Error transforming from frame 'map' to frame '%s'",
        fixed_frame_.c_str());
    /*vis_manager_->getTFClient()->transformPose(fixed_frame_, pose_w, pose_w);*/
  }
  catch (tf::TransformException & e)
  {
    ROS_ERROR("Error transforming from frame 'map' to frame '%s'",
        fixed_frame_.c_str());
  }

  Ogre::Vector3 position_w = Ogre::Vector3(pose_w.getOrigin().x(),
    pose_w.getOrigin().y(), pose_w.getOrigin().z());
  rviz::robotToOgre(position_w);
  position_w.x += position.x;
  position_w.y += position.y;
  position_w.z += position.z;
  Ogre::Matrix3 rot, rot2;
  orientation.ToRotationMatrix(rot);

  rot2 = rot * Ogre::Matrix3(mat[0],mat[1],mat[2],mat[4],mat[5],mat[6],mat[8],mat[9],mat[10]);
  orientation.FromRotationMatrix(rot2);
  btScalar yaw_w, pitch_w, roll_w;
  pose_w.getBasis().getEulerYPR(yaw_w, pitch_w, roll_w);

  Ogre::Matrix3 orientation_w(1,0,0,0,1,0,0,0,1);
  scene_node->setPosition(position_w);

}

void CopAnswerDisplay::processMessage (const vision_msgs::cop_answerConstPtr& msg)
{
  clear();
  if (!msg || msg->error.length() > 0)
  {
    return;
  }
  clear();
  Ogre::Quaternion orientation;
  std::vector<double> matid;
  matid.push_back(1.0f); matid.push_back(0.0f); matid.push_back(0.0f);  matid.push_back(0.0f);
  matid.push_back(0.0f); matid.push_back(1.0f); matid.push_back(0.0f); matid.push_back(0.0f);
  matid.push_back(0.0f);matid.push_back( 0.0f);matid.push_back( 1.0f);matid.push_back( 0.0f);
  setSceneNodePose(scene_node_, matid, orientation);
  printf("Number of attached objects: %d\n",scene_node_->numChildren());
  if(scene_node_->numChildren() > 1)
  {
    clear();
  }

  for(unsigned int i = 0; i < msg->found_poses.size(); i++)
  {

    Ogre::SceneNode* node = scene_node_->createChildSceneNode();
    vision_msgs::partial_lo::_pose_type mat;
    vision_msgs::partial_lo::_cov_type cov;

    if(msg->found_poses[i].position == 1)
    {
      std::copy(matid.begin(), matid.end(), mat.begin());
    }
    else
    {
      if(!GetJlo(msg->found_poses[i].position, 1, mat, cov))
        continue;
    }
    setSceneNodePose(node , mat, orientation);

    ogre_tools::Axes* axes_ = new ogre_tools::Axes( scene_manager_, node, 0.2, 0.02);
    axes_->getSceneNode()->setVisible( true);
    axes_->setOrientation(orientation);
    Ogre::Matrix3 rotMat;
    orientation.ToRotationMatrix(rotMat);
    for(int j = 0; j< 12; j++)
    {
        ogre_tools::Axes* axes_cov = new ogre_tools::Axes( scene_manager_, node, 0.05, 0.005 );
        Ogre::SceneNode* axis_tmp = axes_cov->getSceneNode();
        axis_tmp->setPosition(Ogre::Vector3(cov[0] * (j%6 == 0 ? 1 : 0) * (j>5?-1:1),
                                            cov[7] * (j%6 == 1 ? 1 : 0) * (j>5?-1:1),
                                            cov[14] * (j%6 == 2 ? 1 : 0) * (j>5?-1:1)));
        Ogre::Matrix3  temp;
        temp.FromEulerAnglesZYX(Ogre::Radian(cov[21] * (j%6 == 3 ? 1 : 0) * (j>5?-1:1)),
                                Ogre::Radian(cov[28] * (j%6 == 4 ? 1 : 0) * (j>5?-1:1)),
                                Ogre::Radian(cov[35] * (j%6 == 5 ? 1 : 0) * (j>5?-1:1)));
        Ogre::Quaternion temp_quat;
        temp = rotMat * temp;
        temp_quat.FromRotationMatrix(temp);
        axes_cov->setOrientation(temp_quat);
        axis_tmp->setVisible( true);
    }

    char textbuf[512];
    if(msg->found_poses[i].classes.size() > 0)
    {
      sprintf(textbuf, "%s (%ld at %ld)", msg->found_poses[i].classes[0].c_str(), msg->found_poses[i].objectId,msg->found_poses[i].position);
    }
    else
      sprintf(textbuf, "(%ld at %ld)", msg->found_poses[i].objectId,msg->found_poses[i].position);
    ogre_tools::MovableText* text = new ogre_tools::MovableText(textbuf , "Arial",0.1, Ogre::ColourValue::White);
    Ogre::SceneNode *text_node= node->createChildSceneNode();
    text_node->attachObject(text);
    text_node->setVisible( true);
    node->setVisible(true);
  }
}

void CopAnswerDisplay::incomingMessage(const vision_msgs::cop_answer::ConstPtr& msg)
{
  processMessage(msg);
}

void CopAnswerDisplay::reset()
{
  clear();
}

void CopAnswerDisplay::targetFrameChanged()
{
}

void CopAnswerDisplay::createProperties()
{
  /*lor_property_ = property_manager_->createProperty<rviz::ColorProperty> ("Color", property_prefix_, boost::bind(&CopAnswerDisplay::getColor, this),
                                                                      boost::bind(&CopAnswerDisplay::setColor, this, _1), parent_category_, this);*/

  alpha_property_ = property_manager_->createProperty<rviz::FloatProperty> ("Alpha", property_prefix_, boost::bind(&CopAnswerDisplay::getAlpha, this),
                                                                      boost::bind(&CopAnswerDisplay::setAlpha, this, _1), parent_category_,this);
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> ("Topic", property_prefix_, boost::bind(&CopAnswerDisplay::getTopic, this),
                                                                               boost::bind(&CopAnswerDisplay::setTopic, this, _1), parent_category_, this);
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(vision_msgs::cop_answer::__s_getDataType());


}

} // namespace mapping_rviz_plugin
