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
#include "rviz/common.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include <ogre_tools/shape.h>
#include <ogre_tools/billboard_line.h>

#include "Line3D.h"

#include <algorithm>
#include <vector>
#include <sstream>

using namespace vision_srvs;
namespace rviz_shows_cop
{

CopAnswerDisplay::CopAnswerDisplay(const std::string & name,
                                         rviz::VisualizationManager * manager)
  : JloDisplayBase(name, manager)
{
  m_binited = false;
  setSGP(false);
  setTH(0.72);
  setOffset(0.0);
  /*m_manualObject = scene_manager_->createManualObject( "Fingerlines");
  m_manualObject->setDynamic( true );
  scene_node_->attachObject(m_manualObject);*/
}

CopAnswerDisplay::~CopAnswerDisplay()
{
  unsubscribe();
  /*scene_manager_->destroyManualObject( m_manualObject );*/

}

void CopAnswerDisplay::setTopic(const std::string & topic)
{
  if(m_binited)
    unsubscribe();

  JloDisplayBase::setTopic(topic);
  subscribe();
}

void CopAnswerDisplay::setSGP(bool sgp)
{
  m_sgp = sgp;
  propertyChanged(m_sgp_property);
  processMessage(m_currentMessage);
}

void CopAnswerDisplay::setHand(bool hand)
{
  m_hand = hand;
  propertyChanged(m_hand_property);
  processMessage(m_currentMessage);
}

void CopAnswerDisplay::setAuto(bool hand)
{
  m_auto = hand;
  propertyChanged(m_auto_prop);
  processMessage(m_currentMessage);
}


void CopAnswerDisplay::setSGPColor(rviz::Color col)
{
  m_sgpColor = col;
  propertyChanged(m_sgpColor_prop);
  processMessage(m_currentMessage);
}

void CopAnswerDisplay::setTH(float table_height)
{
  m_th = table_height;
  propertyChanged(m_th_property);
  processMessage(m_currentMessage);
}

void CopAnswerDisplay::setOffset(float offset)
{
  m_offset = offset;
  propertyChanged(m_offset_property);
  processMessage(m_currentMessage);
}
void CopAnswerDisplay::setOfftop(float offset)
{
  m_offtop = offset;
  propertyChanged(m_offset_property);
  processMessage(m_currentMessage);
}

void CopAnswerDisplay::setA(float table_height)
{
  alpha = table_height;
  m_manConf.alpha = alpha;
  propertyChanged(m_alpha_prop);
  processMessage(m_currentMessage);
}
void CopAnswerDisplay::setB(float table_height)
{
  beta = table_height;
  m_manConf.beta = beta;
  propertyChanged(m_beta_prop);
  processMessage(m_currentMessage);
}
void CopAnswerDisplay::setD(float table_height)
{
  delta = table_height;
  m_manConf.delta_max=delta;
  propertyChanged(m_delta_prop);
  processMessage(m_currentMessage);
}


void CopAnswerDisplay::subscribe()
{
  if (getTopic().length() < 2 || !isEnabled())
  {
    m_binited = false;
    return;
  }
  m_binited = true;
  printf("subscribe to %s\n", getTopic().c_str());
  cop_subscriber = update_nh_.subscribe<vision_msgs::cop_answer>(getTopic(),1, boost::bind(&CopAnswerDisplay::incomingMessage, this, _1));
}

void CopAnswerDisplay::unsubscribe()
{
   if(m_binited)
     cop_subscriber.shutdown();
}

double CopAnswerDisplay::GetOffsetBaseLinkRotZ(Ogre::Quaternion quat)
{
  double rot = quat.getRoll().valueRadians();
  printf("Roll: %f Pitch %f  Yaw %f\n", quat.getRoll().valueRadians(), quat.getPitch().valueRadians(),quat.getYaw().valueRadians());
  return rot; /** TODO: get object to baselink rotation*/
}


#define NUM_HAND_POINTS 10
#define MAX_OBSTACLES 100
void CopAnswerDisplay::AttachSGPPoints(Ogre::SceneNode* object, std::vector<vision_msgs::partial_lo::_pose_type > matrices,
                                        std::vector<vision_msgs::partial_lo::_cov_type> covs, size_t index)
{
  Point3D points[NUM_HAND_POINTS];
  Point3D ptemps[NUM_HAND_POINTS];


  Point3D obstacles[MAX_OBSTACLES];
  CovariancePoint obstacle_covs[MAX_OBSTACLES];
  size_t num_obstacles = matrices.size() < MAX_OBSTACLES ? matrices.size() : MAX_OBSTACLES;
  int param_num = num_obstacles;
  /** this does not make sense for an empty list*/
  if(num_obstacles == 0)
    return;
  Ogre::Matrix3 rot (matrices[index][0],matrices[index][1],matrices[index][2],
                      matrices[index][4],matrices[index][5],matrices[index][6],
                      matrices[index][8],matrices[index][9],matrices[index][10]);
  Ogre::Quaternion quat = object->getOrientation();
  Ogre::Vector3 vec = object->getPosition();
  Ogre::Quaternion quat2;
  quat2.FromRotationMatrix(rot);
  double offset_rot_z = m_offset;/*GetOffsetBaseLinkRotZ(quat2);*/
  int handness = m_hand ? -1 : 1;

  /** Init hardcoded hand points */
  points[0].x = 0.00*handness;  points[0].y = 0.000;   points[0].z =-0.03;
  points[1].x = 0.06*handness;  points[1].y = 0.025;   points[1].z = 0.11;
  points[2].x = 0.05*handness;  points[2].y = 0.025;   points[2].z = 0.0;
  points[3].x =-0.06*handness;  points[3].y = 0.010;   points[3].z = 0.11;
  points[4].x =-0.05*handness;  points[4].y = 0.010;   points[4].z = 0.0;
  points[5].x = 0.06*handness;  points[5].y = -0.025;  points[5].z = 0.11;
  points[6].x = 0.05*handness;  points[6].y = -0.025;  points[6].z = 0.0;
  points[7].x = 0.06*handness;  points[7].y = -0.085;  points[7].z = 0.11;
  points[8].x = 0.05*handness;  points[8].y = -0.085;  points[8].z = 0.00;
  points[9].x = 0.00*handness;  points[9].y = -0.010;  points[9].z =-0.03;




  /**  Init target and obstacles (target at index in the incoming list must be a position 0 of the obstacles) */
  size_t done = 0;
  for(size_t i = 0; i < num_obstacles; i++)
  {
    size_t ind_temp = i + 1 - done;
    if(i == index)
    {
      ind_temp = 0;
      done = 1;
    }
    if(matrices[i].size() != 16)
    {
      ROS_INFO(" \nWrong Matrix legnth at mat i=%ld. its not 16 but %ld\n ", i, matrices[i].size());
      throw "Error !!\n";
    }
    if(covs[i].size() != 36)
    {
      ROS_INFO(" \nWrong cov legnth at mat i=%ld. its not 36 but %ld\n ", i, matrices[i].size());
      throw "Error !!\n";
    }

    obstacles[ind_temp].x = matrices[i][3];
    obstacles[ind_temp].y = matrices[i][7];
    obstacles[ind_temp].z = matrices[i][11];
    obstacle_covs[ind_temp].sx  = covs[i][0];
    obstacle_covs[ind_temp].sxy = covs[i][1];
    obstacle_covs[ind_temp].sxz = covs[i][2];
    obstacle_covs[ind_temp].syx = covs[i][6];
    obstacle_covs[ind_temp].sy  = covs[i][7];
    obstacle_covs[ind_temp].syz = covs[i][8];
    obstacle_covs[ind_temp].szx = covs[i][12];
    obstacle_covs[ind_temp].szy = covs[i][13];
    obstacle_covs[ind_temp].sz  = covs[i][14];
  }
  HandConfig conf;
  if(m_auto)
  {
  /** Call  initialization of SGP always for multi-instance support, actually we would need a mutex here, TODO*/
    InitSGP(points, NUM_HAND_POINTS, m_th);
    /*printf("%ld elements with diag of covx = %f, %f, %f\n", num_obstacles, obstacle_covs[0].sx, obstacle_covs[0].sy, obstacle_covs[0].sz);*/
    /** Call SGP */
    conf = GetGraspLM(obstacles, obstacle_covs, param_num, offset_rot_z, m_offtop);
  }
  else
  {
    conf = m_manConf;
  }
  printf("Estimated hand configuration: a: %f b: %f d: %f\n", conf.alpha, conf.beta, conf.delta_max);

  for(size_t j = 0; j< NUM_HAND_POINTS; j++)
  {
    ptemps[j] = TransformPoint(points[j], obstacles[0], conf.alpha, conf.beta, conf.delta_max);
  }
  /*Ogre::SceneNode* handref;*/
  Ogre::SceneNode* tmpnode;


  ogre_tools::BillboardLine  *lines = new ogre_tools::BillboardLine(vis_manager_->getSceneManager(), object);

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;

  /*lines->setPosition(pos);
  lines->setOrientation(orient);*/
  /*lines->setScale(scale);*/
  lines->clear();
  lines->setLineWidth( 0.005 );
  lines->setMaxPointsPerLine(2);
  lines->setNumLines(NUM_HAND_POINTS + 1);
  lines->setColor(m_sgpColor.r_, m_sgpColor.g_, m_sgpColor.b_, 1.0);

  /*m_manualObject->estimateVertexCount(2* NUM_HAND_POINTS );
  m_manualObject->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );*/

  for(size_t j = 0; j< NUM_HAND_POINTS; j++)
  {

    Ogre::Matrix3 rot (matrices[index][0],matrices[index][1],matrices[index][2],
                       matrices[index][4],matrices[index][5],matrices[index][6],
                       matrices[index][8],matrices[index][9],matrices[index][10]);

  /** Visualize results*/
    Point3D ptemp = ptemps[j];
    printf("Point %ld: %f %f %f  (relobj: %f, %f, %f)\n", j, ptemp.x, ptemp.y, ptemp.z, ptemp.x - obstacles[0].x, ptemp.y - obstacles[0].y, ptemp.z - obstacles[0].z);

    /*ogre_tools::Axes* shape= new ogre_tools::Axes( scene_manager_, object, 0.005, 0.005 );*/
    ogre_tools::Shape* shape = new ogre_tools::Shape (ogre_tools::Shape::Sphere, scene_manager_, object);
    shape->setColor(m_sgpColor.r_, m_sgpColor.g_, m_sgpColor.b_, 1.0);
    tmpnode = shape->getRootNode();

    lines->newLine();
    if(j > 0 && j % 2 == 0 && j < 9)
    {
      //m_manualObject->position();
      Ogre::Vector3 vec1 ((ptemps[0].x - obstacles[0].x), (ptemps[0].y - obstacles[0].y), ptemps[0].z - obstacles[0].z);
      Ogre::Matrix3 rotinv =  rot.Inverse();
      Ogre::Vector3 vec_trans1 = rot * vec1;
      lines->addPoint(Ogre::Vector3(-vec_trans1.y,vec_trans1.z,-vec_trans1.x));

      Ogre::Vector3 vec((ptemps[j].x - obstacles[0].x), (ptemps[j].y - obstacles[0].y), ptemps[j].z - obstacles[0].z);
      Ogre::Vector3 vec_trans = rot * vec;
      lines->addPoint(Ogre::Vector3(-vec_trans.y,vec_trans.z,-vec_trans.x));
      //m_manualObject->colour( color );
    }
    else if(j > 0 && j % 2 == 1 && j < 9)
    {
      Ogre::Vector3 vec1 ((ptemps[j].x - obstacles[0].x), (ptemps[j].y - obstacles[0].y), ptemps[j].z - obstacles[0].z);
      Ogre::Matrix3 rotinv =  rot.Inverse();
      Ogre::Vector3 vec_trans1 = rot * vec1;
      lines->addPoint(Ogre::Vector3(-vec_trans1.y,vec_trans1.z,-vec_trans1.x));

      Ogre::Vector3 vec((ptemps[j+1].x - obstacles[0].x), (ptemps[j+1].y - obstacles[0].y), ptemps[j+1].z - obstacles[0].z);
      Ogre::Vector3 vec_trans = rot * vec;
      lines->addPoint(Ogre::Vector3(-vec_trans.y,vec_trans.z,-vec_trans.x));
    }
    else if (j == 0)
    {
      Ogre::Vector3 vec1((ptemps[j].x - obstacles[0].x), (ptemps[j].y - obstacles[0].y), ptemps[j].z - obstacles[0].z);
      Ogre::Matrix3 rotinv =  rot.Inverse();
      Ogre::Vector3 vec_trans1 = rot * vec1;
      lines->addPoint(Ogre::Vector3(-vec_trans1.y,vec_trans1.z,-vec_trans1.x));
      Ogre::Vector3 vec((ptemps[j].x - obstacles[0].x), (ptemps[j].y - obstacles[0].y), ptemps[j].z - obstacles[0].z);
      Ogre::Vector3 vec_trans = rot * vec;
      lines->addPoint(Ogre::Vector3(-vec_trans.y,vec_trans.z,-vec_trans.x));
    }
    else
    {
      Ogre::Vector3 vec1((ptemps[j].x - obstacles[0].x), (ptemps[j].y - obstacles[0].y), ptemps[j].z - obstacles[0].z);
      Ogre::Matrix3 rotinv =  rot.Inverse();
      Ogre::Vector3 vec_trans1 = rot * vec1;
      lines->addPoint(Ogre::Vector3(-vec_trans1.y,vec_trans1.z,-vec_trans1.x));
      Ogre::Vector3 vec((ptemps[j].x - obstacles[0].x), (ptemps[j].y - obstacles[0].y), ptemps[j].z - obstacles[0].z);
      Ogre::Vector3 vec_trans = rot * vec;
      lines->addPoint(Ogre::Vector3(-vec_trans.y,vec_trans.z,-vec_trans.x));
    }
    //m_manualObject->end();
    /*tmpnode  = shape->getSceneNode();*/
    shape->setScale(Ogre::Vector3(0.02,0.02,0.02));

    /*if(j == 0)
       handref = tmpnode = axis->getSceneNode();
    else
    {
       tmpnode = axis->getSceneNode();
       ogre_tools::Arrow* arrow = new ogre_tools::Arrow( scene_manager_, tmpnode, 1.0f, 0.01, 1.0f, 0.08 );
       arrow->setPosition(handref->getPosition());
       arrow->getSceneNode()->setVisible(true);
    }*/
    /*Ogre::Quaternion temp_quat;
    temp_quat.FromRotationMatrix(rotMat);*/

    /*tmpnode->setOrientation(quat);*/
    Ogre::Vector3 vec((ptemp.x - obstacles[0].x), (ptemp.y - obstacles[0].y), ptemp.z - obstacles[0].z);
    Ogre::Matrix3 rotinv =  rot.Inverse();
    Ogre::Vector3 vec_trans =  rotinv * vec;
    /* printf("vec_trans (rotinv): %f %f %f \n", vec_trans.x, vec_trans.y, vec_trans.z);*/

    vec_trans = rot * vec;
    /*printf("vec_trans (rot): %f %f %f \n", vec_trans.x, vec_trans.y, vec_trans.z);*/
    tmpnode->setPosition(Ogre::Vector3(-vec_trans.y, vec_trans.z, -vec_trans.x));
    tmpnode->setVisible(true);
  }


 /* lines->setVisible(true);*/
}

void CopAnswerDisplay::processMessage (vision_msgs::cop_answer& msg)
{
  if(m_binited)
  {
    std::vector<vision_msgs::partial_lo::_pose_type > matrices;
    std::vector<vision_msgs::partial_lo::_cov_type> covs;

    if (msg.error.length() > 0)
    {
      return;
    }

    std::vector<JloDisplayBase::JloDescription> jloSet(msg.found_poses.size());

    for(unsigned int i = 0; i < msg.found_poses.size(); i++)
    {
      std::stringstream strm;

      jloSet[i].pose = msg.found_poses[i].position;

      if(msg.found_poses[i].models.size() > 0)
      {
        strm << msg.found_poses[i].models[0].sem_class << " ";
      }
      strm << "(" << msg.found_poses[i].objectId << " at " << msg.found_poses[i].position << ")";
      jloSet[i].label = strm.str();
    }
    std::list<std::pair<Ogre::SceneNode*, vision_msgs::partial_lo> > &list =  displayJloSet(jloSet.begin(), jloSet.end(), false);
    std::list<std::pair<Ogre::SceneNode*, vision_msgs::partial_lo> >::iterator iter = list.begin();
    for(; iter != list.end(); iter++)
    {
      matrices.push_back((*iter).second.pose);
      covs.push_back((*iter).second.cov);
    }
    iter = list.begin();
    if(m_sgp)
    {
      for(size_t i = 0; i < msg.found_poses.size(); i++)
      {
        AttachSGPPoints((*iter).first, matrices, covs, i);
        iter++;
      }
    }
    causeRender();
  }
}

void CopAnswerDisplay::incomingMessage(const vision_msgs::cop_answer::ConstPtr& msg)
{
  m_currentMessage = *(msg.get());
  printf("Got message: %ld elems\n", m_currentMessage.found_poses.size());
  try
  {
   processMessage(m_currentMessage);
  }
  catch(...)
  {
    ROS_INFO("tf sucks, please start a state publisher or similar stuff\n");
  }

}

void CopAnswerDisplay::createProperties()
{
  /*lor_property_ = property_manager_->createProperty<rviz::ColorProperty> ("Color", property_prefix_, boost::bind(&CopAnswerDisplay::getColor, this),
                                                                      boost::bind(&CopAnswerDisplay::setColor, this, _1), parent_category_, this);*/
  JloDisplayBase::createProperties();

  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> ("Topic", property_prefix_, boost::bind(&JloDisplayBase::getTopic, this),
                                                                               boost::bind(&JloDisplayBase::setTopic, this, _1), parent_category_, this);
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(vision_msgs::cop_answer::__s_getDataType());


  m_sgp_property = property_manager_->createProperty<rviz::BoolProperty> ("GraspPoints", property_prefix_, boost::bind(&CopAnswerDisplay::getSGP, this),
                                                                      boost::bind(&CopAnswerDisplay::setSGP, this, _1), parent_category_,this);
  m_hand_property = property_manager_->createProperty<rviz::BoolProperty> ("Right Hand", property_prefix_, boost::bind(&CopAnswerDisplay::getHand, this),
                                                                      boost::bind(&CopAnswerDisplay::setHand, this, _1), parent_category_,this);
  m_sgpColor_prop = property_manager_->createProperty<rviz::ColorProperty> ("Hand Color", property_prefix_, boost::bind(&CopAnswerDisplay::getSGPColor, this),
                                                                      boost::bind(&CopAnswerDisplay::setSGPColor, this, _1), parent_category_,this);
  m_th_property = property_manager_->createProperty<rviz::FloatProperty> ("Table Height", property_prefix_, boost::bind(&CopAnswerDisplay::getTH, this),
                                                                      boost::bind(&CopAnswerDisplay::setTH, this, _1), parent_category_,this);

  m_offset_property = property_manager_->createProperty<rviz::FloatProperty> ("Offset Z", property_prefix_, boost::bind(&CopAnswerDisplay::getOffset, this),
                                                                      boost::bind(&CopAnswerDisplay::setOffset, this, _1), parent_category_,this);

  m_offset_property = property_manager_->createProperty<rviz::FloatProperty> ("Offset Z top", property_prefix_, boost::bind(&CopAnswerDisplay::getOfftop, this),
                                                                      boost::bind(&CopAnswerDisplay::setOfftop, this, _1), parent_category_,this);

  m_auto_prop = property_manager_->createProperty<rviz::BoolProperty> ("Automatic grasp point", property_prefix_, boost::bind(&CopAnswerDisplay::getAuto, this),
                                                                      boost::bind(&CopAnswerDisplay::setAuto, this, _1), parent_category_,this);
  m_alpha_prop = property_manager_->createProperty<rviz::FloatProperty> ("Alpha", property_prefix_, boost::bind(&CopAnswerDisplay::getA, this),
                                                                      boost::bind(&CopAnswerDisplay::setA, this, _1), parent_category_,this);
  m_beta_prop = property_manager_->createProperty<rviz::FloatProperty> ("Beta", property_prefix_, boost::bind(&CopAnswerDisplay::getB, this),
                                                                      boost::bind(&CopAnswerDisplay::setB, this, _1), parent_category_,this);
  m_delta_prop = property_manager_->createProperty<rviz::FloatProperty> ("Theta", property_prefix_, boost::bind(&CopAnswerDisplay::getD, this),
                                                                      boost::bind(&CopAnswerDisplay::setD, this, _1), parent_category_,this);

}

} // namespace
