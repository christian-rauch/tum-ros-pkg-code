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
 */

#ifndef JLO_DISPLAY_BASE_H
#define JLO_DISPLAY_BASE_H

#include "rviz/display.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"
#include "rviz/visualization_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/common.h"

#include <vision_msgs/cop_answer.h>
#include <vision_srvs/srvjlo.h>

#include <tf/transform_listener.h>

#include <ogre_tools/axes.h>
#include <ogre_tools/movable_text.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <list>

namespace Ogre
{
class SceneNode;
}

namespace rviz_shows_cop
{

/**
 * \class JloDisplayBase
 * \brief Displays a set of jlo objects
 */
class JloDisplayBase : public rviz::Display
{
public:
  struct JloDescription
  {
    unsigned long pose;
    std::string label;
  };
  JloDisplayBase(const std::string& name, rviz::VisualizationManager* manager);

  virtual ~JloDisplayBase();

  virtual void setTopic(const std::string& topic);
  const std::string& getTopic()
  {
    return (topic_);
  }

  void setKeep(uint32_t keep);
  uint32_t getKeep() { return keep_; }

  void setST( bool show_text );
  bool getST() { return show_text_; }
  void setSA( bool show_axis );
  bool getSA() { return show_axis_; }
  void setSC( bool show_cov);
  bool getSC() { return show_cov_; }
  void setAL(double axis_length);
  double getAL() { return axis_length_; }
  void setAT(double axis_thick);
  double getAT() { return axis_thickness_; }
  void setCL(double cov_length);
  double getCL() { return cov_length_; }
  void setCT(double cov_thick);
  double getCT() { return cov_thickness_; }

  template<typename itT> std::list<std::pair<Ogre::SceneNode *, vision_msgs::partial_lo> >& displayJloSet(itT start, itT end, bool render);

  // Overrides from Display
  virtual void targetFrameChanged();
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();
  virtual void processMessage()=0;

protected:
  void clear();

  void popJloSet();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string topic_;
  uint32_t keep_;
  bool show_text_;
  bool show_axis_;
  bool show_cov_;
  double axis_length_;
  double axis_thickness_;
  double cov_length_;
  double cov_thickness_;
  bool m_binited;
  std::list< std::list< std::pair< Ogre::SceneNode *, vision_msgs::partial_lo> > > jlo_nodes_;
  Ogre::SceneNode* scene_node_;

  rviz::ROSTopicStringPropertyWPtr topic_property_;
  rviz::IntPropertyWPtr keep_property_;
  rviz::CategoryPropertyWPtr extended_props_p_;
  rviz::BoolPropertyWPtr show_cov_p_;
  rviz::BoolPropertyWPtr show_axis_p_;
  rviz::BoolPropertyWPtr show_text_p_;
  rviz::FloatPropertyWPtr axis_thickness_p_;
  rviz::FloatPropertyWPtr axis_length_p_;
  rviz::FloatPropertyWPtr cov_length_p_;
  rviz::FloatPropertyWPtr cov_thickness_p_;

  bool inited_jlo;
  ros::ServiceClient jlo_client;
  bool GetJlo(unsigned long id, unsigned long parent_id, vision_msgs::partial_lo &lo);
  unsigned  long NameQueryJlo(std::string name);

  template<typename T> void setSceneNodePose(Ogre::SceneNode* scene_node, T mat, Ogre::Quaternion &orientation);
};


template<typename itT>
std::list<std::pair<Ogre::SceneNode *, vision_msgs::partial_lo> >& JloDisplayBase::displayJloSet(itT start, itT end, bool render=true)
{
  if(m_binited)
  {
    Ogre::Quaternion orientation;
    const double matid[] = {1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1};

    setSceneNodePose(scene_node_, matid, orientation);

    jlo_nodes_.push_back(std::list<std::pair<Ogre::SceneNode *, vision_msgs::partial_lo> >());
    while(jlo_nodes_.size() > keep_)
      popJloSet();

    for(itT it=start; it!=end; it++)
    {
      Ogre::SceneNode* node = scene_node_->createChildSceneNode();

      vision_msgs::partial_lo lo;
      vision_msgs::partial_lo::_pose_type mat;
      vision_msgs::partial_lo::_cov_type cov;

      if(it->pose == 1)
      {
        cov[ 0] = 0.0;
        cov[ 7] = 0.0;
        cov[14] = 0.0;
        cov[21] = 0.0;
        cov[28] = 0.0;
        cov[35] = 0.0;
        std::copy(&matid[0], &matid[16], mat.begin());
        lo.pose = mat;
      }
      else
      {
        unsigned long reference_frame_id = 1; /**1 := /map*/
        reference_frame_id = NameQueryJlo(fixed_frame_);
        printf("NAmeQuery %s resolved to id: %ld\n", fixed_frame_.c_str(), reference_frame_id);
        if(!GetJlo(it->pose, reference_frame_id, lo))
          continue;
        mat = lo.pose;
        cov = lo.cov;
      }
      jlo_nodes_.back().push_back(std::pair<Ogre::SceneNode *, vision_msgs::partial_lo>(node, lo));
      setSceneNodePose(node , mat, orientation);
      if(show_axis_)
      {
        ogre_tools::Axes* axes_ = new ogre_tools::Axes( scene_manager_, node, axis_length_, axis_thickness_);
        axes_->getSceneNode()->setVisible( true);
        axes_->setOrientation(orientation);
      }
      Ogre::Matrix3 rotMat;
      orientation.ToRotationMatrix(rotMat);
      if(show_cov_)
      {
        for(int j = 0; j< 12; j++)
        {
          ogre_tools::Axes* axes_cov = new ogre_tools::Axes( scene_manager_, node, cov_length_, cov_thickness_ );
          Ogre::SceneNode* axis_tmp = axes_cov->getSceneNode();

          Ogre::Vector3 vec(  cov[0] * (j%6 == 0 ? 1 : 0) * (j>5?-1:1),
                              cov[7] * (j%6 == 1 ? 1 : 0) * (j>5?-1:1),
                              cov[14] * (j%6 == 2 ? 1 : 0) * (j>5?-1:1));

          /**  Position is relative but always in ogre coordinates */
           Ogre::Matrix3 rot (mat[0],mat[1],mat[2],
                              mat[4],mat[5],mat[6],
                              mat[8],mat[9],mat[10]);
          /*    normal -> ogre: x = -y, y = z, z = -x*/
          Ogre::Vector3 vec_trans = rot * vec;
          axis_tmp->setPosition(Ogre::Vector3(-vec_trans.y, vec_trans.z, -vec_trans.x));
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
      }
      if(show_text_)
      {
        ogre_tools::MovableText* text = new ogre_tools::MovableText(it->label, "Arial",0.1, Ogre::ColourValue::White);
        Ogre::SceneNode *text_node= node->createChildSceneNode();
        text_node->attachObject(text);
        text_node->setVisible( true);
      }
      node->setVisible(true);
    }
    if(render)
      causeRender();
  }
  return jlo_nodes_.back();
}

template<typename T>
void JloDisplayBase::setSceneNodePose(Ogre::SceneNode* scene_node, T mat, Ogre::Quaternion &orientation)
{
  if(m_binited)
  {
     std::string fxFrame = fixed_frame_;
     if(fixed_frame_.length() < 2)
       fxFrame = "/map";
     try
     {
       tf::Stamped<tf::Pose> pose_w(btTransform(btMatrix3x3(mat[0],mat[1],mat[2],
                                                         mat[4],mat[5],mat[6],
                                                         mat[8],mat[9],mat[10]),
                                  btVector3(mat[3], mat[7], mat[11])), ros::Time(),
                                  "/map");

      Ogre::Vector3 position;
      if(!vis_manager_->getFrameManager()->getTransform(fxFrame, ros::Time(), position, orientation, false))
      {
          ROS_ERROR("Error getting transforming frame '%s'",
            fxFrame.c_str());
            return;
      }
        /*vis_manager_->getTFClient()->transformPose(fixed_frame_, pose_w, pose_w);*/
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
    catch(...)
    {
      return;
    }
  }
}

} // namespace mapping_rviz_plugin


#endif /* RVIZ_POLYGONAL_MAP_DISPLAY_H_ */
