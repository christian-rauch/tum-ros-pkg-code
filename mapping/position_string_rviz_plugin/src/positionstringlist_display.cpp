/* 
 * Copyright (c) 2010, Nico Blodow (blodow@cs.tum.edu)
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

#include "positionstringlist_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/common.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <ogre_tools/movable_text.h>

namespace position_string_rviz_plugin
{

PositionStringListDisplay::PositionStringListDisplay(const std::string & name, rviz::VisualizationManager * manager)
: Display(name, manager)
, color_(0.1f, 1.0f, 0.0f)
, character_height_(0.1)
, new_msg_(true)
, tf_filter_(*manager->getTFClient(), "", 2, update_nh_)
{
  static int count = 0;
  std::stringstream ss;
  ss << "PositionStringList " << count++;

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&PositionStringListDisplay::incomingMessage, this, _1));
}

PositionStringListDisplay::~PositionStringListDisplay()
{
  unsubscribe();
  clearTextItems ();
}

void PositionStringListDisplay::setNumberTextItems (unsigned int n)
{
  if (n == scene_nodes_.size())
    return;
  if (n < scene_nodes_.size())
  {
    unsigned int needed = scene_nodes_.size() - n;
    for (unsigned int too_many = 0; too_many < needed; too_many++)
    {
      //TODO: Is this enough? This a memory leak?
      scene_manager_->getRootSceneNode ()->removeChild (scene_nodes_.back());
      scene_nodes_.pop_back();
      text_objects_.pop_back();
    }
  }
  else 
  {
    unsigned int needed = n - scene_nodes_.size();
    for (unsigned int additional = 0; additional < needed; additional++)
    {
      Ogre::SceneNode* scene_node = scene_manager_->getRootSceneNode()->createChildSceneNode();
      ogre_tools::MovableText *text = new ogre_tools::MovableText("(empty)", "Arial", character_height_, Ogre::ColourValue(color_.r_, color_.g_, color_.b_));
      text->showOnTop (true);
      scene_node->attachObject(text);
  
      text_objects_.push_back (text);
      scene_nodes_.push_back (scene_node);
    }
  }
}

void PositionStringListDisplay::clearTextItems ()
{
  // TODO: This a memory leak??
  for (std::vector<Ogre::SceneNode*>::iterator it = scene_nodes_.begin(); it != scene_nodes_.end(); it++)
    (*it)->detachAllObjects();

  for (std::vector<ogre_tools::MovableText*>::iterator it = text_objects_.begin(); it != text_objects_.end(); it++)
    delete (*it);
}

void PositionStringListDisplay::setTopic(const std::string & topic)
{
  unsubscribe();
  topic_ = topic;
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void PositionStringListDisplay::setColor(const rviz::Color & color)
{
  color_ = color;

  propertyChanged(color_property_);
  processMessage(current_message_);
  causeRender();
}

void PositionStringListDisplay::setCharacterHeight(const double & h)
{
  character_height_ = h;

  propertyChanged(character_height_property_);
  processMessage(current_message_);
  causeRender();
}

void PositionStringListDisplay::subscribe()
{
  if (!isEnabled())
    return;

  if (!topic_.empty())
  {
    sub_.subscribe(update_nh_, topic_, 1);
  }
}

void PositionStringListDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void PositionStringListDisplay::onEnable()
{
  for (std::vector<Ogre::SceneNode*>::iterator it = scene_nodes_.begin(); it != scene_nodes_.end(); it++)
    (*it)->setVisible(true);
  subscribe();
}

void PositionStringListDisplay::onDisable()
{
  unsubscribe();
  for (std::vector<Ogre::SceneNode*>::iterator it = scene_nodes_.begin(); it != scene_nodes_.end(); it++)
    (*it)->setVisible(false);
}

void PositionStringListDisplay::fixedFrameChanged()
{
  tf_filter_.setTargetFrame(fixed_frame_);
}

void PositionStringListDisplay::update(float wall_dt, float ros_dt)
{
}

void PositionStringListDisplay::processMessage(const position_string_rviz_plugin::PositionStringList::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  
  if (new_msg_) // new message means new text
  {
    setNumberTextItems (msg->texts.size ());
    for (unsigned int i = 0; i < msg->texts.size(); i++)
    {
      text_objects_[i]->setCaption(msg->texts[i]);
      if (i < msg->poses.size() && (fabs(msg->poses[i].x) != std::numeric_limits<double>::infinity())
                                && (fabs(msg->poses[i].y) != std::numeric_limits<double>::infinity())
                                && (fabs(msg->poses[i].z) != std::numeric_limits<double>::infinity()))
      {
        msg->poses[i];
        tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(1.0f, 0.0f, 0.0f, 0.0f),
            btVector3(msg->poses[i].x, msg->poses[i].y, msg->poses[i].z)), msg->header.stamp,
            msg->header.frame_id);
      
        try
        {
          vis_manager_->getTFClient()->transformPose(fixed_frame_, pose, pose);
        }
        catch (tf::TransformException & e)
        {
          ROS_ERROR("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), fixed_frame_.c_str());
        }
      
        Ogre::Vector3 position = Ogre::Vector3(pose.getOrigin().x(),
            pose.getOrigin().y(), pose.getOrigin().z());
      
        rviz::robotToOgre(position);

        scene_nodes_[i]->setPosition(position);
      }
      else // this just catches the case that not enough poses were given in the message.
        scene_nodes_[i]->setPosition (Ogre::Vector3(0,0,0));
    }
  }
  else  // otherwise, only settings (color/fontsize) has changed
  {
    for (std::vector<ogre_tools::MovableText*>::iterator it = text_objects_.begin(); it != text_objects_.end(); it++)
    {
      (*it)->setCharacterHeight (character_height_);
      (*it)->setColor (Ogre::ColourValue(color_.r_, color_.g_, color_.b_));
    }
  }
}

void PositionStringListDisplay::incomingMessage(const position_string_rviz_plugin::PositionStringList::ConstPtr& message)
{
  current_message_ = message;
  new_msg_ = true;
  processMessage(message);
  new_msg_ = false;
}

void PositionStringListDisplay::reset()
{
}

void PositionStringListDisplay::targetFrameChanged()
{
}

void PositionStringListDisplay::createProperties()
{
  character_height_property_ = property_manager_->createProperty<rviz::DoubleProperty> ("Character Height", property_prefix_, boost::bind(&PositionStringListDisplay::getCharacterHeight, this),
                                                                            boost::bind(&PositionStringListDisplay::setCharacterHeight, this, _1), parent_category_, this);
  color_property_ = property_manager_->createProperty<rviz::ColorProperty> ("Color", property_prefix_, boost::bind(&PositionStringListDisplay::getColor, this),
                                                                            boost::bind(&PositionStringListDisplay::setColor, this, _1), parent_category_, this);
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> ("Topic", property_prefix_, boost::bind(&PositionStringListDisplay::getTopic, this),
                                                                               boost::bind(&PositionStringListDisplay::setTopic, this, _1), parent_category_, this);
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::DataType<position_string_rviz_plugin::PositionStringList>().value());
}

} // namespace position_string_rviz_plugin
