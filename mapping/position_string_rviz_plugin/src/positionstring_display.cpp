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

#include "positionstring_display.h"
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

namespace positionstring_rviz_plugin
{

PositionStringDisplay::PositionStringDisplay(const std::string & name, rviz::VisualizationManager * manager)
: Display(name, manager)
, color_(0.1f, 1.0f, 0.0f)
, character_height_(0.1)
, tf_filter_(*manager->getTFClient(), "", 2, update_nh_)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "PositionString " << count++;

  text_ = new ogre_tools::MovableText("positionstring", "Arial", character_height_, Ogre::ColourValue::White);
  text_->showOnTop (true);
  scene_node_->attachObject(text_);

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&PositionStringDisplay::incomingMessage, this, _1));
}

PositionStringDisplay::~PositionStringDisplay()
{
  unsubscribe();

  delete text_;
}


void PositionStringDisplay::setTopic(const std::string & topic)
{
  unsubscribe();
  topic_ = topic;
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void PositionStringDisplay::setColor(const rviz::Color & color)
{
  color_ = color;

  propertyChanged(color_property_);
  processMessage(current_message_);
  causeRender();
}

void PositionStringDisplay::setCharacterHeight(const double & h)
{
  character_height_ = h;

  propertyChanged(character_height_property_);
  processMessage(current_message_);
  causeRender();
}

void PositionStringDisplay::subscribe()
{
  if (!isEnabled())
    return;

  if (!topic_.empty())
  {
    sub_.subscribe(update_nh_, topic_, 1);
  }
}

void PositionStringDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void PositionStringDisplay::onEnable()
{
  scene_node_->setVisible(true);
  subscribe();
}

void PositionStringDisplay::onDisable()
{
  unsubscribe();
  scene_node_->setVisible(false);
}

void PositionStringDisplay::fixedFrameChanged()
{
  tf_filter_.setTargetFrame(fixed_frame_);
}

void PositionStringDisplay::update(float wall_dt, float ros_dt)
{
}

void PositionStringDisplay::processMessage(const position_string_rviz_plugin::PositionString::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  
  text_->setCaption (msg->text);
  text_->setCharacterHeight (character_height_);
  text_->setColor (Ogre::ColourValue(color_.r_, color_.g_, color_.b_));

  tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(1.0f, 0.0f, 0.0f, 0.0f),
      btVector3(msg->pose.x, msg->pose.y, msg->pose.z)), msg->header.stamp,
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

  scene_node_->setPosition(position);
}

void PositionStringDisplay::incomingMessage(const position_string_rviz_plugin::PositionString::ConstPtr& message)
{
  current_message_ = message;
  processMessage(message);
}

void PositionStringDisplay::reset()
{
}

void PositionStringDisplay::targetFrameChanged()
{
}

void PositionStringDisplay::createProperties()
{
  character_height_property_ = property_manager_->createProperty<rviz::DoubleProperty> ("Character Height", property_prefix_, boost::bind(&PositionStringDisplay::getCharacterHeight, this),
                                                                            boost::bind(&PositionStringDisplay::setCharacterHeight, this, _1), parent_category_, this);
  color_property_ = property_manager_->createProperty<rviz::ColorProperty> ("Color", property_prefix_, boost::bind(&PositionStringDisplay::getColor, this),
                                                                            boost::bind(&PositionStringDisplay::setColor, this, _1), parent_category_, this);
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> ("Topic", property_prefix_, boost::bind(&PositionStringDisplay::getTopic, this),
                                                                               boost::bind(&PositionStringDisplay::setTopic, this, _1), parent_category_, this);
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::DataType<position_string_rviz_plugin::PositionString>().value());
}

} // namespace positionstring_rviz_plugin 
