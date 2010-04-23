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

#ifndef RVIZ_POSITION_STRING_LIST_DISPLAY_H_
#define RVIZ_POSITION_STRING_LIST_DISPLAY_H_

#include "rviz/display.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"

#include <boost/thread/mutex.hpp>

#include <boost/shared_ptr.hpp>

#include <position_string_rviz_plugin/PositionStringList.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace ogre_tools
{
class MovableText;
}

namespace Ogre
{
class SceneNode;
class ManualObject;
}

namespace positionstring_rviz_plugin
{

/**
 * \class PositionStringDisplay
 * \brief Displays a String at a certain position in space
 */
class PositionStringListDisplay : public rviz::Display
{
public:
  PositionStringListDisplay(const std::string& name, rviz::VisualizationManager* manager);

  virtual ~PositionStringListDisplay();

  void setTopic(const std::string& topic);
  const std::string& getTopic()
  {
    return (topic_);
  }

  void setColor(const rviz::Color& color);
  const rviz::Color& getColor()
  {
    return (color_);
  }
  
  void setCharacterHeight(const double & h);
  const double& getCharacterHeight()
  {
    return (character_height_);
  }

  // Overrides from Display
  virtual void targetFrameChanged();
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

protected:
  void subscribe();
  void unsubscribe();
  void clear();
  void incomingMessage(const position_string_rviz_plugin::PositionStringList::ConstPtr& message);
  void processMessage(const position_string_rviz_plugin::PositionStringList::ConstPtr& message);
  void setNumberTextItems (unsigned int n);
  void clearTextItems ();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string topic_;
  rviz::Color color_;
  double character_height_;
  bool new_msg_;
  position_string_rviz_plugin::PositionStringList::ConstPtr current_message_;
  std::vector<Ogre::SceneNode*> scene_nodes_;
  std::vector<ogre_tools::MovableText*> text_objects_; 

  message_filters::Subscriber<position_string_rviz_plugin::PositionStringList> sub_;
  tf::MessageFilter<position_string_rviz_plugin::PositionStringList> tf_filter_;

  rviz::DoublePropertyWPtr character_height_property_;
  rviz::ColorPropertyWPtr color_property_;
  rviz::ROSTopicStringPropertyWPtr topic_property_;
};

} // namespace positionstring_rviz_plugin

#endif /* RVIZ_POSITION_STRING_LIST_DISPLAY_H_ */
