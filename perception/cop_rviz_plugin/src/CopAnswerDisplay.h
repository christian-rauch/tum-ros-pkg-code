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

#ifndef COPANSWERDISPLAY_H
#define COPANSWERDISPLAY_H

#include "rviz/display.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"

#include <ogre_tools/billboard_line.h>
#include <boost/thread/mutex.hpp>

#include <boost/shared_ptr.hpp>

#include <vision_msgs/cop_answer.h>
#include <vision_srvs/srvjlo.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>


namespace ogre_tools
{
class PointCloud;
}

namespace Ogre
{
class SceneNode;
class ManualObject;
}

namespace rviz_shows_cop
{

/**
 * \class CopAnswerDisplay
 * \brief Displays a vision_msgs::CopAnswer message
 */
class CopAnswerDisplay : public rviz::Display
{
public:
  CopAnswerDisplay(const std::string& name, rviz::VisualizationManager* manager);

  virtual ~CopAnswerDisplay();

  void setTopic(const std::string& topic);
  const std::string& getTopic()
  {
    return (topic_);
  }

  void setAlpha(float alpha);
  float getAlpha()
  {
    return (alpha_);
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
  void incomingMessage(const vision_msgs::cop_answerConstPtr& msg);
  void processMessage(const vision_msgs::cop_answerConstPtr& msg);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string topic_;
  rviz::Color color_;
  int render_operation_;
  float alpha_;

  Ogre::SceneNode* scene_node_;
  Ogre::ManualObject* manual_object_;

  vision_msgs::cop_answerConstPtr current_message_;

  rviz::ROSTopicStringPropertyWPtr topic_property_;
  rviz::FloatPropertyWPtr alpha_property_;


  ros::Subscriber cop_subscriber;

  bool inited_jlo;
  ros::ServiceClient jlo_client;
  bool GetJlo(unsigned long id, unsigned long parent_id, std::vector<double> &mat, std::vector<double> &cov);
  void setSceneNodePose(Ogre::SceneNode* scene_node, std::vector<double> map, Ogre::Quaternion &orientation);


};

} // namespace mapping_rviz_plugin

#endif /* RVIZ_POLYGONAL_MAP_DISPLAY_H_ */
