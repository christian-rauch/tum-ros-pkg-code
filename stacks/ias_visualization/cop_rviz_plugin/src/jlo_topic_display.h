/*
 * Copyright (c) 2010, Lorenz Moesenlechner
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

#ifndef JLO_TOPIC_DISPLAY_H
#define JLO_TOPIC_DISPLAY_H

#include <std_msgs/UInt32.h>

#include "jlo_display_base.h"

namespace rviz_shows_cop
{

/**
 * \class JloTopicDisplay
 * \brief Displays a jlo id from a uint32 topic.
 */
class JloTopicDisplay : public JloDisplayBase
{
public:
  JloTopicDisplay(const std::string& name, rviz::VisualizationManager* manager);

  virtual ~JloTopicDisplay();

  void setTopic(const std::string& topic);
  void processMessage(){if(inited)incomingMessage(m_currMessage);}
  virtual void createProperties();


protected:
  void subscribe();
  void unsubscribe();
  void incomingMessage(const std_msgs::UInt32ConstPtr& msg);


  ros::Subscriber jlo_subscriber;
  bool inited;
  std_msgs::UInt32ConstPtr m_currMessage;
};

} // namespace mapping_rviz_plugin

#endif
