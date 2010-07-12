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
 *
 */

#include "jlo_topic_display.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"

#include "rviz/common.h"

#include <sstream>

using namespace vision_srvs;
namespace rviz_shows_cop
{

JloTopicDisplay::JloTopicDisplay(const std::string & name,
                                         rviz::VisualizationManager * manager)
  : JloDisplayBase(name, manager),
  inited(false)
{
}

JloTopicDisplay::~JloTopicDisplay()
{
  unsubscribe();
}

void JloTopicDisplay::setTopic(const std::string & topic)
{
  unsubscribe();
  JloDisplayBase::setTopic(topic);
  inited = false;
  subscribe();
}


void JloTopicDisplay::subscribe()
{
  if (!isEnabled() || topic_.length() < 2)
    return;

  jlo_subscriber = update_nh_.subscribe<std_msgs::UInt32>(topic_,1, boost::bind(&JloTopicDisplay::incomingMessage, this, _1));
}

void JloTopicDisplay::unsubscribe()
{
  jlo_subscriber.shutdown();
}

void JloTopicDisplay::incomingMessage (const std_msgs::UInt32ConstPtr& msg)
{
  JloDisplayBase::JloDescription jlo_descr[1];

  m_currMessage = msg;
  inited = true;

  std::stringstream strm;

  jlo_descr[0].pose = msg->data;
  strm << msg->data;
  jlo_descr[0].label = strm.str();
  try
  {
  displayJloSet(&jlo_descr[0], &jlo_descr[1]);
  }
  catch(...)
  {
    ROS_INFO("tf sucks, please start a state publisher or similar stuff\n");
  }
}

void JloTopicDisplay::createProperties()
{
  /*lor_property_ = property_manager_->createProperty<rviz::ColorProperty> ("Color", property_prefix_, boost::bind(&CopAnswerDisplay::getColor, this),
                                                                      boost::bind(&CopAnswerDisplay::setColor, this, _1), parent_category_, this);*/
  JloDisplayBase::createProperties();

  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> ("Topic", property_prefix_, boost::bind(&JloDisplayBase::getTopic, this),
                                                                               boost::bind(&JloDisplayBase::setTopic, this, _1), parent_category_, this);
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(std_msgs::UInt32::__s_getDataType());

}


} // namespace mapping_rviz_plugin
