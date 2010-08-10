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

#include "jlo_display_base.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"

#include <boost/bind.hpp>

#include <algorithm>

using namespace vision_srvs;
namespace rviz_shows_cop
{

JloDisplayBase::JloDisplayBase(const std::string & name,
  rviz::VisualizationManager * manager)
  : Display(name, manager),
    keep_(1),
    show_text_(true),
    show_axis_(true),
    show_cov_(true),
    axis_length_(0.2),
    axis_thickness_(0.02),
    cov_length_(0.05),
    cov_thickness_(0.005)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  inited_jlo = false;
}

JloDisplayBase::~JloDisplayBase()
{
  if(m_binited)
    clear();
}

void JloDisplayBase::clear()
{
  if(jlo_nodes_.size() > 0)
  {
    scene_node_->removeAndDestroyAllChildren();
    jlo_nodes_.clear();
  }
}

void JloDisplayBase::setTopic(const std::string & topic)
{
  topic_ = topic;
  propertyChanged(topic_property_);
  if(m_binited)
  {
    clear();
    causeRender();
  }
}


void JloDisplayBase::setKeep(uint32_t keep)
{
  keep_ = keep;
  propertyChanged(keep_property_);
  while( jlo_nodes_.size() > keep_ )
    popJloSet();
  processMessage();
}

void JloDisplayBase::onEnable()
{
  scene_node_->setVisible(true);
}

void JloDisplayBase::onDisable()
{
  scene_node_->setVisible(false);
  if(m_binited)
    clear();
}

void JloDisplayBase::fixedFrameChanged()
{
  if(m_binited)
    clear();
}

void JloDisplayBase::update(float wall_dt, float ros_dt)
{
}

void JloDisplayBase::popJloSet()
{
  while( jlo_nodes_.front().size() )
  {
    jlo_nodes_.front().front().first->removeAndDestroyAllChildren();
    delete jlo_nodes_.front().front().first;
    jlo_nodes_.front().pop_front();
  }
  jlo_nodes_.pop_front();
}

bool JloDisplayBase::GetJlo(unsigned long id, unsigned long parent_id,
    vision_msgs::partial_lo  &lo)
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
  lo = msg.response.answer;
  return true;
}

unsigned  long JloDisplayBase::NameQueryJlo(std::string name)
{
  srvjlo msg;
  msg.request.command = "namequery";
  if(name.length() < 2)
    return 1;
  msg.request.query.name = name;

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
    return 1;
  }
  return msg.response.answer.id;
}



void JloDisplayBase::reset()
{
  if(m_binited)
    clear();
}

void JloDisplayBase::targetFrameChanged()
{
}

void JloDisplayBase::setST( bool show_text )
{
  show_text_ = show_text;
  propertyChanged(show_text_p_);
  processMessage();
}

void JloDisplayBase::setSA( bool show_axis )
{
  show_axis_ = show_axis;
  propertyChanged(show_axis_p_);
  processMessage();
}

void JloDisplayBase::setSC( bool show_cov)
{
  show_cov_ = show_cov;
  propertyChanged(show_cov_p_);
  processMessage();
}
void JloDisplayBase::setAL(double axis_length)
{
  axis_length_ = axis_length;
  propertyChanged(axis_length_p_);
  processMessage();
}
void JloDisplayBase::setAT(double axis_thick)
{
  axis_thickness_ = axis_thick;
  propertyChanged(axis_thickness_p_);
  processMessage();
}
void JloDisplayBase::setCL(double cov_length)
{
  cov_length_ = cov_length;
  propertyChanged(cov_length_p_);
  processMessage();
}
void JloDisplayBase::setCT(double cov_thick)
{
  cov_thickness_ = cov_thick;
  propertyChanged(cov_thickness_p_);
  processMessage();
}

void JloDisplayBase::createProperties()
{



  keep_property_ = property_manager_->createProperty<rviz::IntProperty>( "Keep", property_prefix_, boost::bind( &JloDisplayBase::getKeep, this ),
                                                                    boost::bind( &JloDisplayBase::setKeep, this, _1 ), parent_category_, this );



  extended_props_p_ = property_manager_->createCategory("Appearance Properties", property_prefix_, parent_category_);
  show_text_p_ = property_manager_->createProperty<rviz::BoolProperty>( "Show Text", property_prefix_, boost::bind( &JloDisplayBase::getST, this ),
                                                                      boost::bind( &JloDisplayBase::setST, this, _1 ), extended_props_p_, this );

  show_axis_p_ = property_manager_->createProperty<rviz::BoolProperty>( "Show Axis", property_prefix_, boost::bind( &JloDisplayBase::getSA, this ),
                                                                      boost::bind( &JloDisplayBase::setSA, this, _1 ), extended_props_p_, this );
  axis_length_p_ = property_manager_->createProperty<rviz::FloatProperty>( "Axis Length", property_prefix_, boost::bind( &JloDisplayBase::getAL, this ),
                                                                      boost::bind( &JloDisplayBase::setAL, this, _1 ), extended_props_p_, this );
  axis_thickness_p_ = property_manager_->createProperty<rviz::FloatProperty>( "Axis Thickness", property_prefix_, boost::bind( &JloDisplayBase::getAT, this ),
                                                                      boost::bind( &JloDisplayBase::setAT, this, _1 ), extended_props_p_, this );

  show_cov_p_ = property_manager_->createProperty<rviz::BoolProperty>( "Show Covariance", property_prefix_, boost::bind( &JloDisplayBase::getSC, this ),
                                                                      boost::bind( &JloDisplayBase::setSC, this, _1 ), extended_props_p_, this );

  cov_length_p_ =  property_manager_->createProperty<rviz::FloatProperty>( "Covariance Length", property_prefix_, boost::bind( &JloDisplayBase::getCL, this ),
                                                                      boost::bind( &JloDisplayBase::setCL, this, _1 ), extended_props_p_, this );
  cov_thickness_p_ = property_manager_->createProperty<rviz::FloatProperty>( "Covariance Thickness", property_prefix_, boost::bind( &JloDisplayBase::getCT, this ),
                                                                      boost::bind( &JloDisplayBase::setCT, this, _1 ), extended_props_p_, this );
}

} // namespace mapping_rviz_plugin
