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

#include <vision_msgs/cop_answer.h>
#include "jlo_display_base.h"
#include "SimpleGraspPlanner.h"
namespace rviz_shows_cop
{

/**
 * \class CopAnswerDisplay
 * \brief Displays a vision_msgs::CopAnswer message
 */
class CopAnswerDisplay : public JloDisplayBase
{
public:
  CopAnswerDisplay(const std::string& name, rviz::VisualizationManager* manager);

  virtual ~CopAnswerDisplay();

  virtual void setTopic(const std::string& topic);

  void setSGP	(bool showGrapsPlanner);
  bool getSGP()
  {
    return (m_sgp);
  }
  void setHand	(bool rightHand);
  bool getHand()
  {
    return (m_hand);
  }
  void setSGPColor	(rviz::Color color);
  rviz::Color getSGPColor()
  {
    return (m_sgpColor);
  }
  void setTH	(float showGrapsPlanner);
  float getTH()
  {
    return (m_th);
  }

  void setOffset	(float offset);
  float getOffset()
  {
    return (m_offset);
  }
  void setOfftop	(float offset);
  float getOfftop()
  {
    return (m_offtop);
  }

  void setA	(float showGrapsPlanner);
  float getA()
  {
    return (alpha);
  }
  void setB	(float showGrapsPlanner);
  float getB()
  {
    return (beta);
  }
  void setD	(float showGrapsPlanner);
  float getD()
  {
    return (delta);
  }
  void setAuto	(bool rightHand);
  bool getAuto()
  {
    return (m_auto);
  }


  virtual void createProperties();
protected:
  void subscribe();
  void unsubscribe();
  void incomingMessage(const vision_msgs::cop_answerConstPtr& msg);
  void processMessage(vision_msgs::cop_answer& msg);
  void processMessage(){return processMessage(m_currentMessage);};

  ros::Subscriber cop_subscriber;

  void AttachSGPPoints(Ogre::SceneNode* object, std::vector<vision_msgs::partial_lo::_pose_type > matrices,
                                        std::vector<vision_msgs::partial_lo::_cov_type> covs, size_t index);
  double GetOffsetBaseLinkRotZ(Ogre::Quaternion quat);


  bool m_sgp;
  bool m_hand;
  bool m_auto;
  float m_th;
  float m_offset;
  float m_offtop;
  rviz::Color m_sgpColor;
  HandConfig m_manConf;
  float alpha;
  float beta;
  float delta;

  rviz::BoolPropertyWPtr m_sgp_property;
  rviz::BoolPropertyWPtr m_hand_property;
  rviz::ColorPropertyWPtr m_sgpColor_prop;
  rviz::FloatPropertyWPtr m_th_property;
  rviz::FloatPropertyWPtr m_offset_property;
  rviz::FloatPropertyWPtr m_offtop_property;
  vision_msgs::cop_answer m_currentMessage;

  rviz::BoolPropertyWPtr  m_auto_prop;
  rviz::FloatPropertyWPtr m_alpha_prop;
  rviz::FloatPropertyWPtr m_beta_prop;
  rviz::FloatPropertyWPtr m_delta_prop;



};

} // namespace mapping_rviz_plugin

#endif /* RVIZ_POLYGONAL_MAP_DISPLAY_H_ */
