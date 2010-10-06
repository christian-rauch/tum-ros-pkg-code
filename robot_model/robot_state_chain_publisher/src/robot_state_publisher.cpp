/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include "robot_state_chain_publisher/robot_state_publisher.h"
#include <kdl/frames_io.hpp>
#include <tf_conversions/tf_kdl.h>

using namespace std;
using namespace ros;
using namespace KDL;



namespace robot_state_chain_publisher{

RobotStatePublisher::RobotStatePublisher(const Tree& tree)
  :n_("~"), tree_(tree)
{
  // get tf prefix
  n_.param("tf_prefix", tf_prefix_, string());

  // advertise tf message
  tf_publisher_ = n_.advertise<tf::tfMessage>("/tf", 5);
  tf_msg_.transforms.resize(tree.getNrOfSegments()-1);

  // get the 'real' root segment of the tree, which is the first child of "root"
  SegmentMap::const_iterator root = tree.getRootSegment();
  if (root->second.children.empty())
    throw empty_tree_ex;
  
  root_ = root->first;
  ignore_root = false;
  
  if (root_ == "world")
  {
    ignore_root = true;
    root_ = (*root->second.children.begin())->first;
  }
}



bool RobotStatePublisher::publishTransforms(map<string, RobotStatePublisher::JointState >& joint_positions, const Time& time, const ros::Time& republish_time)
{
  int i=0;
  if (ignore_root)
    tf_msg_.transforms.resize(tree_.getNrOfSegments()-1);
  else
    tf_msg_.transforms.resize(tree_.getNrOfSegments());
  addChildTransforms( joint_positions, tree_.getSegment( root_ ), i, time, republish_time );
  tf_msg_.transforms.resize(i);

  tf_publisher_.publish(tf_msg_);

  return true;
}

void RobotStatePublisher::addChildTransforms(map<string, RobotStatePublisher::JointState >& joint_positions, const SegmentMap::const_iterator segment, int &tf_index, const Time &time, const ros::Time& republish_time)
{
    double jnt_p = 0;
    bool publish=true;
    geometry_msgs::TransformStamped trans;

    for( vector<SegmentMap::const_iterator>::const_iterator child=segment->second.children.begin();
         child != segment->second.children.end(); child++ )
    {
        Time time_frame = time;
        if( (*child)->second.segment.getJoint().getType() != Joint::None )
        {
            map<string, RobotStatePublisher::JointState >::iterator jnt = joint_positions.find((*child)->second.segment.getJoint().getName());
            if (jnt == joint_positions.end()){
                jnt_p = 0;
            }
            else {
                jnt_p = jnt->second.pos;

                // if it's time for republishing, update timestamp
                if( jnt->second.time < republish_time ) {
                    jnt->second.time = time;
                    jnt->second.published = false;
                }

                time_frame = jnt->second.time;
                publish = ! jnt->second.published;
                jnt->second.published = true;
            }
        }

        // only publish new transforms
        if( publish ) {
          Frame frame = (*child)->second.segment.pose( jnt_p );
          tf::Transform tf_frame;
          tf::TransformKDLToTF(frame, tf_frame);
          trans.header.stamp = time_frame;
          trans.header.frame_id = tf::resolve(tf_prefix_, segment->first);
          trans.child_frame_id = tf::resolve(tf_prefix_, (*child)->first);
          if (trans.child_frame_id == string(""))
          { 
          
          }
          tf::transformTFToMsg(tf_frame, trans.transform);
          tf_msg_.transforms[tf_index++] = trans;
        }

        addChildTransforms(joint_positions, *child, tf_index, time, republish_time);
    }
}

}
