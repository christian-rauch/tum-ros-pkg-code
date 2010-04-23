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

#ifndef RVIZ_TRIANGULAR_MESH_DISPLAY_H_
#define RVIZ_TRIANGULAR_MESH_DISPLAY_H_

#include "rviz/display.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"

#include <boost/thread/mutex.hpp>

#include <boost/shared_ptr.hpp>

#include <triangle_mesh/TriangleMesh.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <OGRE/OgreMesh.h>
#include <OGRE/OgreSubMesh.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreHardwareBufferManager.h>

namespace Ogre
{
class SceneNode;
class ManualObject;
}

namespace triangle_mesh
{

/**
 * \class TriangleMeshDisplay
 * \brief Displays a String at a certain position in space
 */
class TriangleMeshDisplay : public rviz::Display
{
public:
  TriangleMeshDisplay(const std::string& name, rviz::VisualizationManager* manager);

  virtual ~TriangleMeshDisplay();

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
  
  void setWireFrameEnabled(const bool& wf);
  const bool& getWireFrameEnabled()
  {
    return (wf_);
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
  void incomingMessage(const triangle_mesh::TriangleMesh::ConstPtr& message);
  void processMessage(const triangle_mesh::TriangleMesh::ConstPtr& message);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string topic_;
  rviz::Color color_;
  bool wf_;
  triangle_mesh::TriangleMesh::ConstPtr current_message_;
  
  Ogre::SceneNode* scene_node_;
  
  //this stuff needs to be freed every time we get a new message..
  Ogre::MeshPtr mesh_;
  Ogre::SubMesh* submesh_;
  Ogre::HardwareVertexBufferSharedPtr vbuf; 
  Ogre::VertexData* data;
  Ogre::HardwareIndexBufferSharedPtr ibuf;
  Ogre::Entity* entity_;

  message_filters::Subscriber<triangle_mesh::TriangleMesh> sub_;
  tf::MessageFilter<triangle_mesh::TriangleMesh> tf_filter_;

  rviz::BoolPropertyWPtr wireframe_property_;
  rviz::ColorPropertyWPtr color_property_;
  rviz::ROSTopicStringPropertyWPtr topic_property_;
  
  int count;
};

} // namespace positionstring_rviz_plugin

#endif /* RVIZ_TRIANGULAR_MESH_DISPLAY_H_ */

