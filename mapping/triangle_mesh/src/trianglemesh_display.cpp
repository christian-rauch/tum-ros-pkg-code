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

#include "trianglemesh_display.h"
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

typedef struct {int a,b,c;} triangle;
////////////////////////////////////////////////////////////////////////////////
namespace triangle_mesh
{

TriangleMeshDisplay::TriangleMeshDisplay(const std::string & name, rviz::VisualizationManager * manager)
  : Display(name, manager)
  , color_(0.1f, 1.0f, 0.0f)
  , wf_(false)
  , tf_filter_(*manager->getTFClient(), "", 2, update_nh_)
  , count (0)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "TriangleMesh " << count++;
  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&TriangleMeshDisplay::incomingMessage, this, _1));
}

TriangleMeshDisplay::~TriangleMeshDisplay()
{
  unsubscribe();
}

void TriangleMeshDisplay::setTopic(const std::string & topic)
{
  unsubscribe();
  topic_ = topic;
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void TriangleMeshDisplay::setColor(const rviz::Color & color)
{
  color_ = color;

  propertyChanged(color_property_);
  processMessage(current_message_);
  causeRender();
}

void TriangleMeshDisplay::setWireFrameEnabled(const bool & wf)
{
  wf_ = wf;

  propertyChanged(wireframe_property_);
  processMessage(current_message_);
  causeRender();
}

void TriangleMeshDisplay::subscribe()
{
  if (!isEnabled())
    return;

  if (!topic_.empty())
  {
    sub_.subscribe(update_nh_, topic_, 1);
  }
}

void TriangleMeshDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void TriangleMeshDisplay::onEnable()
{
  scene_node_->setVisible(true);
  subscribe();
}

void TriangleMeshDisplay::onDisable()
{
  unsubscribe();
  scene_node_->setVisible(false);
}

void TriangleMeshDisplay::fixedFrameChanged()
{
  tf_filter_.setTargetFrame(fixed_frame_);
}

void TriangleMeshDisplay::update(float wall_dt, float ros_dt)
{
}

/** @todo */
//std::vector<geometry_msgs::Point32> computeNormals (const triangle_mesh::TriangleMesh::ConstPtr& msg)
//{
//  std::vector <std::vector <int> > reverse_triangles;
//  for (unsigned int i = 0; i < msg->triangles.size (); i++)
//  {
//
//  }
//  return msg->normals;
//}

void TriangleMeshDisplay::processMessage(const triangle_mesh::TriangleMesh::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  if (count++ > 0)
  {
/** @todo  not sure .. */
//    delete vbuf.get();
//    delete data;
//    delete ibuf;
    scene_manager_->destroyEntity(entity_);
    Ogre::MeshManager::getSingleton().unload ("trianglemesh");
    Ogre::MeshManager::getSingleton().remove ("trianglemesh");
  }
  std::vector<geometry_msgs::Point32> normals;
  bool use_normals = false;

/** @todo */
//  if (msg->normals.size() == msg->points.size())
//  {
//    normals = msg->normals;
//    use_normals = true;
//  }
//  else
//  {
//    normals = computeNormals (msg);
//  }

/** @todo */
//  std::vector<triangle> triangles;
//  for (unsigned int i = 0; i < msg->triangles.size(); i++)
//  {
//    triangle tr;
//    tr.a = msg->triangles[i].i;
//    tr.b = msg->triangles[i].j;
//    tr.c = msg->triangles[i].k;
//    triangles.push_back (tr);
//  }
  mesh_ = Ogre::MeshManager::getSingleton().createManual ("trianglemesh", "General");
  submesh_ = mesh_->createSubMesh();

  // We first create a VertexData
  data = new Ogre::VertexData();
  // Then, we link it to our Mesh/SubMesh :
  mesh_->sharedVertexData = data;
  // submesh_->useSharedVertices = false; // This value is 'true' by default
  // submesh_->vertexData = data;
  // We have to provide the number of verteices we'll put into this Mesh/SubMesh
  data->vertexCount = msg->points.size();
  // Then we can create our VertexDeclaration
  Ogre::VertexDeclaration* decl = data->vertexDeclaration;
  decl->addElement (0,0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  /** @todo: should add more elements, specifically normal vectors! */
  if (use_normals)
    decl->addElement (0,Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3), Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
  
  // create a vertex buffer
  vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
    decl->getVertexSize(0),                     // This value is the size of a vertex in memory
    msg->points.size(),                         // The number of vertices you'll put into this buffer
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY // Properties
  );
 
  double min_x=FLT_MAX,  min_y=FLT_MAX,  min_z=FLT_MAX;
  double max_x=-FLT_MAX, max_y=-FLT_MAX, max_z=-FLT_MAX;
  int vertex_size_in_bytes = 3;
  if (use_normals)
    vertex_size_in_bytes = 6;
  float* array = (float*) malloc (sizeof(float)*vertex_size_in_bytes*msg->points.size());
  for (unsigned int i = 0; i < msg->points.size(); i++)
  {
    array[i*vertex_size_in_bytes+0] = msg->points[i].x;
    array[i*vertex_size_in_bytes+1] = msg->points[i].y;
    array[i*vertex_size_in_bytes+2] = msg->points[i].z;
    if (use_normals)
    {
      array[i*vertex_size_in_bytes+3] = normals[i].x;
      array[i*vertex_size_in_bytes+4] = normals[i].x;
      array[i*vertex_size_in_bytes+5] = normals[i].x;
    }
    if (msg->points[i].x < min_x) min_x = msg->points[i].x;
    if (msg->points[i].y < min_y) min_y = msg->points[i].y;
    if (msg->points[i].z < min_z) min_z = msg->points[i].z;
    if (msg->points[i].x > max_x) max_x = msg->points[i].x;
    if (msg->points[i].y > max_y) max_y = msg->points[i].y;
    if (msg->points[i].z > max_z) max_z = msg->points[i].z;
  }
  
  vbuf->writeData (0, vbuf->getSizeInBytes(), array, true); 
  free (array);
  
  Ogre::VertexBufferBinding* bind = data->vertexBufferBinding;
  bind->setBinding (0, vbuf);
  
  bool back_face_duplicate = true;
  int stride = 3;  
  if (back_face_duplicate)
    stride = 6;

  // create index buffer
  ibuf = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
    Ogre::HardwareIndexBuffer::IT_32BIT,        // You can use several different value types here
    msg->triangles.size()*stride,                    // The number of indices you'll put in that buffer
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY // Properties
  );
  
  submesh_->indexData->indexBuffer = ibuf;     // The pointer to the index buffer

  submesh_->indexData->indexCount = msg->triangles.size()*stride; // The number of indices we'll use
  submesh_->indexData->indexStart = 0;

  unsigned int *index;
  index = (unsigned int*) malloc (sizeof(unsigned int)*stride*msg->triangles.size());
  for (unsigned int i = 0; i < msg->triangles.size(); i++)
  {
    index[i*stride+0] = msg->triangles[i].i;
    index[i*stride+1] = msg->triangles[i].j;
    index[i*stride+2] = msg->triangles[i].k;
    if (back_face_duplicate)
    {
      index[i*stride+3] = msg->triangles[i].i;
      index[i*stride+4] = msg->triangles[i].k;
      index[i*stride+5] = msg->triangles[i].j;
    }
  }

  ibuf->writeData (0, ibuf->getSizeInBytes(), index, true); 
  free (index);
 
  mesh_->_setBounds (Ogre::AxisAlignedBox (min_x, min_y, min_z, max_x, max_y, max_z));
  mesh_->_setBoundingSphereRadius (std::max(max_x-min_x, std::max(max_y-min_y, max_z-min_z))/2.0f);

  mesh_->load ();

  std::stringstream ss;
  ss << "tm" << count;
  entity_ = scene_manager_->createEntity(ss.str(), "trianglemesh");
  for (unsigned int i = 0; i < entity_->getNumSubEntities (); i++)
  {
    entity_->getSubEntity (i)->getMaterial ()->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(color_.r_, color_.g_, color_.b_));
    entity_->getSubEntity (i)->getMaterial ()->getTechnique(0)->getPass(0)->setAmbient(Ogre::Real(color_.r_), Ogre::Real(color_.g_), Ogre::Real(color_.b_));
    if (wf_)
      entity_->getSubEntity (i)->getMaterial ()->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    else
      entity_->getSubEntity (i)->getMaterial ()->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_SOLID);
  }
  
  scene_node_->attachObject(entity_);

  // finally, make sure we get everything in RVIZ's coordinate system
  tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0, 0, 0)), msg->header.stamp,
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
  
  Ogre::Quaternion orientation = Ogre::Quaternion (pose.getRotation().w(), pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z()); 

  rviz::robotToOgre(position);
  rviz::robotToOgre(orientation);

  scene_node_->setOrientation (orientation);
  scene_node_->setPosition (position);

}

void TriangleMeshDisplay::incomingMessage(const triangle_mesh::TriangleMesh::ConstPtr& message)
{
  current_message_ = message;
  processMessage(message);
}

void TriangleMeshDisplay::reset()
{
}

void TriangleMeshDisplay::targetFrameChanged()
{
}

void TriangleMeshDisplay::createProperties()
{
  wireframe_property_ = property_manager_->createProperty<rviz::BoolProperty> 
              ("Wireframe enable", property_prefix_, boost::bind(&TriangleMeshDisplay::getWireFrameEnabled, this),
              boost::bind(&TriangleMeshDisplay::setWireFrameEnabled, this, _1), parent_category_, this);
  color_property_ = property_manager_->createProperty<rviz::ColorProperty> 
              ("Color", property_prefix_, boost::bind(&TriangleMeshDisplay::getColor, this),
              boost::bind(&TriangleMeshDisplay::setColor, this, _1), parent_category_, this);
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> 
              ("Topic", property_prefix_, boost::bind(&TriangleMeshDisplay::getTopic, this),
              boost::bind(&TriangleMeshDisplay::setTopic, this, _1), parent_category_, this);
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::DataType<triangle_mesh::TriangleMesh>().value());
}


} // namespace triangle_mesh
