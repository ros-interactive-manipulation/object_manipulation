/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 */

#include "rviz_interaction_tools/point_cloud.h"
#include <ros/assert.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreBillboardSet.h>
#include <OGRE/OgreBillboard.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTextureManager.h>

#include <sstream>

#define VERTEX_BUFFER_CAPACITY (36 * 1024 * 10)

namespace rviz_interaction_tools
{

static float g_point_vertices[3] =
{
  0.0f, 0.0f, 0.0f
};

Ogre::String PointCloud::sm_Type = "RvizInteractionToolsPointCloud";

PointCloud::PointCloud()
: bounding_radius_( 0.0f )
, point_count_( 0 )
, current_mode_supports_geometry_shader_(false)
{
  setMaterial( "BaseWhiteNoLighting" );
  clear();
}

PointCloud::~PointCloud()
{
}

const Ogre::AxisAlignedBox& PointCloud::getBoundingBox() const
{
  return bounding_box_;
}

float PointCloud::getBoundingRadius() const
{
  return bounding_radius_;
}

void PointCloud::getWorldTransforms(Ogre::Matrix4* xform) const
{
  *xform = _getParentNodeFullTransform();
}

void PointCloud::clear()
{
  point_count_ = 0;
  bounding_box_.setNull();
  bounding_radius_ = 0.0f;

  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->getRenderOperation()->vertexData->vertexStart = 0;
    (*it)->getRenderOperation()->vertexData->vertexCount = 0;
  }

  if (getParentSceneNode())
  {
    getParentSceneNode()->needUpdate();
  }
}

void PointCloud::regenerateAll()
{
  if (point_count_ == 0)
  {
    return;
  }

  V_Point points;
  points.swap(points_);
  uint32_t count = point_count_;

  clear();

  addPoints(&points.front(), count);
}

void PointCloud::setMaterial( const Ogre::String& material_name )
{
  current_material_ = Ogre::MaterialManager::getSingleton().getByName( material_name );

  bool geom_support_changed = false;
  Ogre::Technique* best = current_material_->getBestTechnique();
  if (best)
  {
    if (current_material_->getBestTechnique()->getName() == "gp")
    {
      if (!current_mode_supports_geometry_shader_)
      {
        geom_support_changed = true;
      }

      current_mode_supports_geometry_shader_ = true;

      //ROS_INFO("Using geometry shader");
    }
    else
    {
      if (current_mode_supports_geometry_shader_)
      {
        geom_support_changed = true;
      }

      current_mode_supports_geometry_shader_ = false;
    }
  }
  else
  {
    geom_support_changed = true;
    current_mode_supports_geometry_shader_ = false;

    ROS_ERROR("No techniques available for material [%s]", current_material_->getName().c_str());
  }

  if (geom_support_changed)
  {
    renderables_.clear();
  }

  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setMaterial(current_material_->getName());
  }

  regenerateAll();
}

void PointCloud::setRenderQueueGroup( Ogre::uint8 queueID )
{
  Ogre::MovableObject::setRenderQueueGroup( queueID );
  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setRenderQueueGroup( queueID );
    ROS_INFO( "Setting render queue group %d", queueID );
  }
}

void PointCloud::addPoints(Point* points, uint32_t num_points)
{
  if (num_points == 0)
  {
    return;
  }

  if ( points_.size() < point_count_ + num_points )
  {
    points_.resize( point_count_ + num_points );
  }

  Point* begin = &points_.front() + point_count_;
  memcpy( begin, points, sizeof( Point ) * num_points );

  uint32_t vpp = getVerticesPerPoint();
  Ogre::RenderOperation::OperationType op_type = Ogre::RenderOperation::OT_POINT_LIST;

  float* vertices = 0;
  if (current_mode_supports_geometry_shader_)
  {
    vertices = g_point_vertices;
  }
  else
  {
    vertices = g_point_vertices;
  }

  PointCloudRenderablePtr rend;
  Ogre::HardwareVertexBufferSharedPtr vbuf;
  void* vdata = 0;
  Ogre::RenderOperation* op = 0;
  float* fptr = 0;

  Ogre::AxisAlignedBox aabb;
  aabb.setNull();
  uint32_t current_vertex_count = 0;
  bounding_radius_ = 0.0f;
  uint32_t vertex_size = 0;
  for (uint32_t current_point = 0; current_point < num_points; ++current_point)
  {
    while (current_vertex_count >= VERTEX_BUFFER_CAPACITY || !rend)
    {
      if (rend)
      {
        ROS_ASSERT(current_vertex_count == VERTEX_BUFFER_CAPACITY);

        op->vertexData->vertexCount = VERTEX_BUFFER_CAPACITY - op->vertexData->vertexStart;
        ROS_ASSERT(op->vertexData->vertexCount + op->vertexData->vertexStart <= VERTEX_BUFFER_CAPACITY);
        vbuf->unlock();
        rend->setBoundingBox(aabb);
      }

      rend = getOrCreateRenderable();
      vbuf = rend->getBuffer();
      vdata = vbuf->lock(Ogre::HardwareBuffer::HBL_NO_OVERWRITE);

      op = rend->getRenderOperation();
      op->operationType = op_type;
      current_vertex_count = op->vertexData->vertexStart + op->vertexData->vertexCount;

      vertex_size = op->vertexData->vertexDeclaration->getVertexSize(0);
      uint32_t data_pos = current_vertex_count * vertex_size;
      fptr = (float*)((uint8_t*)vdata + data_pos);

      aabb.setNull();
    }

    const Point& p = points[current_point];
    float x = p.x;
    float y = p.y;
    float z = p.z;
    uint32_t color = p.color;

    Ogre::Vector3 pos(x, y, z);
    aabb.merge(pos);
    bounding_box_.merge( pos );
    bounding_radius_ = std::max( bounding_radius_, pos.squaredLength() );

    for (uint32_t j = 0; j < vpp; ++j, ++current_vertex_count)
    {
      *fptr++ = x;
      *fptr++ = y;
      *fptr++ = z;

      if (!current_mode_supports_geometry_shader_)
      {
        *fptr++ = vertices[(j*3)];
        *fptr++ = vertices[(j*3) + 1];
        *fptr++ = vertices[(j*3) + 2];
      }

      uint32_t* iptr = (uint32_t*)fptr;
      *iptr = color;
      ++fptr;

      ROS_ASSERT((uint8_t*)fptr <= (uint8_t*)vdata + VERTEX_BUFFER_CAPACITY * vertex_size);
    }
  }

  op->vertexData->vertexCount = current_vertex_count - op->vertexData->vertexStart;
  rend->setBoundingBox(aabb);
  ROS_ASSERT(op->vertexData->vertexCount + op->vertexData->vertexStart <= VERTEX_BUFFER_CAPACITY);

  vbuf->unlock();

  point_count_ += num_points;

  shrinkRenderables();

  if (getParentSceneNode())
  {
    getParentSceneNode()->needUpdate();
  }
}

void PointCloud::shrinkRenderables()
{
  while (!renderables_.empty())
  {
    PointCloudRenderablePtr rend = renderables_.back();
    Ogre::RenderOperation* op = rend->getRenderOperation();
    if (op->vertexData->vertexCount == 0)
    {
      renderables_.pop_back();
    }
    else
    {
      break;
    }
  }
}

void PointCloud::_notifyCurrentCamera(Ogre::Camera* camera)
{
  MovableObject::_notifyCurrentCamera( camera );
}

void PointCloud::_updateRenderQueue(Ogre::RenderQueue* queue)
{
  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    queue->addRenderable((*it).get());
  }
}

void PointCloud::_notifyAttached(Ogre::Node *parent, bool isTagPoint)
{
  MovableObject::_notifyAttached(parent, isTagPoint);
}

uint32_t PointCloud::getVerticesPerPoint()
{
  return 1;
}

PointCloudRenderablePtr PointCloud::getOrCreateRenderable()
{
  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    const PointCloudRenderablePtr& rend = *it;
    Ogre::RenderOperation* op = rend->getRenderOperation();
    if (op->vertexData->vertexCount + op->vertexData->vertexStart < VERTEX_BUFFER_CAPACITY)
    {
      return rend;
    }
  }

  PointCloudRenderablePtr rend(new PointCloudRenderable(this, !current_mode_supports_geometry_shader_));
  rend->setMaterial(current_material_->getName());
  rend->setRenderQueueGroup( getRenderQueueGroup() );
  ROS_INFO( "Setting render queue group %d", getRenderQueueGroup() );

  if (getParentSceneNode())
  {
    getParentSceneNode()->attachObject(rend.get());
  }
  renderables_.push_back(rend);

  return rend;
}

#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6)
void PointCloud::visitRenderables(Ogre::Renderable::Visitor* visitor, bool debugRenderables)
{
  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    visitor->visit( it->get(), 0, debugRenderables, 0 );
  }
}
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudRenderable::PointCloudRenderable(PointCloud* parent, bool use_tex_coords)
: parent_(parent)
{
  // Initialize render operation
  mRenderOp.operationType = Ogre::RenderOperation::OT_POINT_LIST;
  mRenderOp.useIndexes = false;
  mRenderOp.vertexData = new Ogre::VertexData;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = 0;

  Ogre::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration;
  size_t offset = 0;

  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  if (use_tex_coords)
  {
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES, 0);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  }

  decl->addElement(0, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);

  Ogre::HardwareVertexBufferSharedPtr vbuf =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
      mRenderOp.vertexData->vertexDeclaration->getVertexSize(0),
      VERTEX_BUFFER_CAPACITY,
      Ogre::HardwareBuffer::HBU_DYNAMIC);

  // Bind buffer
  mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vbuf);
}

PointCloudRenderable::~PointCloudRenderable()
{
  delete mRenderOp.vertexData;
  delete mRenderOp.indexData;
}

Ogre::HardwareVertexBufferSharedPtr PointCloudRenderable::getBuffer()
{
  return mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);
}

void PointCloudRenderable::_notifyCurrentCamera(Ogre::Camera* camera)
{
  SimpleRenderable::_notifyCurrentCamera( camera );
}

Ogre::Real PointCloudRenderable::getBoundingRadius(void) const
{
  return Ogre::Math::Sqrt(std::max(mBox.getMaximum().squaredLength(), mBox.getMinimum().squaredLength()));
}

Ogre::Real PointCloudRenderable::getSquaredViewDepth(const Ogre::Camera* cam) const
{
  Ogre::Vector3 vMin, vMax, vMid, vDist;
   vMin = mBox.getMinimum();
   vMax = mBox.getMaximum();
   vMid = ((vMax - vMin) * 0.5) + vMin;
   vDist = cam->getDerivedPosition() - vMid;

   return vDist.squaredLength();
}

void PointCloudRenderable::getWorldTransforms(Ogre::Matrix4* xform) const
{
   *xform = m_matWorldTransform * parent_->getParentNode()->_getFullTransform();
}

const Ogre::LightList& PointCloudRenderable::getLights() const
{
  return parent_->queryLights();
}

void PointCloudRenderable::_updateRenderQueue(Ogre::RenderQueue* queue)
{
    queue->addRenderable( this, mRenderQueueID, OGRE_RENDERABLE_DEFAULT_PRIORITY);
}

} // namespace rviz_interaction_tools
