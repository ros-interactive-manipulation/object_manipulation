/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include "rviz_interaction_tools/mesh_object.h"

#include <OGRE/OgreMesh.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMeshManager.h>

namespace rviz_interaction_tools
{

MeshObject::MeshObject( Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_root ) :
        entity_(0),
        scene_manager_(scene_manager),
        scene_root_(scene_root)
{
  scene_node_ = scene_root->createChildSceneNode();
}

MeshObject::~MeshObject()
{
  clear();
  scene_root_->removeChild(scene_node_);
}

void MeshObject::clear( )
{
  scene_node_->detachAllObjects();

  if ( entity_ )
  {
    scene_manager_->destroyEntity(entity_);
    entity_ = 0;
  }

  if ( !mesh_ptr_.isNull() )
  {
    Ogre::MeshManager::getSingleton().remove( mesh_ptr_->getHandle() );
    mesh_ptr_.setNull();
  }
}

void MeshObject::setVisible( bool visible )
{
  entity_->setVisible( visible );
  entity_->setQueryFlags( visible ? 1 : 0 );
}

void MeshObject::setMaterialName( std::string name )
{
  entity_->setMaterialName(name);
}

void MeshObject::setPose( const geometry_msgs::Pose& pose )
{
  scene_node_->setPosition( pose.position.x, pose.position.y, pose.position.z );
  scene_node_->setOrientation( pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z );
}

void MeshObject::loadMesh( std::string name, const shape_msgs::Mesh& mesh )
{
  assert(mesh.triangles.size() > 0);
  assert(mesh.vertices.size() > 0);

  //create ogre object
  Ogre::ManualObject *manual_object = new Ogre::ManualObject( name );

  manual_object->setUseIdentityProjection(false);
  manual_object->setUseIdentityView(false);
  manual_object->setDynamic(true);

  manual_object->estimateVertexCount( mesh.vertices.size() );
  manual_object->estimateIndexCount( mesh.triangles.size()*3 );

  manual_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST );

  //add vertices
  for ( size_t v=0; v<mesh.vertices.size(); ++v )
  {
    const geometry_msgs::Point& p = mesh.vertices[v];
    manual_object->position( p.x, p.y, p.z );
    manual_object->colour( 1.0, 1.0, 1.0, 1.0 );
  }

  //add triangles
  for ( size_t t=0; t<mesh.triangles.size(); t++ )
  {
    assert( (size_t)mesh.triangles[t].vertex_indices[0] < mesh.vertices.size() );
    assert( (size_t)mesh.triangles[t].vertex_indices[1] < mesh.vertices.size() );
    assert( (size_t)mesh.triangles[t].vertex_indices[2] < mesh.vertices.size() );

    manual_object->triangle( mesh.triangles[t].vertex_indices[2],
                             mesh.triangles[t].vertex_indices[1],
                             mesh.triangles[t].vertex_indices[0] );
  }

  manual_object->end();

  std::string mesh_name = name + "mesh";

  Ogre::MeshPtr mesh_ptr = manual_object->convertToMesh( mesh_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  mesh_ptr->buildEdgeList();

  entity_ = scene_manager_->createEntity( name, mesh_name );
  entity_->setRenderQueueGroup(Ogre::RENDER_QUEUE_1);

  scene_node_->attachObject( entity_ );

  delete manual_object;

  mesh_ptr_ = mesh_ptr;
}

void MeshObject::loadPoints( std::string name, const std::vector< geometry_msgs::Point32 > &points )
{
  std::vector< Point > points_converted;
  points_converted.reserve( points.size() );

  Point point;
  point.r = point.g = point.b = 0;

  for ( unsigned i=0; i<points.size(); i++ )
  {
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.a = 1.0;
    points_converted.push_back( point );
  }

  loadMesh( name, points_converted );
}

void MeshObject::loadMesh( std::string name, const std::vector< Point > &vertices,
    const std::vector<unsigned> triangles )
{
  assert(vertices.size() > 0);

  //create ogre object
  Ogre::ManualObject *manual_object = new Ogre::ManualObject( name );

  manual_object->setUseIdentityProjection(false);
  manual_object->setUseIdentityView(false);
  manual_object->setDynamic(true);

  manual_object->estimateVertexCount( vertices.size() );
  manual_object->estimateIndexCount( triangles.size() > 0 ? triangles.size() : vertices.size()*3 );

  manual_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST );

  //add vertices
  for ( size_t v=0; v<vertices.size(); ++v )
  {
    const Point& p = vertices[v];
    manual_object->position( p.x, p.y, p.z );
    manual_object->colour( p.r, p.g, p.b );
  }

  //add triangles
  if ( triangles.size() > 0 )
  {
    size_t num_t = triangles.size();
    for ( size_t t=0; t+2<num_t; t+=3 )
    {
      assert( (size_t)triangles[t] < vertices.size() );
      assert( (size_t)triangles[t+1] < vertices.size() );
      assert( (size_t)triangles[t+2] < vertices.size() );

      manual_object->triangle( triangles[t], triangles[t+1], triangles[t+2] );
    }
  }
  else
  {
    for ( size_t t=0; t<vertices.size(); t+=3 )
    {
      manual_object->triangle( t, (t+1)%vertices.size(), (t+2)%vertices.size() );
    }
  }

  manual_object->end();

  std::string mesh_name = name + "mesh";

  Ogre::MeshPtr mesh_ptr = manual_object->convertToMesh( mesh_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  mesh_ptr->buildEdgeList();

  entity_ = scene_manager_->createEntity( name, mesh_name );
  entity_->setRenderQueueGroup(Ogre::RENDER_QUEUE_1);
  entity_->setCastShadows(false);

  scene_node_->attachObject( entity_ );

  delete manual_object;

  mesh_ptr_ = mesh_ptr;

#if 0
  Ogre::Vector3 min = entity_->getBoundingBox().getMinimum();
  Ogre::Vector3 max = entity_->getBoundingBox().getMaximum();
  ROS_INFO("Bbox min: %f %f %f max: %f %f %f",
     min.x, min.y, min.z, max.x, max.y, max.z );
#endif
}

}
