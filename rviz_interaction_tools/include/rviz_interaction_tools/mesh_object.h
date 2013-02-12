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

#ifndef MESH_OBJECT_CPP_
#define MESH_OBJECT_CPP_

#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>

#include <OGRE/OgrePrerequisites.h>
#include <OGRE/OgreEntity.h>

#include <string>

namespace rviz_interaction_tools
{

// holds a mesh constructed from a Shape message or a list of points
class MeshObject
{

public:

  struct Point {
    float x,y,z;
    float r,g,b,a;
  };

  MeshObject( Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_root );

  ~MeshObject();

  void setPose( const geometry_msgs::Pose& pose );

  void loadMesh( std::string name, const shape_msgs::Mesh& mesh );

  //load mesh from a list of points, assuming that 3 consecutive points form a triangle
  void loadPoints( std::string name, const std::vector< geometry_msgs::Point32 > &points );

  void loadMesh( std::string name, const std::vector< Point > &vertices,
      const std::vector<unsigned> triangles = std::vector<unsigned>() );

  void setVisible( bool visible );

  void setMaterialName( std::string name );

  Ogre::Entity* getEntity() { return entity_; }

  void clear();

private:

  MeshObject( const MeshObject& );
  MeshObject& operator=( const MeshObject& );

  Ogre::SceneNode *scene_node_;
  Ogre::MeshPtr mesh_ptr_;
  Ogre::Entity* entity_;

  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* scene_root_;
};

}

#endif /* MESH_OBJECT_CPP_ */

