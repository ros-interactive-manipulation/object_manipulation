/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "rviz_interaction_tools/gripper.h"


#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>

#include <rviz/render_panel.h>
#include <rviz/mesh_loader.h>

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreLight.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreTechnique.h>

#include <rviz_interaction_tools/unique_string_manager.h>

namespace rviz_interaction_tools
{

Gripper::Gripper( Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_root ) :
  gripper_transform_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.5))
{
  scene_manager_ = scene_manager;

  std::string palm_string("package://pr2_description/meshes/gripper_v0/gripper_palm.dae");
  std::string proximal_finger_string("package://pr2_description/meshes/gripper_v0/l_finger.dae");
  std::string distal_finger_string("package://pr2_description/meshes/gripper_v0/l_finger_tip.dae");

  Ogre::MeshPtr palm_mesh = rviz::loadMeshFromResource(palm_string);
  Ogre::MeshPtr proximal_finger_mesh = rviz::loadMeshFromResource(proximal_finger_string);
  Ogre::MeshPtr distal_finger_mesh = rviz::loadMeshFromResource(distal_finger_string);

  if (!palm_mesh.get() || !proximal_finger_mesh.get() || !distal_finger_mesh.get()) 
  {
    ROS_ERROR("Failed to load resource");
  }

  rviz_interaction_tools::UniqueStringManager usm;

  resource_group_name_ = usm.unique("rviz_interaction_tools::Gripper");
  Ogre::ResourceGroupManager::getSingleton().createResourceGroup(resource_group_name_);
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(resource_group_name_);

  Ogre::Entity* palm_entity = scene_manager->createEntity(usm.unique("palm_entity"), palm_string, resource_group_name_);
  Ogre::Entity* l_proximal_finger_entity = scene_manager->createEntity(usm.unique("l_proximal_finger_entity"),
                                                                        proximal_finger_string, resource_group_name_);
  Ogre::Entity* l_distal_finger_entity = scene_manager->createEntity(usm.unique("l_distal_finger_entity"),
                                                                      distal_finger_string, resource_group_name_);
  Ogre::Entity* r_proximal_finger_entity = scene_manager->createEntity(usm.unique("r_proximal_finger_entity"),
                                                                        proximal_finger_string);
  Ogre::Entity* r_distal_finger_entity = scene_manager->createEntity(usm.unique("r_distal_finger_entity"),
                                                                      distal_finger_string, resource_group_name_);

  entities_.push_back( palm_entity );
  entities_.push_back( l_proximal_finger_entity );
  entities_.push_back( l_distal_finger_entity );
  entities_.push_back( r_proximal_finger_entity );
  entities_.push_back( r_distal_finger_entity );

  material_.push_back( palm_entity->getSubEntity(0)->getMaterial()->clone( usm.unique("gripper_mat1"), true, resource_group_name_ ) );
  palm_entity->setMaterial(material_.back());

  material_.push_back( l_proximal_finger_entity->getSubEntity(0)->getMaterial()->clone( usm.unique("gripper_mat2"), true, resource_group_name_ ) );
  l_proximal_finger_entity->setMaterial(material_.back());
  r_proximal_finger_entity->setMaterial(material_.back());

  material_.push_back( l_distal_finger_entity->getSubEntity(0)->getMaterial()->clone( usm.unique("gripper_mat3"), true, resource_group_name_ ) );
  l_distal_finger_entity->setMaterial(material_.back());
  r_distal_finger_entity->setMaterial(material_.back());

  setColour(1,1,0,0.5);

  gripper_root_ = scene_root->createChildSceneNode();

  gripper_root_->setPosition(-0.1, 0.0, 0.5);
  gripper_root_->attachObject(palm_entity);

  l_proximal_finger_node_ = gripper_root_->createChildSceneNode();
  l_proximal_finger_node_->setPosition(0.07691, 0.01, 0);
  l_proximal_finger_node_->attachObject(l_proximal_finger_entity);

  l_distal_finger_node_ = l_proximal_finger_node_->createChildSceneNode();
  l_distal_finger_node_->setPosition(0.09137, 0.00495, 0);
  l_distal_finger_node_->attachObject(l_distal_finger_entity);

  Ogre::SceneNode* r_flip_node = gripper_root_->createChildSceneNode();
  r_flip_node->setOrientation(0, 1, 0, 0);

  r_proximal_finger_node_ = r_flip_node->createChildSceneNode();
  r_proximal_finger_node_->setPosition(0.07691, 0.01, 0);
  r_proximal_finger_node_->attachObject(r_proximal_finger_entity);

  r_distal_finger_node_ = r_proximal_finger_node_->createChildSceneNode();
  r_distal_finger_node_->setPosition(0.09137, 0.00495, 0);
  r_distal_finger_node_->attachObject(r_distal_finger_entity);
}

Gripper::~Gripper()
{
  scene_manager_->destroySceneNode(l_proximal_finger_node_);
  scene_manager_->destroySceneNode(l_distal_finger_node_);
  scene_manager_->destroySceneNode(r_proximal_finger_node_);
  scene_manager_->destroySceneNode(r_distal_finger_node_);
  scene_manager_->destroySceneNode(gripper_root_);
  Ogre::ResourceGroupManager::getSingleton().destroyResourceGroup(resource_group_name_);
}

void Gripper::setColour( float r, float g, float b, float a )
{
  for ( unsigned i=0; i<material_.size(); i++ )
  {
    Ogre::MaterialPtr material = material_[i];

    material->getBestTechnique()->getPass(0)->setDiffuse(r,g,b,a);
    material->getBestTechnique()->getPass(0)->setAmbient(r,g,b);

    if ( a < 0.9998 )
    {
      material->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
      material->getTechnique(0)->setDepthWriteEnabled( false );
    }
    else
    {
      material->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
      material->getTechnique(0)->setDepthWriteEnabled( true );
    }
  }
}

void Gripper::setDepthCheckEnabled( bool enabled )
{
  for ( unsigned i=0; i<material_.size(); i++ )
  {
    material_[i]->setDepthCheckEnabled( enabled );
  }
}

void Gripper::setRenderQueueGroup( unsigned group )
{
  for ( unsigned i=0; i<entities_.size(); i++ )
  {
    entities_[i]->setRenderQueueGroup(group);
  }
}

void Gripper::setVisible( bool visible )
{
  gripper_root_->setVisible(visible);
}

void Gripper::setGripperAngle(float angle)
{
  gripper_angle_ = angle;

  l_proximal_finger_node_->resetOrientation();
  l_proximal_finger_node_->roll(Ogre::Radian(angle));
  l_distal_finger_node_->resetOrientation();
  l_distal_finger_node_->roll(Ogre::Radian(-angle));

  r_proximal_finger_node_->resetOrientation();
  r_proximal_finger_node_->roll(Ogre::Radian(angle));
  r_distal_finger_node_->resetOrientation();
  r_distal_finger_node_->roll(Ogre::Radian(-angle));
}

void Gripper::setPosition( Ogre::Vector3 &position )
{
  gripper_root_->setPosition(position);
}

void Gripper::setOrientation( Ogre::Quaternion &orientation )
{
  gripper_root_->setOrientation( orientation );
}

}
