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

#ifndef GRIPPER_CLICK_GRIPPER_H
#define GRIPPER_CLICK_GRIPPER_H

#include <tf/transform_datatypes.h>

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMaterial.h>

namespace Ogre {
class SceneManager;
class SceneNode;
}

namespace rviz_interaction_tools {

class Gripper
{

public:

  Gripper( Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_root );
  ~Gripper();

  void setGripperAngle(float angle);

  void setPosition( Ogre::Vector3 &position );

  void setOrientation( Ogre::Quaternion &orientation );

  void setVisible( bool visible );

  Ogre::SceneNode* getRootNode() { return gripper_root_; }

  void setColour( float r, float g, float b, float a );

  void setDepthCheckEnabled( bool );

  void setRenderQueueGroup( unsigned group );

protected:

  Ogre::SceneManager* scene_manager_;

  tf::Transform computeOrientation();

  float gripper_angle_;
  float gripper_roll_;
  float gripper_approach_;

  tf::Transform gripper_transform_;

  Ogre::SceneNode* gripper_root_;
  Ogre::SceneNode* l_proximal_finger_node_;
  Ogre::SceneNode* l_distal_finger_node_;
  Ogre::SceneNode* r_proximal_finger_node_;
  Ogre::SceneNode* r_distal_finger_node_;

  std::string resource_group_name_;

  std::vector<Ogre::MaterialPtr> material_;
  std::vector<Ogre::Entity*> entities_;
};

}

#endif
