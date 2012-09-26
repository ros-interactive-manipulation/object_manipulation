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

#include "rviz_interaction_tools/line.h"

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz_interaction_tools/unique_string_manager.h>

namespace rviz_interaction_tools {

Line::Line( Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node ) :
    scene_node_(scene_node),
    scene_manager_(scene_manager)
{
  rviz_interaction_tools::UniqueStringManager usm;

  line_object_ = scene_manager->createManualObject( usm.unique("GripperClickLineIndicator") );
  line_object_->setDynamic(true);
  line_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
  line_object_->position(0.0, 0.0, 0.0);
  line_object_->position(0.0, 0.0, 0.0);
  line_object_->index(0);
  line_object_->index(1);
  line_object_->index(0);
  line_object_->end();

  scene_node_->attachObject(line_object_);
}

Line::~Line()
{
  scene_node_->detachObject(line_object_);
  scene_manager_->destroyManualObject(line_object_);
}

void Line::setLimits( Ogre::Vector3 start, Ogre::Vector3 end )
{
  line_object_->beginUpdate(0);
  line_object_->position(start);
  line_object_->position(end);
  line_object_->index(0);
  line_object_->index(1);
  line_object_->index(0);
  line_object_->end();

  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  line_object_->setBoundingBox(aabInf);
}

}
