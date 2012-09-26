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

#include "rviz_interaction_tools/mesh_object_switcher.h"
#include "rviz_interaction_tools/mesh_object.h"

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgrePass.h>
#include <OGRE/OgreEdgeListBuilder.h>


namespace rviz_interaction_tools
{

MeshObjectSwitcher::MeshObjectSwitcher( std::string valid_mat_name,
    std::string valid_sel_mat_name,
    std::string invalid_mat_name,
    std::string invalid_sel_mat_name ) :
    visible_mesh_(-1),
    valid_(true),
    selected_(false),
    valid_mat_name_(valid_mat_name),
    valid_sel_mat_name_(valid_sel_mat_name),
    invalid_mat_name_(invalid_mat_name),
    invalid_sel_mat_name_(invalid_sel_mat_name)
{
}


MeshObjectSwitcher::~MeshObjectSwitcher()
{
}


void MeshObjectSwitcher::addObject( MeshObject* mesh_object )
{
  mesh_object->setVisible(false);
  mesh_objects_.push_back( boost::shared_ptr<MeshObject>( mesh_object ) );
  updateMaterials();
}


void MeshObjectSwitcher::setSelected( bool selected )
{
  selected_ = selected;
  updateMaterials();
}

void MeshObjectSwitcher::setValid( bool valid )
{
  valid_ = valid;
  updateMaterials();
}


void MeshObjectSwitcher::setVisible( unsigned index )
{
  if ( visible_mesh_ < mesh_objects_.size() )
  {
    ROS_INFO_STREAM( "Hiding mesh " << visible_mesh_ );
    mesh_objects_[visible_mesh_]->setVisible(false);
  }

  if ( index < mesh_objects_.size() )
  {
    ROS_INFO_STREAM( "Showing mesh " << index << "."
        << " Entity: " << mesh_objects_[index]->getEntity()->getName()
        << " Mesh: " << mesh_objects_[index]->getEntity()->getMesh()->getName()
        << " Triangles: " << mesh_objects_[index]->getEntity()->getMesh()->getEdgeList()->triangles.size()
        );
    mesh_objects_[index]->setVisible(true);
  }

  visible_mesh_ = index;
}

void MeshObjectSwitcher::next()
{
  setVisible( (visible_mesh_ + 1) % mesh_objects_.size() );
}

void MeshObjectSwitcher::updateMaterials()
{
  for ( unsigned i=0; i<mesh_objects_.size(); ++i )
  {
    if ( valid_ && selected_ )
    {
      mesh_objects_[i]->setMaterialName( valid_sel_mat_name_ );
    }
    else if ( valid_ )
    {
      mesh_objects_[i]->setMaterialName( valid_mat_name_ );
    }
    else if ( selected_ )
    {
      mesh_objects_[i]->setMaterialName( invalid_sel_mat_name_ );
    }
    else
    {
      mesh_objects_[i]->setMaterialName( invalid_mat_name_ );
    }
  }
}

}
