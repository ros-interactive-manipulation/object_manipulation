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

#ifndef MESH_OBJECT_SWITCHER_H_
#define MESH_OBJECT_SWITCHER_H_

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace rviz_interaction_tools
{

class MeshObject;

// allows to switch between different meshes and display modes
class MeshObjectSwitcher
{
public:

  /// the material names must exist and will be used for the different modes
  MeshObjectSwitcher( std::string valid_mat_name,
      std::string valid_sel_mat_name,
      std::string invalid_mat_name,
      std::string invalid_sel_mat_name );

  virtual ~MeshObjectSwitcher();

  // add another mesh object to the list (take ownership)
  void addObject( MeshObject* mesh_object );

  // change the currently visible mesh
  void setVisible( unsigned index );

  // change 'valid' flag
  void setValid( bool valid );

  // change 'selected' flag
  void setSelected( bool selected );

  /// increment the visible index by one
  void next( );

  /// @return currently visible mesh object
  unsigned getVisible( ) { return visible_mesh_; }

  unsigned isValid( ) { return valid_; }
  unsigned isSelected( ) { return selected_; }

  /// @return number of meshes being managed
  unsigned size( ) { return mesh_objects_.size(); }

private:

  void updateMaterials();

  MeshObjectSwitcher( const MeshObjectSwitcher& );
  MeshObjectSwitcher& operator=( const MeshObjectSwitcher& );

  std::vector< boost::shared_ptr<MeshObject> > mesh_objects_;

  unsigned visible_mesh_;

  bool valid_;
  bool selected_;

  std::string valid_mat_name_;
  std::string valid_sel_mat_name_;
  std::string invalid_mat_name_;
  std::string invalid_sel_mat_name_;
};

}

#endif /* MESH_OBJECT_SWITCHER_H_ */
