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

#ifndef CARTESIAN_CONTROL_H_
#define CARTESIAN_CONTROL_H_

#include "rviz_interaction_tools/gripper.h"

#include "rviz/ogre_helpers/shape.h"
#include "rviz_interaction_tools/mesh_object.h"

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRay.h>

#include <string>

namespace rviz {
class VisualizationManager;
}

namespace Ogre {
class SceneNode;
}

namespace rviz_interaction_tools
{

// Displays a 6-dof control that can be dragged and rotated using the mouse
// - the position/orientation of the controls being displayed and the
//   position/orientation being controlled do not necessarily have to be identical
// - the orientation of the controls can be fixed
class CartesianControl
{
public:

  static const float NO_INTERSECTION = 99999;

  enum StatusT
  {
    HIDDEN,
    IDLE,
    ROTATING,
    DRAGGING
  };

  CartesianControl( Ogre::SceneNode *scene_node, rviz::VisualizationManager *vis_manager );

  virtual ~CartesianControl();

  //set the position and orientation of the frame being controlled
  void setPosition( Ogre::Vector3 position );
  void setOrientation( Ogre::Quaternion orientation );

  //get the position and orientation of the frame being controlled
  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

  //set the position and orientation of the controls
  void setControlsPosition( Ogre::Vector3 position );
  void setControlsOrientation( Ogre::Quaternion orientation );

  void setFixedControlsOrientation( bool use ) { fixed_controls_orientation_ = use; }
  bool getFixedControlsOrientation() { return fixed_controls_orientation_; }

  //get the position and orientation of the controls
  Ogre::Vector3 getControlsPosition();
  Ogre::Quaternion getControlsOrientation( );

  //get the pose of the controlled frame in ROS format
  geometry_msgs::PoseStamped getPose();

  void show();
  void hide();

  void update();

  void mouseMove( Ogre::Ray mouse_ray );
  void mouseDown( Ogre::Ray mouse_ray );
  void mouseUp( Ogre::Ray mouse_ray );

  //get the smallest distance between the ray's origin and any of the controls, -1 if there's no intersection
  float getClosestIntersection( Ogre::Ray mouse_ray );

protected:

  static const float HANDLE_RADIUS_OUTER = 0.13;
  static const float HANDLE_RADIUS_INNER = 0.10;
  static const float HANDLE_SIZE = 0.03;
  static const float HANDLE_ALPHA = 0.5;

  enum SideT
  {
    POS,
    NEG
  };

  void addControls( unsigned axis );

  rviz::Shape* addDragBox( Ogre::Vector3 axis );

  void addRing(
      Ogre::Vector3 axis1, Ogre::Vector3 axis2,
      Ogre::MaterialPtr material, unsigned render_queue_group );

  //we need to split the discs into segments because of the z ordering for alpha blending
  void addRingSegment(
      Ogre::Vector3 axis1, Ogre::Vector3 axis2,
      Ogre::MaterialPtr material, unsigned render_queue_group,
      float a_min, float a_max );

  bool intersectRing( Ogre::Ray mouse_ray, unsigned axis,
      Ogre::Vector3 &intersection_3d, Ogre::Vector2 &intersection_2d, float &ray_t,
      float inner_radius=HANDLE_RADIUS_INNER, float outer_radius=HANDLE_RADIUS_OUTER );

  bool intersectPlane( Ogre::Ray mouse_ray, unsigned axis,
      Ogre::Vector3 &intersection_3d, Ogre::Vector2 &intersection_2d, float &ray_t );

  bool getClosestRing( Ogre::Ray mouse_ray,
      Ogre::Vector3 &nearest_intersection_3d, Ogre::Vector2 &nearest_intersection_2d,
      float &nearest_t, unsigned &nearest_axis );

  bool intersectBox( Ogre::Ray mouse_ray, unsigned axis, SideT side,
      Ogre::Vector3 &intersection_3d, float &intersection_1d, float &ray_t );

  bool getClosestBox( Ogre::Ray mouse_ray,
      Ogre::Vector3 &nearest_intersection_3d, float &nearest_intersection_1d,
      float &nearest_t, unsigned &nearest_axis, SideT &nearest_side );

  Ogre::Vector3 getAxis( unsigned axis );

  bool getClosestPosition( Ogre::Ray mouse_ray, unsigned axis, float &pos );

  rviz::VisualizationManager *vis_manager_;

  bool fixed_controls_orientation_;

  Ogre::SceneNode *main_node_;
  Ogre::SceneNode *controls_node_;

  // the first key is the axis, the second is the side (1 or -1)
  std::map< unsigned, std::map<SideT, rviz::Shape*> > drag_boxes_;

  std::vector<rviz_interaction_tools::MeshObject*> ring_segments_;

  rviz::Shape* marker_;

  StatusT status_;

  double last_angle_;
  double last_drag_pos_;
  SideT last_drag_side_;
  int last_axis_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
};

}

#endif /* CARTESIAN_GRIPPER_CONTROL_H_ */
