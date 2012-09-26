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

#include "rviz_interaction_tools/cartesian_control.h"

#include <visualization_msgs/Marker.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/selection/selection_manager.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreResourceGroupManager.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreMath.h>

#include "rviz_interaction_tools/unique_string_manager.h"

#include <math.h>

namespace rviz_interaction_tools
{


CartesianControl::CartesianControl( Ogre::SceneNode *scene_node, rviz::VisualizationManager *vis_manager ) :
    vis_manager_( vis_manager ),
    fixed_controls_orientation_(false),
    status_(HIDDEN),
    position_(Ogre::Vector3::ZERO),
    orientation_(Ogre::Quaternion::IDENTITY)
{
  main_node_ = scene_node->createChildSceneNode();
  main_node_->setVisible(false);

  controls_node_ = main_node_->createChildSceneNode();

  addControls(0);
  addControls(1);
  addControls(2);

  marker_ = new rviz::Shape( rviz::Shape::Sphere, vis_manager->getSceneManager(), main_node_ );
  marker_->setScale( Ogre::Vector3( 0.01,0.01,0.01 ) );
}

CartesianControl::~CartesianControl()
{
  vis_manager_->getSceneManager()->destroySceneNode(main_node_);
  vis_manager_->getSceneManager()->destroySceneNode(controls_node_);

  for ( unsigned i=0; i<3; i++ )
  {
    delete drag_boxes_[i][POS];
    delete drag_boxes_[i][NEG];
  }
}

void CartesianControl::addControls( unsigned axis )
{
  std::vector<Ogre::Vector3> axes;
  axes.push_back(Ogre::Vector3::UNIT_X);
  axes.push_back(Ogre::Vector3::UNIT_Y);
  axes.push_back(Ogre::Vector3::UNIT_Z);

  drag_boxes_[axis][POS] = addDragBox( axes[axis] );
  drag_boxes_[axis][NEG] = addDragBox( -1 * axes[axis] );

  Ogre::MaterialPtr material = drag_boxes_[axis][POS]->getEntity()->getSubEntity(0)->getMaterial();
  material->getBestTechnique()->setCullingMode( Ogre::CULL_NONE );
  material->getBestTechnique()->setDepthWriteEnabled( true );
  material->getBestTechnique()->setDepthCheckEnabled( true );

  addRing( axes[(axis+1)%3], axes[(axis+2)%3],
      material, drag_boxes_[axis][POS]->getEntity()->getRenderQueueGroup() );
}

rviz::Shape* CartesianControl::addDragBox( Ogre::Vector3 axis )
{
  rviz::Shape *shape =
      new rviz::Shape( rviz::Shape::Cube, vis_manager_->getSceneManager(), controls_node_ );

  shape->setPosition( axis * (HANDLE_RADIUS_OUTER + HANDLE_SIZE) );
  shape->setScale( Ogre::Vector3(HANDLE_SIZE,HANDLE_SIZE,HANDLE_SIZE) );
  shape->setColor( fabs(axis.x), fabs(axis.y), fabs(axis.z), HANDLE_ALPHA );

  //put everything into a higher render queue to transparency will work
  shape->getEntity()->setRenderQueueGroup( Ogre::RENDER_QUEUE_6 );

  Ogre::MaterialPtr material = shape->getEntity()->getSubEntity(0)->getMaterial();
  material->getBestTechnique()->setCullingMode( Ogre::CULL_NONE );
  material->getBestTechnique()->setDepthWriteEnabled( true );
  material->getBestTechnique()->setDepthCheckEnabled( true );
//  material->getBestTechnique()->setLightingEnabled(false);

  return shape;
}

void CartesianControl::addRing(
    Ogre::Vector3 axis1, Ogre::Vector3 axis2,
    Ogre::MaterialPtr material, unsigned render_queue_group)
{
  static const float pi = 3.14159265;
  for ( float a=0; a<2.0*pi; a+=0.5*pi )
  {
    addRingSegment( axis1, axis2, material, render_queue_group, a, a+pi*0.5 );
  }
}

void CartesianControl::addRingSegment(
    Ogre::Vector3 axis1, Ogre::Vector3 axis2,
    Ogre::MaterialPtr material, unsigned render_queue_group,
    float a_min, float a_max )
{
  rviz_interaction_tools::MeshObject *mesh_object =
      new rviz_interaction_tools::MeshObject( vis_manager_->getSceneManager(), controls_node_ );

  Ogre::Vector3 axis0 = Ogre::Vector3(1,1,1) - axis1 - axis2;

  std::vector< rviz_interaction_tools::MeshObject::Point > points;

  int steps = 20;
  for ( int i=0; i<=steps; i++ )
  {
    float a = a_min + (float(i)/float(steps) * (a_max-a_min) );
    Ogre::Vector3 p = (sin(a)*axis1 + cos(a)*axis2);
    Ogre::Vector3 p1 = p * (HANDLE_RADIUS_INNER);
    Ogre::Vector3 p2 = p * (HANDLE_RADIUS_OUTER);
    rviz_interaction_tools::MeshObject::Point p_inner = { p1.x, p1.y, p1.z, axis0.x, axis0.y, axis0.z, HANDLE_ALPHA };
    rviz_interaction_tools::MeshObject::Point p_outer = { p2.x, p2.y, p2.z, axis0.x, axis0.y, axis0.z, HANDLE_ALPHA };
    points.push_back(p_inner);
    points.push_back(p_outer);
  }

  std::vector< unsigned > triangles;

  for ( unsigned i=0; i<points.size()-2; i++ )
  {
    triangles.push_back(i);
    triangles.push_back(i+1);
    triangles.push_back(i+2);
  }

  rviz_interaction_tools::UniqueStringManager usm;
  mesh_object->loadMesh( usm.unique("CartesianControl_disc"), points, triangles );

  mesh_object->getEntity()->setMaterial( material );
  mesh_object->getEntity()->setRenderQueueGroup( render_queue_group );
}

void CartesianControl::show()
{
  status_ = IDLE;
  main_node_->setVisible(true);
}

void CartesianControl::hide()
{
  main_node_->setVisible(false);

  if ( status_ == HIDDEN )
  {
    return;
  }

  status_ = HIDDEN;
}

void CartesianControl::setPosition( Ogre::Vector3 position )
{
  position_ = position;
}

void CartesianControl::setOrientation( Ogre::Quaternion orientation )
{
  orientation_ = orientation;
}

Ogre::Vector3 CartesianControl::getPosition()
{
  return position_;
}

Ogre::Quaternion CartesianControl::getOrientation()
{
  return orientation_;
}

void CartesianControl::setControlsPosition( Ogre::Vector3 position )
{
  controls_node_->setPosition( position );
}

void CartesianControl::setControlsOrientation( Ogre::Quaternion orientation )
{
  controls_node_->setOrientation( orientation );
}

Ogre::Vector3 CartesianControl::getControlsPosition()
{
  return controls_node_->getPosition();
}

Ogre::Quaternion CartesianControl::getControlsOrientation()
{
  return controls_node_->getOrientation();
}

void CartesianControl::update()
{
}

float CartesianControl::getClosestIntersection( Ogre::Ray mouse_ray )
{
  Ogre::Vector3 disc_intersection_3d;
  Ogre::Vector2 disc_intersection_2d;
  float disc_ray_t;
  unsigned disc_axis;

  Ogre::Vector3 box_intersection_3d;
  float box_intersection_1d;
  float box_ray_t;
  unsigned box_axis;
  SideT box_side;

  if ( !getClosestRing( mouse_ray, disc_intersection_3d, disc_intersection_2d, disc_ray_t, disc_axis )
      && !getClosestBox( mouse_ray, box_intersection_3d, box_intersection_1d, box_ray_t, box_axis, box_side ) )
    return NO_INTERSECTION;

  //choose what's closer to camera, box or ring
  return disc_ray_t < box_ray_t ? disc_ray_t : box_ray_t;
}


void CartesianControl::mouseMove( Ogre::Ray mouse_ray )
{
  Ogre::Vector3 intersection_3d;
  Ogre::Vector2 intersection_2d;
  float intersection_1d;
  float ray_t;

  switch ( status_ )
  {
    case HIDDEN:
    case IDLE:
    {
      unsigned axis;
      getClosestRing( mouse_ray, intersection_3d, intersection_2d, ray_t, axis );
//      getClosestBox( mouse_ray, intersection_3d, intersection_1d, ray_t, axis, side );
      marker_->setPosition( intersection_3d );
      break;
    }

    case ROTATING:
    {
      if ( intersectRing( mouse_ray, last_axis_, intersection_3d, intersection_2d, ray_t,
          HANDLE_RADIUS_INNER*0.5, HANDLE_RADIUS_OUTER*100 ) )
      {
        double angle = atan2( intersection_2d.x, intersection_2d.y );
        Ogre::Radian delta_angle( (last_angle_-angle) );
        Ogre::Vector3 axis_vec = getControlsOrientation() * getAxis( last_axis_ );
        Ogre::Quaternion delta_orientation( delta_angle, axis_vec );

        Ogre::Vector3 offset = getPosition() - getControlsPosition();

        //rotate controlled frame around controls
        orientation_ = delta_orientation * orientation_;
        position_ = getControlsPosition() + delta_orientation * offset;

        if ( fixed_controls_orientation_ )
        {
          last_angle_ = angle;
        }
        else
        {
          controls_node_->setOrientation( delta_orientation * controls_node_->getOrientation() );
        }

        update();
        ROS_DEBUG( "delta angle for axis %d = %f (%f - %f)", last_axis_, angle-last_angle_, angle, last_angle_ );

        Ogre::Vector3 diff = intersection_3d - getControlsPosition();
        diff.normalise();
        diff *= (HANDLE_RADIUS_OUTER+HANDLE_RADIUS_INNER) / 2.0;
        marker_->setPosition( getControlsPosition() + diff );
      }
      else
      {
        //make sure the angle doesn't jump when we get back to the ring
        last_angle_ = atan2( intersection_2d.x, intersection_2d.y );
      }
      break;
    }

    case DRAGGING:
    {
      getClosestPosition( mouse_ray, last_axis_, intersection_1d );
      {
        float delta = intersection_1d - last_drag_pos_;
        Ogre::Vector3 translate_delta = getControlsOrientation() * getAxis(last_axis_) * delta;
        position_ = position_ + translate_delta;
        controls_node_->setPosition( controls_node_->getPosition() + translate_delta );

        update();

        marker_->setPosition( getControlsPosition() + getControlsOrientation() * getAxis(last_axis_) * intersection_1d );
      }
      break;
    }
  }
}

void CartesianControl::mouseDown( Ogre::Ray mouse_ray )
{
  Ogre::Vector3 disc_intersection_3d;
  Ogre::Vector2 disc_intersection_2d;
  float disc_ray_t;
  unsigned disc_axis;

  Ogre::Vector3 box_intersection_3d;
  float box_intersection_1d;
  float box_ray_t;
  unsigned box_axis;
  SideT box_side;

  getClosestRing( mouse_ray, disc_intersection_3d, disc_intersection_2d, disc_ray_t, disc_axis );
  getClosestBox( mouse_ray, box_intersection_3d, box_intersection_1d, box_ray_t, box_axis, box_side );

  if ( disc_ray_t == NO_INTERSECTION && box_ray_t == NO_INTERSECTION )
    return;

  //choose what's closer to camera, box or ring
  if ( disc_ray_t < box_ray_t )
  {
    //choose disc
    if ( fabs( (getControlsOrientation()*getAxis(disc_axis)).dotProduct( mouse_ray.getDirection() )) < 0.01 )
    {
      //if the ray points at the plane at a steep angle, it's too dangerous
      ROS_DEBUG("the ray points at the plane at a steep angle, that's too dangerous");
      return;
    }

    ROS_DEBUG("Rotating around axis %d", disc_axis);
    marker_->setPosition( disc_intersection_3d );
    status_ = ROTATING;
    last_angle_ = atan2( disc_intersection_2d.x, disc_intersection_2d.y );
    last_axis_ = disc_axis;
  }
  else
  {
    //choose box
    if ( fabs( (getControlsOrientation()*getAxis(box_axis)).dotProduct( mouse_ray.getDirection() )) > 0.99 )
    {
      ROS_DEBUG("the ray points at the line at a steep angle, that's too dangerous");
      return;
    }

    marker_->setPosition( box_intersection_3d );
    status_ = DRAGGING;
    last_drag_pos_ = box_intersection_1d;
    last_drag_side_ = box_side;
    last_axis_ = box_axis;
    ROS_DEBUG("Dragging axis %d t=%f", box_axis, last_drag_pos_);
  }

}

void CartesianControl::mouseUp( Ogre::Ray mouse_ray )
{
  switch ( status_ )
  {
    case IDLE:
    case HIDDEN:

      break;

    case ROTATING:
    case DRAGGING:

      status_ = IDLE;
      break;
  }
}


bool CartesianControl::getClosestRing( Ogre::Ray mouse_ray,
    Ogre::Vector3 &nearest_intersection_3d, Ogre::Vector2 &nearest_intersection_2d,
    float &nearest_t, unsigned &nearest_axis )
{
  nearest_t = NO_INTERSECTION;
  bool found = false;

  for ( int axis = 0; axis < 3; axis++ )
  {
    Ogre::Vector3 intersection_3d;
    Ogre::Vector2 intersection_2d;
    float ray_t;
    if ( intersectRing( mouse_ray, axis, intersection_3d, intersection_2d, ray_t ) && ray_t < nearest_t )
    {
      nearest_axis = axis;
      nearest_t = ray_t;
      nearest_intersection_3d = intersection_3d;
      nearest_intersection_2d = intersection_2d;
      found = true;
    }
  }

//  marker_->setPosition( nearest_intersection_3d );
  return found;
}


bool CartesianControl::intersectRing( Ogre::Ray mouse_ray, unsigned axis,
    Ogre::Vector3 &intersection_3d, Ogre::Vector2 &intersection_2d, float &ray_t,
    float inner_radius, float outer_radius )
{
  return ( intersectPlane( mouse_ray, axis, intersection_3d, intersection_2d, ray_t )
      && intersection_2d.length() > inner_radius
      && intersection_2d.length() < outer_radius );
}


bool CartesianControl::intersectPlane( Ogre::Ray mouse_ray, unsigned axis,
    Ogre::Vector3 &intersection_3d, Ogre::Vector2 &intersection_2d, float &ray_t )
{
  Ogre::Vector3 position = getControlsPosition();
  Ogre::Quaternion orientation = getControlsOrientation();

  Ogre::Vector3 normal = orientation * getAxis(axis);
  Ogre::Vector3 x_axis = orientation * getAxis(axis+1);
  Ogre::Vector3 y_axis = orientation * getAxis(axis+2);

  Ogre::Plane plane( normal, position );


  Ogre::Vector2 origin_2d( position.dotProduct( x_axis ), position.dotProduct( y_axis ) );

  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects( plane );
  if ( intersection.first )
  {
    intersection_3d = mouse_ray.getPoint( intersection.second );
    intersection_2d = Ogre::Vector2( intersection_3d.dotProduct(x_axis), intersection_3d.dotProduct(y_axis) );
    intersection_2d -= origin_2d;

    ray_t = intersection.second;
    return true;
  }

  ray_t = 0;
  return false;
}


bool CartesianControl::getClosestBox( Ogre::Ray mouse_ray,
    Ogre::Vector3 &nearest_intersection_3d, float &nearest_intersection_1d,
    float &nearest_t, unsigned &nearest_axis, SideT &nearest_side )
{
  nearest_t = NO_INTERSECTION;
  bool found = false;

  for ( int axis = 0; axis < 3; axis++ )
  {
    SideT sides[] = {POS,NEG};
    for ( unsigned s = 0; s < 2; s++ )
    {
      SideT side = sides[s];
      Ogre::Vector3 intersection_3d;
      float ray_t;
      float intersection_1d;
      if ( intersectBox( mouse_ray, axis, side, intersection_3d, intersection_1d, ray_t ) && ray_t < nearest_t )
      {
        nearest_intersection_1d = intersection_1d;
        nearest_intersection_3d = intersection_3d;
        nearest_t = ray_t;
        nearest_side = side;
        nearest_axis = axis;
        found = true;
      }
    }
  }

  return found;
}


bool CartesianControl::intersectBox( Ogre::Ray mouse_ray, unsigned axis, SideT side,
    Ogre::Vector3 &intersection_3d, float &intersection_1d, float &ray_t )
{
  rviz::Shape* shape = drag_boxes_[axis][side];

  Ogre::Vector3 position = shape->getRootNode()->convertLocalToWorldPosition(Ogre::Vector3(0,0,0));
  Ogre::Quaternion orientation = shape->getRootNode()->convertLocalToWorldOrientation(Ogre::Quaternion());

  ROS_DEBUG_STREAM( "intersectBox axis " << axis << " side " << side << " position " << position << " orientation " << orientation );

  Ogre::PlaneBoundedVolume volume;

  float box_size = HANDLE_SIZE*0.5;

  for ( unsigned i = 0; i<3; i++ )
  {
    Ogre::Vector3 axis_vec = orientation * getAxis(i);
    volume.planes.push_back( Ogre::Plane( -1 * axis_vec, position + (axis_vec*box_size) ) );
    volume.planes.push_back( Ogre::Plane( axis_vec, position - (axis_vec*box_size) ) );
  }

  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects( volume );
  if ( intersection.first )
  {
    intersection_3d = mouse_ray.getPoint( intersection.second );
    ray_t = intersection.second;
    getClosestPosition( mouse_ray, axis, intersection_1d );
    return true;
  }

  ray_t = 0;
  return false;
}



Ogre::Vector3 CartesianControl::getAxis( unsigned axis )
{
  axis = axis % 3;
  switch ( axis )
  {
    case 0:
      return Ogre::Vector3::UNIT_X;
    case 1:
      return Ogre::Vector3::UNIT_Y;
    case 2:
      return Ogre::Vector3::UNIT_Z;
    default:
      break;
  }
  return Ogre::Vector3();
}


bool CartesianControl::getClosestPosition( Ogre::Ray mouse_ray, unsigned axis, float &pos )
{
  Ogre::Vector3 axis_vec = getControlsOrientation()*getAxis(axis);

  //axis2 is perpendicular to mouse ray and axis ray
  Ogre::Vector3 axis2 = mouse_ray.getDirection().crossProduct( axis_vec );

  //axis3 is perpendicular to axis and axis2, thus the normal of the plane
  //that contains the shortest connection line
  Ogre::Vector3 normal = axis2.crossProduct( mouse_ray.getDirection() );

  Ogre::Plane mouse_plane( normal, mouse_ray.getOrigin() );
  Ogre::Ray axis_ray( getControlsPosition(), axis_vec );

  std::pair<bool,float> result = axis_ray.intersects( mouse_plane );

  pos = result.second;
  return result.first;
}


geometry_msgs::PoseStamped CartesianControl::getPose()
{
  geometry_msgs::PoseStamped desired_pose;

  desired_pose.header.frame_id = vis_manager_->getFrameManager()->getFixedFrame();
  desired_pose.header.stamp = ros::Time::now();

  Ogre::Vector3 position = getPosition();
  Ogre::Quaternion orientation = getOrientation();

  desired_pose.pose.position.x = position.x;
  desired_pose.pose.position.y = position.y;
  desired_pose.pose.position.z = position.z;

  desired_pose.pose.orientation.w = orientation.w;
  desired_pose.pose.orientation.x = orientation.x;
  desired_pose.pose.orientation.y = orientation.y;
  desired_pose.pose.orientation.z = orientation.z;

  return desired_pose;
}

}
