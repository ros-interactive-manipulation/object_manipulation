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


#include "rviz_interaction_tools/camera_tools.h"

namespace rviz_interaction_tools {

void updateCamera( Ogre::Camera* camera, const sensor_msgs::CameraInfo &camera_info )
{
  //align camera with robot camera coordinate frame (z forward and x right)
  camera->setPosition(0.0, 0.0, 0.0);
  camera->lookAt(Ogre::Vector3(0, 0, 1));
  camera->roll(Ogre::Radian(3.141592653));
  camera->setNearClipDistance( 0.01f );

  double width = camera_info.width;
  double height = camera_info.height;

  double fx = camera_info.P[0];
  double fy = camera_info.P[5];

  // Add the camera's translation relative to the left camera (untested);

  double tx = -1 * (camera_info.P[3] / fx);
  double ty = -1 * (camera_info.P[7] / fy);

  camera->setPosition( tx, ty, 0 );

  // calculate the projection matrix
  double cx = camera_info.P[2];
  double cy = camera_info.P[6]+1;

  double far_plane = 100;
  double near_plane = 0.01;

  Ogre::Matrix4 proj_matrix;
  proj_matrix = Ogre::Matrix4::ZERO;

  proj_matrix[0][0]= 2.0*fx/width;
  proj_matrix[1][1]= 2.0*fy/height;

  proj_matrix[0][2]= 2*(0.5 - cx/width);
  proj_matrix[1][2]= 2*(cy/height - 0.5);

  proj_matrix[2][2]= -(far_plane+near_plane) / (far_plane-near_plane);
  proj_matrix[2][3]= -2.0*far_plane*near_plane / (far_plane-near_plane);

  proj_matrix[3][2]= -1;

  camera->setCustomProjectionMatrix( true, proj_matrix );
}

}

