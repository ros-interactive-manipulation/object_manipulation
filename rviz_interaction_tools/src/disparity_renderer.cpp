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

#include "rviz_interaction_tools/disparity_renderer.h"
#include "rviz_interaction_tools/image_tools.h"

#include <sensor_msgs/image_encodings.h>

#include <OGRE/OgreSceneNode.h>

namespace rviz_interaction_tools
{

DisparityRenderer::DisparityRenderer( Ogre::SceneNode* scene_root, unsigned char render_queue_group ) :
  scene_root_(scene_root),
  new_point_cloud_(0)
{
  image_point_cloud_.setRenderQueueGroup(render_queue_group);
  scene_root_->attachObject(&image_point_cloud_);
}

DisparityRenderer::~DisparityRenderer()
{
  scene_root_->detachObject(&image_point_cloud_);
}

bool DisparityRenderer::setDisparityImage(
    const stereo_msgs::DisparityImage &disparity_image,
    const sensor_msgs::CameraInfo &camera_info, const sensor_msgs::Image *image )
{
  boost::mutex::scoped_lock(mutex_);

  //-------------------- prepare point cloud from image

  raw_points_.clear();
  raw_points_.reserve(disparity_image.image.width
      * disparity_image.image.height);

  rviz_interaction_tools::PointCloud::Point point;
  point.setColor(1, 1, 1);

  for (unsigned int i = 0; i < disparity_image.image.height; i++)
  {
    for (unsigned int j = 0; j < disparity_image.image.width; j++)
    {
      if (rviz_interaction_tools::hasDisparityValue(disparity_image, i, j))
      {

        rviz_interaction_tools::getPoint(disparity_image, camera_info, i, j,
            point.x, point.y, point.z);
        if (std::isnan(point.x) || std::isinf(point.x))
          continue;
        if (std::isnan(point.y) || std::isinf(point.y))
          continue;
        if (std::isnan(point.z) || std::isinf(point.z))
          continue;

        if (point.x < -50 || point.x > 50)
          ROS_INFO("Point x: %f", point.x);
        if (point.y < -50 || point.y > 50)
          ROS_INFO("Point x: %f", point.y);
        if (point.z < -50 || point.z > 50)
          ROS_INFO("Point x: %f", point.z);

        if (image)
        {
          if (image->encoding == sensor_msgs::image_encodings::MONO8)
          {
            float l = float(image->data[i * image->step + j]) / 255.0;
            point.setColor(l, l, l);
          }
          else if (image->encoding == sensor_msgs::image_encodings::BGR8)
          {
            float b = float(image->data[i * image->step + j*3]) / 255.0;
            float g = float(image->data[i * image->step + j*3 + 1]) / 255.0;
            float r = float(image->data[i * image->step + j*3 + 2]) / 255.0;
            point.setColor(r, g, b);
          }
          else if (image->encoding == sensor_msgs::image_encodings::RGB8)
          {
            float r = float(image->data[i * image->step + j*3]) / 255.0;
            float g = float(image->data[i * image->step + j*3 + 1]) / 255.0;
            float b = float(image->data[i * image->step + j*3 + 2]) / 255.0;
            point.setColor(r, g, b);
          }
          else
          {
            point.setColor(1,1,1);
          }
        }

        raw_points_.push_back(point);
      }
    }
  }

  new_point_cloud_ = true;
  return true;
}

bool DisparityRenderer::update()
{
  boost::mutex::scoped_lock(mutex_);

  if (!new_point_cloud_)
  {
    return false;
  }

  image_point_cloud_.clear();
  image_point_cloud_.addPoints(&raw_points_[0], raw_points_.size());

  Ogre::Vector3 min = image_point_cloud_.getBoundingBox().getMinimum();
  Ogre::Vector3 max = image_point_cloud_.getBoundingBox().getMaximum();
  ROS_DEBUG("Bbox min: %f %f %f max: %f %f %f  # of points: %d",
      min.x, min.y, min.z, max.x, max.y, max.z, (int)raw_points_.size() );

  return true;
}

void DisparityRenderer::clear()
{
  boost::mutex::scoped_lock(mutex_);
  image_point_cloud_.clear();
}

}
