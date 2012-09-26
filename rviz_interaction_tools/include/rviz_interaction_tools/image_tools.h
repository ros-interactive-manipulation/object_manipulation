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

/*! A few tools for retrieving points and info from point clouds, images or disparity images,
  and converting from 2D to 3D coordinates.
 */
#ifndef IMAGE_TOOLS_H
#define IMAGE_TOOLS_H

#include <cstring>

#include <ros/console.h>
#include <ros/assert.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <stereo_msgs/DisparityImage.h>

#include <cmath>

namespace rviz_interaction_tools {

//! Get a point as three floats from a dense point cloud using image coordinates
inline void getPoint(const sensor_msgs::PointCloud2 &cloud, 
                     unsigned int h, unsigned int w, float &x, float &y, float &z)
{
  ROS_ASSERT(h<cloud.height && w<cloud.width);
  memcpy(&x, &(cloud.data.at(h*cloud.row_step + w*cloud.point_step)), sizeof(float));
  memcpy(&y, &(cloud.data.at(h*cloud.row_step + w*cloud.point_step + sizeof(float))), sizeof(float));
  memcpy(&z, &(cloud.data.at(h*cloud.row_step + w*cloud.point_step + 2*sizeof(float))), sizeof(float));  
}

//! Check if a dense point cloud has a point at given image coordinates
inline bool hasPoint(const sensor_msgs::PointCloud2 &cloud, unsigned int h, unsigned int w)
{
  ROS_ASSERT(h<cloud.height || w<cloud.width);
  float x;
  memcpy(&x, &(cloud.data.at(h*cloud.row_step + w*cloud.point_step)), sizeof(float));
  if (std::isnan(x) || std::isinf(x)) return false;
  return true;
}

//! Get the value from an image at given image coordinates as a float
inline void getValue(const sensor_msgs::Image &image, unsigned int h, unsigned int w, float &val)
{
  ROS_ASSERT(h<image.height && w<image.width);
  memcpy(&val, &(image.data.at( h*image.step + sizeof(float)*w )), sizeof(float));
}

//! Set the value from an image at given image coordinates as a float
inline void setValue(sensor_msgs::Image &image, unsigned int h, unsigned int w, 
		     float val)
{
  ROS_ASSERT(h<image.height && w<image.width);
  memcpy(&(image.data.at( h*image.step + sizeof(float)*w )), &val, sizeof(float));
}

//! Check if an image has a value at given image coordinates that is not inf or nan
inline bool hasPoint(const sensor_msgs::Image &image, unsigned int h, unsigned int w)
{
  ROS_ASSERT(h<image.height && w<image.width);
  float val;
  memcpy(&val, &(image.data.at( h*image.step + sizeof(float)*w )), sizeof(float));
  if (std::isnan(val) || std::isinf(val)) return false;
  return true;
}

//! Check if a disparity image as a valid value at given image coordinates
inline bool hasDisparityValue(const stereo_msgs::DisparityImage &disparity_image, unsigned int h, unsigned int w)
{
  ROS_ASSERT(h<disparity_image.image.height && w<disparity_image.image.width);
  float val;
  memcpy(&val, &(disparity_image.image.data.at( h*disparity_image.image.step + sizeof(float)*w )), sizeof(float));
  if (std::isnan(val) || std::isinf(val)) return false;
  if (val <=0 || val < disparity_image.min_disparity || val > disparity_image.max_disparity) return false;
  return true;
}

//! Get the 3d coordinates of a point, given its image coordinates, its disparity value, as well as the 
//! images themselves which contain additional calibration information
inline void projectTo3d(float u, float v, float disparity,
                        const stereo_msgs::DisparityImage &disparity_image,
                        const sensor_msgs::CameraInfo &camera_info,
                        float &x, float &y, float &z)
{
  float fx = camera_info.P[0*4+0]; 
  float fy = camera_info.P[1*4+1]; 
  float cx = camera_info.P[0*4+2]; 
  float cy = camera_info.P[1*4+2]; 
  
  z = (disparity_image.f * disparity_image.T) / disparity;
  x = (u - cx) * z / fx;
  y = (v - cy) * z / fy;
}

//! Get the 3D coordinates of a point given its image coordinates and the disparity image
inline void getPoint(const stereo_msgs::DisparityImage &disparity_image, 
                     const sensor_msgs::CameraInfo &camera_info,
                     unsigned int h, unsigned int w,
                     float &x, float &y, float &z)
{
  ROS_ASSERT(hasDisparityValue(disparity_image, h, w));
  float disparity;
  getValue(disparity_image.image, h, w, disparity);
  projectTo3d(w, h, disparity, disparity_image, camera_info, x, y, z);
}

}


#endif
