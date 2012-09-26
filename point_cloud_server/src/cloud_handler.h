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

// author: Adam Leeper

#ifndef _CLOUD_CONTAINER_H_
#define _CLOUD_CONTAINER_H_

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

class CloudContainer
{
public:

  CloudContainer( ros::NodeHandle *nh, tf::TransformListener *tfl,
                  std::string name, const std::string &topic);

  ~CloudContainer();

  //! Clear the cloud stored in this object
  void clear();

  //! Refresh the cloud stored in this object
  bool refresh(const std::string &topic, const std::string &frame);

  //! Get the cloud stored in this object
  //sensor_msgs::PointCloud2 get();
  bool get(sensor_msgs::PointCloud2 &cloud, const std::string &frame);

  //! Set the cloud stored in this object
  void set(const sensor_msgs::PointCloud2 &cloud, const std::string &frame);

  //! Use TF to transform the internally stored cloud to this target frame
  void changeFrame(const std::string &frame);

private:

  sensor_msgs::PointCloud2 cloud_;

  std::string name_, topic_;
  ros::NodeHandle *nh_;
  ros::Publisher pub_point_;
  tf::TransformListener *tfl_;
};

#endif
