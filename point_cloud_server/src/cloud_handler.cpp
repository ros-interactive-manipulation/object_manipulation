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

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include "cloud_handler.h"
#include <pcl_ros/transforms.h>

CloudContainer::CloudContainer( ros::NodeHandle *nh, tf::TransformListener *tfl,
                                std::string name, const std::string &topic) :
  name_(name)
  , topic_(topic)
  , nh_(nh)
  , tfl_(tfl)
{

}

CloudContainer::~CloudContainer()
{
}

void CloudContainer::clear()
{
  cloud_ = sensor_msgs::PointCloud2();
}

void CloudContainer::changeFrame(const std::string &frame)
{
  if(!frame.empty() && frame.compare(cloud_.header.frame_id))
  {
    ROS_DEBUG("Cloud in frame [%s] is being transformed to frame [%s].", 
              cloud_.header.frame_id.c_str(), frame.c_str());
    // Transform
    ros::Time saved_time = cloud_.header.stamp;
    cloud_.header.stamp = ros::Time(0);
    try{
      pcl_ros::transformPointCloud(frame, cloud_, cloud_, *tfl_);
    }
    catch(...){
      ROS_ERROR("TF had a problem transforming between [%s] and [%s].",
                frame.c_str(), cloud_.header.frame_id.c_str());
    }
    cloud_.header.stamp = saved_time;
  }
}

void CloudContainer::set( const sensor_msgs::PointCloud2 &cloud, const std::string &frame )
{
  cloud_ = cloud;
  changeFrame( frame );
  
}

bool CloudContainer::get( sensor_msgs::PointCloud2 &cloud, const std::string &frame )
{

  // This is different because we aren't changed the local cloud.
  if(!frame.empty() && frame.compare(cloud_.header.frame_id))
  {
    ROS_DEBUG("Cloud in frame [%s] is being returned in frame [%s].", 
              cloud_.header.frame_id.c_str(), frame.c_str());
    // Transform
    ros::Time saved_time = cloud_.header.stamp;
    cloud_.header.stamp = ros::Time(0);
    try{
      pcl_ros::transformPointCloud(frame, cloud_, cloud, *tfl_);
    }
    catch(...){
      ROS_ERROR("TF had a problem transforming between [%s] and [%s].",
                frame.c_str(), cloud_.header.frame_id.c_str());
      return false;
    }
    cloud_.header.stamp = saved_time;
  }
  else
  {
    cloud = cloud_;
  }
  return true;
}

//sensor_msgs::PointCloud2 CloudContainer::get( const std::string &frame )
//{
//  return cloud_;
//}

bool CloudContainer::refresh(const std::string &topic, const std::string &frame)
{
  topic_ = topic;
  if(topic_.empty()) return false;

  ROS_DEBUG("Waiting for cloud on topic [%s]", topic_.c_str());
  sensor_msgs::PointCloud2::ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, *nh_, ros::Duration(10.0));
  if(cloud)
  {
    cloud_ = *cloud;
    changeFrame( frame );
    ROS_DEBUG("Received cloud on topic [%s] with %d points!", topic_.c_str(), cloud_.width * cloud_.height);
    return true;
  }
  else
  {
    this->cloud_ = sensor_msgs::PointCloud2();
    ROS_WARN("No message received on topic [%s]... is the remapping correct?", topic.c_str());
    return false;
  }
}

