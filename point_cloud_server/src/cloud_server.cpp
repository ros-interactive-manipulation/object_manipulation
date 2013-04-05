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

// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

#include <ros/ros.h>
#include <math.h>

#include <boost/thread.hpp>
#include <tf/transform_listener.h>

#include <actionlib/server/simple_action_server.h>

#include "pcl_ros/transforms.h"

#include "cloud_handler.h"
#include <point_cloud_server/StoreCloudAction.h>

/** \brief A class for storing a map of point cloud handlers.
  */
class CloudServer
{
public:

  CloudServer() :
      nh_("/"),
      pnh_("~"),
      store_cloud_name_(ros::this_node::getName()),
      store_cloud_server_(nh_, store_cloud_name_, boost::bind(&CloudServer::storeCloud, this, _1), false)
  {
    ROS_INFO("%s is running.", store_cloud_name_.c_str() );
    store_cloud_server_.start();
  }

  ~CloudServer()
  {
  }

// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120
protected:

  //! Convenience function 1 to handle errors
  void handleTopicError(const point_cloud_server::StoreCloudGoalConstPtr &goal, const std::string &resolved_topic)
  {
    point_cloud_server::StoreCloudResult result;
    std::stringstream ss;
    ss << "Failed to get cloud [" << goal->name << "] on topic [" << resolved_topic << "]. Deleting from server.";
    ROS_WARN_STREAM(ss);
    cloud_map_.erase(cloud_map_it_);
    result.result = result.TOPIC_ERROR;
    store_cloud_server_.setAborted(result, ss.str());
  }

  //! Convenience function 2 to handle errors
  void handleMissingEntryError(const point_cloud_server::StoreCloudGoalConstPtr &goal)
  {
    point_cloud_server::StoreCloudResult result;
    std::stringstream ss;
    ROS_WARN("Didn't find cloud on server! (If you were trying to get a cloud, did you forget to provide a topic?)");
    ROS_WARN("goal->name was %s", goal->name.c_str());
    ROS_WARN("goal->topic was %s", goal->topic.c_str());
    result.result = result.NAME_ERROR;
    store_cloud_server_.setAborted(result, ss.str() );
  }

  //! Convenience function 3 to handle errors
  void handleTFError(const point_cloud_server::StoreCloudGoalConstPtr &goal)
  {
    point_cloud_server::StoreCloudResult result;
    std::stringstream ss;
    ss << "There was an error applying a transform.";
    ROS_WARN_STREAM(ss);
    result.result = result.TF_ERROR;
    store_cloud_server_.setAborted(result, ss.str() );
  }

  //! Convenience function 4 to handle errors
  void handleOtherError(const std::string &text)
  {
    point_cloud_server::StoreCloudResult result;
    ROS_ERROR_STREAM(text);
    result.result = result.OTHER_ERROR;
    store_cloud_server_.setAborted(result, text);
    return;
  }


  //! The action goal callback. Implements the StoreCLoudAction API.
  void storeCloud(const point_cloud_server::StoreCloudGoalConstPtr &goal)
  {
    std::string resolved_topic = nh_.resolveName(goal->topic, true);

    if(goal->name.empty())
    {
      handleOtherError("You must provide either a topic or a non-empty point cloud when specifying a STORE action.");
      return;
    }
    
    ROS_DEBUG(  "Received goal with name [%s], topic [%s] resolved to [%s], and action %d.", 
                goal->name.c_str(), goal->topic.c_str(), resolved_topic.c_str(), goal->action);
    
    point_cloud_server::StoreCloudResult result;

//    if(goal->action == goal->STORE || goal->action == goal->GET)
//    {
//      ROS_DEBUG("Cloud server is doing a STORE/GET action");
//
//      bool cloud_exists = goal->cloud.width*goal->cloud.height > 0;
//
//
//      // First check if we have either a topic or a point cloud to use...
//      if(goal->action == goal->STORE && goal->topic.empty() && !cloud_exists)
//      {
//        handleOtherError("You must provide either a topic or a non-empty point cloud when specifying a STORE action.");
//        return;
//      }
//
//      // Check if named handler exists, and if not, add it to the map:
//      cloud_map_it_ = cloud_map_.find(goal->name);
//      if(cloud_map_it_ == cloud_map_.end())
//      {
//        // Return error if we called GET without a topic or cloud
//        if(goal->action == goal->GET && goal->topic.empty() && !cloud_exists)
//        {
//          handleMissingEntryError(goal);
//          return;
//        }
//        else
//        {
//          ROS_DEBUG("Adding new entry [%s] to cloud server.", goal->name.c_str());
//          cloud_map_it_ = cloud_map_.insert(
//              std::pair<std::string, boost::shared_ptr<CloudContainer> >(
//                  goal->name, boost::shared_ptr<CloudContainer>(new CloudContainer(&nh_, goal->name, resolved_topic)))
//              ).first;
//        }
//      }
//
//      // Now that we have a handler...
//      if( goal->action == goal->STORE )
//      {
//        if(goal->topic.empty() )
//        {
//          cloud_map_it_->second->set( goal->cloud, goal->storage_frame_id );
//          ROS_DEBUG("Stored the cloud provided in the action goal into entry [%s].", goal->name.c_str());
//        }
//        else
//        {
//          if( !cloud_map_it_->second->refresh( resolved_topic, goal->storage_frame_id ) )
//          {
//            handleTopicError(goal, resolved_topic);
//            return;
//          }
//          ROS_DEBUG("Stored cloud [%s] from topic [%s].",
//                    goal->name.c_str(), resolved_topic.c_str());
//        }
//        store_cloud_server_.setSucceeded(result);
//        return;
//      }
//    }




    if(goal->action == goal->STORE)
    {
      ROS_DEBUG("Cloud server is doing a STORE/GET action");

      // First check if we have either a topic or a point cloud to use...
      if(goal->topic.empty() && (goal->cloud.width*goal->cloud.height == 0))
      {
        std::stringstream ss;
        ss << "You are incorrectly using the StoreCloud API "
           << "You must provide either a topic or a non-empty point cloud when specifying a STORE action.";
        ROS_ERROR_STREAM(ss);
        store_cloud_server_.setAborted(result, ss.str());
        return;
      }

      // Check if named handler exists, and if not, add it to the map:
      cloud_map_it_ = cloud_map_.find(goal->name);
      if(cloud_map_it_ == cloud_map_.end())
      {
        ROS_DEBUG("Adding new cloud [%s] to cloud server.", goal->name.c_str());
        cloud_map_it_ = cloud_map_.insert(
            std::pair<std::string, boost::shared_ptr<CloudContainer> >(
                goal->name, boost::shared_ptr<CloudContainer>(
                    new CloudContainer(&nh_, &tfl_, goal->name, resolved_topic)))).first;
      }

      if(goal->topic.empty())
      {
        cloud_map_it_->second->set( goal->cloud, goal->storage_frame_id );
        ROS_DEBUG("Stored the cloud provided in the action goal into cloud [%s].", goal->name.c_str());
      }
      else
      {
        if( !cloud_map_it_->second->refresh( resolved_topic, goal->storage_frame_id ) )
        {
          handleTopicError(goal, resolved_topic);
          return;
        }
        ROS_DEBUG("Stored cloud [%s] from topic [%s].",
                  goal->name.c_str(), resolved_topic.c_str());
      }
      store_cloud_server_.setSucceeded(result);
      return;
    }
    else if(goal->action == goal->GET)
    {
      ROS_DEBUG("Cloud server is doing a GET action");
      cloud_map_it_ = cloud_map_.find(goal->name);
      if(cloud_map_it_ != cloud_map_.end() || !goal->topic.empty())
      {
        if(goal->topic.empty())
        {
          ROS_DEBUG("Fetching cloud [%s] from cloud server.", goal->name.c_str());
        }
        else if(cloud_map_it_ == cloud_map_.end())
        {
          ROS_DEBUG("Storing new cloud [%s] on cloud server.", goal->name.c_str());
          cloud_map_it_ = cloud_map_.insert(
              std::pair<std::string, boost::shared_ptr<CloudContainer> >(
                  goal->name, boost::shared_ptr<CloudContainer>(
                      new CloudContainer(&nh_, &tfl_, goal->name, resolved_topic)))).first;
          if( !cloud_map_it_->second->refresh( resolved_topic, goal->storage_frame_id ) )
          {
            handleTopicError(goal, resolved_topic);
            return;
          }
        }
        else
        {
          ROS_DEBUG("Refreshing cloud [%s] on cloud server.", goal->name.c_str());
          if( !cloud_map_it_->second->refresh( resolved_topic, goal->storage_frame_id ) )
          {
            handleTopicError(goal, resolved_topic);
            return;
          }
        }

        // Actually get the cloud
        ROS_DEBUG("Getting cloud [%s] on cloud server and returning with result.", goal->name.c_str());
        if(!cloud_map_it_->second->get( result.cloud, goal->result_frame_id ))
        {
          handleTFError(goal);
          return;
        }
        store_cloud_server_.setSucceeded(result);
        return;
      }
      else
      {
        handleMissingEntryError(goal);
        return;
      }
    }
    else if(goal->action == goal->CLEAR)
    {
      ROS_DEBUG("Cloud server is doing a CLEAR action.");
      cloud_map_it_ = cloud_map_.find(goal->name);
      if(cloud_map_it_ != cloud_map_.end())
      {
        ROS_DEBUG("Removing cloud [%s] from cloud server.", goal->name.c_str());
        cloud_map_.erase(cloud_map_it_);
        store_cloud_server_.setSucceeded(result);
      }
      else
      {
        handleMissingEntryError(goal);
        return;
      }
    }
  }

// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

  //! Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  //! TF listener for doing transforms
  tf::TransformListener tfl_;

  //! A map of cloud handlers, allowing us to fetch clouds by name.
  std::map< std::string, boost::shared_ptr<CloudContainer> > cloud_map_;

  //! A convenience handle for accessing cloud handlers in the map.
  std::map< std::string, boost::shared_ptr<CloudContainer> >::iterator cloud_map_it_;

  //! The name of this action server
  std::string store_cloud_name_;

  //! The actual action server
  actionlib::SimpleActionServer<point_cloud_server::StoreCloudAction> store_cloud_server_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_server_action");
  CloudServer cloud_server;
  ros::spin();
}
