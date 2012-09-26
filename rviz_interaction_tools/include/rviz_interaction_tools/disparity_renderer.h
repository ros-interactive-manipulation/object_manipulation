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

#ifndef DISPARITY_RENDERER_H_
#define DISPARITY_RENDERER_H_

#include "rviz_interaction_tools/point_cloud.h"

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>

namespace Ogre {
class SceneNode;
}

namespace rviz_interaction_tools {


/// Fills the Z Buffer based on a disparity image
class DisparityRenderer
{
public:

  DisparityRenderer( Ogre::SceneNode* scene_root, unsigned char render_queue_group );

  virtual ~DisparityRenderer();

  // convert disparity data to point cloud and store
  bool setDisparityImage( const stereo_msgs::DisparityImage &disparity_image,
      const sensor_msgs::CameraInfo &camera_info,
      const sensor_msgs::Image *image=0 );

  // insert the data into the Ogre scene (must be called from main app thread)
  bool update();

  void clear();


private:

  Ogre::SceneNode* scene_root_;

  boost::mutex mutex_;

  rviz_interaction_tools::PointCloud image_point_cloud_;

  //used to pass data between threads
  bool new_point_cloud_;
  std::vector<rviz_interaction_tools::PointCloud::Point> raw_points_;
};

}

#endif /* DISPARITY_RENDERER_H_ */
