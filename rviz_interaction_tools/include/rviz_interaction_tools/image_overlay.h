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

#ifndef IMAGE_OVERLAY_H_
#define IMAGE_OVERLAY_H_

#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreImage.h>
#include <OGRE/OgreMaterial.h>

#include <boost/thread/mutex.hpp>


namespace Ogre
{
class SceneNode;
class Rectangle2D;
}

namespace rviz_interaction_tools
{

/// paints an image, filling the whole viewport
/// performs locking so it can used in a thread-safe way
class ImageOverlay
{
public:

  // @note always setup the render queue group so that the image gets rendered
  //       before the other geometry
  ImageOverlay( Ogre::SceneNode* scene_root, unsigned char render_queue_group );

  virtual ~ImageOverlay();

  /// copy mono or RGB image to texture, color areas with missing disparity in red (if present)
  /// @note disparity_image is optional
  /// @note you have to call update() for the changes to take effect
  bool setImage( const sensor_msgs::Image &image, const stereo_msgs::DisparityImage &disparity_image );

  bool setImage( const sensor_msgs::Image &image );

  /// copy RGB data to texture
  bool setImage( unsigned char *rgb_data, int width, int height );

  // copy the image data to the actual texture buffer
  // make sure to call this from the thread in which ogre is running
  bool update();

  void clear();

  int getWidth();
  int getHeight();

  Ogre::MaterialPtr getMaterial() { return texture_material_; }

private:

  // used by the other setImage methods
  bool setImageNoLock( const sensor_msgs::Image &image );

  boost::mutex mutex_;

  int count_;

  Ogre::MaterialPtr texture_material_;
  Ogre::TexturePtr texture_;
  Ogre::Image empty_image_;
  Ogre::Rectangle2D* image_rect_;

  Ogre::SceneNode* scene_root_;

  // used to transport data from one thread to the other
  bool new_image_;
  std::vector<unsigned char> image_buffer_;
  int width_;
  int height_;

  std::string resource_group_name_;
};

}

#endif /* IMAGE_OVERLAY_H_ */
