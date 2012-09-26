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

#include "rviz_interaction_tools/image_overlay.h"
#include "rviz_interaction_tools/image_tools.h"
#include "rviz_interaction_tools/unique_string_manager.h"

#include <sensor_msgs/image_encodings.h>

#include <vector>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTechnique.h>

using namespace Ogre;

namespace rviz_interaction_tools
{

ImageOverlay::ImageOverlay( Ogre::SceneNode* scene_root, unsigned char render_queue_group ) :
  scene_root_(scene_root),
  new_image_(false),
  width_(0),
  height_(0)
{
  //create empty image
  Ogre::PixelFormat format = Ogre::PF_BYTE_RGB;
  std::vector<unsigned char> empty_buffer;

  empty_buffer.assign(100 * 100 * 3, 0);
  void* data_ptr = (void*) &empty_buffer[0];
  uint32_t data_size = empty_buffer.size();

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(data_ptr, data_size));

  try
  {
    empty_image_.loadRawData(pixel_stream, 100, 100, 1, format, 1, 0);
  } catch (Ogre::Exception& e)
  {
    ROS_ERROR("Error loading image: %s", e.what());
  }

  UniqueStringManager unique_string_mgr;

  resource_group_name_ = unique_string_mgr.unique("rviz_interaction_tools::ImageOverlay");
  Ogre::ResourceGroupManager::getSingleton().createResourceGroup(resource_group_name_);
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(resource_group_name_);

  //prepare empty texture
  texture_ = Ogre::TextureManager::getSingleton().loadImage(
      unique_string_mgr.unique("ImageOverlayTexture"),
      resource_group_name_, empty_image_,
      Ogre::TEX_TYPE_2D, 0);

  //prepare material
  texture_material_ = Ogre::MaterialManager::getSingleton().create(
      unique_string_mgr.unique("ImageOverlayMaterial"),
      resource_group_name_);

  Ogre::TextureUnitState* texture_unit =
      texture_material_->getTechnique(0)->getPass(0)->createTextureUnitState();
  texture_unit->setTextureName(texture_->getName());
  texture_unit->setTextureFiltering(Ogre::TFO_NONE);

  texture_material_->setSceneBlending(Ogre::SBT_REPLACE);
  texture_material_->setDepthWriteEnabled(false);
  texture_material_->setReceiveShadows(false);
  texture_material_->setDepthCheckEnabled(false);
  texture_material_->getTechnique(0)->setLightingEnabled(false);
  texture_material_->setCullingMode(Ogre::CULL_NONE);

  image_rect_ = new Ogre::Rectangle2D(true);
  //image texture will be rendered after point cloud, but before any other geometry
  image_rect_->setRenderQueueGroup(render_queue_group);
  image_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
  image_rect_->setMaterial(texture_material_->getName());
  image_rect_->setQueryFlags(0);

  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  image_rect_->setBoundingBox(aabInf);

  scene_root_->attachObject(image_rect_);
}

ImageOverlay::~ImageOverlay()
{
  Ogre::ResourceGroupManager::getSingleton().destroyResourceGroup(resource_group_name_);
  scene_root_->detachObject(image_rect_);
  delete image_rect_;
}

bool ImageOverlay::setImage( const sensor_msgs::Image &image,
    const stereo_msgs::DisparityImage &disparity_image )
{
  boost::mutex::scoped_lock lock(mutex_);

  if (!setImageNoLock(image))
  {
    return false;
  }

  if ((image.width != disparity_image.image.width) || (image.height
      != disparity_image.image.height))
  {
    ROS_ERROR("Size mismatch between image (%i x %i) and disparity image (%i x %i)!",
        image.width, image.height, disparity_image.image.width, disparity_image.image.height );
    return false;
  }

  //paint over the parts with no depth info
  for (unsigned int i = 0; i < image.height; i++)
  {
    for (unsigned int j = 0; j < image.width; j++)
    {
      unsigned int adr = i * image.width + j;
      if (!rviz_interaction_tools::hasDisparityValue(disparity_image, i, j))
      {
        if (image_buffer_[3 * adr + 0] < 215)
          image_buffer_[3 * adr + 0] += 40;
        else
          image_buffer_[3 * adr + 0] = 255;
        if (image_buffer_[3 * adr + 1] > 200)
          image_buffer_[3 * adr + 1] = 200;
        if (image_buffer_[3 * adr + 2] > 200)
          image_buffer_[3 * adr + 2] = 200;
      }
    }
  }

  new_image_ = true;

  return true;
}

bool ImageOverlay::setImage( const sensor_msgs::Image &image )
{
  boost::mutex::scoped_lock lock(mutex_);

  if (!setImageNoLock(image))
  {
    return false;
  }

  new_image_ = true;
  return true;
}

bool ImageOverlay::setImageNoLock( const sensor_msgs::Image &image )
{
  width_ = image.width;
  height_ = image.height;

  //copy image to a new buffer
  image_buffer_.resize(3 * width_ * height_, 0);

  if (image.encoding == sensor_msgs::image_encodings::MONO8)
  {
    for (unsigned int i = 0; i < image.height; i++)
    {
      for (unsigned int j = 0; j < image.width; j++)
      {
        unsigned int adr = i * image.width + j;
        image_buffer_[3 * adr + 0] = image.data[adr];
        image_buffer_[3 * adr + 1] = image.data[adr];
        image_buffer_[3 * adr + 2] = image.data[adr];
      }
    }
  }
  else if (image.encoding == sensor_msgs::image_encodings::BGR8)
  {
    for (unsigned int i = 0; i < image.height; i++)
    {
      for (unsigned int j = 0; j < image.width; j++)
      {
        unsigned int adr = i * image.width + j;
        unsigned int adr2 = i * image.step + j*3;
        image_buffer_[3 * adr + 0] = image.data[adr2 + 2];
        image_buffer_[3 * adr + 1] = image.data[adr2 + 1];
        image_buffer_[3 * adr + 2] = image.data[adr2 + 0];
      }
    }
  }
  else if (image.encoding == sensor_msgs::image_encodings::RGB8)
  {
    image_buffer_ = image.data;
  }
  else
  {
    ROS_ERROR("ImageDisplay only supports MONO8, RGB8 and BGR8 images");
    return false;
  }

  return true;
}

bool ImageOverlay::setImage( unsigned char *rgb_data, int width, int height )
{
  boost::mutex::scoped_lock lock(mutex_);

  if (width <= 0 || height <= 0)
  {
    ROS_ERROR( "Image dimensions must be > 0" );
    return false;
  }

  width_ = width;
  height_ = height;

  image_buffer_.resize(width * height * 3, 0);
  memcpy(&image_buffer_[0], rgb_data, width * height * 3);

  return true;
}

bool ImageOverlay::update()
{
  boost::mutex::scoped_lock lock(mutex_);

  if (!new_image_ || !width_ || image_buffer_.size() != unsigned(width_
      * height_ * 3))
  {
    return false;
  }

  Ogre::PixelFormat format = Ogre::PF_BYTE_RGB;

  void* data_ptr = (void*) &image_buffer_[0];
  uint32_t data_size = image_buffer_.size();

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(data_ptr, data_size));

  Ogre::Image ogre_image;

  try
  {
    ogre_image.loadRawData(pixel_stream, width_, height_, 1, format, 1, 0);
  } catch (Ogre::Exception& e)
  {
    ROS_ERROR("Error loading image: %s", e.what());
    return false;
  }

  texture_->unload();
  texture_->loadImage(ogre_image);

  return true;
}

void ImageOverlay::clear()
{
  boost::mutex::scoped_lock lock(mutex_);

  texture_->unload();

  new_image_ = false;
}

int ImageOverlay::getWidth()
{
  boost::mutex::scoped_lock lock(mutex_);
  int w = width_;
  return w;
}

int ImageOverlay::getHeight()
{
  boost::mutex::scoped_lock lock(mutex_);
  int h = height_;
  return h;
}

}
