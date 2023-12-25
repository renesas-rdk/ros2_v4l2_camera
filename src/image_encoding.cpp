// Copyright 2023 Mużyk Biełarus
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "image_encoding.hpp"

#include <linux/videodev2.h>

#include <unordered_map>

#include <sensor_msgs/image_encodings.hpp>

namespace v4l2_camera
{
namespace
{

struct EncodingInfo
{
  std::string encodingString;
  ImageEncodingType encodingType;
};


static std::unordered_map<std::uint32_t, EncodingInfo> const pixel_format_map =
{
  {V4L2_PIX_FMT_YUYV,
    {sensor_msgs::image_encodings::YUV422_YUY2, ImageEncodingType::raw}},
  {V4L2_PIX_FMT_UYVY, {sensor_msgs::image_encodings::YUV422, ImageEncodingType::raw}},
  {V4L2_PIX_FMT_YUV420, {sensor_msgs::image_encodings::NV21, ImageEncodingType::raw}},
  {V4L2_PIX_FMT_GREY, {sensor_msgs::image_encodings::MONO8, ImageEncodingType::raw}},
  {V4L2_PIX_FMT_BGR24, {sensor_msgs::image_encodings::BGR8, ImageEncodingType::raw}},
  {V4L2_PIX_FMT_RGB24, {sensor_msgs::image_encodings::RGB8, ImageEncodingType::raw}},
  {V4L2_PIX_FMT_ABGR32, {sensor_msgs::image_encodings::BGRA8, ImageEncodingType::raw}},
  {V4L2_PIX_FMT_XBGR32, {"bgrx8", ImageEncodingType::raw}},
  {V4L2_PIX_FMT_ARGB32, {"argb8", ImageEncodingType::raw}},
  {V4L2_PIX_FMT_XRGB32, {"xrgb8", ImageEncodingType::raw}},
  {V4L2_PIX_FMT_JPEG, {"jpeg", ImageEncodingType::compressed}},
  {V4L2_PIX_FMT_MJPEG, {"mjpg", ImageEncodingType::compressed}},
};

}  // namespace


std::string imageEncodingString(std::uint32_t pixel_format)
{
  // For V4L2 pixel formats, see https://www.kernel.org/doc/html/v4.19/media/uapi/v4l/pixfmt-packed-rgb.html
  // For ROS image encoding strings, see
  // http://docs.ros.org/en/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html
  auto const it = pixel_format_map.find(pixel_format);
  return it != pixel_format_map.end() ? it->second.encodingString : std::string {};
}


ImageEncodingType imageEncodingType(std::uint32_t pixel_format)
{
  auto const it = pixel_format_map.find(pixel_format);
  return it != pixel_format_map.end() ? it->second.encodingType : ImageEncodingType::unknown;
}

}  // namespace v4l2_camera
