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

#pragma once

#include <cstdint>
#include <string>

namespace v4l2_camera {

/**
 * @brief Identifies image encoding type --
 * raw, compressed, or unknown.
*/
enum class ImageEncodingType
{
    unknown,
    raw,
    compressed
};

/**
 * @brief Get ROS encoding string from V4L2 pixel format
 *
 * @param pixel_format V4L2 pixel format, as defined here:
 * https://www.kernel.org/doc/html/v4.14/media/uapi/v4l/pixfmt.html
 *
 * @return ROS image encoding string (e.g. "rgb8", "yuyv", etc.)
*/
std::string imageEncodingString(std::uint32_t pixel_format);

/**
 * @brief Get encoding type from V4L2 pixel format
 *
 * @param pixel_format V4L2 pixel format, as defined here:
 * https://www.kernel.org/doc/html/v4.14/media/uapi/v4l/pixfmt.html
 *
 * @return encoding type of @a pixel_format
*/
ImageEncodingType imageEncodingType(std::uint32_t pixel_format);

} // namespace v4l2_camera