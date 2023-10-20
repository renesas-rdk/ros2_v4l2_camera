#pragma once

#include <string>
#include <cstdint>

namespace v4l2_camera
{
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
}