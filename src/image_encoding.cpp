#include <unordered_map>

#include <linux/videodev2.h>

#include <sensor_msgs/image_encodings.hpp>

#include "image_encoding.hpp"

namespace v4l2_camera
{
    std::string imageEncoding(std::uint32_t pixel_format)
    {
        // For V4L2 pixel formats, see https://www.kernel.org/doc/html/v4.19/media/uapi/v4l/pixfmt-packed-rgb.html
        // For ROS image encoding strings, see
        // http://docs.ros.org/en/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html
        static std::unordered_map<std::uint32_t, std::string> const pixel_format_map =
        {
            {V4L2_PIX_FMT_YUYV, sensor_msgs::image_encodings::YUV422_YUY2},
            {V4L2_PIX_FMT_UYVY, sensor_msgs::image_encodings::YUV422},
            {V4L2_PIX_FMT_GREY, sensor_msgs::image_encodings::MONO8},
            {V4L2_PIX_FMT_BGR24, sensor_msgs::image_encodings::BGR8},
            {V4L2_PIX_FMT_RGB24, sensor_msgs::image_encodings::RGB8},
            {V4L2_PIX_FMT_ABGR32, sensor_msgs::image_encodings::BGRA8},
            {V4L2_PIX_FMT_XBGR32, "bgrx8"},
            {V4L2_PIX_FMT_ARGB32, "argb8"},
            {V4L2_PIX_FMT_XRGB32, "xrgb8"},
        };

        auto const it = pixel_format_map.find(pixel_format);
        return it != pixel_format_map.end() ? it->second : std::string {};
    }
}