#pragma once

#include <vector>
#include <cstdint>

#include <v4l2_camera/pixel_format.hpp>

namespace v4l2_camera
{
    struct Image
    {
        PixelFormat format;
        std::vector<std::uint8_t> data;
    };
}