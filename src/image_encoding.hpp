#pragma once

#include <string>
#include <cstdint>

namespace v4l2_camera
{
    std::string imageEncoding(std::uint32_t pixel_format);
}