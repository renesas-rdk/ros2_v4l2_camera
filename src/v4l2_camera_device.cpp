// Copyright 2019 Bold Hearts
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

#include "v4l2_camera/v4l2_camera_device.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <system_error>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "v4l2_camera/fourcc.hpp"

using v4l2_camera::V4l2CameraDevice;

V4l2CameraDevice::V4l2CameraDevice(std::string device)
: device_{std::move(device)}
{
}

void V4l2CameraDevice::open()
{
  fd_ = ::open(device_.c_str(), O_RDWR);

  if (fd_ < 0) {
    std::ostringstream msg;
    msg << "Failed opening device " << device_;
    throw std::system_error {errno, std::system_category(), msg.str()};
  }

  // List capabilities
  ioctl(fd_, VIDIOC_QUERYCAP, &capabilities_);

  auto canRead = capabilities_.capabilities & V4L2_CAP_READWRITE;
  auto canStream = capabilities_.capabilities & V4L2_CAP_STREAMING;

  RCLCPP_INFO(
    rclcpp::get_logger("v4l2_camera"),
    "Driver: %s", capabilities_.driver);
  RCLCPP_INFO(
    rclcpp::get_logger("v4l2_camera"),
    "Version: %s", std::to_string(capabilities_.version).c_str());
  RCLCPP_INFO(
    rclcpp::get_logger("v4l2_camera"),
    "Device: %s", capabilities_.card);
  RCLCPP_INFO(
    rclcpp::get_logger("v4l2_camera"),
    "Location: %s", capabilities_.bus_info);

  RCLCPP_INFO(
    rclcpp::get_logger("v4l2_camera"),
    "Capabilities:");
  RCLCPP_INFO(
    rclcpp::get_logger("v4l2_camera"),
    "  Read/write: %s", (canRead ? "YES" : "NO"));
  RCLCPP_INFO(
    rclcpp::get_logger("v4l2_camera"),
    "  Streaming: %s", (canStream ? "YES" : "NO"));

  // Get current data (pixel) format
  auto formatReq = v4l2_format{};
  formatReq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(fd_, VIDIOC_G_FMT, &formatReq);
  cur_data_format_ = PixelFormat{formatReq.fmt.pix};

  RCLCPP_INFO(
    rclcpp::get_logger("v4l2_camera"),
    "Current pixel format: %s @ %sx%s", FourCC::toString(cur_data_format_.pixelFormat).c_str(),
    std::to_string(cur_data_format_.width).c_str(),
    std::to_string(cur_data_format_.height).c_str());

  // List all available image formats and controls
  listImageFormats();
  listImageSizes();
  listControls();

  RCLCPP_INFO(rclcpp::get_logger("v4l2_camera"), "Available pixel formats: ");
  for (auto const & format : image_formats_) {
    RCLCPP_INFO(
      rclcpp::get_logger("v4l2_camera"),
      "  %s - %s", FourCC::toString(format.pixelFormat).c_str(), format.description.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("v4l2_camera"), "Available controls: ");
  for (auto const & control : controls_) {
    try
    {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger("v4l2_camera"),
          "  " << control.name << " (" <<
          static_cast<unsigned>(control.type) << ") = " <<
          getControlValue(control.id) <<
          (control.inactive ? " [inactive]" : ""));
    }
    catch (std::runtime_error const& e)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("v4l2_camera"), e.what());
    }
  }
}

void V4l2CameraDevice::start()
{
  RCLCPP_INFO(rclcpp::get_logger("v4l2_camera"), "Starting camera");
  initMemoryMapping();

  // Queue the buffers
  for (auto const & buffer : buffers_) {
    auto buf = v4l2_buffer{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = buffer.index;

    if (-1 == ioctl(fd_, VIDIOC_QBUF, &buf))
      throw std::system_error {errno, std::system_category(), "Buffer failure on capture start"};
  }

  // Start stream
  unsigned type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl(fd_, VIDIOC_STREAMON, &type))
    throw std::system_error {errno, std::system_category(),  "Failed stream start"};
}

void V4l2CameraDevice::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("v4l2_camera"), "Stopping camera");
  // Stop stream
  unsigned type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl(fd_, VIDIOC_STREAMOFF, &type))
    throw std::system_error {errno, std::system_category(), "Failed stream stop"};

  // De-initialize buffers
  for (auto const & buffer : buffers_) {
    munmap(buffer.start, buffer.length);
  }

  buffers_.clear();

  auto req = v4l2_requestbuffers{};

  // Free all buffers
  req.count = 0;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  ioctl(fd_, VIDIOC_REQBUFS, &req);
}

std::string V4l2CameraDevice::getCameraName()
{
  auto name = std::string{reinterpret_cast<char *>(capabilities_.card)};
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);
  std::replace(name.begin(), name.end(), ' ', '_');
  return name;
}

v4l2_camera::Image V4l2CameraDevice::capture()
{
  auto buf = v4l2_buffer{};

  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  // Dequeue buffer with new image
  if (-1 == ioctl(fd_, VIDIOC_DQBUF, &buf))
    throw std::system_error {errno, std::system_category(), "Error dequeueing buffer"};

  // Copy over buffer data
  auto const & buffer = buffers_[buf.index];
  Image image {
    cur_data_format_,
    {buffer.start, buffer.start + cur_data_format_.imageByteSize},
  };

  // Requeue buffer to be reused for new captures
  if (-1 == ioctl(fd_, VIDIOC_QBUF, &buf))
    throw std::system_error {errno, std::system_category(), "Error re-queueing buffer"};

  return image;
}

int32_t V4l2CameraDevice::getControlValue(uint32_t id) const
{
  auto ctrl = v4l2_control{};
  ctrl.id = id;
  if (-1 == ioctl(fd_, VIDIOC_G_CTRL, &ctrl)) {
    std::stringstream msg;
    msg << "Failed getting value for control " << id;
    throw std::system_error {errno, std::system_category(), msg.str()};
  }
  return ctrl.value;
}

void V4l2CameraDevice::setControlValue(uint32_t id, int32_t value)
{
  auto ctrl = v4l2_control{};
  ctrl.id = id;
  ctrl.value = value;

  auto control = std::find_if(
    controls_.begin(), controls_.end(),
    [id](Control const & c) {return c.id == id;});

  if (-1 == ioctl(fd_, VIDIOC_S_CTRL, &ctrl)) {
    std::stringstream msg;
    msg << "Failed setting value for control " << control->name << " to " << value;
    throw std::system_error {errno, std::system_category(), msg.str()};
  }

  RCLCPP_INFO(
    rclcpp::get_logger(
      "v4l2_camera"), "Succesfully set value for control %s to %s", control->name.c_str(),
    std::to_string(value).c_str());
}

void V4l2CameraDevice::requestDataFormat(const PixelFormat & format)
{
  auto formatReq = v4l2_format{};
  formatReq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  formatReq.fmt.pix.pixelformat = format.pixelFormat;
  formatReq.fmt.pix.width = format.width;
  formatReq.fmt.pix.height = format.height;

  RCLCPP_INFO(
    rclcpp::get_logger("v4l2_camera"),
    "Requesting format: %sx%s %s",
    std::to_string(format.width).c_str(),
    std::to_string(format.height).c_str(),
    FourCC::toString(format.pixelFormat).c_str());

  // Perform request
  if (-1 == ioctl(fd_, VIDIOC_S_FMT, &formatReq))
    throw std::system_error {errno, std::system_category(), "Failed requesting pixel format"};

  RCLCPP_INFO(rclcpp::get_logger("v4l2_camera"), "Success");
  cur_data_format_ = PixelFormat{formatReq.fmt.pix};
}

void V4l2CameraDevice::listImageFormats()
{
  image_formats_.clear();

  struct v4l2_fmtdesc fmtDesc;
  fmtDesc.index = 0;
  fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  while (ioctl(fd_, VIDIOC_ENUM_FMT, &fmtDesc) == 0) {
    image_formats_.emplace_back(fmtDesc);
    fmtDesc.index++;
  }
}

void V4l2CameraDevice::listImageSizes()
{
  image_sizes_.clear();
  struct v4l2_frmsizeenum frmSizeEnum;
  // Supported sizes can be different per format
  for (auto const & f : image_formats_) {
    frmSizeEnum.index = 0;
    frmSizeEnum.pixel_format = f.pixelFormat;

    if (-1 == ioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &frmSizeEnum)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("v4l2_camera"),
        "Failed listing frame size " << strerror(errno) << " (" << errno << ")");
      continue;
    }

    switch (frmSizeEnum.type) {
      case V4L2_FRMSIZE_TYPE_DISCRETE:
        image_sizes_[f.pixelFormat] = listDiscreteImageSizes(frmSizeEnum);
        break;
      case V4L2_FRMSIZE_TYPE_STEPWISE:
        image_sizes_[f.pixelFormat] = listStepwiseImageSizes(frmSizeEnum);
        break;
      case V4L2_FRMSIZE_TYPE_CONTINUOUS:
        image_sizes_[f.pixelFormat] = listContinuousImageSizes(frmSizeEnum);
        break;
      default:
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger("v4l2_camera"),
          "Frame size type not supported: " << frmSizeEnum.type);
        continue;
    }
  }
}

V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listDiscreteImageSizes(
  v4l2_frmsizeenum frm_size_enum)
{
  auto sizes = ImageSizesVector{};

  do {
    sizes.emplace_back(std::make_pair(frm_size_enum.discrete.width, frm_size_enum.discrete.height));
    frm_size_enum.index++;
  } while (ioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &frm_size_enum) == 0);

  return std::make_pair(ImageSizeType::DISCRETE, std::move(sizes));
}

V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listStepwiseImageSizes(
  v4l2_frmsizeenum frm_size_enum)
{
  // Three entries: min size, max size and stepsize
  auto sizes = ImageSizesVector(3);
  sizes[0] = std::make_pair(frm_size_enum.stepwise.min_width, frm_size_enum.stepwise.min_height);
  sizes[1] = std::make_pair(frm_size_enum.stepwise.max_width, frm_size_enum.stepwise.max_height);
  sizes[2] = std::make_pair(frm_size_enum.stepwise.step_width, frm_size_enum.stepwise.step_height);

  return std::make_pair(ImageSizeType::STEPWISE, std::move(sizes));
}

V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listContinuousImageSizes(
  v4l2_frmsizeenum frm_size_enum)
{
  // Two entries: min size and max size, stepsize is implicitly 1
  auto sizes = ImageSizesVector(2);
  sizes[0] = std::make_pair(frm_size_enum.stepwise.min_width, frm_size_enum.stepwise.min_height);
  sizes[1] = std::make_pair(frm_size_enum.stepwise.max_width, frm_size_enum.stepwise.max_height);

  return std::make_pair(ImageSizeType::CONTINUOUS, std::move(sizes));
}

v4l2_camera::Control V4l2CameraDevice::queryControl(uint32_t id, bool silent)
{
  auto queryctrl = v4l2_queryctrl{};
  queryctrl.id = id;

  if (ioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl) != 0) {
    if (!silent) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("v4l2_camera"),
        "Failed querying control with ID: " << id << " - " << strerror(
          errno) << " (" << errno << ")");
    }
    return {};
  }

  auto menuItems = std::map<int, std::string>{};
  if (queryctrl.type == (unsigned)ControlType::MENU) {
    auto querymenu = v4l2_querymenu{};
    querymenu.id = queryctrl.id;

    // Query all enum values
    for (auto i = queryctrl.minimum; i <= queryctrl.maximum; i++) {
      querymenu.index = i;
      if (ioctl(fd_, VIDIOC_QUERYMENU, &querymenu) == 0) {
        menuItems[i] = (const char *)querymenu.name;
      }
    }
  }

  auto control = Control{};
  control.id = queryctrl.id;
  control.name = std::string{reinterpret_cast<char *>(queryctrl.name)};
  control.type = static_cast<ControlType>(queryctrl.type);
  control.minimum = queryctrl.minimum;
  control.maximum = queryctrl.maximum;
  control.defaultValue = queryctrl.default_value;
  control.menuItems = std::move(menuItems);
  control.disabled = (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) != 0;
  control.inactive = (queryctrl.flags & V4L2_CTRL_FLAG_INACTIVE) != 0;

  return control;
}

void V4l2CameraDevice::listControls()
{
  controls_.clear();

  auto query_id = V4L2_CID_BASE;

  while (true) {
    auto control = queryControl(query_id, true);
    if (control.id == 0) {break;}

    if (control.disabled) {
      query_id = control.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
      continue;
    }

    controls_.push_back(control);

    // Get ready to query next item
    query_id = control.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  }
}

void V4l2CameraDevice::initMemoryMapping()
{
  auto req = v4l2_requestbuffers{};

  // Request 4 buffers
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  ioctl(fd_, VIDIOC_REQBUFS, &req);

  // Didn't get more than 1 buffer
  if (req.count < 2)
    throw std::runtime_error {"Insufficient buffer memory"};

  buffers_ = std::vector<Buffer>(req.count);

  for (auto i = 0u; i < req.count; ++i) {
    auto buf = v4l2_buffer{};

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    ioctl(fd_, VIDIOC_QUERYBUF, &buf);

    buffers_[i].index = buf.index;
    buffers_[i].length = buf.length;
    buffers_[i].start =
      static_cast<unsigned char *>(
      mmap(
        NULL /* start anywhere */,
        buf.length,
        PROT_READ | PROT_WRITE /* required */,
        MAP_SHARED /* recommended */,
        fd_, buf.m.offset));

    if (MAP_FAILED == buffers_[i].start)
      throw std::system_error {errno, std::system_category(), "Failed mapping device memory"};
  }
}
