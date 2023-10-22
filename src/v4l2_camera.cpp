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

#include "v4l2_camera/v4l2_camera.hpp"

#include <cv_bridge/cv_bridge.h>

#include <bit>
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>

#include <sensor_msgs/image_encodings.hpp>

#include "v4l2_camera/fourcc.hpp"
#include "v4l2_camera/parameters.hpp"
#include "image_encoding.hpp"

using namespace std::chrono_literals;

namespace v4l2_camera
{

V4L2Camera::V4L2Camera(rclcpp::NodeOptions const & options)
: rclcpp::Node{"v4l2_camera", options},
  parameters_{get_node_parameters_interface(), get_node_topics_interface(),
    get_node_logging_interface()},
  canceled_{false}
{
  // Prepare publisher
  // This should happen before registering on_set_parameters_callback,
  // else transport plugins will fail to declare their parameters
  if (options.use_intra_process_comms()) {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
  } else {
    camera_transport_pub_ = image_transport::create_camera_publisher(this, "image_raw");
  }

  compressed_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    "image_compressed", rclcpp::SensorDataQoS {});

  parameters_.declareStaticParameters();
  parameters_.declareOutputParameters();

  // Prepare camera
  camera_ = std::make_shared<V4l2CameraDevice>(parameters_.getVideoDevice());

  camera_->open();

  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_->getCameraName());

  parameters_.declareDeviceParameters(*camera_);

  // Read parameters and set up callback
  applyParameters();

  parameters_.setParameterChangedCallback(
    [this](rclcpp::Parameter parameter) {
      handleParameter(parameter);
    });

  // Start the camera
  camera_->start();

  // Start capture thread
  capture_thread_ = std::thread {std::bind(&V4L2Camera::captureThreadFunc, this)};
}

void V4L2Camera::captureThreadFunc()
{
  while (rclcpp::ok() && !canceled_.load()) {
    RCLCPP_DEBUG(get_logger(), "Capture...");

    try {
      auto captured_image = camera_->capture();
      auto const stamp = now();
      auto const image_encoding = imageEncodingString(captured_image.format.pixelFormat);

      std_msgs::msg::Header header;
      header.stamp = stamp;
      header.frame_id = camera_frame_id_;

      sensor_msgs::msg::Image::UniquePtr img;

      switch (imageEncodingType(captured_image.format.pixelFormat)) {
        case ImageEncodingType::raw:
          {
            img = std::make_unique<sensor_msgs::msg::Image>();
            img->header = header;
            img->encoding = image_encoding;
            img->width = captured_image.format.width;
            img->height = captured_image.format.height;
            img->step = captured_image.format.bytesPerLine;
            img->is_bigendian = std::endian::native == std::endian::big;
            img->data = std::move(captured_image.data);

            if (image_encoding != output_encoding_) {
              RCLCPP_WARN_STREAM_ONCE(
                get_logger(),
                "Image encoding not the same as requested output, "
                "performing possibly slow conversion: " <<
                  image_encoding << " => " << output_encoding_);
              img = convert(*img);
            }

            break;
          }
        case ImageEncodingType::compressed:
          {
            auto compressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
            compressed_img->header = header;
            compressed_img->format = image_encoding;
            compressed_img->data = std::move(captured_image.data);

            // decompress into raw image
            // if (pub_image->get_subscription_count())
            {
              img = std::make_unique<sensor_msgs::msg::Image>();
              cv_bridge::toCvCopy(*compressed_img, output_encoding_)->toImageMsg(*img);
              RCLCPP_INFO_STREAM_ONCE(
                get_logger(),
                "Decompressing " << image_encoding << " => " << output_encoding_);
            }

            compressed_image_pub_->publish(std::move(compressed_img));

            break;
          }
        default:
          {
            RCLCPP_ERROR_STREAM_ONCE(
              get_logger(),
              "Can't get image encoding type for " <<
                FourCC::toString(captured_image.format.pixelFormat));
            break;
          }
      }

      if (img) {
        auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
        if (!checkCameraInfo(*img, *ci)) {
          *ci = sensor_msgs::msg::CameraInfo{};
          ci->height = img->height;
          ci->width = img->width;
        }

        ci->header.stamp = stamp;
        ci->header.frame_id = camera_frame_id_;

        if (get_node_options().use_intra_process_comms()) {
          RCLCPP_DEBUG_STREAM(get_logger(), "Image message address [PUBLISH]:\t" << img.get());
          image_pub_->publish(std::move(img));
          info_pub_->publish(std::move(ci));
        } else {
          camera_transport_pub_.publish(*img, *ci);
        }
      }
    } catch (std::runtime_error const & e) {
      // Failed capturing image, assume it is temporarily and continue a bit later
      RCLCPP_ERROR_STREAM_THROTTLE(
        get_logger(), *get_clock(), 1000, "Image capture error: " << e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

V4L2Camera::~V4L2Camera()
{
  canceled_.store(true);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

void V4L2Camera::applyParameters()
{
  output_encoding_ = parameters_.getOutputEncoding();

  // Camera info parameters
  auto camera_info_url = parameters_.getCameraInfoUrl();
  if (camera_info_url != "") {
    if (cinfo_->validateURL(camera_info_url)) {
      cinfo_->loadCameraInfo(camera_info_url);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }
  }

  camera_frame_id_ = parameters_.getCameraFrameId();

  // Format parameters
  // Pixel format
  auto pixel_format = parameters_.getPixelFormat();
  requestPixelFormat(pixel_format);

  // Image size
  auto image_size = parameters_.getImageSize();
  requestImageSize(image_size);

  // Control parameters
  auto control_parameters = parameters_.getControlParameters();
  for (auto const & param : control_parameters) {
    auto control_id = parameters_.getControlId(param);
    auto control = camera_->queryControl(control_id);
    if (control.inactive) {
      RCLCPP_DEBUG(get_logger(), "Skipping inactive control: %s", control.name.c_str());
      continue;
    }

    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        if (static_cast<bool>(camera_->getControlValue(control.id)) == param.as_bool()) {continue;}
        camera_->setControlValue(control_id, param.as_bool());
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        if (camera_->getControlValue(control.id) == param.as_int()) {continue;}
        camera_->setControlValue(control_id, param.as_int());
        break;
      default:
        RCLCPP_WARN(
          get_logger(),
          "Control parameter type not currently supported: %d, for parameter: %s",
          unsigned(param.get_type()), param.get_name().c_str());
    }
  }
}

bool V4L2Camera::handleParameter(rclcpp::Parameter const & param)
{
  auto name = param.get_name();
  try {
    if (parameters_.isControlParameter(param)) {
      auto control_id = parameters_.getControlId(param);
      auto control = camera_->queryControl(control_id);
      if (control.inactive) {
        RCLCPP_WARN(get_logger(), "Cannot set inactive control: %s", control.name.c_str());
        return false;
      }
      switch (param.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          if (static_cast<bool>(camera_->getControlValue(control.id)) == param.as_bool()) {
            RCLCPP_DEBUG(
              get_logger(), "Parameter %s already set at requested value: %d",
              control.name.c_str(), param.as_bool());
            return true;
          }
          camera_->setControlValue(control_id, param.as_bool());
          return true;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          if (camera_->getControlValue(control.id) == param.as_int()) {
            RCLCPP_DEBUG(
              get_logger(), "Parameter %s already set at requested value: %ld",
              control.name.c_str(), param.as_int());
            return true;
          }
          camera_->setControlValue(control_id, param.as_int());
          return true;
        default:
          RCLCPP_WARN(
            get_logger(),
            "Control parameter type not currently supported: %s, for parameter: %s",
            std::to_string(unsigned(param.get_type())).c_str(), param.get_name().c_str());
      }
    } else if (param.get_name() == "output_encoding") {
      output_encoding_ = param.as_string();
      return true;
    } else if (param.get_name() == "pixel_format") {
      camera_->stop();
      requestPixelFormat(param.as_string());
      camera_->start();
      return true;
    } else if (param.get_name() == "image_size") {
      camera_->stop();
      requestImageSize(param.as_integer_array());
      camera_->start();
      return true;
    } else if (param.get_name() == "camera_info_url") {
      auto camera_info_url = param.as_string();
      if (cinfo_->validateURL(camera_info_url)) {
        return cinfo_->loadCameraInfo(camera_info_url);
      } else {
        RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
        return false;
      }
    }
  } catch (std::runtime_error const & e) {
    RCLCPP_ERROR_STREAM(get_logger(), e.what());
    return false;
  }

  return false;
}

void V4L2Camera::requestPixelFormat(std::string const & fourcc)
{
  if (fourcc.size() != 4) {
    throw std::invalid_argument {"Invalid pixel format size: must be a 4 character code (FOURCC)."};
  }

  auto code = v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given pixel format
  if (dataFormat.pixelFormat == code) {
    return;
  }

  dataFormat.pixelFormat = code;
  camera_->requestDataFormat(dataFormat);
}

void V4L2Camera::requestImageSize(std::vector<int64_t> const & size)
{
  if (size.size() != 2) {
    std::stringstream msg;
    msg << "Invalid image size; expected dimensions: 2, actual: " << size.size();
    throw std::invalid_argument {msg.str()};
  }

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given size
  if (dataFormat.width == size[0] && dataFormat.height == size[1]) {
    return;
  }

  dataFormat.width = size[0];
  dataFormat.height = size[1];
  camera_->requestDataFormat(dataFormat);
}

sensor_msgs::msg::Image::UniquePtr V4L2Camera::convert(sensor_msgs::msg::Image const & img) const
{
  auto tracked_object = std::shared_ptr<const void>{};
  auto cvImg = cv_bridge::toCvShare(img, tracked_object);
  auto outImg = std::make_unique<sensor_msgs::msg::Image>();
  auto cvConvertedImg = cv_bridge::cvtColor(cvImg, output_encoding_);
  cvConvertedImg->toImageMsg(*outImg);
  return outImg;
}

bool V4L2Camera::checkCameraInfo(
  sensor_msgs::msg::Image const & img,
  sensor_msgs::msg::CameraInfo const & ci)
{
  return ci.width == img.width && ci.height == img.height;
}

}  // namespace v4l2_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_camera::V4L2Camera)
