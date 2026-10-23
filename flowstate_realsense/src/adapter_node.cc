// Copyright 2026 Google LLC
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

#include "flowstate_realsense/adapter_node.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "ament_index_cpp/get_resource.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"
#include "snapshot_interfaces/msg/sensor_info.hpp"

namespace flowstate_realsense {
namespace {

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using snapshot_interfaces::msg::ImageSnapshot;
using snapshot_interfaces::msg::SensorInfo;

// The vendor node's namespace; its name and camera_name are camera_<serial>
// (CameraName), so its topics read /realsense/camera_<serial>/<stream>/...
// and its frames camera_<serial>_<stream>_optical_frame. Each stream's frame
// is read from its images, as infra2's CameraInfo carries infra1's frame (the
// ROS stereo convention: the baseline in its P).
constexpr char kDriverNamespace[] = "/realsense";
constexpr char kDriverPackage[] = "realsense2_camera";
constexpr char kDriverClass[] = "realsense2_camera::RealSenseNodeFactory";

// The four streams a device can have, in the order the describe lists them
// (Flowstate's sensor ids). Topics are under the vendor node; Y8 infrared
// publishes as image_rect_raw. The vendor declares enable_parameter for each
// stream the device has, once it has opened it.
struct StreamSpec {
  const char* sensor_name;
  const char* image_topic;
  const char* info_topic;
  uint32_t sensor_type;
  const char* enable_parameter;
};
constexpr size_t kColor = 0;
constexpr size_t kDepth = 1;
constexpr std::array<StreamSpec, AdapterNode::kStreamCount> kStreams = {{
    {"rgb", "color/image_raw", "color/camera_info", SensorInfo::IMAGE,
     "enable_color"},
    {"depth", "aligned_depth_to_color/image_raw",
     "aligned_depth_to_color/camera_info", SensorInfo::DEPTH, "enable_depth"},
    {"ir_left", "infra1/image_rect_raw", "infra1/camera_info",
     SensorInfo::IMAGE, "enable_infra1"},
    {"ir_right", "infra2/image_rect_raw", "infra2/camera_info",
     SensorInfo::IMAGE, "enable_infra2"},
}};
// The depth module's own rectified depth, served on a device without color.
constexpr char kRawDepthImageTopic[] = "depth/image_rect_raw";
constexpr char kRawDepthInfoTopic[] = "depth/camera_info";
// Aligned depth needs it on. The vendor's filter setup declares it after every
// sensor has declared its enable_<stream>, so once it exists the set of
// enable_* parameters is the set of streams the device has.
constexpr char kAlignDepthParameter[] = "align_depth.enable";

// A snapshot waits this long for a frame of each served stream newer than the
// request; the driver streams at 30 Hz.
constexpr absl::Duration kImageDeadline = absl::Seconds(5);
// Without a frame of a served stream for this long the node tears itself
// down; the spawner re-creates it (and the vendor node with it) on its next
// pass.
constexpr double kLivenessTimeoutS = 30.0;
// The driver publishes depth as 16UC1 millimeters whatever the device's own
// depth unit (its fix_depth_scale); Flowstate wants 32FC1 meters.
constexpr float kMillimeter = 0.001f;

geometry_msgs::msg::TransformStamped Identity(const std::string& parent,
                                              const std::string& child) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.rotation.w = 1.0;
  return t;
}

absl::StatusOr<Image> Meters(const Image& depth_mm) {
  if (depth_mm.encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
    return absl::InvalidArgumentError(absl::StrCat(
        "depth encoding ", depth_mm.encoding, ", expected 16UC1 millimeters"));
  }
  Image depth;
  depth.header = depth_mm.header;
  depth.height = depth_mm.height;
  depth.width = depth_mm.width;
  depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth.is_bigendian = false;
  depth.step = sizeof(float) * depth_mm.width;
  depth.data.resize(depth.step * depth_mm.height);
  for (uint32_t row = 0; row < depth_mm.height; ++row) {
    const uint8_t* src = depth_mm.data.data() + row * depth_mm.step;
    float* dst = reinterpret_cast<float*>(depth.data.data() + row * depth.step);
    for (uint32_t col = 0; col < depth_mm.width; ++col) {
      uint16_t mm;
      std::memcpy(&mm, src + 2 * col, sizeof(mm));
      dst[col] = mm * kMillimeter;  // 0 (no measurement) stays 0
    }
  }
  return depth;
}

// The library path and class-loader name of the component `class_name`
// registered by `package`, from the ament resource index, as
// rclcpp_components::ComponentManager resolves them.
absl::StatusOr<std::string> ComponentLibrary(const std::string& package,
                                             const std::string& class_name) {
  std::string content;
  std::string prefix;
  if (!ament_index_cpp::get_resource("rclcpp_components", package, content,
                                     &prefix)) {
    return absl::NotFoundError(absl::StrCat(
        "package ", package, " registers no rclcpp components"));
  }
  for (absl::string_view line : absl::StrSplit(content, '\n')) {
    const std::vector<std::string> parts = absl::StrSplit(line, ';');
    if (parts.size() != 2 || parts[0] != class_name) continue;
    const std::string& path = parts[1];
    return path.empty() || path[0] == '/' ? path
                                          : absl::StrCat(prefix, "/", path);
  }
  return absl::NotFoundError(absl::StrCat("component ", class_name,
                                          " not registered by ", package));
}

}  // namespace

AdapterNode::AdapterNode(const std::string& serial)
    : flowstate_common::BaseAdapterNode(
          serial, /*locators=*/{}, "realsense",
          // The parameter services are the vendor node's (StartDriver).
          rclcpp::NodeOptions().use_intra_process_comms(true)
              .start_parameter_services(false)) {
  {
    const rclcpp::Time now = get_clock()->now();
    absl::MutexLock lock(&mutex_);
    t_started_ = now;
    t_settings_changed_ = now;
    for (StreamState& stream : streams_) stream.t_last_frame = now;
  }
  subscription_cb_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // The driver's static transforms, for the extrinsics.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (const absl::Status status = StartDriver(); !status.ok()) {
    // Without a driver no device is ever opened and the liveness check tears
    // the node down; the spawner retries.
    RCLCPP_ERROR_STREAM(get_logger(), "realsense2_camera not started: "
                                          << status);
  }

  // The subscriptions follow once DetectStreams knows the device's streams.
  CreateFlowstateServices();
  StartExecutorThread();
}

AdapterNode::~AdapterNode() {
  // This node's executor and thread first: Main() is still spinning when the
  // spawner deletes the node of an unplugged camera, and the members below
  // are what its callbacks read.
  executor_.cancel();
  if (thread_.joinable()) thread_.join();
  // The proxy and the callback reach into the vendor node: they go first.
  parameter_service_.reset();
  settings_changed_handle_.reset();
  vendor_.reset();
  if (driver_executor_) driver_executor_->cancel();
  if (driver_thread_.joinable()) driver_thread_.join();
  // The executor's entity collection holds weak references to the vendor
  // node's subscriptions and timers, whose control blocks the vendor library's
  // code made: released after the library is unloaded, they call into its
  // unmapped vtables (the container's segfault on every teardown).
  driver_executor_.reset();
  driver_.reset();
  driver_factory_.reset();
  driver_loader_.reset();
}

std::vector<rclcpp::Parameter> AdapterNode::StartupParameters() const {
  // The serial keeps a leading underscore: the driver strips it, and it
  // stops an all-digit string from becoming an integer parameter. camera_name
  // prefixes the driver's frame ids, which several cameras would otherwise
  // share on /tf_static. An enable_* for a stream the device lacks is never
  // declared, so its override is ignored. Y8 infrared is mono8 on
  // image_rect_raw (rectified); the static transforms stay on for the
  // extrinsics and anyone watching tf.
  return {
      rclcpp::Parameter("serial_no", absl::StrCat("_", serial_)),
      rclcpp::Parameter("camera_name", CameraName()),
      rclcpp::Parameter("enable_color", true),
      rclcpp::Parameter("enable_depth", true),
      rclcpp::Parameter("enable_infra1", true),
      rclcpp::Parameter("enable_infra2", true),
      rclcpp::Parameter("depth_module.infra1_format", std::string("Y8")),
      rclcpp::Parameter("depth_module.infra2_format", std::string("Y8")),
      rclcpp::Parameter("align_depth.enable", true),
      rclcpp::Parameter("publish_tf", true),
      rclcpp::Parameter("tf_publish_rate", 0.0),
  };
}

absl::Status AdapterNode::StartDriver() {
  absl::StatusOr<std::string> library =
      ComponentLibrary(kDriverPackage, kDriverClass);
  if (!library.ok()) return library.status();

  try {
    driver_loader_ = std::make_unique<class_loader::ClassLoader>(*library);
    const std::string factory_class = absl::StrCat(
        "rclcpp_components::NodeFactoryTemplate<", kDriverClass, ">");
    driver_factory_ =
        driver_loader_->createInstance<rclcpp_components::NodeFactory>(
            factory_class);
  } catch (const std::exception& e) {
    return absl::InternalError(
        absl::StrCat("loading ", *library, ": ", e.what()));
  }

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.arguments({"--ros-args", "-r", absl::StrCat("__node:=", CameraName()),
                     "-r", absl::StrCat("__ns:=", kDriverNamespace)});
  options.parameter_overrides(StartupParameters());
  try {
    driver_ = std::make_unique<rclcpp_components::NodeInstanceWrapper>(
        driver_factory_->create_node_instance(options));
  } catch (const std::exception& e) {
    return absl::InternalError(
        absl::StrCat("creating ", kDriverClass, ": ", e.what()));
  }

  // The factory derives from rclcpp::Node alone (realsense-ros built without
  // USE_LIFECYCLE_NODE, as the apt package is), so the instance is that Node:
  // checked against the wrapper's node base.
  vendor_ =
      std::static_pointer_cast<rclcpp::Node>(driver_->get_node_instance());
  if (vendor_->get_node_base_interface() !=
      driver_->get_node_base_interface()) {
    vendor_.reset();
    return absl::InternalError(
        absl::StrCat(kDriverClass, " is not an rclcpp::Node"));
  }
  // Flowstate's settings are parameter calls on this node: serve the vendor's.
  parameter_service_ = std::make_shared<rclcpp::ParameterService>(
      get_node_base_interface(), get_node_services_interface(),
      vendor_->get_node_parameters_interface().get());
  settings_changed_handle_ = vendor_->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        const bool restreams = std::any_of(
            parameters.begin(), parameters.end(),
            [](const rclcpp::Parameter& p) {
              return absl::EndsWith(p.get_name(), "_profile") ||
                     absl::EndsWith(p.get_name(), "_format");
            });
        const rclcpp::Time now = get_clock()->now();
        absl::MutexLock lock(&mutex_);
        t_settings_changed_ = now;
        restart_pending_ = restart_pending_ || restreams;
      });

  driver_executor_ =
      std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  driver_executor_->add_node(driver_->get_node_base_interface());
  driver_thread_ = std::thread([this]() { driver_executor_->spin(); });
  RCLCPP_INFO(get_logger(), "realsense2_camera running as %s/%s",
              kDriverNamespace, CameraName().c_str());
  return absl::OkStatus();
}

void AdapterNode::DetectStreams() {
  {
    absl::MutexLock lock(&mutex_);
    if (detected_) return;
  }
  if (!vendor_ || !vendor_->has_parameter(kAlignDepthParameter)) return;
  std::array<bool, kStreamCount> present;
  std::vector<std::string> names;
  for (size_t i = 0; i < kStreamCount; ++i) {
    present[i] = vendor_->has_parameter(kStreams[i].enable_parameter);
    if (present[i]) names.push_back(kStreams[i].sensor_name);
  }
  depth_aligned_ = present[kColor];
  for (size_t i = 0; i < kStreamCount; ++i) {
    if (present[i]) Subscribe(i);
  }
  RCLCPP_INFO(get_logger(), "device streams: %s%s",
              absl::StrJoin(names, ", ").c_str(),
              depth_aligned_ ? "" : " (no color: depth unaligned)");
  absl::MutexLock lock(&mutex_);
  present_ = present;
  detected_ = true;
}

void AdapterNode::Subscribe(size_t i) {
  rclcpp::SubscriptionOptions options;
  options.callback_group = subscription_cb_group_;
  const rclcpp::QoS qos = rclcpp::SensorDataQoS();
  subscriptions_.push_back(create_subscription<CameraInfo>(
      InfoTopic(i), qos,
      [this, i](CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&mutex_);
        streams_[i].info = std::move(msg);
      },
      options));
  subscriptions_.push_back(create_subscription<Image>(
      ImageTopic(i), qos,
      [this, i](Image::UniquePtr msg) {
        const rclcpp::Time now = get_clock()->now();
        absl::MutexLock lock(&mutex_);
        StreamState& stream = streams_[i];
        stream.image = std::move(msg);
        ++stream.frame_count;
        stream.t_last_frame = now;
      },
      options));
}

std::array<bool, AdapterNode::kStreamCount> AdapterNode::ServedStreams()
    const {
  std::array<bool, kStreamCount> present;
  {
    absl::MutexLock lock(&mutex_);
    if (!detected_) return {};
    present = present_;
  }
  auto on = [this](const char* name) {
    rclcpp::Parameter parameter;
    return vendor_ && vendor_->get_parameter(name, parameter) &&
           parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL &&
           parameter.as_bool();
  };
  std::array<bool, kStreamCount> served;
  for (size_t i = 0; i < kStreamCount; ++i) {
    served[i] = present[i] && on(kStreams[i].enable_parameter);
  }
  if (depth_aligned_) {
    served[kDepth] =
        served[kDepth] && served[kColor] && on(kAlignDepthParameter);
  }
  return served;
}

void AdapterNode::RestartStreamsIfPending() {
  {
    absl::MutexLock lock(&mutex_);
    if (!restart_pending_) return;
    restart_pending_ = false;
  }
  rclcpp::Parameter enable_color;
  if (!vendor_ || !vendor_->get_parameter("enable_color", enable_color)) return;
  const rcl_interfaces::msg::SetParametersResult result =
      vendor_->set_parameter(enable_color);
  if (!result.successful) {
    RCLCPP_ERROR(get_logger(), "restarting the streams failed: %s",
                 result.reason.c_str());
  }
}

std::string AdapterNode::CameraName() const {
  return absl::StrCat("camera_", serial_);
}

std::string AdapterNode::ColorImageTopic() const { return ImageTopic(kColor); }

std::string AdapterNode::ImageTopic(size_t stream) const {
  const char* topic = stream == kDepth && !depth_aligned_
                          ? kRawDepthImageTopic
                          : kStreams[stream].image_topic;
  return absl::StrFormat("%s/%s/%s", kDriverNamespace, CameraName(), topic);
}

std::string AdapterNode::InfoTopic(size_t stream) const {
  const char* topic = stream == kDepth && !depth_aligned_
                          ? kRawDepthInfoTopic
                          : kStreams[stream].info_topic;
  return absl::StrFormat("%s/%s/%s", kDriverNamespace, CameraName(), topic);
}

void AdapterNode::CacheExtrinsics() {
  if (!tf_buffer_) return;
  // The camera frame is the color optical frame (the depth optical frame on
  // a device without color) and each sensor's the one its images are stamped
  // in. Color and aligned depth are the camera frame itself; the frames still
  // to place are read under the lock, looked up outside it.
  std::string camera_frame;
  std::array<std::string, kStreamCount> frames;
  {
    absl::MutexLock lock(&mutex_);
    if (!detected_) return;
    const StreamState& reference = streams_[present_[kColor] ? kColor : kDepth];
    if (!reference.image) return;
    camera_frame = reference.image->header.frame_id;
    for (size_t i = 0; i < kStreamCount; ++i) {
      StreamState& stream = streams_[i];
      if (!present_[i] || stream.camera_t_sensor || !stream.image) continue;
      const std::string& frame = stream.image->header.frame_id;
      if (i == kColor || (i == kDepth && depth_aligned_) ||
          frame == camera_frame) {
        stream.camera_t_sensor = Identity(camera_frame, frame);
      } else {
        frames[i] = frame;
      }
    }
  }
  std::array<std::optional<geometry_msgs::msg::TransformStamped>, kStreamCount>
      found;
  for (size_t i = 0; i < kStreamCount; ++i) {
    if (frames[i].empty()) continue;
    try {
      found[i] = tf_buffer_->lookupTransform(camera_frame, frames[i],
                                             tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_DEBUG(get_logger(), "%s extrinsics not yet known: %s",
                   kStreams[i].sensor_name, e.what());
    }
  }
  bool all_known = true;
  {
    absl::MutexLock lock(&mutex_);
    for (size_t i = 0; i < kStreamCount; ++i) {
      StreamState& stream = streams_[i];
      if (found[i]) {
        const auto& t = found[i]->transform;
        RCLCPP_INFO(get_logger(),
                    "%s extrinsics: [%.4f, %.4f, %.4f], [%.4f, %.4f, %.4f, "
                    "%.4f]",
                    kStreams[i].sensor_name, t.translation.x, t.translation.y,
                    t.translation.z, t.rotation.x, t.rotation.y, t.rotation.z,
                    t.rotation.w);
        stream.camera_t_sensor = std::move(found[i]);
      }
      all_known =
          all_known && (!present_[i] || stream.camera_t_sensor.has_value());
    }
  }
  // Static for the device: the listener's thread is no longer needed.
  if (all_known) {
    tf_listener_.reset();
    tf_buffer_.reset();
  }
}

void AdapterNode::CheckLiveness() {
  const std::array<bool, kStreamCount> served = ServedStreams();
  const rclcpp::Time now = get_clock()->now();
  double silent_s = 0.0;
  std::string silence;
  {
    absl::MutexLock lock(&mutex_);
    if (!detected_) {
      // Nothing is served until the driver has opened the device.
      silent_s = (now - t_started_).seconds();
      silence = "no device opened by the driver";
    }
    for (size_t i = 0; i < kStreamCount; ++i) {
      StreamState& stream = streams_[i];
      if (served[i] && !stream.served) stream.t_last_frame = now;
      stream.served = served[i];
      if (!served[i]) continue;
      const double s = (now - stream.t_last_frame).seconds();
      if (s > silent_s) {
        silent_s = s;
        silence = absl::StrCat("no ", kStreams[i].sensor_name, " frame");
      }
    }
  }
  if (silent_s > kLivenessTimeoutS) {
    RCLCPP_ERROR(get_logger(),
                 "%s for %.0f s: tearing the node down, the spawner "
                 "re-creates it",
                 silence.c_str(), silent_s);
    executor_.cancel();
  }
}

absl::Status AdapterNode::Main() {
  liveness_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
    DetectStreams();
    RestartStreamsIfPending();
    CacheExtrinsics();
    CheckLiveness();
  });
  executor_.add_node(get_node_base_interface());
  executor_.spin();
  liveness_timer_.reset();
  RCLCPP_INFO(get_logger(), "exiting AdapterNode::Main()");
  return absl::OkStatus();
}

absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
AdapterNode::BuildDescribeResponse() {
  snapshot_interfaces::srv::Describe::Response response;
  absl::MutexLock lock(&mutex_);
  if (!detected_) {
    return absl::UnavailableError("the driver has not opened the device yet");
  }
  std::vector<std::string> pending;
  for (size_t i = 0; i < kStreamCount; ++i) {
    if (present_[i] && (!streams_[i].info || !streams_[i].camera_t_sensor)) {
      pending.push_back(kStreams[i].sensor_name);
    }
  }
  if (!pending.empty()) {
    return absl::UnavailableError(
        absl::StrCat("CameraInfo or extrinsics not yet received: ",
                     absl::StrJoin(pending, ", ")));
  }
  for (size_t i = 0; i < kStreamCount; ++i) {
    if (!present_[i]) continue;
    SensorInfo sensor =
        BuildSensorInformation(*streams_[i].info, kStreams[i].sensor_name,
                               ImageTopic(i), *streams_[i].camera_t_sensor);
    sensor.sensor_type = kStreams[i].sensor_type;
    response.sensors.push_back(std::move(sensor));
  }
  return response;
}

absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
AdapterNode::BuildSnapshotResponse() {
  // A profile set by this capture's settings: its frames, not the old ones.
  RestartStreamsIfPending();
  const std::array<bool, kStreamCount> served = ServedStreams();
  if (std::none_of(served.begin(), served.end(), [](bool on) { return on; })) {
    absl::MutexLock lock(&mutex_);
    if (!detected_) {
      return absl::UnavailableError(
          "the driver has not opened the device yet");
    }
    return absl::FailedPreconditionError(
        "no stream is on: every enable_* parameter is false");
  }

  snapshot_interfaces::srv::Snapshot::Response response;
  absl::MutexLock lock(&mutex_);
  std::array<uint64_t, kStreamCount> targets;
  for (size_t i = 0; i < kStreamCount; ++i) {
    targets[i] = streams_[i].frame_count;
  }
  // A frame of each served stream newer than the request and received after
  // the last settings change, both on this node's clock.
  auto fresh = [this, &targets](size_t i) {
    const StreamState& stream = streams_[i];
    return stream.info && stream.image && stream.frame_count > targets[i] &&
           stream.t_last_frame >= t_settings_changed_;
  };
  auto all_fresh = [&served, &fresh]() {
    for (size_t i = 0; i < kStreamCount; ++i) {
      if (served[i] && !fresh(i)) return false;
    }
    return true;
  };
  if (!mutex_.AwaitWithTimeout(absl::Condition(&all_fresh), kImageDeadline)) {
    std::vector<std::string> stale;
    for (size_t i = 0; i < kStreamCount; ++i) {
      if (served[i] && !fresh(i)) stale.push_back(kStreams[i].sensor_name);
    }
    return absl::DeadlineExceededError(absl::StrFormat(
        "no new %s frames within %s", absl::StrJoin(stale, ", "),
        absl::FormatDuration(kImageDeadline)));
  }

  for (size_t i = 0; i < kStreamCount; ++i) {
    if (!served[i]) continue;
    const StreamState& stream = streams_[i];
    ImageSnapshot snapshot;
    snapshot.topic_name = ImageTopic(i);
    snapshot.camera_info = *stream.info;
    if (i == kDepth) {
      absl::StatusOr<Image> depth = Meters(*stream.image);
      if (!depth.ok()) return depth.status();
      snapshot.image = *std::move(depth);
    } else {
      snapshot.image = *stream.image;
    }
    response.images.push_back(std::move(snapshot));
  }
  return response;
}

}  // namespace flowstate_realsense
