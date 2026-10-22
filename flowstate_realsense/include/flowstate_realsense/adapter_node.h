/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FLOWSTATE_REALSENSE_FLOWSTATE_REALSENSE_ADAPTER_NODE_H_
#define FLOWSTATE_REALSENSE_FLOWSTATE_REALSENSE_ADAPTER_NODE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "class_loader/class_loader.hpp"
#include "flowstate_common/base_adapter_node.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rclcpp_components/node_instance_wrapper.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace flowstate_realsense {

// One RealSense camera for Flowstate: realsense2_camera's node composed
// in-process (loaded as the rclcpp component it registers, so none of the
// vendor's headers are needed here; namespace /realsense, name and
// camera_name camera_<serial>), its color, aligned-depth and two infrared
// topics cached here, and the Flowstate describe and snapshot services on this
// node, named realsense_<serial>.
//
// The camera's settings are the vendor node's parameters: this node's
// parameter services serve the vendor node's parameters, so a Flowstate
// CameraSetting names a realsense2_camera parameter verbatim (enable_infra1,
// depth_module.exposure, ...), with the vendor's types, ranges and live values.
//
// Up to four sensors, described in this fixed order (Flowstate numbers them
// 1..N): rgb, depth (converted from the driver's 16-bit millimeters to 32-bit
// float meters, the one depth encoding Flowstate's ROS image source accepts),
// ir_left and ir_right (Y8, mono8). Which of them the device has is read from
// the parameters the vendor declares once it has opened the device; a stream
// the device lacks is neither described nor served. The camera frame is the
// color optical frame and depth is served aligned to it; a device without
// color serves the depth module's rectified depth in its own frame, which is
// then the camera frame. Color and aligned depth report camera_t_sensor =
// identity, the other sensors their static transforms. Every stream starts on,
// so the describe Flowstate takes when it opens the camera holds all the device
// has; a snapshot returns those the vendor's enable_* parameters leave on.
class AdapterNode : public flowstate_common::BaseAdapterNode {
 public:
  explicit AdapterNode(const std::string& serial);
  ~AdapterNode() override;

  static constexpr size_t kStreamCount = 4;

 private:
  // One stream's latest messages. The CameraInfo and camera_t_sensor stay
  // after the stream is turned off, so the describe keeps every sensor the
  // device has.
  struct StreamState {
    std::unique_ptr<sensor_msgs::msg::CameraInfo> info;
    std::unique_ptr<sensor_msgs::msg::Image> image;
    std::optional<geometry_msgs::msg::TransformStamped> camera_t_sensor;
    // A snapshot waits for frames counted after the request. t_last_frame is
    // the latest image's arrival on this node's clock: the liveness check and
    // the settings gate read it, never the vendor's header stamps, which map
    // the camera's clock onto a time base set once at the first frame.
    uint64_t frame_count = 0;
    rclcpp::Time t_last_frame;
    // Whether the stream was served at the last liveness check: its silence
    // counts from the moment it was turned (back) on.
    bool served = false;
  };

  absl::Status Main() override ABSL_LOCKS_EXCLUDED(mutex_);
  std::string ColorImageTopic() const override;
  std::string ImageTopic(size_t stream) const;
  std::string InfoTopic(size_t stream) const;
  std::string CameraName() const;
  absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
  BuildDescribeResponse() override ABSL_LOCKS_EXCLUDED(mutex_);
  absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
  BuildSnapshotResponse() override ABSL_LOCKS_EXCLUDED(mutex_);

  // The parameters realsense2_camera starts with: this serial, every stream
  // on (Y8 infrared), depth aligned to color, static transforms only.
  std::vector<rclcpp::Parameter> StartupParameters() const;
  // Loads realsense2_camera's component for this serial, spins it on its own
  // executor thread and serves its parameters on this node.
  absl::Status StartDriver();
  // Reads which streams the device has from the parameters the vendor declares
  // once it has opened the device, and subscribes to them. Housekeeping, until
  // it has run once.
  void DetectStreams() ABSL_LOCKS_EXCLUDED(mutex_);
  void Subscribe(size_t stream);
  // Which streams the device has and the vendor's parameters leave on, in
  // sensor order; none before DetectStreams has run. Aligned depth also needs
  // enable_color and align_depth.enable. Reads the vendor's parameters outside
  // mutex_, which the vendor's post-set callback takes.
  std::array<bool, kStreamCount> ServedStreams() const
      ABSL_LOCKS_EXCLUDED(mutex_);
  // realsense-ros only stores a changed profile or format until the stream
  // restarts: after one, re-sets enable_color to its own value, which makes
  // the driver restart every sensor whose profiles no longer match. Called
  // before each snapshot and by the housekeeping timer.
  void RestartStreamsIfPending() ABSL_LOCKS_EXCLUDED(mutex_);
  // The 1 s housekeeping: camera_t_sensor for each stream whose CameraInfo
  // has arrived, and the liveness check over the served streams.
  void CacheExtrinsics() ABSL_LOCKS_EXCLUDED(mutex_);
  void CheckLiveness() ABSL_LOCKS_EXCLUDED(mutex_);

  // This node's executor, run by Main() on the base class's thread. A member
  // so the destructor can cancel it: the spawner also deletes a node whose
  // camera was unplugged while Main() still spins, and the base class joins
  // the thread without cancelling anything. Four threads: the two services,
  // the subscriptions and the default group (the parameter services and the
  // housekeeping timer) share them.
  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(),
                                                     4};

  // The composed vendor node: the loader must outlive the factory, which must
  // outlive the instance and its executor, which holds weak references to the
  // instance's entities (whose control blocks are the library's code); the
  // destructor releases them in that order.
  std::unique_ptr<class_loader::ClassLoader> driver_loader_;
  std::shared_ptr<rclcpp_components::NodeFactory> driver_factory_;
  std::unique_ptr<rclcpp_components::NodeInstanceWrapper> driver_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> driver_executor_;
  std::thread driver_thread_;
  // The vendor node as an rclcpp::Node, its parameters served as this node's
  // (the proxy holds the vendor's parameters interface), and the callback that
  // stamps each committed change of them.
  std::shared_ptr<rclcpp::Node> vendor_;
  std::shared_ptr<rclcpp::ParameterService> parameter_service_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
      settings_changed_handle_;

  // The static transforms realsense2_camera publishes for its active streams,
  // until both infrared extrinsics are cached.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr subscription_cb_group_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  std::array<StreamState, kStreamCount> streams_ ABSL_GUARDED_BY(mutex_);
  // The streams the device has (DetectStreams); nothing is served or described
  // before they are known. depth_aligned_ is written there, before detected_,
  // and only read afterwards: depth is aligned to color when the device has
  // color, else the depth module's own rectified depth in its own frame.
  bool detected_ ABSL_GUARDED_BY(mutex_) = false;
  std::array<bool, kStreamCount> present_ ABSL_GUARDED_BY(mutex_) = {};
  bool depth_aligned_ = true;
  // Construction time: the liveness check counts the driver's silence from it
  // until the device is open.
  rclcpp::Time t_started_;
  // The last committed change of the vendor's parameters: a snapshot skips
  // frames received before it (exposed under the previous settings).
  rclcpp::Time t_settings_changed_ ABSL_GUARDED_BY(mutex_);
  // A profile or format changed since the streams last restarted.
  bool restart_pending_ ABSL_GUARDED_BY(mutex_) = false;
};

}  // namespace flowstate_realsense

#endif  // FLOWSTATE_REALSENSE_FLOWSTATE_REALSENSE_ADAPTER_NODE_H_
