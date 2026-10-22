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

#include "flowstate_realsense/spawner_node.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_join.h"
#include "flowstate_realsense/adapter_node.h"
#include "librealsense2/rs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_realsense {

SpawnerNode::SpawnerNode(std::vector<std::string> serials)
    : flowstate_common::BaseSpawnerNode("realsense_spawner", "realsense",
                                        std::chrono::seconds(10)),
      configured_serials_(std::move(serials)) {
  if (!configured_serials_.empty()) {
    RCLCPP_INFO(get_logger(), "Serving only the RealSense cameras %s",
                absl::StrJoin(configured_serials_, ", ").c_str());
  }
}

std::vector<std::string> SpawnerNode::GetSerials() {
  std::vector<std::string> serials;
  try {
    for (const rs2::device& device : context_.query_devices()) {
      if (!device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) continue;
      std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      if (!configured_serials_.empty() &&
          std::find(configured_serials_.begin(), configured_serials_.end(),
                    serial) == configured_serials_.end()) {
        continue;
      }
      serials.push_back(std::move(serial));
    }
  } catch (const rs2::error& e) {
    RCLCPP_ERROR(get_logger(), "librealsense device query failed: %s",
                 e.what());
  }
  return serials;
}

std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>>
SpawnerNode::SpawnNodes(const std::vector<std::string>& serials) {
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> new_nodes;
  for (const std::string& serial : serials) {
    RCLCPP_INFO(get_logger(), "Spawning RealSense node for serial %s",
                serial.c_str());
    new_nodes.push_back(std::make_shared<AdapterNode>(serial));
  }
  return new_nodes;
}

}  // namespace flowstate_realsense
