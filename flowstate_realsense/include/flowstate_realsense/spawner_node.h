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

#ifndef FLOWSTATE_REALSENSE_FLOWSTATE_REALSENSE_SPAWNER_NODE_H_
#define FLOWSTATE_REALSENSE_FLOWSTATE_REALSENSE_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "flowstate_common/base_adapter_node.h"
#include "flowstate_common/base_spawner_node.h"
#include "librealsense2/rs.hpp"

namespace flowstate_realsense {

// Discovers the RealSense cameras on the host's USB buses (librealsense's
// device list) and spawns one AdapterNode per serial. The driver type is
// "realsense": Flowstate addresses a camera's services as
// /realsense_<serial>/{describe,snapshot}. Given serials (the service
// config's), it serves only those cameras; empty serves every camera found.
class SpawnerNode : public flowstate_common::BaseSpawnerNode {
 public:
  explicit SpawnerNode(std::vector<std::string> serials = {});

 private:
  std::vector<std::string> GetSerials() override;
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> SpawnNodes(
      const std::vector<std::string>& serials) override;

  rs2::context context_;
  const std::vector<std::string> configured_serials_;
};

}  // namespace flowstate_realsense

#endif  // FLOWSTATE_REALSENSE_FLOWSTATE_REALSENSE_SPAWNER_NODE_H_
