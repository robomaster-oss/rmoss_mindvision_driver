// Copyright 2022 robomaster-oss.
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

#include "rmoss_mindvision_driver/mindvision_cam_node.hpp"

#include <memory>
#include <string>

#include <rmoss_util/url_resolver.hpp>

namespace rmoss_mindvision_driver
{
MindVisionCamNode::MindVisionCamNode(
  const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("mindvision_cam", options);

  node_->declare_parameter("sn", "");
  node_->declare_parameter("config_path", "");

  std::string cam_sn = node_->get_parameter("sn").as_string();
  std::string config_path = node_->get_parameter("config_path").as_string();

  std::string config_path_resolved = rmoss_util::URLResolver::get_resolved_path(config_path);
  if (config_path_resolved != "") {
    config_path = config_path_resolved;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Try to open camera at sn:%s", cam_sn.c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "Camera config file path:%s", config_path.c_str());

  cam_dev_ = std::make_shared<MindVisionCam>(cam_sn, node_, config_path);

  if (config_path == "") {
    if (!cam_dev_->open()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Try to open camera failed!");
      return;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Writing camera config to file.");
    if (!cam_dev_->save_config(cam_sn + ".config")) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Write failed!");
      return;
    }
    cam_dev_->close();
  }

  // create server
  cam_server_ = std::make_shared<rmoss_cam::CamServer>(this->node_, this->cam_dev_);
}
}  // namespace rmoss_mindvision_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmoss_mindvision_driver::MindVisionCamNode)
