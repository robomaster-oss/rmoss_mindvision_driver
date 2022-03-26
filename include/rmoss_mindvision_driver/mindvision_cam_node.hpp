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

#ifndef RMOSS_MINDVISION_DRIVER__MINDVISION_CAM_NODE_HPP_
#define RMOSS_MINDVISION_DRIVER__MINDVISION_CAM_NODE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rmoss_cam/cam_server.hpp"
#include "rmoss_mindvision_driver/mindvision_cam.hpp"

namespace rmoss_mindvision_driver
{
// Node warpper for MindVisionCamera
class MindVisionCamNode
{
public:
  explicit MindVisionCamNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rmoss_mindvision_driver::MindVisionCam> cam_dev_;
  std::shared_ptr<rmoss_cam::CamServer> cam_server_;
};
}  // namespace rmoss_mindvision_driver

#endif  // RMOSS_MINDVISION_DRIVER__MINDVISION_CAM_NODE_HPP_
