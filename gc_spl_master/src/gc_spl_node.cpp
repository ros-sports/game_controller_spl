// Copyright 2022 Kenji Brameld
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

#include <memory>

#include "gc_spl_master/gc_spl_node.hpp"

namespace gc_spl_master
{

GCSPLNode::GCSPLNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"gc_spl", options}
{
  // // Declare parameters
  // RCLCPP_DEBUG(get_logger(), "Declare parameters");
  // std::string model = this->declare_parameter<std::string>("model", "rsg/agent/nao/nao.rsg");
  // std::string rcss3d_host = this->declare_parameter<std::string>("rcss3d/host", "127.0.0.1");
  // int rcss3d_port = this->declare_parameter<int>("rcss3d/port", 3100);
  // std::string team = this->declare_parameter<std::string>("team", "Anonymous");
  // int unum = this->declare_parameter<int>("unum", 0);

  // // Create Rcss3dAgent
  // params = std::make_unique<rcss3d_agent::Params>(model, rcss3d_host, rcss3d_port, team, unum);
  // rcss3dAgent = std::make_unique<rcss3d_agent::Rcss3dAgent>(*params);

  // // Create publisher
  // perceptPub =
  //   create_publisher<rcss3d_agent_msgs::msg::Percept>("percept", rclcpp::SensorDataQoS());

  // // Register callback
  // rcss3dAgent->registerPerceptCallback(
  //   [this](const rcss3d_agent_msgs::msg::Percept & percept) {
  //     perceptPub->publish(std::make_unique<rcss3d_agent_msgs::msg::Percept>(percept));
  //   });

  // // Subscriptions
  // hingeJointSub =
  //   create_subscription<rcss3d_agent_msgs::msg::HingeJointVel>(
  //   "effectors/hinge_joint", rclcpp::ServicesQoS(),
  //   [this](rcss3d_agent_msgs::msg::HingeJointVel::SharedPtr cmd) {
  //     rcss3dAgent->sendHingeJointVel(*cmd);
  //   });

  // universalJointSub =
  //   create_subscription<rcss3d_agent_msgs::msg::UniversalJointVel>(
  //   "effectors/universal_joint", rclcpp::ServicesQoS(),
  //   [this](rcss3d_agent_msgs::msg::UniversalJointVel::SharedPtr cmd) {
  //     rcss3dAgent->sendUniversalJointVel(*cmd);
  //   });

  // beamSub =
  //   create_subscription<rcss3d_agent_msgs::msg::Beam>(
  //   "effectors/beam", rclcpp::ServicesQoS(),
  //   [this](rcss3d_agent_msgs::msg::Beam::SharedPtr cmd) {
  //     rcss3dAgent->sendBeam(*cmd);
  //   });

  // saySub =
  //   create_subscription<rcss3d_agent_msgs::msg::Say>(
  //   "effectors/say", rclcpp::ServicesQoS(),
  //   [this](rcss3d_agent_msgs::msg::Say::SharedPtr cmd) {
  //     rcss3dAgent->sendSay(*cmd);
  //   });

  // synchronizeSub =
  //   create_subscription<rcss3d_agent_msgs::msg::Synchronize>(
  //   "effectors/synchronize", rclcpp::ServicesQoS(),
  //   [this](rcss3d_agent_msgs::msg::Synchronize::SharedPtr) {
  //     rcss3dAgent->sendSynchronize();
  //   });
}

GCSPLNode::~GCSPLNode()
{
}

}  // namespace gc_spl_master

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gc_spl_master::GCSPLNode)
