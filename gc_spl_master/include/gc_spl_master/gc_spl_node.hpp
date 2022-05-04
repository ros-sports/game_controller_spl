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

#ifndef GC_SPL_MASTER__GC_SPL_NODE_HPP_
#define GC_SPL_MASTER__GC_SPL_NODE_HPP_

#include <memory>

#include "rclcpp/node.hpp"
#include "rcgcd_14/msg/rcgcd.hpp"
#include "rcgcrd_4/msg/rcgcrd.hpp"

namespace gc_spl_master
{

class GCSPLNode : public rclcpp::Node
{
public:
  explicit GCSPLNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~GCSPLNode();

private:
  rclcpp::Publisher<rcgcd_14::msg::RCGCD>::SharedPtr rcgcdPub;
  rclcpp::Subscription<rcgcrd_4::msg::RCGCRD>::SharedPtr rcgcrdSub;
};

}  // namespace gc_spl_master

#endif  // GC_SPL_MASTER__GC_SPL_NODE_HPP_
