#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/blackboard.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace sura_safety
{

void registerSafetyNodes(BT::BehaviorTreeFactory & factory);

void configureSafetyBlackboard(
  const rclcpp::Node::SharedPtr & node,
  const BT::Blackboard::Ptr & blackboard,
  const std::string & robot_namespace,
  const std::string & diagnostics_topic = "/diagnostics_agg");

void configureSafety(
  const rclcpp::Node::SharedPtr & node,
  BT::BehaviorTreeFactory & factory,
  const BT::Blackboard::Ptr & blackboard,
  const std::string & robot_namespace,
  const std::string & diagnostics_topic = "/diagnostics_agg");

}  // namespace sura_safety
