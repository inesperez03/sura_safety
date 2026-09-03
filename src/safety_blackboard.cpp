#include "sura_safety/safety_blackboard.hpp"

#include <algorithm>
#include <cctype>
#include <memory>

#include "sura_safety/diagnostics_monitor.hpp"
#include "sura_safety/safety_diagnostic_nodes.hpp"

namespace sura_safety
{
namespace
{

std::string toUpper(std::string text)
{
  std::transform(
    text.begin(),
    text.end(),
    text.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); }
  );

  return text;
}

template<typename NodeT>
void registerNodeWithDescription(
  BT::BehaviorTreeFactory & factory,
  const std::string & node_id)
{
  factory.registerNodeType<NodeT>(node_id);
  factory.addDescriptionToManifest(node_id, NodeT::main_description());
}

}  // namespace

void registerSafetyNodes(BT::BehaviorTreeFactory & factory)
{
  registerNodeWithDescription<SafetyError>(
    factory, "SafetyError");

  registerNodeWithDescription<SafetyCriticalError>(
    factory, "SafetyCriticalError");

  registerNodeWithDescription<SafetyWarning>(
    factory, "SafetyWarning");

  registerNodeWithDescription<DiagnosticsUnavailableFor>(
    factory, "DiagnosticsUnavailableFor");

  registerNodeWithDescription<SafetyOk>(
    factory, "SafetyOk");

  registerNodeWithDescription<UpdateMissionControlFromSafety>(
    factory, "UpdateMissionControlFromSafety");
}

void configureSafetyBlackboard(
  const rclcpp::Node::SharedPtr & node,
  const BT::Blackboard::Ptr & blackboard,
  const std::string & robot_namespace,
  const std::string & diagnostics_topic)
{
  const std::string diagnostic_prefix = "/" + toUpper(robot_namespace);

  auto diagnostics_monitor =
    std::make_shared<DiagnosticsMonitor>(
      node,
      diagnostics_topic);

  blackboard->set("ros_node", node);
  blackboard->set("robot_namespace", robot_namespace);
  blackboard->set("diagnostic_prefix", diagnostic_prefix);
  blackboard->set("diagnostics_monitor", diagnostics_monitor);

  RCLCPP_INFO(
    node->get_logger(),
    "Configured sura_safety blackboard: robot_namespace='%s', diagnostic_prefix='%s'",
    robot_namespace.c_str(),
    diagnostic_prefix.c_str());
}

void configureSafety(
  const rclcpp::Node::SharedPtr & node,
  BT::BehaviorTreeFactory & factory,
  const BT::Blackboard::Ptr & blackboard,
  const std::string & robot_namespace,
  const std::string & diagnostics_topic)
{
  registerSafetyNodes(factory);
  configureSafetyBlackboard(
    node,
    blackboard,
    robot_namespace,
    diagnostics_topic);
}

}  // namespace sura_safety
