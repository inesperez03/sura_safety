#include "sura_safety/safety_blackboard.hpp"

#include <memory>

#include "sura_safety/diagnostics_monitor.hpp"
#include "sura_safety/safety_diagnostic_nodes.hpp"

namespace sura_safety
{
namespace
{

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

  registerNodeWithDescription<SafetyErrorAsk>(
    factory, "SafetyErrorAsk");

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
  const double timeout = node->has_parameter("safety.diagnostics_timeout") ?
    node->get_parameter("safety.diagnostics_timeout").as_double() :
    node->declare_parameter<double>("safety.diagnostics_timeout", 5.0);

  auto diagnostics_monitor =
    std::make_shared<DiagnosticsMonitor>(
      node,
      diagnostics_topic, robot_namespace, timeout);

  blackboard->set("ros_node", node);
  blackboard->set("robot_namespace", robot_namespace);
  blackboard->set("diagnostics_monitor", diagnostics_monitor);

  RCLCPP_INFO(
    node->get_logger(),
    "Configured sura_safety blackboard: robot_namespace='%s', diagnostics_topic='%s'",
    robot_namespace.c_str(),
    diagnostics_topic.c_str());
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
