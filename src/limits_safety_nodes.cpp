#include "sura_safety/limits_safety_nodes.hpp"

namespace sura_safety
{
namespace
{

void appendSummaryValue(
  std::string & summary,
  const std::string & key,
  const std::string & value)
{
  if (value.empty())
  {
    return;
  }

  if (!summary.empty())
  {
    summary += " ";
  }

  summary += key + "=" + value;
}

std::string safetyValueSummary(
  const DiagnosticsMonitor & monitor,
  const std::string & diagnostic_name)
{
  std::string summary;

  appendSummaryValue(
    summary, "current_depth",
    monitor.getValue(diagnostic_name, "current_depth"));
  appendSummaryValue(
    summary, "max_depth",
    monitor.getValue(diagnostic_name, "max_depth"));
  appendSummaryValue(
    summary, "remaining_depth_margin",
    monitor.getValue(diagnostic_name, "remaining_depth_margin"));
  appendSummaryValue(
    summary, "current_altitude",
    monitor.getValue(diagnostic_name, "current_altitude"));
  appendSummaryValue(
    summary, "min_altitude",
    monitor.getValue(diagnostic_name, "min_altitude"));
  appendSummaryValue(
    summary, "remaining_altitude_margin",
    monitor.getValue(diagnostic_name, "remaining_altitude_margin"));
  appendSummaryValue(
    summary, "current_x",
    monitor.getValue(diagnostic_name, "current_x"));
  appendSummaryValue(
    summary, "current_y",
    monitor.getValue(diagnostic_name, "current_y"));
  appendSummaryValue(
    summary, "leak_detected",
    monitor.getValue(diagnostic_name, "leak_detected"));
  appendSummaryValue(
    summary, "has_state",
    monitor.getValue(diagnostic_name, "has_state"));
  appendSummaryValue(
    summary, "age_seconds",
    monitor.getValue(diagnostic_name, "age_seconds"));
  appendSummaryValue(
    summary, "stale_timeout",
    monitor.getValue(diagnostic_name, "stale_timeout"));
  appendSummaryValue(
    summary, "voltage",
    monitor.getValue(diagnostic_name, "voltage"));
  appendSummaryValue(
    summary, "current_draw_a",
    monitor.getValue(diagnostic_name, "current_draw_a"));
  appendSummaryValue(
    summary, "percentage",
    monitor.getValue(diagnostic_name, "percentage"));
  appendSummaryValue(
    summary, "present",
    monitor.getValue(diagnostic_name, "present"));
  appendSummaryValue(
    summary, "frequency_hz",
    monitor.getValue(diagnostic_name, "frequency_hz"));
  appendSummaryValue(
    summary, "sample_count",
    monitor.getValue(diagnostic_name, "sample_count"));
  appendSummaryValue(
    summary, "warn_min_frequency_hz",
    monitor.getValue(diagnostic_name, "warn_min_frequency_hz"));
  appendSummaryValue(
    summary, "error_min_frequency_hz",
    monitor.getValue(diagnostic_name, "error_min_frequency_hz"));

  if (summary.empty())
  {
    return "values=unknown";
  }

  return summary;
}

std::string diagnosticName(
  const BT::NodeConfiguration & config,
  const std::string & diagnostic_suffix)
{
  const std::string diagnostic_prefix =
    config.blackboard->get<std::string>("diagnostic_prefix");

  return diagnostic_prefix + "/Navigation/ Navigation " + diagnostic_suffix;
}

std::string diagnosticNameFromNodeName(
  const BT::NodeConfiguration & config,
  const std::string & node_name)
{
  const std::string diagnostic_prefix =
    config.blackboard->get<std::string>("diagnostic_prefix");

  if (node_name.find("leak") != std::string::npos)
  {
    return diagnostic_prefix + "/Sensors/ Sensors LeakSensors";
  }

  if (node_name.find("battery") != std::string::npos)
  {
    return diagnostic_prefix + "/Sensors/ Sensors Battery";
  }

  if (node_name.find("imu") != std::string::npos)
  {
    return diagnostic_prefix + "/Sensors/ Sensors IMU";
  }

  if (node_name.find("localization") != std::string::npos)
  {
    return diagnosticName(config, "Frequency");
  }

  if (node_name.find("altitude") != std::string::npos)
  {
    return diagnosticName(config, "AltitudeLimit");
  }

  if (node_name.find("area") != std::string::npos)
  {
    return diagnosticName(config, "AreaLimit");
  }

  return diagnosticName(config, "DepthLimit");
}

bool staleCountsAsError(const std::string & node_name)
{
  return node_name.find("leak") != std::string::npos ||
         node_name.find("battery") != std::string::npos ||
         node_name.find("imu") != std::string::npos ||
         node_name.find("localization") != std::string::npos;
}

BT::NodeStatus safetyWarningTick(
  const BT::NodeConfiguration & config,
  const std::string & diagnostic_suffix)
{
  const auto monitor =
    config.blackboard->get<std::shared_ptr<DiagnosticsMonitor>>(
      "diagnostics_monitor");
  const auto ros_node =
    config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  const auto name = diagnosticName(config, diagnostic_suffix);

  if (monitor->isWarn(name))
  {
    const auto value_summary = safetyValueSummary(*monitor, name);

    RCLCPP_WARN(
      ros_node->get_logger(),
      "[sura_safety] Safety WARNING. diagnostic=%s %s",
      name.c_str(),
      value_summary.c_str());

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

std::string warningDiagnosticSuffixFromNodeName(const std::string & node_name)
{
  if (node_name.find("altitude") != std::string::npos)
  {
    return "AltitudeLimit";
  }

  if (node_name.find("area") != std::string::npos)
  {
    return "AreaLimit";
  }

  return "DepthLimit";
}

}  // namespace

std::shared_ptr<DiagnosticsMonitor> LimitsSafetyBase::getMonitor(
  const BT::NodeConfiguration & config)
{
  return config.blackboard->get<std::shared_ptr<DiagnosticsMonitor>>(
    "diagnostics_monitor");
}

rclcpp::Node::SharedPtr LimitsSafetyBase::getRosNode(
  const BT::NodeConfiguration & config)
{
  return config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
}

std::string LimitsSafetyBase::getDepthDiagnosticName(
  const BT::NodeConfiguration & config)
{
  const std::string diagnostic_prefix =
    config.blackboard->get<std::string>("diagnostic_prefix");

  return diagnostic_prefix + "/Navigation/ Navigation DepthLimit";
}


// =======================================================
// SafetyError
// =======================================================

SafetyError::SafetyError(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SafetyError::providedPorts()
{
  return {};
}

BT::NodeStatus SafetyError::tick()
{
  const auto monitor = getMonitor(config());
  const auto ros_node = getRosNode(config());
  const auto diagnostic_name = diagnosticNameFromNodeName(config(), name());
  const auto status = monitor->getStatus(diagnostic_name);
  const bool safety_error = status &&
    (status->level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ||
     (staleCountsAsError(name()) &&
      status->level == diagnostic_msgs::msg::DiagnosticStatus::STALE));

  if (safety_error)
  {
    const auto value_summary =
      safetyValueSummary(*monitor, diagnostic_name);

    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_safety] Safety ERROR/STALE. diagnostic=%s %s",
      diagnostic_name.c_str(),
      value_summary.c_str());

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}


// =======================================================
// SafetyWarning
// =======================================================

SafetyWarning::SafetyWarning(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SafetyWarning::providedPorts()
{
  return {};
}

BT::NodeStatus SafetyWarning::tick()
{
  return safetyWarningTick(
    config(),
    warningDiagnosticSuffixFromNodeName(name()));
}


// =======================================================
// SafetyOk
// =======================================================

SafetyOk::SafetyOk(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SafetyOk::providedPorts()
{
  return {};
}

BT::NodeStatus SafetyOk::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sura_safety
