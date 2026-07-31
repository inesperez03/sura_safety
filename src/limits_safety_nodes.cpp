#include "sura_safety/limits_safety_nodes.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <vector>

namespace sura_safety
{
namespace
{

std::string trim(std::string value)
{
  const auto not_space = [](unsigned char c) {
    return !std::isspace(c);
  };

  value.erase(
    value.begin(),
    std::find_if(value.begin(), value.end(), not_space));
  value.erase(
    std::find_if(value.rbegin(), value.rend(), not_space).base(),
    value.end());
  return value;
}

std::vector<std::string> splitCommaList(const std::string & text)
{
  std::vector<std::string> values;
  std::stringstream stream(text);
  std::string item;

  while (std::getline(stream, item, ','))
  {
    item = trim(item);
    if (!item.empty())
    {
      values.push_back(item);
    }
  }

  return values;
}

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

std::string aggregatedDiagnosticName(
  const BT::NodeConfiguration & config,
  const std::string & diagnostic_group,
  const std::string & diagnostic_path)
{
  const std::string diagnostic_prefix =
    config.blackboard->get<std::string>("diagnostic_prefix");
  const auto group = trim(diagnostic_group);
  const auto path = trim(diagnostic_path);

  if (path.empty())
  {
    return diagnostic_prefix;
  }

  if (!group.empty() && path.front() != '/')
  {
    return diagnostic_prefix + "/" + group + "/ " + group + " " + path;
  }

  if (path.rfind(diagnostic_prefix + "/", 0) == 0)
  {
    return path;
  }

  std::string normalized_path = path;
  if (normalized_path.front() != '/')
  {
    normalized_path = "/" + normalized_path;
  }

  const auto slash_pos = normalized_path.find('/', 1);
  if (slash_pos == std::string::npos)
  {
    return diagnostic_prefix + normalized_path;
  }

  const auto path_group = normalized_path.substr(1, slash_pos - 1);
  const auto item = normalized_path.substr(slash_pos + 1);
  return diagnostic_prefix + "/" + path_group + "/ " + path_group + " " + item;
}

bool isSensorUnavailable(
  const DiagnosticsMonitor & monitor,
  const std::string & diagnostic_name)
{
  const auto status = monitor.getStatus(diagnostic_name);
  if (!status)
  {
    return true;
  }

  return status->level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ||
         status->level == diagnostic_msgs::msg::DiagnosticStatus::STALE;
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

bool statusIsErrorForNode(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & node_name)
{
  return status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ||
         (staleCountsAsError(node_name) &&
          status.level == diagnostic_msgs::msg::DiagnosticStatus::STALE);
}

bool nodeHasSafetyError(
  const DiagnosticsMonitor & monitor,
  const BT::NodeConfiguration & config,
  const std::string & node_name)
{
  const auto diagnostic_name = diagnosticNameFromNodeName(config, node_name);
  const auto status = monitor.getStatus(diagnostic_name);
  return status && statusIsErrorForNode(*status, node_name);
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
  const bool safety_error = status && statusIsErrorForNode(*status, name());

  if (safety_error)
  {
    config().blackboard->set("mission_control", std::string{"pause"});

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
// SafetyCriticalError
// =======================================================

SafetyCriticalError::SafetyCriticalError(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SafetyCriticalError::providedPorts()
{
  return {};
}

BT::NodeStatus SafetyCriticalError::tick()
{
  const auto monitor = getMonitor(config());
  const auto ros_node = getRosNode(config());
  const auto diagnostic_name = diagnosticNameFromNodeName(config(), name());
  const auto status = monitor->getStatus(diagnostic_name);
  const bool safety_error = status && statusIsErrorForNode(*status, name());

  if (safety_error)
  {
    config().blackboard->set("mission_control", std::string{"abort"});
    config().blackboard->set("mission_state", std::string{"aborted"});

    const auto value_summary =
      safetyValueSummary(*monitor, diagnostic_name);

    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_safety] Safety CRITICAL ERROR/STALE. diagnostic=%s %s",
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
// DiagnosticsUnavailableFor
// =======================================================

DiagnosticsUnavailableFor::DiagnosticsUnavailableFor(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList DiagnosticsUnavailableFor::providedPorts()
{
  return {
    BT::InputPort<std::string>("diagnostics"),
    BT::InputPort<double>("seconds")
  };
}

BT::NodeStatus DiagnosticsUnavailableFor::tick()
{
  const auto monitor = getMonitor(config());
  const auto ros_node = getRosNode(config());
  auto diagnostics_text = getInput<std::string>("diagnostics");
  auto seconds = getInput<double>("seconds");

  if (!diagnostics_text)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_safety] DiagnosticsUnavailableFor requires diagnostics input");
    condition_active_ = false;
    return BT::NodeStatus::FAILURE;
  }

  const auto diagnostics = splitCommaList(diagnostics_text.value());
  if (diagnostics.empty())
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_safety] DiagnosticsUnavailableFor received empty diagnostics input");
    condition_active_ = false;
    return BT::NodeStatus::FAILURE;
  }

  bool all_unavailable = true;
  std::string unavailable_summary;
  for (const auto & diagnostic : diagnostics)
  {
    const auto diagnostic_name = aggregatedDiagnosticName(
      config(),
      "Sensors",
      diagnostic);
    if (!isSensorUnavailable(*monitor, diagnostic_name))
    {
      all_unavailable = false;
      break;
    }

    if (!unavailable_summary.empty())
    {
      unavailable_summary += ",";
    }
    unavailable_summary += diagnostic;
  }

  if (!all_unavailable)
  {
    condition_active_ = false;
    reported_ = false;
    return BT::NodeStatus::FAILURE;
  }

  const auto now = ros_node->now();
  if (!condition_active_)
  {
    condition_active_ = true;
    condition_start_time_ = now;
    return BT::NodeStatus::FAILURE;
  }

  const double timeout = seconds && seconds.value() > 0.0 ?
    seconds.value() : 10.0;
  const double elapsed = (now - condition_start_time_).seconds();
  if (elapsed < timeout)
  {
    return BT::NodeStatus::FAILURE;
  }

  if (!reported_)
  {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_safety] Safety ERROR. sensors unavailable for %.3fs: %s",
      elapsed,
      unavailable_summary.c_str());
    reported_ = true;
  }
  return BT::NodeStatus::SUCCESS;
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


// =======================================================
// UpdateMissionControlFromSafety
// =======================================================

UpdateMissionControlFromSafety::UpdateMissionControlFromSafety(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList UpdateMissionControlFromSafety::providedPorts()
{
  return {};
}

BT::NodeStatus UpdateMissionControlFromSafety::tick()
{
  const auto monitor = getMonitor(config());

  const std::vector<std::string> fatal_nodes = {
    "leak_safety_error_condition",
    "battery_safety_error_condition",
    "localization_safety_error_condition",
    "imu_safety_error_condition"
  };
  for (const auto & node_name : fatal_nodes)
  {
    if (nodeHasSafetyError(*monitor, config(), node_name))
    {
      config().blackboard->set("mission_control", std::string{"abort"});
      config().blackboard->set("mission_state", std::string{"aborted"});
      return BT::NodeStatus::SUCCESS;
    }
  }

  const std::vector<std::string> recoverable_nodes = {
    "depth_safety_error_condition",
    "altitude_safety_error_condition",
    "area_safety_error_condition"
  };
  for (const auto & node_name : recoverable_nodes)
  {
    if (nodeHasSafetyError(*monitor, config(), node_name))
    {
      config().blackboard->set("mission_control", std::string{"pause"});
      return BT::NodeStatus::SUCCESS;
    }
  }

  config().blackboard->set("mission_control", std::string{"run"});
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sura_safety
