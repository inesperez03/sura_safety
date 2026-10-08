#include "sura_safety/safety_diagnostic_nodes.hpp"
#include "sura_safety/safety_ask_state.hpp"
#include "sura_safety/critical_safety_state.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <sstream>
#include <vector>

namespace sura_safety
{
namespace
{

std::string trim(std::string value) // remove spaces from the beginning and end of a string
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

BT::PortsList diagnosticPorts()
{
  return {
    BT::InputPort<std::string>("diagnostic", "Source identity, relative to the robot or absolute."),
    BT::InputPort<bool>("stale_is_error", false, "Treat a STALE diagnostic as an error.")
  };
}

std::string requiredDiagnostic(const BT::TreeNode & node)
{
  const auto input = node.getInput<std::string>("diagnostic");
  if (!input || trim(input.value()).empty())
  {
    throw BT::RuntimeError("Missing diagnostic identity for ", node.name());
  }
  return trim(input.value());
}

bool statusIsError(
  const diagnostic_msgs::msg::DiagnosticStatus & status, bool stale_is_error)
{
  return status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ||
    (stale_is_error && status.level == diagnostic_msgs::msg::DiagnosticStatus::STALE);
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

BT::NodeStatus safetyWarningTick(
  const BT::NodeConfiguration & config,
  const std::string & diagnostic_name)
{
  const auto monitor =
    config.blackboard->get<std::shared_ptr<DiagnosticsMonitor>>(
      "diagnostics_monitor");
  const auto ros_node =
    config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  const auto & name = diagnostic_name;

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
  return diagnosticPorts();
}

const char * SafetyError::main_description()
{
  return "Detects a recoverable safety error and requests mission pause.";
}

BT::NodeStatus SafetyError::tick()
{
  const auto monitor = getMonitor(config());
  const auto ros_node = getRosNode(config());
  const auto diagnostic_name = requiredDiagnostic(*this);
  const auto status = monitor->getStatus(diagnostic_name);
  const bool safety_error = status && statusIsError(*status, getInput<bool>("stale_is_error").value());

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
  return diagnosticPorts();
}

const char * SafetyCriticalError::main_description()
{
  return "Detects a critical safety error and requests mission abort.";
}

BT::NodeStatus SafetyCriticalError::tick()
{
  const auto monitor = getMonitor(config());
  const auto ros_node = getRosNode(config());
  const auto diagnostic_name = requiredDiagnostic(*this);
  const auto status = monitor->getStatus(diagnostic_name);
  const bool safety_error = status && statusIsError(*status, getInput<bool>("stale_is_error").value());
  std::shared_ptr<CriticalSafetyState> critical_state;
  config().blackboard->get("critical_safety_state", critical_state);

  if (safety_error)
  {
    if (critical_state)
    {
      const bool stale = status->level == diagnostic_msgs::msg::DiagnosticStatus::STALE;
      std::string reason = "Diagnostic " + diagnostic_name +
        (stale ? " has stopped updating." : " reports an error.");
      const auto detail = trim(status->message);
      if (!detail.empty() && detail != "OK") {reason += " Details: " + detail;}
      critical_state->set(diagnostic_name, reason);
    }
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

  if (critical_state) {critical_state->clear(diagnostic_name);}

  return BT::NodeStatus::FAILURE;
}


// =======================================================
// SafetyErrorAsk
// =======================================================

SafetyErrorAsk::SafetyErrorAsk(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SafetyErrorAsk::providedPorts()
{
  return diagnosticPorts();
}

const char * SafetyErrorAsk::main_description()
{
  return "Pauses the mission on a diagnostic ERROR until an intervention is resolved.";
}

BT::NodeStatus SafetyErrorAsk::tick()
{
  const auto monitor = getMonitor(config());
  const auto diagnostic_name = requiredDiagnostic(*this);
  const auto source = monitor->resolveName(diagnostic_name);
  const auto state = config().blackboard->get<std::shared_ptr<SafetyAskState>>("safety_ask_state");
  const auto status = monitor->getStatus(diagnostic_name);
  if (!status || !statusIsError(*status, getInput<bool>("stale_is_error").value()))
  {
    state->rearm(source);
    return BT::NodeStatus::FAILURE;
  }

  if (state->blocked_sources.count(source) != 0)
  {
    config().blackboard->set("mission_control", std::string{"abort"});
    config().blackboard->set("mission_state", std::string{"aborted"});
    return BT::NodeStatus::SUCCESS;
  }
  std::string detail = status->message;
  for (const auto & value : status->values) {
    detail += " " + value.key + "=" + value.value;
  }
  if (state->activate(source, detail))
  {
    RCLCPP_ERROR(
      getRosNode(config())->get_logger(),
      "[sura_safety] Safety decision required. diagnostic=%s %s",
      diagnostic_name.c_str(), detail.c_str());
    config().blackboard->set("mission_state", std::string{"awaiting_decision"});
  }
  if (state->active_source == source)
  {
    config().blackboard->set("mission_control", std::string{"ask"});
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
  return diagnosticPorts();
}

const char * SafetyWarning::main_description()
{
  return "Detects a safety warning condition for mission awareness.";
}

BT::NodeStatus SafetyWarning::tick()
{
  return safetyWarningTick(
    config(),
    requiredDiagnostic(*this));
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
    BT::InputPort<std::string>(
      "diagnostics",
      "Comma-separated source identities that must all be unavailable."),
    BT::InputPort<double>(
      "seconds",
      "Minimum duration that the diagnostics must remain unavailable, in seconds.")
  };
}

const char * DiagnosticsUnavailableFor::main_description()
{
  return "Checks whether selected diagnostics have been unavailable for a required duration.";
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
    const auto & diagnostic_name = diagnostic;
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

const char * SafetyOk::main_description()
{
  return "Reports that the safety branch is currently clear.";
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
  return {
    BT::InputPort<std::string>("critical_diagnostics", "", "Comma-separated diagnostics whose ERROR aborts the mission."),
    BT::InputPort<std::string>("recoverable_diagnostics", "", "Comma-separated diagnostics whose ERROR pauses the mission."),
    BT::InputPort<std::string>("stale_error_diagnostics", "", "Diagnostics whose STALE also counts as ERROR.")
  };
}

const char * UpdateMissionControlFromSafety::main_description()
{
  return "Updates mission control state according to active safety diagnostics.";
}

BT::NodeStatus UpdateMissionControlFromSafety::tick()
{
  const auto monitor = getMonitor(config());

  const auto stale_errors = splitCommaList(getInput<std::string>("stale_error_diagnostics").value());
  const auto has_error = [&](const std::string & identity) {
    const auto status = monitor->getStatus(identity);
    const bool stale_is_error = std::find(stale_errors.begin(), stale_errors.end(), identity) !=
      stale_errors.end();
    return status && statusIsError(*status, stale_is_error);
  };
  for (const auto & identity : splitCommaList(getInput<std::string>("critical_diagnostics").value()))
  {
    if (has_error(identity))
    {
      config().blackboard->set("mission_control", std::string{"abort"});
      config().blackboard->set("mission_state", std::string{"aborted"});
      return BT::NodeStatus::SUCCESS;
    }
  }
  const auto ask_state = config().blackboard->get<std::shared_ptr<SafetyAskState>>("safety_ask_state");
  if (!ask_state->blocked_sources.empty())
  {
    config().blackboard->set("mission_control", std::string{"abort"});
    config().blackboard->set("mission_state", std::string{"aborted"});
    return BT::NodeStatus::SUCCESS;
  }
  if (ask_state->pending())
  {
    config().blackboard->set("mission_control", std::string{"ask"});
    return BT::NodeStatus::SUCCESS;
  }

  for (const auto & identity : splitCommaList(getInput<std::string>("recoverable_diagnostics").value()))
  {
    if (has_error(identity))
    {
      config().blackboard->set("mission_control", std::string{"pause"});
      return BT::NodeStatus::SUCCESS;
    }
  }

  config().blackboard->set("mission_control", std::string{"run"});
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sura_safety
