#include "sura_safety/diagnostics_monitor.hpp"

#include <functional>

namespace sura_safety
{

DiagnosticsMonitor::DiagnosticsMonitor(
  const rclcpp::Node::SharedPtr & node,
  const std::string & diagnostics_topic)
: node_(node)
{
  diagnostics_sub_ =
    node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic,
      rclcpp::QoS(10),
      std::bind(
        &DiagnosticsMonitor::diagnosticsCallback,
        this,
        std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "[sura_safety] Subscribed to diagnostics topic: %s",
    diagnostics_topic.c_str());
}

void DiagnosticsMonitor::diagnosticsCallback(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & status : msg->status)
  {
    statuses_[status.name] = status;
  }
}

bool DiagnosticsMonitor::hasStatus(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  return statuses_.find(name) != statuses_.end();
}

std::optional<diagnostic_msgs::msg::DiagnosticStatus>
DiagnosticsMonitor::getStatus(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto it = statuses_.find(name);

  if (it == statuses_.end())
  {
    return std::nullopt;
  }

  return it->second;
}

bool DiagnosticsMonitor::isWarn(const std::string & name) const
{
  const auto status = getStatus(name);

  if (!status)
  {
    return false;
  }

  return status->level == diagnostic_msgs::msg::DiagnosticStatus::WARN;
}

bool DiagnosticsMonitor::isErrorOrStale(const std::string & name) const
{
  const auto status = getStatus(name);

  if (!status)
  {
    return false;
  }

  return status->level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ||
         status->level == diagnostic_msgs::msg::DiagnosticStatus::STALE;
}

std::string DiagnosticsMonitor::getValue(
  const std::string & name,
  const std::string & key,
  const std::string & default_value) const
{
  const auto status = getStatus(name);

  if (!status)
  {
    return default_value;
  }

  for (const auto & kv : status->values)
  {
    if (kv.key == key)
    {
      return kv.value;
    }
  }

  return default_value;
}

}  // namespace sura_safety
