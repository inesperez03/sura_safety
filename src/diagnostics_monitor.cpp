#include "sura_safety/diagnostics_monitor.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

namespace sura_safety
{
namespace
{
std::string canonicalName(const std::string & name)
{
  const auto first = name.find_first_not_of('/');
  if (first == std::string::npos) { return ""; }
  return "/" + name.substr(first, name.find_last_not_of('/') - first + 1);
}
}  // namespace

std::string DiagnosticsMonitor::resolveName(const std::string & name) const
{
  if (name.empty()) { throw std::invalid_argument("Diagnostic identity must not be empty"); }
  return name.front() == '/' ? canonicalName(name) :
    canonicalName(robot_namespace_ + "/" + name);
}


DiagnosticsMonitor::DiagnosticsMonitor(
  const rclcpp::Node::SharedPtr & node,
  const std::string & diagnostics_topic,
  const std::string & robot_namespace,
  double timeout_seconds)
: node_(node), robot_namespace_(canonicalName(robot_namespace)), timeout_seconds_(timeout_seconds)
{
  if (!std::isfinite(timeout_seconds_) || timeout_seconds_ <= 0.0)
  {
    throw std::invalid_argument("Diagnostics timeout must be finite and positive");
  }
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

  // /diagnostics_agg is a complete snapshot. Removed or renamed entries must
  // not survive the next snapshot, nor leave stale aliases behind.
  statuses_.clear();
  source_names_.clear();
  ambiguous_names_.clear();
  const auto received = std::chrono::steady_clock::now();
  for (const auto & status : msg->status)
  {
    const auto published_name = canonicalName(status.name);
    if (published_name.empty()) { continue; }
    if (!statuses_.emplace(published_name, Entry{status, received}).second)
    {
      ambiguous_names_.insert(published_name);
    }
    for (const auto & value : status.values)
    {
      if (value.key != "source_name") { continue; }
      const auto source = canonicalName(value.value);
      if (source.empty()) { continue; }
      if (!source_names_.emplace(source, published_name).second)
      {
        ambiguous_names_.insert(source);
      }
    }
  }
  for (const auto & source : ambiguous_names_)
  {
    source_names_.erase(source);
    RCLCPP_ERROR(node_->get_logger(), "Ambiguous diagnostic source identity: %s", source.c_str());
  }
}

bool DiagnosticsMonitor::hasStatus(const std::string & name) const
{
  return getStatus(name).has_value();
}

std::optional<diagnostic_msgs::msg::DiagnosticStatus>
DiagnosticsMonitor::getStatus(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto resolved = resolveName(name);
  if (ambiguous_names_.count(resolved)) { return std::nullopt; }
  const auto alias = source_names_.find(resolved);
  if (alias != source_names_.end() && ambiguous_names_.count(alias->second))
  {
    return std::nullopt;
  }
  const auto it = statuses_.find(alias == source_names_.end() ? resolved : alias->second);

  if (it == statuses_.end())
  {
    return std::nullopt;
  }

  auto status = it->second.status;
  const auto age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - it->second.received).count();
  if (age > timeout_seconds_)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.message = "Diagnostics reception timed out";
  }
  return status;
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
