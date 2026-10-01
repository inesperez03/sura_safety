#pragma once

#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace sura_safety
{

class DiagnosticsMonitor
{
public:
  DiagnosticsMonitor(
    const rclcpp::Node::SharedPtr & node,
    const std::string & diagnostics_topic,
    const std::string & robot_namespace = "",
    double timeout_seconds = 5.0);

  // Relative paths identify a source diagnostic within this robot.
  // Absolute paths explicitly identify a source or a published entry.
  std::string resolveName(const std::string & name) const;

  bool hasStatus(const std::string & name) const;

  std::optional<diagnostic_msgs::msg::DiagnosticStatus>
  getStatus(const std::string & name) const;

  bool isWarn(const std::string & name) const;
  bool isErrorOrStale(const std::string & name) const;

  std::string getValue(
    const std::string & name,
    const std::string & key,
    const std::string & default_value = "") const;

private:
  void diagnosticsCallback(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_sub_;

  mutable std::mutex mutex_;

  struct Entry
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    std::chrono::steady_clock::time_point received;
  };

  std::string robot_namespace_;
  double timeout_seconds_;
  std::map<std::string, Entry> statuses_;
  std::map<std::string, std::string> source_names_;
  std::set<std::string> ambiguous_names_;
};

}  // namespace sura_safety