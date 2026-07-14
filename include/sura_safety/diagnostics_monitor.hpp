#pragma once

#include <map>
#include <mutex>
#include <optional>
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
    const std::string & diagnostics_topic);

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

  std::map<std::string, diagnostic_msgs::msg::DiagnosticStatus> statuses_;
};

}  // namespace sura_safety