#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "sura_safety/diagnostics_monitor.hpp"
#include "sura_msgs/msg/navigator.hpp"
#include "sura_msgs/msg/sura_wrench_command.hpp"

namespace sura_safety
{

class LimitsSafetyBase
{
protected:
  static std::shared_ptr<DiagnosticsMonitor> getMonitor(
    const BT::NodeConfiguration & config);

  static rclcpp::Node::SharedPtr getRosNode(
    const BT::NodeConfiguration & config);

  static std::string getDepthDiagnosticName(
    const BT::NodeConfiguration & config);
};


class SafetyError : public BT::SyncActionNode, protected LimitsSafetyBase
{
public:
  SafetyError(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class SafetyWarning : public BT::SyncActionNode, protected LimitsSafetyBase
{
public:
  SafetyWarning(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class EmergencyWrench : public BT::SyncActionNode, protected LimitsSafetyBase
{
public:
  EmergencyWrench(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  struct BodyForce
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  using WrenchCommand = sura_msgs::msg::SuraWrenchCommand;
  using Navigator = sura_msgs::msg::Navigator;

  std::optional<BodyForce> computeAreaRecoveryForce() const;
  void navigatorCallback(const Navigator::SharedPtr msg);

  rclcpp::Publisher<WrenchCommand>::SharedPtr wrench_pub_;
  rclcpp::Subscription<Navigator>::SharedPtr navigator_sub_;

  mutable std::mutex navigator_mutex_;
  Navigator::SharedPtr last_navigator_msg_;
};


class SafetyOk : public BT::SyncActionNode
{
public:
  SafetyOk(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace sura_safety
