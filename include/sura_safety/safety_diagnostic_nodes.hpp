#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "sura_safety/diagnostics_monitor.hpp"

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
  static const char * main_description();

  BT::NodeStatus tick() override;
};

class SafetyCriticalError : public BT::SyncActionNode, protected LimitsSafetyBase
{
public:
  SafetyCriticalError(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  static const char * main_description();

  BT::NodeStatus tick() override;
};


class SafetyWarning : public BT::SyncActionNode, protected LimitsSafetyBase
{
public:
  SafetyWarning(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  static const char * main_description();

  BT::NodeStatus tick() override;
};

class DiagnosticsUnavailableFor : public BT::SyncActionNode, protected LimitsSafetyBase
{
public:
  DiagnosticsUnavailableFor(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  static const char * main_description();

  BT::NodeStatus tick() override;

private:
  bool condition_active_{false};
  bool reported_{false};
  rclcpp::Time condition_start_time_;
};


class SafetyOk : public BT::SyncActionNode
{
public:
  SafetyOk(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  static const char * main_description();

  BT::NodeStatus tick() override;
};

class UpdateMissionControlFromSafety : public BT::SyncActionNode, protected LimitsSafetyBase
{
public:
  UpdateMissionControlFromSafety(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  static const char * main_description();

  BT::NodeStatus tick() override;
};

}  // namespace sura_safety
