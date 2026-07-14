#include "sura_safety/limits_safety_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>

namespace sura_safety
{
namespace
{

std::string stripSlashes(std::string value)
{
  while (!value.empty() && value.front() == '/')
  {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/')
  {
    value.pop_back();
  }
  return value;
}

std::string namespacedTopic(const std::string & robot_namespace, const std::string & suffix)
{
  const auto normalized_namespace = stripSlashes(robot_namespace);
  const auto normalized_suffix = stripSlashes(suffix);

  if (normalized_namespace.empty())
  {
    return "/" + normalized_suffix;
  }

  return "/" + normalized_namespace + "/" + normalized_suffix;
}

std::optional<double> parseDouble(const std::string & text)
{
  char * end = nullptr;
  const double value = std::strtod(text.c_str(), &end);

  if (end == text.c_str() || *end != '\0' || !std::isfinite(value))
  {
    return std::nullopt;
  }

  return value;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
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
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("ros_node"),
    BT::InputPort<std::string>("diagnostic_prefix"),
    BT::InputPort<std::shared_ptr<DiagnosticsMonitor>>("diagnostics_monitor"),
    BT::InputPort<std::string>("diagnostic_name"),
    BT::InputPort<std::string>("diagnostic_suffix")
  };
}

BT::NodeStatus SafetyError::tick()
{
  const auto monitor = getMonitor(config());
  const auto ros_node = getRosNode(config());
  auto diagnostic_name = getInput<std::string>("diagnostic_name");
  if (!diagnostic_name)
  {
    auto diagnostic_suffix = getInput<std::string>("diagnostic_suffix");
    if (!diagnostic_suffix)
    {
      diagnostic_suffix = "DepthLimit";
    }

    const std::string diagnostic_prefix =
      config().blackboard->get<std::string>("diagnostic_prefix");
    diagnostic_name =
      diagnostic_prefix + "/Navigation/ Navigation " + diagnostic_suffix.value();
  }

  if (monitor->isErrorOrStale(diagnostic_name.value()))
  {
    const auto current_depth =
      monitor->getValue(diagnostic_name.value(), "current_depth", "unknown");

    const auto max_depth =
      monitor->getValue(diagnostic_name.value(), "max_depth", "unknown");

    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_safety] Safety ERROR/STALE. diagnostic=%s current_depth=%s max_depth=%s",
      diagnostic_name.value().c_str(),
      current_depth.c_str(),
      max_depth.c_str());

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
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("ros_node"),
    BT::InputPort<std::string>("diagnostic_prefix"),
    BT::InputPort<std::shared_ptr<DiagnosticsMonitor>>("diagnostics_monitor"),
    BT::InputPort<std::string>("diagnostic_name"),
    BT::InputPort<std::string>("diagnostic_suffix")
  };
}

BT::NodeStatus SafetyWarning::tick()
{
  const auto monitor = getMonitor(config());
  const auto ros_node = getRosNode(config());
  auto diagnostic_name = getInput<std::string>("diagnostic_name");
  if (!diagnostic_name)
  {
    auto diagnostic_suffix = getInput<std::string>("diagnostic_suffix");
    if (!diagnostic_suffix)
    {
      diagnostic_suffix = "DepthLimit";
    }

    const std::string diagnostic_prefix =
      config().blackboard->get<std::string>("diagnostic_prefix");
    diagnostic_name =
      diagnostic_prefix + "/Navigation/ Navigation " + diagnostic_suffix.value();
  }

  if (monitor->isWarn(diagnostic_name.value()))
  {
    const auto current_depth =
      monitor->getValue(diagnostic_name.value(), "current_depth", "unknown");

    const auto remaining_margin =
      monitor->getValue(diagnostic_name.value(), "remaining_depth_margin", "unknown");

    RCLCPP_WARN(
      ros_node->get_logger(),
      "[sura_safety] Safety WARNING. diagnostic=%s current_depth=%s remaining_depth_margin=%s",
      diagnostic_name.value().c_str(),
      current_depth.c_str(),
      remaining_margin.c_str());

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}


// =======================================================
// EmergencyWrench
// =======================================================

EmergencyWrench::EmergencyWrench(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  const auto ros_node = getRosNode(config);
  const auto robot_namespace =
    config.blackboard->get<std::string>("robot_namespace");

  wrench_pub_ = ros_node->create_publisher<WrenchCommand>(
    namespacedTopic(robot_namespace, "controller/arbitrator/wrench"),
    rclcpp::SystemDefaultsQoS());

  navigator_sub_ = ros_node->create_subscription<Navigator>(
    namespacedTopic(robot_namespace, "navigator/navigation"),
    rclcpp::SystemDefaultsQoS(),
    std::bind(&EmergencyWrench::navigatorCallback, this, std::placeholders::_1));
}

BT::PortsList EmergencyWrench::providedPorts()
{
  return {
    BT::InputPort<std::string>("reason"),
    BT::InputPort<rclcpp::Node::SharedPtr>("ros_node"),
    BT::InputPort<std::string>("controller"),
    BT::InputPort<int>("priority"),
    BT::InputPort<double>("force_x"),
    BT::InputPort<double>("force_y"),
    BT::InputPort<double>("force_z"),
    BT::InputPort<bool>("area_recovery"),
    BT::InputPort<double>("area_recovery_force"),
    BT::InputPort<std::string>("diagnostic_name"),
    BT::InputPort<std::string>("diagnostic_suffix")
  };
}

BT::NodeStatus EmergencyWrench::tick()
{
  const auto ros_node = getRosNode(config());

  auto reason = getInput<std::string>("reason");

  if (!reason)
  {
    reason = "unknown";
  }

  auto controller = getInput<std::string>("controller");
  if (!controller)
  {
    controller = "body_force";
  }

  auto priority = getInput<int>("priority");
  if (!priority)
  {
    priority = 85;
  }

  auto force_x = getInput<double>("force_x");
  if (!force_x)
  {
    force_x = 0.0;
  }

  auto force_y = getInput<double>("force_y");
  if (!force_y)
  {
    force_y = 0.0;
  }

  auto force_z = getInput<double>("force_z");
  if (!force_z)
  {
    force_z = 0.0;
  }

  auto area_recovery = getInput<bool>("area_recovery");
  if (area_recovery && area_recovery.value())
  {
    const auto area_force = computeAreaRecoveryForce();

    if (!area_force)
    {
      RCLCPP_WARN(
        ros_node->get_logger(),
        "[sura_safety] EmergencyWrench area_recovery requested but area force could not be computed");
      return BT::NodeStatus::FAILURE;
    }

    force_x = area_force->x;
    force_y = area_force->y;
    force_z = area_force->z;
  }

  WrenchCommand msg;
  msg.header.stamp = ros_node->now();
  msg.requester = "sura_safety";
  msg.controller = controller.value();
  msg.priority = static_cast<uint8_t>(std::clamp(priority.value(), 1, 100));
  msg.wrench.force.x = force_x.value();
  msg.wrench.force.y = force_y.value();
  msg.wrench.force.z = force_z.value();

  wrench_pub_->publish(msg);

  RCLCPP_ERROR(
    ros_node->get_logger(),
    "[sura_safety] Action: emergency wrench. reason=%s controller=%s priority=%u force=(%.3f, %.3f, %.3f)",
    reason.value().c_str(),
    msg.controller.c_str(),
    msg.priority,
    msg.wrench.force.x,
    msg.wrench.force.y,
    msg.wrench.force.z);

  return BT::NodeStatus::SUCCESS;
}

void EmergencyWrench::navigatorCallback(const Navigator::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(navigator_mutex_);
  last_navigator_msg_ = msg;
}

std::optional<EmergencyWrench::BodyForce> EmergencyWrench::computeAreaRecoveryForce() const
{
  const auto monitor = getMonitor(config());
  auto diagnostic_name = getInput<std::string>("diagnostic_name");
  if (!diagnostic_name)
  {
    auto diagnostic_suffix = getInput<std::string>("diagnostic_suffix");
    if (!diagnostic_suffix)
    {
      diagnostic_suffix = "AreaLimit";
    }

    const std::string diagnostic_prefix =
      config().blackboard->get<std::string>("diagnostic_prefix");
    diagnostic_name =
      diagnostic_prefix + "/Navigation/ Navigation " + diagnostic_suffix.value();
  }

  const auto current_x = parseDouble(
    monitor->getValue(diagnostic_name.value(), "current_x"));
  const auto current_y = parseDouble(
    monitor->getValue(diagnostic_name.value(), "current_y"));
  const auto center_x = parseDouble(
    monitor->getValue(diagnostic_name.value(), "center_x"));
  const auto center_y = parseDouble(
    monitor->getValue(diagnostic_name.value(), "center_y"));

  if (!current_x || !current_y || !center_x || !center_y)
  {
    return std::nullopt;
  }

  double force = 40.0;
  auto area_recovery_force = getInput<double>("area_recovery_force");
  if (area_recovery_force)
  {
    force = std::abs(area_recovery_force.value());
  }

  double target_x = center_x.value();
  double target_y = center_y.value();

  const auto shape = monitor->getValue(diagnostic_name.value(), "shape");
  if (shape == "rectangle" || shape == "rect")
  {
    const auto width = parseDouble(
      monitor->getValue(diagnostic_name.value(), "width"));
    const auto height = parseDouble(
      monitor->getValue(diagnostic_name.value(), "height"));

    if (width && height)
    {
      const double half_width = width.value() * 0.5;
      const double half_height = height.value() * 0.5;

      target_x = std::clamp(
        current_x.value(),
        center_x.value() - half_width,
        center_x.value() + half_width);
      target_y = std::clamp(
        current_y.value(),
        center_y.value() - half_height,
        center_y.value() + half_height);
    }
  }

  const double world_x = target_x - current_x.value();
  const double world_y = target_y - current_y.value();
  const double norm = std::hypot(world_x, world_y);

  if (norm <= 1e-6)
  {
    return BodyForce{};
  }

  Navigator::SharedPtr navigator_msg;
  {
    std::lock_guard<std::mutex> lock(navigator_mutex_);
    navigator_msg = last_navigator_msg_;
  }

  if (!navigator_msg)
  {
    return std::nullopt;
  }

  const double yaw = yawFromQuaternion(navigator_msg->position.orientation);
  const double unit_world_x = world_x / norm;
  const double unit_world_y = world_y / norm;

  BodyForce body_force;
  body_force.x = force * (std::cos(yaw) * unit_world_x + std::sin(yaw) * unit_world_y);
  body_force.y = force * (-std::sin(yaw) * unit_world_x + std::cos(yaw) * unit_world_y);
  body_force.z = 0.0;
  return body_force;
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
