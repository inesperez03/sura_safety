#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "sura_safety/diagnostics_monitor.hpp"
#include "sura_safety/safety_blackboard.hpp"
#include "sura_safety/safety_ask_state.hpp"
#include "sura_safety/critical_safety_state.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using namespace std::chrono_literals;

class DiagnosticsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("diagnostics_contract_test",
      rclcpp::NodeOptions().use_intra_process_comms(true));
    monitor = std::make_shared<sura_safety::DiagnosticsMonitor>(node, "/test_diagnostics", "/robot_a/", 0.15);
    pub = node->create_publisher<DiagnosticArray>("/test_diagnostics", 10);
    executor.add_node(node);
    blackboard = BT::Blackboard::create();
    blackboard->set("ros_node", node);
    blackboard->set("diagnostics_monitor", monitor);
    blackboard->set("safety_ask_state", std::make_shared<sura_safety::SafetyAskState>());
    critical_state = std::make_shared<sura_safety::CriticalSafetyState>();
    blackboard->set("critical_safety_state", critical_state);
    sura_safety::registerSafetyNodes(factory);
  }

  DiagnosticStatus leaf(const std::string & source, unsigned char level = DiagnosticStatus::OK,
    const std::string & display = "/ROBOTS/ROBOT_A/Sensors/GPS")
  {
    DiagnosticStatus status;
    status.name = display;
    status.level = level;
    diagnostic_msgs::msg::KeyValue identity;
    identity.key = "source_name";
    identity.value = source;
    status.values.push_back(identity);
    return status;
  }

  void publish(std::vector<DiagnosticStatus> statuses)
  {
    auto msg = std::make_unique<DiagnosticArray>();
    msg->status = std::move(statuses);
    pub->publish(std::move(msg));
    executor.spin_some();
  }

  BT::Tree tree(const std::string & nodes)
  {
    return factory.createTreeFromText(
      "<root main_tree_to_execute=\"Test\"><BehaviorTree ID=\"Test\">" + nodes +
      "</BehaviorTree></root>", blackboard);
  }

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<sura_safety::DiagnosticsMonitor> monitor;
  std::shared_ptr<sura_safety::CriticalSafetyState> critical_state;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub;
  rclcpp::executors::SingleThreadedExecutor executor;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
};

TEST_F(DiagnosticsTest, IdentitySurvivesDisplayRenameAndSeparatesRobots)
{
  publish({leaf("/robot_a/Sensors/GPS", DiagnosticStatus::WARN),
    leaf("/robot_b/Sensors/GPS", DiagnosticStatus::ERROR, "/ROBOTS/ROBOT_B/Sensors/GPS")});
  ASSERT_TRUE(monitor->isWarn("Sensors/GPS"));
  EXPECT_TRUE(monitor->isErrorOrStale("/robot_b/Sensors/GPS"));
  publish({leaf("/robot_a/Sensors/GPS", DiagnosticStatus::WARN, "/fleet/vehicles/arbitrary/gps")});
  EXPECT_TRUE(monitor->isWarn("Sensors/GPS"));
  EXPECT_TRUE(monitor->hasStatus("/fleet/vehicles/arbitrary/gps"));
  EXPECT_FALSE(monitor->hasStatus("/ROBOTS/ROBOT_A/Sensors/GPS"));
  EXPECT_FALSE(monitor->hasStatus("/robot_b/Sensors/GPS"));
}

TEST_F(DiagnosticsTest, RejectsAmbiguousSourceIdentity)
{
  publish({leaf("/robot_a/Sensors/GPS"),
    leaf("/robot_a/Sensors/GPS", DiagnosticStatus::ERROR, "/other/gps")});
  EXPECT_FALSE(monitor->hasStatus("Sensors/GPS"));
}

TEST_F(DiagnosticsTest, ReceptionExpiresAndRecovers)
{
  publish({leaf("/robot_a/Sensors/GPS")});
  ASSERT_TRUE(monitor->hasStatus("Sensors/GPS"));
  std::this_thread::sleep_for(180ms);
  EXPECT_TRUE(monitor->isErrorOrStale("Sensors/GPS"));
  publish({leaf("/robot_a/Sensors/GPS")});
  EXPECT_FALSE(monitor->isErrorOrStale("Sensors/GPS"));
  publish({});
  EXPECT_FALSE(monitor->hasStatus("Sensors/GPS"));
}

TEST_F(DiagnosticsTest, ConditionsUseExplicitIdentityAndStalePolicy)
{
  auto check = tree("<SafetyError name=\"arbitrary_label\" diagnostic=\"Navigation/DepthLimit\"/>");
  publish({leaf("/robot_a/Navigation/DepthLimit", DiagnosticStatus::ERROR)});
  EXPECT_EQ(check.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<std::string>("mission_control"), "pause");
  publish({leaf("/robot_a/Navigation/DepthLimit", DiagnosticStatus::STALE)});
  EXPECT_EQ(check.tickRoot(), BT::NodeStatus::FAILURE);
  auto critical = tree("<SafetyCriticalError diagnostic=\"Sensors/IMU\" stale_is_error=\"true\"/>");
  publish({leaf("/robot_a/Sensors/IMU", DiagnosticStatus::STALE)});
  EXPECT_EQ(critical.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<std::string>("mission_control"), "abort");
}

TEST_F(DiagnosticsTest, CriticalNavigationFailureHasAnExplanation)
{
  auto critical = tree(
    "<SafetyCriticalError diagnostic=\"Navigation/Frequency\" stale_is_error=\"true\"/>");
  publish({leaf("/robot_a/Navigation/Frequency", DiagnosticStatus::STALE)});
  EXPECT_EQ(critical.tickRoot(), BT::NodeStatus::SUCCESS);
  const auto incident = critical_state->first();
  ASSERT_TRUE(incident.has_value());
  EXPECT_EQ(incident->first, "Navigation/Frequency");
  EXPECT_NE(incident->second.find("Navigation/Frequency ha dejado de actualizarse"),
    std::string::npos);

  publish({leaf("/robot_a/Navigation/Frequency", DiagnosticStatus::OK)});
  EXPECT_EQ(critical.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(critical_state->first().has_value());

  auto imu = tree("<SafetyCriticalError diagnostic=\"Sensors/IMU\"/>");
  publish({leaf("/robot_a/Sensors/IMU", DiagnosticStatus::ERROR)});
  EXPECT_EQ(imu.tickRoot(), BT::NodeStatus::SUCCESS);
  const auto second_incident = critical_state->first();
  ASSERT_TRUE(second_incident.has_value());
  EXPECT_NE(second_incident->second.find("Sensors/IMU indica un error"), std::string::npos);
}

TEST_F(DiagnosticsTest, SafetyErrorAskWaitsAndRearmsAfterClear)
{
  auto ask = tree("<SafetyErrorAsk diagnostic=\"Navigation/FrontObstacle\"/>");
  auto state = blackboard->get<std::shared_ptr<sura_safety::SafetyAskState>>("safety_ask_state");
  publish({leaf("/robot_a/Navigation/FrontObstacle", DiagnosticStatus::ERROR)});
  EXPECT_EQ(ask.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<std::string>("mission_control"), "ask");
  EXPECT_EQ(blackboard->get<std::string>("mission_state"), "awaiting_decision");
  ASSERT_TRUE(state->pending());

  state->resolve();
  EXPECT_EQ(ask.tickRoot(), BT::NodeStatus::FAILURE);
  publish({leaf("/robot_a/Navigation/FrontObstacle", DiagnosticStatus::OK)});
  EXPECT_EQ(ask.tickRoot(), BT::NodeStatus::FAILURE);
  publish({leaf("/robot_a/Navigation/FrontObstacle", DiagnosticStatus::ERROR)});
  EXPECT_EQ(ask.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(state->pending());
}

TEST_F(DiagnosticsTest, SafetyErrorAskAbortStaysLatchedUntilDiagnosticClears)
{
  auto ask = tree("<SafetyErrorAsk diagnostic=\"Navigation/FrontObstacle\"/>");
  auto state = blackboard->get<std::shared_ptr<sura_safety::SafetyAskState>>("safety_ask_state");
  publish({leaf("/robot_a/Navigation/FrontObstacle", DiagnosticStatus::ERROR)});
  EXPECT_EQ(ask.tickRoot(), BT::NodeStatus::SUCCESS);
  state->abort();
  EXPECT_EQ(ask.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<std::string>("mission_control"), "abort");
  EXPECT_EQ(blackboard->get<std::string>("mission_state"), "aborted");
  publish({leaf("/robot_a/Navigation/FrontObstacle", DiagnosticStatus::OK)});
  EXPECT_EQ(ask.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(state->blocked_sources.empty());
}

TEST_F(DiagnosticsTest, HealthyGpsPreventsFalseUnavailableCondition)
{
  auto check = tree("<DiagnosticsUnavailableFor diagnostics=\"Sensors/GPS,Sensors/ArucoPose\" seconds=\"0.01\"/>");
  publish({leaf("/robot_a/Sensors/GPS")});
  EXPECT_EQ(check.tickRoot(), BT::NodeStatus::FAILURE);
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(check.tickRoot(), BT::NodeStatus::FAILURE);
  publish({});
  EXPECT_EQ(check.tickRoot(), BT::NodeStatus::FAILURE);
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(check.tickRoot(), BT::NodeStatus::SUCCESS);
  publish({leaf("/robot_a/Sensors/GPS")});
  EXPECT_EQ(check.tickRoot(), BT::NodeStatus::FAILURE);
}

TEST_F(DiagnosticsTest, MissionPolicyUsesConfiguredIdentities)
{
  auto check = tree("<UpdateMissionControlFromSafety critical_diagnostics=\"Sensors/Battery\" "
    "recoverable_diagnostics=\"Navigation/AreaLimit\" stale_error_diagnostics=\"Sensors/Battery\"/>");
  publish({leaf("/robot_a/Navigation/AreaLimit", DiagnosticStatus::ERROR)});
  check.tickRoot();
  EXPECT_EQ(blackboard->get<std::string>("mission_control"), "pause");
  publish({leaf("/robot_a/Sensors/Battery", DiagnosticStatus::STALE)});
  check.tickRoot();
  EXPECT_EQ(blackboard->get<std::string>("mission_control"), "abort");
}
