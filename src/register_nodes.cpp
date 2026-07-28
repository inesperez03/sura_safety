#include "behaviortree_cpp_v3/bt_factory.h"

#include "sura_safety/limits_safety_nodes.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sura_safety::SafetyError>(
    "SafetyError");

  factory.registerNodeType<sura_safety::SafetyWarning>(
    "SafetyWarning");

  factory.registerNodeType<sura_safety::DiagnosticsUnavailableFor>(
    "DiagnosticsUnavailableFor");

  factory.registerNodeType<sura_safety::SafetyOk>(
    "SafetyOk");
}
