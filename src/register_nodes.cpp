#include "behaviortree_cpp_v3/bt_factory.h"

#include "sura_safety/safety_blackboard.hpp"

BT_REGISTER_NODES(factory)
{
  sura_safety::registerSafetyNodes(factory);
}
