#pragma once

#include <set>
#include <string>

namespace sura_safety
{

// Shared by the safety subtrees and bt_runner on the single-threaded BT executor.
struct SafetyAskState
{
  std::string active_source;
  std::string active_detail;
  std::set<std::string> suppressed_sources;
  std::set<std::string> blocked_sources;

  bool pending() const {return !active_source.empty();}

  bool activate(const std::string & source, const std::string & detail = "")
  {
    if (pending() || !blocked_sources.empty() ||
      suppressed_sources.count(source) != 0) {return false;}
    active_source = source;
    active_detail = detail;
    return true;
  }

  void resolve()
  {
    if (pending()) {suppressed_sources.insert(active_source);}
    active_source.clear();
    active_detail.clear();
  }

  void abort()
  {
    if (pending()) {blocked_sources.insert(active_source);}
    active_source.clear();
    active_detail.clear();
  }

  void rearm(const std::string & source)
  {
    suppressed_sources.erase(source);
    blocked_sources.erase(source);
  }
};

}  // namespace sura_safety
