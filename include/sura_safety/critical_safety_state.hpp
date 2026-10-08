#pragma once

#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace sura_safety
{

// Active critical diagnostics shared with the mission status publisher.
class CriticalSafetyState
{
public:
  void set(const std::string & diagnostic, const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reasons_[diagnostic] = reason;
  }

  void clear(const std::string & diagnostic)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reasons_.erase(diagnostic);
  }

  std::optional<std::pair<std::string, std::string>> first() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (reasons_.empty()) {return std::nullopt;}
    return *reasons_.begin();
  }

private:
  mutable std::mutex mutex_;
  std::map<std::string, std::string> reasons_;
};

}  // namespace sura_safety
