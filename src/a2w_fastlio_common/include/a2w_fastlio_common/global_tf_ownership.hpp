#pragma once

#include <cstdint>
#include <mutex>
#include <string>

namespace a2w_fastlio_common
{

struct GlobalTfOwnerConfig
{
  std::string owner_id;
  std::string mode;
  std::string parent_frame;
  std::string child_frame;
  std::int64_t conflict_window_ns{0};
};

struct GlobalTfOwnerObservation
{
  std::int64_t stamp_ns{0};
  std::string owner_id;
  std::string mode;
  std::string parent_frame;
  std::string child_frame;
  bool active{false};
};

class GlobalTfOwnerState
{
public:
  explicit GlobalTfOwnerState(GlobalTfOwnerConfig config);

  void start(std::int64_t now_ns);
  bool observe(const GlobalTfOwnerObservation & observation, std::int64_t received_ns);
  bool mayPublish(std::int64_t now_ns) const;
  bool conflicted(std::int64_t now_ns) const;
  std::string faultReason(std::int64_t now_ns) const;

private:
  GlobalTfOwnerConfig config_;
  mutable std::mutex mutex_;
  std::int64_t start_ns_{0};
  bool started_{false};
  bool conflict_latched_{false};
};

}  // namespace a2w_fastlio_common
