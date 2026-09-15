#include "a2w_fastlio_common/global_tf_ownership.hpp"

#include <stdexcept>
#include <utility>

namespace a2w_fastlio_common
{

GlobalTfOwnerState::GlobalTfOwnerState(GlobalTfOwnerConfig config)
: config_{std::move(config)}
{
  if (config_.owner_id.empty() || config_.mode.empty() || config_.parent_frame.empty() ||
    config_.child_frame.empty())
  {
    throw std::invalid_argument{"global TF ownership strings must not be empty"};
  }
  if (config_.parent_frame == config_.child_frame) {
    throw std::invalid_argument{"global TF parent and child frames must differ"};
  }
  if (config_.conflict_window_ns <= 0) {
    throw std::invalid_argument{"global TF conflict window must be positive"};
  }
}

void GlobalTfOwnerState::start(const std::int64_t now_ns)
{
  if (now_ns < 0) {
    throw std::invalid_argument{"start time must not be negative"};
  }
  std::lock_guard<std::mutex> lock{mutex_};
  if (!started_) {
    start_ns_ = now_ns;
    started_ = true;
  }
}

bool GlobalTfOwnerState::observe(
  const GlobalTfOwnerObservation & observation, const std::int64_t received_ns)
{
  if (received_ns < 0) {
    throw std::invalid_argument{"receive time must not be negative"};
  }
  std::lock_guard<std::mutex> lock{mutex_};
  const bool conflict = observation.active && observation.owner_id != config_.owner_id &&
    observation.parent_frame == config_.parent_frame &&
    observation.child_frame == config_.child_frame;
  conflict_latched_ = conflict_latched_ || conflict;
  return conflict;
}

bool GlobalTfOwnerState::mayPublish(const std::int64_t now_ns) const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return started_ && !conflict_latched_ && now_ns >= start_ns_ + config_.conflict_window_ns;
}

bool GlobalTfOwnerState::conflicted(const std::int64_t) const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return conflict_latched_;
}

std::string GlobalTfOwnerState::faultReason(const std::int64_t) const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return conflict_latched_ ? "foreign_global_tf_owner_active" : std::string{};
}

}  // namespace a2w_fastlio_common
