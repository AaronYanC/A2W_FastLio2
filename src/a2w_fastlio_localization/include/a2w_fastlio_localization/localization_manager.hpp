#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include "a2w_fastlio_localization/map_matcher.hpp"

namespace a2w_fastlio_localization
{

struct LocalizationManagerConfig
{
  std::int64_t match_interval_ns{1'000'000'000LL};
  double maximum_correction_translation_jump_m{5.0};
  double maximum_correction_rotation_jump_rad{1.0};
};

struct LocalizationOutput
{
  bool valid{false};
  bool match_attempted{false};
  bool match_accepted{false};
  std::string reason;
  std::int64_t stamp_ns{0};
  std::int64_t correction_stamp_ns{0};
  std::int64_t correction_age_ns{0};
  std::uint64_t candidate_id{0U};
  a2w_fastlio_common::Pose3d map_body{};
  a2w_fastlio_common::Pose3d map_camera_init{};
  a2w_fastlio_common::RegistrationResult registration{};
};

using MatchFunction = std::function<MapMatchResult(
    const a2w_fastlio_common::FrontendFrame &,
    const a2w_fastlio_common::Pose3d &)>;

class LocalizationManager
{
public:
  LocalizationManager(LocalizationManagerConfig config, MatchFunction matcher);
  LocalizationOutput process(const a2w_fastlio_common::FrontendFrame & frame);
  void restoreCorrection(
    std::int64_t stamp_ns, const a2w_fastlio_common::Pose3d & map_camera_init);
  std::optional<LocalizationOutput> latest() const;

private:
  LocalizationManagerConfig config_;
  MatchFunction matcher_;
  mutable std::mutex mutex_;
  std::optional<a2w_fastlio_common::Pose3d> correction_;
  std::optional<std::int64_t> correction_stamp_ns_;
  std::optional<std::int64_t> last_attempt_ns_;
  std::optional<std::int64_t> last_frame_ns_;
  std::optional<LocalizationOutput> latest_;
};

}  // namespace a2w_fastlio_localization
