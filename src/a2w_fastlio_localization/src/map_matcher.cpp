#include "a2w_fastlio_localization/map_matcher.hpp"

#include <optional>
#include <stdexcept>
#include <utility>

namespace a2w_fastlio_localization
{

class MapMatcher::Provider final : public a2w_fastlio_common::KeyFrameProvider
{
public:
  explicit Provider(a2w_fastlio_map::MapSnapshot snapshot) : snapshot_{std::move(snapshot)} {}
  std::optional<a2w_fastlio_common::KeyFrame> get(const std::uint64_t id) const override
  {
    return id < snapshot_.keyframes.size() ?
      std::optional<a2w_fastlio_common::KeyFrame>{snapshot_.keyframes[id]} : std::nullopt;
  }
  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    return snapshot_.keyframes.empty() ? std::nullopt :
      std::optional<std::pair<std::uint64_t, std::uint64_t>>{{0U, snapshot_.keyframes.size() - 1U}};
  }
private:
  a2w_fastlio_map::MapSnapshot snapshot_;
};

MapMatcher::MapMatcher(
  a2w_fastlio_map::MapSnapshot snapshot,
  std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> registration,
  a2w_fastlio_common::LocalMapConfig local_map_config)
: provider_{std::make_shared<Provider>(std::move(snapshot))},
  registration_{std::move(registration)}, local_map_builder_{local_map_config}
{
  if (!registration_) {
    throw std::invalid_argument{"MapMatcher requires a registration pipeline"};
  }
}

MapMatchResult MapMatcher::match(
  const a2w_fastlio_common::FrontendFrame & frame,
  const SelectionResult & selection) const
{
  MapMatchResult result;
  result.candidate_id = selection.nearest_keyframe_id;
  if (!selection.success) {
    result.reason = "selection_invalid:" + selection.reason;
    return result;
  }
  try {
    const auto target = local_map_builder_.build(selection.keyframe_ids, *provider_);
    a2w_fastlio_common::ValidationContext context;
    context.predicted_transform = selection.initial_map_body;
    const auto pipeline = registration_->run(
      frame.body_cloud, target.cloud, selection.initial_map_body, context);
    result.success = pipeline.success;
    result.reason = pipeline.validation.reason;
    result.map_body = pipeline.final_transform;
    result.registration = pipeline.fine;
  } catch (const std::exception & error) {
    result.reason = std::string{"map_match_failed:"} + error.what();
  }
  return result;
}

}  // namespace a2w_fastlio_localization
