#pragma once

#include <memory>
#include <string>

#include "a2w_fastlio_common/local_map_builder.hpp"
#include "a2w_fastlio_common/registration_pipeline.hpp"
#include "a2w_fastlio_localization/local_map_selector.hpp"

namespace a2w_fastlio_localization
{

struct MapMatchResult
{
  bool success{false};
  std::string reason;
  std::uint64_t candidate_id{0U};
  a2w_fastlio_common::Pose3d map_body{};
  a2w_fastlio_common::RegistrationResult registration{};
};

class MapMatcher
{
public:
  MapMatcher(
    a2w_fastlio_map::MapSnapshot snapshot,
    std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> registration,
    a2w_fastlio_common::LocalMapConfig local_map_config);
  MapMatchResult match(
    const a2w_fastlio_common::FrontendFrame & frame,
    const SelectionResult & selection) const;

private:
  class Provider;
  std::shared_ptr<Provider> provider_;
  std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> registration_;
  a2w_fastlio_common::LocalMapBuilder local_map_builder_;
};

}  // namespace a2w_fastlio_localization
