#include <memory>

#include <gtest/gtest.h>

#include "a2w_fastlio_localization/map_matcher.hpp"

namespace a2w_fastlio_localization
{
namespace
{
using namespace a2w_fastlio_common;

class FakeRegistration final : public CoarseRegistration, public FineRegistration
{
public:
  RegistrationResult align(const CloudConstPtr &, const CloudConstPtr &,
    const std::optional<Pose3d> &) const override
  {
    ++calls;
    RegistrationResult result;
    result.success = true;
    result.converged = true;
    result.fitness = fitness;
    result.overlap = 1.0;
    result.correspondence_count = 40U;
    return result;
  }
  mutable int calls{0};
  double fitness{0.01};
};

a2w_fastlio_map::MapSnapshot mapSnapshot()
{
  a2w_fastlio_map::MapSnapshot map;
  KeyFrame frame;
  for (int index = 0; index < 40; ++index) {
    PointT point;
    point.x = static_cast<float>(index) * 0.1F;
    frame.body_cloud->push_back(point);
  }
  map.keyframes.push_back(frame);
  return map;
}

TEST(MapMatcher, UsesCoarseFineAndValidatorInterfaces)
{
  auto algorithm = std::make_shared<FakeRegistration>();
  auto pipeline = std::make_shared<RegistrationPipeline>(
    algorithm, algorithm, MatchValidator{{0.1, 0.5, 10U, 5.0, 1.0, 0.0}},
    RegistrationPipelineConfig{0.5});
  MapMatcher matcher{mapSnapshot(), pipeline, {0.0, 1000U}};
  FrontendFrame frame;
  frame.body_cloud = mapSnapshot().keyframes[0].body_cloud;
  SelectionResult selection;
  selection.success = true;
  selection.keyframe_ids = {0U};
  const auto result = matcher.match(frame, selection);
  EXPECT_TRUE(result.success) << result.reason;
  EXPECT_EQ(algorithm->calls, 2);
  EXPECT_DOUBLE_EQ(result.map_body.translation.x(), 0.0);
}

TEST(MapMatcher, RejectsInvalidSelectionWithoutCallingAlgorithms)
{
  auto algorithm = std::make_shared<FakeRegistration>();
  auto pipeline = std::make_shared<RegistrationPipeline>(
    algorithm, algorithm, MatchValidator{{0.1, 0.5, 10U, 5.0, 1.0, 0.0}},
    RegistrationPipelineConfig{0.5});
  MapMatcher matcher{mapSnapshot(), pipeline, {0.0, 1000U}};
  EXPECT_FALSE(matcher.match({}, {}).success);
  EXPECT_EQ(algorithm->calls, 0);
}

}  // namespace
}  // namespace a2w_fastlio_localization
