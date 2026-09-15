#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "a2w_fastlio_localization/global_relocalizer.hpp"

namespace a2w_fastlio_localization
{
namespace
{

using namespace a2w_fastlio_common;

CloudPtr cloud()
{
  auto result = CloudPtr{new Cloud{}};
  for (int index = 0; index < 40; ++index) {
    PointT point;
    point.x = 0.1F * static_cast<float>(index);
    point.y = 0.2F * static_cast<float>(index % 5);
    point.z = 0.1F * static_cast<float>(index % 3);
    result->push_back(point);
  }
  return result;
}

class ScriptedIndex final : public DescriptorIndex
{
public:
  void add(std::uint64_t, const ScanDescriptor &) override {}
  std::vector<LoopCandidate> queryTopK(
    const ScanDescriptor &, const std::size_t k, const CandidateFilter & filter) const override
  {
    last_k = k;
    last_filter = filter;
    return candidates;
  }
  std::vector<LoopCandidate> candidates;
  mutable std::size_t last_k{0U};
  mutable CandidateFilter last_filter{};
};

class Provider final : public KeyFrameProvider
{
public:
  std::optional<KeyFrame> get(const std::uint64_t id) const override
  {
    for (const auto & item : frames) {
      if (item.id == id) {
        return item;
      }
    }
    return std::nullopt;
  }
  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    return frames.empty() ? std::nullopt :
      std::optional<std::pair<std::uint64_t, std::uint64_t>>{{0U, frames.back().id}};
  }
  std::vector<KeyFrame> frames;
};

RegistrationResult usable(const double x)
{
  RegistrationResult result;
  result.success = true;
  result.converged = true;
  result.transform.translation.x() = x;
  result.fitness = 0.0;
  result.elapsed_ms = 0.1;
  return result;
}

class Coarse final : public CoarseRegistration
{
public:
  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr & target,
    const std::optional<Pose3d> & hint) const override
  {
    ++calls;
    EXPECT_TRUE(hint.has_value());
    return usable(target->front().x);
  }
  mutable std::size_t calls{0U};
};

class Fine final : public FineRegistration
{
public:
  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr & target,
    const std::optional<Pose3d> &) const override
  {
    ++calls;
    const double target_x = target->front().x;
    // The rank-zero repeated structure deliberately converges to a false transform.
    return usable(target_x > 50.0 ? 0.0 : target_x);
  }
  mutable std::size_t calls{0U};
};

KeyFrame frame(const std::uint64_t id, const double x)
{
  KeyFrame result;
  result.id = id;
  result.optimized_pose.translation.x() = x;
  result.body_cloud = cloud();
  return result;
}

RelocalizationConfig config()
{
  RelocalizationConfig result;
  result.top_k = 5U;
  result.neighbor_keyframes_before = 0U;
  result.neighbor_keyframes_after = 0U;
  result.maximum_descriptor_distance = 1.0;
  result.minimum_score_margin = 0.05;
  result.descriptor_score_weight = 0.0;
  result.fitness_score_weight = 1.0;
  result.overlap_score_weight = 1.0;
  result.strong_maximum_fitness = 0.01;
  result.strong_minimum_overlap = 0.9;
  result.strong_minimum_correspondences = 30U;
  return result;
}

GlobalRelocalizer make(
  const std::shared_ptr<ScriptedIndex> & index, const std::shared_ptr<Provider> & provider,
  const std::shared_ptr<Coarse> & coarse, const std::shared_ptr<Fine> & fine)
{
  return GlobalRelocalizer{
    index, provider, coarse, fine,
    MatchValidator{MatchValidationConfig{0.05, 0.8, 30U, 1000.0, 3.0, 0.0}},
    LocalMapConfig{0.0, 10000U}, RegistrationPipelineConfig{0.25}};
}

TEST(GlobalRelocalizer, AuditsTopKAndGeometryBeatsScanContextRank)
{
  const auto index = std::make_shared<ScriptedIndex>();
  index->candidates = {
    {0U, 0U, 0.01, 0.0}, {99U, 1U, 0.02, 0.0}, {1U, 2U, 0.20, 0.0}};
  const auto provider = std::make_shared<Provider>();
  provider->frames = {frame(0U, 100.0), frame(1U, 5.0)};
  const auto coarse = std::make_shared<Coarse>();
  const auto fine = std::make_shared<Fine>();

  const auto result = make(index, provider, coarse, fine).evaluate(
    cloud(), ScanDescriptor{1U, 1U, {1.0F}}, config());

  ASSERT_EQ(result.audits.size(), 3U);
  EXPECT_EQ(result.audits[0].reason, "fitness_above_limit");
  EXPECT_EQ(result.audits[1].reason, "candidate_local_map_invalid");
  EXPECT_TRUE(result.audits[2].accepted);
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.strong);
  EXPECT_EQ(result.candidate_id, 1U);
  EXPECT_DOUBLE_EQ(result.map_body.translation.x(), 5.0);
  EXPECT_EQ(coarse->calls, 2U);
  EXPECT_EQ(fine->calls, 2U);
  EXPECT_EQ(index->last_k, 5U);
  EXPECT_EQ(index->last_filter.exclude_recent, 0U);
}

TEST(GlobalRelocalizer, RejectsTwoGeometricallyValidRepeatedStructuresAsAmbiguous)
{
  const auto index = std::make_shared<ScriptedIndex>();
  index->candidates = {{0U, 0U, 0.01, 0.0}, {1U, 1U, 0.02, 0.0}};
  const auto provider = std::make_shared<Provider>();
  provider->frames = {frame(0U, 0.0), frame(1U, 5.0)};
  const auto coarse = std::make_shared<Coarse>();
  const auto fine = std::make_shared<Fine>();

  const auto result = make(index, provider, coarse, fine).evaluate(
    cloud(), ScanDescriptor{1U, 1U, {1.0F}}, config());

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "candidate_score_ambiguous");
  ASSERT_EQ(result.audits.size(), 2U);
  EXPECT_TRUE(result.audits[0].accepted);
  EXPECT_TRUE(result.audits[1].accepted);
}

TEST(GlobalRelocalizer, RejectsInvalidConfigurationAndEmptyInputs)
{
  const auto index = std::make_shared<ScriptedIndex>();
  const auto provider = std::make_shared<Provider>();
  const auto coarse = std::make_shared<Coarse>();
  const auto fine = std::make_shared<Fine>();
  auto invalid = config();
  invalid.top_k = 1U;
  EXPECT_THROW(
    make(index, provider, coarse, fine).evaluate(
      cloud(), ScanDescriptor{1U, 1U, {1.0F}}, invalid), std::invalid_argument);
  EXPECT_THROW(
    make(index, provider, coarse, fine).evaluate(
      CloudPtr{new Cloud{}}, ScanDescriptor{1U, 1U, {1.0F}}, config()),
    std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_localization
