#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "a2w_fastlio_mapping/loop_pipeline.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

using namespace a2w_fastlio_common;

KeyFrame frame(std::uint64_t id)
{
  KeyFrame result;
  result.id = id;
  result.stamp_ns = static_cast<std::int64_t>(id + 1U) * 1000000000LL;
  for (int point_index = 0; point_index < 40; ++point_index) {
    PointT point;
    point.x = 0.1F * static_cast<float>(point_index);
    point.y = 0.2F * static_cast<float>(point_index % 5);
    point.z = 0.1F * static_cast<float>(point_index % 3);
    result.body_cloud->push_back(point);
  }
  return result;
}

class FakePlaceRecognition final : public PlaceRecognition
{
public:
  ScanDescriptor describe(const CloudConstPtr &) const override
  {
    return ScanDescriptor{1U, 1U, {1.0F}};
  }
};

class ScriptedIndex final : public DescriptorIndex
{
public:
  void add(std::uint64_t id, const ScanDescriptor &) override {added_ids.push_back(id);}

  std::vector<LoopCandidate> queryTopK(
    const ScanDescriptor &, std::size_t k, const CandidateFilter & filter) const override
  {
    ++query_calls;
    last_k = k;
    last_filter = filter;
    if (query_calls <= empty_query_count) {
      return {};
    }
    return candidates;
  }

  mutable std::size_t query_calls{0U};
  mutable std::size_t last_k{0U};
  mutable CandidateFilter last_filter{};
  std::size_t empty_query_count{1U};
  std::vector<LoopCandidate> candidates{};
  std::vector<std::uint64_t> added_ids{};
};

RegistrationResult successfulRegistration()
{
  RegistrationResult result;
  result.success = true;
  result.converged = true;
  result.fitness = 0.0;
  result.elapsed_ms = 1.0;
  return result;
}

class ScriptedCoarse final : public CoarseRegistration
{
public:
  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr &,
    const std::optional<Pose3d> &) const override
  {
    const auto call = calls++;
    if (call < results.size()) {
      return results[call];
    }
    return successfulRegistration();
  }
  mutable std::size_t calls{0U};
  std::vector<RegistrationResult> results{};
};

class RecordingFine final : public FineRegistration
{
public:
  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr &,
    const std::optional<Pose3d> &) const override
  {
    ++calls;
    return successfulRegistration();
  }
  mutable std::size_t calls{0U};
};

struct Fixture
{
  std::shared_ptr<FakePlaceRecognition> place{std::make_shared<FakePlaceRecognition>()};
  std::shared_ptr<ScriptedIndex> index{std::make_shared<ScriptedIndex>()};
  std::shared_ptr<ScriptedCoarse> coarse{std::make_shared<ScriptedCoarse>()};
  std::shared_ptr<RecordingFine> fine{std::make_shared<RecordingFine>()};
  std::shared_ptr<RegistrationPipeline> registration{
    std::make_shared<RegistrationPipeline>(
      coarse, fine,
      MatchValidator{MatchValidationConfig{0.1, 0.8, 30U, 5.0, 1.0, 0.05}},
      RegistrationPipelineConfig{0.25})};
  std::shared_ptr<PoseGraphOptimizer> graph{
    std::make_shared<PoseGraphOptimizer>(PoseGraphConfig{})};

  LoopPipeline make(LoopPipelineConfig config = {})
  {
    return LoopPipeline{
      place, index, registration, graph, LocalMapBuilder{LocalMapConfig{0.0, 10000U}}, config};
  }
};

void seed(LoopPipeline & pipeline)
{
  EXPECT_FALSE(pipeline.process(frame(0U)).empty());
  EXPECT_FALSE(pipeline.process(frame(1U)).empty());
}

TEST(LoopPipeline, IteratesTopKUntilOneCandidateIsAccepted)
{
  Fixture fixture;
  auto failure = RegistrationResult{};
  failure.rejection_reason = "coarse_rejected";
  fixture.coarse->results = {failure, successfulRegistration()};
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.30, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{5U, 0U, 0U, 0U, 1U, 10U, 1.0});
  seed(pipeline);

  const auto events = pipeline.process(frame(2U));

  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].candidate_id, 0U);
  EXPECT_EQ(events[0].reason, "coarse_registration_failed:coarse_rejected");
  EXPECT_EQ(events[1].candidate_id, 1U);
  EXPECT_TRUE(events[1].accepted);
  EXPECT_EQ(fixture.coarse->calls, 2U);
  EXPECT_EQ(fixture.fine->calls, 1U);
  EXPECT_EQ(events[1].graph_factor_count, 4U);
  EXPECT_EQ(fixture.index->last_k, 5U);
}

TEST(LoopPipeline, RejectedCandidatesDoNotAddLoopFactors)
{
  Fixture fixture;
  auto failure = RegistrationResult{};
  failure.rejection_reason = "no_match";
  fixture.coarse->results = {failure, failure};
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.30, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 1.0});
  seed(pipeline);

  const auto events = pipeline.process(frame(2U));

  ASSERT_EQ(events.size(), 2U);
  EXPECT_FALSE(events[0].accepted);
  EXPECT_FALSE(events[1].accepted);
  EXPECT_EQ(events.back().graph_factor_count, 3U);
}

TEST(LoopPipeline, RejectsCandidatesBeyondConfiguredDescriptorDistance)
{
  Fixture fixture;
  fixture.index->candidates = {LoopCandidate{0U, 0U, 0.30, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 0.20});
  seed(pipeline);

  const auto events = pipeline.process(frame(2U));

  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().reason, "no_loop_candidates");
  EXPECT_EQ(fixture.coarse->calls, 0U);
}

TEST(LoopPipeline, AcceptsAtMostOneLoopAndHonorsPolicyWindow)
{
  Fixture fixture;
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.30, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 2U, 10U, 1.0});
  seed(pipeline);
  const auto accepted = pipeline.process(frame(2U));
  ASSERT_EQ(accepted.size(), 1U);
  ASSERT_TRUE(accepted.front().accepted);

  const auto cooldown = pipeline.process(frame(3U));

  ASSERT_EQ(cooldown.size(), 1U);
  EXPECT_FALSE(cooldown.front().accepted);
  EXPECT_EQ(cooldown.front().reason, "loop_policy_cooldown");
  EXPECT_EQ(fixture.coarse->calls, 1U);
}

TEST(LoopPipeline, QueueIsBoundedUnderConcurrentProducers)
{
  Fixture fixture;
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 8U, 1.0});
  std::atomic<std::size_t> accepted{0U};
  std::vector<std::thread> producers;
  for (std::uint64_t id = 0U; id < 32U; ++id) {
    producers.emplace_back([&pipeline, &accepted, id]() {
        if (pipeline.enqueue(frame(id))) {
          ++accepted;
        }
      });
  }
  for (auto & producer : producers) {
    producer.join();
  }

  EXPECT_EQ(accepted.load(), 8U);
  EXPECT_EQ(pipeline.pending(), 8U);
}

TEST(LoopPipeline, ProcessNextConsumesOneQueuedFrameOutsideProducerPath)
{
  Fixture fixture;
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 2U, 1.0});
  ASSERT_TRUE(pipeline.enqueue(frame(0U)));
  ASSERT_TRUE(pipeline.enqueue(frame(1U)));

  const auto first = pipeline.processNext();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(pipeline.pending(), 1U);
  const auto second = pipeline.processNext();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(pipeline.pending(), 0U);
  EXPECT_FALSE(pipeline.processNext().has_value());
}

TEST(LoopPipeline, RejectsOutOfOrderFramesWithoutGraphMutation)
{
  Fixture fixture;
  auto pipeline = fixture.make();
  seed(pipeline);
  const auto events = pipeline.process(frame(1U));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().reason, "keyframe_id_not_contiguous");
  EXPECT_EQ(events.front().graph_factor_count, 2U);
}

}  // namespace
}  // namespace a2w_fastlio_mapping
