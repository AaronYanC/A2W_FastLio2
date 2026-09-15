#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "a2w_fastlio_common/local_map_builder.hpp"
#include "a2w_fastlio_common/place_recognition.hpp"
#include "a2w_fastlio_common/registration_pipeline.hpp"
#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

namespace a2w_fastlio_mapping
{

struct LoopPipelineConfig
{
  std::size_t top_k{5U};
  std::size_t exclude_recent{30U};
  std::size_t local_map_before{5U};
  std::size_t local_map_after{5U};
  std::size_t minimum_keyframes_between_accepted_loops{10U};
  std::size_t queue_capacity{32U};
};

struct LoopPipelineEvent
{
  std::uint64_t current_id{0U};
  std::uint64_t candidate_id{0U};
  std::size_t candidate_rank{0U};
  bool accepted{false};
  std::string reason{};
  a2w_fastlio_common::RegistrationResult registration{};
  std::size_t graph_factor_count{0U};
};

class LoopPipeline
{
public:
  LoopPipeline(
    std::shared_ptr<a2w_fastlio_common::PlaceRecognition> place_recognition,
    std::shared_ptr<a2w_fastlio_common::DescriptorIndex> descriptor_index,
    std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> registration,
    std::shared_ptr<PoseGraphOptimizer> graph,
    a2w_fastlio_common::LocalMapBuilder local_map_builder,
    LoopPipelineConfig config = {});

  std::vector<LoopPipelineEvent> process(const a2w_fastlio_common::KeyFrame & current);
  bool enqueue(a2w_fastlio_common::KeyFrame keyframe);
  std::optional<std::vector<LoopPipelineEvent>> processNext();
  std::size_t pending() const noexcept;

private:
  class StoredKeyFrameProvider;

  std::shared_ptr<a2w_fastlio_common::PlaceRecognition> place_recognition_;
  std::shared_ptr<a2w_fastlio_common::DescriptorIndex> descriptor_index_;
  std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> registration_;
  std::shared_ptr<PoseGraphOptimizer> graph_;
  a2w_fastlio_common::LocalMapBuilder local_map_builder_;
  LoopPipelineConfig config_;
  std::map<std::uint64_t, a2w_fastlio_common::KeyFrame> keyframes_;
  std::optional<std::uint64_t> last_accepted_loop_id_{};
  mutable std::mutex queue_mutex_;
  std::deque<a2w_fastlio_common::KeyFrame> queue_;
};

}  // namespace a2w_fastlio_mapping
