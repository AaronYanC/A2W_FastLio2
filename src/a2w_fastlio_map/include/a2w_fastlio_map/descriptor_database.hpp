#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

#include "a2w_fastlio_common/place_recognition.hpp"
#include "a2w_fastlio_common/scan_context_index.hpp"

namespace a2w_fastlio_map
{

struct DescriptorRecord
{
  std::uint64_t keyframe_id{0U};
  a2w_fastlio_common::ScanDescriptor descriptor{1U, 1U, {0.0F}};
};

class DescriptorDatabase final : public a2w_fastlio_common::DescriptorIndex
{
public:
  void add(
    std::uint64_t keyframe_id,
    const a2w_fastlio_common::ScanDescriptor & descriptor) override;
  std::vector<a2w_fastlio_common::LoopCandidate> queryTopK(
    const a2w_fastlio_common::ScanDescriptor & query, std::size_t k,
    const a2w_fastlio_common::CandidateFilter & filter) const override;
  std::vector<DescriptorRecord> records() const;
  std::size_t size() const noexcept;

private:
  mutable std::mutex mutex_;
  a2w_fastlio_common::ScanContextIndex index_;
  std::vector<DescriptorRecord> records_;
};

}  // namespace a2w_fastlio_map
