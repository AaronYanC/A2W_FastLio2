#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "a2w_fastlio_common/place_recognition.hpp"

namespace a2w_fastlio_common
{

class ScanContextIndex final : public DescriptorIndex
{
public:
  void add(std::uint64_t keyframe_id, const ScanDescriptor & descriptor) override;
  std::vector<LoopCandidate> queryTopK(
    const ScanDescriptor & query, std::size_t k,
    const CandidateFilter & filter) const override;

  std::size_t size() const noexcept;

private:
  struct Entry
  {
    std::uint64_t keyframe_id;
    ScanDescriptor descriptor;
  };

  std::vector<Entry> entries_;
};

}  // namespace a2w_fastlio_common
