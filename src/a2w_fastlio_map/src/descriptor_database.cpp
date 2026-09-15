#include "a2w_fastlio_map/descriptor_database.hpp"

#include <stdexcept>

namespace a2w_fastlio_map
{

void DescriptorDatabase::add(
  const std::uint64_t keyframe_id,
  const a2w_fastlio_common::ScanDescriptor & descriptor)
{
  std::lock_guard<std::mutex> lock{mutex_};
  if (keyframe_id != records_.size()) {
    throw std::invalid_argument{"descriptor IDs must be contiguous and unique"};
  }
  index_.add(keyframe_id, descriptor);
  records_.push_back({keyframe_id, descriptor});
}

std::vector<a2w_fastlio_common::LoopCandidate> DescriptorDatabase::queryTopK(
  const a2w_fastlio_common::ScanDescriptor & query, const std::size_t k,
  const a2w_fastlio_common::CandidateFilter & filter) const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return index_.queryTopK(query, k, filter);
}

std::vector<DescriptorRecord> DescriptorDatabase::records() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return records_;
}

std::size_t DescriptorDatabase::size() const noexcept
{
  std::lock_guard<std::mutex> lock{mutex_};
  return records_.size();
}

}  // namespace a2w_fastlio_map
