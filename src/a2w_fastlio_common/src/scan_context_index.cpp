#include "a2w_fastlio_common/scan_context_index.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace a2w_fastlio_common
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

struct DescriptorMatch
{
  double distance;
  double yaw_hint_rad;
};

DescriptorMatch compareDescriptors(
  const ScanDescriptor & query, const ScanDescriptor & candidate)
{
  if (query.rings() != candidate.rings() || query.sectors() != candidate.sectors()) {
    throw std::invalid_argument("Scan Context descriptor dimensions do not match");
  }

  const auto rings = query.rings();
  const auto sectors = query.sectors();
  double best_distance = std::numeric_limits<double>::infinity();
  std::size_t best_shift = 0U;

  for (std::size_t shift = 0U; shift < sectors; ++shift) {
    double cosine_sum = 0.0;
    std::size_t effective_columns = 0U;
    for (std::size_t query_sector = 0U; query_sector < sectors; ++query_sector) {
      const auto candidate_sector = (query_sector + shift) % sectors;
      double dot = 0.0;
      double query_norm_sq = 0.0;
      double candidate_norm_sq = 0.0;
      for (std::size_t ring = 0U; ring < rings; ++ring) {
        const double query_value = query.values()[ring * sectors + query_sector];
        const double candidate_value = candidate.values()[ring * sectors + candidate_sector];
        dot += query_value * candidate_value;
        query_norm_sq += query_value * query_value;
        candidate_norm_sq += candidate_value * candidate_value;
      }
      if (query_norm_sq > 0.0 && candidate_norm_sq > 0.0) {
        cosine_sum += dot / std::sqrt(query_norm_sq * candidate_norm_sq);
        ++effective_columns;
      }
    }
    const double distance = effective_columns == 0U ?
      1.0 : 1.0 - cosine_sum / static_cast<double>(effective_columns);
    if (distance < best_distance) {
      best_distance = distance;
      best_shift = shift;
    }
  }

  double yaw = static_cast<double>(best_shift) * kTwoPi / static_cast<double>(sectors);
  if (yaw > kTwoPi / 2.0) {
    yaw -= kTwoPi;
  }
  return DescriptorMatch{std::max(0.0, best_distance), yaw};
}

bool eligible(std::uint64_t id, const CandidateFilter & filter)
{
  if (id > filter.max_inclusive_id) {
    return false;
  }
  if (filter.exclude_recent == 0U) {
    return true;
  }
  const auto exclusion = static_cast<std::uint64_t>(filter.exclude_recent);
  if (filter.max_inclusive_id < exclusion) {
    return false;
  }
  return id <= filter.max_inclusive_id - exclusion;
}

}  // namespace

void ScanContextIndex::add(std::uint64_t keyframe_id, const ScanDescriptor & descriptor)
{
  if (std::any_of(entries_.begin(), entries_.end(), [keyframe_id](const Entry & entry) {
      return entry.keyframe_id == keyframe_id;
    }))
  {
    throw std::invalid_argument("Scan Context keyframe ID already exists");
  }
  if (!entries_.empty() &&
    (entries_.front().descriptor.rings() != descriptor.rings() ||
    entries_.front().descriptor.sectors() != descriptor.sectors()))
  {
    throw std::invalid_argument("Scan Context descriptor dimensions do not match the index");
  }
  entries_.push_back(Entry{keyframe_id, descriptor});
}

std::vector<LoopCandidate> ScanContextIndex::queryTopK(
  const ScanDescriptor & query, std::size_t k, const CandidateFilter & filter) const
{
  if (k == 0U) {
    throw std::invalid_argument("Scan Context Top-K must be greater than zero");
  }
  if (entries_.empty()) {
    return {};
  }
  if (entries_.front().descriptor.rings() != query.rings() ||
    entries_.front().descriptor.sectors() != query.sectors())
  {
    throw std::invalid_argument("Scan Context query dimensions do not match the index");
  }

  std::vector<LoopCandidate> candidates;
  candidates.reserve(entries_.size());
  for (const auto & entry : entries_) {
    if (!eligible(entry.keyframe_id, filter)) {
      continue;
    }
    const auto match = compareDescriptors(query, entry.descriptor);
    candidates.push_back(LoopCandidate{entry.keyframe_id, 0U, match.distance, match.yaw_hint_rad});
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
    if (lhs.distance != rhs.distance) {
      return lhs.distance < rhs.distance;
    }
    return lhs.keyframe_id < rhs.keyframe_id;
  });
  if (candidates.size() > k) {
    candidates.resize(k);
  }
  for (std::size_t rank = 0U; rank < candidates.size(); ++rank) {
    candidates[rank].rank = rank;
  }
  return candidates;
}

std::size_t ScanContextIndex::size() const noexcept
{
  return entries_.size();
}

}  // namespace a2w_fastlio_common
