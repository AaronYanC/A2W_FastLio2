#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "a2w_fastlio_common/point_types.hpp"

namespace a2w_fastlio_common
{

class ScanDescriptor
{
public:
  ScanDescriptor(std::size_t rings, std::size_t sectors, std::vector<float> values)
  : rings_(rings), sectors_(sectors), values_(std::move(values))
  {
    if (rings_ == 0U || sectors_ == 0U) {
      throw std::invalid_argument("scan descriptor dimensions must be greater than zero");
    }
    if (rings_ > std::numeric_limits<std::size_t>::max() / sectors_ ||
      values_.size() != rings_ * sectors_)
    {
      throw std::invalid_argument("scan descriptor value count does not match its dimensions");
    }
    if (!std::all_of(values_.begin(), values_.end(), [](float value) {
        return std::isfinite(value);
      }))
    {
      throw std::invalid_argument("scan descriptor values must be finite");
    }
  }

  std::size_t rings() const noexcept {return rings_;}
  std::size_t sectors() const noexcept {return sectors_;}
  const std::vector<float> & values() const noexcept {return values_;}

private:
  std::size_t rings_;
  std::size_t sectors_;
  std::vector<float> values_;
};

struct LoopCandidate
{
  std::uint64_t keyframe_id{0};
  std::size_t rank{0};
  double distance{std::numeric_limits<double>::infinity()};
  double yaw_hint_rad{0.0};
};

struct CandidateFilter
{
  std::uint64_t max_inclusive_id{std::numeric_limits<std::uint64_t>::max()};
  std::size_t exclude_recent{0};
};

class PlaceRecognition
{
public:
  virtual ~PlaceRecognition() = default;
  virtual ScanDescriptor describe(const CloudConstPtr & cloud) const = 0;
};

class DescriptorIndex
{
public:
  virtual ~DescriptorIndex() = default;
  virtual void add(std::uint64_t keyframe_id, const ScanDescriptor & descriptor) = 0;
  virtual std::vector<LoopCandidate> queryTopK(
    const ScanDescriptor & query, std::size_t k,
    const CandidateFilter & filter) const = 0;
};

}  // namespace a2w_fastlio_common
