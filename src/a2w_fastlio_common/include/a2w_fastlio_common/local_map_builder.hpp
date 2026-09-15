#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "a2w_fastlio_common/types.hpp"

namespace a2w_fastlio_common
{

class KeyFrameProvider
{
public:
  virtual ~KeyFrameProvider() = default;
  virtual std::optional<KeyFrame> get(std::uint64_t id) const = 0;
  virtual std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const = 0;
};

struct LocalMapConfig
{
  double voxel_leaf_m{0.2};
  std::size_t max_points{200000U};
};

struct LocalMapResult
{
  CloudPtr cloud{new Cloud{}};
  std::vector<std::uint64_t> included_ids{};
};

class LocalMapBuilder
{
public:
  explicit LocalMapBuilder(LocalMapConfig config);

  LocalMapResult build(
    std::uint64_t center_id, std::size_t before, std::size_t after,
    const KeyFrameProvider & provider) const;

private:
  LocalMapConfig config_;
};

}  // namespace a2w_fastlio_common
