#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "a2w_fastlio_common/local_map_builder.hpp"

namespace a2w_fastlio_map
{

class KeyFrameDatabase final : public a2w_fastlio_common::KeyFrameProvider
{
public:
  bool add(const a2w_fastlio_common::KeyFrame & keyframe);
  std::optional<a2w_fastlio_common::KeyFrame> get(std::uint64_t id) const override;
  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override;
  std::vector<a2w_fastlio_common::KeyFrame> ordered() const;
  std::size_t size() const noexcept;

private:
  static a2w_fastlio_common::KeyFrame clone(const a2w_fastlio_common::KeyFrame & keyframe);

  mutable std::mutex mutex_;
  std::vector<a2w_fastlio_common::KeyFrame> keyframes_;
};

}  // namespace a2w_fastlio_map
