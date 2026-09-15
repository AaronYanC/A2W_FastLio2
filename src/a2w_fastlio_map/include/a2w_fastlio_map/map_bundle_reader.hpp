#pragma once

#include <filesystem>

#include "a2w_fastlio_map/map_bundle.hpp"

namespace a2w_fastlio_map
{

class MapBundleReader
{
public:
  MapBundle read(
    const std::filesystem::path & root,
    ReadMode mode = ReadMode::kReadOnly) const;
};

}  // namespace a2w_fastlio_map
