#pragma once

#include <filesystem>

#include "a2w_fastlio_map/map_bundle.hpp"

namespace a2w_fastlio_map
{

class MapBundleWriter
{
public:
  explicit MapBundleWriter(BundleFailurePoint failure_point = BundleFailurePoint::kNone);
  BundleWriteResult write(
    const std::filesystem::path & destination, const MapBundleData & data) const;

private:
  BundleFailurePoint failure_point_;
};

}  // namespace a2w_fastlio_map
