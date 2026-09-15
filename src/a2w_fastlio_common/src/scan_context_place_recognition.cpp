#include "a2w_fastlio_common/scan_context_place_recognition.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "scancontext_tro/scan_context.hpp"

namespace a2w_fastlio_common
{

ScanContextPlaceRecognition::ScanContextPlaceRecognition(ScanContextConfig config)
: config_(config)
{
  if (config_.rings == 0U || config_.sectors == 0U ||
    !std::isfinite(config_.max_radius_m) || config_.max_radius_m <= 0.0 ||
    !std::isfinite(config_.sensor_height_m))
  {
    throw std::invalid_argument("invalid Scan Context configuration");
  }
}

ScanDescriptor ScanContextPlaceRecognition::describe(const CloudConstPtr & cloud) const
{
  if (!cloud) {
    throw std::invalid_argument("Scan Context cloud pointer must not be null");
  }
  scancontext_tro::Config upstream_config{
    config_.rings, config_.sectors, config_.max_radius_m, config_.sensor_height_m};
  return ScanDescriptor{
    config_.rings, config_.sectors,
    scancontext_tro::makeScanContext(*cloud, upstream_config)};
}

}  // namespace a2w_fastlio_common
