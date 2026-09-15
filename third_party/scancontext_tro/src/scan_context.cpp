// Adapted from engcang/scancontext_tro at
// c8ef5b496a159cdfd7fa4761121178f25cd0a6bb under CC BY-NC-SA 4.0.

#include "scancontext_tro/scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace scancontext_tro
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

bool finitePoint(const pcl::PointXYZI & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

std::size_t clampedBin(double value, double unit, std::size_t count)
{
  const auto one_based = static_cast<long long>(std::ceil(value / unit));
  const auto clamped = std::clamp<long long>(one_based, 1LL, static_cast<long long>(count));
  return static_cast<std::size_t>(clamped - 1LL);
}

}  // namespace

std::vector<float> makeScanContext(
  const pcl::PointCloud<pcl::PointXYZI> & cloud, const Config & config)
{
  if (config.rings == 0U || config.sectors == 0U ||
    !std::isfinite(config.max_radius_m) || config.max_radius_m <= 0.0 ||
    !std::isfinite(config.sensor_height_m))
  {
    throw std::invalid_argument("invalid Scan Context configuration");
  }
  if (cloud.empty()) {
    throw std::invalid_argument("cannot describe an empty point cloud");
  }

  std::vector<float> descriptor(config.rings * config.sectors, 0.0F);
  const double ring_gap = config.max_radius_m / static_cast<double>(config.rings);
  const double sector_angle = kTwoPi / static_cast<double>(config.sectors);

  for (const auto & point : cloud.points) {
    if (!finitePoint(point)) {
      throw std::invalid_argument("Scan Context input contains a non-finite point");
    }
    const double radius = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    if (radius > config.max_radius_m) {
      continue;
    }

    double azimuth = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x));
    if (azimuth < 0.0) {
      azimuth += kTwoPi;
    }
    const auto ring = clampedBin(radius, ring_gap, config.rings);
    const auto sector = clampedBin(azimuth, sector_angle, config.sectors);
    const auto index = ring * config.sectors + sector;
    const auto encoded_height = static_cast<float>(point.z + config.sensor_height_m);
    descriptor[index] = std::max(descriptor[index], encoded_height);
  }
  return descriptor;
}

}  // namespace scancontext_tro
