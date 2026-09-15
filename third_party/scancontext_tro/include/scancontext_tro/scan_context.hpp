#pragma once

#include <cstddef>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace scancontext_tro
{

struct Config
{
  std::size_t rings;
  std::size_t sectors;
  double max_radius_m;
  double sensor_height_m;
};

std::vector<float> makeScanContext(
  const pcl::PointCloud<pcl::PointXYZI> & cloud, const Config & config);

}  // namespace scancontext_tro
