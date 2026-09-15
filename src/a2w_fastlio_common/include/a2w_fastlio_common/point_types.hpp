#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace a2w_fastlio_common
{

using PointT = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;

}  // namespace a2w_fastlio_common
