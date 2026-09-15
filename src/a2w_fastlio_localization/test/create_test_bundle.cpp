#include <cstdint>
#include <filesystem>
#include <iostream>

#include "a2w_fastlio_map/map_bundle_writer.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: create_test_bundle OUTPUT_PATH\n";
    return 2;
  }
  a2w_fastlio_map::MapBundleData data;
  data.metadata.bundle_uuid = "00000000-0000-4000-8000-000000000007";
  data.metadata.created_utc = "2026-09-15T00:00:00Z";
  data.metadata.frontend_revision = "offline-test-fixture";
  data.metadata.dependency_revisions = {{"fixture", "stage7"}};
  a2w_fastlio_common::KeyFrame frame;
  frame.id = 0U;
  frame.stamp_ns = 1'000'000'000LL;
  frame.optimized_pose.translation.x() = 5.0;
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      for (int z = 0; z < 4; ++z) {
        a2w_fastlio_common::PointT point;
        point.x = 0.4F * static_cast<float>(x) + 0.03F * static_cast<float>(y * z);
        point.y = 0.35F * static_cast<float>(y) + 0.02F * static_cast<float>(x * z);
        point.z = 0.3F * static_cast<float>(z) + 0.01F * static_cast<float>(x * y);
        point.intensity = static_cast<float>(x * 20 + y * 4 + z);
        frame.body_cloud->push_back(point);
        point.x += 5.0F;
        data.global_map->push_back(point);
      }
    }
  }
  data.keyframes.push_back(frame);
  data.descriptors.push_back({
    0U, a2w_fastlio_common::ScanDescriptor{2U, 2U, {0.0F, 1.0F, 2.0F, 3.0F}}});
  data.config_snapshots = {
    {"mapping_effective.yaml", "fixture: stage7\n"},
    {"scan_context.yaml", "rings: 2\n"},
    {"registration.yaml", "offline: true\n"},
  };
  const auto result = a2w_fastlio_map::MapBundleWriter{}.write(argv[1], data);
  if (!result.success) {
    std::cerr << result.message << '\n';
    return 1;
  }
  return 0;
}
