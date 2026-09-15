#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include "a2w_fastlio_map/map_manager.hpp"

namespace a2w_fastlio_map
{

inline constexpr const char * kMapBundleSchema = "a2w_map_bundle_v1";

struct MapBundleMetadata
{
  std::string schema{kMapBundleSchema};
  std::string bundle_uuid;
  std::string created_utc;
  std::string map_frame{"map"};
  std::string odom_frame{"camera_init"};
  std::string tracking_frame{"body"};
  std::string point_type{"PointXYZI"};
  double global_map_leaf_m{0.0};
  std::string frontend_revision;
  std::map<std::string, std::string> dependency_revisions;
  std::map<std::string, std::string> config_hashes;
  std::string creation_status{"offline_verified"};
  std::string hardware_validation_status{"pending"};
};

struct MapBundleData
{
  MapBundleMetadata metadata;
  std::vector<a2w_fastlio_common::KeyFrame> keyframes;
  std::vector<DescriptorRecord> descriptors;
  a2w_fastlio_common::CloudPtr global_map{new a2w_fastlio_common::Cloud{}};
  std::map<std::string, std::string> config_snapshots;
};

enum class ReadMode
{
  kReadOnly,
};

struct MapBundle
{
  std::filesystem::path root;
  ReadMode mode{ReadMode::kReadOnly};
  MapBundleData data;
};

struct BundleWriteResult
{
  bool success{false};
  std::string message;
  std::string bundle_uuid;
  std::size_t keyframe_count{0U};
  std::filesystem::path resolved_path;
};

enum class BundleFailurePoint
{
  kNone,
  kBeforeValidation,
  kAfterBackupRename,
};

}  // namespace a2w_fastlio_map
