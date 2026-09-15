#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include <a2w_fastlio_msgs/msg/global_tf_owner.hpp>
#include <a2w_fastlio_msgs/msg/localization_status.hpp>
#include <a2w_fastlio_msgs/msg/registration_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "a2w_fastlio_common/default_algorithm_suite.hpp"
#include "a2w_fastlio_common/global_tf_ownership.hpp"
#include "a2w_fastlio_localization/local_map_selector.hpp"
#include "a2w_fastlio_localization/global_relocalizer.hpp"
#include "a2w_fastlio_localization/localization_manager.hpp"
#include "a2w_fastlio_localization/localization_monitor.hpp"
#include "a2w_fastlio_localization/map_matcher.hpp"
#include "a2w_fastlio_localization/relocalization_worker.hpp"
#include "a2w_fastlio_map/descriptor_database.hpp"
#include "a2w_fastlio_map/keyframe_database.hpp"
#include "a2w_fastlio_map/map_bundle_reader.hpp"

namespace a2w_fastlio_localization
{
namespace
{

std::int64_t nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

builtin_interfaces::msg::Time stampMessage(const std::int64_t value)
{
  return static_cast<builtin_interfaces::msg::Time>(rclcpp::Time{value});
}

geometry_msgs::msg::Pose poseMessage(const a2w_fastlio_common::Pose3d & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.translation.x();
  result.position.y = pose.translation.y();
  result.position.z = pose.translation.z();
  result.orientation.w = pose.rotation.w();
  result.orientation.x = pose.rotation.x();
  result.orientation.y = pose.rotation.y();
  result.orientation.z = pose.rotation.z();
  return result;
}

a2w_fastlio_common::Pose3d compose(
  const a2w_fastlio_common::Pose3d & lhs, const a2w_fastlio_common::Pose3d & rhs)
{
  a2w_fastlio_common::Pose3d result;
  result.rotation = (lhs.rotation.normalized() * rhs.rotation.normalized()).normalized();
  result.translation = lhs.rotation.normalized() * rhs.translation + lhs.translation;
  return result;
}

a2w_fastlio_common::Pose3d inverse(const a2w_fastlio_common::Pose3d & pose)
{
  a2w_fastlio_common::Pose3d result;
  result.rotation = pose.rotation.normalized().conjugate();
  result.translation = -(result.rotation * pose.translation);
  return result;
}

struct AdaptedFrame
{
  bool success{false};
  std::string reason;
  a2w_fastlio_common::FrontendFrame frame;
};

AdaptedFrame adapt(
  const nav_msgs::msg::Odometry & odometry,
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & odom_frame,
  const std::string & tracking_frame)
{
  AdaptedFrame result;
  if (nanoseconds(odometry.header.stamp) != nanoseconds(cloud.header.stamp)) {
    result.reason = "timestamp_mismatch";
    return result;
  }
  if (odometry.header.frame_id != odom_frame || odometry.child_frame_id != tracking_frame ||
    cloud.header.frame_id != tracking_frame)
  {
    result.reason = "frame_contract_mismatch";
    return result;
  }
  const auto & p = odometry.pose.pose.position;
  const auto & q = odometry.pose.pose.orientation;
  result.frame.stamp_ns = nanoseconds(odometry.header.stamp);
  result.frame.odom_pose.translation = {p.x, p.y, p.z};
  result.frame.odom_pose.rotation = {q.w, q.x, q.y, q.z};
  if (!a2w_fastlio_common::isFinitePose(result.frame.odom_pose) ||
    result.frame.odom_pose.rotation.squaredNorm() <= 1.0e-12)
  {
    result.reason = "invalid_odometry_pose";
    return result;
  }
  result.frame.odom_pose.rotation.normalize();
  auto converted = a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{}};
  pcl::fromROSMsg(cloud, *converted);
  if (converted->empty()) {
    result.reason = "empty_body_cloud";
    return result;
  }
  result.frame.body_cloud = converted;
  result.success = true;
  return result;
}

}  // namespace

class LocalizationNode final : public rclcpp::Node
{
public:
  LocalizationNode() : Node("localization_node")
  {
    const auto bundle_path = declare_parameter<std::string>("map_bundle_path", "");
    if (bundle_path.empty() || !std::filesystem::path{bundle_path}.is_absolute()) {
      throw std::invalid_argument{"map_bundle_path must be a non-empty absolute path"};
    }
    bundle_ = a2w_fastlio_map::MapBundleReader{}.read(bundle_path);
    if (bundle_.mode != a2w_fastlio_map::ReadMode::kReadOnly) {
      throw std::runtime_error{"Localization requires a read-only Map Bundle"};
    }
    map_frame_ = declare_parameter<std::string>("map_frame", bundle_.data.metadata.map_frame);
    odom_frame_ = declare_parameter<std::string>("odom_frame", bundle_.data.metadata.odom_frame);
    tracking_frame_ = declare_parameter<std::string>(
      "tracking_frame", bundle_.data.metadata.tracking_frame);
    if (map_frame_ != bundle_.data.metadata.map_frame ||
      odom_frame_ != bundle_.data.metadata.odom_frame ||
      tracking_frame_ != bundle_.data.metadata.tracking_frame)
    {
      throw std::invalid_argument{"configured frames differ from Map Bundle metadata"};
    }

    a2w_fastlio_map::MapSnapshot snapshot;
    snapshot.keyframes = bundle_.data.keyframes;
    snapshot.descriptors = bundle_.data.descriptors;
    snapshot.global_map = bundle_.data.global_map;
    const auto algorithms = a2w_fastlio_common::createDefaultAlgorithmSuite(algorithmConfig());
    const auto local_map_config = localMapConfig();
    selector_ = std::make_shared<LocalMapSelector>(snapshot, selectorConfig());
    matcher_ = std::make_shared<MapMatcher>(snapshot, algorithms.registration, local_map_config);
    manager_ = std::make_unique<LocalizationManager>(managerConfig(),
      [this](const auto & frame, const auto & predicted) {
        const auto selection = selector_->select(predicted);
        return matcher_->match(frame, selection);
      });
    const auto monitor_config = monitorConfig();
    monitor_ = std::make_unique<LocalizationMonitor>(monitor_config);
    global_relocalization_enabled_ = declare_parameter<bool>(
      "global_relocalization.enabled", true);
    if (global_relocalization_enabled_) {
      relocalization_keyframes_ = std::make_shared<a2w_fastlio_map::KeyFrameDatabase>();
      relocalization_descriptors_ = std::make_shared<a2w_fastlio_map::DescriptorDatabase>();
      for (const auto & keyframe : bundle_.data.keyframes) {
        if (!relocalization_keyframes_->add(keyframe)) {
          throw std::runtime_error{"Map Bundle has invalid relocalization keyframes"};
        }
      }
      for (const auto & descriptor : bundle_.data.descriptors) {
        relocalization_descriptors_->add(descriptor.keyframe_id, descriptor.descriptor);
      }
      place_recognition_ = algorithms.place_recognition;
      relocalization_config_ = globalRelocalizationConfig();
      relocalizer_ = std::make_shared<GlobalRelocalizer>(
        relocalization_descriptors_, relocalization_keyframes_, algorithms.registration,
        local_map_config);
      relocalization_session_ = std::make_unique<RelocalizationSession>(
        sessionConfig(monitor_config.relocalization_timeout_ns));
      const auto queue_capacity = declare_parameter<int>(
        "global_relocalization.worker_queue_capacity", 1);
      const auto time_budget_ms = declare_parameter<int>(
        "global_relocalization.evaluation_time_budget_ms", 5000);
      if (queue_capacity <= 0 || time_budget_ms <= 0) {
        throw std::invalid_argument{"global relocalization worker settings must be positive"};
      }
      relocalization_time_budget_ms_ = static_cast<double>(time_budget_ms);
      relocalization_worker_ = std::make_unique<RelocalizationWorker>(
        [this](const auto & frame) {return evaluateRelocalization(frame);},
        [this](const auto & result) {handleRelocalizationResult(result);},
        static_cast<std::size_t>(queue_capacity));
    }

    const auto input_qos = qos("input", 20, false);
    const auto output_qos = qos("output", 20, false);
    const auto path_qos = qos("path", 1, true);
    const auto owner_qos = qos("owner", 10, true);
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/Odometry");
    const auto cloud_topic = declare_parameter<std::string>(
      "body_cloud_topic", "/cloud_registered_body");
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      declare_parameter<std::string>("pose_topic", "/localization/pose"), output_qos);
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("localized_odom_topic", "/localization/odom"), output_qos);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      declare_parameter<std::string>("path_topic", "/localization/path"), path_qos);
    status_publisher_ = create_publisher<a2w_fastlio_msgs::msg::RegistrationStatus>(
      declare_parameter<std::string>("registration_status_topic", "/localization/registration_status"),
      output_qos);
    monitor_status_publisher_ = create_publisher<a2w_fastlio_msgs::msg::LocalizationStatus>(
      declare_parameter<std::string>("localization_status_topic", "/localization/status"),
      output_qos);
    owner_topic_ = declare_parameter<std::string>(
      "global_tf_owner_topic", "/a2w_fastlio/global_tf_owner");
    owner_publisher_ = create_publisher<a2w_fastlio_msgs::msg::GlobalTfOwner>(owner_topic_, owner_qos);
    owner_subscription_ = create_subscription<a2w_fastlio_msgs::msg::GlobalTfOwner>(
      owner_topic_, owner_qos,
      std::bind(&LocalizationNode::observeOwner, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto queue = declare_parameter<int>("sync_queue_size", 20);
    if (queue <= 0) {
      throw std::invalid_argument{"sync_queue_size must be positive"};
    }
    odometry_subscriber_.subscribe(this, odom_topic, input_qos.get_rmw_qos_profile());
    cloud_subscriber_.subscribe(this, cloud_topic, input_qos.get_rmw_qos_profile());
    synchronizer_ = std::make_shared<Synchronizer>(
      odometry_subscriber_, cloud_subscriber_, static_cast<std::uint32_t>(queue));
    synchronizer_->registerCallback(std::bind(
      &LocalizationNode::inputCallback, this, std::placeholders::_1, std::placeholders::_2));

    owner_id_ = declare_parameter<std::string>("owner_id", "localization_backend");
    const auto conflict_ms = declare_parameter<int>("owner_conflict_window_ms", 1000);
    const auto heartbeat_ms = declare_parameter<int>("owner_heartbeat_ms", 250);
    if (owner_id_.empty() || conflict_ms <= 0 || heartbeat_ms <= 0) {
      throw std::invalid_argument{"invalid global TF owner configuration"};
    }
    owner_state_ = std::make_unique<a2w_fastlio_common::GlobalTfOwnerState>(
      a2w_fastlio_common::GlobalTfOwnerConfig{
        owner_id_, "localization", map_frame_, odom_frame_,
        static_cast<std::int64_t>(conflict_ms) * 1'000'000LL});
    owner_state_->start(now().nanoseconds());
    owner_timer_ = create_wall_timer(
      std::chrono::milliseconds{heartbeat_ms}, std::bind(&LocalizationNode::heartbeat, this));
    const auto max_path_poses = declare_parameter<int>("max_path_poses", 10000);
    if (max_path_poses <= 0) {
      throw std::invalid_argument{"max_path_poses must be positive"};
    }
    max_path_poses_ = static_cast<std::size_t>(max_path_poses);
    RCLCPP_INFO(get_logger(), "Loaded read-only Map Bundle %s", bundle_.root.c_str());
  }

  ~LocalizationNode() override
  {
    relocalization_worker_.reset();
  }

private:
  using Synchronizer = message_filters::TimeSynchronizer<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

  rclcpp::QoS qos(const std::string & prefix, int depth_default, bool transient)
  {
    const auto depth = declare_parameter<int>(prefix + "_qos_depth", depth_default);
    const auto reliability = declare_parameter<std::string>(
      prefix + "_qos_reliability", "reliable");
    const auto durability = declare_parameter<std::string>(
      prefix + "_qos_durability", transient ? "transient_local" : "volatile");
    if (depth <= 0 || (reliability != "reliable" && reliability != "best_effort") ||
      (durability != "volatile" && durability != "transient_local"))
    {
      throw std::invalid_argument{"invalid QoS configuration for " + prefix};
    }
    auto result = rclcpp::QoS{rclcpp::KeepLast{static_cast<std::size_t>(depth)}};
    reliability == "reliable" ? result.reliable() : result.best_effort();
    durability == "transient_local" ? result.transient_local() : result.durability_volatile();
    return result;
  }

  a2w_fastlio_common::DefaultAlgorithmSuiteConfig algorithmConfig()
  {
    a2w_fastlio_common::DefaultAlgorithmSuiteConfig config;
    const auto rings = declare_parameter<int>(
      "scan_context.rings", static_cast<int>(config.place_recognition.rings));
    const auto sectors = declare_parameter<int>(
      "scan_context.sectors", static_cast<int>(config.place_recognition.sectors));
    config.place_recognition.max_radius_m = declare_parameter<double>(
      "scan_context.max_radius_m", config.place_recognition.max_radius_m);
    config.place_recognition.sensor_height_m = declare_parameter<double>(
      "scan_context.sensor_height_m", config.place_recognition.sensor_height_m);
    if (rings <= 0 || sectors <= 0) {
      throw std::invalid_argument{"scan-context dimensions must be positive"};
    }
    config.place_recognition.rings = static_cast<std::size_t>(rings);
    config.place_recognition.sectors = static_cast<std::size_t>(sectors);
    config.coarse.normal_radius_m = declare_parameter<double>(
      "quatro.fpfh_normal_radius_m", config.coarse.normal_radius_m);
    config.coarse.feature_radius_m = declare_parameter<double>(
      "quatro.fpfh_radius_m", config.coarse.feature_radius_m);
    config.coarse.noise_bound_m = declare_parameter<double>(
      "quatro.noise_bound_m", config.coarse.noise_bound_m);
    config.coarse.rotation_gnc_factor = declare_parameter<double>(
      "quatro.rotation_gnc_factor", config.coarse.rotation_gnc_factor);
    config.coarse.rotation_cost_threshold = declare_parameter<double>(
      "quatro.rotation_cost_threshold", config.coarse.rotation_cost_threshold);
    config.coarse.rotation_max_iterations = declare_parameter<int>(
      "quatro.rotation_max_iterations", config.coarse.rotation_max_iterations);
    config.coarse.estimate_scale = declare_parameter<bool>(
      "quatro.estimate_scale", config.coarse.estimate_scale);
    config.coarse.optimized_matching = declare_parameter<bool>(
      "quatro.optimized_matching", config.coarse.optimized_matching);
    config.coarse.descriptor_distance_threshold = declare_parameter<double>(
      "quatro.descriptor_distance_threshold", config.coarse.descriptor_distance_threshold);
    config.coarse.maximum_correspondences = declare_parameter<int>(
      "quatro.maximum_correspondences", config.coarse.maximum_correspondences);
    const auto coarse_minimum = declare_parameter<int>(
      "quatro.minimum_points", static_cast<int>(config.coarse.minimum_points));
    config.fine.thread_count = declare_parameter<int>(
      "nano_gicp.thread_count", config.fine.thread_count);
    config.fine.correspondence_randomness = declare_parameter<int>(
      "nano_gicp.correspondence_randomness", config.fine.correspondence_randomness);
    config.fine.maximum_iterations = declare_parameter<int>(
      "nano_gicp.maximum_iterations", config.fine.maximum_iterations);
    config.fine.transformation_epsilon = declare_parameter<double>(
      "nano_gicp.transformation_epsilon", config.fine.transformation_epsilon);
    config.fine.rotation_epsilon = declare_parameter<double>(
      "nano_gicp.rotation_epsilon", config.fine.rotation_epsilon);
    config.fine.regularization_method = declare_parameter<int>(
      "nano_gicp.regularization_method", config.fine.regularization_method);
    config.fine.fitness_score_max_range_m = declare_parameter<double>(
      "nano_gicp.fitness_score_max_range_m", config.fine.fitness_score_max_range_m);
    const auto fine_minimum = declare_parameter<int>(
      "nano_gicp.minimum_points", static_cast<int>(config.fine.minimum_points));
    config.fine.maximum_correspondence_distance_m = declare_parameter<double>(
      "nano_gicp.maximum_correspondence_distance_m",
      config.fine.maximum_correspondence_distance_m);
    config.validation.maximum_fitness = declare_parameter<double>(
      "validation.maximum_fitness", config.validation.maximum_fitness);
    config.validation.minimum_overlap = declare_parameter<double>(
      "validation.minimum_overlap", config.validation.minimum_overlap);
    const auto minimum_correspondences = declare_parameter<int>(
      "validation.minimum_correspondences",
      static_cast<int>(config.validation.minimum_correspondences));
    config.validation.maximum_translation_jump_m = declare_parameter<double>(
      "validation.maximum_translation_jump_m", config.validation.maximum_translation_jump_m);
    config.validation.maximum_rotation_jump_rad = declare_parameter<double>(
      "validation.maximum_rotation_jump_rad", config.validation.maximum_rotation_jump_rad);
    config.validation.minimum_candidate_distance_separation = declare_parameter<double>(
      "validation.minimum_candidate_distance_separation",
      config.validation.minimum_candidate_distance_separation);
    config.evidence_distance_m = declare_parameter<double>(
      "pipeline.evidence_distance_m", config.evidence_distance_m);
    if (coarse_minimum <= 0 || fine_minimum <= 0 || minimum_correspondences <= 0) {
      throw std::invalid_argument{"registration point/count thresholds must be positive"};
    }
    config.coarse.minimum_points = static_cast<std::size_t>(coarse_minimum);
    config.fine.minimum_points = static_cast<std::size_t>(fine_minimum);
    config.validation.minimum_correspondences =
      static_cast<std::size_t>(minimum_correspondences);
    return config;
  }

  LocalMapSelectorConfig selectorConfig()
  {
    LocalMapSelectorConfig config;
    config.radius_m = declare_parameter<double>("local_map.selection_radius_m", config.radius_m);
    const auto maximum = declare_parameter<int>(
      "local_map.max_neighbors", static_cast<int>(config.max_neighbors));
    const auto minimum = declare_parameter<int>(
      "local_map.minimum_neighbors", static_cast<int>(config.minimum_neighbors));
    if (maximum <= 0 || minimum <= 0) {
      throw std::invalid_argument{"local-map neighbor thresholds must be positive"};
    }
    config.max_neighbors = static_cast<std::size_t>(maximum);
    config.minimum_neighbors = static_cast<std::size_t>(minimum);
    return config;
  }

  a2w_fastlio_common::LocalMapConfig localMapConfig()
  {
    a2w_fastlio_common::LocalMapConfig config;
    config.voxel_leaf_m = declare_parameter<double>("local_map.voxel_leaf_m", config.voxel_leaf_m);
    const auto max_points = declare_parameter<int>(
      "local_map.max_points", static_cast<int>(config.max_points));
    if (max_points <= 0) {
      throw std::invalid_argument{"local_map.max_points must be positive"};
    }
    config.max_points = static_cast<std::size_t>(max_points);
    return config;
  }

  LocalizationManagerConfig managerConfig()
  {
    LocalizationManagerConfig config;
    const auto interval_ms = declare_parameter<int>("match_interval_ms", 1000);
    if (interval_ms <= 0) {
      throw std::invalid_argument{"match_interval_ms must be positive"};
    }
    config.match_interval_ns = static_cast<std::int64_t>(interval_ms) * 1'000'000LL;
    config.maximum_correction_translation_jump_m = declare_parameter<double>(
      "maximum_correction_translation_jump_m", config.maximum_correction_translation_jump_m);
    config.maximum_correction_rotation_jump_rad = declare_parameter<double>(
      "maximum_correction_rotation_jump_rad", config.maximum_correction_rotation_jump_rad);
    return config;
  }

  LocalizationMonitorConfig monitorConfig()
  {
    LocalizationMonitorConfig config;
    const auto positiveCount = [this](const std::string & name, const std::size_t value) {
        const auto declared = declare_parameter<int>(name, static_cast<int>(value));
        if (declared <= 0) {
          throw std::invalid_argument{name + " must be positive"};
        }
        return static_cast<std::size_t>(declared);
      };
    config.initialization_successes_required = positiveCount(
      "monitor.initialization_successes_required", config.initialization_successes_required);
    config.degraded_failures_required = positiveCount(
      "monitor.degraded_failures_required", config.degraded_failures_required);
    config.lost_failures_required = positiveCount(
      "monitor.lost_failures_required", config.lost_failures_required);
    config.normal_recovery_successes_required = positiveCount(
      "monitor.normal_recovery_successes_required", config.normal_recovery_successes_required);
    config.relocalization_successes_required = positiveCount(
      "monitor.relocalization_successes_required", config.relocalization_successes_required);
    const auto stale_ms = declare_parameter<int>(
      "monitor.correction_stale_after_ms",
      static_cast<int>(config.correction_stale_after_ns / 1'000'000LL));
    const auto timeout_ms = declare_parameter<int>(
      "monitor.relocalization_timeout_ms",
      static_cast<int>(config.relocalization_timeout_ns / 1'000'000LL));
    if (stale_ms <= 0 || timeout_ms <= 0) {
      throw std::invalid_argument{"monitor time thresholds must be positive"};
    }
    config.correction_stale_after_ns = static_cast<std::int64_t>(stale_ms) * 1'000'000LL;
    config.relocalization_timeout_ns = static_cast<std::int64_t>(timeout_ms) * 1'000'000LL;
    config.strong_maximum_fitness = declare_parameter<double>(
      "monitor.strong_maximum_fitness", config.strong_maximum_fitness);
    config.strong_minimum_overlap = declare_parameter<double>(
      "monitor.strong_minimum_overlap", config.strong_minimum_overlap);
    config.strong_minimum_correspondences = positiveCount(
      "monitor.strong_minimum_correspondences", config.strong_minimum_correspondences);
    return config;
  }

  RelocalizationConfig globalRelocalizationConfig()
  {
    RelocalizationConfig config;
    const auto top_k = declare_parameter<int>(
      "global_relocalization.top_k", static_cast<int>(config.top_k));
    const auto before = declare_parameter<int>(
      "global_relocalization.neighbor_keyframes_before",
      static_cast<int>(config.neighbor_keyframes_before));
    const auto after = declare_parameter<int>(
      "global_relocalization.neighbor_keyframes_after",
      static_cast<int>(config.neighbor_keyframes_after));
    const auto strong_count = declare_parameter<int>(
      "global_relocalization.strong_minimum_correspondences",
      static_cast<int>(config.strong_minimum_correspondences));
    if (top_k < 2 || before < 0 || after < 0 || strong_count <= 0) {
      throw std::invalid_argument{"invalid global relocalization count settings"};
    }
    config.top_k = static_cast<std::size_t>(top_k);
    config.neighbor_keyframes_before = static_cast<std::size_t>(before);
    config.neighbor_keyframes_after = static_cast<std::size_t>(after);
    config.strong_minimum_correspondences = static_cast<std::size_t>(strong_count);
    config.maximum_descriptor_distance = declare_parameter<double>(
      "global_relocalization.maximum_descriptor_distance",
      config.maximum_descriptor_distance);
    config.minimum_score_margin = declare_parameter<double>(
      "global_relocalization.minimum_score_margin", config.minimum_score_margin);
    config.descriptor_score_weight = declare_parameter<double>(
      "global_relocalization.descriptor_score_weight", config.descriptor_score_weight);
    config.fitness_score_weight = declare_parameter<double>(
      "global_relocalization.fitness_score_weight", config.fitness_score_weight);
    config.overlap_score_weight = declare_parameter<double>(
      "global_relocalization.overlap_score_weight", config.overlap_score_weight);
    config.strong_maximum_fitness = declare_parameter<double>(
      "global_relocalization.strong_maximum_fitness", config.strong_maximum_fitness);
    config.strong_minimum_overlap = declare_parameter<double>(
      "global_relocalization.strong_minimum_overlap", config.strong_minimum_overlap);
    const auto finite_nonnegative = [](const double value) {
        return std::isfinite(value) && value >= 0.0;
      };
    if (!finite_nonnegative(config.maximum_descriptor_distance) ||
      !finite_nonnegative(config.minimum_score_margin) ||
      !finite_nonnegative(config.descriptor_score_weight) ||
      !finite_nonnegative(config.fitness_score_weight) ||
      !finite_nonnegative(config.overlap_score_weight) ||
      config.fitness_score_weight + config.overlap_score_weight <= 0.0 ||
      !finite_nonnegative(config.strong_maximum_fitness) ||
      !std::isfinite(config.strong_minimum_overlap) ||
      config.strong_minimum_overlap < 0.0 || config.strong_minimum_overlap > 1.0)
    {
      throw std::invalid_argument{"invalid global relocalization quality settings"};
    }
    return config;
  }

  RelocalizationSessionConfig sessionConfig(const std::int64_t timeout_ns)
  {
    RelocalizationSessionConfig config;
    const auto confirmations = declare_parameter<int>(
      "global_relocalization.confirmation_count", static_cast<int>(config.confirmation_count));
    if (confirmations < 2) {
      throw std::invalid_argument{"global relocalization confirmation_count must be at least two"};
    }
    config.confirmation_count = static_cast<std::size_t>(confirmations);
    config.maximum_translation_difference_m = declare_parameter<double>(
      "global_relocalization.maximum_translation_difference_m",
      config.maximum_translation_difference_m);
    config.maximum_rotation_difference_rad = declare_parameter<double>(
      "global_relocalization.maximum_rotation_difference_rad",
      config.maximum_rotation_difference_rad);
    config.timeout_ns = timeout_ns;
    return config;
  }

  RelocalizationResult evaluateRelocalization(
    const a2w_fastlio_common::FrontendFrame & frame) const
  {
    const auto started = std::chrono::steady_clock::now();
    auto result = relocalizer_->evaluate(
      frame.body_cloud, place_recognition_->describe(frame.body_cloud), relocalization_config_);
    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    if (elapsed_ms > relocalization_time_budget_ms_) {
      result.success = false;
      result.strong = false;
      result.reason = "evaluation_time_budget_exceeded";
    }
    return result;
  }

  void publishCandidateAudits(const RelocalizationWorkResult & work)
  {
    for (const auto & audit : work.result.audits) {
      publishStatus(
        work.frame.stamp_ns, audit.accepted, audit.reason,
        audit.candidate.keyframe_id, audit.registration, "global_relocalization");
    }
    if (work.result.audits.empty()) {
      publishStatus(
        work.frame.stamp_ns, false, work.result.reason, 0U, {}, "global_relocalization");
    }
  }

  void handleRelocalizationResult(const RelocalizationWorkResult & work)
  {
    if (monitor_->latest().state != LocalizationState::kRelocalizing) {
      return;
    }
    publishCandidateAudits(work);
    auto session_result = work.result;
    if (session_result.success) {
      session_result.map_body = compose(work.result.map_body, inverse(work.frame.odom_pose));
    }
    try {
      const auto session = relocalization_session_->observe(work.frame.stamp_ns, session_result);
      const bool confirmed = session.confirmed_correction.has_value();
      if (confirmed) {
        manager_->restoreCorrection(work.frame.stamp_ns, *session.confirmed_correction);
        std::lock_guard<std::mutex> lock{correction_mutex_};
        latest_correction_ = *session.confirmed_correction;
        latest_manager_correction_stamp_ns_ = work.frame.stamp_ns;
      }
      const auto monitor_status = monitor_->update(MatchEvidence{
        work.frame.stamp_ns, true, work.result.success, confirmed, 0,
        EvidenceSource::kRelocalization, work.result.candidate_id,
        work.result.registration, true, confirmed});
      publishMonitorStatus(monitor_status);
      tf_allowed_.store(monitor_status.state == LocalizationState::kLocalized ||
        monitor_status.state == LocalizationState::kDegraded);
      if (!confirmed || monitor_status.state != LocalizationState::kLocalized) {
        return;
      }
      relocalization_worker_->cancelPending();
      RCLCPP_INFO(
        get_logger(), "Global relocalization restored candidate %lu",
        static_cast<unsigned long>(work.result.candidate_id));
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Rejected relocalization result: %s", error.what());
    }
  }

  void beginGlobalRelocalization(const std::int64_t stamp_ns)
  {
    if (!global_relocalization_enabled_ || !relocalization_worker_ ||
      monitor_->latest().state != LocalizationState::kLost)
    {
      return;
    }
    relocalization_worker_->cancelPending();
    relocalization_session_->start(stamp_ns);
    publishMonitorStatus(monitor_->beginRelocalization(stamp_ns));
  }

  void inputCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    const auto input = adapt(*odometry, *cloud, odom_frame_, tracking_frame_);
    if (!input.success) {
      publishStatus(nanoseconds(odometry->header.stamp), false, input.reason, 0U, {});
      try {
        const auto stamp_ns = nanoseconds(odometry->header.stamp);
        std::optional<std::int64_t> correction_stamp;
        {
          std::lock_guard<std::mutex> lock{correction_mutex_};
          correction_stamp = latest_manager_correction_stamp_ns_;
        }
        const auto correction_available = correction_stamp.has_value();
        const auto correction_age_ns = correction_available ?
          std::max<std::int64_t>(0, stamp_ns - *correction_stamp) : 0;
        const auto monitor_status = monitor_->update(MatchEvidence{
          stamp_ns, false, false, correction_available, correction_age_ns,
          EvidenceSource::kNormal, 0U, {}});
        publishMonitorStatus(monitor_status);
        tf_allowed_.store(monitor_status.state == LocalizationState::kLocalized ||
          monitor_status.state == LocalizationState::kDegraded);
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "Rejected monitor evidence: %s", error.what());
      }
      return;
    }
    const auto current_state = monitor_->latest().state;
    if (current_state == LocalizationState::kLost && global_relocalization_enabled_) {
      beginGlobalRelocalization(input.frame.stamp_ns);
      return;
    }
    if (current_state == LocalizationState::kRelocalizing) {
      if (!relocalization_worker_->enqueue(input.frame)) {
        publishStatus(
          input.frame.stamp_ns, false, "relocalization_queue_full", 0U, {},
          "global_relocalization");
      }
      return;
    }
    try {
      const auto output = manager_->process(input.frame);
      if (output.match_attempted) {
        publishStatus(
          output.stamp_ns, output.match_accepted, output.reason,
          output.candidate_id, output.registration);
      }
      if (output.valid) {
        std::lock_guard<std::mutex> lock{correction_mutex_};
        latest_manager_correction_stamp_ns_ = output.correction_stamp_ns;
      }
      const auto monitor_status = monitor_->update(MatchEvidence{
        output.stamp_ns, output.match_attempted, output.match_accepted, output.valid,
        output.correction_age_ns, EvidenceSource::kNormal, output.candidate_id,
        output.registration});
      publishMonitorStatus(monitor_status);
      tf_allowed_.store(monitor_status.state == LocalizationState::kLocalized ||
        monitor_status.state == LocalizationState::kDegraded);
      if (monitor_status.state == LocalizationState::kLost) {
        beginGlobalRelocalization(output.stamp_ns);
      }
      if (!output.valid || !tf_allowed_.load()) {
        return;
      }
      {
        std::lock_guard<std::mutex> lock{correction_mutex_};
        latest_correction_ = output.map_camera_init;
      }
      const auto stamp = stampMessage(output.stamp_ns);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = map_frame_;
      pose.pose = poseMessage(output.map_body);
      pose_publisher_->publish(pose);
      nav_msgs::msg::Odometry localized;
      localized.header = pose.header;
      localized.child_frame_id = tracking_frame_;
      localized.pose.pose = pose.pose;
      odometry_publisher_->publish(localized);
      path_.header = pose.header;
      path_.poses.push_back(pose);
      if (path_.poses.size() > max_path_poses_) {
        path_.poses.erase(path_.poses.begin(), path_.poses.begin() +
          static_cast<std::ptrdiff_t>(path_.poses.size() - max_path_poses_));
      }
      path_publisher_->publish(path_);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Rejected Localization input: %s", error.what());
    }
  }

  void publishMonitorStatus(const LocalizationStatusSnapshot & status)
  {
    a2w_fastlio_msgs::msg::LocalizationStatus message;
    message.stamp = stampMessage(status.stamp_ns);
    message.state = static_cast<std::uint8_t>(status.state);
    message.state_label = localizationStateName(status.state);
    message.reason = status.reason;
    message.consecutive_successes = static_cast<std::uint32_t>(status.consecutive_successes);
    message.consecutive_failures = static_cast<std::uint32_t>(status.consecutive_failures);
    message.relocalization_successes = static_cast<std::uint32_t>(
      status.relocalization_successes);
    message.correction_available = status.correction_available;
    message.correction_age_ns = status.correction_age_ns;
    message.candidate_id = status.candidate_id;
    message.fitness = status.registration.fitness;
    message.overlap = status.registration.overlap;
    message.correspondence_count = status.registration.correspondence_count;
    message.hardware_validation_pending = status.hardware_validation_pending;
    monitor_status_publisher_->publish(message);
  }

  void publishStatus(
    const std::int64_t stamp, const bool accepted, const std::string & reason,
    const std::uint64_t candidate_id,
    const a2w_fastlio_common::RegistrationResult & registration,
    const std::string & stage = "localization")
  {
    a2w_fastlio_msgs::msg::RegistrationStatus message;
    message.stamp = stampMessage(stamp);
    message.keyframe_id = 0U;
    message.candidate_id = candidate_id;
    message.accepted = accepted;
    message.stage = stage;
    message.reason = reason;
    message.fitness = registration.fitness;
    message.overlap = registration.overlap;
    message.correspondence_count = registration.correspondence_count;
    message.elapsed_ms = registration.elapsed_ms;
    status_publisher_->publish(message);
  }

  void observeOwner(const a2w_fastlio_msgs::msg::GlobalTfOwner & message)
  {
    if (owner_state_->observe({
        nanoseconds(message.stamp), message.owner_id, message.mode,
        message.parent_frame, message.child_frame, message.active}, now().nanoseconds()))
    {
      RCLCPP_ERROR(get_logger(), "Global TF disabled: a foreign owner is active");
    }
  }

  void heartbeat()
  {
    const auto current = now();
    a2w_fastlio_msgs::msg::GlobalTfOwner message;
    message.stamp = static_cast<builtin_interfaces::msg::Time>(current);
    message.owner_id = owner_id_;
    message.mode = "localization";
    message.parent_frame = map_frame_;
    message.child_frame = odom_frame_;
    message.active = true;
    owner_publisher_->publish(message);
    if (!tf_allowed_.load() || !owner_state_->mayPublish(current.nanoseconds())) {
      return;
    }
    std::optional<a2w_fastlio_common::Pose3d> correction;
    {
      std::lock_guard<std::mutex> lock{correction_mutex_};
      correction = latest_correction_;
    }
    if (!correction) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = message.stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = correction->translation.x();
    transform.transform.translation.y = correction->translation.y();
    transform.transform.translation.z = correction->translation.z();
    transform.transform.rotation.w = correction->rotation.w();
    transform.transform.rotation.x = correction->rotation.x();
    transform.transform.rotation.y = correction->rotation.y();
    transform.transform.rotation.z = correction->rotation.z();
    tf_broadcaster_->sendTransform(transform);
  }

  std::string map_frame_, odom_frame_, tracking_frame_, owner_id_, owner_topic_;
  a2w_fastlio_map::MapBundle bundle_;
  std::shared_ptr<LocalMapSelector> selector_;
  std::shared_ptr<MapMatcher> matcher_;
  std::shared_ptr<a2w_fastlio_common::PlaceRecognition> place_recognition_;
  std::shared_ptr<a2w_fastlio_map::KeyFrameDatabase> relocalization_keyframes_;
  std::shared_ptr<a2w_fastlio_map::DescriptorDatabase> relocalization_descriptors_;
  std::shared_ptr<GlobalRelocalizer> relocalizer_;
  std::unique_ptr<LocalizationManager> manager_;
  std::unique_ptr<LocalizationMonitor> monitor_;
  std::unique_ptr<RelocalizationSession> relocalization_session_;
  std::unique_ptr<RelocalizationWorker> relocalization_worker_;
  std::unique_ptr<a2w_fastlio_common::GlobalTfOwnerState> owner_state_;
  RelocalizationConfig relocalization_config_;
  double relocalization_time_budget_ms_{5000.0};
  bool global_relocalization_enabled_{true};
  mutable std::mutex correction_mutex_;
  std::optional<a2w_fastlio_common::Pose3d> latest_correction_;
  std::optional<std::int64_t> latest_manager_correction_stamp_ns_;
  std::atomic_bool tf_allowed_{false};
  std::size_t max_path_poses_{10000U};
  nav_msgs::msg::Path path_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
  std::shared_ptr<Synchronizer> synchronizer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<a2w_fastlio_msgs::msg::RegistrationStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<a2w_fastlio_msgs::msg::LocalizationStatus>::SharedPtr
    monitor_status_publisher_;
  rclcpp::Publisher<a2w_fastlio_msgs::msg::GlobalTfOwner>::SharedPtr owner_publisher_;
  rclcpp::Subscription<a2w_fastlio_msgs::msg::GlobalTfOwner>::SharedPtr owner_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr owner_timer_;
};

}  // namespace a2w_fastlio_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<a2w_fastlio_localization::LocalizationNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("localization_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
