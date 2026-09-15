#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <a2w_fastlio_msgs/msg/global_tf_owner.hpp>
#include <a2w_fastlio_msgs/msg/registration_status.hpp>
#include <a2w_fastlio_msgs/srv/save_map_bundle.hpp>
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
#include "a2w_fastlio_common/local_map_builder.hpp"
#include "a2w_fastlio_mapping/frontend_message_adapter.hpp"
#include "a2w_fastlio_mapping/global_tf_ownership.hpp"
#include "a2w_fastlio_mapping/map_odom_manager.hpp"
#include "a2w_fastlio_mapping/loop_pipeline.hpp"
#include "a2w_fastlio_mapping/map_bundle_service.hpp"
#include "a2w_fastlio_mapping/optimized_map_builder.hpp"
#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

std::int64_t toNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

builtin_interfaces::msg::Time toTimeMessage(const rclcpp::Time & time)
{
  return static_cast<builtin_interfaces::msg::Time>(time);
}

geometry_msgs::msg::Pose toPoseMessage(const a2w_fastlio_common::Pose3d & pose)
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose.translation.x();
  message.position.y = pose.translation.y();
  message.position.z = pose.translation.z();
  message.orientation.w = pose.rotation.w();
  message.orientation.x = pose.rotation.x();
  message.orientation.y = pose.rotation.y();
  message.orientation.z = pose.rotation.z();
  return message;
}

std::string generatedUuid()
{
  static std::atomic<std::uint64_t> sequence{0U};
  const auto now = static_cast<std::uint64_t>(
    std::chrono::system_clock::now().time_since_epoch().count());
  const auto tail = now ^ (++sequence * 0x9e3779b97f4a7c15ULL);
  std::ostringstream stream;
  stream << std::hex << std::setfill('0')
         << std::setw(8) << static_cast<std::uint32_t>(now >> 32U) << '-'
         << std::setw(4) << static_cast<std::uint16_t>(now >> 16U) << '-'
         << std::setw(4) << static_cast<std::uint16_t>((now & 0x0fffU) | 0x4000U) << '-'
         << std::setw(4) << static_cast<std::uint16_t>((tail >> 48U & 0x3fffU) | 0x8000U) << '-'
         << std::setw(12) << (tail & 0xffffffffffffULL);
  return stream.str();
}

std::string utcNow()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&time, &utc);
  std::ostringstream stream;
  stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

class VectorKeyFrameProvider final : public a2w_fastlio_common::KeyFrameProvider
{
public:
  void append(a2w_fastlio_common::KeyFrame keyframe)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (keyframe.id != keyframes_.size()) {
      throw std::invalid_argument{"keyframes must be appended contiguously"};
    }
    keyframes_.push_back(std::move(keyframe));
  }

  std::optional<a2w_fastlio_common::KeyFrame> get(const std::uint64_t id) const override
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (id >= keyframes_.size()) {
      return std::nullopt;
    }
    return keyframes_[id];
  }

  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (keyframes_.empty()) {
      return std::nullopt;
    }
    return std::pair<std::uint64_t, std::uint64_t>{0U, keyframes_.size() - 1U};
  }

private:
  mutable std::mutex mutex_;
  std::vector<a2w_fastlio_common::KeyFrame> keyframes_;
};

}  // namespace

class MappingBackendNode final : public rclcpp::Node
{
public:
  MappingBackendNode()
  : Node("mapping_backend_node")
  {
    graph_ = std::make_shared<PoseGraphOptimizer>(poseGraphConfig());
    map_builder_ = std::make_unique<OptimizedMapBuilder>(optimizedMapConfig());
    owner_state_ = std::make_unique<GlobalTfOwnerState>(ownerConfig());
    const auto algorithms =
      a2w_fastlio_common::createDefaultAlgorithmSuite(algorithmSuiteConfig());
    place_recognition_ = algorithms.place_recognition;
    const auto local_map_config = localMapConfig();
    loop_pipeline_ = std::make_unique<LoopPipeline>(
      algorithms.place_recognition, algorithms.descriptor_index, algorithms.registration,
      graph_, a2w_fastlio_common::LocalMapBuilder{local_map_config}, loopPipelineConfig());

    const auto keyframe_odom_topic =
      declare_parameter<std::string>("keyframe_odom_topic", "/mapping/keyframe_odom");
    const auto keyframe_cloud_topic =
      declare_parameter<std::string>("keyframe_cloud_topic", "/mapping/keyframe_cloud");
    const auto optimized_odom_topic =
      declare_parameter<std::string>("optimized_odom_topic", "/mapping/optimized_odom");
    const auto optimized_path_topic =
      declare_parameter<std::string>("optimized_path_topic", "/mapping/optimized_path");
    const auto preview_topic = declare_parameter<std::string>(
      "optimized_map_preview_topic", "/mapping/optimized_map_preview");
    const auto registration_status_topic = declare_parameter<std::string>(
      "registration_status_topic", "/mapping/registration_status");
    owner_topic_ = declare_parameter<std::string>(
      "global_tf_owner_topic", "/a2w_fastlio/global_tf_owner");
    if (keyframe_odom_topic.empty() || keyframe_cloud_topic.empty() ||
      optimized_odom_topic.empty() || optimized_path_topic.empty() || preview_topic.empty() ||
      registration_status_topic.empty() || owner_topic_.empty())
    {
      throw std::invalid_argument{"mapping Topic parameters must not be empty"};
    }

    const auto input_qos = qos("input", 20, false);
    const auto optimized_qos = qos("optimized", 20, false);
    const auto latched_qos = qos("preview", 1, true);
    const auto owner_qos = qos("owner", 10, true);

    optimized_odom_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(optimized_odom_topic, optimized_qos);
    optimized_path_publisher_ =
      create_publisher<nav_msgs::msg::Path>(optimized_path_topic, latched_qos);
    preview_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(preview_topic, latched_qos);
    registration_status_publisher_ =
      create_publisher<a2w_fastlio_msgs::msg::RegistrationStatus>(
      registration_status_topic, optimized_qos);
    owner_publisher_ = create_publisher<a2w_fastlio_msgs::msg::GlobalTfOwner>(
      owner_topic_, owner_qos);
    owner_subscription_ = create_subscription<a2w_fastlio_msgs::msg::GlobalTfOwner>(
      owner_topic_, owner_qos,
      std::bind(&MappingBackendNode::observeOwner, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto sync_queue_size = declare_parameter<int>("sync_queue_size", 20);
    if (sync_queue_size <= 0) {
      throw std::invalid_argument{"sync_queue_size must be positive"};
    }
    odometry_subscriber_.subscribe(this, keyframe_odom_topic, input_qos.get_rmw_qos_profile());
    cloud_subscriber_.subscribe(this, keyframe_cloud_topic, input_qos.get_rmw_qos_profile());
    synchronizer_ = std::make_shared<Synchronizer>(
      odometry_subscriber_, cloud_subscriber_, static_cast<std::uint32_t>(sync_queue_size));
    synchronizer_->registerCallback(
      std::bind(&MappingBackendNode::keyframeCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    const auto heartbeat_ms = declare_parameter<int>("owner_heartbeat_ms", 250);
    if (heartbeat_ms <= 0) {
      throw std::invalid_argument{"owner_heartbeat_ms must be positive"};
    }
    owner_state_->start(now().nanoseconds());
    owner_timer_ = create_wall_timer(
      std::chrono::milliseconds{heartbeat_ms}, std::bind(&MappingBackendNode::heartbeat, this));
    const auto worker_period_ms = declare_parameter<int>("worker_period_ms", 10);
    if (worker_period_ms <= 0) {
      throw std::invalid_argument{"worker_period_ms must be positive"};
    }
    worker_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    worker_timer_ = create_wall_timer(
      std::chrono::milliseconds{worker_period_ms},
      std::bind(&MappingBackendNode::processOneKeyframe, this), worker_callback_group_);

    const auto bundle_root = declare_parameter<std::string>("bundle_root", "maps");
    const auto save_queue_capacity = declare_parameter<int>("save_queue_capacity", 1);
    if (bundle_root.empty() || save_queue_capacity <= 0) {
      throw std::invalid_argument{"invalid Map Bundle save configuration"};
    }
    frontend_revision_ = declare_parameter<std::string>(
      "frontend_revision", "16e97cdcc4260b9ba6241518a19ad8384224ba48");
    scan_context_revision_ = declare_parameter<std::string>("scan_context_revision", "curated");
    quatro_revision_ = declare_parameter<std::string>("quatro_revision", "d27109b");
    nano_gicp_revision_ = declare_parameter<std::string>("nano_gicp_revision", "b21e79e");
    teaser_revision_ = declare_parameter<std::string>("teaser_revision", "974574c");
    pmc_revision_ = declare_parameter<std::string>("pmc_revision", "a2dfd61");
    bundle_service_ = std::make_unique<MapBundleService>(
      bundle_root, static_cast<std::size_t>(save_queue_capacity));
    save_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    save_service_ = create_service<a2w_fastlio_msgs::srv::SaveMapBundle>(
      declare_parameter<std::string>("save_map_bundle_service", "/mapping/save_map_bundle"),
      std::bind(&MappingBackendNode::saveMapBundle, this,
      std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, save_callback_group_);
  }

private:
  using Synchronizer = message_filters::TimeSynchronizer<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

  PoseGraphConfig poseGraphConfig()
  {
    PoseGraphConfig config;
    config.prior_rotation_sigma_rad =
      declare_parameter<double>("pose_graph.prior_rotation_sigma_rad", config.prior_rotation_sigma_rad);
    config.prior_translation_sigma_m =
      declare_parameter<double>("pose_graph.prior_translation_sigma_m", config.prior_translation_sigma_m);
    config.odometry_rotation_sigma_rad =
      declare_parameter<double>("pose_graph.odometry_rotation_sigma_rad", config.odometry_rotation_sigma_rad);
    config.odometry_translation_sigma_m = declare_parameter<double>(
      "pose_graph.odometry_translation_sigma_m", config.odometry_translation_sigma_m);
    config.loop_rotation_sigma_rad =
      declare_parameter<double>("pose_graph.loop_rotation_sigma_rad", config.loop_rotation_sigma_rad);
    config.loop_translation_sigma_m =
      declare_parameter<double>("pose_graph.loop_translation_sigma_m", config.loop_translation_sigma_m);
    const auto robust_kernel =
      declare_parameter<std::string>("pose_graph.robust_kernel", "huber");
    if (robust_kernel == "huber") {
      config.robust_kernel = RobustKernel::HUBER;
    } else if (robust_kernel == "cauchy") {
      config.robust_kernel = RobustKernel::CAUCHY;
    } else {
      throw std::invalid_argument{"pose_graph.robust_kernel must be huber or cauchy"};
    }
    config.robust_kernel_scale =
      declare_parameter<double>("pose_graph.robust_kernel_scale", config.robust_kernel_scale);
    config.relinearization_threshold = declare_parameter<double>(
      "pose_graph.relinearization_threshold", config.relinearization_threshold);
    config.relinearization_skip =
      declare_parameter<int>("pose_graph.relinearization_skip", config.relinearization_skip);
    return config;
  }

  OptimizedMapConfig optimizedMapConfig()
  {
    OptimizedMapConfig config;
    config.full_voxel_leaf_m =
      declare_parameter<double>("full_map_voxel_leaf_m", config.full_voxel_leaf_m);
    config.preview_voxel_leaf_m =
      declare_parameter<double>("preview_voxel_leaf_m", config.preview_voxel_leaf_m);
    const auto max_points =
      declare_parameter<int>("preview_max_points", static_cast<int>(config.preview_max_points));
    if (max_points <= 0) {
      throw std::invalid_argument{"preview_max_points must be positive"};
    }
    config.preview_max_points = static_cast<std::size_t>(max_points);
    return config;
  }

  a2w_fastlio_common::DefaultAlgorithmSuiteConfig algorithmSuiteConfig()
  {
    a2w_fastlio_common::DefaultAlgorithmSuiteConfig config;
    const auto rings = declare_parameter<int>(
      "scan_context.rings", static_cast<int>(config.place_recognition.rings));
    const auto sectors = declare_parameter<int>(
      "scan_context.sectors", static_cast<int>(config.place_recognition.sectors));
    if (rings <= 0 || sectors <= 0) {
      throw std::invalid_argument{"Scan Context dimensions must be positive"};
    }
    config.place_recognition.rings = static_cast<std::size_t>(rings);
    config.place_recognition.sectors = static_cast<std::size_t>(sectors);
    config.place_recognition.max_radius_m = declare_parameter<double>(
      "scan_context.max_radius_m", config.place_recognition.max_radius_m);
    config.place_recognition.sensor_height_m = declare_parameter<double>(
      "scan_context.sensor_height_m", config.place_recognition.sensor_height_m);

    config.coarse.normal_radius_m =
      declare_parameter<double>("quatro.fpfh_normal_radius_m", config.coarse.normal_radius_m);
    config.coarse.feature_radius_m =
      declare_parameter<double>("quatro.fpfh_radius_m", config.coarse.feature_radius_m);
    config.coarse.noise_bound_m =
      declare_parameter<double>("quatro.noise_bound_m", config.coarse.noise_bound_m);
    config.coarse.rotation_gnc_factor = declare_parameter<double>(
      "quatro.rotation_gnc_factor", config.coarse.rotation_gnc_factor);
    config.coarse.rotation_cost_threshold = declare_parameter<double>(
      "quatro.rotation_cost_threshold", config.coarse.rotation_cost_threshold);
    config.coarse.rotation_max_iterations = declare_parameter<int>(
      "quatro.rotation_max_iterations", config.coarse.rotation_max_iterations);
    config.coarse.estimate_scale =
      declare_parameter<bool>("quatro.estimate_scale", config.coarse.estimate_scale);
    config.coarse.optimized_matching = declare_parameter<bool>(
      "quatro.optimized_matching", config.coarse.optimized_matching);
    config.coarse.descriptor_distance_threshold = declare_parameter<double>(
      "quatro.descriptor_distance_threshold", config.coarse.descriptor_distance_threshold);
    config.coarse.maximum_correspondences = declare_parameter<int>(
      "quatro.maximum_correspondences", config.coarse.maximum_correspondences);
    const auto coarse_minimum_points = declare_parameter<int>(
      "quatro.minimum_points", static_cast<int>(config.coarse.minimum_points));
    if (coarse_minimum_points <= 0) {
      throw std::invalid_argument{"quatro.minimum_points must be positive"};
    }
    config.coarse.minimum_points = static_cast<std::size_t>(coarse_minimum_points);

    config.fine.maximum_correspondence_distance_m = declare_parameter<double>(
      "nano_gicp.maximum_correspondence_distance_m",
      config.fine.maximum_correspondence_distance_m);
    config.fine.thread_count =
      declare_parameter<int>("nano_gicp.thread_count", config.fine.thread_count);
    config.fine.correspondence_randomness = declare_parameter<int>(
      "nano_gicp.correspondence_randomness", config.fine.correspondence_randomness);
    config.fine.maximum_iterations = declare_parameter<int>(
      "nano_gicp.maximum_iterations", config.fine.maximum_iterations);
    config.fine.transformation_epsilon = declare_parameter<double>(
      "nano_gicp.transformation_epsilon", config.fine.transformation_epsilon);
    config.fine.rotation_epsilon =
      declare_parameter<double>("nano_gicp.rotation_epsilon", config.fine.rotation_epsilon);
    config.fine.regularization_method = declare_parameter<int>(
      "nano_gicp.regularization_method", config.fine.regularization_method);
    config.fine.fitness_score_max_range_m = declare_parameter<double>(
      "nano_gicp.fitness_score_max_range_m", config.fine.fitness_score_max_range_m);
    const auto fine_minimum_points = declare_parameter<int>(
      "nano_gicp.minimum_points", static_cast<int>(config.fine.minimum_points));
    if (fine_minimum_points <= 0) {
      throw std::invalid_argument{"nano_gicp.minimum_points must be positive"};
    }
    config.fine.minimum_points = static_cast<std::size_t>(fine_minimum_points);

    config.validation.maximum_fitness = declare_parameter<double>(
      "loop_validation.maximum_fitness", config.validation.maximum_fitness);
    config.validation.minimum_overlap = declare_parameter<double>(
      "loop_validation.minimum_overlap", config.validation.minimum_overlap);
    const auto minimum_correspondences = declare_parameter<int>(
      "loop_validation.minimum_correspondences",
      static_cast<int>(config.validation.minimum_correspondences));
    if (minimum_correspondences <= 0) {
      throw std::invalid_argument{"loop_validation.minimum_correspondences must be positive"};
    }
    config.validation.minimum_correspondences =
      static_cast<std::size_t>(minimum_correspondences);
    config.validation.maximum_translation_jump_m = declare_parameter<double>(
      "loop_validation.maximum_translation_jump_m",
      config.validation.maximum_translation_jump_m);
    config.validation.maximum_rotation_jump_rad = declare_parameter<double>(
      "loop_validation.maximum_rotation_jump_rad",
      config.validation.maximum_rotation_jump_rad);
    config.validation.minimum_candidate_distance_separation = declare_parameter<double>(
      "loop_validation.minimum_candidate_distance_separation",
      config.validation.minimum_candidate_distance_separation);
    config.evidence_distance_m =
      declare_parameter<double>("pipeline.evidence_distance_m", config.evidence_distance_m);
    return config;
  }

  a2w_fastlio_common::LocalMapConfig localMapConfig()
  {
    a2w_fastlio_common::LocalMapConfig config;
    config.voxel_leaf_m =
      declare_parameter<double>("local_map.voxel_leaf_m", config.voxel_leaf_m);
    const auto max_points =
      declare_parameter<int>("local_map.max_points", static_cast<int>(config.max_points));
    if (max_points <= 0) {
      throw std::invalid_argument{"local_map.max_points must be positive"};
    }
    config.max_points = static_cast<std::size_t>(max_points);
    return config;
  }

  LoopPipelineConfig loopPipelineConfig()
  {
    LoopPipelineConfig config;
    const auto positive_size = [this](const std::string & name, const std::size_t value) {
        const auto parameter = declare_parameter<int>(name, static_cast<int>(value));
        if (parameter <= 0) {
          throw std::invalid_argument{name + " must be positive"};
        }
        return static_cast<std::size_t>(parameter);
      };
    const auto nonnegative_size = [this](const std::string & name, const std::size_t value) {
        const auto parameter = declare_parameter<int>(name, static_cast<int>(value));
        if (parameter < 0) {
          throw std::invalid_argument{name + " must not be negative"};
        }
        return static_cast<std::size_t>(parameter);
      };
    config.top_k = positive_size("scan_context.top_k", config.top_k);
    config.exclude_recent = nonnegative_size("scan_context.exclude_recent", config.exclude_recent);
    config.local_map_before = nonnegative_size(
      "local_map.neighbor_keyframes_before", config.local_map_before);
    config.local_map_after = nonnegative_size(
      "local_map.neighbor_keyframes_after", config.local_map_after);
    config.minimum_keyframes_between_accepted_loops = positive_size(
      "loop_pipeline.minimum_keyframes_between_accepted_loops",
      config.minimum_keyframes_between_accepted_loops);
    config.queue_capacity = positive_size("loop_pipeline.queue_capacity", config.queue_capacity);
    config.maximum_descriptor_distance = declare_parameter<double>(
      "scan_context.max_distance", config.maximum_descriptor_distance);
    return config;
  }

  GlobalTfOwnerConfig ownerConfig()
  {
    owner_id_ = declare_parameter<std::string>("owner_id", "mapping_backend");
    mode_ = declare_parameter<std::string>("mode", "mapping");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    tracking_frame_ = declare_parameter<std::string>("tracking_frame", "body");
    const auto conflict_window_ms = declare_parameter<int>("owner_conflict_window_ms", 1000);
    if (tracking_frame_.empty() || conflict_window_ms <= 0) {
      throw std::invalid_argument{"invalid tracking frame or owner conflict window"};
    }
    return {
      owner_id_, mode_, map_frame_, odom_frame_,
      static_cast<std::int64_t>(conflict_window_ms) * 1'000'000LL};
  }

  rclcpp::QoS qos(const std::string & prefix, const int default_depth, const bool transient)
  {
    const auto depth = declare_parameter<int>(prefix + "_qos_depth", default_depth);
    const auto reliability =
      declare_parameter<std::string>(prefix + "_qos_reliability", "reliable");
    const auto durability = declare_parameter<std::string>(
      prefix + "_qos_durability", transient ? "transient_local" : "volatile");
    if (depth <= 0 || (reliability != "reliable" && reliability != "best_effort") ||
      (durability != "volatile" && durability != "transient_local"))
    {
      throw std::invalid_argument{"invalid QoS parameters for " + prefix};
    }
    auto result = rclcpp::QoS{rclcpp::KeepLast{static_cast<std::size_t>(depth)}};
    reliability == "reliable" ? result.reliable() : result.best_effort();
    durability == "transient_local" ? result.transient_local() : result.durability_volatile();
    return result;
  }

  void keyframeCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    const auto adapted = makeFrontendFrame(*odometry, *cloud, odom_frame_, tracking_frame_);
    if (!adapted.success || (last_stamp_ns_ && adapted.frame.stamp_ns <= *last_stamp_ns_)) {
      return;
    }

    const auto id = static_cast<std::uint64_t>(keyframe_count_);
    a2w_fastlio_common::KeyFrame keyframe;
    keyframe.id = id;
    keyframe.stamp_ns = adapted.frame.stamp_ns;
    keyframe.odom_pose = adapted.frame.odom_pose;
    keyframe.optimized_pose = adapted.frame.odom_pose;
    keyframe.body_cloud = a2w_fastlio_common::CloudPtr{
      new a2w_fastlio_common::Cloud{*adapted.frame.body_cloud}};

    if (!loop_pipeline_->enqueue(keyframe)) {
      publishStatus(id, adapted.frame.stamp_ns, false, "ingress", "worker_queue_full");
      return;
    }
    provider_.append(std::move(keyframe));
    ++keyframe_count_;
    last_stamp_ns_ = adapted.frame.stamp_ns;
  }

  void processOneKeyframe()
  {
    std::lock_guard<std::mutex> backend_lock{backend_state_mutex_};
    const auto events = loop_pipeline_->processNext();
    if (!events || events->empty()) {
      return;
    }
    for (const auto & event : *events) {
      const auto event_keyframe = provider_.get(event.current_id);
      if (!event_keyframe) {
        throw std::runtime_error{"loop event keyframe is missing from Mapping provider"};
      }
      publishStatus(
        event.current_id, event_keyframe->stamp_ns, event.accepted,
        "loop", event.reason, event.candidate_id, event.registration);
    }
    const auto snapshot = graph_->optimizedPoses();
    if (snapshot.poses.empty()) {
      return;
    }
    const auto & optimized = snapshot.poses.back();
    const auto current = provider_.get(optimized.id);
    if (!current) {
      throw std::runtime_error{"optimized keyframe is missing from Mapping provider"};
    }
    correction_manager_.update(
      optimized.id, current->stamp_ns, optimized.pose, current->odom_pose);
    publishProducts(snapshot, current->stamp_ns);
    publishStatus(
      optimized.id, current->stamp_ns, true, "pose_graph", "optimized_snapshot_updated");
  }

  std::string parameterSnapshot(const std::vector<std::string> & prefixes)
  {
    auto names = list_parameters({}, 10U).names;
    std::sort(names.begin(), names.end());
    std::ostringstream stream;
    for (const auto & name : names) {
      const bool included = prefixes.empty() || std::any_of(
        prefixes.begin(), prefixes.end(), [&name](const auto & prefix) {
          return name.rfind(prefix, 0U) == 0U;
        });
      if (included) {
        stream << name << ": " << get_parameter(name).value_to_string() << '\n';
      }
    }
    return stream.str();
  }

  a2w_fastlio_map::MapBundleData bundleSnapshot()
  {
    a2w_fastlio_map::MapBundleData data;
    {
      std::lock_guard<std::mutex> backend_lock{backend_state_mutex_};
      const auto optimized = graph_->optimizedPoses();
      if (optimized.poses.empty() || !latest_full_map_ || latest_full_map_->empty()) {
        throw std::runtime_error{"no optimized Mapping snapshot is available"};
      }
      data.keyframes.reserve(optimized.poses.size());
      for (const auto & pose : optimized.poses) {
        auto keyframe = provider_.get(pose.id);
        if (!keyframe) {
          throw std::runtime_error{"Mapping snapshot has a missing keyframe"};
        }
        keyframe->optimized_pose = pose.pose;
        data.keyframes.push_back(std::move(*keyframe));
      }
      data.global_map = a2w_fastlio_common::CloudPtr{
        new a2w_fastlio_common::Cloud{*latest_full_map_}};
    }
    data.descriptors.reserve(data.keyframes.size());
    for (const auto & keyframe : data.keyframes) {
      data.descriptors.push_back({
        keyframe.id, place_recognition_->describe(keyframe.body_cloud)});
    }
    data.metadata.bundle_uuid = generatedUuid();
    data.metadata.created_utc = utcNow();
    data.metadata.map_frame = map_frame_;
    data.metadata.odom_frame = odom_frame_;
    data.metadata.tracking_frame = tracking_frame_;
    data.metadata.global_map_leaf_m = get_parameter("full_map_voxel_leaf_m").as_double();
    data.metadata.frontend_revision = frontend_revision_;
    data.metadata.dependency_revisions = {
      {"scan_context", scan_context_revision_}, {"quatro", quatro_revision_},
      {"nano_gicp", nano_gicp_revision_}, {"teaser++", teaser_revision_},
      {"pmc", pmc_revision_}, {"gtsam", "4.1.1"}};
    data.config_snapshots = {
      {"mapping_effective.yaml", parameterSnapshot({})},
      {"scan_context.yaml", parameterSnapshot({"scan_context."})},
      {"registration.yaml", parameterSnapshot(
          {"local_map.", "quatro.", "nano_gicp.", "pipeline.", "loop_validation."})},
    };
    return data;
  }

  void saveMapBundle(
    const std::shared_ptr<a2w_fastlio_msgs::srv::SaveMapBundle::Request> request,
    std::shared_ptr<a2w_fastlio_msgs::srv::SaveMapBundle::Response> response)
  {
    try {
      auto data = bundleSnapshot();
      const auto result = bundle_service_->save(request->output_path, std::move(data));
      response->success = result.success;
      response->message = result.message;
      response->bundle_uuid = result.bundle_uuid;
      response->keyframe_count = result.keyframe_count;
      response->resolved_path = result.resolved_path.string();
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
    }
  }

  void publishProducts(const OptimizedPoseSnapshot & snapshot, const std::int64_t stamp_ns)
  {
    const auto stamp = toTimeMessage(rclcpp::Time{stamp_ns});
    const auto & current = snapshot.poses.back();
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = map_frame_;
    odometry.child_frame_id = tracking_frame_;
    odometry.pose.pose = toPoseMessage(current.pose);
    optimized_odom_publisher_->publish(odometry);

    nav_msgs::msg::Path path;
    path.header = odometry.header;
    path.poses.reserve(snapshot.poses.size());
    for (const auto & optimized : snapshot.poses) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose = toPoseMessage(optimized.pose);
      path.poses.push_back(std::move(pose));
    }
    optimized_path_publisher_->publish(path);

    latest_full_map_ = map_builder_->buildFull(provider_, snapshot);
    const auto preview = map_builder_->buildPreview(provider_, snapshot);
    sensor_msgs::msg::PointCloud2 preview_message;
    pcl::toROSMsg(*preview, preview_message);
    preview_message.header.stamp = stamp;
    preview_message.header.frame_id = map_frame_;
    preview_publisher_->publish(preview_message);
  }

  void publishStatus(
    const std::uint64_t id, const std::int64_t stamp_ns, const bool accepted,
    const std::string & stage, const std::string & reason,
    const std::uint64_t candidate_id = std::numeric_limits<std::uint64_t>::max(),
    const a2w_fastlio_common::RegistrationResult & registration = {})
  {
    a2w_fastlio_msgs::msg::RegistrationStatus status;
    status.stamp = toTimeMessage(rclcpp::Time{stamp_ns});
    status.keyframe_id = id;
    status.candidate_id = candidate_id;
    status.accepted = accepted;
    status.stage = stage;
    status.reason = reason;
    status.fitness = registration.fitness;
    status.overlap = registration.overlap;
    status.correspondence_count = registration.correspondence_count;
    status.elapsed_ms = registration.elapsed_ms;
    registration_status_publisher_->publish(status);
  }

  void observeOwner(const a2w_fastlio_msgs::msg::GlobalTfOwner & message)
  {
    const bool newly_conflicted = owner_state_->observe(
      {toNanoseconds(message.stamp), message.owner_id, message.mode,
        message.parent_frame, message.child_frame, message.active},
      now().nanoseconds());
    if (newly_conflicted && !fault_published_) {
      fault_published_ = true;
      publishStatus(
        keyframe_count_ == 0U ? 0U : keyframe_count_ - 1U, now().nanoseconds(), false,
        "ownership", owner_state_->faultReason(now().nanoseconds()));
      RCLCPP_ERROR(get_logger(), "Global TF disabled: a foreign owner is active");
    }
  }

  void heartbeat()
  {
    const auto current_time = now();
    a2w_fastlio_msgs::msg::GlobalTfOwner owner;
    owner.stamp = toTimeMessage(current_time);
    owner.owner_id = owner_id_;
    owner.mode = mode_;
    owner.parent_frame = map_frame_;
    owner.child_frame = odom_frame_;
    owner.active = true;
    owner_publisher_->publish(owner);

    const auto correction = correction_manager_.latest();
    if (!correction || !owner_state_->mayPublish(current_time.nanoseconds())) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = toTimeMessage(current_time);
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = correction->map_camera_init.translation.x();
    transform.transform.translation.y = correction->map_camera_init.translation.y();
    transform.transform.translation.z = correction->map_camera_init.translation.z();
    transform.transform.rotation.w = correction->map_camera_init.rotation.w();
    transform.transform.rotation.x = correction->map_camera_init.rotation.x();
    transform.transform.rotation.y = correction->map_camera_init.rotation.y();
    transform.transform.rotation.z = correction->map_camera_init.rotation.z();
    tf_broadcaster_->sendTransform(transform);
  }

  std::string owner_id_;
  std::string mode_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string tracking_frame_;
  std::string owner_topic_;
  std::shared_ptr<PoseGraphOptimizer> graph_;
  std::unique_ptr<OptimizedMapBuilder> map_builder_;
  std::unique_ptr<GlobalTfOwnerState> owner_state_;
  std::unique_ptr<LoopPipeline> loop_pipeline_;
  std::shared_ptr<a2w_fastlio_common::PlaceRecognition> place_recognition_;
  std::unique_ptr<MapBundleService> bundle_service_;
  MapOdomManager correction_manager_;
  VectorKeyFrameProvider provider_;
  a2w_fastlio_common::CloudPtr latest_full_map_;
  std::atomic<std::size_t> keyframe_count_{0U};
  std::optional<std::int64_t> last_stamp_ns_;
  bool fault_published_{false};
  std::string frontend_revision_;
  std::string scan_context_revision_;
  std::string quatro_revision_;
  std::string nano_gicp_revision_;
  std::string teaser_revision_;
  std::string pmc_revision_;
  std::mutex backend_state_mutex_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
  std::shared_ptr<Synchronizer> synchronizer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optimized_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preview_publisher_;
  rclcpp::Publisher<a2w_fastlio_msgs::msg::RegistrationStatus>::SharedPtr
    registration_status_publisher_;
  rclcpp::Publisher<a2w_fastlio_msgs::msg::GlobalTfOwner>::SharedPtr owner_publisher_;
  rclcpp::Subscription<a2w_fastlio_msgs::msg::GlobalTfOwner>::SharedPtr owner_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr owner_timer_;
  rclcpp::CallbackGroup::SharedPtr worker_callback_group_;
  rclcpp::TimerBase::SharedPtr worker_timer_;
  rclcpp::CallbackGroup::SharedPtr save_callback_group_;
  rclcpp::Service<a2w_fastlio_msgs::srv::SaveMapBundle>::SharedPtr save_service_;
};

}  // namespace a2w_fastlio_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<a2w_fastlio_mapping::MappingBackendNode>();
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 2U};
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mapping_backend_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
