import struct
import tempfile
import time
import unittest
from pathlib import Path as FilePath

import launch
import launch_ros.actions
import launch_testing.asserts
import launch_testing.actions
from a2w_fastlio_msgs.msg import RegistrationStatus
from a2w_fastlio_msgs.srv import SaveMapBundle
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage


BUNDLE_ROOT = tempfile.mkdtemp(prefix="a2w-mapping-bundle-")


def backend(name, prefix, owner_topic, parent="map", child="camera_init"):
    return launch_ros.actions.Node(
        package="a2w_fastlio_mapping",
        executable="mapping_backend_node",
        name=name,
        output="screen",
        parameters=[{
            "keyframe_odom_topic": f"/{prefix}/keyframe_odom",
            "keyframe_cloud_topic": f"/{prefix}/keyframe_cloud",
            "optimized_odom_topic": f"/{prefix}/optimized_odom",
            "optimized_path_topic": f"/{prefix}/optimized_path",
            "optimized_map_preview_topic": f"/{prefix}/optimized_map_preview",
            "registration_status_topic": f"/{prefix}/registration_status",
            "global_tf_owner_topic": owner_topic,
            "owner_id": name,
            "map_frame": parent,
            "odom_frame": child,
            "tracking_frame": "body",
            "owner_conflict_window_ms": 300,
            "owner_heartbeat_ms": 50,
            "preview_voxel_leaf_m": 0.01,
            "preview_max_points": 2,
            "bundle_root": BUNDLE_ROOT,
        }],
    )


def generate_test_description():
    single = backend("single_mapping_owner", "single", "/single/global_tf_owner")
    conflict_a = backend(
        "conflicting_mapping_owner_a", "conflict_a", "/conflict/global_tf_owner",
        "conflict_map", "conflict_camera_init")
    conflict_b = backend(
        "conflicting_mapping_owner_b", "conflict_b", "/conflict/global_tf_owner",
        "conflict_map", "conflict_camera_init")
    return launch.LaunchDescription([
        single,
        conflict_a,
        conflict_b,
        launch_testing.actions.ReadyToTest(),
    ]), {"single": single, "conflict_a": conflict_a, "conflict_b": conflict_b}


def set_stamp(header, nanoseconds):
    header.stamp.sec = nanoseconds // 1_000_000_000
    header.stamp.nanosec = nanoseconds % 1_000_000_000


def make_odom(nanoseconds, x, parent="camera_init"):
    message = Odometry()
    set_stamp(message.header, nanoseconds)
    message.header.frame_id = parent
    message.child_frame_id = "body"
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.0
    return message


def make_cloud(nanoseconds):
    message = PointCloud2()
    set_stamp(message.header, nanoseconds)
    message.header.frame_id = "body"
    points = [(0.0, 0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 2.0), (2.0, 0.0, 0.0, 3.0)]
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 16
    message.row_step = message.point_step * message.width
    message.data = list(b"".join(struct.pack("<ffff", *point) for point in points))
    message.is_dense = True
    return message


class TestMappingOutputs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("mapping_outputs_test_driver")
        self.input_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.node.create_publisher(Odometry, "/single/keyframe_odom", self.input_qos)
        self.cloud_pub = self.node.create_publisher(PointCloud2, "/single/keyframe_cloud", self.input_qos)
        self.odoms, self.paths, self.previews, self.statuses, self.transforms = [], [], [], [], []
        self.node.create_subscription(Odometry, "/single/optimized_odom", self.odoms.append, volatile)
        self.node.create_subscription(Path, "/single/optimized_path", self.paths.append, transient)
        self.node.create_subscription(
            PointCloud2, "/single/optimized_map_preview", self.previews.append, transient)
        self.node.create_subscription(
            RegistrationStatus, "/single/registration_status", self.statuses.append, volatile)
        self.node.create_subscription(TFMessage, "/tf", self.transforms.append, 100)

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, predicate, timeout=6.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def test_publishes_bounded_optimized_products_and_single_global_tf(self):
        stamp_ns = 9_123_000_000
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and not (
                self.odoms and self.paths and self.previews and
                any(status.accepted for status in self.statuses)):
            self.odom_pub.publish(make_odom(stamp_ns, 2.0))
            self.cloud_pub.publish(make_cloud(stamp_ns))
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertTrue(self.odoms and self.paths and self.previews)
        accepted_statuses = [status for status in self.statuses if status.accepted]
        self.assertTrue(accepted_statuses)
        self.assertEqual(self.odoms[-1].header.frame_id, "map")
        self.assertEqual(self.odoms[-1].child_frame_id, "body")
        self.assertEqual(self.odoms[-1].header.stamp.sec, 9)
        self.assertEqual(self.odoms[-1].header.stamp.nanosec, 123_000_000)
        self.assertEqual(self.paths[-1].header.frame_id, "map")
        self.assertEqual(len(self.paths[-1].poses), 1)
        self.assertEqual(self.previews[-1].header.frame_id, "map")
        self.assertLessEqual(self.previews[-1].width * self.previews[-1].height, 2)
        self.assertEqual(accepted_statuses[-1].stage, "pose_graph")
        self.assertEqual(accepted_statuses[-1].reason, "optimized_snapshot_updated")

        self.assertTrue(self.spin_until(lambda: any(
            transform.header.frame_id == "map" and transform.child_frame_id == "camera_init"
            for batch in self.transforms for transform in batch.transforms)))
        self.assertEqual(self.node.get_publishers_info_by_topic("/mapping/optimized_map"), [])

        preview_info = self.node.get_publishers_info_by_topic("/single/optimized_map_preview")
        self.assertEqual(len(preview_info), 1)
        self.assertEqual(preview_info[0].qos_profile.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(preview_info[0].qos_profile.durability, DurabilityPolicy.TRANSIENT_LOCAL)
        self.assertEqual(preview_info[0].qos_profile.depth, 1)

    def test_save_map_bundle_service_uses_backend_snapshot(self):
        stamp_ns = 12_000_000_000
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and not self.odoms:
            self.odom_pub.publish(make_odom(stamp_ns, 3.0))
            self.cloud_pub.publish(make_cloud(stamp_ns))
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(self.odoms)
        client = self.node.create_client(SaveMapBundle, "/mapping/save_map_bundle")
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))

        invalid = client.call_async(SaveMapBundle.Request(output_path="../escape"))
        self.assertTrue(self.spin_until(lambda: invalid.done()))
        self.assertFalse(invalid.result().success)
        self.assertEqual(invalid.result().message, "output_path_not_confined")

        request = SaveMapBundle.Request(output_path="offline_test_map")
        future = client.call_async(request)
        self.assertTrue(self.spin_until(lambda: future.done(), timeout=10.0))
        response = future.result()
        self.assertTrue(response.success, response.message)
        self.assertGreaterEqual(response.keyframe_count, 1)
        self.assertTrue(response.bundle_uuid)
        self.assertEqual(
            FilePath(response.resolved_path), FilePath(BUNDLE_ROOT) / "offline_test_map")
        self.assertTrue((FilePath(response.resolved_path) / "global_map.pcd").is_file())
        self.assertTrue((FilePath(response.resolved_path) / "manifest.sha256").is_file())

    def test_conflicting_owners_never_reach_global_tf_publication(self):
        publishers = {}
        for prefix, parent in (("conflict_a", "conflict_camera_init"),
                               ("conflict_b", "conflict_camera_init")):
            publishers[prefix] = (
                self.node.create_publisher(Odometry, f"/{prefix}/keyframe_odom", self.input_qos),
                self.node.create_publisher(PointCloud2, f"/{prefix}/keyframe_cloud", self.input_qos),
                parent,
            )
        deadline = time.monotonic() + 1.5
        stamp_ns = 10_000_000_000
        while time.monotonic() < deadline:
            for odom_pub, cloud_pub, parent in publishers.values():
                odom_pub.publish(make_odom(stamp_ns, 0.0, parent))
                cloud_pub.publish(make_cloud(stamp_ns))
            rclpy.spin_once(self.node, timeout_sec=0.05)

        conflicting = [
            transform for batch in self.transforms for transform in batch.transforms
            if transform.header.frame_id == "conflict_map" and
            transform.child_frame_id == "conflict_camera_init"
        ]
        self.assertEqual(conflicting, [])


@launch_testing.post_shutdown_test()
class TestProcesses(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
