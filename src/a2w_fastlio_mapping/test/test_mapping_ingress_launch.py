import struct
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def generate_test_description():
    ingress = launch_ros.actions.Node(
        package="a2w_fastlio_mapping",
        executable="mapping_ingress_node",
        name="mapping_ingress_test",
        output="screen",
        parameters=[{
            "translation_threshold_m": 1.0,
            "rotation_threshold_deg": 10.0,
            "max_interval_s": 2.0,
        }],
    )
    invalid_nodes = [
        launch_ros.actions.Node(
            package="a2w_fastlio_mapping",
            executable="mapping_ingress_node",
            name="invalid_sync_queue",
            output="screen",
            parameters=[{"sync_queue_size": 0}],
        ),
        launch_ros.actions.Node(
            package="a2w_fastlio_mapping",
            executable="mapping_ingress_node",
            name="invalid_translation_threshold",
            output="screen",
            parameters=[{"translation_threshold_m": 0.0}],
        ),
        launch_ros.actions.Node(
            package="a2w_fastlio_mapping",
            executable="mapping_ingress_node",
            name="invalid_rotation_threshold",
            output="screen",
            parameters=[{"rotation_threshold_deg": 0.0}],
        ),
        launch_ros.actions.Node(
            package="a2w_fastlio_mapping",
            executable="mapping_ingress_node",
            name="invalid_interval",
            output="screen",
            parameters=[{"max_interval_s": 0.0}],
        ),
    ]
    return launch.LaunchDescription([
        ingress,
        *invalid_nodes,
        launch_testing.actions.ReadyToTest(),
    ]), {"ingress": ingress, "invalid_nodes": invalid_nodes}


def stamp(message, nanoseconds):
    message.header.stamp.sec = nanoseconds // 1_000_000_000
    message.header.stamp.nanosec = nanoseconds % 1_000_000_000


def make_odom(nanoseconds, x):
    message = Odometry()
    stamp(message, nanoseconds)
    message.header.frame_id = "camera_init"
    message.child_frame_id = "body"
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.0
    return message


def make_cloud(nanoseconds):
    message = PointCloud2()
    stamp(message, nanoseconds)
    message.header.frame_id = "body"
    message.height = 1
    message.width = 1
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = 16
    message.data = list(struct.pack("<ffff", 1.0, 2.0, 3.0, 4.0))
    message.is_dense = True
    return message


class TestMappingIngress(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("mapping_ingress_test_driver")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.node.create_publisher(Odometry, "/Odometry", qos)
        self.cloud_pub = self.node.create_publisher(PointCloud2, "/cloud_registered_body", qos)
        self.received_odom = []
        self.received_cloud = []
        self.node.create_subscription(
            Odometry, "/mapping/keyframe_odom", self.received_odom.append, qos)
        self.node.create_subscription(
            PointCloud2, "/mapping/keyframe_cloud", self.received_cloud.append, qos)

    def tearDown(self):
        self.node.destroy_node()

    def publish_pair_until(self, odom, cloud, predicate, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.odom_pub.publish(odom)
            self.cloud_pub.publish(cloud)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def test_publishes_first_keyframe_and_suppresses_below_threshold_frame(self):
        first_stamp = 12_345_000_000
        self.assertTrue(self.publish_pair_until(
            make_odom(first_stamp, 0.0),
            make_cloud(first_stamp),
            lambda: len(self.received_odom) >= 1 and len(self.received_cloud) >= 1,
        ))
        self.assertEqual(self.received_odom[0].header.stamp.sec, 12)
        self.assertEqual(self.received_odom[0].header.stamp.nanosec, 345_000_000)
        self.assertEqual(self.received_odom[0].header.frame_id, "camera_init")
        self.assertEqual(self.received_odom[0].child_frame_id, "body")
        self.assertEqual(self.received_cloud[0].header.frame_id, "body")
        self.assertEqual(self.node.get_publishers_info_by_topic("/tf"), [])

        initial_odom_count = len(self.received_odom)
        initial_cloud_count = len(self.received_cloud)
        second_stamp = first_stamp + 500_000_000
        deadline = time.monotonic() + 0.75
        while time.monotonic() < deadline:
            self.odom_pub.publish(make_odom(second_stamp, 0.5))
            self.cloud_pub.publish(make_cloud(second_stamp))
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertEqual(len(self.received_odom), initial_odom_count)
        self.assertEqual(len(self.received_cloud), initial_cloud_count)

    def test_rejects_invalid_startup_parameters(self, proc_info, invalid_nodes):
        for invalid_node in invalid_nodes:
            proc_info.assertWaitForShutdown(process=invalid_node, timeout=2.0)
            self.assertNotEqual(proc_info[invalid_node].returncode, 0)
