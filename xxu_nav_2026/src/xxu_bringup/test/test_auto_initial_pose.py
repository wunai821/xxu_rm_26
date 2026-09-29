"""Exercise initialization timing without publishing to a running ROS graph."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

SPEC = importlib.util.spec_from_file_location(
    "auto_initial_pose", Path(__file__).resolve().parents[1] / "scripts/auto_initial_pose.py"
)
auto = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(auto)


def transform(seconds, x=0.0, y=0.0, yaw=0.0):
    tf = TransformStamped()
    tf.header.stamp = Time(seconds=seconds).to_msg()
    tf.transform.translation.x = float(x)
    tf.transform.translation.y = float(y)
    tf.transform.rotation.z = math.sin(yaw / 2)
    tf.transform.rotation.w = math.cos(yaw / 2)
    return tf


def scan(seconds):
    msg = LaserScan()
    msg.header.stamp = Time(seconds=seconds).to_msg()
    return msg


class TimingTests(unittest.TestCase):
    def setUp(self):
        # Bind real state-machine methods to a lightweight harness; only ROS I/O
        # and the expensive global matcher are replaced.
        self.node = SimpleNamespace()
        for name in ("scan_callback", "scan_age", "require_fresh_publish_reference",
                     "publish_pose", "start_publishing"):
            setattr(self.node, name, getattr(auto.AutoInitialPose, name).__get__(self.node))
        n = self.node
        self.now = 6.4
        n.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=self.now))
        n.get_logger = lambda: Mock()
        n.get_parameter = lambda name: SimpleNamespace(value=0.25)
        n.matching_started = False
        n.scans_seen = 0
        n.scan_warmup_count = 0
        n.max_scan_age = 0.5
        n.scan_tf_timeout = 0.25
        n.min_match_score = 0.18
        n.timeout_timer = Mock()
        n.timer = None
        n.odom_frame = "odom"
        n.base_frame = "base_footprint"
        n.frame_id = "map"
        n.pending_match = None
        n.publish_odom_reference = None
        n.publish_source = ""
        n.sent = 0
        n.publish_count = 1
        n.publish_period = 0.5
        n.defer_amcl_activation = False
        n.scan_callback_group = None
        n.create_timer = Mock()
        n.pub = Mock()
        n.tf_buffer = Mock()
        n.tf_buffer.lookup_transform.side_effect = lambda *args, **kw: transform(
            args[2].nanoseconds / 1e9, x=2.0 if args[2].nanoseconds / 1e9 > 20 else 0.0
        )
        n.matcher = Mock()
        n.matcher.validate.return_value = 1.2

        def slow_search(msg):
            self.now = 26.4
            return 10.0, 3.0, math.pi / 2, 1.3

        n.matcher.estimate.side_effect = slow_search

    def test_twenty_second_search_waits_then_propagates_and_validates(self):
        n = self.node
        n.scan_callback(scan(6.3))
        n.pub.publish.assert_not_called()
        n.scan_callback(scan(6.4))  # Queued sample from before search completed.
        n.matcher.validate.assert_not_called()
        n.scan_callback(scan(26.3))
        msg = n.pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.pose.pose.position.x, 10.0)
        self.assertAlmostEqual(msg.pose.pose.position.y, 5.0)
        self.assertAlmostEqual(auto.stamp_seconds(msg.header.stamp), 26.4)
        self.assertAlmostEqual(auto.stamp_seconds(n.publish_odom_reference.header.stamp), 26.3)
        self.assertEqual(n.matcher.estimate.call_count, 1)
        self.assertEqual(n.sent, 1)

    def test_bad_fresh_scan_never_publishes_fixed_fallback(self):
        n = self.node
        n.scan_callback(scan(6.3))
        n.matcher.validate.return_value = -1.0
        n.scan_callback(scan(26.3))
        n.pub.publish.assert_not_called()
        self.assertIsNone(n.pending_match)
        self.assertFalse(n.matching_started)

    def test_scan_expiring_during_tf_lookup_is_revalidated(self):
        n = self.node
        n.scan_callback(scan(6.3))
        original = n.tf_buffer.lookup_transform.side_effect

        def delayed(*args, **kwargs):
            result = original(*args, **kwargs)
            if args[2].nanoseconds / 1e9 > 26.35:
                self.now = 27.1
            return result

        n.tf_buffer.lookup_transform.side_effect = delayed
        n.scan_callback(scan(26.3))
        n.pub.publish.assert_not_called()
        self.assertFalse(n.matching_started)
        n.create_timer.assert_not_called()
        n.tf_buffer.lookup_transform.side_effect = original
        n.scan_callback(scan(27.0))
        n.pub.publish.assert_called_once()

    def test_failed_tf_retry_cannot_publish_expired_observation(self):
        n = self.node
        n.scan_callback(scan(6.3))
        n.tf_buffer.lookup_transform.side_effect = [transform(26.3), RuntimeError("TF late")]
        n.scan_callback(scan(26.3))
        n.pub.publish.assert_not_called()
        n.timer = Mock()
        timer = n.timer
        self.now = 27.0
        n.publish_pose()
        n.pub.publish.assert_not_called()
        timer.cancel.assert_called_once()
        self.assertFalse(n.matching_started)

    def test_fixed_mode_retains_tf_retry(self):
        n = self.node
        n.tf_buffer.lookup_transform.side_effect = RuntimeError("TF late")
        n.start_publishing(1.0, 2.0, 0.0, "fixed")
        n.pub.publish.assert_not_called()
        n.create_timer.assert_called_once()

    def test_revalidation_preserves_requested_publish_count(self):
        n = self.node
        n.publish_count = 2
        n.scan_callback(scan(6.3))
        n.scan_callback(scan(26.3))
        self.assertEqual(n.sent, 1)
        self.now = 27.0
        n.publish_pose()
        n.scan_callback(scan(26.9))
        self.assertEqual(n.sent, 2)
        self.assertEqual(n.pub.publish.call_count, 2)
        n.scan_callback(scan(27.0))
        self.assertEqual(n.pub.publish.call_count, 2)

    def test_future_scan_is_ignored(self):
        self.node.scan_callback(scan(7.0))
        self.node.matcher.estimate.assert_not_called()
        self.node.pub.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()
