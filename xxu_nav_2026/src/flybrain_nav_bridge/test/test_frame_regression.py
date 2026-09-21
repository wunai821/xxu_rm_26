import math
from types import SimpleNamespace
from unittest.mock import Mock
import unittest

from geometry_msgs.msg import TransformStamped, TwistStamped
from tf2_ros import Buffer, TransformException

from flybrain_nav_bridge.controller_fusion import TwistCommand
from flybrain_nav_bridge.flybrain_node import FlyBrainNode
from flybrain_nav_bridge.safety_gate import SafetyGate


class FrameRegression(unittest.TestCase):
    def test_tf_rotates_auxiliary_into_command_frame(self):
        buffer = Buffer()
        transform = TransformStamped()
        transform.header.frame_id = 'gimbal_yaw_fake'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.z = math.sin(math.pi / 4)
        transform.transform.rotation.w = math.cos(math.pi / 4)
        buffer.set_transform_static(transform, 'test')
        node = SimpleNamespace(tf_buffer=buffer, expected_scan_frame='base_footprint', scan_timeout_s=.5)
        source = TwistStamped()
        source.header.frame_id = 'gimbal_yaw_fake'
        yaw = FlyBrainNode._front_yaw(node, source)
        result = TwistCommand(0, -1).rotated(yaw)
        self.assertAlmostEqual(result.vx, 1)
        self.assertAlmostEqual(result.vy, 0)
        source.header.frame_id = 'missing'
        with self.assertRaises(TransformException):
            FlyBrainNode._front_yaw(node, source)
        source.header.frame_id = ''
        with self.assertRaises(ValueError):
            FlyBrainNode._front_yaw(node, source)

    def test_missing_transform_publishes_stop_and_resets_history(self):
        node = SimpleNamespace(
            _front_yaw=Mock(side_effect=ValueError('missing transform')),
            safety_gate=Mock(spec=SafetyGate),
            fused_debug_pub=Mock(), output_pub=Mock(),
            get_logger=Mock(return_value=Mock()),
            _publish_command=FlyBrainNode._publish_command)
        source = TwistStamped()
        source.header.frame_id = 'gimbal_yaw_fake'
        source.twist.linear.x = 1.0
        FlyBrainNode._mppi_callback(node, source)
        node.safety_gate.reset.assert_called_once()
        output = node.output_pub.publish.call_args.args[0]
        self.assertEqual(output.twist.linear.x, 0)
        self.assertEqual(output.twist.linear.y, 0)
        self.assertEqual(output.twist.angular.z, 0)
        self.assertEqual(output.header.frame_id, source.header.frame_id)

    def test_stale_transform_is_rejected(self):
        buffer = Buffer()
        transform = TransformStamped()
        transform.header.frame_id = 'gimbal_yaw_fake'
        transform.child_frame_id = 'base_footprint'
        transform.header.stamp.sec = 1
        transform.transform.rotation.w = 1.0
        buffer.set_transform(transform, 'test')
        node = SimpleNamespace(tf_buffer=buffer, expected_scan_frame='base_footprint', scan_timeout_s=.5)
        source = TwistStamped()
        source.header.frame_id = 'gimbal_yaw_fake'
        source.header.stamp.sec = 2
        with self.assertRaisesRegex(ValueError, 'stale'):
            FlyBrainNode._front_yaw(node, source)


if __name__ == '__main__':
    unittest.main()
