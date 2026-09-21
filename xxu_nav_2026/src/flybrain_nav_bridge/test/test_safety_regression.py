import math
import unittest

from flybrain_nav_bridge.controller_fusion import TwistCommand
from flybrain_nav_bridge.safety_gate import SafetyGate
from flybrain_nav_bridge.threat_extractor import SectorThreat


class SafetyRegression(unittest.TestCase):
    def setUp(self):
        self.gate = SafetyGate(max_vx=1.5, max_vy=1.5, max_wz=.8,
                               max_ax=.6, max_ay=.4, max_awz=1.2,
                               max_jx=4, max_jy=4, max_jwz=8,
                               emergency_distance_m=.25)

    def apply(self, command, stamp, emergency=False, yaw=0):
        return self.gate.apply(
            candidate=command, mppi_fallback=TwistCommand(),
            threats={'front': SectorThreat('front', distance_m=.2, valid=True)} if emergency else {},
            scan_fresh=True, stamp_s=stamp, front_yaw=yaw)

    def test_emergency_does_not_cause_reverse(self):
        self.apply(TwistCommand(1.5), 1)
        self.assertEqual(self.apply(TwistCommand(1.5), 1.05, True).command.vx, 0)
        for i in range(1, 100):
            self.assertEqual(self.apply(TwistCommand(), 1.05 + i*.05).command.vx, 0)

    def test_velocity_bounds_and_no_target_overshoot(self):
        self.apply(TwistCommand(), 1)
        for i in range(1, 160):
            result = self.apply(TwistCommand(1.5, -1.5, .8), 1 + i*.05)
            self.assertTrue(0 <= result.command.vx <= 1.5)
            self.assertTrue(-1.5 <= result.command.vy <= 0)
            self.assertTrue(0 <= result.command.wz <= .8)
            self.assertLessEqual(abs(result.accel.vx), .6 + 1e-9)
            self.assertLessEqual(abs(result.accel.vy), .4 + 1e-9)

    def test_rotated_front_emergency(self):
        for yaw in (0, math.pi/2, math.pi, -math.pi/2, .7):
            self.gate.reset()
            command = TwistCommand(1, .2).rotated(yaw)
            self.apply(command, 1)
            result = self.apply(command, 1.05, True, yaw)
            body = result.command.rotated(-yaw)
            self.assertAlmostEqual(body.vx, 0, places=10)
            self.assertGreaterEqual(body.vy, 0)
            self.assertLessEqual(body.vy, .2 + 1e-10)

    def test_invalid_auxiliary_does_not_skip_emergency(self):
        result = self.gate.apply(
            candidate=TwistCommand(float('nan')), mppi_fallback=TwistCommand(1),
            threats={'front': SectorThreat('front', distance_m=.2, valid=True)},
            scan_fresh=True, stamp_s=1)
        self.assertEqual(result.command.vx, 0)


if __name__ == '__main__':
    unittest.main()
