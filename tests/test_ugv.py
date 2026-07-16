"""单辆无人车状态与运动学测试。"""

import math
import unittest

from agents.ugv import UGV
from airport_map import AirportMap, Passability
from config import (
    UGV_LOCAL_HISTORY_LENGTH,
    UGV_MAX_ACCELERATION_MPS2,
    UGV_MAX_SPEED_MPS,
    UGV_MAX_TURN_RATE_DEG_S,
)


class UGVKinematicsTests(unittest.TestCase):
    """验证单车移动、转向、限幅和碰撞约束。"""

    def setUp(self) -> None:
        self.airport_map = AirportMap()

    def test_vehicle_moves_along_heading(self) -> None:
        ugv = UGV(
            agent_id=0,
            position=(1500.0, 2200.0),
            heading_rad=0.0,
            speed_mps=5.0,
        )

        ugv.update(1.0, self.airport_map)

        self.assertAlmostEqual(ugv.position[0], 1505.0)
        self.assertAlmostEqual(ugv.position[1], 2200.0)

    def test_acceleration_and_speed_are_limited(self) -> None:
        ugv = UGV(0, (1500.0, 2200.0), 0.0)
        ugv.update(
            1.0,
            self.airport_map,
            acceleration_mps2=UGV_MAX_ACCELERATION_MPS2 * 10.0,
        )
        self.assertEqual(ugv.speed_mps, UGV_MAX_ACCELERATION_MPS2)

        fast_ugv = UGV(
            1,
            (1500.0, 2200.0),
            0.0,
            speed_mps=UGV_MAX_SPEED_MPS,
        )
        fast_ugv.update(
            1.0,
            self.airport_map,
            acceleration_mps2=UGV_MAX_ACCELERATION_MPS2,
        )
        self.assertEqual(fast_ugv.speed_mps, UGV_MAX_SPEED_MPS)

    def test_turn_rate_is_limited(self) -> None:
        ugv = UGV(0, (1500.0, 2200.0), 0.0)
        ugv.update(
            1.0,
            self.airport_map,
            turn_rate_rad_s=math.radians(180.0),
        )
        self.assertAlmostEqual(
            ugv.heading_rad,
            math.radians(UGV_MAX_TURN_RATE_DEG_S),
        )

    def test_vehicle_cannot_enter_building(self) -> None:
        ugv = UGV(
            agent_id=0,
            position=(1000.0, 650.0),
            heading_rad=0.0,
            speed_mps=UGV_MAX_SPEED_MPS,
        )

        ugv.update(3.0, self.airport_map)

        self.assertLessEqual(ugv.position[0], 1017.0)
        self.assertEqual(ugv.speed_mps, 0.0)
        self.assertEqual(
            self.airport_map.get_passability(ugv.position, ugv.radius_m),
            Passability.DRIVABLE,
        )

    def test_runway_requires_explicit_permission(self) -> None:
        ugv = UGV(0, (1500.0, 1550.0), -math.pi / 2.0, speed_mps=8.0)
        ugv.update(5.0, self.airport_map)
        self.assertGreaterEqual(ugv.position[1], 1533.0)
        self.assertEqual(ugv.speed_mps, 0.0)

        inspection_ugv = UGV(
            1,
            (1500.0, 1550.0),
            -math.pi / 2.0,
            speed_mps=8.0,
        )
        inspection_ugv.update(
            5.0,
            self.airport_map,
            allow_restricted=True,
        )
        self.assertLess(inspection_ugv.position[1], 1530.0)

    def test_unhealthy_vehicle_stops(self) -> None:
        ugv = UGV(
            0,
            (1500.0, 2200.0),
            0.0,
            speed_mps=5.0,
            health_status="failed",
        )
        ugv.update(1.0, self.airport_map)
        self.assertEqual(ugv.position, (1500.0, 2200.0))
        self.assertEqual(ugv.speed_mps, 0.0)

    def test_local_history_has_fixed_length(self) -> None:
        ugv = UGV(0, (1500.0, 2200.0), 0.0)
        for step in range(UGV_LOCAL_HISTORY_LENGTH + 10):
            ugv.update(
                0.1,
                self.airport_map,
                simulation_time_s=step * 0.1,
            )
        self.assertEqual(len(ugv.local_history), UGV_LOCAL_HISTORY_LENGTH)

    def test_invalid_values_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "agent_id"):
            UGV(-1, (1500.0, 2200.0), 0.0)

        ugv = UGV(0, (1500.0, 2200.0), 0.0)
        with self.assertRaisesRegex(ValueError, "delta_time_s"):
            ugv.update(-0.1, self.airport_map)


if __name__ == "__main__":
    unittest.main()
