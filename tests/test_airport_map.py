"""机场地图通行语义测试。"""

import math
import unittest

from airport_map import AirportMap, Passability
from config import OPEN_AREA_MOVEMENT_COST, ROAD_MOVEMENT_COST


class AirportMapPassabilityTests(unittest.TestCase):
    """验证道路、限制区域和建筑障碍的分类。"""

    def setUp(self) -> None:
        self.airport_map = AirportMap()

    def test_roads_and_gate_are_drivable(self) -> None:
        self.assertEqual(
            self.airport_map.get_passability((200.0, 200.0)),
            Passability.DRIVABLE,
        )
        self.assertEqual(
            self.airport_map.get_passability((1500.0, 2200.0)),
            Passability.DRIVABLE,
        )
        self.assertEqual(
            self.airport_map.get_passability((1500.0, 2780.0)),
            Passability.DRIVABLE,
        )

    def test_runway_is_restricted(self) -> None:
        point = (1500.0, 1480.0)
        self.assertEqual(
            self.airport_map.get_passability(point),
            Passability.RESTRICTED,
        )
        self.assertFalse(self.airport_map.is_position_drivable(point))
        self.assertTrue(
            self.airport_map.is_position_drivable(point, allow_restricted=True)
        )

    def test_buildings_are_blocked(self) -> None:
        blocked_points = [
            (1150.0, 650.0),  # 机库1
            (610.0, 750.0),   # 塔台
            (2400.0, 740.0),  # 油料库
            (925.0, 2530.0),  # 充电站
            (2100.0, 2530.0), # 维修站
        ]
        for point in blocked_points:
            with self.subTest(point=point):
                self.assertEqual(
                    self.airport_map.get_passability(point),
                    Passability.BLOCKED,
                )

    def test_perimeter_outside_is_blocked(self) -> None:
        self.assertEqual(
            self.airport_map.get_passability((100.0, 1500.0)),
            Passability.BLOCKED,
        )

    def test_vehicle_radius_keeps_clear_of_building(self) -> None:
        point_near_hangar = (1000.0, 650.0)
        self.assertEqual(
            self.airport_map.get_passability(point_near_hangar, 10.0),
            Passability.DRIVABLE,
        )
        self.assertEqual(
            self.airport_map.get_passability(point_near_hangar, 25.0),
            Passability.BLOCKED,
        )

    def test_movement_cost_prefers_roads(self) -> None:
        self.assertEqual(
            self.airport_map.movement_cost_at((1500.0, 2200.0)),
            ROAD_MOVEMENT_COST,
        )
        self.assertEqual(
            self.airport_map.movement_cost_at((800.0, 1800.0)),
            OPEN_AREA_MOVEMENT_COST,
        )
        self.assertTrue(
            math.isinf(self.airport_map.movement_cost_at((1150.0, 650.0)))
        )

    def test_negative_vehicle_radius_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "vehicle_radius"):
            self.airport_map.get_passability((1500.0, 2200.0), -1.0)

    def test_all_buildings_have_visible_names(self) -> None:
        building_names = {
            region.name
            for region in [
                *self.airport_map.hangars,
                *self.airport_map.blocked_regions,
            ]
        }
        expected_names = {
            "机库 1",
            "机库 2",
            "机库 3",
            "塔台",
            "油料库",
            "充电站",
            "维修站",
        }
        self.assertTrue(expected_names.issubset(building_names))

    def test_offset_viewport_preserves_scale_and_coordinate_round_trip(self) -> None:
        self.airport_map.update_viewport(
            width=400,
            height=300,
            origin_x=200,
            origin_y=50,
        )

        self.assertEqual(self.airport_map.scale_x, 0.1)
        self.assertEqual(self.airport_map.scale_y, 0.1)
        self.assertEqual(
            self.airport_map.world_to_screen_point((1500.0, 1500.0)),
            (400, 200),
        )
        world_point = self.airport_map.screen_to_world_point((400, 200))
        self.assertAlmostEqual(world_point[0], 1500.0)
        self.assertAlmostEqual(world_point[1], 1500.0)


if __name__ == "__main__":
    unittest.main()
