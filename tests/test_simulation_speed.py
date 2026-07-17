"""界面仿真倍速配置、切换和布局测试。"""

import math
import time
import unittest

import pygame

from agents.manager import UGVManager
from airport_map import AirportMap
from config import (
    DEFAULT_SIMULATION_SPEED,
    SIMULATION_DT_S,
    SIMULATION_SPEED_OPTIONS,
    UGV_COLLISION_CLEARANCE_M,
)
from main import (
    calculate_interface_layout,
    calculate_render_fps,
    calculate_speed_control_layout,
    change_simulation_speed,
    scaled_frame_time,
)


class SimulationSpeedTests(unittest.TestCase):
    """验证倍速不会改变固定时间步，并能在界面中安全选择。"""

    def test_configured_options_cover_half_to_twenty_times(self) -> None:
        self.assertEqual(
            SIMULATION_SPEED_OPTIONS,
            (0.5, 1.0, 2.0, 5.0, 10.0, 20.0),
        )
        self.assertIn(DEFAULT_SIMULATION_SPEED, SIMULATION_SPEED_OPTIONS)

    def test_keyboard_speed_change_follows_configured_order(self) -> None:
        self.assertEqual(change_simulation_speed(1.0, 1), 2.0)
        self.assertEqual(change_simulation_speed(1.0, -1), 0.5)
        self.assertEqual(change_simulation_speed(0.5, -1), 0.5)
        self.assertEqual(change_simulation_speed(20.0, 1), 20.0)

        with self.assertRaisesRegex(ValueError, "direction"):
            change_simulation_speed(1.0, 0)
        with self.assertRaisesRegex(ValueError, "current_speed"):
            change_simulation_speed(3.0, 1)

    def test_frame_time_is_scaled_or_paused(self) -> None:
        self.assertEqual(scaled_frame_time(0.1, 0.5, False), 0.05)
        self.assertEqual(scaled_frame_time(0.1, 20.0, False), 2.0)
        self.assertEqual(scaled_frame_time(0.1, 20.0, True), 0.0)

        with self.assertRaisesRegex(ValueError, "frame_time_s"):
            scaled_frame_time(-0.1, 1.0, False)
        with self.assertRaisesRegex(ValueError, "simulation_speed"):
            scaled_frame_time(0.1, 0.0, False)

    def test_speed_buttons_stay_inside_sidebar_without_overlap(self) -> None:
        pygame.font.init()
        for size in ((640, 480), (1280, 720), (1920, 1080)):
            with self.subTest(size=size):
                sidebar, _ = calculate_interface_layout(size)
                font = pygame.font.Font(None, 13)
                button_rects, pause_rect = calculate_speed_control_layout(
                    sidebar,
                    font,
                )
                all_rects = list(button_rects.values()) + [pause_rect]

                self.assertEqual(set(button_rects), set(SIMULATION_SPEED_OPTIONS))
                for rect in all_rects:
                    self.assertTrue(sidebar.contains(rect))
                for index, first in enumerate(all_rects):
                    for second in all_rects[index + 1:]:
                        self.assertFalse(first.colliderect(second))

    def test_render_rate_is_decoupled_for_200_agents_at_twenty_times(self) -> None:
        self.assertEqual(calculate_render_fps(20, 20.0, False), 60)
        self.assertEqual(calculate_render_fps(200, 20.0, False), 15)
        self.assertEqual(calculate_render_fps(200, 20.0, True), 60)

        with self.assertRaisesRegex(ValueError, "agent_count"):
            calculate_render_fps(0, 20.0, False)

    def test_200_agents_advance_twenty_seconds_within_one_second(self) -> None:
        manager = UGVManager(AirportMap())
        manager.deploy_in_staging(200)
        step_count = 200

        started = time.perf_counter()
        for step in range(step_count):
            manager.update_all(
                SIMULATION_DT_S,
                autonomous=True,
                simulation_time_s=(step + 1) * SIMULATION_DT_S,
            )
        elapsed_s = time.perf_counter() - started

        self.assertLess(
            elapsed_s,
            1.0,
            f"200节点20倍速性能未达标：耗时{elapsed_s:.3f}秒",
        )
        for index, first in enumerate(manager.agents):
            self.assertTrue(
                manager.airport_map.is_position_drivable(
                    first.position,
                    first.radius_m,
                )
            )
            for second in manager.agents[index + 1:]:
                minimum_distance = (
                    first.radius_m
                    + second.radius_m
                    + UGV_COLLISION_CLEARANCE_M
                )
                self.assertGreaterEqual(
                    math.dist(first.position, second.position),
                    minimum_distance,
                )


if __name__ == "__main__":
    unittest.main()
