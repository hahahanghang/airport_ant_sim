"""覆盖信息素沉积、挥发、局部读取和管理器接入测试。"""

import math
import unittest

import numpy as np
import pygame

from agents.manager import UGVManager
from airport_map import AirportMap
from main import draw_coverage_heatmap
from pheromone.field import CoverageField


class CoverageFieldTests(unittest.TestCase):
    """验证覆盖场数值正确，并且单车只能获得有限局部观察。"""

    def test_default_grid_matches_world_size(self) -> None:
        coverage = CoverageField()

        self.assertEqual(coverage.values.shape, (100, 100))
        self.assertEqual(coverage.world_to_cell((0.0, 0.0)), (0, 0))
        self.assertEqual(
            coverage.world_to_cell((3000.0, 3000.0)),
            (99, 99),
        )

    def test_deposit_accumulates_and_saturates(self) -> None:
        coverage = CoverageField(maximum_value=1.0)
        position = (1500.0, 1500.0)

        coverage.deposit_many([position, position], 0.3)
        self.assertAlmostEqual(coverage.sample(position), 0.6, places=6)

        coverage.deposit_many([position, position], 0.5)
        self.assertEqual(coverage.sample(position), 1.0)

    def test_evaporation_uses_simulation_time(self) -> None:
        coverage = CoverageField(evaporation_rate_per_s=0.2)
        position = (1500.0, 1500.0)
        coverage.deposit_many([position], 1.0)

        coverage.evaporate(2.0)

        self.assertAlmostEqual(
            coverage.sample(position),
            math.exp(-0.4),
            places=6,
        )

    def test_local_observation_contains_only_four_values(self) -> None:
        coverage = CoverageField()
        position = (1500.0, 1500.0)
        points_and_values = (
            (position, 0.1),
            ((1560.0, 1500.0), 0.2),
            ((1500.0, 1440.0), 0.3),
            ((1500.0, 1560.0), 0.4),
        )
        for point, value in points_and_values:
            row, column = coverage.world_to_cell(point)
            coverage.values[row, column] = value

        observation = coverage.observe_local(position, 0.0, 60.0)

        self.assertAlmostEqual(observation.center, 0.1, places=6)
        self.assertAlmostEqual(observation.forward, 0.2, places=6)
        self.assertAlmostEqual(observation.left, 0.3, places=6)
        self.assertAlmostEqual(observation.right, 0.4, places=6)
        self.assertFalse(hasattr(observation, "values"))

    def test_manager_deposits_and_refreshes_only_local_observation(self) -> None:
        manager = UGVManager(
            AirportMap(),
            coverage_observation_interval_s=0.5,
        )
        ugv = manager.deploy_in_staging(1)[0]
        self.assertIsNotNone(ugv.local_coverage)
        self.assertEqual(ugv.local_coverage.center, 0.0)

        manager.update_all(0.5, simulation_time_s=0.5)

        self.assertGreater(manager.coverage_field.sample(ugv.position), 0.0)
        self.assertGreater(ugv.local_coverage.center, 0.0)
        self.assertNotIn("coverage_field", vars(ugv))
        self.assertFalse(
            any(
                isinstance(value, np.ndarray)
                for value in vars(ugv).values()
            )
        )

    def test_heatmap_draws_deposited_cell(self) -> None:
        pygame.init()
        surface = pygame.Surface((300, 300))
        surface.fill((0, 0, 0))
        airport_map = AirportMap()
        airport_map.update_viewport(300, 300)
        coverage = CoverageField()
        position = (1500.0, 1500.0)
        coverage.deposit_many([position], 1.0)
        pixel = airport_map.world_to_screen_point(position)
        before = surface.get_at(pixel)

        draw_coverage_heatmap(surface, airport_map, coverage)

        self.assertNotEqual(surface.get_at(pixel), before)

    def test_invalid_inputs_have_clear_errors(self) -> None:
        with self.assertRaisesRegex(ValueError, "cell_size_m"):
            CoverageField(cell_size_m=0.0)
        coverage = CoverageField()
        with self.assertRaisesRegex(ValueError, "x坐标"):
            coverage.sample((-1.0, 0.0))
        with self.assertRaisesRegex(ValueError, "delta_time_s"):
            coverage.evaporate(-0.1)
        with self.assertRaisesRegex(ValueError, "amount"):
            coverage.deposit_many([(0.0, 0.0)], -1.0)


if __name__ == "__main__":
    unittest.main()
