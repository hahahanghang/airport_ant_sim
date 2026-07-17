"""局部感知邻居和通信邻居搜索测试。"""

import random
import unittest

from agents.manager import UGVManager
from agents.ugv import UGV
from airport_map import AirportMap
from communication.neighborhood import (
    BruteForceNeighborSearch,
    SpatialHashNeighborSearch,
)
from config import (
    UGV_COMMUNICATION_RANGE_M,
    UGV_SENSING_RANGE_M,
)


class NeighborSearchTests(unittest.TestCase):
    """验证范围配置、局部隔离、顺序无关性和规模可用性。"""

    def setUp(self) -> None:
        self.search = BruteForceNeighborSearch()

    def test_configured_boundaries_separate_neighbor_types(self) -> None:
        agents = [
            UGV(0, (0.0, 0.0), 0.0),
            UGV(1, (UGV_SENSING_RANGE_M, 0.0), 0.0),
            UGV(2, (UGV_SENSING_RANGE_M + 20.0, 0.0), 0.0),
            UGV(3, (UGV_COMMUNICATION_RANGE_M + 1.0, 0.0), 0.0),
        ]

        result = self.search.search(agents)[0]

        self.assertEqual(
            {observation.agent_id for observation in result.sensed},
            {1},
        )
        self.assertEqual(
            {observation.agent_id for observation in result.communicable},
            {1, 2},
        )

    def test_relative_observation_does_not_expose_absolute_position(self) -> None:
        first = UGV(0, (100.0, 200.0), 0.0)
        second = UGV(1, (130.0, 240.0), 0.0)

        observation = self.search.search([first, second])[0].sensed[0]

        self.assertEqual(observation.relative_position, (30.0, 40.0))
        self.assertEqual(observation.distance_m, 50.0)
        self.assertFalse(hasattr(observation, "position"))

    def test_each_agent_uses_its_own_configured_ranges(self) -> None:
        wide = UGV(
            0,
            (0.0, 0.0),
            0.0,
            sensing_range_m=50.0,
            communication_range_m=80.0,
        )
        narrow = UGV(
            1,
            (30.0, 0.0),
            0.0,
            sensing_range_m=10.0,
            communication_range_m=20.0,
        )

        result = self.search.search([wide, narrow])

        self.assertEqual(
            {observation.agent_id for observation in result[0].sensed},
            {1},
        )
        self.assertEqual(result[1].sensed, ())
        self.assertEqual(
            {observation.agent_id for observation in result[0].communicable},
            {1},
        )
        self.assertEqual(result[1].communicable, ())

    def test_search_is_independent_of_agent_list_order(self) -> None:
        agents = [
            UGV(0, (0.0, 0.0), 0.0),
            UGV(1, (30.0, 0.0), 0.0),
            UGV(2, (90.0, 0.0), 0.0),
        ]

        forward = self.search.search(agents)
        reverse = self.search.search(list(reversed(agents)))

        self.assertEqual(forward, reverse)

    def test_invalid_ranges_and_duplicate_ids_have_clear_errors(self) -> None:
        with self.assertRaisesRegex(ValueError, "sensing_range_m"):
            UGV(0, (0.0, 0.0), 0.0, sensing_range_m=0.0)
        with self.assertRaisesRegex(ValueError, "communication_range_m"):
            UGV(0, (0.0, 0.0), 0.0, communication_range_m=-1.0)

        duplicate = [
            UGV(0, (0.0, 0.0), 0.0),
            UGV(0, (10.0, 0.0), 0.0),
        ]
        with self.assertRaisesRegex(ValueError, "重复节点编号"):
            self.search.search(duplicate)

    def test_manager_refreshes_only_each_agents_local_result(self) -> None:
        manager = UGVManager(AirportMap())
        first, second, third = manager.deploy_in_staging(3)
        first.position = (1500.0, 2200.0)
        second.position = (1530.0, 2200.0)
        third.position = (1600.0, 2200.0)

        manager.refresh_neighborhoods()

        self.assertEqual(set(first.sensed_neighbors), {second.agent_id})
        self.assertEqual(
            set(first.communication_neighbors),
            {second.agent_id, third.agent_id},
        )
        self.assertNotIn(first.agent_id, first.neighbors)
        self.assertEqual(
            first.neighbors,
            set(first.sensed_neighbors) | set(first.communication_neighbors),
        )

    def test_manager_refreshes_neighbors_after_motion_commit(self) -> None:
        manager = UGVManager(
            AirportMap(),
            neighbor_update_interval_s=1.0,
        )
        first, second = manager.deploy_in_staging(2)
        first.position = (1500.0, 2200.0)
        first.heading_rad = 0.0
        first.speed_mps = 1.0
        second.position = (
            1500.0 + UGV_SENSING_RANGE_M + 0.5,
            2200.0,
        )
        second.speed_mps = 0.0
        manager.refresh_neighborhoods()
        self.assertNotIn(second.agent_id, first.sensed_neighbors)

        manager.update_all(1.0, simulation_time_s=1.0)

        self.assertIn(second.agent_id, first.sensed_neighbors)
        self.assertLessEqual(
            first.sensed_neighbors[second.agent_id].distance_m,
            first.sensing_range_m,
        )

    def test_spatial_hash_matches_brute_force_for_200_agents(self) -> None:
        random_generator = random.Random(20260717)
        agents = [
            UGV(
                agent_id=agent_id,
                position=(
                    random_generator.uniform(0.0, 3000.0),
                    random_generator.uniform(0.0, 3000.0),
                ),
                heading_rad=0.0,
                sensing_range_m=random_generator.choice(
                    (30.0, 60.0, 90.0)
                ),
                communication_range_m=random_generator.choice(
                    (80.0, 120.0, 160.0)
                ),
            )
            for agent_id in range(200)
        ]

        expected = BruteForceNeighborSearch().search(agents)
        actual = SpatialHashNeighborSearch().search(agents)

        self.assertEqual(actual, expected)

    def test_manager_uses_spatial_hash_by_default(self) -> None:
        manager = UGVManager(AirportMap())

        self.assertIsInstance(
            manager.neighbor_search,
            SpatialHashNeighborSearch,
        )

    def test_50_and_200_agent_searches_complete(self) -> None:
        for count in (50, 200):
            with self.subTest(count=count):
                manager = UGVManager(AirportMap())
                agents = manager.deploy_in_staging(count)

                self.assertEqual(len(agents), count)
                for agent in agents:
                    self.assertNotIn(agent.agent_id, agent.neighbors)
                    self.assertTrue(
                        set(agent.sensed_neighbors).issubset(
                            set(range(count))
                        )
                    )
                    self.assertTrue(
                        set(agent.communication_neighbors).issubset(
                            set(range(count))
                        )
                    )


if __name__ == "__main__":
    unittest.main()
