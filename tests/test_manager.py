"""无人车批量部署管理器测试。"""

import math
import unittest

from agents.manager import UGVManager
from airport_map import AirportMap
from config import (
    INITIAL_UGV_COUNT,
    RANDOM_SEED,
    UGV_COLLISION_CLEARANCE_M,
    UGV_DEPLOYMENT_SPACING_M,
    UGV_RADIUS_M,
)


class UGVManagerTests(unittest.TestCase):
    """验证节点数量、部署范围、防重叠和随机可重复性。"""

    def setUp(self) -> None:
        self.airport_map = AirportMap()

    def test_default_deployment_has_unique_stationary_agents(self) -> None:
        manager = UGVManager(self.airport_map)
        agents = manager.deploy_in_staging()

        self.assertEqual(len(agents), INITIAL_UGV_COUNT)
        self.assertEqual(
            {agent.agent_id for agent in agents},
            set(range(INITIAL_UGV_COUNT)),
        )
        self.assertTrue(all(agent.speed_mps == 0.0 for agent in agents))

    def test_all_agents_share_one_meter_vehicle_parameters(self) -> None:
        manager = UGVManager(self.airport_map)
        agents = manager.deploy_in_staging()
        first = agents[0]

        self.assertEqual(UGV_RADIUS_M, 1.0)
        for agent in agents:
            with self.subTest(agent_id=agent.agent_id):
                self.assertEqual(agent.radius_m, 1.0)
                self.assertEqual(agent.sensing_range_m, first.sensing_range_m)
                self.assertEqual(
                    agent.communication_range_m,
                    first.communication_range_m,
                )
                self.assertEqual(agent.energy_level, first.energy_level)
                self.assertEqual(agent.health_status, first.health_status)

    def test_all_agents_are_inside_staging_and_drivable(self) -> None:
        manager = UGVManager(self.airport_map)
        agents = manager.deploy_in_staging()
        staging = self.airport_map.staging_area

        for agent in agents:
            with self.subTest(agent_id=agent.agent_id):
                x, y = agent.position
                self.assertGreaterEqual(x - agent.radius_m, staging.x)
                self.assertLessEqual(
                    x + agent.radius_m,
                    staging.x + staging.width,
                )
                self.assertGreaterEqual(y - agent.radius_m, staging.y)
                self.assertLessEqual(
                    y + agent.radius_m,
                    staging.y + staging.height,
                )
                self.assertTrue(
                    self.airport_map.is_position_drivable(
                        agent.position,
                        agent.radius_m,
                    )
                )

    def test_initial_positions_do_not_overlap(self) -> None:
        manager = UGVManager(self.airport_map)
        agents = manager.deploy_in_staging()

        for index, first in enumerate(agents):
            for second in agents[index + 1:]:
                with self.subTest(
                    first=first.agent_id,
                    second=second.agent_id,
                ):
                    self.assertGreaterEqual(
                        math.dist(first.position, second.position),
                        UGV_DEPLOYMENT_SPACING_M,
                    )

    def test_same_seed_repeats_positions_and_headings(self) -> None:
        first = UGVManager(self.airport_map, RANDOM_SEED)
        second = UGVManager(AirportMap(), RANDOM_SEED)

        first_agents = first.deploy_in_staging()
        second_agents = second.deploy_in_staging()

        first_state = [
            (agent.position, agent.heading_rad)
            for agent in first_agents
        ]
        second_state = [
            (agent.position, agent.heading_rad)
            for agent in second_agents
        ]
        self.assertEqual(first_state, second_state)

    def test_different_seed_changes_deployment(self) -> None:
        first = UGVManager(self.airport_map, RANDOM_SEED)
        second = UGVManager(AirportMap(), RANDOM_SEED + 1)
        first_positions = [
            agent.position for agent in first.deploy_in_staging()
        ]
        second_positions = [
            agent.position for agent in second.deploy_in_staging()
        ]
        self.assertNotEqual(first_positions, second_positions)

    def test_update_all_only_applies_controls_to_selected_agent(self) -> None:
        manager = UGVManager(self.airport_map)
        agents = manager.deploy_in_staging()
        original_positions = [agent.position for agent in agents]

        manager.update_all(
            1.0,
            controlled_agent_id=0,
            acceleration_mps2=2.5,
            turn_rate_rad_s=0.0,
            simulation_time_s=1.0,
        )

        self.assertNotEqual(agents[0].position, original_positions[0])
        self.assertGreater(agents[0].speed_mps, 0.0)
        self.assertEqual(
            [agent.position for agent in agents[1:]],
            original_positions[1:],
        )
        self.assertTrue(all(agent.speed_mps == 0.0 for agent in agents[1:]))

    def test_autonomous_mode_moves_all_agents(self) -> None:
        manager = UGVManager(self.airport_map)
        agents = manager.deploy_in_staging()
        original_positions = {
            agent.agent_id: agent.position
            for agent in agents
        }

        manager.update_all(
            1.0,
            autonomous=True,
            simulation_time_s=1.0,
        )

        self.assertTrue(
            all(
                agent.position != original_positions[agent.agent_id]
                for agent in agents
            )
        )
        self.assertTrue(all(agent.speed_mps > 0.0 for agent in agents))

    def test_same_seed_repeats_autonomous_motion(self) -> None:
        first = UGVManager(self.airport_map, RANDOM_SEED)
        second = UGVManager(AirportMap(), RANDOM_SEED)
        first.deploy_in_staging()
        second.deploy_in_staging()

        for step in range(50):
            simulation_time_s = (step + 1) * 0.1
            first.update_all(
                0.1,
                autonomous=True,
                simulation_time_s=simulation_time_s,
            )
            second.update_all(
                0.1,
                autonomous=True,
                simulation_time_s=simulation_time_s,
            )

        first_state = [
            (
                agent.agent_id,
                agent.position,
                agent.heading_rad,
                agent.speed_mps,
                agent.avoidance_turn_remaining_rad,
            )
            for agent in first.agents
        ]
        second_state = [
            (
                agent.agent_id,
                agent.position,
                agent.heading_rad,
                agent.speed_mps,
                agent.avoidance_turn_remaining_rad,
            )
            for agent in second.agents
        ]
        self.assertEqual(first_state, second_state)

    def test_autonomous_motion_preserves_map_and_vehicle_clearance(self) -> None:
        manager = UGVManager(self.airport_map)
        agents = manager.deploy_in_staging()

        for step in range(200):
            manager.update_all(
                0.1,
                autonomous=True,
                simulation_time_s=(step + 1) * 0.1,
            )
            for index, first in enumerate(agents):
                self.assertTrue(
                    self.airport_map.is_position_drivable(
                        first.position,
                        first.radius_m,
                    )
                )
                for second in agents[index + 1:]:
                    minimum_distance = (
                        first.radius_m
                        + second.radius_m
                        + UGV_COLLISION_CLEARANCE_M
                    )
                    self.assertGreaterEqual(
                        math.dist(first.position, second.position),
                        minimum_distance,
                    )

    def test_invalid_count_and_unknown_id_have_clear_errors(self) -> None:
        manager = UGVManager(self.airport_map)
        with self.assertRaisesRegex(ValueError, "count"):
            manager.deploy_in_staging(0)
        with self.assertRaisesRegex(ValueError, "最多可部署"):
            manager.deploy_in_staging(10000)

        manager.deploy_in_staging()
        with self.assertRaisesRegex(KeyError, "不存在编号"):
            manager.get_agent(100)

    def test_moving_agent_stops_before_another_agent(self) -> None:
        manager = UGVManager(self.airport_map)
        first, second = manager.deploy_in_staging(2)
        first.position = (1500.0, 2200.0)
        first.heading_rad = 0.0
        first.speed_mps = 5.0
        minimum_distance = (
            first.radius_m
            + second.radius_m
            + UGV_COLLISION_CLEARANCE_M
        )
        second.position = (
            first.position[0] + minimum_distance,
            first.position[1],
        )

        manager.update_all(
            0.1,
            controlled_agent_id=first.agent_id,
            simulation_time_s=0.1,
        )

        self.assertEqual(first.position, (1500.0, 2200.0))
        self.assertEqual(first.speed_mps, 0.0)
        self.assertEqual(manager.last_collision_agent_ids, {first.agent_id})
        self.assertEqual(manager.total_collision_blocks, 1)
        self.assertEqual(first.local_history[-1].position, first.position)

    def test_agent_can_move_away_without_collision(self) -> None:
        manager = UGVManager(self.airport_map)
        first, second = manager.deploy_in_staging(2)
        first.position = (1500.0, 2200.0)
        first.heading_rad = math.pi
        first.speed_mps = 5.0
        minimum_distance = (
            first.radius_m
            + second.radius_m
            + UGV_COLLISION_CLEARANCE_M
        )
        second.position = (
            first.position[0] + minimum_distance,
            first.position[1],
        )

        manager.update_all(
            0.1,
            controlled_agent_id=first.agent_id,
            simulation_time_s=0.1,
        )

        self.assertLess(first.position[0], 1500.0)
        self.assertEqual(manager.last_collision_agent_ids, set())

    def test_head_on_agents_are_blocked_simultaneously(self) -> None:
        manager = UGVManager(self.airport_map)
        first, second = manager.deploy_in_staging(2)
        first.position = (1500.0, 2200.0)
        first.heading_rad = 0.0
        first.speed_mps = 5.0
        second.position = (1503.5, 2200.0)
        second.heading_rad = math.pi
        second.speed_mps = 5.0

        manager.update_all(0.1, simulation_time_s=0.1)

        self.assertEqual(first.position, (1500.0, 2200.0))
        self.assertEqual(second.position, (1503.5, 2200.0))
        self.assertEqual(first.speed_mps, 0.0)
        self.assertEqual(second.speed_mps, 0.0)
        self.assertEqual(manager.last_collision_agent_ids, {0, 1})

    def test_synchronous_result_does_not_depend_on_list_order(self) -> None:
        first_manager = UGVManager(self.airport_map)
        second_manager = UGVManager(AirportMap())
        first_manager.deploy_in_staging(3)
        second_manager.deploy_in_staging(3)
        states = {
            0: ((1500.0, 2200.0), 0.0, 5.0),
            1: ((1503.5, 2200.0), math.pi, 5.0),
            2: ((1600.0, 2200.0), 0.0, 2.0),
        }
        for manager in (first_manager, second_manager):
            for agent in manager.agents:
                position, heading, speed = states[agent.agent_id]
                agent.position = position
                agent.heading_rad = heading
                agent.speed_mps = speed
        second_manager.agents.reverse()

        first_manager.update_all(0.1, simulation_time_s=0.1)
        second_manager.update_all(0.1, simulation_time_s=0.1)

        first_result = {
            agent.agent_id: (agent.position, agent.heading_rad, agent.speed_mps)
            for agent in first_manager.agents
        }
        second_result = {
            agent.agent_id: (agent.position, agent.heading_rad, agent.speed_mps)
            for agent in second_manager.agents
        }
        self.assertEqual(first_result, second_result)
        self.assertEqual(
            first_manager.last_collision_agent_ids,
            second_manager.last_collision_agent_ids,
        )

    def test_collision_clearance_is_preserved(self) -> None:
        manager = UGVManager(self.airport_map)
        first, second = manager.deploy_in_staging(2)
        first.position = (1450.0, 2200.0)
        first.heading_rad = 0.0
        first.speed_mps = 8.0
        second.position = (1470.0, 2200.0)

        for step in range(30):
            manager.update_all(
                0.1,
                controlled_agent_id=first.agent_id,
                acceleration_mps2=2.5,
                simulation_time_s=(step + 1) * 0.1,
            )

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
