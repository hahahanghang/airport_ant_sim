"""无人车节点的可重复生成与批量更新。"""

import math
import random
from typing import Dict, List, Optional, Set, Tuple

from agents.ugv import UGV, UGVMotionProposal
from airport_map import AirportMap
from communication.neighborhood import (
    NeighborSearch,
    SpatialHashNeighborSearch,
)
from config import (
    COVERAGE_OBSERVATION_INTERVAL_S,
    INITIAL_UGV_COUNT,
    NEIGHBOR_UPDATE_INTERVAL_S,
    RANDOM_SEED,
    UGV_DEPLOYMENT_SPACING_M,
    UGV_COLLISION_CLEARANCE_M,
    UGV_RADIUS_M,
)
from pheromone.field import CoverageField


class UGVManager:
    """由仿真器使用的无人车集合管理器。"""

    def __init__(
        self,
        airport_map: AirportMap,
        random_seed: int = RANDOM_SEED,
        neighbor_search: Optional[NeighborSearch] = None,
        neighbor_update_interval_s: float = NEIGHBOR_UPDATE_INTERVAL_S,
        coverage_field: Optional[CoverageField] = None,
        coverage_observation_interval_s: float = (
            COVERAGE_OBSERVATION_INTERVAL_S
        ),
    ) -> None:
        if neighbor_update_interval_s <= 0.0:
            raise ValueError("neighbor_update_interval_s必须大于0")
        if coverage_observation_interval_s <= 0.0:
            raise ValueError("coverage_observation_interval_s必须大于0")
        self.airport_map = airport_map
        self.random_seed = random_seed
        self.neighbor_search = neighbor_search or SpatialHashNeighborSearch()
        self.neighbor_update_interval_s = neighbor_update_interval_s
        self.coverage_field = coverage_field or CoverageField()
        self.coverage_observation_interval_s = (
            coverage_observation_interval_s
        )
        self._neighbor_time_accumulator_s = 0.0
        self._coverage_observation_accumulator_s = 0.0
        self.agents: List[UGV] = []
        self.last_collision_agent_ids: Set[int] = set()
        self.last_map_blocked_agent_ids: Set[int] = set()
        self.total_collision_blocks = 0
        self.total_map_blocks = 0

    def deploy_in_staging(
        self,
        count: int = INITIAL_UGV_COUNT,
    ) -> List[UGV]:
        """在待命区的随机网格位置生成无重叠无人车。"""

        if count <= 0:
            raise ValueError("count必须大于0")
        if UGV_DEPLOYMENT_SPACING_M < 2.0 * UGV_RADIUS_M:
            raise ValueError("部署间距不能小于无人车直径")

        slots = self._build_deployment_slots()
        if count > len(slots):
            raise ValueError(
                f"待命区最多可部署{len(slots)}辆无人车，收到{count}辆"
            )

        random_generator = random.Random(self.random_seed)
        selected_slots = random_generator.sample(slots, count)
        self.agents = []
        for agent_id, position in enumerate(selected_slots):
            self.agents.append(
                UGV(
                    agent_id=agent_id,
                    position=position,
                    heading_rad=random_generator.uniform(-math.pi, math.pi),
                    avoidance_turn_direction=random_generator.choice((-1, 1)),
                )
            )
        self.last_collision_agent_ids.clear()
        self.last_map_blocked_agent_ids.clear()
        self.total_collision_blocks = 0
        self.total_map_blocks = 0
        self._neighbor_time_accumulator_s = 0.0
        self._coverage_observation_accumulator_s = 0.0
        self.coverage_field.reset()
        self.refresh_neighborhoods()
        self.refresh_local_coverage()
        return self.agents

    def get_agent(self, agent_id: int) -> UGV:
        """按唯一编号获取无人车，不存在时给出清晰错误。"""

        for agent in self.agents:
            if agent.agent_id == agent_id:
                return agent
        raise KeyError(f"不存在编号为{agent_id}的无人车")

    def update_all(
        self,
        delta_time_s: float,
        *,
        controlled_agent_id: Optional[int] = None,
        acceleration_mps2: float = 0.0,
        turn_rate_rad_s: float = 0.0,
        simulation_time_s: float = 0.0,
        autonomous: bool = False,
        manual_control_active: bool = False,
    ) -> None:
        """先生成全部候选状态，再同步检查并统一提交。"""

        if delta_time_s < 0.0:
            raise ValueError("delta_time_s不能为负数")
        if delta_time_s == 0.0:
            return

        proposals: Dict[int, UGVMotionProposal] = {}
        for ugv in self.agents:
            use_manual_control = (
                ugv.agent_id == controlled_agent_id
                and (not autonomous or manual_control_active)
            )
            if use_manual_control:
                acceleration = acceleration_mps2
                turn_rate = turn_rate_rad_s
            elif autonomous:
                acceleration, turn_rate = ugv.autonomous_control(delta_time_s)
            else:
                acceleration = 0.0
                turn_rate = 0.0

            proposals[ugv.agent_id] = ugv.propose_motion(
                delta_time_s,
                self.airport_map,
                acceleration_mps2=acceleration,
                turn_rate_rad_s=turn_rate,
            )

        self.last_map_blocked_agent_ids = {
            agent_id
            for agent_id, proposal in proposals.items()
            if proposal.map_blocked
        }
        self.last_collision_agent_ids = self._find_collision_blocks(proposals)
        blocked_agent_ids = (
            self.last_map_blocked_agent_ids | self.last_collision_agent_ids
        )

        for ugv in self.agents:
            ugv.apply_motion(
                proposals[ugv.agent_id],
                blocked=ugv.agent_id in blocked_agent_ids,
                simulation_time_s=simulation_time_s,
            )

        self.total_map_blocks += len(self.last_map_blocked_agent_ids)
        self.total_collision_blocks += len(self.last_collision_agent_ids)
        self.coverage_field.update(
            delta_time_s,
            [ugv.position for ugv in self.agents],
        )
        self._coverage_observation_accumulator_s += delta_time_s
        if (
            self._coverage_observation_accumulator_s + 1e-12
            >= self.coverage_observation_interval_s
        ):
            self.refresh_local_coverage()
            self._coverage_observation_accumulator_s %= (
                self.coverage_observation_interval_s
            )
        self._neighbor_time_accumulator_s += delta_time_s
        if (
            self._neighbor_time_accumulator_s + 1e-12
            >= self.neighbor_update_interval_s
        ):
            self.refresh_neighborhoods()
            self._neighbor_time_accumulator_s %= self.neighbor_update_interval_s

    def refresh_neighborhoods(self) -> None:
        """由仿真管理器计算全局距离，只向各车写入自己的局部结果。"""

        neighborhoods = self.neighbor_search.search(self.agents)
        for ugv in self.agents:
            ugv.set_local_neighborhood(neighborhoods[ugv.agent_id])

    def refresh_local_coverage(self) -> None:
        """只向每辆车写入以自身位置和航向为基准的四个覆盖值。"""

        observations = self.coverage_field.observe_many(
            [ugv.position for ugv in self.agents],
            [ugv.heading_rad for ugv in self.agents],
        )
        for ugv, observation in zip(self.agents, observations):
            ugv.set_local_coverage(observation)

    def _find_collision_blocks(
        self,
        proposals: Dict[int, UGVMotionProposal],
    ) -> Set[int]:
        """同步检查运动轨迹，返回本步因车辆冲突而停止的节点编号。"""

        agents_by_id = {
            agent.agent_id: agent
            for agent in self.agents
        }
        ordered_ids = sorted(agents_by_id)
        candidate_pairs = self._collision_candidate_pairs(
            agents_by_id,
            proposals,
        )
        if not candidate_pairs:
            return set()
        blocked_ids = {
            agent_id
            for agent_id, proposal in proposals.items()
            if proposal.map_blocked
        }
        collision_ids: Set[int] = set()

        while True:
            newly_blocked: Set[int] = set()
            final_positions = {
                agent_id: (
                    agents_by_id[agent_id].position
                    if agent_id in blocked_ids
                    else proposals[agent_id].position
                )
                for agent_id in ordered_ids
            }

            for first_id, second_id in candidate_pairs:
                first = agents_by_id[first_id]
                second = agents_by_id[second_id]
                minimum_distance = (
                    first.radius_m
                    + second.radius_m
                    + UGV_COLLISION_CLEARANCE_M
                )
                if not self._motions_conflict(
                    first.position,
                    final_positions[first_id],
                    second.position,
                    final_positions[second_id],
                    minimum_distance,
                ):
                    continue

                if (
                    first_id not in blocked_ids
                    and final_positions[first_id] != first.position
                ):
                    newly_blocked.add(first_id)
                if (
                    second_id not in blocked_ids
                    and final_positions[second_id] != second.position
                ):
                    newly_blocked.add(second_id)

            if not newly_blocked:
                return collision_ids

            collision_ids.update(newly_blocked)
            blocked_ids.update(newly_blocked)

    @staticmethod
    def _collision_candidate_pairs(
        agents_by_id: Dict[int, UGV],
        proposals: Dict[int, UGVMotionProposal],
    ) -> Tuple[Tuple[int, int], ...]:
        """用扫描线筛出运动轨迹包围盒可能重叠的车辆对。"""

        bounds = []
        for agent_id, agent in agents_by_id.items():
            proposal = proposals[agent_id]
            expansion = (
                agent.radius_m + UGV_COLLISION_CLEARANCE_M / 2.0
            )
            bounds.append(
                (
                    min(agent.position[0], proposal.position[0]) - expansion,
                    min(agent.position[1], proposal.position[1]) - expansion,
                    max(agent.position[0], proposal.position[0]) + expansion,
                    max(agent.position[1], proposal.position[1]) + expansion,
                    agent_id,
                )
            )
        bounds.sort(key=lambda item: (item[0], item[4]))

        candidate_pairs = []
        for index, first in enumerate(bounds):
            first_minimum_y = first[1]
            first_maximum_x = first[2]
            first_maximum_y = first[3]
            first_id = first[4]
            for second_index in range(index + 1, len(bounds)):
                second = bounds[second_index]
                if second[0] > first_maximum_x:
                    break
                if (
                    second[1] > first_maximum_y
                    or second[3] < first_minimum_y
                ):
                    continue
                candidate_pairs.append(
                    (min(first_id, second[4]), max(first_id, second[4]))
                )
        return tuple(sorted(candidate_pairs))

    @staticmethod
    def _motions_conflict(
        first_start: Tuple[float, float],
        first_end: Tuple[float, float],
        second_start: Tuple[float, float],
        second_end: Tuple[float, float],
        minimum_distance: float,
    ) -> bool:
        """计算两条同步直线运动轨迹在一个时间步内是否过近。"""

        relative_start = (
            first_start[0] - second_start[0],
            first_start[1] - second_start[1],
        )
        relative_velocity = (
            (first_end[0] - first_start[0])
            - (second_end[0] - second_start[0]),
            (first_end[1] - first_start[1])
            - (second_end[1] - second_start[1]),
        )
        velocity_squared = (
            relative_velocity[0] ** 2 + relative_velocity[1] ** 2
        )
        if velocity_squared == 0.0:
            closest_time = 0.0
        else:
            closest_time = max(
                0.0,
                min(
                    1.0,
                    -(
                        relative_start[0] * relative_velocity[0]
                        + relative_start[1] * relative_velocity[1]
                    )
                    / velocity_squared,
                ),
            )
        closest_offset = (
            relative_start[0] + relative_velocity[0] * closest_time,
            relative_start[1] + relative_velocity[1] * closest_time,
        )
        return math.hypot(*closest_offset) < minimum_distance

    def _build_deployment_slots(self) -> List[Tuple[float, float]]:
        """在待命区内构造满足车辆半径和固定间距的候选位置。"""

        staging = self.airport_map.staging_area
        minimum_x = staging.x + UGV_RADIUS_M
        maximum_x = staging.x + staging.width - UGV_RADIUS_M
        minimum_y = staging.y + UGV_RADIUS_M
        maximum_y = staging.y + staging.height - UGV_RADIUS_M

        columns = math.floor(
            (maximum_x - minimum_x) / UGV_DEPLOYMENT_SPACING_M
        ) + 1
        rows = math.floor(
            (maximum_y - minimum_y) / UGV_DEPLOYMENT_SPACING_M
        ) + 1

        slots: List[Tuple[float, float]] = []
        for row in range(rows):
            y = minimum_y + row * UGV_DEPLOYMENT_SPACING_M
            for column in range(columns):
                x = minimum_x + column * UGV_DEPLOYMENT_SPACING_M
                position = (x, y)
                if self.airport_map.is_position_drivable(position, UGV_RADIUS_M):
                    slots.append(position)

        return slots
