"""无人车节点的可重复生成与批量更新。"""

import math
import random
from typing import Dict, List, Optional, Set, Tuple

from agents.ugv import UGV, UGVMotionProposal
from airport_map import AirportMap
from communication.neighborhood import (
    BruteForceNeighborSearch,
    NeighborSearch,
)
from config import (
    INITIAL_UGV_COUNT,
    RANDOM_SEED,
    UGV_DEPLOYMENT_SPACING_M,
    UGV_COLLISION_CLEARANCE_M,
    UGV_RADIUS_M,
)


class UGVManager:
    """由仿真器使用的无人车集合管理器。"""

    def __init__(
        self,
        airport_map: AirportMap,
        random_seed: int = RANDOM_SEED,
        neighbor_search: Optional[NeighborSearch] = None,
    ) -> None:
        self.airport_map = airport_map
        self.random_seed = random_seed
        self.neighbor_search = neighbor_search or BruteForceNeighborSearch()
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
        self.refresh_neighborhoods()
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
        self.refresh_neighborhoods()

    def refresh_neighborhoods(self) -> None:
        """由仿真管理器计算全局距离，只向各车写入自己的局部结果。"""

        neighborhoods = self.neighbor_search.search(self.agents)
        for ugv in self.agents:
            ugv.set_local_neighborhood(neighborhoods[ugv.agent_id])

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

            for index, first_id in enumerate(ordered_ids):
                first = agents_by_id[first_id]
                for second_id in ordered_ids[index + 1:]:
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
