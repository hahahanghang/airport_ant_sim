"""无人车节点的可重复生成与批量更新。"""

import math
import random
from typing import List, Optional, Set, Tuple

from agents.ugv import UGV
from airport_map import AirportMap
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
    ) -> None:
        self.airport_map = airport_map
        self.random_seed = random_seed
        self.agents: List[UGV] = []
        self.last_collision_agent_ids: Set[int] = set()
        self.total_collision_blocks = 0

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
        self.agents = [
            UGV(
                agent_id=agent_id,
                position=position,
                heading_rad=random_generator.uniform(-math.pi, math.pi),
            )
            for agent_id, position in enumerate(selected_slots)
        ]
        self.last_collision_agent_ids.clear()
        self.total_collision_blocks = 0
        return self.agents

    def get_agent(self, agent_id: int) -> UGV:
        """按唯一编号获取无人车，不存在时给出清晰错误。"""

        if not 0 <= agent_id < len(self.agents):
            raise KeyError(f"不存在编号为{agent_id}的无人车")
        return self.agents[agent_id]

    def update_all(
        self,
        delta_time_s: float,
        *,
        controlled_agent_id: Optional[int] = None,
        acceleration_mps2: float = 0.0,
        turn_rate_rad_s: float = 0.0,
        simulation_time_s: float = 0.0,
    ) -> None:
        """推进所有节点，只把人工控制输入交给指定节点。"""

        self.last_collision_agent_ids.clear()
        for ugv in self.agents:
            is_controlled = ugv.agent_id == controlled_agent_id
            ugv.update(
                delta_time_s,
                self.airport_map,
                acceleration_mps2=(acceleration_mps2 if is_controlled else 0.0),
                turn_rate_rad_s=(turn_rate_rad_s if is_controlled else 0.0),
                simulation_time_s=simulation_time_s,
                position_validator=self._is_clear_of_other_agents,
            )
        self.total_collision_blocks += len(self.last_collision_agent_ids)

    def _is_clear_of_other_agents(
        self,
        moving_agent_id: int,
        position: Tuple[float, float],
        radius_m: float,
    ) -> bool:
        """只向单车返回安全与否，不暴露其他节点的全局位置。"""

        for other in self.agents:
            if other.agent_id == moving_agent_id:
                continue
            minimum_distance = (
                radius_m
                + other.radius_m
                + UGV_COLLISION_CLEARANCE_M
            )
            if math.dist(position, other.position) < minimum_distance:
                self.last_collision_agent_ids.add(moving_agent_id)
                return False
        return True

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
