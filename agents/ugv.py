"""小型无人车状态与轻量级二维运动学。"""

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Deque, Dict, Optional, Set, Tuple

from airport_map import AirportMap
from config import (
    UGV_COMMUNICATION_RANGE_M,
    UGV_LOCAL_HISTORY_LENGTH,
    UGV_MAX_ACCELERATION_MPS2,
    UGV_MAX_SPEED_MPS,
    UGV_MAX_TURN_RATE_DEG_S,
    UGV_RADIUS_M,
    UGV_SENSING_RANGE_M,
)


@dataclass(frozen=True)
class UGVHistoryEntry:
    """无人车在某个仿真时刻保存的一条本地历史记录。"""

    simulation_time_s: float
    position: Tuple[float, float]
    heading_rad: float
    speed_mps: float


@dataclass
class UGV:
    """单辆无人车的本地状态和运动学模型。"""

    agent_id: int
    position: Tuple[float, float]
    heading_rad: float
    speed_mps: float = 0.0
    role: str = "idle"
    current_task: Optional[str] = None
    energy_level: float = 1.0
    health_status: str = "healthy"
    sensing_range_m: float = UGV_SENSING_RANGE_M
    communication_range_m: float = UGV_COMMUNICATION_RANGE_M
    radius_m: float = UGV_RADIUS_M
    neighbors: Set[int] = field(default_factory=set)
    response_thresholds: Dict[str, float] = field(default_factory=dict)
    local_pheromone: Dict[str, float] = field(default_factory=dict)
    local_history: Deque[UGVHistoryEntry] = field(
        default_factory=lambda: deque(maxlen=UGV_LOCAL_HISTORY_LENGTH)
    )

    def __post_init__(self) -> None:
        """检查创建无人车时最容易输入错误的参数。"""

        if self.agent_id < 0:
            raise ValueError("agent_id不能为负数")
        if self.radius_m <= 0.0:
            raise ValueError("radius_m必须大于0")
        if not 0.0 <= self.energy_level <= 1.0:
            raise ValueError("energy_level必须位于0到1之间")
        if not 0.0 <= self.speed_mps <= UGV_MAX_SPEED_MPS:
            raise ValueError("speed_mps超出允许范围")

        self.heading_rad = self.normalize_heading(self.heading_rad)

    @staticmethod
    def normalize_heading(heading_rad: float) -> float:
        """将航向角规范到[-π, π)范围。"""

        return (heading_rad + math.pi) % (2.0 * math.pi) - math.pi

    def heading_vector(self) -> Tuple[float, float]:
        """返回当前航向对应的二维单位向量。"""

        return math.cos(self.heading_rad), math.sin(self.heading_rad)

    def update(
        self,
        delta_time_s: float,
        airport_map: AirportMap,
        *,
        acceleration_mps2: float = 0.0,
        turn_rate_rad_s: float = 0.0,
        allow_restricted: bool = False,
        simulation_time_s: float = 0.0,
    ) -> None:
        """根据控制输入推进一个固定时间步，并阻止车辆穿过障碍。"""

        if delta_time_s < 0.0:
            raise ValueError("delta_time_s不能为负数")
        if delta_time_s == 0.0:
            return

        if self.health_status != "healthy":
            self.speed_mps = 0.0
            self._record_history(simulation_time_s)
            return

        acceleration = max(
            -UGV_MAX_ACCELERATION_MPS2,
            min(acceleration_mps2, UGV_MAX_ACCELERATION_MPS2),
        )
        maximum_turn_rate = math.radians(UGV_MAX_TURN_RATE_DEG_S)
        turn_rate = max(
            -maximum_turn_rate,
            min(turn_rate_rad_s, maximum_turn_rate),
        )

        next_speed = max(
            0.0,
            min(
                self.speed_mps + acceleration * delta_time_s,
                UGV_MAX_SPEED_MPS,
            ),
        )
        self.heading_rad = self.normalize_heading(
            self.heading_rad + turn_rate * delta_time_s
        )

        distance_m = next_speed * delta_time_s
        if distance_m > 0.0:
            completed = self._move_with_collision_checks(
                distance_m,
                airport_map,
                allow_restricted,
            )
            self.speed_mps = next_speed if completed else 0.0
        else:
            self.speed_mps = next_speed

        self._record_history(simulation_time_s)

    def _move_with_collision_checks(
        self,
        distance_m: float,
        airport_map: AirportMap,
        allow_restricted: bool,
    ) -> bool:
        """分成多个短距离检查，避免单个大时间步跨过薄障碍。"""

        step_length_m = max(0.5, self.radius_m)
        step_count = max(1, math.ceil(distance_m / step_length_m))
        distance_per_step = distance_m / step_count
        direction_x, direction_y = self.heading_vector()

        for _ in range(step_count):
            candidate = (
                self.position[0] + direction_x * distance_per_step,
                self.position[1] + direction_y * distance_per_step,
            )
            if not airport_map.is_position_drivable(
                candidate,
                self.radius_m,
                allow_restricted,
            ):
                return False
            self.position = candidate

        return True

    def _record_history(self, simulation_time_s: float) -> None:
        """保存有限长度的本地状态历史。"""

        self.local_history.append(
            UGVHistoryEntry(
                simulation_time_s=simulation_time_s,
                position=self.position,
                heading_rad=self.heading_rad,
                speed_mps=self.speed_mps,
            )
        )
