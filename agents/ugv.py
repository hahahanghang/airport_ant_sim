"""小型无人车状态与轻量级二维运动学。"""

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Callable, Deque, Dict, Optional, Set, Tuple

from airport_map import AirportMap
from communication.neighborhood import LocalNeighborhood, NeighborObservation
from config import (
    UGV_AUTONOMOUS_ACCELERATION_MPS2,
    UGV_AUTONOMOUS_CRUISE_SPEED_MPS,
    UGV_AUTONOMOUS_TURN_ANGLE_DEG,
    UGV_AUTONOMOUS_TURN_RATE_DEG_S,
    UGV_COMMUNICATION_RANGE_M,
    UGV_LOCAL_HISTORY_LENGTH,
    UGV_MAX_ACCELERATION_MPS2,
    UGV_MAX_SPEED_MPS,
    UGV_MAX_TURN_RATE_DEG_S,
    UGV_RADIUS_M,
    UGV_SENSING_RANGE_M,
)


PositionValidator = Callable[[int, Tuple[float, float], float], bool]


@dataclass(frozen=True)
class UGVHistoryEntry:
    """无人车在某个仿真时刻保存的一条本地历史记录。"""

    simulation_time_s: float
    position: Tuple[float, float]
    heading_rad: float
    speed_mps: float


@dataclass(frozen=True)
class UGVMotionProposal:
    """单辆车为当前时间步生成、但尚未提交的候选运动状态。"""

    agent_id: int
    position: Tuple[float, float]
    heading_rad: float
    speed_mps: float
    path_points: Tuple[Tuple[float, float], ...]
    map_blocked: bool = False


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
    sensed_neighbors: Dict[int, NeighborObservation] = field(
        default_factory=dict
    )
    communication_neighbors: Dict[int, NeighborObservation] = field(
        default_factory=dict
    )
    response_thresholds: Dict[str, float] = field(default_factory=dict)
    local_pheromone: Dict[str, float] = field(default_factory=dict)
    local_history: Deque[UGVHistoryEntry] = field(
        default_factory=lambda: deque(maxlen=UGV_LOCAL_HISTORY_LENGTH)
    )
    avoidance_turn_direction: int = 1
    avoidance_turn_remaining_rad: float = 0.0
    was_blocked: bool = False

    def __post_init__(self) -> None:
        """检查创建无人车时最容易输入错误的参数。"""

        if self.agent_id < 0:
            raise ValueError("agent_id不能为负数")
        if self.radius_m <= 0.0:
            raise ValueError("radius_m必须大于0")
        if self.sensing_range_m <= 0.0:
            raise ValueError("sensing_range_m必须大于0")
        if self.communication_range_m <= 0.0:
            raise ValueError("communication_range_m必须大于0")
        if not 0.0 <= self.energy_level <= 1.0:
            raise ValueError("energy_level必须位于0到1之间")
        if not 0.0 <= self.speed_mps <= UGV_MAX_SPEED_MPS:
            raise ValueError("speed_mps超出允许范围")
        if self.avoidance_turn_direction not in (-1, 1):
            raise ValueError("avoidance_turn_direction必须为-1或1")
        if self.avoidance_turn_remaining_rad < 0.0:
            raise ValueError("avoidance_turn_remaining_rad不能为负数")

        self.heading_rad = self.normalize_heading(self.heading_rad)

    @staticmethod
    def normalize_heading(heading_rad: float) -> float:
        """将航向角规范到[-π, π)范围。"""

        return (heading_rad + math.pi) % (2.0 * math.pi) - math.pi

    def heading_vector(self) -> Tuple[float, float]:
        """返回当前航向对应的二维单位向量。"""

        return math.cos(self.heading_rad), math.sin(self.heading_rad)

    def set_local_neighborhood(
        self,
        neighborhood: LocalNeighborhood,
    ) -> None:
        """只接收属于本车的局部邻居观测，不接收全局车辆表。"""

        self.sensed_neighbors = {
            observation.agent_id: observation
            for observation in neighborhood.sensed
        }
        self.communication_neighbors = {
            observation.agent_id: observation
            for observation in neighborhood.communicable
        }
        self.neighbors = (
            set(self.sensed_neighbors) | set(self.communication_neighbors)
        )

    def autonomous_control(
        self,
        delta_time_s: float,
    ) -> Tuple[float, float]:
        """只根据本车状态生成基础自主加速度和转向速度。"""

        if delta_time_s <= 0.0 or self.health_status != "healthy":
            return 0.0, 0.0

        if self.avoidance_turn_remaining_rad > 0.0:
            maximum_turn_rate = min(
                math.radians(UGV_AUTONOMOUS_TURN_RATE_DEG_S),
                math.radians(UGV_MAX_TURN_RATE_DEG_S),
            )
            turn_this_step = min(
                self.avoidance_turn_remaining_rad,
                maximum_turn_rate * delta_time_s,
            )
            self.avoidance_turn_remaining_rad = max(
                0.0,
                self.avoidance_turn_remaining_rad - turn_this_step,
            )
            return (
                -UGV_AUTONOMOUS_ACCELERATION_MPS2,
                self.avoidance_turn_direction
                * turn_this_step
                / delta_time_s,
            )

        target_speed = min(
            UGV_AUTONOMOUS_CRUISE_SPEED_MPS,
            UGV_MAX_SPEED_MPS,
        )
        autonomous_acceleration = min(
            UGV_AUTONOMOUS_ACCELERATION_MPS2,
            UGV_MAX_ACCELERATION_MPS2,
        )
        speed_error = target_speed - self.speed_mps
        maximum_speed_change = autonomous_acceleration * delta_time_s
        if speed_error > maximum_speed_change:
            acceleration = autonomous_acceleration
        elif speed_error < -maximum_speed_change:
            acceleration = -autonomous_acceleration
        else:
            acceleration = speed_error / delta_time_s
        return acceleration, 0.0

    def propose_motion(
        self,
        delta_time_s: float,
        airport_map: AirportMap,
        *,
        acceleration_mps2: float = 0.0,
        turn_rate_rad_s: float = 0.0,
        allow_restricted: bool = False,
    ) -> UGVMotionProposal:
        """计算候选运动但不修改真实位置、航向和速度。"""

        if delta_time_s < 0.0:
            raise ValueError("delta_time_s不能为负数")
        if delta_time_s == 0.0 or self.health_status != "healthy":
            return UGVMotionProposal(
                agent_id=self.agent_id,
                position=self.position,
                heading_rad=self.heading_rad,
                speed_mps=0.0 if self.health_status != "healthy" else self.speed_mps,
                path_points=(),
            )

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
        next_heading = self.normalize_heading(
            self.heading_rad + turn_rate * delta_time_s
        )

        distance_m = next_speed * delta_time_s
        if distance_m == 0.0:
            return UGVMotionProposal(
                agent_id=self.agent_id,
                position=self.position,
                heading_rad=next_heading,
                speed_mps=next_speed,
                path_points=(),
            )

        step_length_m = max(0.5, self.radius_m)
        step_count = max(1, math.ceil(distance_m / step_length_m))
        distance_per_step = distance_m / step_count
        direction_x = math.cos(next_heading)
        direction_y = math.sin(next_heading)
        candidate = self.position
        path_points = []

        for _ in range(step_count):
            candidate = (
                candidate[0] + direction_x * distance_per_step,
                candidate[1] + direction_y * distance_per_step,
            )
            path_points.append(candidate)
            if not airport_map.is_position_drivable(
                candidate,
                self.radius_m,
                allow_restricted,
            ):
                return UGVMotionProposal(
                    agent_id=self.agent_id,
                    position=self.position,
                    heading_rad=next_heading,
                    speed_mps=0.0,
                    path_points=tuple(path_points),
                    map_blocked=True,
                )

        return UGVMotionProposal(
            agent_id=self.agent_id,
            position=candidate,
            heading_rad=next_heading,
            speed_mps=next_speed,
            path_points=tuple(path_points),
        )

    def apply_motion(
        self,
        proposal: UGVMotionProposal,
        *,
        blocked: bool,
        simulation_time_s: float,
    ) -> None:
        """提交候选状态；被阻挡时保留位置并准备下一步转向。"""

        if proposal.agent_id != self.agent_id:
            raise ValueError("候选状态与无人车编号不一致")

        self.heading_rad = proposal.heading_rad
        if blocked:
            self.speed_mps = 0.0
            self.was_blocked = True
            self.avoidance_turn_remaining_rad = math.radians(
                UGV_AUTONOMOUS_TURN_ANGLE_DEG
            )
        else:
            self.position = proposal.position
            self.speed_mps = proposal.speed_mps
            self.was_blocked = False

        self._record_history(simulation_time_s)

    def update(
        self,
        delta_time_s: float,
        airport_map: AirportMap,
        *,
        acceleration_mps2: float = 0.0,
        turn_rate_rad_s: float = 0.0,
        allow_restricted: bool = False,
        simulation_time_s: float = 0.0,
        position_validator: Optional[PositionValidator] = None,
    ) -> None:
        """根据控制输入推进一个固定时间步，并阻止车辆穿过障碍。"""

        if delta_time_s == 0.0:
            return
        proposal = self.propose_motion(
            delta_time_s,
            airport_map,
            acceleration_mps2=acceleration_mps2,
            turn_rate_rad_s=turn_rate_rad_s,
            allow_restricted=allow_restricted,
        )
        blocked = proposal.map_blocked
        if not blocked and position_validator is not None:
            blocked = any(
                not position_validator(
                    self.agent_id,
                    point,
                    self.radius_m,
                )
                for point in proposal.path_points
            )
        self.apply_motion(
            proposal,
            blocked=blocked,
            simulation_time_s=simulation_time_s,
        )

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
