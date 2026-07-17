"""基于局部范围的感知邻居和通信邻居搜索。"""

from dataclasses import dataclass
import math
from typing import Dict, Protocol, Sequence, Tuple

from config import NEIGHBOR_GRID_CELL_SIZE_M
from core.spatial_hash import SpatialHashGrid


class NeighborSearchAgent(Protocol):
    """邻居搜索所需的最小车辆只读接口。"""

    agent_id: int
    position: Tuple[float, float]
    sensing_range_m: float
    communication_range_m: float


@dataclass(frozen=True)
class NeighborObservation:
    """一辆车对单个邻居持有的局部相对观测。"""

    agent_id: int
    relative_position: Tuple[float, float]
    distance_m: float


@dataclass(frozen=True)
class LocalNeighborhood:
    """单辆车当前时间步可获得的两类局部邻居。"""

    sensed: Tuple[NeighborObservation, ...] = ()
    communicable: Tuple[NeighborObservation, ...] = ()


class NeighborSearch(Protocol):
    """可替换的邻居搜索接口，后续可接入空间网格实现。"""

    def search(
        self,
        agents: Sequence[NeighborSearchAgent],
    ) -> Dict[int, LocalNeighborhood]:
        """为每辆车生成独立的局部邻居结果。"""


class BruteForceNeighborSearch:
    """两两检查车辆距离的基础实现，适用于当前20至200节点。"""

    def search(
        self,
        agents: Sequence[NeighborSearchAgent],
    ) -> Dict[int, LocalNeighborhood]:
        """使用各车自身配置范围，分别计算感知和通信邻居。"""

        sensed: Dict[int, list[NeighborObservation]] = {}
        communicable: Dict[int, list[NeighborObservation]] = {}
        for agent in agents:
            if agent.agent_id in sensed:
                raise ValueError(f"发现重复节点编号{agent.agent_id}")
            if agent.sensing_range_m <= 0.0:
                raise ValueError("sensing_range_m必须大于0")
            if agent.communication_range_m <= 0.0:
                raise ValueError("communication_range_m必须大于0")
            sensed[agent.agent_id] = []
            communicable[agent.agent_id] = []

        for index, first in enumerate(agents):
            for second in agents[index + 1:]:
                relative_from_first = (
                    second.position[0] - first.position[0],
                    second.position[1] - first.position[1],
                )
                distance_m = math.hypot(*relative_from_first)
                first_is_sensed = distance_m <= first.sensing_range_m
                first_can_communicate = (
                    distance_m <= first.communication_range_m
                )
                second_is_sensed = distance_m <= second.sensing_range_m
                second_can_communicate = (
                    distance_m <= second.communication_range_m
                )

                if first_is_sensed or first_can_communicate:
                    first_observation = NeighborObservation(
                        agent_id=second.agent_id,
                        relative_position=relative_from_first,
                        distance_m=distance_m,
                    )
                    if first_is_sensed:
                        sensed[first.agent_id].append(first_observation)
                    if first_can_communicate:
                        communicable[first.agent_id].append(first_observation)
                if second_is_sensed or second_can_communicate:
                    second_observation = NeighborObservation(
                        agent_id=first.agent_id,
                        relative_position=(
                            -relative_from_first[0],
                            -relative_from_first[1],
                        ),
                        distance_m=distance_m,
                    )
                    if second_is_sensed:
                        sensed[second.agent_id].append(second_observation)
                    if second_can_communicate:
                        communicable[second.agent_id].append(second_observation)

        return {
            agent_id: LocalNeighborhood(
                sensed=tuple(
                    sorted(
                        sensed[agent_id],
                        key=lambda observation: observation.agent_id,
                    )
                ),
                communicable=tuple(
                    sorted(
                        communicable[agent_id],
                        key=lambda observation: observation.agent_id,
                    )
                ),
            )
            for agent_id in sorted(sensed)
        }


class SpatialHashNeighborSearch:
    """使用局部空间网格查询邻居，节点不遍历全部其他节点。"""

    def __init__(self, cell_size_m: float = NEIGHBOR_GRID_CELL_SIZE_M) -> None:
        if cell_size_m <= 0.0:
            raise ValueError("cell_size_m必须大于0")
        self.cell_size_m = cell_size_m

    def search(
        self,
        agents: Sequence[NeighborSearchAgent],
    ) -> Dict[int, LocalNeighborhood]:
        """按每辆车自己的范围查询附近网格并生成局部相对观察。"""

        agents_by_id: Dict[int, NeighborSearchAgent] = {}
        sensed: Dict[int, list[NeighborObservation]] = {}
        communicable: Dict[int, list[NeighborObservation]] = {}
        sensing_squared_by_id: Dict[int, float] = {}
        communication_squared_by_id: Dict[int, float] = {}
        grid: SpatialHashGrid[NeighborSearchAgent] = SpatialHashGrid(
            self.cell_size_m
        )
        maximum_query_radius = 0.0
        for agent in agents:
            if agent.agent_id in agents_by_id:
                raise ValueError(f"发现重复节点编号{agent.agent_id}")
            if agent.sensing_range_m <= 0.0:
                raise ValueError("sensing_range_m必须大于0")
            if agent.communication_range_m <= 0.0:
                raise ValueError("communication_range_m必须大于0")
            agents_by_id[agent.agent_id] = agent
            sensed[agent.agent_id] = []
            communicable[agent.agent_id] = []
            sensing_squared_by_id[agent.agent_id] = (
                agent.sensing_range_m * agent.sensing_range_m
            )
            communication_squared_by_id[agent.agent_id] = (
                agent.communication_range_m
                * agent.communication_range_m
            )
            grid.insert_point(agent, agent.position)
            maximum_query_radius = max(
                maximum_query_radius,
                agent.sensing_range_m,
                agent.communication_range_m,
            )

        for agent_id in sorted(agents_by_id):
            agent = agents_by_id[agent_id]
            agent_x, agent_y = agent.position
            agent_sensing_squared = sensing_squared_by_id[agent_id]
            agent_communication_squared = (
                communication_squared_by_id[agent_id]
            )
            agent_sensed = sensed[agent_id]
            agent_communicable = communicable[agent_id]
            for other in grid.nearby_items(
                agent.position,
                maximum_query_radius,
            ):
                # 每一对节点只由编号较小的一方计算一次，再把相反方向的
                # 相对观察分别写入两辆车的本地结果。
                if other.agent_id <= agent_id:
                    continue
                relative_position = (
                    other.position[0] - agent_x,
                    other.position[1] - agent_y,
                )
                distance_squared = (
                    relative_position[0] ** 2
                    + relative_position[1] ** 2
                )
                first_is_sensed = (
                    distance_squared <= agent_sensing_squared
                )
                first_can_communicate = (
                    distance_squared <= agent_communication_squared
                )
                second_is_sensed = (
                    distance_squared
                    <= sensing_squared_by_id[other.agent_id]
                )
                second_can_communicate = (
                    distance_squared
                    <= communication_squared_by_id[other.agent_id]
                )
                if not (
                    first_is_sensed
                    or first_can_communicate
                    or second_is_sensed
                    or second_can_communicate
                ):
                    continue

                distance_m = math.hypot(*relative_position)
                first_observation = NeighborObservation(
                    agent_id=other.agent_id,
                    relative_position=relative_position,
                    distance_m=distance_m,
                )
                second_observation = NeighborObservation(
                    agent_id=agent.agent_id,
                    relative_position=(
                        -relative_position[0],
                        -relative_position[1],
                    ),
                    distance_m=distance_m,
                )
                if first_is_sensed:
                    agent_sensed.append(first_observation)
                if first_can_communicate:
                    agent_communicable.append(first_observation)
                if second_is_sensed:
                    sensed[other.agent_id].append(second_observation)
                if second_can_communicate:
                    communicable[other.agent_id].append(second_observation)

        return {
            agent_id: LocalNeighborhood(
                sensed=tuple(
                    sorted(
                        sensed[agent_id],
                        key=lambda observation: observation.agent_id,
                    )
                ),
                communicable=tuple(
                    sorted(
                        communicable[agent_id],
                        key=lambda observation: observation.agent_id,
                    )
                ),
            )
            for agent_id in sorted(agents_by_id)
        }
