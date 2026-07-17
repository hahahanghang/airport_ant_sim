"""覆盖信息素网格、沉积、挥发与有限局部观察。"""

from dataclasses import dataclass, field
import math
from typing import Sequence, Tuple

import numpy as np

from config import (
    COVERAGE_CELL_SIZE_M,
    COVERAGE_DEPOSIT_RATE_PER_S,
    COVERAGE_EVAPORATION_RATE_PER_S,
    COVERAGE_MAX_VALUE,
    COVERAGE_SAMPLE_DISTANCE_M,
    WORLD_HEIGHT_M,
    WORLD_WIDTH_M,
)


@dataclass(frozen=True)
class LocalCoverageObservation:
    """单辆车可读取的四方向覆盖信息，不包含完整信息素矩阵。"""

    center: float
    forward: float
    left: float
    right: float
    sample_distance_m: float


@dataclass
class CoverageField:
    """使用NumPy网格保存机场区域的覆盖访问痕迹。"""

    world_width_m: float = WORLD_WIDTH_M
    world_height_m: float = WORLD_HEIGHT_M
    cell_size_m: float = COVERAGE_CELL_SIZE_M
    deposit_rate_per_s: float = COVERAGE_DEPOSIT_RATE_PER_S
    evaporation_rate_per_s: float = COVERAGE_EVAPORATION_RATE_PER_S
    maximum_value: float = COVERAGE_MAX_VALUE
    values: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if self.world_width_m <= 0.0 or self.world_height_m <= 0.0:
            raise ValueError("信息素场世界尺寸必须大于0")
        if self.cell_size_m <= 0.0:
            raise ValueError("cell_size_m必须大于0")
        if self.deposit_rate_per_s < 0.0:
            raise ValueError("deposit_rate_per_s不能为负数")
        if self.evaporation_rate_per_s < 0.0:
            raise ValueError("evaporation_rate_per_s不能为负数")
        if self.maximum_value <= 0.0:
            raise ValueError("maximum_value必须大于0")

        row_count = math.ceil(self.world_height_m / self.cell_size_m)
        column_count = math.ceil(self.world_width_m / self.cell_size_m)
        self.values = np.zeros(
            (row_count, column_count),
            dtype=np.float32,
        )

    @property
    def row_count(self) -> int:
        return int(self.values.shape[0])

    @property
    def column_count(self) -> int:
        return int(self.values.shape[1])

    def reset(self) -> None:
        """清空全部覆盖痕迹，用于开始一次新的可重复实验。"""

        self.values.fill(0.0)

    def world_to_cell(self, point: Tuple[float, float]) -> Tuple[int, int]:
        """把世界坐标转换为(row, column)网格索引。"""

        x, y = point
        if not (0.0 <= x <= self.world_width_m):
            raise ValueError("x坐标超出覆盖信息素场")
        if not (0.0 <= y <= self.world_height_m):
            raise ValueError("y坐标超出覆盖信息素场")
        column = min(int(x / self.cell_size_m), self.column_count - 1)
        row = min(int(y / self.cell_size_m), self.row_count - 1)
        return row, column

    def sample(self, point: Tuple[float, float]) -> float:
        """读取一个世界位置所在网格的覆盖值。"""

        row, column = self.world_to_cell(point)
        return float(self.values[row, column])

    def deposit_many(
        self,
        positions: Sequence[Tuple[float, float]],
        amount: float,
    ) -> None:
        """在多个车辆位置沉积信息素，并限制到统一最大值。"""

        if amount < 0.0:
            raise ValueError("amount不能为负数")
        if amount == 0.0 or len(positions) == 0:
            return
        points = np.asarray(positions, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("positions必须是二维坐标序列")
        if np.any(points[:, 0] < 0.0) or np.any(
            points[:, 0] > self.world_width_m
        ):
            raise ValueError("x坐标超出覆盖信息素场")
        if np.any(points[:, 1] < 0.0) or np.any(
            points[:, 1] > self.world_height_m
        ):
            raise ValueError("y坐标超出覆盖信息素场")
        columns = np.minimum(
            (points[:, 0] / self.cell_size_m).astype(np.intp),
            self.column_count - 1,
        )
        rows = np.minimum(
            (points[:, 1] / self.cell_size_m).astype(np.intp),
            self.row_count - 1,
        )
        np.add.at(self.values, (rows, columns), amount)
        np.minimum(self.values, self.maximum_value, out=self.values)

    def evaporate(self, delta_time_s: float) -> None:
        """按指数规律批量挥发，结果不依赖显示帧率。"""

        if delta_time_s < 0.0:
            raise ValueError("delta_time_s不能为负数")
        if delta_time_s == 0.0 or self.evaporation_rate_per_s == 0.0:
            return
        retention = math.exp(-self.evaporation_rate_per_s * delta_time_s)
        self.values *= retention

    def update(
        self,
        delta_time_s: float,
        positions: Sequence[Tuple[float, float]],
    ) -> None:
        """先挥发旧痕迹，再按经过当前位置的时间沉积新痕迹。"""

        if delta_time_s < 0.0:
            raise ValueError("delta_time_s不能为负数")
        self.evaporate(delta_time_s)
        self.deposit_many(
            positions,
            self.deposit_rate_per_s * delta_time_s,
        )

    def observe_local(
        self,
        position: Tuple[float, float],
        heading_rad: float,
        sample_distance_m: float = COVERAGE_SAMPLE_DISTANCE_M,
    ) -> LocalCoverageObservation:
        """只返回本车中心及三个相对方向的有限覆盖采样。"""

        return self.observe_many(
            [position],
            [heading_rad],
            sample_distance_m,
        )[0]

    def observe_many(
        self,
        positions: Sequence[Tuple[float, float]],
        headings_rad: Sequence[float],
        sample_distance_m: float = COVERAGE_SAMPLE_DISTANCE_M,
    ) -> Tuple[LocalCoverageObservation, ...]:
        """批量计算多辆车的四方向采样，返回结果仍逐车隔离。"""

        if sample_distance_m <= 0.0:
            raise ValueError("sample_distance_m必须大于0")
        if len(positions) != len(headings_rad):
            raise ValueError("positions与headings_rad长度必须相同")
        if len(positions) == 0:
            return ()

        points = np.asarray(positions, dtype=np.float64)
        headings = np.asarray(headings_rad, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("positions必须是二维坐标序列")
        if (
            np.any(points[:, 0] < 0.0)
            or np.any(points[:, 0] > self.world_width_m)
            or np.any(points[:, 1] < 0.0)
            or np.any(points[:, 1] > self.world_height_m)
        ):
            raise ValueError("局部覆盖采样位置超出信息素场")

        def sample_offsets(angle_offsets: np.ndarray) -> np.ndarray:
            sample_x = np.clip(
                points[:, 0]
                + np.cos(headings + angle_offsets) * sample_distance_m,
                0.0,
                self.world_width_m,
            )
            sample_y = np.clip(
                points[:, 1]
                + np.sin(headings + angle_offsets) * sample_distance_m,
                0.0,
                self.world_height_m,
            )
            columns = np.minimum(
                (sample_x / self.cell_size_m).astype(np.intp),
                self.column_count - 1,
            )
            rows = np.minimum(
                (sample_y / self.cell_size_m).astype(np.intp),
                self.row_count - 1,
            )
            return self.values[rows, columns]

        zero_offsets = np.zeros_like(headings)
        center_columns = np.minimum(
            (points[:, 0] / self.cell_size_m).astype(np.intp),
            self.column_count - 1,
        )
        center_rows = np.minimum(
            (points[:, 1] / self.cell_size_m).astype(np.intp),
            self.row_count - 1,
        )
        center_values = self.values[center_rows, center_columns]
        forward_values = sample_offsets(zero_offsets)
        left_values = sample_offsets(
            np.full_like(headings, -math.pi / 2.0)
        )
        right_values = sample_offsets(
            np.full_like(headings, math.pi / 2.0)
        )
        return tuple(
            LocalCoverageObservation(
                center=float(center_values[index]),
                forward=float(forward_values[index]),
                left=float(left_values[index]),
                right=float(right_values[index]),
                sample_distance_m=sample_distance_m,
            )
            for index in range(len(positions))
        )
