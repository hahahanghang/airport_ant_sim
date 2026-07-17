"""二维均匀空间哈希，用于快速查询局部候选对象。"""

from dataclasses import dataclass, field
import math
from typing import Callable, Dict, Generic, Iterable, List, Tuple, TypeVar


T = TypeVar("T")
Cell = Tuple[int, int]


@dataclass
class SpatialHashGrid(Generic[T]):
    """把连续坐标划分为固定尺寸网格，只返回附近候选对象。"""

    cell_size_m: float
    cells: Dict[Cell, List[T]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.cell_size_m <= 0.0:
            raise ValueError("cell_size_m必须大于0")

    def insert_point(self, item: T, point: Tuple[float, float]) -> None:
        """把点对象放入其所在的单个网格。"""

        self.cells.setdefault(self._point_cell(point), []).append(item)

    def insert_bounds(
        self,
        item: T,
        minimum_x: float,
        minimum_y: float,
        maximum_x: float,
        maximum_y: float,
    ) -> None:
        """把带包围盒的对象放入它覆盖的全部网格。"""

        if minimum_x > maximum_x or minimum_y > maximum_y:
            raise ValueError("包围盒最小坐标不能大于最大坐标")
        for cell in self._cells_for_bounds(
            minimum_x,
            minimum_y,
            maximum_x,
            maximum_y,
        ):
            self.cells.setdefault(cell, []).append(item)

    def nearby_items(
        self,
        point: Tuple[float, float],
        radius_m: float,
    ) -> Iterable[T]:
        """返回查询点半径外接方形覆盖网格中的候选对象。"""

        if radius_m < 0.0:
            raise ValueError("radius_m不能为负数")
        for cell in self._cells_for_bounds(
            point[0] - radius_m,
            point[1] - radius_m,
            point[0] + radius_m,
            point[1] + radius_m,
        ):
            yield from self.cells.get(cell, ())

    def candidate_pairs(
        self,
        key: Callable[[T], int],
    ) -> Tuple[Tuple[int, int], ...]:
        """返回至少共享一个网格的去重对象编号对。"""

        pairs = set()
        for items in self.cells.values():
            item_ids = sorted({key(item) for item in items})
            for index, first_id in enumerate(item_ids):
                for second_id in item_ids[index + 1:]:
                    pairs.add((first_id, second_id))
        return tuple(sorted(pairs))

    def _point_cell(self, point: Tuple[float, float]) -> Cell:
        return (
            math.floor(point[0] / self.cell_size_m),
            math.floor(point[1] / self.cell_size_m),
        )

    def _cells_for_bounds(
        self,
        minimum_x: float,
        minimum_y: float,
        maximum_x: float,
        maximum_y: float,
    ) -> Iterable[Cell]:
        minimum_cell = self._point_cell((minimum_x, minimum_y))
        maximum_cell = self._point_cell((maximum_x, maximum_y))
        for cell_y in range(minimum_cell[1], maximum_cell[1] + 1):
            for cell_x in range(minimum_cell[0], maximum_cell[0] + 1):
                yield (cell_x, cell_y)
