"""机场二维地图及其可视化。"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import List, Optional, Tuple

import pygame

from config import (
    COLORS,
    OPEN_AREA_MOVEMENT_COST,
    PERIMETER_MARGIN_M,
    RESTRICTED_AREA_MOVEMENT_COST,
    ROAD_MOVEMENT_COST,
    WORLD_HEIGHT_M,
    WORLD_WIDTH_M,
)


class Passability(str, Enum):
    """地图位置的通行状态。"""

    DRIVABLE = "drivable"
    RESTRICTED = "restricted"
    BLOCKED = "blocked"


@dataclass(frozen=True)
class RectRegion:
    """机场中的矩形功能区域。"""

    name: str
    x: float
    y: float
    width: float
    height: float
    kind: str
    priority: int = 0
    passability: Passability = Passability.DRIVABLE
    movement_cost: float = ROAD_MOVEMENT_COST


class AirportMap:
    """典型机场区域的抽象二维模型。"""

    def __init__(self) -> None:
        self.scale_x = 1.0
        self.scale_y = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0

        # 主跑道
        self.runway = RectRegion(
            name="主跑道",
            x=250.0,
            y=1430.0,
            width=2500.0,
            height=100.0,
            kind="runway",
            passability=Passability.RESTRICTED,
            movement_cost=RESTRICTED_AREA_MOVEMENT_COST,
        )

        # 滑行道：一条平行滑行道和三条连接道
        self.taxiways: List[RectRegion] = [
            RectRegion(
                "平行滑行道",
                400.0,
                1190.0,
                2200.0,
                65.0,
                "taxiway",
            ),
            RectRegion(
                "滑行道 A",
                650.0,
                1190.0,
                65.0,
                340.0,
                "taxiway",
            ),
            RectRegion(
                "滑行道 B",
                1480.0,
                1190.0,
                65.0,
                340.0,
                "taxiway",
            ),
            RectRegion(
                "滑行道 C",
                2300.0,
                1190.0,
                65.0,
                340.0,
                "taxiway",
            ),
        ]

        # 停机坪
        self.aprons: List[RectRegion] = [
            RectRegion(
                "主停机坪",
                970.0,
                830.0,
                1100.0,
                330.0,
                "apron",
            )
        ]

        # 机库区
        self.hangars: List[RectRegion] = [
            RectRegion(
                "机库 1", 1020.0, 570.0, 260.0, 190.0, "hangar",
                passability=Passability.BLOCKED,
                movement_cost=math.inf,
            ),
            RectRegion(
                "机库 2", 1370.0, 570.0, 260.0, 190.0, "hangar",
                passability=Passability.BLOCKED,
                movement_cost=math.inf,
            ),
            RectRegion(
                "机库 3", 1720.0, 570.0, 260.0, 190.0, "hangar",
                passability=Passability.BLOCKED,
                movement_cost=math.inf,
            ),
        ]

        # 关键设施
        self.critical_regions: List[RectRegion] = [
            RectRegion(
                "塔台",
                540.0,
                680.0,
                140.0,
                140.0,
                "tower",
                priority=4,
                passability=Passability.BLOCKED,
                movement_cost=math.inf,
            ),
            RectRegion(
                "油料库",
                2260.0,
                610.0,
                330.0,
                270.0,
                "fuel",
                priority=5,
                passability=Passability.BLOCKED,
                movement_cost=math.inf,
            ),
            RectRegion(
                "机场大门",
                1350.0,
                2730.0,
                300.0,
                100.0,
                "gate",
                priority=3,
            ),
        ]

        # 周界巡逻道路形成闭合环路，为道路封闭后的绕行保留替代路径。
        self.perimeter_roads: List[RectRegion] = [
            RectRegion("North Perimeter Road", 180.0, 180.0, 2640.0, 60.0, "perimeter_road"),
            RectRegion("South Perimeter Road", 180.0, 2760.0, 2640.0, 60.0, "perimeter_road"),
            RectRegion("West Perimeter Road", 180.0, 180.0, 60.0, 2640.0, "perimeter_road"),
            RectRegion("East Perimeter Road", 2760.0, 180.0, 60.0, 2640.0, "perimeter_road"),
        ]

        # 机场内部道路：保障路、入口路、横向联络路和两侧响应道路。
        self.service_roads: List[RectRegion] = [
            RectRegion("North Service Road", 240.0, 480.0, 2520.0, 65.0, "road"),
            RectRegion("North Connector", 1468.0, 240.0, 65.0, 240.0, "road"),
            RectRegion("West Apron Access", 875.0, 480.0, 65.0, 710.0, "road"),
            RectRegion("East Apron Access", 2070.0, 480.0, 65.0, 710.0, "road"),
            RectRegion("Tower Access", 470.0, 480.0, 65.0, 270.0, "road"),
            RectRegion("Fuel Access", 2160.0, 480.0, 65.0, 330.0, "road"),
            RectRegion("Fuel Branch", 2160.0, 780.0, 100.0, 65.0, "road"),
            RectRegion("Main Access Road", 1480.0, 1530.0, 65.0, 1230.0, "road"),
            RectRegion("West Response Road", 520.0, 1530.0, 65.0, 650.0, "road"),
            RectRegion("East Response Road", 2415.0, 1530.0, 65.0, 650.0, "road"),
            RectRegion("South Connector", 240.0, 2180.0, 2520.0, 70.0, "road"),
            RectRegion("West Experiment Access", 585.0, 1855.0, 420.0, 55.0, "road"),
            RectRegion("East Experiment Access", 1995.0, 1855.0, 420.0, 55.0, "road"),
        ]

        # 南部开放区用于搜索、覆盖、临时障碍和通信黑区实验。
        self.experiment_areas: List[RectRegion] = [
            RectRegion(
                "西侧开放实验区", 310.0, 1670.0, 720.0, 440.0, "experiment",
                movement_cost=OPEN_AREA_MOVEMENT_COST,
            ),
            RectRegion(
                "东侧开放实验区", 1970.0, 1670.0, 720.0, 440.0, "experiment",
                movement_cost=OPEN_AREA_MOVEMENT_COST,
            ),
        ]

        self.staging_area = RectRegion(
            "无人车待命区", 1120.0, 2420.0, 760.0, 230.0, "staging"
        )

        # 建筑物本体不可通行，车辆只能到达其外部道路和停车位置。
        self.support_buildings: List[RectRegion] = [
            RectRegion(
                "充电站", 800.0, 2440.0, 250.0, 190.0, "charging",
                passability=Passability.BLOCKED,
                movement_cost=math.inf,
            ),
            RectRegion(
                "维修站", 1950.0, 2440.0, 300.0, 190.0, "maintenance",
                passability=Passability.BLOCKED,
                movement_cost=math.inf,
            ),
        ]

        self.blocked_regions: List[RectRegion] = [
            *self.hangars,
            *self.support_buildings,
            *[
                region
                for region in self.critical_regions
                if region.passability == Passability.BLOCKED
            ],
        ]
        self.preferred_drivable_regions: List[RectRegion] = [
            *self.perimeter_roads,
            *self.service_roads,
            *self.taxiways,
            *self.aprons,
            self.staging_area,
            *[
                region
                for region in self.critical_regions
                if region.kind == "gate"
            ],
        ]

        # 机场周界
        self.perimeter_margin_m = PERIMETER_MARGIN_M

    @staticmethod
    def _point_in_region(
        point: Tuple[float, float],
        region: RectRegion,
    ) -> bool:
        """判断一个点是否位于矩形区域内。"""

        x, y = point
        return (
            region.x <= x <= region.x + region.width
            and region.y <= y <= region.y + region.height
        )

    @staticmethod
    def _circle_intersects_region(
        point: Tuple[float, float],
        radius: float,
        region: RectRegion,
    ) -> bool:
        """判断以无人车为圆心的包围圆是否接触矩形区域。"""

        x, y = point
        nearest_x = max(region.x, min(x, region.x + region.width))
        nearest_y = max(region.y, min(y, region.y + region.height))
        distance_squared = (x - nearest_x) ** 2 + (y - nearest_y) ** 2
        return distance_squared <= radius ** 2

    def get_passability(
        self,
        point: Tuple[float, float],
        vehicle_radius: float = 0.0,
    ) -> Passability:
        """返回某位置对指定半径无人车的通行状态。"""

        if vehicle_radius < 0.0:
            raise ValueError("vehicle_radius不能为负数")

        x, y = point
        minimum = self.perimeter_margin_m + vehicle_radius
        maximum_x = WORLD_WIDTH_M - self.perimeter_margin_m - vehicle_radius
        maximum_y = WORLD_HEIGHT_M - self.perimeter_margin_m - vehicle_radius
        if not (minimum <= x <= maximum_x and minimum <= y <= maximum_y):
            return Passability.BLOCKED

        for region in self.blocked_regions:
            if self._circle_intersects_region(point, vehicle_radius, region):
                return Passability.BLOCKED

        if self._circle_intersects_region(point, vehicle_radius, self.runway):
            return Passability.RESTRICTED

        return Passability.DRIVABLE

    def is_position_drivable(
        self,
        point: Tuple[float, float],
        vehicle_radius: float = 0.0,
        allow_restricted: bool = False,
    ) -> bool:
        """判断无人车能否进入某位置。"""

        passability = self.get_passability(point, vehicle_radius)
        return passability == Passability.DRIVABLE or (
            allow_restricted and passability == Passability.RESTRICTED
        )

    def movement_cost_at(self, point: Tuple[float, float]) -> float:
        """返回某位置的基础移动代价，供后续路径选择使用。"""

        passability = self.get_passability(point)
        if passability == Passability.BLOCKED:
            return math.inf
        if passability == Passability.RESTRICTED:
            return RESTRICTED_AREA_MOVEMENT_COST

        for region in self.preferred_drivable_regions:
            if self._point_in_region(point, region):
                return ROAD_MOVEMENT_COST

        return OPEN_AREA_MOVEMENT_COST

    def update_viewport(
        self,
        width: int,
        height: int,
        origin_x: int = 0,
        origin_y: int = 0,
    ) -> None:
        """保持地图比例，并在指定屏幕区域内居中显示。"""

        scale = min(width / WORLD_WIDTH_M, height / WORLD_HEIGHT_M)
        self.scale_x = scale
        self.scale_y = scale
        self.offset_x = (
            origin_x + (width - WORLD_WIDTH_M * scale) / 2.0
        )
        self.offset_y = (
            origin_y + (height - WORLD_HEIGHT_M * scale) / 2.0
        )

    def world_to_screen_point(
        self,
        point: Tuple[float, float],
    ) -> Tuple[int, int]:
        """将世界坐标转换为屏幕坐标。"""

        x, y = point
        return (
            round(self.offset_x + x * self.scale_x),
            round(self.offset_y + y * self.scale_y),
        )

    def screen_to_world_point(
        self,
        point: Tuple[int, int],
    ) -> Tuple[float, float]:
        """将屏幕坐标转换为世界坐标。"""

        x, y = point
        return (
            (x - self.offset_x) / self.scale_x,
            (y - self.offset_y) / self.scale_y,
        )

    def region_to_screen_rect(
        self,
        region: RectRegion,
    ) -> pygame.Rect:
        """将物理区域转换为Pygame矩形。"""

        return pygame.Rect(
            round(self.offset_x + region.x * self.scale_x),
            round(self.offset_y + region.y * self.scale_y),
            max(1, round(region.width * self.scale_x)),
            max(1, round(region.height * self.scale_y)),
        )

    def draw_region(
        self,
        surface: pygame.Surface,
        region: RectRegion,
        color: Tuple[int, int, int],
        font: pygame.font.Font,
        show_label: bool = True,
        force_label: bool = False,
        show_outline: bool = True,
    ) -> None:
        """绘制矩形区域。"""

        rect = self.region_to_screen_rect(region)
        pygame.draw.rect(surface, color, rect)
        if show_outline:
            pygame.draw.rect(surface, COLORS["fence"], rect, width=1)

        label_fits = rect.width >= 65 and rect.height >= 25
        if show_label and (force_label or label_fits):
            label = font.render(region.name, True, COLORS["text"])
            label_rect = label.get_rect(center=rect.center)
            surface.blit(label, label_rect)

    def draw_world_label(
        self,
        surface: pygame.Surface,
        text: str,
        point: Tuple[float, float],
        font: pygame.font.Font,
    ) -> None:
        """在指定世界坐标绘制居中的地图文字。"""

        label = font.render(text, True, COLORS["text"])
        label_rect = label.get_rect(center=self.world_to_screen_point(point))
        surface.blit(label, label_rect)

    def draw_runway_markings(
        self,
        surface: pygame.Surface,
    ) -> None:
        """绘制跑道中线和端部标记。"""

        runway_rect = self.region_to_screen_rect(self.runway)
        center_y = runway_rect.centery

        dash_length_m = 80.0
        gap_length_m = 70.0
        x_m = self.runway.x + 100.0
        end_x_m = self.runway.x + self.runway.width - 100.0

        while x_m < end_x_m:
            start = self.world_to_screen_point((x_m, self.runway.y + 50.0))
            finish = self.world_to_screen_point(
                (
                    min(x_m + dash_length_m, end_x_m),
                    self.runway.y + 50.0,
                )
            )

            pygame.draw.line(
                surface,
                COLORS["runway_mark"],
                (start[0], center_y),
                (finish[0], center_y),
                width=2,
            )

            x_m += dash_length_m + gap_length_m

        # 跑道两端标记
        left_x = runway_rect.left + 20
        right_x = runway_rect.right - 20

        for offset in (-12, -4, 4, 12):
            pygame.draw.line(
                surface,
                COLORS["runway_mark"],
                (left_x, center_y + offset),
                (left_x + 25, center_y + offset),
                width=2,
            )
            pygame.draw.line(
                surface,
                COLORS["runway_mark"],
                (right_x - 25, center_y + offset),
                (right_x, center_y + offset),
                width=2,
            )

    def draw_perimeter(
        self,
        surface: pygame.Surface,
    ) -> None:
        """绘制机场周界围栏。"""

        margin = self.perimeter_margin_m
        right = WORLD_WIDTH_M - margin
        bottom = WORLD_HEIGHT_M - margin
        gate = next(
            region for region in self.critical_regions if region.kind == "gate"
        )
        line_width = max(2, round(self.scale_x * 8.0))

        segments = [
            ((margin, margin), (right, margin)),
            ((margin, margin), (margin, bottom)),
            ((right, margin), (right, bottom)),
            ((margin, bottom), (gate.x, bottom)),
            ((gate.x + gate.width, bottom), (right, bottom)),
        ]
        for start, finish in segments:
            pygame.draw.line(
                surface,
                COLORS["fence"],
                self.world_to_screen_point(start),
                self.world_to_screen_point(finish),
                width=line_width,
            )

    def draw_legend(
        self,
        surface: pygame.Surface,
        font: pygame.font.Font,
    ) -> None:
        """绘制地图图例。"""

        legend_items = self.legend_items()

        padding = max(8, font.get_height() // 2)
        row_height = max(18, font.get_height() + 4)
        swatch_width = max(16, font.get_height())
        text_width = max(font.size(name)[0] for name, _ in legend_items)
        panel_width = padding * 3 + swatch_width + text_width
        panel_height = padding * 2 + font.get_height() + row_height * len(legend_items)
        panel = pygame.Rect(
            round(self.offset_x) + 15,
            round(self.offset_y) + 15,
            panel_width,
            panel_height,
        )
        pygame.draw.rect(surface, COLORS["panel"], panel, border_radius=5)
        pygame.draw.rect(surface, COLORS["fence"], panel, width=1, border_radius=5)

        title = font.render("机场地图", True, COLORS["text"])
        surface.blit(title, (panel.x + padding, panel.y + padding))

        for index, (name, color) in enumerate(legend_items):
            y = panel.y + padding + font.get_height() + 6 + index * row_height
            swatch_height = max(10, font.get_height() - 3)
            pygame.draw.rect(
                surface,
                color,
                pygame.Rect(panel.x + padding, y, swatch_width, swatch_height),
            )
            text = font.render(name, True, COLORS["text"])
            surface.blit(text, (panel.x + padding * 2 + swatch_width, y - 2))

    @staticmethod
    def legend_items() -> Tuple[Tuple[str, Tuple[int, int, int]], ...]:
        """返回可供左侧栏或地图使用的统一图例内容。"""

        return (
            ("道路：可通行", COLORS["road"]),
            ("滑行道 / 停机坪", COLORS["taxiway"]),
            ("跑道：限制通行", COLORS["runway"]),
            ("开放实验区", COLORS["experiment"]),
            ("无人车待命区", COLORS["staging"]),
            ("建筑：不可通行", COLORS["building"]),
        )

    def draw(
        self,
        surface: pygame.Surface,
        font: pygame.font.Font,
        viewport: Optional[pygame.Rect] = None,
        show_legend: bool = False,
    ) -> None:
        """在完整窗口或指定视口内绘制机场地图。"""

        if viewport is None:
            viewport = surface.get_rect()
        self.update_viewport(
            viewport.width,
            viewport.height,
            viewport.x,
            viewport.y,
        )
        surface.fill(COLORS["background"])

        for area in self.experiment_areas:
            self.draw_region(
                surface,
                area,
                COLORS["experiment"],
                font,
                show_label=False,
            )

        for road in self.perimeter_roads:
            self.draw_region(
                surface,
                road,
                COLORS["perimeter_road"],
                font,
                show_label=False,
                show_outline=False,
            )

        for road in self.service_roads:
            self.draw_region(
                surface,
                road,
                COLORS["road"],
                font,
                show_label=False,
                show_outline=False,
            )

        for taxiway in self.taxiways:
            self.draw_region(
                surface,
                taxiway,
                COLORS["taxiway"],
                font,
                show_label=False,
            )

        for apron in self.aprons:
            self.draw_region(
                surface,
                apron,
                COLORS["apron"],
                font,
            )

        self.draw_region(
            surface,
            self.runway,
            COLORS["runway"],
            font,
            show_label=False,
        )
        self.draw_runway_markings(surface)

        self.draw_world_label(surface, "西侧开放实验区", (670.0, 1740.0), font)
        self.draw_world_label(surface, "东侧开放实验区", (2330.0, 1740.0), font)

        for hangar in self.hangars:
            self.draw_region(
                surface,
                hangar,
                COLORS["hangar"],
                font,
                force_label=True,
            )

        for region in self.critical_regions:
            self.draw_region(
                surface,
                region,
                COLORS[region.kind],
                font,
                force_label=True,
            )

        self.draw_region(
            surface,
            self.staging_area,
            COLORS["staging"],
            font,
            show_label=False,
        )
        self.draw_world_label(surface, "无人车待命区", (1500.0, 2380.0), font)

        for building in self.support_buildings:
            self.draw_region(
                surface,
                building,
                COLORS[building.kind],
                font,
                force_label=True,
            )

        self.draw_perimeter(surface)
        if show_legend:
            self.draw_legend(surface, font)
