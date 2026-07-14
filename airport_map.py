"""机场二维地图及其可视化。"""

from dataclasses import dataclass
from typing import List, Tuple

import pygame

from config import (
    COLORS,
    WORLD_HEIGHT_M,
    WORLD_WIDTH_M,
)


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


class AirportMap:
    """典型机场区域的抽象二维模型。"""

    def __init__(self) -> None:
        self.scale_x = 1.0
        self.scale_y = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0

        # 主跑道
        self.runway = RectRegion(
            name="Main Runway",
            x=250.0,
            y=1430.0,
            width=2500.0,
            height=100.0,
            kind="runway",
        )

        # 滑行道：一条平行滑行道和三条连接道
        self.taxiways: List[RectRegion] = [
            RectRegion(
                "Parallel Taxiway",
                400.0,
                1190.0,
                2200.0,
                65.0,
                "taxiway",
            ),
            RectRegion(
                "Taxiway A",
                650.0,
                1190.0,
                65.0,
                340.0,
                "taxiway",
            ),
            RectRegion(
                "Taxiway B",
                1480.0,
                1190.0,
                65.0,
                340.0,
                "taxiway",
            ),
            RectRegion(
                "Taxiway C",
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
                "Main Apron",
                970.0,
                830.0,
                1100.0,
                330.0,
                "apron",
            )
        ]

        # 机库区
        self.hangars: List[RectRegion] = [
            RectRegion("Hangar 1", 1020.0, 570.0, 260.0, 190.0, "hangar"),
            RectRegion("Hangar 2", 1370.0, 570.0, 260.0, 190.0, "hangar"),
            RectRegion("Hangar 3", 1720.0, 570.0, 260.0, 190.0, "hangar"),
        ]

        # 关键设施
        self.critical_regions: List[RectRegion] = [
            RectRegion(
                "Control Tower",
                540.0,
                680.0,
                140.0,
                140.0,
                "tower",
                priority=4,
            ),
            RectRegion(
                "Fuel Depot",
                2260.0,
                610.0,
                330.0,
                270.0,
                "fuel",
                priority=5,
            ),
            RectRegion(
                "Main Gate",
                1350.0,
                2730.0,
                300.0,
                100.0,
                "gate",
                priority=3,
            ),
        ]

        # 机场周界
        self.perimeter_margin_m = 120.0

    def update_viewport(self, width: int, height: int) -> None:
        """Scale the world to fit the current window without distortion."""

        scale = min(width / WORLD_WIDTH_M, height / WORLD_HEIGHT_M)
        self.scale_x = scale
        self.scale_y = scale
        self.offset_x = (width - WORLD_WIDTH_M * scale) / 2.0
        self.offset_y = (height - WORLD_HEIGHT_M * scale) / 2.0

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
    ) -> None:
        """绘制矩形区域。"""

        rect = self.region_to_screen_rect(region)
        pygame.draw.rect(surface, color, rect)
        pygame.draw.rect(surface, COLORS["fence"], rect, width=1)

        if show_label and rect.width >= 65 and rect.height >= 25:
            label = font.render(region.name, True, COLORS["text"])
            label_rect = label.get_rect(center=rect.center)
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

        x = self.perimeter_margin_m
        y = self.perimeter_margin_m
        width = WORLD_WIDTH_M - 2 * self.perimeter_margin_m
        height = WORLD_HEIGHT_M - 2 * self.perimeter_margin_m

        rect = pygame.Rect(
            round(self.offset_x + x * self.scale_x),
            round(self.offset_y + y * self.scale_y),
            round(width * self.scale_x),
            round(height * self.scale_y),
        )

        pygame.draw.rect(
            surface,
            COLORS["fence"],
            rect,
            width=3,
        )

    def draw_legend(
        self,
        surface: pygame.Surface,
        font: pygame.font.Font,
    ) -> None:
        """绘制地图图例。"""

        legend_items = [
            ("Runway", COLORS["runway"]),
            ("Taxiway", COLORS["taxiway"]),
            ("Apron", COLORS["apron"]),
            ("Hangar", COLORS["hangar"]),
            ("Critical facility", COLORS["fuel"]),
        ]

        padding = max(8, font.get_height() // 2)
        row_height = max(18, font.get_height() + 4)
        swatch_width = max(16, font.get_height())
        text_width = max(font.size(name)[0] for name, _ in legend_items)
        panel_width = padding * 3 + swatch_width + text_width
        panel_height = padding * 2 + font.get_height() + row_height * len(legend_items)
        panel = pygame.Rect(15, 15, panel_width, panel_height)
        pygame.draw.rect(surface, COLORS["panel"], panel, border_radius=5)
        pygame.draw.rect(surface, COLORS["fence"], panel, width=1, border_radius=5)

        title = font.render("Airport map", True, COLORS["text"])
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

    def draw(
        self,
        surface: pygame.Surface,
        font: pygame.font.Font,
    ) -> None:
        """绘制完整机场地图。"""

        self.update_viewport(*surface.get_size())
        surface.fill(COLORS["background"])

        self.draw_perimeter(surface)

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

        for hangar in self.hangars:
            self.draw_region(
                surface,
                hangar,
                COLORS["hangar"],
                font,
            )

        for region in self.critical_regions:
            self.draw_region(
                surface,
                region,
                COLORS[region.kind],
                font,
            )

        self.draw_legend(surface, font)
