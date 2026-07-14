"""机场蚁群仿真第一步：显示机场基础地图。"""

import sys

import pygame

from airport_map import AirportMap
from config import (
    COLORS,
    FPS,
    WINDOW_MIN_HEIGHT_PX,
    WINDOW_MIN_WIDTH_PX,
    WINDOW_RESIZABLE,
    WINDOW_TITLE,
    WINDOW_USE_DESKTOP_SIZE,
    WORLD_HEIGHT_M,
    WORLD_WIDTH_M,
)


def create_font(size: int) -> pygame.font.Font:
    """优先使用Windows常见字体，失败时使用Pygame默认字体。"""

    font_path = r"C:\Windows\Fonts\msyh.ttc"

    try:
        return pygame.font.Font(font_path, size)
    except (FileNotFoundError, pygame.error):
        return pygame.font.Font(None, size)


def create_interface_fonts(size: tuple[int, int]) -> tuple[pygame.font.Font, pygame.font.Font]:
    """Create fonts proportional to the current window size."""

    shortest_side = min(size)
    map_size = max(12, min(22, round(shortest_side * 0.015)))
    status_size = max(13, min(24, map_size + 1))
    return create_font(map_size), create_font(status_size)


def main() -> None:
    pygame.init()

    display_flags = pygame.RESIZABLE if WINDOW_RESIZABLE else 0
    if WINDOW_USE_DESKTOP_SIZE:
        initial_window_size = pygame.display.get_desktop_sizes()[0]
    else:
        initial_window_size = (WINDOW_MIN_WIDTH_PX, WINDOW_MIN_HEIGHT_PX)

    screen = pygame.display.set_mode(initial_window_size, display_flags)
    pygame.display.set_caption(WINDOW_TITLE)

    clock = pygame.time.Clock()
    airport_map = AirportMap()

    map_font, status_font = create_interface_fonts(screen.get_size())

    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.VIDEORESIZE:
                window_size = (
                    max(WINDOW_MIN_WIDTH_PX, event.w),
                    max(WINDOW_MIN_HEIGHT_PX, event.h),
                )
                screen = pygame.display.set_mode(window_size, display_flags)
                map_font, status_font = create_interface_fonts(window_size)

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                if event.key == pygame.K_s:
                    pygame.image.save(
                        screen,
                        "airport_map_step1.png",
                    )
                    print("已保存截图：airport_map_step1.png")

        airport_map.draw(screen, map_font)

        mouse_screen = pygame.mouse.get_pos()
        mouse_world = airport_map.screen_to_world_point(mouse_screen)

        inside_map = (
            0.0 <= mouse_world[0] <= WORLD_WIDTH_M
            and 0.0 <= mouse_world[1] <= WORLD_HEIGHT_M
        )
        if inside_map:
            coordinate_text = (
                f"Mouse world coordinate: "
                f"({mouse_world[0]:.1f} m, {mouse_world[1]:.1f} m)"
            )
        else:
            coordinate_text = "Mouse is outside the airport map"

        status_text = f"{coordinate_text}  |  S: save screenshot  |  ESC: exit"

        screen_width, screen_height = screen.get_size()
        margin = max(10, min(screen_width, screen_height) // 100)
        panel_padding = max(8, status_font.get_height() // 2)
        panel_height = status_font.get_height() + panel_padding * 2

        status_panel = pygame.Rect(
            margin,
            screen_height - margin - panel_height,
            screen_width - margin * 2,
            panel_height,
        )

        if status_font.size(status_text)[0] > status_panel.width - panel_padding * 2:
            status_text = f"{coordinate_text}  |  S: save  |  ESC: exit"

        status_surface = status_font.render(status_text, True, COLORS["text"])

        pygame.draw.rect(
            screen,
            COLORS["panel"],
            status_panel,
            border_radius=4,
        )
        pygame.draw.rect(
            screen,
            COLORS["fence"],
            status_panel,
            width=1,
            border_radius=4,
        )

        status_rect = status_surface.get_rect(center=status_panel.center)
        screen.blit(status_surface, status_rect)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
