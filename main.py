"""机场蚁群仿真第二步：显示地图并驾驶单辆无人车。"""

import math
import sys

import pygame

from agents.manager import UGVManager
from agents.ugv import UGV
from airport_map import AirportMap, Passability
from config import (
    COLORS,
    FPS,
    INITIAL_UGV_COUNT,
    MAX_FRAME_TIME_S,
    SIMULATION_DT_S,
    UGV_MARKER_HALF_WIDTH_PX,
    UGV_MARKER_LENGTH_PX,
    UGV_MAX_ACCELERATION_MPS2,
    UGV_MAX_TURN_RATE_DEG_S,
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
    """根据当前窗口尺寸创建界面字体。"""

    shortest_side = min(size)
    map_size = max(12, min(22, round(shortest_side * 0.015)))
    status_size = max(13, min(24, map_size + 1))
    return create_font(map_size), create_font(status_size)


def draw_ugv(
    surface: pygame.Surface,
    airport_map: AirportMap,
    ugv: UGV,
    font: pygame.font.Font,
    selected: bool = False,
) -> None:
    """用带航向的三角形绘制无人车，高亮车辆额外显示感知范围。"""

    center = airport_map.world_to_screen_point(ugv.position)
    sensing_radius_px = max(
        1,
        round(ugv.sensing_range_m * airport_map.scale_x),
    )
    if selected:
        pygame.draw.circle(
            surface,
            COLORS["sensing_range"],
            center,
            sensing_radius_px,
            width=1,
        )

    direction_x, direction_y = ugv.heading_vector()
    side_x, side_y = -direction_y, direction_x
    # 所有车辆使用完全相同的屏幕标记尺寸；红色只表示当前被选中。
    tip = (
        center[0] + direction_x * UGV_MARKER_LENGTH_PX,
        center[1] + direction_y * UGV_MARKER_LENGTH_PX,
    )
    back_center = (
        center[0] - direction_x * UGV_MARKER_LENGTH_PX * 0.6,
        center[1] - direction_y * UGV_MARKER_LENGTH_PX * 0.6,
    )
    back_left = (
        back_center[0] + side_x * UGV_MARKER_HALF_WIDTH_PX,
        back_center[1] + side_y * UGV_MARKER_HALF_WIDTH_PX,
    )
    back_right = (
        back_center[0] - side_x * UGV_MARKER_HALF_WIDTH_PX,
        back_center[1] - side_y * UGV_MARKER_HALF_WIDTH_PX,
    )
    polygon = [tip, back_left, back_right]
    vehicle_color = COLORS["ugv"] if selected else COLORS["ugv_idle"]
    pygame.draw.polygon(surface, vehicle_color, polygon)
    if selected:
        pygame.draw.line(
            surface,
            COLORS["ugv_heading"],
            center,
            tip,
            width=2,
        )

    if selected:
        label = font.render(f"UGV-{ugv.agent_id}", True, COLORS["text"])
        label_rect = label.get_rect(
            midleft=(center[0] + max(8, sensing_radius_px) + 4, center[1])
        )
        surface.blit(label, label_rect)


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
    ugv_manager = UGVManager(airport_map)
    ugv_manager.deploy_in_staging(INITIAL_UGV_COUNT)
    controlled_agent_id = 0
    ugv = ugv_manager.get_agent(controlled_agent_id)

    map_font, status_font = create_interface_fonts(screen.get_size())

    running = True
    simulation_time_s = 0.0
    time_accumulator_s = 0.0

    while running:
        frame_time_s = min(clock.tick(FPS) / 1000.0, MAX_FRAME_TIME_S)
        time_accumulator_s += frame_time_s

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
                        "airport_sim_step2.png",
                    )
                    print("已保存截图：airport_sim_step2.png")

        keys = pygame.key.get_pressed()
        acceleration_mps2 = 0.0
        if keys[pygame.K_UP]:
            acceleration_mps2 += UGV_MAX_ACCELERATION_MPS2
        if keys[pygame.K_DOWN]:
            acceleration_mps2 -= UGV_MAX_ACCELERATION_MPS2

        turn_rate_rad_s = 0.0
        maximum_turn_rate = math.radians(UGV_MAX_TURN_RATE_DEG_S)
        if keys[pygame.K_LEFT]:
            turn_rate_rad_s -= maximum_turn_rate
        if keys[pygame.K_RIGHT]:
            turn_rate_rad_s += maximum_turn_rate

        while time_accumulator_s >= SIMULATION_DT_S:
            simulation_time_s += SIMULATION_DT_S
            ugv_manager.update_all(
                SIMULATION_DT_S,
                controlled_agent_id=controlled_agent_id,
                acceleration_mps2=acceleration_mps2,
                turn_rate_rad_s=turn_rate_rad_s,
                simulation_time_s=simulation_time_s,
            )
            time_accumulator_s -= SIMULATION_DT_S

        airport_map.draw(screen, map_font)
        for agent in ugv_manager.agents:
            if agent.agent_id != controlled_agent_id:
                draw_ugv(screen, airport_map, agent, map_font)
        draw_ugv(screen, airport_map, ugv, map_font, selected=True)

        mouse_screen = pygame.mouse.get_pos()
        mouse_world = airport_map.screen_to_world_point(mouse_screen)

        inside_map = (
            0.0 <= mouse_world[0] <= WORLD_WIDTH_M
            and 0.0 <= mouse_world[1] <= WORLD_HEIGHT_M
        )
        if inside_map:
            passability = airport_map.get_passability(mouse_world)
            if passability == Passability.DRIVABLE:
                area_text = "可通行"
            elif passability == Passability.RESTRICTED:
                area_text = "限制通行"
            else:
                area_text = "不可通行"
            coordinate_text = (
                f"世界坐标: ({mouse_world[0]:.1f} m, {mouse_world[1]:.1f} m) "
                f"| 区域: {area_text}"
            )
        else:
            coordinate_text = "鼠标位于机场地图之外"

        vehicle_text = (
            f"节点: {len(ugv_manager.agents)} | 控制UGV-{ugv.agent_id}: "
            f"({ugv.position[0]:.1f}, {ugv.position[1]:.1f}) m "
            f"| 速度: {ugv.speed_mps:.1f} m/s | 仿真时间: {simulation_time_s:.1f} s "
            f"| 碰撞阻挡: {ugv_manager.total_collision_blocks} "
            f"| ↑↓加减速  ←→转向  | S: 保存截图  ESC: 退出"
        )

        screen_width, screen_height = screen.get_size()
        margin = max(10, min(screen_width, screen_height) // 100)
        panel_padding = max(8, status_font.get_height() // 2)
        panel_height = status_font.get_height() * 2 + panel_padding * 3

        status_panel = pygame.Rect(
            margin,
            screen_height - margin - panel_height,
            screen_width - margin * 2,
            panel_height,
        )

        available_width = status_panel.width - panel_padding * 2
        if status_font.size(coordinate_text)[0] > available_width:
            if inside_map:
                coordinate_text = (
                    f"鼠标: ({mouse_world[0]:.0f}, {mouse_world[1]:.0f}) m "
                    f"| {area_text}"
                )
            else:
                coordinate_text = "鼠标位于地图外"
        if status_font.size(vehicle_text)[0] > available_width:
            vehicle_text = (
                f"{len(ugv_manager.agents)}辆 | UGV-{ugv.agent_id} "
                f"| {ugv.speed_mps:.1f} m/s "
                f"| 碰撞: {ugv_manager.total_collision_blocks} "
                f"| 方向键驾驶 | S: 保存 | ESC: 退出"
            )

        coordinate_surface = status_font.render(
            coordinate_text,
            True,
            COLORS["text"],
        )
        vehicle_surface = status_font.render(
            vehicle_text,
            True,
            COLORS["text"],
        )

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

        text_x = status_panel.x + panel_padding
        screen.blit(
            coordinate_surface,
            (text_x, status_panel.y + panel_padding),
        )
        screen.blit(
            vehicle_surface,
            (
                text_x,
                status_panel.y + panel_padding * 2 + status_font.get_height(),
            ),
        )

        pygame.display.flip()

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
