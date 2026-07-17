"""机场蚁群仿真第二步：20辆无人车同步基础自主移动。"""

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
    SIDEBAR_MAX_WIDTH_PX,
    SIDEBAR_MIN_WIDTH_PX,
    SIDEBAR_WIDTH_RATIO,
    SIMULATION_DT_S,
    UGV_AUTONOMOUS_ENABLED,
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


def calculate_interface_layout(
    size: tuple[int, int],
) -> tuple[pygame.Rect, pygame.Rect]:
    """把窗口拆成互不重叠的左侧状态栏和右侧地图视口。"""

    screen_width, screen_height = size
    reserved_width = max(
        SIDEBAR_MIN_WIDTH_PX,
        min(
            SIDEBAR_MAX_WIDTH_PX,
            round(screen_width * SIDEBAR_WIDTH_RATIO),
        ),
    )
    reserved_width = min(reserved_width, max(1, screen_width // 2))
    margin = max(10, min(screen_width, screen_height) // 100)
    sidebar_rect = pygame.Rect(
        margin,
        margin,
        max(1, reserved_width - margin * 2),
        max(1, screen_height - margin * 2),
    )
    map_viewport = pygame.Rect(
        reserved_width,
        0,
        max(1, screen_width - reserved_width),
        screen_height,
    )
    return sidebar_rect, map_viewport


def draw_ugv(
    surface: pygame.Surface,
    airport_map: AirportMap,
    ugv: UGV,
    font: pygame.font.Font,
    selected: bool = False,
) -> None:
    """用带航向的三角形绘制无人车，高亮车辆额外显示感知范围。"""

    center = airport_map.world_to_screen_point(ugv.position)
    communication_radius_px = max(
        1,
        round(ugv.communication_range_m * airport_map.scale_x),
    )
    sensing_radius_px = max(
        1,
        round(ugv.sensing_range_m * airport_map.scale_x),
    )
    if selected:
        pygame.draw.circle(
            surface,
            COLORS["communication_range"],
            center,
            communication_radius_px,
            width=1,
        )
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


def draw_neighbor_links(
    surface: pygame.Surface,
    airport_map: AirportMap,
    ugv: UGV,
) -> None:
    """只根据高亮车辆保存的局部观测绘制邻居连线。"""

    start = airport_map.world_to_screen_point(ugv.position)
    for observation in ugv.communication_neighbors.values():
        target_world = (
            ugv.position[0] + observation.relative_position[0],
            ugv.position[1] + observation.relative_position[1],
        )
        pygame.draw.line(
            surface,
            COLORS["communication_link"],
            start,
            airport_map.world_to_screen_point(target_world),
            width=1,
        )

    for observation in ugv.sensed_neighbors.values():
        target_world = (
            ugv.position[0] + observation.relative_position[0],
            ugv.position[1] + observation.relative_position[1],
        )
        pygame.draw.line(
            surface,
            COLORS["sensing_range"],
            start,
            airport_map.world_to_screen_point(target_world),
            width=2,
        )


def draw_sidebar(
    surface: pygame.Surface,
    sidebar_rect: pygame.Rect,
    font: pygame.font.Font,
    airport_map: AirportMap,
    ugv_manager: UGVManager,
    ugv: UGV,
    simulation_time_s: float,
    mode_text: str,
    mouse_world: tuple[float, float],
    inside_map: bool,
    area_text: str,
) -> None:
    """在左侧独立区域纵向绘制仿真状态，不覆盖机场地图。"""

    pygame.draw.rect(
        surface,
        COLORS["panel"],
        sidebar_rect,
        border_radius=5,
    )
    pygame.draw.rect(
        surface,
        COLORS["fence"],
        sidebar_rect,
        width=1,
        border_radius=5,
    )

    lines = [
        "仿真状态",
        f"模式: {mode_text}",
        f"节点/时间: {len(ugv_manager.agents)} / {simulation_time_s:.1f}s",
        "",
        f"高亮: UGV-{ugv.agent_id}",
        f"位置: {ugv.position[0]:.0f}, {ugv.position[1]:.0f}m",
        f"速度: {ugv.speed_mps:.1f} m/s",
        f"感知/通信范围: {ugv.sensing_range_m:.0f}/{ugv.communication_range_m:.0f}m",
        f"感知/通信邻居: {len(ugv.sensed_neighbors)}/"
        f"{len(ugv.communication_neighbors)}",
        f"车辆/地图阻挡: {ugv_manager.total_collision_blocks}/"
        f"{ugv_manager.total_map_blocks}",
        "",
    ]
    if inside_map:
        lines.extend(
            [
                f"鼠标: {mouse_world[0]:.0f}, {mouse_world[1]:.0f}m",
                f"区域: {area_text}",
            ]
        )
    else:
        lines.append("鼠标: 地图外")
    lines.extend(
        [
            "",
            "方向键: 临时接管",
            "S: 截图  ESC: 退出",
            "",
        ]
    )

    padding = max(8, font.get_height() // 2)
    line_gap = max(1, font.get_height() // 7)
    y = sidebar_rect.y + padding
    for line in lines:
        if not line:
            y += max(4, line_gap * 2)
            continue
        text_surface = font.render(line, True, COLORS["text"])
        if y + text_surface.get_height() > sidebar_rect.bottom - padding:
            break
        surface.blit(text_surface, (sidebar_rect.x + padding, y))
        y += text_surface.get_height() + line_gap

    legend_title = font.render("地图图例", True, COLORS["text"])
    if y + legend_title.get_height() <= sidebar_rect.bottom - padding:
        surface.blit(legend_title, (sidebar_rect.x + padding, y))
        y += legend_title.get_height() + line_gap

    swatch_width = max(14, font.get_height())
    swatch_height = max(9, font.get_height() - 3)
    for name, color in airport_map.legend_items():
        if y + font.get_height() > sidebar_rect.bottom - padding:
            break
        pygame.draw.rect(
            surface,
            color,
            pygame.Rect(
                sidebar_rect.x + padding,
                y + 1,
                swatch_width,
                swatch_height,
            ),
        )
        text_surface = font.render(name, True, COLORS["text"])
        surface.blit(
            text_surface,
            (sidebar_rect.x + padding + swatch_width + padding, y),
        )
        y += max(text_surface.get_height(), swatch_height) + line_gap


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
        manual_control_active = any(
            (
                keys[pygame.K_UP],
                keys[pygame.K_DOWN],
                keys[pygame.K_LEFT],
                keys[pygame.K_RIGHT],
            )
        )

        while time_accumulator_s >= SIMULATION_DT_S:
            simulation_time_s += SIMULATION_DT_S
            ugv_manager.update_all(
                SIMULATION_DT_S,
                controlled_agent_id=controlled_agent_id,
                acceleration_mps2=acceleration_mps2,
                turn_rate_rad_s=turn_rate_rad_s,
                simulation_time_s=simulation_time_s,
                autonomous=UGV_AUTONOMOUS_ENABLED,
                manual_control_active=manual_control_active,
            )
            time_accumulator_s -= SIMULATION_DT_S

        sidebar_rect, map_viewport = calculate_interface_layout(
            screen.get_size()
        )
        airport_map.draw(screen, map_font, map_viewport)
        draw_neighbor_links(screen, airport_map, ugv)
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
        else:
            area_text = "地图外"

        mode_text = "手动接管UGV-0" if manual_control_active else "20车自主移动"
        draw_sidebar(
            screen,
            sidebar_rect,
            status_font,
            airport_map,
            ugv_manager,
            ugv,
            simulation_time_s,
            mode_text,
            mouse_world,
            inside_map,
            area_text,
        )

        pygame.display.flip()

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
