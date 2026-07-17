"""机场蚁群仿真第二步：20辆无人车同步基础自主移动。"""

import math
import sys

import numpy as np
import pygame

from agents.manager import UGVManager
from agents.ugv import UGV
from airport_map import AirportMap, Passability
from config import (
    COLORS,
    COVERAGE_HEATMAP_COLOR,
    COVERAGE_HEATMAP_MAX_ALPHA,
    COVERAGE_HEATMAP_REFRESH_FPS,
    COVERAGE_HEATMAP_VISIBLE_DEFAULT,
    DEFAULT_SIMULATION_SPEED,
    FPS,
    FULL_RATE_RENDER_LOAD,
    INITIAL_UGV_COUNT,
    MAX_FRAME_TIME_S,
    MIN_RENDER_FPS,
    SIDEBAR_MAX_WIDTH_PX,
    SIDEBAR_MIN_WIDTH_PX,
    SIDEBAR_WIDTH_RATIO,
    SIMULATION_DT_S,
    SIMULATION_SPEED_OPTIONS,
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
from pheromone.field import CoverageField


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


def create_map_background(
    size: tuple[int, int],
    airport_map: AirportMap,
    map_font: pygame.font.Font,
    map_viewport: pygame.Rect,
) -> pygame.Surface:
    """创建只在窗口尺寸变化时重绘的静态机场底图。"""

    background = pygame.Surface(size)
    airport_map.draw(background, map_font, map_viewport)
    return background


def format_speed_multiplier(speed: float) -> str:
    """把倍速格式化为适合按钮显示的短文本。"""

    return f"{speed:g}×"


def change_simulation_speed(current_speed: float, direction: int) -> float:
    """按配置顺序切换倍速，并在最小、最大档位处停止。"""

    if direction not in (-1, 1):
        raise ValueError("direction必须是-1或1")
    if current_speed not in SIMULATION_SPEED_OPTIONS:
        raise ValueError("current_speed必须是已配置的仿真倍速")

    current_index = SIMULATION_SPEED_OPTIONS.index(current_speed)
    next_index = max(
        0,
        min(len(SIMULATION_SPEED_OPTIONS) - 1, current_index + direction),
    )
    return SIMULATION_SPEED_OPTIONS[next_index]


def scaled_frame_time(
    frame_time_s: float,
    simulation_speed: float,
    paused: bool,
) -> float:
    """将现实帧时间换算成待推进的仿真时间。"""

    if frame_time_s < 0.0:
        raise ValueError("frame_time_s不能为负数")
    if simulation_speed <= 0.0:
        raise ValueError("simulation_speed必须大于0")
    if paused:
        return 0.0
    return frame_time_s * simulation_speed


def calculate_render_fps(
    agent_count: int,
    simulation_speed: float,
    paused: bool,
) -> int:
    """根据节点规模和倍速降低绘制负载，但不减少仿真时间步。"""

    if agent_count <= 0:
        raise ValueError("agent_count必须大于0")
    if simulation_speed <= 0.0:
        raise ValueError("simulation_speed必须大于0")
    if paused:
        return FPS

    load = agent_count * simulation_speed
    if load <= FULL_RATE_RENDER_LOAD:
        return FPS
    return max(
        MIN_RENDER_FPS,
        min(FPS, round(FPS * FULL_RATE_RENDER_LOAD / load)),
    )


def calculate_speed_control_layout(
    sidebar_rect: pygame.Rect,
    font: pygame.font.Font,
) -> tuple[dict[float, pygame.Rect], pygame.Rect]:
    """计算左侧栏中的六个倍速按钮和暂停按钮位置。"""

    padding = max(8, font.get_height() // 2)
    line_gap = max(1, font.get_height() // 7)
    line_step = font.get_height() + line_gap
    label_y = sidebar_rect.y + padding + 3 * line_step

    pause_width = max(48, min(70, sidebar_rect.width // 3))
    pause_rect = pygame.Rect(
        sidebar_rect.right - padding - pause_width,
        label_y - 2,
        pause_width,
        font.get_height() + 5,
    )

    column_gap = 4
    available_width = max(3, sidebar_rect.width - 2 * padding)
    button_width = max(1, (available_width - 2 * column_gap) // 3)
    button_height = font.get_height() + 7
    start_y = label_y + font.get_height() + line_gap + 3
    button_rects: dict[float, pygame.Rect] = {}
    for index, speed in enumerate(SIMULATION_SPEED_OPTIONS):
        row, column = divmod(index, 3)
        button_rects[speed] = pygame.Rect(
            sidebar_rect.x + padding + column * (button_width + column_gap),
            start_y + row * (button_height + column_gap),
            button_width,
            button_height,
        )
    return button_rects, pause_rect


def calculate_heatmap_toggle_layout(
    sidebar_rect: pygame.Rect,
    font: pygame.font.Font,
) -> pygame.Rect:
    """计算位于倍速按钮下方的覆盖热力图开关位置。"""

    speed_button_rects, _ = calculate_speed_control_layout(
        sidebar_rect,
        font,
    )
    padding = max(8, font.get_height() // 2)
    gap = 4
    return pygame.Rect(
        sidebar_rect.x + padding,
        max(rect.bottom for rect in speed_button_rects.values()) + gap,
        max(1, sidebar_rect.width - 2 * padding),
        font.get_height() + 7,
    )


def draw_control_button(
    surface: pygame.Surface,
    rect: pygame.Rect,
    font: pygame.font.Font,
    text: str,
    active: bool,
) -> None:
    """绘制一个可点击的倍速或暂停按钮。"""

    fill_color = COLORS["button_active"] if active else COLORS["button"]
    text_color = (
        COLORS["button_active_text"] if active else COLORS["text"]
    )
    pygame.draw.rect(surface, fill_color, rect, border_radius=4)
    pygame.draw.rect(
        surface,
        COLORS["fence"],
        rect,
        width=1,
        border_radius=4,
    )
    text_surface = font.render(text, True, text_color)
    surface.blit(text_surface, text_surface.get_rect(center=rect.center))


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


def draw_coverage_heatmap(
    surface: pygame.Surface,
    airport_map: AirportMap,
    coverage_field: CoverageField,
) -> None:
    """把低分辨率覆盖网格缩放为半透明热力图。"""

    heatmap, position = create_coverage_heatmap_surface(
        airport_map,
        coverage_field,
    )
    surface.blit(heatmap, position)


def create_coverage_heatmap_surface(
    airport_map: AirportMap,
    coverage_field: CoverageField,
) -> tuple[pygame.Surface, tuple[int, int]]:
    """创建可复用的热力图图层，避免每个显示帧重复缩放。"""

    normalized = np.clip(
        coverage_field.values.T / coverage_field.maximum_value,
        0.0,
        1.0,
    )
    heatmap = pygame.Surface(
        (coverage_field.column_count, coverage_field.row_count),
        pygame.SRCALPHA,
    )
    rgb = pygame.surfarray.pixels3d(heatmap)
    alpha = pygame.surfarray.pixels_alpha(heatmap)
    rgb[:, :, 0] = COVERAGE_HEATMAP_COLOR[0]
    rgb[:, :, 1] = COVERAGE_HEATMAP_COLOR[1]
    rgb[:, :, 2] = COVERAGE_HEATMAP_COLOR[2]
    alpha[:, :] = (normalized * COVERAGE_HEATMAP_MAX_ALPHA).astype(
        np.uint8
    )
    del rgb
    del alpha

    top_left = airport_map.world_to_screen_point((0.0, 0.0))
    bottom_right = airport_map.world_to_screen_point(
        (
            coverage_field.world_width_m,
            coverage_field.world_height_m,
        )
    )
    display_size = (
        max(1, bottom_right[0] - top_left[0]),
        max(1, bottom_right[1] - top_left[1]),
    )
    scaled_heatmap = pygame.transform.scale(heatmap, display_size)
    return scaled_heatmap, top_left


def draw_sidebar(
    surface: pygame.Surface,
    sidebar_rect: pygame.Rect,
    font: pygame.font.Font,
    airport_map: AirportMap,
    ugv_manager: UGVManager,
    ugv: UGV,
    simulation_time_s: float,
    simulation_speed: float,
    simulation_paused: bool,
    show_coverage_heatmap: bool,
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

    padding = max(8, font.get_height() // 2)
    line_gap = max(1, font.get_height() // 7)
    line_step = font.get_height() + line_gap
    top_y = sidebar_rect.y + padding
    header_lines = [
        "仿真状态",
        f"模式: {mode_text}",
        f"节点/时间: {len(ugv_manager.agents)} / {simulation_time_s:.1f}s",
    ]
    for index, line in enumerate(header_lines):
        text_surface = font.render(line, True, COLORS["text"])
        surface.blit(text_surface, (sidebar_rect.x + padding, top_y + index * line_step))

    button_rects, pause_rect = calculate_speed_control_layout(
        sidebar_rect,
        font,
    )
    speed_label = font.render(
        f"仿真倍速: {format_speed_multiplier(simulation_speed)}",
        True,
        COLORS["text"],
    )
    surface.blit(speed_label, (sidebar_rect.x + padding, top_y + 3 * line_step))
    draw_control_button(
        surface,
        pause_rect,
        font,
        "继续" if simulation_paused else "暂停",
        simulation_paused,
    )
    for speed, button_rect in button_rects.items():
        draw_control_button(
            surface,
            button_rect,
            font,
            format_speed_multiplier(speed),
            speed == simulation_speed,
        )

    heatmap_toggle_rect = calculate_heatmap_toggle_layout(
        sidebar_rect,
        font,
    )
    draw_control_button(
        surface,
        heatmap_toggle_rect,
        font,
        f"覆盖热力图: {'开' if show_coverage_heatmap else '关'}",
        show_coverage_heatmap,
    )

    y = heatmap_toggle_rect.bottom + max(5, line_gap * 2)
    local_coverage = ugv.local_coverage
    lines = [
        f"高亮: UGV-{ugv.agent_id}",
        f"位置: {ugv.position[0]:.0f}, {ugv.position[1]:.0f}m",
        f"速度: {ugv.speed_mps:.1f} m/s",
        f"感知/通信范围: {ugv.sensing_range_m:.0f}/{ugv.communication_range_m:.0f}m",
        f"感知/通信邻居: {len(ugv.sensed_neighbors)}/"
        f"{len(ugv.communication_neighbors)}",
        f"车辆/地图阻挡: {ugv_manager.total_collision_blocks}/"
        f"{ugv_manager.total_map_blocks}",
    ]
    if local_coverage is not None:
        lines.extend(
            [
                f"覆盖 中/前: {local_coverage.center:.2f}/"
                f"{local_coverage.forward:.2f}",
                f"覆盖 左/右: {local_coverage.left:.2f}/"
                f"{local_coverage.right:.2f}",
            ]
        )
    lines.append("")
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
            "空格: 暂停  +/-: 倍速",
            f"H: 覆盖热图 {'开' if show_coverage_heatmap else '关'}",
            "S: 截图  ESC: 退出",
            "",
        ]
    )

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
    sidebar_rect, map_viewport = calculate_interface_layout(
        screen.get_size()
    )
    map_background = create_map_background(
        screen.get_size(),
        airport_map,
        map_font,
        map_viewport,
    )

    running = True
    simulation_time_s = 0.0
    time_accumulator_s = 0.0
    render_time_accumulator_s = 1.0 / FPS
    simulation_speed = DEFAULT_SIMULATION_SPEED
    simulation_paused = False
    show_coverage_heatmap = COVERAGE_HEATMAP_VISIBLE_DEFAULT
    coverage_heatmap_surface = None
    coverage_heatmap_position = (0, 0)
    coverage_heatmap_time_accumulator_s = (
        1.0 / COVERAGE_HEATMAP_REFRESH_FPS
    )

    while running:
        frame_time_s = min(clock.tick(FPS) / 1000.0, MAX_FRAME_TIME_S)

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
                sidebar_rect, map_viewport = calculate_interface_layout(
                    window_size
                )
                map_background = create_map_background(
                    window_size,
                    airport_map,
                    map_font,
                    map_viewport,
                )
                coverage_heatmap_surface = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                if event.key == pygame.K_SPACE:
                    simulation_paused = not simulation_paused

                if event.key == pygame.K_h:
                    show_coverage_heatmap = not show_coverage_heatmap
                    coverage_heatmap_surface = None

                if event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    simulation_speed = change_simulation_speed(
                        simulation_speed,
                        -1,
                    )

                if event.key in (
                    pygame.K_EQUALS,
                    pygame.K_PLUS,
                    pygame.K_KP_PLUS,
                ):
                    simulation_speed = change_simulation_speed(
                        simulation_speed,
                        1,
                    )

                if event.key == pygame.K_s:
                    pygame.image.save(
                        screen,
                        "airport_sim_step3.png",
                    )
                    print("已保存截图：airport_sim_step3.png")

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                sidebar_rect, _ = calculate_interface_layout(screen.get_size())
                speed_button_rects, pause_rect = calculate_speed_control_layout(
                    sidebar_rect,
                    status_font,
                )
                heatmap_toggle_rect = calculate_heatmap_toggle_layout(
                    sidebar_rect,
                    status_font,
                )
                if pause_rect.collidepoint(event.pos):
                    simulation_paused = not simulation_paused
                elif heatmap_toggle_rect.collidepoint(event.pos):
                    show_coverage_heatmap = not show_coverage_heatmap
                    coverage_heatmap_surface = None
                else:
                    for speed, button_rect in speed_button_rects.items():
                        if button_rect.collidepoint(event.pos):
                            simulation_speed = speed
                            simulation_paused = False
                            break

        time_accumulator_s += scaled_frame_time(
            frame_time_s,
            simulation_speed,
            simulation_paused,
        )
        render_time_accumulator_s += frame_time_s
        coverage_heatmap_time_accumulator_s += frame_time_s

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

        render_fps = calculate_render_fps(
            len(ugv_manager.agents),
            simulation_speed,
            simulation_paused,
        )
        render_interval_s = 1.0 / render_fps
        if render_time_accumulator_s + 1e-12 < render_interval_s:
            continue
        render_time_accumulator_s %= render_interval_s

        screen.blit(map_background, (0, 0))
        if show_coverage_heatmap:
            heatmap_refresh_interval_s = (
                1.0 / COVERAGE_HEATMAP_REFRESH_FPS
            )
            if (
                coverage_heatmap_surface is None
                or coverage_heatmap_time_accumulator_s + 1e-12
                >= heatmap_refresh_interval_s
            ):
                (
                    coverage_heatmap_surface,
                    coverage_heatmap_position,
                ) = create_coverage_heatmap_surface(
                    airport_map,
                    ugv_manager.coverage_field,
                )
                coverage_heatmap_time_accumulator_s %= (
                    heatmap_refresh_interval_s
                )
            screen.blit(
                coverage_heatmap_surface,
                coverage_heatmap_position,
            )
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

        if simulation_paused:
            mode_text = "已暂停"
        elif manual_control_active:
            mode_text = "手动接管UGV-0"
        else:
            mode_text = f"{len(ugv_manager.agents)}车自主移动"
        draw_sidebar(
            screen,
            sidebar_rect,
            status_font,
            airport_map,
            ugv_manager,
            ugv,
            simulation_time_s,
            simulation_speed,
            simulation_paused,
            show_coverage_heatmap,
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
