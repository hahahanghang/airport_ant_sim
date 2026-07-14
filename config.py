"""机场蚁群仿真的全局配置。"""

# 世界尺寸，单位：米
WORLD_WIDTH_M = 3000.0
WORLD_HEIGHT_M = 3000.0

# Pygame窗口自适应配置，尺寸单位：像素
# 默认按主显示器的桌面分辨率启动，并允许用户拖动调整窗口大小。
WINDOW_USE_DESKTOP_SIZE = True
WINDOW_RESIZABLE = True
WINDOW_MIN_WIDTH_PX = 640
WINDOW_MIN_HEIGHT_PX = 480

FPS = 60
WINDOW_TITLE = "Airport Ant Swarm Simulation - Step 1"

# 地图颜色
COLORS = {
    "background": (232, 238, 226),
    "runway": (80, 84, 88),
    "taxiway": (126, 130, 134),
    "apron": (166, 170, 174),
    "building": (116, 126, 138),
    "hangar": (98, 115, 132),
    "tower": (191, 137, 78),
    "fuel": (184, 93, 73),
    "gate": (71, 130, 145),
    "fence": (45, 59, 72),
    "runway_mark": (240, 240, 235),
    "text": (30, 38, 46),
    "panel": (245, 247, 249),
}
