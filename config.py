"""机场蚁群仿真的全局配置。"""

# 世界尺寸，单位：米
WORLD_WIDTH_M = 3000.0
WORLD_HEIGHT_M = 3000.0
PERIMETER_MARGIN_M = 120.0

# Pygame窗口自适应配置，尺寸单位：像素
# 默认按主显示器的桌面分辨率启动，并允许用户拖动调整窗口大小。
WINDOW_USE_DESKTOP_SIZE = True
WINDOW_RESIZABLE = True
WINDOW_MIN_WIDTH_PX = 640
WINDOW_MIN_HEIGHT_PX = 480

FPS = 60
WINDOW_TITLE = "Airport Ant Swarm Simulation - Step 2"

# 仿真时间与随机性配置
SIMULATION_DT_S = 0.1
MAX_FRAME_TIME_S = 0.25
RANDOM_SEED = 42
INITIAL_UGV_COUNT = 20

# 初期无人车运动学参数
UGV_RADIUS_M = 1.0
UGV_MAX_SPEED_MPS = 8.0
UGV_MAX_ACCELERATION_MPS2 = 2.5
UGV_MAX_TURN_RATE_DEG_S = 45.0
UGV_SENSING_RANGE_M = 60.0
UGV_COMMUNICATION_RANGE_M = 120.0
UGV_LOCAL_HISTORY_LENGTH = 50
UGV_INITIAL_HEADING_DEG = -90.0
UGV_DEPLOYMENT_SPACING_M = 22.0
UGV_COLLISION_CLEARANCE_M = 1.0

# 车辆的物理半径只有1米，在整幅3000米地图上不足1像素。
# 因此使用统一的屏幕标记尺寸保证可见性，标记大小不参与碰撞计算。
UGV_MARKER_LENGTH_PX = 8
UGV_MARKER_HALF_WIDTH_PX = 4

# 地图颜色
COLORS = {
    "background": (232, 238, 226),
    "runway": (80, 84, 88),
    "taxiway": (126, 130, 134),
    "apron": (166, 170, 174),
    "road": (151, 139, 122),
    "perimeter_road": (117, 126, 105),
    "experiment": (211, 220, 193),
    "staging": (104, 158, 166),
    "building": (116, 126, 138),
    "hangar": (98, 115, 132),
    "tower": (191, 137, 78),
    "fuel": (184, 93, 73),
    "charging": (94, 151, 112),
    "maintenance": (126, 112, 151),
    "ugv": (222, 76, 54),
    "ugv_idle": (45, 104, 120),
    "ugv_heading": (255, 246, 214),
    "sensing_range": (208, 96, 75),
    "gate": (71, 130, 145),
    "fence": (45, 59, 72),
    "runway_mark": (240, 240, 235),
    "text": (30, 38, 46),
    "panel": (245, 247, 249),
}

# 不同地面的相对移动代价。数值越小，后续路径选择越倾向该区域。
ROAD_MOVEMENT_COST = 1.0
OPEN_AREA_MOVEMENT_COST = 1.5
RESTRICTED_AREA_MOVEMENT_COST = 2.0
