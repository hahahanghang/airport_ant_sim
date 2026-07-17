"""机场蚁群仿真的全局配置。"""

# 世界尺寸，单位：米
WORLD_WIDTH_M = 3000.0
WORLD_HEIGHT_M = 3000.0
PERIMETER_MARGIN_M = 120.0
MAP_COLLISION_GRID_CELL_SIZE_M = 300.0

# Pygame窗口自适应配置，尺寸单位：像素
# 默认按主显示器的桌面分辨率启动，并允许用户拖动调整窗口大小。
WINDOW_USE_DESKTOP_SIZE = True
WINDOW_RESIZABLE = True
WINDOW_MIN_WIDTH_PX = 640
WINDOW_MIN_HEIGHT_PX = 480

# 左侧状态栏独占窗口宽度，机场地图只在右侧剩余区域绘制。
SIDEBAR_WIDTH_RATIO = 0.20
SIDEBAR_MIN_WIDTH_PX = 200
SIDEBAR_MAX_WIDTH_PX = 340

FPS = 60
MIN_RENDER_FPS = 15
# 节点数×倍速不超过此负载时保持60帧；超过后只降低绘制频率，
# 不减少任何固定仿真时间步。
FULL_RATE_RENDER_LOAD = 1000.0
WINDOW_TITLE = "Airport Ant Swarm Simulation - Step 3: Coverage Pheromone"

# 仿真时间与随机性配置
SIMULATION_DT_S = 0.1
MAX_FRAME_TIME_S = 0.25
SIMULATION_SPEED_OPTIONS = (0.5, 1.0, 2.0, 5.0, 10.0, 20.0)
DEFAULT_SIMULATION_SPEED = 1.0
RANDOM_SEED = 42
INITIAL_UGV_COUNT = 20

# 空间网格与局部邻居更新参数。
# 邻居表每5.0仿真秒刷新一次；车辆策略只能读取自己的局部表。
# 该周期可按实验精度要求调小，但会增加高倍速下的计算负载。
NEIGHBOR_GRID_CELL_SIZE_M = 60.0
NEIGHBOR_UPDATE_INTERVAL_S = 5.0

# 覆盖信息素参数。30米网格对应100×100数组，足够轻量且便于观察。
COVERAGE_CELL_SIZE_M = 30.0
COVERAGE_DEPOSIT_RATE_PER_S = 0.5
COVERAGE_EVAPORATION_RATE_PER_S = 0.01
COVERAGE_MAX_VALUE = 1.0
COVERAGE_SAMPLE_DISTANCE_M = 60.0
COVERAGE_OBSERVATION_INTERVAL_S = 0.5
COVERAGE_HEATMAP_VISIBLE_DEFAULT = False
COVERAGE_HEATMAP_REFRESH_FPS = 1
COVERAGE_HEATMAP_COLOR = (235, 96, 52)
COVERAGE_HEATMAP_MAX_ALPHA = 105

# 初期无人车运动学参数
UGV_RADIUS_M = 1.0
UGV_MAX_SPEED_MPS = 8.0
UGV_MAX_ACCELERATION_MPS2 = 2.5
UGV_MAX_TURN_RATE_DEG_S = 45.0
UGV_SENSING_RANGE_M = 60.0
UGV_COMMUNICATION_RANGE_M = 120.0
UGV_LOCAL_HISTORY_LENGTH = 50
UGV_HISTORY_SAMPLE_INTERVAL_S = 0.5
UGV_INITIAL_HEADING_DEG = -90.0
UGV_DEPLOYMENT_SPACING_M = 22.0
UGV_COLLISION_CLEARANCE_M = 1.0

# 20节点基础自主移动参数。
# 第一版只实现“前进—受阻—原地转向—继续”，不包含路径规划和信息素。
UGV_AUTONOMOUS_ENABLED = True
UGV_AUTONOMOUS_CRUISE_SPEED_MPS = 4.0
UGV_AUTONOMOUS_ACCELERATION_MPS2 = 1.5
UGV_AUTONOMOUS_TURN_RATE_DEG_S = 45.0
UGV_AUTONOMOUS_TURN_ANGLE_DEG = 90.0

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
    "communication_range": (70, 117, 168),
    "communication_link": (105, 144, 184),
    "gate": (71, 130, 145),
    "fence": (45, 59, 72),
    "runway_mark": (240, 240, 235),
    "text": (30, 38, 46),
    "panel": (245, 247, 249),
    "button": (225, 231, 234),
    "button_active": (71, 130, 145),
    "button_active_text": (255, 255, 255),
}

# 不同地面的相对移动代价。数值越小，后续路径选择越倾向该区域。
ROAD_MOVEMENT_COST = 1.0
OPEN_AREA_MOVEMENT_COST = 1.5
RESTRICTED_AREA_MOVEMENT_COST = 2.0
