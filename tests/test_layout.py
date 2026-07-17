"""自适应左侧状态栏和地图视口布局测试。"""

import unittest
from unittest.mock import patch

import pygame

from airport_map import AirportMap
from main import calculate_interface_layout


class InterfaceLayoutTests(unittest.TestCase):
    """验证状态栏不会与机场地图视口重叠。"""

    def test_sidebar_and_map_do_not_overlap_at_supported_sizes(self) -> None:
        for size in ((640, 480), (1280, 720), (1920, 1080)):
            with self.subTest(size=size):
                sidebar, map_viewport = calculate_interface_layout(size)

                self.assertLessEqual(sidebar.right, map_viewport.left)
                self.assertEqual(map_viewport.right, size[0])
                self.assertEqual(map_viewport.top, 0)
                self.assertEqual(map_viewport.height, size[1])
                self.assertGreater(sidebar.width, 0)
                self.assertGreater(map_viewport.width, 0)

    def test_map_keeps_at_least_half_window_width(self) -> None:
        for size in ((640, 480), (800, 1200), (1920, 1080)):
            with self.subTest(size=size):
                _, map_viewport = calculate_interface_layout(size)
                self.assertGreaterEqual(map_viewport.width, size[0] // 2)

    def test_map_does_not_draw_floating_legend_by_default(self) -> None:
        pygame.font.init()
        surface = pygame.Surface((640, 480))
        font = pygame.font.Font(None, 12)
        airport_map = AirportMap()
        _, map_viewport = calculate_interface_layout(surface.get_size())

        with patch.object(airport_map, "draw_legend") as draw_legend:
            airport_map.draw(surface, font, map_viewport)

        draw_legend.assert_not_called()


if __name__ == "__main__":
    unittest.main()
