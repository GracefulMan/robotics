from threading import RLock
from typing import List
import matplotlib.pyplot as plt
import numpy as np

from map.base_map import GridMap, MapValue
from common.pose2d import Position2d
from path_planner.planner import GridPath
class __NavigationVisualize:
    _instance_lock = RLock()
    _instance = None
    def __init__(self):
        self._maps:List[GridMap] = []
        self._points:List[Position2d] = []
        self._paths:List[GridPath] = []
        
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            with cls._instance_lock:
                if not cls._instance:
                    cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance
                    
        
    def add_point(self, point:Position2d):
        self._points.append(point)

    def add_map(self, map: GridMap):
        self._maps.append(map)
        
    def add_path(self, path:GridPath):
        self._paths.append(path)
        
    def visualize(self):
        # 定义颜色映射
        cmap = plt.get_cmap('jet', len(MapValue))
        # colors = cmap(np.arange(len(MapValue)))
        # values = [int(m.value) for m in MapValue]
        color_map = {
            int(MapValue.FREE):[255, 255, 255],
            int(MapValue.OBSTACLE):[0, 0, 0],
            int(MapValue.SAFETY):[120, 0, 0],
            int(MapValue.WARNING):[150, 0, 0]
            }

        # 计算画布大小
        width = max([m.width() for m in self._maps])
        length = max([m.length() for m in self._maps])

        # 将所有地图拼接到一起
        color_grid = np.zeros((width, length, 3))
        for map in self._maps:
            print(map.map().shape)
            for i in range(map.width()):
                for j in range(map.length()):
                    color_grid[i,  j] = color_map[map.get_value(i, j)][:3]
        
         # 将所有点绘制到地图上
        for point in self._points:
            color_grid[point.x, point.y] = [0, 255, 0]  # 将点的颜色设置为红色
            
        # 将所有路径绘制到地图上
        for path in self._paths:
            path_points = path.get_path()
            for i in range(len(path_points) - 1):
                start_p = path_points[i]
                end_p = path_points[i + 1]
                color_grid[start_p.x, start_p.y] = [255, 0, 0]  # 将起点颜色设置为红色
                color_grid[end_p.x, end_p.y] = [255, 0, 0]  # 将终点颜色设置为红色
                plt.plot([start_p.y, end_p.y], [start_p.x, end_p.x], color='r',linewidth=2)

        
         # 显示地图
        plt.imshow(color_grid)
        plt.axis('off')
        plt.show()
        

        


       
visualizer = __NavigationVisualize()