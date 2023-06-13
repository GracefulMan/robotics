import sys
import time
sys.path.append("/Users/bytedance/Desktop/robotics/python")

from map.global_map import GlobalMap
from common.visualize import visualizer
from common.pose2d import Position2d
from common.logger import logger
from path_planner.global_path_planner.a_star_planner import AStarPlanner
from path_planner.global_path_planner.rrt_star_planner import InformedRRTStarPlanner

map1 = GlobalMap()
filepath = 'files/map_files/map_resolution_0.05.png' 
map1.loadmap_from_image(filepath)
visualizer.add_map(map1)
start_p = Position2d(85, 10)
end_p = Position2d(50, 110)


global_planner = AStarPlanner()
global_planner.prepare(map1,heuristics_coef=0.01)
start_time = time.time()
print(start_p, end_p)
success = global_planner.plan(start_p, end_p)
if success:
    path_length = global_planner.get_planned_path().path_length()
    logger.info(f"A Star Path Length:{path_length}, cost time:f{time.time() - start_time}")
    visualizer.add_path(global_planner.get_planned_path())

global_planner = InformedRRTStarPlanner()
global_planner.prepare(map1)
start_time = time.time()
success = global_planner.plan(start_p, end_p)
if success:
    path_length = global_planner.get_planned_path().path_length()
    logger.info(f"Informed RRT* Path Length:{path_length}, cost time:{time.time() - start_time}")
    visualizer.add_path(global_planner.get_planned_path())
else:
    path_list = global_planner.get_all_path()
    for path in path_list:
        visualizer.add_path(path)

visualizer.add_point(start_p)
visualizer.add_point(end_p)
visualizer.visualize()
