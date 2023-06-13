from typing import List
from common.pose2d import Position2d
from map.base_map import GridMap

class GridPath:
    def __init__(self):
        # path length
        self._path:List[Position2d] = []
        
    def add_point(self, point:Position2d)->None:
        self._path.append(point)
    
    def path_length(self) -> float:
        if len(self._path) == 0: return 0
        length = 0
        start_p = self._path[0]
        for p in self._path:
            length += (p - start_p).norm()
            start_p = p
        return length
    
    def reverse(self):
        self._path[::-1]
    
    def get_path(self):
        return self._path
    
    def get_node_nums(self):
        return len(self._path)
    

class BasePlanner:
    def __init__(self) -> None:
        self._path:GridPath = GridPath()
        self._initialized = False
        self._map:GridMap = None
        
    def prepare(self,*args, **kwargs) -> bool:
        raise NotImplementedError
    
    def plan(self, start_p:Position2d, end_p:Position2d) -> bool:
        if not self._initialized: return False
        if not self.__is_position_valid(start_p, end_p):return False
        return self._plan(start_p, end_p)
        
    
    def _plan(self, start_p:Position2d, end_p:Position2d) -> bool:
        raise NotImplementedError    
    
    def get_planned_path(self):
        return self._path
    
    def __is_position_valid(self, start_p:Position2d, end_p: Position2d) -> bool:
        return self._map is not None and self._map.can_reached(start_p) and self._map.can_reached(end_p)
    
    