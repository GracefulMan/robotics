from collections import defaultdict
from typing import Set, List, Dict
from queue import PriorityQueue
import random

from common.pose2d import Position2d
from common.pose2d import euclidean_distance
from map.global_map import GlobalMap
from path_planner.planner import BasePlanner, GridPath


class AStarPlanner(BasePlanner):
    def prepare(self,global_map:GlobalMap,heuristics_coef:float = 0.5) -> bool:
        self._heuristics_coef = heuristics_coef
        self._map = global_map
        self._initialized = True
        return True
    
    
    def _get_cost(self,current_p:Position2d, next_p:Position2d) -> float:
        distance_cost = euclidean_distance(current_p, next_p)
        return distance_cost
    
    def _heuristic_cost(self, p:Position2d, end_p:Position2d) -> float:
        return euclidean_distance(p, end_p)
    
    def _plan(self, start_p: Position2d, end_p: Position2d) -> bool:
        visited:Set[Position2d] = set()
        cost_value = defaultdict(float)
        current_p = start_p
        visited.add(current_p)
        parent:Dict[Position2d, Position2d] = {}
        queue = PriorityQueue()
        queue.put((0., current_p))
        cost_value[current_p] = 0
        visited.add(current_p)
        parent[current_p] = current_p
          
        while not queue.empty():
            _, current_p = queue.get()
            if current_p == end_p:
                break
            for neibor_p in self._map.get_valid_neighbor(current_p):
                new_cost = self._get_cost(current_p, neibor_p) + cost_value[current_p]
                if neibor_p not in visited or new_cost < cost_value[neibor_p]:
                    cost_value[neibor_p] = new_cost
                    visited.add(neibor_p)
                    search_cost = new_cost + self._heuristics_coef * self._heuristic_cost(neibor_p, end_p) + random.random() * 1e-3
                    queue.put((search_cost, neibor_p))
                    parent[neibor_p] = current_p
        
        # can't reached from current
        if end_p not in parent: return False
        # generate path
        p = end_p
        while parent[p] != p:
            self._path.add_point(p)
            p = parent[p]
        self._path.reverse()
        return True
    
        
                  
                    
                    
    
                    
                
                
                
                
                
            
            
        
        
        
        
        
        
        
    