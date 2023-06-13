import math
import random
from typing import Set, List

from common.pose2d import Position2d
from path_planner.planner import BasePlanner, GridPath
from common.pose2d import Position2d, euclidean_distance
from map.global_map import GlobalMap

from common.logger import logger

class Node:
    def __init__(self, p:Position2d) -> None:
        self.p = p
        self.cost = 0.
        self.parent:Node = None
    def __hash__(self) -> int:
        return hash(self.p)

class InformedRRTStarPlanner(BasePlanner):
    def prepare(self, global_map:GlobalMap,iter_times=20000, step_size:float=0.5,sample_end_p_porb:float=0.2,r_goal:float=5,heuristics_coef:float=1) -> bool:
        self._map = global_map
        self._initialized = True
        self._sample_end_p_porb = sample_end_p_porb
        self._iter_times= iter_times
        self._step_size = step_size
        self._r_goal = r_goal
        self._heuristics_coef = heuristics_coef
        
    def __get_current_min_path(self) -> GridPath:
        # generate path
        path = GridPath()
        node = self._end_node
        while node.parent != node:
            path.add_point(node.p)
            node = node.parent
        path.add_point(node.p)
        path.reverse()
        return path
    
    def _plan(self, start_p: Position2d, end_p: Position2d) -> bool:
        self._end_node = Node(end_p)
        start_node = Node(start_p)
        start_node.parent = start_node
        self._tree:Set[Node] = set()
        self._tree.add(start_node)
        c_best = math.inf
        best_path = GridPath()
        for _ in range(self._iter_times):
            if _ % 100 == 0:
                logger.info(f"current step:{_}")
            rand_node = self._sample(start_node, self._end_node, c_best)
            if not self._map.can_reached(rand_node.p):break
            nearest_node = self._get_nearest(rand_node)
            new_node = self._steer(nearest_node, rand_node)
            if self._collision_free(nearest_node, new_node):
                near_nodes_set = self._get_near_nodes(new_node)
                new_node = self._rewire(new_node, nearest_node, near_nodes_set)                
                self._tree.add(new_node)
                if self._is_in_goal_region(new_node) and self._collision_free(new_node, self._end_node):
                    self._end_node = self._rewire(self._end_node, new_node, set())
                    tmp_path = self.__get_current_min_path()
                    tmp_path_length = tmp_path.path_length()
                    if tmp_path_length < c_best:
                        best_path = tmp_path
                        c_best = tmp_path_length
        if self._end_node.parent is None: return False
        self._path = best_path
        return True
  
    def _get_near_nodes(self, new_node:Node) -> Set[Node]:
        nums = len(self._tree)
        r = 50.0 * math.sqrt(math.log(nums) / nums)
        node_set:Set[Node] = set()
        for node in self._tree:
            if euclidean_distance(new_node.p, node.p) <= r:
                node_set.add(node)
        return node_set        
        
    
    def _rewire(self, new_node:Node, nearest_node:Node, near_node_sets:Set[Node]) -> Node:
        node_min = nearest_node
        cost_min = nearest_node.cost + self._heuristics_coef * euclidean_distance(nearest_node.p, new_node.p)
        for node in near_node_sets:
            cost_new = node.cost + self._heuristics_coef * euclidean_distance(node.p, new_node.p)
            if cost_new < cost_min and self._collision_free(node, new_node):
                cost_min = cost_new
                node_min = node
        new_node.parent = node_min
        new_node.cost = cost_min
        return new_node
        
    def _is_in_goal_region(self, node:Node) -> bool:
        return euclidean_distance(node.p, self._end_node.p) <= self._r_goal
    
    
    def _collision_free(self, start:Node, end:Node) -> bool:
        start_p = start.p
        end_p = end.p
        vec = end_p - start_p
        length = vec.norm()
        for i in range(0, int(length) * 10):
            p = start_p + vec * (i/ 10.)
            if not self._map.can_reached(p):
                return False
        return True
    
    def _steer(self, near_node:Node, rand_node:Node) -> Node:
        vec = rand_node.p - near_node.p
        new_p = near_node.p +  self._step_size * vec
        return Node(new_p)
    
    
    def _sample(self, start: Node, goal:Node, c_best:float) -> Node:
        if random.random() < self._sample_end_p_porb: return self._end_node
        start_p = start.p
        goal_p = goal.p
        rand_p = Position2d(-1, -1)
        all_sampled = 0
        while True:
            all_sampled += 1
            if c_best < math.inf:
                c_min = (start_p - goal_p).norm()
                centre_p = (start_p + goal_p) / 2
                # build ellipse
                a = c_best / 2
                c = c_min / 2
                b = math.sqrt(a * a - c * c)
                standard_rand_p = self.__sample_in_standard_ellipse(a, b)
                theta = (goal_p - start_p).angle_x()
                # [x', y']^T = [cos\theta, sin\theta; -sin\theta,cos\theta] * [x, y] + [x_center, y_center]
                rand_p = standard_rand_p.rotate(theta) + centre_p
                
            else:
            # sample from anywhile of map.
                rand_p.x = int(random.uniform(0, 1) * self._map.width())
                rand_p.y = int(random.uniform(0, 1) * self._map.length())
            
            if self._map.can_reached(rand_p):
                break
            if all_sampled > 100:
                return Node(Position2d(-1, -1))
        return Node(rand_p)
                
            
    def _get_nearest(self, rand:Node) -> Node:
        min_distance = math.inf
        ans_node = None
        for node in self._tree:    
            distance = euclidean_distance(node.p, rand.p) 
            if distance < min_distance:
                min_distance =  distance
                ans_node = node
        return ans_node
                 
    
    @staticmethod
    def __sample_in_standard_ellipse(a: float, b:float) ->Position2d:
        """sample point in standard ellipse: x^2/a^2 + y^2/b^2 = 1
        here we use polar corridnate via Box-Muller algorithm to sample
        reference: https://stackoverflow.com/questions/5529148/algorithm-calculate-pseudo-random-point-inside-an-ellipse
        Args:
            a (float): semi-major axis
            b (float):"semi-minor axis

        Returns:
            Position2d: sampled point in standard ellipse.
        """
        # ellipse area
        S = math.pi * a * b
        # sample num
        N = 1
        # sample random theta
        theta = 2 * math.pi * random.uniform(0, 1)
        # calcuate radius
        r = math.sqrt(S / N / math.pi)
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        return Position2d(x, y)        
    
    # for debug
    def get_all_path(self):
        def generate_path(node:Node)->GridPath:
            path = GridPath()
            while node.parent != node:
                path.add_point(node.p)
                node = node.parent
            path.reverse()
            return path

        path_list:List[GridPath] = []
        for node in self._tree:
            path = generate_path(node)
            if path.get_node_nums() > 1:
                path_list.append(path)
        return path_list
            
        
                
        
        
    
        
        
