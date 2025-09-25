# -*- coding: utf-8 -*-
"""
Created on Thu Sep 25 10:09:20 2025

@author: muskan
"""

"""
Search algorithms for pathfinding: BFS, Uniform-cost, A*, and local search methods.
"""

import heapq
import math
import random
from collections import deque

class SearchAlgorithm:
    """Base class for search algorithms."""
    
    def __init__(self, environment):
        self.env = environment
        self.nodes_expanded = 0
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from start to goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def manhattan_distance(self, pos1, pos2):
        """Calculate Manhattan distance between two positions."""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

class BFS(SearchAlgorithm):
    """Breadth-First Search implementation."""
    
    def search(self, start, goal):
        """Perform BFS search from start to goal."""
        frontier = deque([start])
        came_from = {start: None}
        self.nodes_expanded = 0
        
        while frontier:
            current = frontier.popleft()
            self.nodes_expanded += 1
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            for neighbor in self.env.get_neighbors(current):
                if neighbor not in came_from:
                    frontier.append(neighbor)
                    came_from[neighbor] = current
        
        return None  # No path found

class UniformCostSearch(SearchAlgorithm):
    """Uniform-Cost Search implementation."""
    
    def search(self, start, goal):
        """Perform UCS search from start to goal."""
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        self.nodes_expanded = 0
        
        while frontier:
            current_cost, current = heapq.heappop(frontier)
            self.nodes_expanded += 1
            
            if current == goal:
                return self.reconstruct_path(came_from, current), current_cost
            
            for neighbor in self.env.get_neighbors(current):
                new_cost = cost_so_far[current] + self.env.get_cost(neighbor)
                
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    heapq.heappush(frontier, (new_cost, neighbor))
                    came_from[neighbor] = current
        
        return None, float('inf')  # No path found

class AStarSearch(SearchAlgorithm):
    """A* Search implementation."""
    
    def __init__(self, environment, heuristic='manhattan'):
        super().__init__(environment)
        self.heuristic = heuristic
    
    def heuristic_function(self, pos, goal):
        """Calculate heuristic value."""
        if self.heuristic == 'manhattan':
            return self.manhattan_distance(pos, goal)
        elif self.heuristic == 'euclidean':
            return math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)
        else:
            return 0  # Default to greedy search
    
    def search(self, start, goal):
        """Perform A* search from start to goal."""
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        self.nodes_expanded = 0
        
        while frontier:
            current_priority, current = heapq.heappop(frontier)
            self.nodes_expanded += 1
            
            if current == goal:
                path = self.reconstruct_path(came_from, current)
                total_cost = cost_so_far[current]
                return path, total_cost
            
            for neighbor in self.env.get_neighbors(current):
                new_cost = cost_so_far[current] + self.env.get_cost(neighbor)
                
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic_function(neighbor, goal)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current
        
        return None, float('inf')  # No path found

class HillClimbingSearch(SearchAlgorithm):
    """Hill Climbing with Random Restarts for local search/replanning."""
    
    def __init__(self, environment, max_restarts=10, max_steps=100):
        super().__init__(environment)
        self.max_restarts = max_restarts
        self.max_steps = max_steps
    
    def evaluate_path(self, path):
        """Evaluate total cost of a path."""
        if not path or len(path) < 2:
            return float('inf')
        
        total_cost = 0
        for i in range(len(path) - 1):
            total_cost += self.env.get_cost(path[i+1])
        return total_cost
    
    def get_random_path(self, start, goal, max_length=50):
        """Generate a random valid path from start to goal."""
        path = [start]
        current = start
        steps = 0
        
        while current != goal and steps < max_length:
            neighbors = self.env.get_neighbors(current)
            if not neighbors:
                break
            
            next_pos = random.choice(neighbors)
            # Avoid cycles
            if next_pos in path[-10:]:  # Simple cycle prevention
                neighbors.remove(next_pos)
                if neighbors:
                    next_pos = random.choice(neighbors)
                else:
                    break
            
            path.append(next_pos)
            current = next_pos
            steps += 1
        
        return path if current == goal else None
    
    def perturb_path(self, path, perturbation_strength=2):
        """Perturb an existing path to create a variation."""
        if len(path) <= 3:
            return path
        
        new_path = path.copy()
        # Randomly modify some segments
        for _ in range(perturbation_strength):
            if len(new_path) > 4:
                # Remove a small segment and find alternative
                idx = random.randint(1, len(new_path) - 3)
                segment_start = new_path[idx-1]
                segment_end = new_path[idx+2] if idx + 2 < len(new_path) else new_path[-1]
                
                # Try to find alternative path for this segment
                ucs = UniformCostSearch(self.env)
                segment_path, _ = ucs.search(segment_start, segment_end)
                
                if segment_path and len(segment_path) > 2:
                    new_path = new_path[:idx] + segment_path[1:-1] + new_path[idx+2:]
        
        return new_path
    
    def search(self, start, goal, initial_path=None):
        """Perform hill climbing with random restarts."""
        best_path = initial_path
        best_cost = self.evaluate_path(initial_path) if initial_path else float('inf')
        
        self.nodes_expanded = 0
        
        for restart in range(self.max_restarts):
            # Start with random path if no initial path or on restart
            if restart > 0 or not initial_path:
                current_path = self.get_random_path(start, goal)
                if not current_path:
                    continue
            else:
                current_path = initial_path.copy()
            
            current_cost = self.evaluate_path(current_path)
            
            for step in range(self.max_steps):
                self.nodes_expanded += 1
                
                # Generate neighbor by perturbing current path
                neighbor_path = self.perturb_path(current_path)
                neighbor_cost = self.evaluate_path(neighbor_path)
                
                # If neighbor is better, move to it
                if neighbor_cost < current_cost:
                    current_path = neighbor_path
                    current_cost = neighbor_cost
                
                # Local optimum reached
                else:
                    break
            
            # Update global best
            if current_cost < best_cost:
                best_path = current_path
                best_cost = current_cost
        
        return best_path, best_cost