# -*- coding: utf-8 -*-
"""
Created on Thu Sep 25 10:08:21 2025

@author: muskan
"""

"""
Environment module for the autonomous delivery agent.
Models the 2D grid world with static/dynamic obstacles and terrain costs.
"""

class Cell:
    """Represents a single cell in the grid."""
    def __init__(self, terrain_cost=1, is_obstacle=False, dynamic_obstacle=False):
        self.terrain_cost = terrain_cost
        self.is_obstacle = is_obstacle
        self.dynamic_obstacle = dynamic_obstacle
        self.dynamic_id = None  # ID for tracking moving obstacles
    
    def __str__(self):
        if self.is_obstacle:
            return 'X'
        elif self.dynamic_obstacle:
            return 'D'
        elif self.terrain_cost > 1:
            return str(self.terrain_cost)
        else:
            return '.'
    
    def is_traversable(self, time_step=0):
        """Check if cell can be traversed at given time step."""
        return not self.is_obstacle and not self.dynamic_obstacle

class DynamicObstacle:
    """Represents a moving obstacle with a predictable path."""
    def __init__(self, obstacle_id, path, speed=1):
        self.id = obstacle_id
        self.path = path  # List of (row, col) positions
        self.speed = speed
        self.current_step = 0
    
    def get_position_at_time(self, time_step):
        """Get position at specific time step."""
        if not self.path:
            return None
        index = (time_step // self.speed) % len(self.path)
        return self.path[index]
    
    def move(self):
        """Move to next position."""
        self.current_step += 1

class GridEnvironment:
    """2D grid environment for the delivery agent."""
    
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [[Cell() for _ in range(width)] for _ in range(height)]
        self.dynamic_obstacles = {}
        self.agent_start = (0, 0)
        self.delivery_points = []
        self.time_step = 0
    
    def load_from_file(self, filename):
        """Load grid configuration from file."""
        try:
            with open(filename, 'r') as f:
                lines = [line.strip() for line in f if line.strip() and not line.startswith('#')]
            
            # Parse basic dimensions
            if lines[0].startswith('SIZE'):
                _, w, h = lines[0].split()
                self.width, self.height = int(w), int(h)
                self.grid = [[Cell() for _ in range(self.width)] for _ in range(self.height)]
                lines = lines[1:]
            
            for line in lines:
                parts = line.split()
                if not parts:
                    continue
                
                cmd = parts[0]
                
                if cmd == 'START':
                    self.agent_start = (int(parts[2]), int(parts[1]))  # (row, col)
                
                elif cmd == 'GOAL':
                    self.delivery_points.append((int(parts[2]), int(parts[1])))
                
                elif cmd == 'TERRAIN':
                    col, row, cost = map(int, parts[1:4])
                    if 0 <= row < self.height and 0 <= col < self.width:
                        self.grid[row][col].terrain_cost = cost
                
                elif cmd == 'OBSTACLE':
                    col, row = map(int, parts[1:3])
                    if 0 <= row < self.height and 0 <= col < self.width:
                        self.grid[row][col].is_obstacle = True
                
                elif cmd == 'DYNAMIC':
                    obstacle_id = parts[1]
                    path = []
                    for i in range(2, len(parts), 2):
                        col, row = int(parts[i]), int(parts[i+1])
                        path.append((row, col))
                    
                    if path:
                        dynamic_obs = DynamicObstacle(obstacle_id, path)
                        self.dynamic_obstacles[obstacle_id] = dynamic_obs
                        # Mark initial position
                        start_pos = path[0]
                        if 0 <= start_pos[0] < self.height and 0 <= start_pos[1] < self.width:
                            self.grid[start_pos[0]][start_pos[1]].dynamic_obstacle = True
                            self.grid[start_pos[0]][start_pos[1]].dynamic_id = obstacle_id
        
        except Exception as e:
            print(f"Error loading map file: {e}")
    
    def update_dynamic_obstacles(self):
        """Update positions of dynamic obstacles."""
        # Clear previous dynamic obstacle positions
        for row in range(self.height):
            for col in range(self.width):
                if self.grid[row][col].dynamic_obstacle:
                    self.grid[row][col].dynamic_obstacle = False
                    self.grid[row][col].dynamic_id = None
        
        # Update to new positions
        for obs_id, obstacle in self.dynamic_obstacles.items():
            pos = obstacle.get_position_at_time(self.time_step)
            if pos and 0 <= pos[0] < self.height and 0 <= pos[1] < self.width:
                self.grid[pos[0]][pos[1]].dynamic_obstacle = True
                self.grid[pos[0]][pos[1]].dynamic_id = obs_id
        
        self.time_step += 1
    
    def get_neighbors(self, position, allow_diagonal=False):
        """Get traversable neighboring cells."""
        row, col = position
        neighbors = []
        
        # 4-connected movements
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if allow_diagonal:
            directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if (0 <= new_row < self.height and 0 <= new_col < self.width and
                self.grid[new_row][new_col].is_traversable()):
                neighbors.append((new_row, new_col))
        
        return neighbors
    
    def get_cost(self, position):
        """Get movement cost for a cell."""
        row, col = position
        return self.grid[row][col].terrain_cost
    
    def is_valid_position(self, position):
        """Check if position is within bounds and traversable."""
        row, col = position
        return (0 <= row < self.height and 0 <= col < self.width and 
                self.grid[row][col].is_traversable())
    
    def display(self, agent_position=None):
        """Display the grid with optional agent position."""
        for row in range(self.height):
            row_str = ""
            for col in range(self.width):
                if agent_position and (row, col) == agent_position:
                    row_str += "A "
                else:
                    row_str += str(self.grid[row][col]) + " "
            print(row_str)
        print()