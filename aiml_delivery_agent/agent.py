# -*- coding: utf-8 -*-
"""
Created on Thu Sep 25 10:09:46 2025

@author: muskan
"""

"""
Autonomous delivery agent that uses different planning strategies.
"""

import os
import sys

# Add current directory to Python path to ensure imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import all required search algorithms
from search_algorithms import BFS, UniformCostSearch, AStarSearch, HillClimbingSearch

class DeliveryAgent:
    """Autonomous delivery agent."""
    
    def __init__(self, environment):
        self.env = environment
        self.position = environment.agent_start
        self.path = []
        self.current_goal_index = 0
        self.total_distance = 0
        self.fuel = 1000  # Initial fuel capacity
        self.log = []
    
    def plan_path(self, goal, algorithm='astar', **kwargs):
        """Plan path to goal using specified algorithm."""
        if algorithm == 'bfs':
            search = BFS(self.env)
            path = search.search(self.position, goal)
            cost = len(path) if path else float('inf')
            nodes_expanded = search.nodes_expanded
        
        elif algorithm == 'ucs':
            search = UniformCostSearch(self.env)
            path, cost = search.search(self.position, goal)
            nodes_expanded = search.nodes_expanded
        
        elif algorithm == 'astar':
            heuristic = kwargs.get('heuristic', 'manhattan')
            search = AStarSearch(self.env, heuristic=heuristic)
            path, cost = search.search(self.position, goal)
            nodes_expanded = search.nodes_expanded
        
        elif algorithm == 'hillclimb':
            max_restarts = kwargs.get('max_restarts', 10)
            initial_path = kwargs.get('initial_path', None)
            search = HillClimbingSearch(self.env, max_restarts=max_restarts)
            path, cost = search.search(self.position, goal, initial_path)
            nodes_expanded = search.nodes_expanded
        
        else:
            raise ValueError(f"Unknown algorithm: {algorithm}")
        
        self.log.append({
            'algorithm': algorithm,
            'from': self.position,
            'to': goal,
            'path_length': len(path) if path else 0,
            'cost': cost,
            'nodes_expanded': nodes_expanded,
            'success': path is not None
        })
        
        return path, cost, nodes_expanded
    
    def move_along_path(self, path):
        """Move along the planned path."""
        if not path or len(path) <= 1:
            return False
        
        # Skip first position (current position)
        for next_pos in path[1:]:
            if not self.env.is_valid_position(next_pos):
                self.log.append({'event': 'blocked', 'position': next_pos})
                return False  # Path blocked
            
            # Move to next position
            move_cost = self.env.get_cost(next_pos)
            if self.fuel >= move_cost:
                self.position = next_pos
                self.total_distance += 1
                self.fuel -= move_cost
                self.log.append({
                    'event': 'move', 
                    'to': next_pos, 
                    'cost': move_cost, 
                    'fuel_remaining': self.fuel
                })
            else:
                self.log.append({'event': 'out_of_fuel'})
                return False
        
        return True
    
    def deliver_packages(self, goals, algorithm='astar', replan_strategy='hillclimb'):
        """Deliver packages to multiple goals."""
        remaining_goals = goals.copy()
        current_path = None
        
        while remaining_goals and self.fuel > 0:
            next_goal = remaining_goals[0]
            
            # Plan path to next goal
            path, cost, nodes_expanded = self.plan_path(next_goal, algorithm=algorithm)
            
            if not path:
                self.log.append({'event': 'no_path_to_goal', 'goal': next_goal})
                break
            
            # Try to follow the path
            success = self.move_along_path(path)
            
            if success:
                # Reached goal
                remaining_goals.pop(0)
                self.log.append({
                    'event': 'delivery_complete',
                    'goal': next_goal,
                    'goals_remaining': len(remaining_goals)
                })
            else:
                # Path blocked, need to replan
                self.log.append({'event': 'replanning_triggered'})
                
                if replan_strategy:
                    # Use local search for quick replanning
                    replan_path, replan_cost, _ = self.plan_path(
                        next_goal, 
                        algorithm=replan_strategy,
                        initial_path=path  # Use original path as starting point
                    )
                    
                    if replan_path:
                        path = replan_path
                        success = self.move_along_path(path)
                        
                        if success:
                            remaining_goals.pop(0)
                            self.log.append({
                                'event': 'delivery_complete_after_replan',
                                'goal': next_goal
                            })
                        else:
                            self.log.append({'event': 'replanning_failed'})
                            break
                    else:
                        self.log.append({'event': 'replanning_failed'})
                        break
        
        return len(goals) - len(remaining_goals)  # Number of deliveries completed
    
    def get_status(self):
        """Get current agent status."""
        return {
            'position': self.position,
            'fuel_remaining': self.fuel,
            'distance_traveled': self.total_distance,
            'log_entries': len(self.log)
        }