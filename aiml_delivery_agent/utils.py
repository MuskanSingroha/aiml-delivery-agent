# -*- coding: utf-8 -*-
"""
Created on Thu Sep 25 10:11:20 2025

@author: muskan
"""

"""
Utility functions for the project.
"""

import time
import json
import os
import sys

# Add current directory to Python path to ensure imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import all required classes
from environment import GridEnvironment
from search_algorithms import BFS, UniformCostSearch, AStarSearch

def load_map(filename):
    """Load map file and create environment."""
    # First, read the map file to get dimensions
    try:
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith('SIZE'):
                    parts = line.split()
                    width, height = int(parts[1]), int(parts[2])
                    env = GridEnvironment(width, height)
                    env.load_from_file(filename)
                    return env
    except:
        pass
    
    # If SIZE not found or error, use defaults
    env = GridEnvironment(10, 10)
    env.load_from_file(filename)
    return env

def create_test_maps():
    """Create test map files for the project."""
    
    # Create maps directory if it doesn't exist
    os.makedirs('maps', exist_ok=True)
    
    # Small map
    small_map = """SIZE 5 5
START 0 0
GOAL 4 4
TERRAIN 1 1 2
TERRAIN 2 2 3
OBSTACLE 1 2
OBSTACLE 2 1
OBSTACLE 3 3"""
    
    # Medium map
    medium_map = """SIZE 10 10
START 0 0
GOAL 9 9
TERRAIN 2 2 2
TERRAIN 3 3 3
TERRAIN 4 4 4
TERRAIN 5 5 2
OBSTACLE 1 1
OBSTACLE 2 2
OBSTACLE 3 3
OBSTACLE 4 4
OBSTACLE 5 5
OBSTACLE 6 6
OBSTACLE 7 7
OBSTACLE 8 8"""
    
    # Large map
    large_map = """SIZE 20 20
START 0 0
GOAL 19 19
TERRAIN 5 5 2
TERRAIN 10 10 3
TERRAIN 15 15 4
# Multiple obstacles creating a maze-like structure
OBSTACLE 5 0
OBSTACLE 5 1
OBSTACLE 5 2
OBSTACLE 5 3
OBSTACLE 5 4
OBSTACLE 10 15
OBSTACLE 10 16
OBSTACLE 10 17
OBSTACLE 10 18
OBSTACLE 15 5
OBSTACLE 15 6
OBSTACLE 15 7
OBSTACLE 15 8"""
    
    # Dynamic map with moving obstacles
    dynamic_map = """SIZE 8 8
START 0 0
GOAL 7 7
TERRAIN 2 2 2
TERRAIN 4 4 3
OBSTACLE 1 1
OBSTACLE 2 2
# Dynamic obstacle moving horizontally
DYNAMIC d1 3 1 3 2 3 3 3 4 3 5
# Dynamic obstacle moving vertically
DYNAMIC d2 1 3 2 3 3 3 4 3 5 3"""
    
    # Write map files
    with open('maps/small.map', 'w') as f:
        f.write(small_map)
    
    with open('maps/medium.map', 'w') as f:
        f.write(medium_map)
    
    with open('maps/large.map', 'w') as f:
        f.write(large_map)
    
    with open('maps/dynamic.map', 'w') as f:
        f.write(dynamic_map)
    
    print("Test maps created successfully!")

def run_experiment(env, algorithms, start, goal, trials=1):
    """Run comparative experiment with different algorithms."""
    results = []
    
    for algorithm in algorithms:
        total_time = 0
        total_cost = 0
        total_nodes = 0
        successes = 0
        
        for trial in range(trials):
            start_time = time.time()
            
            if algorithm == 'bfs':
                search = BFS(env)
                path = search.search(start, goal)
                cost = len(path) if path else float('inf')
                nodes = search.nodes_expanded
            
            elif algorithm == 'ucs':
                search = UniformCostSearch(env)
                path, cost = search.search(start, goal)
                nodes = search.nodes_expanded
            
            elif algorithm == 'astar':
                search = AStarSearch(env)
                path, cost = search.search(start, goal)
                nodes = search.nodes_expanded
            
            end_time = time.time()
            
            if path:
                total_time += (end_time - start_time)
                total_cost += cost
                total_nodes += nodes
                successes += 1
        
        avg_time = total_time / successes if successes > 0 else float('inf')
        avg_cost = total_cost / successes if successes > 0 else float('inf')
        avg_nodes = total_nodes / successes if successes > 0 else float('inf')
        
        results.append({
            'algorithm': algorithm,
            'success_rate': successes / trials * 100,
            'avg_time': avg_time,
            'avg_cost': avg_cost,
            'avg_nodes_expanded': avg_nodes
        })
    
    return results

def print_results(results):
    """Print experiment results in a formatted table."""
    print("\n" + "="*80)
    print(f"{'Algorithm':<12} {'Success Rate':<12} {'Avg Time (s)':<12} {'Avg Cost':<12} {'Avg Nodes':<12}")
    print("="*80)
    
    for result in results:
        print(f"{result['algorithm']:<12} {result['success_rate']:<12.1f} "
              f"{result['avg_time']:<12.4f} {result['avg_cost']:<12.1f} "
              f"{result['avg_nodes_expanded']:<12.0f}")
    print("="*80)

def save_logs(agent, filename):
    """Save agent logs to file."""
    with open(filename, 'w') as f:
        json.dump(agent.log, f, indent=2)
