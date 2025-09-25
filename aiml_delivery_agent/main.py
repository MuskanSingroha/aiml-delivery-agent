# -*- coding: utf-8 -*-
"""
Created on Thu Sep 25 10:12:09 2025

@author: muskan
"""

#!/usr/bin/env python3
"""
Main CLI interface for the autonomous delivery agent project.
"""

import argparse
import sys
from environment import GridEnvironment
from agent import DeliveryAgent
from utils import load_map, create_test_maps, run_experiment, print_results, save_logs

def main():
    parser = argparse.ArgumentParser(description='Autonomous Delivery Agent')
    parser.add_argument('--map', type=str, help='Map file to load')
    parser.add_argument('--algorithm', choices=['bfs', 'ucs', 'astar', 'hillclimb'], 
                       default='astar', help='Pathfinding algorithm to use')
    parser.add_argument('--create-maps', action='store_true', help='Create test maps')
    parser.add_argument('--experiment', action='store_true', help='Run comparative experiment')
    parser.add_argument('--demo', action='store_true', help='Run dynamic obstacle demo')
    
    args = parser.parse_args()
    
    if args.create_maps:
        create_test_maps()
        print("Test maps created in 'maps/' directory")
        return
    
    if not args.map:
        print("Please specify a map file with --map")
        sys.exit(1)
    
    # Load environment
    env = load_map(args.map)
    agent = DeliveryAgent(env)
    
    if args.experiment:
        # Run comparative experiment
        algorithms = ['bfs', 'ucs', 'astar']
        start = env.agent_start
        goal = env.delivery_points[0] if env.delivery_points else (env.height-1, env.width-1)
        
        print(f"Running experiment on {args.map}")
        print(f"Start: {start}, Goal: {goal}")
        
        results = run_experiment(env, algorithms, start, goal, trials=5)
        print_results(results)
        
        # Save results
        with open('experiment_results.txt', 'w') as f:
            for result in results:
                f.write(f"{result}\n")
        
        return
    
    if args.demo:
        # Dynamic obstacle demonstration
        print("=== Dynamic Obstacle Demo ===")
        env.display(agent.position)
        
        goals = env.delivery_points if env.delivery_points else [(env.height-1, env.width-1)]
        
        step = 0
        while goals and agent.fuel > 0:
            print(f"\n--- Step {step} ---")
            env.update_dynamic_obstacles()
            env.display(agent.position)
            
            next_goal = goals[0]
            path, cost, nodes = agent.plan_path(next_goal, algorithm=args.algorithm)
            
            if path:
                print(f"Path found (cost: {cost}, nodes expanded: {nodes})")
                success = agent.move_along_path(path[:2])  # Move one step at a time for demo
                
                if agent.position == next_goal:
                    goals.pop(0)
                    print(f"Reached goal! {len(goals)} goals remaining.")
            
            else:
                print("No path found! Attempting replanning...")
                replan_path, replan_cost, _ = agent.plan_path(next_goal, algorithm='hillclimb')
                if replan_path:
                    print("Replanning successful!")
                    success = agent.move_along_path(replan_path[:2])
                else:
                    print("Replanning failed!")
                    break
            
            step += 1
            if step > 50:  # Safety limit
                break
        
        print(f"\nDemo completed. Final position: {agent.position}")
        save_logs(agent, 'demo_log.json')
        return
    
    # Normal operation
    goals = env.delivery_points if env.delivery_points else [(env.height-1, env.width-1)]
    deliveries = agent.deliver_packages(goals, algorithm=args.algorithm)
    
    print(f"Delivery completed: {deliveries}/{len(goals)} packages delivered")
    print(f"Final status: {agent.get_status()}")
    
    # Save logs
    save_logs(agent, 'delivery_log.json')
    print("Logs saved to delivery_log.json")

if __name__ == "__main__":
    main()"# Test comment" 
