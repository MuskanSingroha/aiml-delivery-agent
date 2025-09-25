# Autonomous Delivery Agent Project

## 📋 Project Overview
This project implements an autonomous delivery agent that navigates a 2D grid city to deliver packages using various AI search algorithms. Developed as part of **CSA2001 - Fundamentals of AI and ML** course project.

## 🎯 Project Objectives
- Model environment with static/dynamic obstacles and varying terrain costs
- Implement multiple pathfinding algorithms (BFS, UCS, A*, Hill Climbing)
- Compare algorithm performance experimentally
- Handle dynamic obstacles with replanning strategies
- Provide comprehensive analysis and documentation

## 👥 Authors
- Muskan
- Course: CSA2001 - Fundamentals of AI and ML

## 📁 Project Structure
aiml-delivery-agent/
├── main.py # Main CLI interface
├── environment.py # Grid environment model
├── agent.py # Delivery agent implementation
├── search_algorithms.py # Pathfinding algorithms implementation
├── utils.py # Utility functions and experiments
├── requirements.txt # Python dependencies
├── README.md # This file
└── maps/ # Test environments
      ├── small.map # 5x5 grid for basic testing
      ├── medium.map # 10x10 grid for standard testing
      ├── large.map # 20x20 grid for performance testing
      └── dynamic.map # 15x15 grid with moving obstacles
      
## 📁 Project Files

### Source Code
- [main.py](main.py) - Main CLI interface
- [environment.py](environment.py) - Grid environment model
- [agent.py](agent.py) - Autonomous delivery agent
- [search_algorithms.py](search_algorithms.py) - Search algorithms implementation
- [utils.py](utils.py) - Utility functions

### Test Maps
- [maps/small.map](maps/small.map) - 5x5 test grid
- [maps/medium.map](maps/medium.map) - 10x10 test grid
- [maps/large.map](maps/large.map) - 20x20 test grid
- [maps/dynamic.map](maps/dynamic.map) - Dynamic obstacles map

### Documentation
- [requirements.txt](requirements.txt) - Python dependencies      
## 🧠 Algorithms Implemented

### **Uninformed Search Algorithms**
- **Breadth-First Search (BFS)** - Complete and optimal for uniform costs
- **Uniform-Cost Search (UCS)** - Optimal for varying terrain costs

### **Informed Search Algorithms**
- **A* Search** - Optimal with admissible heuristics (Manhattan distance)

### **Local Search Algorithms**
- **Hill Climbing with Random Restarts** - For dynamic replanning

## 🚀 Features
- **4-connected grid movement** with customizable terrain costs
- **Dynamic obstacles** with predictable movement patterns
- **Multiple delivery points** with efficient routing
- **Real-time replanning** when paths are blocked
- **Comprehensive logging** and experimental data collection
- **CLI interface** for easy algorithm testing and comparison

## ⚙️ Setup Instructions

### Prerequisites
- Python 3.6 or higher
- pip (Python package manager)

### Installation Steps

#### 1. Clone the Repository
```bash
git clone https://github.com/YOUR_USERNAME/aiml-delivery-agent.git
cd aiml-delivery-agent

2. Install Dependencies
bash
pip install -r requirements.txt
3. Create Test Maps
bash
python main.py --create-maps
Verification
Run a quick test to verify installation:

bash
python main.py --map maps/small.map --algorithm astar
💻 Usage Instructions
Basic Commands
1. Run Specific Algorithm on a Map
bash
# Breadth-First Search
python main.py --map maps/small.map --algorithm bfs

# Uniform-Cost Search
python main.py --map maps/small.map --algorithm ucs

# A* Search
python main.py --map maps/small.map --algorithm astar

# Hill Climbing
python main.py --map maps/small.map --algorithm hillclimb
2. Run Comparative Experiment
bash
# Compare all algorithms on medium map
python main.py --map maps/medium.map --experiment

# Compare on large map
python main.py --map maps/large.map --experiment
3. Demo Dynamic Obstacles
bash
# Show dynamic obstacle handling
python main.py --map maps/dynamic.map --demo
Advanced Usage
Custom Heuristics for A*
bash
# Use Manhattan distance (default)
python main.py --map maps/medium.map --algorithm astar

# Use Euclidean distance
# (Modify code in search_algorithms.py to change heuristic)
Custom Replanning Strategies
bash
# Use Hill Climbing for replanning (default)
python main.py --map maps/dynamic.map --algorithm astar

# Use different restart parameters
# (Modify max_restarts in agent.py)
🔬 Experimental Approach
1. Environment Modeling
Grid Representation: 2D matrix with integer cost cells

Terrain Costs: Variable movement costs (1 = road, 2 = grass, 3 = forest, etc.)

Obstacles: Static (permanent) and Dynamic (moving) obstacles

Agent Movement: 4-connected (up, down, left, right)

2. Algorithm Implementation
BFS: Frontier as queue, expands uniformly

UCS: Priority queue based on path cost

A*: Combines path cost and heuristic estimate

Hill Climbing: Local search with random restarts for replanning

3. Performance Metrics
Path Cost: Total movement cost to goal

Nodes Expanded: Search efficiency measure

Execution Time: Algorithm performance

Success Rate: Reliability in dynamic environments

4. Dynamic Obstacle Handling
Predictable Movement: Obstacles follow known patterns

Replanning Trigger: Agent detects blocked paths
Local Search: Quick adaptation using Hill Climbing

📊 Expected Output
Algorithm Comparison Example
================================================================================
Algorithm    Success Rate Avg Time (s) Avg Cost     Avg Nodes
================================================================================
bfs         100.0        0.0023       25.0         145
ucs         100.0        0.0056       18.0         89
astar       100.0        0.0011       18.0         23
hillclimb   85.0         0.0008       19.5         15
================================================================================

If --replanning is enabled, it will then simulate the agent's movement step-by-step and log when a dynamic obstacle appears and a replan is triggered.

Git and Reproducibility
It is highly encouraged to manage this source code using Git.

# Initialize a new git repository
git init
git add .
git commit -m "Initial commit of autonomous agent project"

The code is designed to be deterministic. Running the same command with the same inputs will always produce the identical output in terms of path, cost,
and nodes expanded. Runtimes may vary slightly due to system load.
