# A* Planner

A ROS 2 package implementing the A* (A-star) pathfinding algorithm for autonomous robot navigation in a 2D grid-based environment.

## Overview

The `astar_planning` package provides a fast and efficient pathfinding solution using the A* algorithm. It subscribes to occupancy grid maps, receives goal poses, and computes optimal paths from the robot's current position to the goal. This package is ideal for autonomous navigation systems in ROS 2.

## Features

- **A* Algorithm Implementation**: Efficient pathfinding using heuristic-guided search
- **ROS 2 Integration**: Native ROS 2 support with proper message handling and TF transforms
- **Occupancy Grid Support**: Works with standard ROS 2 occupancy grid maps
- **Path Publishing**: Publishes computed paths for robot navigation
- **Visited Map Visualization**: Publishes explored nodes for visualization and debugging
- **Transform Support**: Uses TF2 for coordinate frame transformations between map and base_link

## Dependencies

- ROS 2 (tested with latest distributions)
- C++17 or higher
- Standard ROS 2 packages:
  - `rclcpp` - ROS 2 C++ client library
  - `nav_msgs` - Navigation message definitions
  - `geometry_msgs` - Geometry message definitions
  - `tf2_ros` - TF2 transform support

## Installation

1. Clone this package into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> astar_planning
   ```

2. Install dependencies using rosdep:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the package:
   ```bash
   colcon build --packages-select astar_planning
   ```

4. Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Usage

### Running the Node

Launch the A* planning node:

```bash
ros2 run astar_planning astar_planner --ros-args -p use_sim_time:=true
```

The `use_sim_time:=true` parameter is useful when running with simulated time in ROS 2.

### Topics

#### Subscriptions

- **`/map`** (`nav_msgs/msg/OccupancyGrid`)
  - The occupancy grid map of the environment
  - Latched topic (transient local with keep last = 1)

- **`/goal_pose`** (`geometry_msgs/msg/PoseStamped`)
  - The target goal pose for path planning
  - Published when a new path needs to be computed

#### Publications

- **`/astar/path`** (`nav_msgs/msg/Path`)
  - The computed path from start to goal
  - Published after successful planning
  - Empty message if no path is found

- **`/astar/visited_map`** (`nav_msgs/msg/OccupancyGrid`)
  - Visualization of explored nodes during A* search
  - Useful for debugging and understanding the search pattern
  - Same frame and dimensions as the input map

### Required Transforms

- **`map` → `base_link`**: Transform from the map frame to the robot's base frame is required to determine the robot's starting position for pathfinding.

## Algorithm Details

### A* Algorithm

The A* algorithm is a graph search algorithm that combines the benefits of Dijkstra's algorithm and greedy best-first search:

- **g-cost**: The actual distance from the start node to the current node
- **h-cost**: The heuristic estimate of the distance from the current node to the goal (usually Euclidean or Manhattan distance)
- **f-cost**: The sum of g-cost and h-cost (f = g + h)

The algorithm explores nodes in order of lowest f-cost, ensuring optimality while being more efficient than Dijkstra's algorithm.

### Grid Resolution

The planner works on a grid representation where each cell represents a discrete position in the map. The resolution is inherited from the input occupancy grid.

### Movement

The planner supports 4-directional movement (up, down, left, right) through neighboring grid cells.

## Building from Source

```bash
cd ~/ros2_ws
colcon build --packages-select astar_planning
```

## Example Workflow

1. Launch your robot simulation with a map:
   ```bash
   ros2 launch mybot map.launch.py
   ```

2. In another terminal, run the A* planner:
   ```bash
   ros2 run astar_planning astar_planner --ros-args -p use_sim_time:=true
   ```

3. Publish a goal pose using RViz or a custom node:
   ```bash
   ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}'
   ```

4. View the computed path in RViz by adding the `/astar/path` topic to your visualization.

## Architecture

### Main Components

- **AStarPlanner Node** (`astar.cpp`)
  - Main ROS 2 node implementation
  - Handles subscriptions and publishers
  - Coordinates the planning process

- **GraphNode Structure** (`astar.hpp`)
  - Represents a single node in the search graph
  - Stores position (x, y) and cost information (g, h, f)
  - Maintains parent node reference for path reconstruction

### Key Methods

- `plan()`: Main planning function that executes the A* algorithm
- `map_callback()`: Processes incoming occupancy grid maps
- `goal_callback()`: Triggers planning when a new goal is received
- `worldToGrid()`: Converts world coordinates to grid indices
- `calHeuristic()`: Computes heuristic distance estimate

## Troubleshooting

### "No map received" error
- Ensure the `/map` topic is being published
- Check the topic name matches the subscription

### "couldnt transform map to base_link" error
- Verify the TF2 transform between `map` and `base_link` is being published
- Check your robot's TF2 publisher configuration

### No path found
- Verify the goal is within the map boundaries
- Check if the goal is in a free space (not in an occupied region)
- Visualize the costmap in RViz to debug

## Performance Considerations

- The A* algorithm's performance depends on the heuristic function quality
- Larger maps and complex environments may require longer computation times
- 4-directional movement is efficient but may not find the shortest Euclidean paths

## License

This package is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.

## Author

Maintained by the robotics development team.

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests to improve the package.

## References

- A* Algorithm: https://en.wikipedia.org/wiki/A*_search_algorithm
- ROS 2 Navigation: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Navigation.html
- ROS 2 Documentation: https://docs.ros.org/
