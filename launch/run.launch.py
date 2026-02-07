import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Configuration
    # We assume your config file is in 'mybot'. Change if it's 'mybot_navigation'
    pkg_name = 'mybot_navigation' 
    yaml_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'costmap.yaml'
    )

    # 2. Lifecycle Manager
    # This manages the state of the costmap (Unconfigured -> Inactive -> Active)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        parameters=[
            {'use_sim_time': True}, 
            {'autostart': True},     
            {'node_names': ['costmap']} 
        ]
    )

    print(f"Loading YAML from: {yaml_file_path}")

    # 3. The Nav2 Costmap Node
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        output='screen',
        parameters=[
            yaml_file_path,
            {'use_sim_time': True} 
        ]
    )

    # 4. Your A* Planner Node
    astar_node = Node(
        package='astar_planning',
        executable='astar_planner', 
        name='astar_planner',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        lifecycle_manager,
        costmap_node,
        astar_node
    ])