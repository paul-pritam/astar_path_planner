#include "../include/astar_planning/astar.hpp"
#include <rmw/qos_profiles.h>
#include <queue>
#include <algorithm>

namespace astar{

    AStarPlanner::AStarPlanner() : Node("astar_planning_node") {

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        map_qos.reliable();

        map_qos.keep_last(1);

        //subscribes
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            map_qos,
            std::bind(&AStarPlanner::map_callback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10,
            std::bind(&AStarPlanner::goal_callback,this,std::placeholders::_1));
        
        //publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/astar/path",map_qos);
        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/astar/visited_map", map_qos);
    }

    void AStarPlanner::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map){
        map_ = map;
        visited_map_.header.frame_id = map->header.frame_id;
        visited_map_.info = map->info;
        visited_map_.data = std::vector<int8_t>(visited_map_.info.height*visited_map_.info.width, -1);
    }

    void AStarPlanner::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_){

        if (!map_){
            RCLCPP_ERROR(get_logger(), "No map received");
            return;
        }

        visited_map_.data = std::vector<int8_t>(visited_map_.info.height*visited_map_.info.width, -1);

        geometry_msgs::msg::TransformStamped map_to_base_tf;
        try{
            map_to_base_tf = tf_buffer_->lookupTransform(map_->header.frame_id,"base_link", tf2::TimePointZero);
        }catch(const tf2::TransformException &ex){
            RCLCPP_INFO(get_logger(), "couldnt transform map to base_link");
            return;
        }

        geometry_msgs::msg::Pose map_to_base_pose;
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

        auto path = plan(map_to_base_pose, pose_->pose);

        if (!path.poses.empty()){
            RCLCPP_INFO(this->get_logger(), "shortest path found");
            path_pub_->publish(path);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "no path to goal found");
        }
    }

    nav_msgs::msg::Path AStarPlanner::plan(const geometry_msgs::msg::Pose &start_pose, const geometry_msgs::msg::Pose &goal_pose){
        std::vector<std::pair<int,int>> explore_dir = {{-1,0}, {1,0}, {0,-1}, {0,1}};
        std::priority_queue<GraphNode, std::vector<GraphNode>,std::greater<GraphNode>> pending_nodes;
        std::vector<bool> visited_grid(map_->info.height*map_->info.width, false);

        GraphNode start_node = worldToGrid(start_pose);
        GraphNode goal_node = worldToGrid(goal_pose);
        
        //bound check
        if (!poseOnMap(start_node) || !poseOnMap(goal_node)){
            RCLCPP_ERROR(this->get_logger(), "start or goal is off the map");
            return nav_msgs::msg::Path();
        }

        // inmit start node
        start_node.g_cost = 0;
        start_node.h_cost = calHeuristic(start_node, goal_node);
        start_node.f_cost = start_node.g_cost + start_node.h_cost;

        pending_nodes.push(start_node);
        visited_grid[poseToCell(start_node)] = true;

        GraphNode active_node;
        bool found = false ;

        while (!pending_nodes.empty() && rclcpp::ok()){
            active_node = pending_nodes.top();
            pending_nodes.pop();

            //check goal
            if(active_node.x == goal_node.x && active_node.y == goal_node.y){
                found = true;
                break;
            }

            //explore neighbors
            for (const auto &dir : explore_dir){
                GraphNode new_node = active_node + dir;

                //bound check
                if (!poseOnMap(new_node)) continue;

                unsigned int idx = poseToCell(new_node);

                //check visited
                if(visited_grid[idx]) continue;
                //obstacle check
                int8_t val = map_->data[idx];

                //obstacles >=50 ==-1 ==100
                bool is_obstacle = (val == 100 || val == -1 || val >= 50);

                if (!is_obstacle){
                    new_node.g_cost = active_node.g_cost + 1;
                    new_node.h_cost = calHeuristic(new_node, goal_node);
                    new_node.f_cost = new_node.g_cost + new_node.h_cost;
                    new_node.prev = std::make_shared<GraphNode>(active_node);

                    pending_nodes.push(new_node);
                    visited_grid[idx] = true;

                    // visited map color
                    visited_map_.data[idx] = 10;//light blue  
                }
            }
        }

        //publish
        map_pub->publish(visited_map_);

        nav_msgs::msg::Path path;
        path.header.frame_id = map_->header.frame_id;
        path.header.stamp = this->now();

        if (found) {
            GraphNode *current_ptr = &active_node;
            while (current_ptr != nullptr && rclcpp::ok()){
                geometry_msgs::msg::Pose last_pose = gridToWorld(*current_ptr);
                geometry_msgs::msg::PoseStamped last_pose_stamped;
                last_pose_stamped.header.frame_id = map_->header.frame_id;
                last_pose_stamped.header.stamp = this->now();
                last_pose_stamped.pose = last_pose;
                path.poses.push_back(last_pose_stamped);

                if (current_ptr->prev){
                    current_ptr = current_ptr->prev.get();
                }
                else{
                    break;
                }
            }
            std::reverse(path.poses.begin(), path.poses.end());
        }
        return path;
    }

    double AStarPlanner::calHeuristic(const GraphNode &node, const GraphNode &goal){
        return static_cast<double>(std::abs(node.x - goal.x) + std::abs(node.y - goal.y));
    }

    GraphNode AStarPlanner::worldToGrid (const geometry_msgs::msg::Pose &pose){
        int grid_x = static_cast<int>((pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
        int grid_y = static_cast<int>((pose.position.y - map_->info.origin.position.y) / map_->info.resolution);       
        return GraphNode(grid_x, grid_y); 
    }

    bool AStarPlanner::poseOnMap (const GraphNode &node){
        return node.x < static_cast<int>(map_->info.width) && node.x >= 0 && 
               node.y < static_cast<int>(map_->info.height) && node.y >= 0;
    }

    unsigned int AStarPlanner::poseToCell(const GraphNode &node){
        return map_->info.width * node.y + node.x;
    }

    geometry_msgs::msg::Pose AStarPlanner::gridToWorld(const GraphNode &node){
        geometry_msgs::msg::Pose pose;
        pose.position.x = node.x * map_->info.resolution + map_->info.origin.position.x;
        pose.position.y = node.y * map_->info.resolution + map_->info.origin.position.y;
        return pose;
    }
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<astar::AStarPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}