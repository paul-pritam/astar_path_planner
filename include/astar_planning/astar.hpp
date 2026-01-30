#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <memory>
#include <vector>
#include <cmath>
#include <functional>


namespace astar{

    struct GraphNode {
        int x;
        int y;
        double g_cost;
        double h_cost;
        double f_cost;
        std::shared_ptr<GraphNode> prev;

        GraphNode() : GraphNode(0,0){}
        GraphNode(int x_, int y_) : x(x_), y(y_),g_cost(0.0),h_cost(0.0),f_cost(0.0),prev(nullptr) {};

        GraphNode operator + (const std::pair<int, int> &dir) const {
            return GraphNode(x + dir.first, y + dir.second);
        }
        bool operator>(const GraphNode& other) const {
            return f_cost > other.f_cost;
        }
        bool operator<(const GraphNode& other) const {
            return f_cost < other.f_cost;
        }

    };

    class AStarPlanner : public rclcpp::Node {

        public:
            AStarPlanner();
        
        private:
            //subscriber
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
            //publisher
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;

            //tf2
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

            //for storing data
            nav_msgs::msg::OccupancyGrid::SharedPtr map_;
            nav_msgs::msg::OccupancyGrid visited_map_;

            void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
            void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_);

            nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose &start_pose,
                                     const geometry_msgs::msg::Pose &goal_pose);
            
            GraphNode worldToGrid(const geometry_msgs::msg::Pose &pose);
            geometry_msgs::msg::Pose gridToWorld(const GraphNode &node);
            bool poseOnMap (const GraphNode &node);
            unsigned int poseToCell(const GraphNode &node);
            
            //heuristic calculate
            double calHeuristic(const GraphNode &node, const GraphNode &goal);
    };

}

#endif