// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "astar_path_planner.hpp"
#include <nav2_core/global_planner.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <memory>
#include <string>
#include <vector>
#include "path_reduction.hpp"

namespace astar_path_planner
{

geometry_msgs::msg::PoseStamped pointToMessage(const Point & point)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = point.x();
  pose.pose.position.y = point.y();
  return pose;
}

class AStarPathPlannerPlugin : public nav2_core::GlobalPlanner
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = node;
    global_frame_ = costmap_ros->getGlobalFrameID();
    costmap_ros_ = costmap_ros;
    AStarPathPlanner::DeclareParameters(node);
  }

  void cleanup() override {}

  void activate() override {}

  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = global_frame_;

    if (start.header.frame_id != global_frame_) {
      RCLCPP_ERROR(
        node_->get_logger(), "Planner will only except start position from %s frame",
        global_frame_.c_str());
      return path;
    }

    if (goal.header.frame_id != global_frame_) {
      RCLCPP_INFO(
        node_->get_logger(), "Planner will only except goal position from %s frame",
        global_frame_.c_str());
      return path;
    }

    Point start_point{start.pose.position.x, start.pose.position.y};
    Point goal_point{goal.pose.position.x, goal.pose.position.y};

    AStarPathPlanner planner(node_, costmap_ros_);

    const auto point_path = planner.Plan(start_point, goal_point);

    std::vector<Point> reduced_point_path(point_path);
    ReducePath(reduced_point_path);

    RCLCPP_INFO(node_->get_logger(), "Calculated path with %d points.", reduced_point_path.size());

    std::transform(
      reduced_point_path.begin(), reduced_point_path.end(),
      std::back_inserter(path.poses), &pointToMessage);

    return path;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string global_frame_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

}  // namespace astar_path_planner

PLUGINLIB_EXPORT_CLASS(astar_path_planner::AStarPathPlannerPlugin, nav2_core::GlobalPlanner)
