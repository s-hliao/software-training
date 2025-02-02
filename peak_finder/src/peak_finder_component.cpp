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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stsl_interfaces/action/park_at_peak.hpp>
#include <stsl_interfaces/srv/sample_elevation.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <memory>
#include <string>
#include <vector>
#include "navigator.hpp"

namespace peak_finder
{
class PeakFinderComponent : public rclcpp::Node
{
public:
  using ParkAtPeak = stsl_interfaces::action::ParkAtPeak;
  using ParkAtPeakGoalHandle = rclcpp_action::ServerGoalHandle<ParkAtPeak>;

  explicit PeakFinderComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("peak_finder", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    navigator_(*this)
  {
    declare_parameter<double>("search_radius", 0.1);
    declare_parameter<int>("sample_count", 8);

    action_server_ = rclcpp_action::create_server<ParkAtPeak>(
      this, "park_at_peak",
      std::bind(
        &PeakFinderComponent::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&PeakFinderComponent::handle_cancel, this, std::placeholders::_1),
      std::bind(&PeakFinderComponent::handle_accepted, this, std::placeholders::_1));

    elevation_client_ = create_client<stsl_interfaces::srv::SampleElevation>("/sample_elevation");
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Server<ParkAtPeak>::SharedPtr action_server_;
  rclcpp::Client<stsl_interfaces::srv::SampleElevation>::SharedPtr elevation_client_;
  Navigator navigator_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const ParkAtPeak::Goal>/*goal*/)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<ParkAtPeakGoalHandle> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<ParkAtPeakGoalHandle> goal_handle)
  {
    std::thread{std::bind(&PeakFinderComponent::execute, this, std::placeholders::_1),
      goal_handle}
    .detach();
  }

  void execute(const std::shared_ptr<ParkAtPeakGoalHandle> goal_handle)
  {
    if (!elevation_client_->service_is_ready()) {
      RCLCPP_ERROR(
        get_logger(), "%s service must be available to run peak_finder action!",
        elevation_client_->get_service_name());
      goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
      return;
    }

    std::string tf_error_msg;
    if (!tf_buffer_.canTransform("map", "base_footprint", tf2::TimePointZero, &tf_error_msg)) {
      RCLCPP_ERROR(
        get_logger(), "Robot position could not be looked up via TF. Error: %s",
        tf_error_msg.c_str());
      goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
      return;
    }

    if (!navigator_.server_available()) {
      RCLCPP_ERROR(
        get_logger(),
        "/navigate_to_point action must be available to run peak_finder action!");
      goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
      return;
    }

    try {
      while (rclcpp::ok() && !goal_handle->is_canceling()) {
        const auto robot_position = GetRobotPosition();

        const double current_elevation = SampleElevation(robot_position);

        double search_radius;
        get_parameter("search_radius", search_radius);
        int sample_count;
        get_parameter("sample_count", sample_count);

        auto generate_next_pose =
          [&robot_position, search_radius, sample_count, angle = 0.0]() mutable {
            Eigen::Vector2d pose =
              (search_radius * Eigen::Vector2d(std::cos(angle), std::sin(angle))) + robot_position;
            angle += (2 * M_PI) / sample_count;
            return pose;
          };

        std::vector<Eigen::Vector2d> sample_positions;

        std::generate_n(std::back_inserter(sample_positions), sample_count, generate_next_pose);

        std::vector<double> elevations;

        std::transform(
          sample_positions.begin(), sample_positions.end(),
          std::back_inserter(elevations), [this](const auto & p) {return SampleElevation(p);});

        const auto max_elevation_iter = std::max_element(elevations.begin(), elevations.end());

        if (*max_elevation_iter <= current_elevation) {
          RCLCPP_INFO(get_logger(), "At peak!");
          goal_handle->succeed(std::make_shared<ParkAtPeak::Result>());
          return;
        }

        const auto goal_position =
          sample_positions[std::distance(elevations.begin(), max_elevation_iter)];

        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = now();
        goal_pose.pose.position.x = goal_position.x();
        goal_pose.pose.position.y = goal_position.y();

        if (!navigator_.go_to_pose(goal_pose)) {
          RCLCPP_ERROR(get_logger(), "Navigation server rejected request");
          goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
          return;
        }

        while (!navigator_.wait_for_completion(std::chrono::milliseconds(100))) {
          if (!rclcpp::ok() || goal_handle->is_canceling()) {
            navigator_.cancel();
            goal_handle->canceled(std::make_shared<ParkAtPeak::Result>());
            return;
          }
        }

        if (!navigator_.succeeded()) {
          RCLCPP_ERROR(get_logger(), "Navigation failed!");
          goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
          return;
        }
      }
      goal_handle->canceled(std::make_shared<ParkAtPeak::Result>());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
    }
  }

  Eigen::Vector2d GetRobotPosition()
  {
    const auto robot_pose_transform =
      tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
    Eigen::Isometry3d robot_pose_3d = tf2::transformToEigen(robot_pose_transform);
    return robot_pose_3d.translation().head<2>();
  }

  double SampleElevation(const Eigen::Vector2d & position)
  {
    auto sample_request = std::make_shared<stsl_interfaces::srv::SampleElevation::Request>();
    sample_request->x = position.x();
    sample_request->y = position.y();
    const auto result_future = elevation_client_->async_send_request(sample_request);

    if (result_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      throw std::runtime_error("Elevation service call timed out.");
    }
    const auto response = result_future.get();

    if (!response->success) {
      throw std::runtime_error("Elevation server reported failure.");
    }

    return response->elevation;
  }
};

}  // namespace peak_finder

RCLCPP_COMPONENTS_REGISTER_NODE(peak_finder::PeakFinderComponent)
