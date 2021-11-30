#include "odometry_sensor_model.hpp"
#include <cmath>
#include <vector>

namespace localization {
    OdometrySensorModel::OdometrySensorModel(rclcpp::Node &node){
        covariance_ = node.declare_parameter<std::vector<double>>("sensors/odom/covariance", {.1, .1});
        timeout_ = node.declare_parameter<double>("sensors/odom/measurement_timeout", .1);

        odom_sub_ = rclcpp::create_subscription<nav_msgs::msg::Odometry>(node, "/odom", 
            rclcpp::SystemDefaultsQoS(), std::bind(&OdometrySensorModel::UpdateMeasurement, this, std::placeholders::_1));
    }

    void OdometrySensorModel::UpdateMeasurement(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_msg_ = *msg;
    }

    bool OdometrySensorModel::IsMeasurementAvailable(const rclcpp::Time &current_time){
        if(last_msg_.header.stamp.sec == 0){
            return false;
        }
        const auto time_since_last_msg = current_time - rclcpp::Time(last_msg_.header.stamp);

        return time_since_last_msg.seconds() < timeout_;
    }

    double OdometrySensorModel::ComputeLogProb(const Particle & particle){
        double log_prob = 0.0;
        log_prob += pow(last_msg_.twist.twist.linear.x - particle.x_vel, 2) / covariance_[0];
        log_prob += pow(last_msg_.twist.twist.angular.z - particle.yaw_vel, 2) / covariance_[1];

        return log_prob;
    }

    double OdometrySensorModel::ComputeLogNormalizer(){
        return log(sqrt(pow(2 * M_PI, 2))) +
            log(sqrt(covariance_[0])) + log(sqrt(covariance_[1]));
    }


}