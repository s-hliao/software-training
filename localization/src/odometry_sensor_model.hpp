#ifndef ODOMETRY_SENSOR_MODEL_HPP
#define ODOMETRY_SENSOR_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_model.hpp"

namespace localization
{
class OdometrySensorModel : public SensorModel{ 
    private:
    nav_msgs::msg::Odometry last_msg_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    public:
    OdometrySensorModel(rclcpp::Node &node);
    void UpdateMeasurement(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    double ComputeLogProb(const Particle & particle) override;
    double ComputeLogNormalizer() override;
    bool IsMeasurementAvailable(const rclcpp::Time & cur_time) override;


};
}

#endif