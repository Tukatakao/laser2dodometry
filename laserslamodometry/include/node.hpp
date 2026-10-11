#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <eigen3/Eigen/Dense>

class dingyuezhe : public rclcpp::Node
{
public:
    dingyuezhe();

private:
    void jisuanweizi(const sensor_msgs::msg::LaserScan::ConstSharedPtr& LEIDA);
    std::vector<Eigen::Vector2d> scan_to_points(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr leidajieshou;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fabulichengji;

    std::vector<Eigen::Vector2d> points1;                      // 上一帧点
    Eigen::Vector3d xi = Eigen::Vector3d::Zero();              // 帧间增量位姿 (dx,dy,dtheta)
    Eigen::Vector3d x = Eigen::Vector3d::Zero();               // 绝对位姿 (x,y,theta)
};
