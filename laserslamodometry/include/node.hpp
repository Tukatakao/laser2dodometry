#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

class dingyuezhe : public rclcpp::Node
{
public:
    dingyuezhe();

private:
    void jisuanweizi(const sensor_msgs::msg::LaserScan::ConstSharedPtr& LEIDA);
    std::vector<Eigen::Vector2d> scan_to_points(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
    void wheel_odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr leidajieshou;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fabulichengji;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr wheel_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr laser_path_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::msg::Path wheel_path_;   // 轮式里程计轨迹
    nav_msgs::msg::Path laser_path_;   // 2D 激光里程计轨迹

    std::vector<Eigen::Vector2d> points1;                      // 上一帧点
    Eigen::Vector3d xi = Eigen::Vector3d::Zero();              // 帧间增量位姿 (dx,dy,dtheta)
    Eigen::Vector3d x = Eigen::Vector3d::Zero();               // 绝对位姿 (x,y,theta)
};
