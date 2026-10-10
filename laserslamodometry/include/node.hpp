#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>

class dingyuezhe:public rclcpp::Node
{
    public:
    dingyuezhe();
    private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr leidajieshou;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fabulichengji;
    void jisuanweizi(const sensor_msgs::msg::LaserScan::SharedPtr& LEIDA,const Eigen::Vector3d& xi);
    std::vector<Eigen::Vector2d> scan_to_points(const sensor_msgs::msg::LaserScan::SharedPtr& msg);
    std::vector<Eigen::Vector2d> points1;
    Eigen::Vector3d xi;
};