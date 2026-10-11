#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <eigen3/Eigen/Dense>

#include "gaosiniudun.hpp"
#include "node.hpp"

dingyuezhe::dingyuezhe()
: Node("geiwoleidashuju")
{
    leidajieshou = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
            this->jisuanweizi(msg);
        });
    fabulichengji = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
}

void dingyuezhe::jisuanweizi(const sensor_msgs::msg::LaserScan::ConstSharedPtr& LEIDA)
{
    std::vector<Eigen::Vector2d> points2 = scan_to_points(LEIDA);

    if (points2.empty()) {
        return;                       // 这一帧没有有效点，直接跳过
    }

    if (points1.empty()) {
        points1 = points2;            // 第一帧只记录，不计算
        return;
    }

    xi.setZero();                     // 增量位姿的初始猜测
    gaosiniudundiedai(points1, points2, xi);
    points1 = points2;

    // 把增量 (dx,dy,dtheta) 叠加到绝对位姿 x=(x,y,theta)
    const double dx = xi(0);
    const double dy = xi(1);
    const double dtheta = xi(2);

    const double coss = std::cos(x(2));
    const double sinn = std::sin(x(2));

    x(0) += dx * coss - dy * sinn;
    x(1) += dx * sinn + dy * coss;
    x(2) += dtheta;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "cheti";

    msg.pose.pose.position.x = x(0);
    msg.pose.pose.position.y = x(1);
    msg.pose.pose.position.z = 0.0;

    // yaw 转四元数（roll=pitch=0）
    msg.pose.pose.orientation.z = std::sin(x(2) / 2.0);
    msg.pose.pose.orientation.w = std::cos(x(2) / 2.0);

    msg.pose.covariance.fill(0.0);
    msg.pose.covariance[0]  = 0.1;   // x
    msg.pose.covariance[7]  = 0.1;   // y
    msg.pose.covariance[35] = 0.2;   // yaw

    msg.twist.twist.linear.x = 0.0;
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.angular.z = 0.0;

    fabulichengji->publish(msg);
}

std::vector<Eigen::Vector2d> dingyuezhe::scan_to_points(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    std::vector<Eigen::Vector2d> points;
    points.reserve(msg->ranges.size());

    for (size_t i = 0; i < msg->ranges.size(); i++) {
        const double r = msg->ranges[i];
        // 跳过无效测量（inf / nan 表示无回波，或超出量程）
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
            continue;
        }
        const double theta = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
        points.emplace_back(r * std::cos(theta), r * std::sin(theta));
    }
    return points;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dingyuezhe>();
    RCLCPP_INFO(node->get_logger(), "dingyuezhe node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
