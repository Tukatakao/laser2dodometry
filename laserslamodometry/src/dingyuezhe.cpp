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
    fabulichengji = this->create_publisher<nav_msgs::msg::Odometry>("/odom1", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 订阅轮式里程计(/odom)，发布两条轨迹
    wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
            this->wheel_odom_cb(msg);
        });
    wheel_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/wheel_path", 10);
    laser_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path_2d", 10);
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

    // 广播 odom -> cheti 的 TF，让 RViz 能看到小车跟着里程计动
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg.header.stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "cheti";
    t.transform.translation.x = x(0);
    t.transform.translation.y = x(1);
    t.transform.translation.z = 0.0;
    t.transform.rotation.z = std::sin(x(2) / 2.0);
    t.transform.rotation.w = std::cos(x(2) / 2.0);
    tf_broadcaster_->sendTransform(t);

    // 记录 2D 激光里程计轨迹
    geometry_msgs::msg::PoseStamped laser_pose;
    laser_pose.header.stamp = msg.header.stamp;
    laser_pose.header.frame_id = "odom";
    laser_pose.pose = msg.pose.pose;

    laser_path_.header.stamp = msg.header.stamp;
    laser_path_.header.frame_id = "odom";
    laser_path_.poses.push_back(laser_pose);
    laser_path_pub_->publish(laser_path_);
}

void dingyuezhe::wheel_odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    // 把轮式里程计的每一帧位姿追加到 /wheel_path
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = msg->header.frame_id;
    pose.pose = msg->pose.pose;

    wheel_path_.header.stamp = msg->header.stamp;
    wheel_path_.header.frame_id = msg->header.frame_id;
    wheel_path_.poses.push_back(pose);
    wheel_path_pub_->publish(wheel_path_);
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
