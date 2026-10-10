#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include "gaosiniudun.hpp"
#include "node.hpp"
dingyuezhe::dingyuezhe():Node("geiwoleidashuju")
{
    leidajieshou = this ->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,std::bind(dingyuezhe::jisuanweizi,this,std::placeholders::_1));
    fabulichengji = this -> create_publisher<nav_msgs::msg::Odometry>("/odom",10);
    
};
void dingyuezhe::jisuanweizi(const sensor_msgs::msg::LaserScan::SharedPtr& LEIDA,const Eigen::Vector3d& xi)
{   
    
    std::vector<Eigen::Vector2d> points2 = scan_to_points(LEIDA);
    if(points1.empty())
    {
        points1=points2;
        return ;
    }
    gaosiniudundiedai(points1,points2,xi);
    points1 = points2;
    return ;

}
std::vector<Eigen::Vector2d> dingyuezhe::scan_to_points(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
{
    std::vector<Eigen::Vector2d> points;
    for(int i =0 ;i < msg->ranges.size();i++)
    {
        double r = msg->ranges[i];
        double theta = msg->angle_min + (i*msg->angle_increment);
        double x = r*cos(theta);
        double y = r*sin(theta);
        points.emplace_back(Eigen::Vector2d(x,y));
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
