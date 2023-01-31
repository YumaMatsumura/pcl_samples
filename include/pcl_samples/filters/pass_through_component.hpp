#pragma once

#include <chrono>
#include <iostream>
#include <memory>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcl_filters
{

class PassThrough : public rclcpp::Node
{
public:
  explicit PassThrough(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
private:
  void timerCallback();
  void generatePointCloud();
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_raw_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_filtered_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  sensor_msgs::msg::PointCloud2::SharedPtr raw_data_;
  sensor_msgs::msg::PointCloud2::SharedPtr filtered_data_;
};

} // namespace pcl_filters
