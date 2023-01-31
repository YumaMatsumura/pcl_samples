#include "pcl_samples/filters/voxel_grid_component.hpp"

namespace pcl_filters
{
VoxelGrid::VoxelGrid(
  const rclcpp::NodeOptions& options)
: Node("voxel_grid_node", options),
  raw_data_(new sensor_msgs::msg::PointCloud2),
  filtered_data_(new sensor_msgs::msg::PointCloud2)
{
  using namespace std::chrono_literals;

  generatePointCloud();
  
  pub_pcl_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sample_pcl/raw_data", 10);
  pub_pcl_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sample_pcl/filter_data", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&VoxelGrid::timerCallback, this));
}

void VoxelGrid::timerCallback()
{
  pub_pcl_raw_->publish(*raw_data_);
  pub_pcl_filtered_->publish(*filtered_data_);
}

void VoxelGrid::generatePointCloud()
{
  sensor_msgs::msg::PointCloud2::SharedPtr raw_data(new sensor_msgs::msg::PointCloud2);
  sensor_msgs::msg::PointCloud2::SharedPtr filtered_data(new sensor_msgs::msg::PointCloud2);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  
  cloud->width = 10000;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  
  for(auto& point: *cloud){
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }
  
  
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*cloud_filtered);
  
  pcl::toROSMsg(*cloud, *raw_data_);
  pcl::toROSMsg(*cloud_filtered, *filtered_data_);
  
  raw_data_->header.frame_id = "map";
  filtered_data_->header.frame_id = "map";
}


} // namespace pcl_filters

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_filters::VoxelGrid)
