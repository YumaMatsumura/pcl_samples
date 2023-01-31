#include "pcl_samples/filters/pass_through_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  
  auto node = std::make_shared<pcl_filters::PassThrough>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  
  return 0;
}
