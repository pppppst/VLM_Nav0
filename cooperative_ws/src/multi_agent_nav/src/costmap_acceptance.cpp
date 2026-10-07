// Humble Costmap2DROS has no registered component. Host only native costmaps.
#include <memory>
#include <string>
#include <vector>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<std::shared_ptr<nav2_costmap_2d::Costmap2DROS>> nodes;
  for (const std::string name : {"local_costmap", "global_costmap"}) {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=" + name,
      "-r", "__ns:=/go2_costmap_acceptance/" + name,
      "-r", "tf:=/tf", "-r", "tf_static:=/tf_static"});
    auto node = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    executor.add_node(node->get_node_base_interface());
    nodes.push_back(node);
  }
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
