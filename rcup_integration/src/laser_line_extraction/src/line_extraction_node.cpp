#include "laser_line_extraction/line_extraction_ros.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  RCLCPP_DEBUG(rclcpp::get_logger("line_extraction_node"), 
               "Starting line_extraction_node.");

  auto node = std::make_shared<line_extraction::LineExtractionROS>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}