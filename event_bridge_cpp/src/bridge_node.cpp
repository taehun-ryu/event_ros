#include <memory>

#include "rclcpp/rclcpp.hpp"

class EventBridgeCppNode : public rclcpp::Node {
public:
  EventBridgeCppNode() : Node("event_bridge_cpp_node") {
    this->declare_parameter<std::string>("camera_name", "event_camera");
    const auto camera_name = this->get_parameter("camera_name").as_string();
    RCLCPP_INFO(get_logger(), "event_bridge_cpp started for camera_name=%s", camera_name.c_str());
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EventBridgeCppNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
