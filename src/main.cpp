#include "zed-gst/zed_streamer.hpp"
#include <rclcpp/utilities.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin((new ZedStreamer)->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
