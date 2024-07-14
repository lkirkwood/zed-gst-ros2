#include "rclcpp/rclcpp.hpp"
#include "zed-gst/zed_streamer.hpp"
#include <rclcpp/utilities.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  ZedStreamer().stream();
  rclcpp::shutdown();
  return 0;
}
