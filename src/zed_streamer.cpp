#include "zed-gst/zed_streamer.hpp"
#include "utsma_common/lifecycle_node.hpp"
#include <rclcpp_lifecycle/state.hpp>

ZedStreamer::ZedStreamer()
    : utsma_common::LifecycleNode("zed-cam", "", 1000.0, true), active(false),
      logger(std::make_shared<ROSLogger>(2000000000ULL)) {
  logger->log_debug("zed-cam initialised.");
}

utsma_common::CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
  return utsma_common::CallbackReturn::SUCCESS;
};

utsma_common::CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
  return utsma_common::CallbackReturn::SUCCESS;
};

utsma_common::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
  return utsma_common::CallbackReturn::SUCCESS;
};

utsma_common::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) {
  return utsma_common::CallbackReturn::SUCCESS;
};

utsma_common::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) {
  return utsma_common::CallbackReturn::SUCCESS;
};

void process(){};
void create_publishers(){};
