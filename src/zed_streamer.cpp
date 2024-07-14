#include "zed-gst/zed_streamer.hpp"
#include "gst/gst.h"
#include "rclcpp_lifecycle/state.hpp"
#include "utsma_common/lifecycle_node.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

ZedStreamer::ZedStreamer()
    : utsma_common::LifecycleNode("zed_cam", "", 1000.0, false) {
  timer =
      this->create_wall_timer(500ms, std::bind(&ZedStreamer::callback, this));

  RCLCPP_DEBUG(this->get_logger(), "Streamer initialised.");
}

ZedStreamer::~ZedStreamer() {}

utsma_common::CallbackReturn
ZedStreamer::on_configure(const rclcpp_lifecycle::State &) {
  GError *error = new GError;
  if (gst_init_check(NULL, NULL, &error)) {
    return utsma_common::CallbackReturn::SUCCESS;
  } else {
    std::string message = "Failed to initialise gstreamer: ";
    message += error->message;
    return utsma_common::CallbackReturn::ERROR;
  }
};

utsma_common::CallbackReturn
ZedStreamer::on_activate(const rclcpp_lifecycle::State &) {
  return utsma_common::CallbackReturn::SUCCESS;
};

utsma_common::CallbackReturn
ZedStreamer::on_deactivate(const rclcpp_lifecycle::State &) {
  return utsma_common::CallbackReturn::SUCCESS;
};

utsma_common::CallbackReturn
ZedStreamer::on_cleanup(const rclcpp_lifecycle::State &) {
  return utsma_common::CallbackReturn::SUCCESS;
};

utsma_common::CallbackReturn
ZedStreamer::on_shutdown(const rclcpp_lifecycle::State &) {
  return utsma_common::CallbackReturn::SUCCESS;
};

void ZedStreamer::callback() { RCLCPP_INFO(this->get_logger(), "Callback."); }
