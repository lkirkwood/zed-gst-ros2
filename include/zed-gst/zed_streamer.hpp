#ifndef ZED_STREAMER_H_
#define ZED_STREAMER_H_

#include "utsma_common/lifecycle_node.hpp"
#include <gst/gstbus.h>
#include <rclcpp/timer.hpp>

class ZedStreamer : public utsma_common::LifecycleNode {
public:
  ZedStreamer();
  ~ZedStreamer();

  utsma_common::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state);

  utsma_common::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state);

  utsma_common::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state);

  utsma_common::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);

  utsma_common::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state);

  void stream();

  GMainLoop *loop;
};

#endif // ZED_STREAMER_H_
