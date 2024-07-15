#ifndef ZED_STREAMER_H_
#define ZED_STREAMER_H_

#include "utsma_common/lifecycle_node.hpp"
#include <gst/gstbus.h>
#include <gst/gstelement.h>
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

  void poll_stream();
  int start_stream();
  void stop_stream();

private:
  rclcpp::TimerBase::SharedPtr timer;
  GstElement *pipeline;
  guint bus_watch_id;
  bool initialised;
  bool active;

  int init_gst();
};

#endif // ZED_STREAMER_H_
