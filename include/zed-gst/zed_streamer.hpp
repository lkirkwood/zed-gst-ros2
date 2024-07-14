#ifndef ZED_STREAMER_H_
#define ZED_STREAMER_H_

#include "utsma_common/lifecycle_node.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>

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

  void callback();

private:
  rclcpp::TimerBase::SharedPtr timer;

  void process();
  void create_publishers();
};

#endif // ZED_STREAMER_H_
