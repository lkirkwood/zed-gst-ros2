#ifndef ZED_STREAMER_H_
#define ZED_STREAMER_H_

#include "logging/ros_logger.hpp"
#include "utsma_common/lifecycle_node.hpp"

class ZedStreamer : public utsma_common::LifecycleNode {
public:
  const std::string node_name = "zed-cam";

  ZedStreamer() : LifecycleNode(node_name){};
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

private:
  bool exit_requested_{false};
  bool is_active_{false};

  std::shared_ptr<ROSLogger> logger_;
  rclcpp::TimerBase::SharedPtr process_timer_;

  void process();
  void create_publishers();
};

#endif // ZED_STREAMER_H_
