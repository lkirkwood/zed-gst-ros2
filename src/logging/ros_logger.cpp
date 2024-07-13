#include "logging/ros_logger.hpp"

/*
 * Copyright (C) 2015-2023 Swift Navigation Inc.
 * Contact: https://support.swiftnav.com
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "logging/ros_logger.hpp"

void ROSLogger::log_debug(const std::string_view ss) {
  if (can_log(ss)) {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), ss.data());
    update_log_status(ss);
  }
}

void ROSLogger::log_info(const std::string_view ss) {
  if (can_log(ss)) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.data());
    update_log_status(ss);
  }
}

void ROSLogger::log_warning(const std::string_view ss) {
  if (can_log(ss)) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), ss.data());
    update_log_status(ss);
  }
}

void ROSLogger::log_error(const std::string_view ss) {
  if (can_log(ss)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), ss.data());
    update_log_status(ss);
  }
}

void ROSLogger::log_fatal(const std::string_view ss) {
  if (can_log(ss)) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), ss.data());
    update_log_status(ss);
  }
}

bool ROSLogger::can_log(const std::string_view output_str) const {
  if (output_str == last_output_str_) {
    const auto now =
        std::chrono::time_point<std::chrono::system_clock>::clock::now();
    return ((now - last_output_time_).count() >= timeout_);
  } else {
    return true;
  }
}

void ROSLogger::update_log_status(const std::string_view output_str) {
  last_output_str_ = output_str;
  last_output_time_ =
      std::chrono::time_point<std::chrono::system_clock>::clock::now();
}

ROSLogger::ROSLogger(const int64_t log_delay) : timeout_(log_delay) {}
