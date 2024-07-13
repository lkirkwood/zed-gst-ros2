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

#ifndef ZED_GST__LOGGING__ROS_LOGGER_HPP_
#define ZED_GST__LOGGING__ROS_LOGGER_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>

#include "logging/issue_logger.hpp"
#include "rclcpp/rclcpp.hpp"

class ROSLogger : public IIssueLogger {
public:
  ROSLogger() = delete;
  explicit ROSLogger(const int64_t log_delay);
  void log_debug(const std::string_view ss) override;
  void log_info(const std::string_view ss) override;
  void log_warning(const std::string_view ss) override;
  void log_error(const std::string_view ss) override;
  void log_fatal(const std::string_view ss) override;

private:
  /**
   * @brief Method to determine if we could output a log to ROS or not
   *
   * @param output_str String to output
   * @return true We are allowed to output the string
   * @return false We're not allowed to output the string
   */
  bool can_log(const std::string_view output_str) const;

  /**
   * @brief This method updates the information about the last message sent to
   * ROS log
   *
   * @param output_str Las string sent to ROS
   */
  void update_log_status(const std::string_view output_str);

  std::string last_output_str_; /** @brief Last string sent to ROS */
  std::chrono::time_point<std::chrono::system_clock>
      last_output_time_; /** @brief Time of the last message sent to ROS */
  int64_t timeout_{0LL}; /** @brief Time that must pass before we're allowed to
                            send the same string */
};

#endif
