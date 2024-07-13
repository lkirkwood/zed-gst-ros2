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

#ifndef UTSMA_GPS__LOGGING__ISSUE_LOGGER_HPP_
#define UTSMA_GPS__LOGGING__ISSUE_LOGGER_HPP_

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string_view>
#include <vector>

/**
 * @brief Abstract base class for a logging facility
 */
class IIssueLogger {
 public:
  /**
   * @brief Method used to log Debug information
   *
   * @param ss String Stream containing the data to log
   */
  virtual void log_debug(const std::string_view ss) = 0;

  /**
   * @brief Method used to log general information
   *
   * @param ss String Stream containing the data to log
   */
  virtual void log_info(const std::string_view ss) = 0;

  /**
   * @brief Method used to log Warnings
   *
   * @param ss String Stream containing the data to log
   */
  virtual void log_warning(const std::string_view ss) = 0;

  /**
   * @brief Method used to log Errors
   *
   * @param ss String Stream containing the data to log
   */
  virtual void log_error(const std::string_view ss) = 0;

  /**
   * @brief Method used to log fatal conditions or events
   *
   * @param ss String Stream containing the data to log
   */
  virtual void log_fatal(const std::string_view ss) = 0;
};

/**
 * @brief Fantasy name for a shared pointer object to a logger
 */
using LoggerPtr = std::shared_ptr<IIssueLogger>;

// Logging macros
inline void LOG_FUNC(const LoggerPtr& logger, const int level, const char* format, ...) {
  va_list args;

  if (!logger) return;

  char buffer[1024];
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer) - 1, format, args);
  va_end(args);

  switch (level) {
    case 0:
      logger->log_debug(buffer);
      break;

    case 1:
      logger->log_info(buffer);
      break;

    case 2:
      logger->log_warning(buffer);
      break;

    case 3:
      logger->log_error(buffer);
      break;

    case 4:
      logger->log_fatal(buffer);
      break;

    default:
      break;
  }
}

#define LOG_DEBUG(logger, ...) LOG_FUNC(logger, 0, __VA_ARGS__)
#define LOG_INFO(logger, ...) LOG_FUNC(logger, 1, __VA_ARGS__)
#define LOG_WARN(logger, ...) LOG_FUNC(logger, 2, __VA_ARGS__)
#define LOG_ERROR(logger, ...) LOG_FUNC(logger, 3, __VA_ARGS__)
#define LOG_FATAL(logger, ...) LOG_FUNC(logger, 4, __VA_ARGS__)

// Contract checking macros
#define ASSERT_COND(cond, logger, str) \
  do {                                 \
    if (!(cond)) {                     \
      LOG_FATAL(logger, str);          \
      exit(1);                         \
    }                                  \
  } while (0)

#endif
