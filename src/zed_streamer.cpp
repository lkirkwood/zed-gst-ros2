#include "zed-gst/zed_streamer.hpp"
#include "gst/gst.h"
#include "rclcpp_lifecycle/state.hpp"
#include "utsma_common/lifecycle_node.hpp"
#include <gst/gstbin.h>
#include <gst/gstbus.h>
#include <gst/gstelement.h>
#include <gst/gstelementfactory.h>
#include <gst/gstmessage.h>
#include <gst/gstobject.h>
#include <gst/gstparse.h>
#include <gst/gstpipeline.h>
#include <gst/gstutils.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

ZedStreamer::ZedStreamer()
    : utsma_common::LifecycleNode("zed_cam", "", 1000.0, false) {
  RCLCPP_DEBUG(this->get_logger(), "Streamer initialised.");
}

ZedStreamer::~ZedStreamer() {}

utsma_common::CallbackReturn
ZedStreamer::on_configure(const rclcpp_lifecycle::State &) {
  return utsma_common::CallbackReturn::SUCCESS;
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

static int gst_bus_call(GstBus *, GstMessage *message, gpointer data) {
  ZedStreamer *streamer = (ZedStreamer *)data;
  switch (GST_MESSAGE_TYPE(message)) {

  case GST_MESSAGE_EOS:
    RCLCPP_INFO(streamer->get_logger(), "End of stream reached.");
    g_main_loop_quit(streamer->loop);
    break;

  case GST_MESSAGE_ERROR: {
    gchar *debug;
    GError *error;

    gst_message_parse_error(message, &error, &debug);
    g_free(debug);

    RCLCPP_ERROR(streamer->get_logger(), "Error from gstreamer pipeline: %s",
                 error->message);
    g_error_free(error);

    g_main_loop_quit(streamer->loop);
    break;
  }
  default:
    break;
  }

  return 0;
};

void ZedStreamer::stream() {
  RCLCPP_INFO(this->get_logger(), "Starting gstreamer pipeline...");

  GError *error = new GError;
  if (!gst_init_check(NULL, NULL, &error)) {
    std::string message = "Failed to initialise gstreamer: ";
    message += error->message;
    RCLCPP_FATAL(this->get_logger(), "%s", message.c_str());
    return;
  }

  auto pipeline = gst_pipeline_new("depth");
  if (!pipeline) {
    std::string message = "Failed to launch gstreamer pipeline: ";
    message += error->message;
    RCLCPP_FATAL(this->get_logger(), "%s", message.c_str());
    return;
  }

  auto source = gst_element_factory_make("videotestsrc", "source");
  auto sink = gst_element_factory_make("autovideosink", "sink");
  gst_bin_add_many(GST_BIN(pipeline), source, sink, NULL);

  if (!gst_element_link_many(source, sink, NULL)) {
    RCLCPP_FATAL(this->get_logger(),
                 "Failed to link gstreamer pipeline elements.");
    return;
  }

  loop = g_main_loop_new(NULL, FALSE);
  GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));

  auto bus_watch_id = gst_bus_add_watch(bus, &gst_bus_call, this);
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  RCLCPP_INFO(this->get_logger(), "Starting main pipeline loop.");
  g_main_loop_run(loop);

  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
  g_source_remove(bus_watch_id);
  g_main_loop_unref(loop);
}
