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
#include <initializer_list>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

ZedStreamer::ZedStreamer()
    : utsma_common::LifecycleNode("zed_cam", "", 1000.0, false) {
  RCLCPP_DEBUG(this->get_logger(), "Streamer initialised.");
  pipeline = nullptr;
  bus_watch_id = 0;
  active = false;

  if (this->init_gst()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialised gstreamer.");
    initialised = false;
  } else {
    initialised = true;
  }

  timer =
      this->create_wall_timer(1s, std::bind(&ZedStreamer::poll_stream, this));
}

ZedStreamer::~ZedStreamer() {}

utsma_common::CallbackReturn
ZedStreamer::on_configure(const rclcpp_lifecycle::State &) {
  if (this->initialised) {
    return utsma_common::CallbackReturn::SUCCESS;
  } else {
    return utsma_common::CallbackReturn::ERROR;
  }
};

utsma_common::CallbackReturn
ZedStreamer::on_activate(const rclcpp_lifecycle::State &) {
  if (this->initialised) {
    if (this->start_stream()) {
      return utsma_common::CallbackReturn::ERROR;
    } else {
      return utsma_common::CallbackReturn::SUCCESS;
    }
  } else {
    return utsma_common::CallbackReturn::FAILURE;
  }
};

utsma_common::CallbackReturn
ZedStreamer::on_deactivate(const rclcpp_lifecycle::State &) {
  if (this->initialised) {
    this->stop_stream();
  }
  return utsma_common::CallbackReturn::SUCCESS;
};

utsma_common::CallbackReturn
ZedStreamer::on_cleanup(const rclcpp_lifecycle::State &) {
  return utsma_common::CallbackReturn::SUCCESS;
};

utsma_common::CallbackReturn
ZedStreamer::on_shutdown(const rclcpp_lifecycle::State &) {
  this->shutdown();
  return utsma_common::CallbackReturn::SUCCESS;
};

static int gst_bus_call(GstBus *, GstMessage *message, gpointer data) {
  ZedStreamer *streamer = (ZedStreamer *)data;
  switch (GST_MESSAGE_TYPE(message)) {

  case GST_MESSAGE_EOS:
    RCLCPP_INFO(streamer->get_logger(), "End of stream reached.");
    streamer->stop_stream();
    break;

  case GST_MESSAGE_ERROR: {
    gchar *debug;
    GError *error;

    gst_message_parse_error(message, &error, &debug);
    g_free(debug);

    RCLCPP_ERROR(streamer->get_logger(), "Error from gstreamer pipeline: %s",
                 error->message);
    g_error_free(error);

    streamer->stop_stream();
    break;
  }
  default:
    break;
  }

  return 0;
};

void ZedStreamer::poll_stream() {
  if (!this->initialised) {
    this->
  } else if (this->start_stream()) {
    RCLCPP_FATAL(this->get_logger(),
                 "Failed to start stream. Shutting down...");
    this->shutdown();
  }
}

int ZedStreamer::start_stream() {
  RCLCPP_INFO(this->get_logger(), "Starting gstreamer pipeline...");

  pipeline = gst_pipeline_new("depth");
  if (!pipeline) {
    RCLCPP_FATAL(this->get_logger(), "Failed to launch gstreamer pipeline");
    return 1;
  }

  auto source = gst_element_factory_make("videotestsrc", "source");
  auto sink = gst_element_factory_make("autovideosink", "sink");
  gst_bin_add_many(GST_BIN(pipeline), source, sink, NULL);

  if (!gst_element_link_many(source, sink, NULL)) {
    RCLCPP_FATAL(this->get_logger(),
                 "Failed to link gstreamer pipeline elements.");
    return 1;
  }

  GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  bus_watch_id = gst_bus_add_watch(bus, &gst_bus_call, this);

  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  active = true;

  RCLCPP_INFO(this->get_logger(), "Starting main pipeline loop.");
  return 0;
}

void ZedStreamer::stop_stream() {
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
  g_source_remove(bus_watch_id);
}
