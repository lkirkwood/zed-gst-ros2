#include "zed-gst/zed_streamer.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "utsma_common/lifecycle_node.hpp"
#include <gst/gst.h>
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
    : utsma_common::LifecycleNode("zed_cam", "", 1000.0, false),
      left_cam{nullptr}, right_cam{nullptr}, depth_cam{nullptr} {
  active = false;

  if (this->init_gst()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialised gstreamer.");
    initialised = false;
    return;
  }

  left_cam = this->create_pipeline("left_cam", "image/left_cam", {});

  right_cam = this->create_pipeline("right_cam", "image/right_cam", {});

  depth_cam = this->create_pipeline("depth_cam", "image/depth_cam", {});

  for (auto pipe : pipelines) {
    if (pipe == nullptr) {
      RCLCPP_FATAL(this->get_logger(), "Failed to create gstreamer pipeline.");
      initialised = false;
      return;
    };
  }

  initialised = true;
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
  for (Pipeline **pipe : this->pipelines) {
    gst_object_unref((*pipe)->pipeline);
    g_source_remove((*pipe)->bus_watch_id);
  }
  return utsma_common::CallbackReturn::SUCCESS;
};

utsma_common::CallbackReturn
ZedStreamer::on_shutdown(const rclcpp_lifecycle::State &) {
  this->shutdown();
  return utsma_common::CallbackReturn::SUCCESS;
};

int ZedStreamer::init_gst() {
  GError *error = new GError;
  if (!gst_init_check(NULL, NULL, &error)) {
    std::string message = "Failed to initialise gstreamer: ";
    message += error->message;
    RCLCPP_FATAL(this->get_logger(), "%s", message.c_str());
    return 1;
  }
  return 0;
}

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

ZedStreamer::Pipeline *
ZedStreamer::create_pipeline(const char *name, const char *ros_topic,
                             std::map<const char *, const char *> src_opts) {
  GstElement *source = gst_element_factory_make("videotestsrc", "source");
  for (auto [key, val] : src_opts) {
    g_object_set(G_OBJECT(source), key, val, NULL);
  }

  GstElement *sink = gst_element_factory_make("rosimagesink", "sink");
  g_object_set(G_OBJECT(sink), "ros-topic", ros_topic, NULL);

  GstElement *gst_pipe = gst_pipeline_new(name);
  gst_bin_add_many(GST_BIN(gst_pipe), source, sink, NULL);

  if (!gst_element_link_many(source, sink, NULL)) {
    RCLCPP_FATAL(this->get_logger(),
                 "Failed to link gstreamer pipeline elements.");
    return nullptr;
  }

  GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(gst_pipe));
  guint bus_watch_id = gst_bus_add_watch(bus, &gst_bus_call, this);

  Pipeline *pipeline =
      new Pipeline{gst_pipe, bus_watch_id, name, ros_topic, src_opts};

  RCLCPP_INFO(this->get_logger(), "Pipeline: %s", pipeline->name);
  if (pipeline == nullptr) {
    RCLCPP_INFO(this->get_logger(), "Pipeline: %s", pipeline->name);
  }
  return pipeline;
}

void ZedStreamer::poll_stream() {
  if (!this->active) {
    if (this->start_stream()) {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to start stream. Shutting down...");
      this->shutdown();
    }
  }
}

int ZedStreamer::start_stream() {
  RCLCPP_INFO(this->get_logger(), "Starting gstreamer pipeline...");

  for (Pipeline **pipe : this->pipelines) {
    gst_element_set_state((*pipe)->pipeline, GST_STATE_PLAYING);
  }

  active = true;

  return 0;
}

void ZedStreamer::stop_stream() {
  for (Pipeline **pipe : this->pipelines) {
    if ((*pipe) != nullptr) {
      gst_element_set_state((*pipe)->pipeline, GST_STATE_NULL);
    }
  }
}
