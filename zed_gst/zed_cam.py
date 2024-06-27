import gi

gi.require_version("Gst", "1.0")
import os
import sys
from gi.repository import Gst

Gst.init(sys.argv)


def main():
    camera_fps = os.environ.get("ZED_CAM_FPS") or "60"
    topic = os.environ.get("ZED_ROS_TOPIC") or "image/depth"
    pipeline = Gst.parse_launch(
        f"zedsrc stream-type=3 camera-fps={camera_fps} ! rosimagesink ros-topic={topic}"
    )
    bus = pipeline.get_bus()
    pipeline.set_state(Gst.State.PLAYING)

    try:
        while True:
            ...
    except KeyboardInterrupt:
        print("Stopping stream.")

    pipeline.send_event(Gst.Event.new_eos())
    bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS)
    pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
