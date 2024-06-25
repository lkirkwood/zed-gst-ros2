import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst
import sys

Gst.init(sys.argv)


def main():
    pipeline = Gst.parse_launch(
        "v4l2src device=/dev/video0 ! rosimagesink ros-topic=image"
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
