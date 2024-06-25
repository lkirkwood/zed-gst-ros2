FROM dustynv/ros:humble-desktop-l4t-r35.4.1

RUN apt-get update

# Install GStreamer
RUN dpkg -P opencv-dev opencv-libs
RUN apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 gstreamer1.0-pulseaudio

# Install python binding deps
RUN apt-get install -y libgirepository1.0-dev gcc libcairo2-dev pkg-config python3-dev gir1.2-gtk-3.0
RUN pip install pycairo pygobject

RUN mkdir -p /opt/workspace/src
RUN git clone https://github.com/BrettRD/ros-gst-bridge /opt/workspace/src/gst_bridge

COPY . /opt/workspace/src/zed-gst-ros2

WORKDIR /opt/workspace
RUN source /opt/ros/humble/install/setup.bash && colcon build

ENV GST_PLUGIN_PATH /opt/workspace/install/gst_bridge/lib/gst_bridge

RUN chmod +x /opt/workspace/src/zed-gst-ros2/entrypoint.sh
ENTRYPOINT ["/opt/workspace/src/zed-gst-ros2/entrypoint.sh"]
