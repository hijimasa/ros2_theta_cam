FROM ros:humble

ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  apt-get install -y -qq \
    libyaml-cpp-dev \
    vim v4l-utils exfat-* \
    python3-pip && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

# for ros2
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    && apt-get clean

RUN set -x && \
  apt-get update -y -qq && \
  apt-get install -y -qq ros-humble-desktop \
    ros-humble-ament-cmake ros-humble-angles ros-humble-controller-manager \
    ros-humble-cv-bridge ros-humble-usb-cam && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

RUN set -x && \
  pip3 install -U pip
  
# for theta v
RUN cd /opt && \
  git clone https://github.com/nickel110/libuvc.git && \
  cd libuvc && \
  mkdir build && \
  cd build && \
  cmake .. && \
  make && make install
ENV PKG_CONFIG_PATH ${PKG_CONFIG_PATH}:/usr/local/lib
RUN set -x && \
  apt-get update -y -qq && \
  apt-get install -y -qq libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio libgstreamer-plugins-base1.0-dev
RUN cd /opt && \
  git clone https://github.com/nickel110/gstthetauvc.git && \
  cd gstthetauvc/thetauvc && \
  make
ENV GST_PLUGIN_PATH /opt/gstthetauvc/thetauvc

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENTRYPOINT ["/bin/bash"]
