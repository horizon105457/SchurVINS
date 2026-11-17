ARG ROS_IMAGE=ros:jazzy-ros-base@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f
FROM ${ROS_IMAGE} AS runtime
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ARG UBUNTU_MIRROR=https://mirrors.ustc.edu.cn
RUN if [ -n "$UBUNTU_MIRROR" ]; then \
      sed -i -e "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}/ubuntu|g" \
             -e "s|http://security.ubuntu.com/ubuntu|${UBUNTU_MIRROR}/ubuntu|g" \
             -e "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_MIRROR}/ubuntu-ports|g" \
        /etc/apt/sources.list.d/ubuntu.sources; \
    fi
RUN apt-get -o Acquire::Retries=3 update && apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
    build-essential cmake ninja-build git curl ca-certificates unzip pkg-config \
    libeigen3-dev libopencv-dev libopencv-contrib-dev libyaml-cpp-dev \
    libgoogle-glog-dev libgflags-dev libboost-filesystem-dev libboost-system-dev \
    libboost-thread-dev libopengv-dev libceres-dev libglew-dev \
    python3-colcon-common-extensions python3-numpy python3-opencv python3-yaml \
    python3-scipy python3-matplotlib python3-pip python3-venv \
    ros-jazzy-cv-bridge ros-jazzy-message-filters ros-jazzy-tf2-ros \
    ros-jazzy-rosbag2-storage-mcap ros-jazzy-rosgraph-msgs
RUN python3 -m venv --system-site-packages /opt/evo \
    && /opt/evo/bin/pip install --no-cache-dir evo==1.38.1
ENV HEXBOT_VIO_CONTAINER=1 ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp
WORKDIR /workspaces/schurvins_ws
