FROM moveit/moveit2:main-humble-tutorial-source

SHELL ["/bin/bash", "-c", "-o", "pipefail"]

ARG CUDA_MAJOR_VERSION=11
ARG CUDA_MINOR_VERSION=7

# Remove all ROS repository configurations and fix GPG key
RUN rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros*.list && \
    find /etc/apt/sources.list.d/ -name "*ros*" -delete && \
    sed -i '/packages.ros.org/d' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends curl gnupg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Bring the container up to date to get the latest ROS sync
# hadolint ignore=DL3008, DL3013
RUN apt-get update && apt-get upgrade -y && rosdep update

# Install packages required to run the demo including missing MoveIt dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-topic-based-ros2-control \
    ros-humble-geometric-shapes \
    ros-humble-realtime-tools \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    wget \
    python3-pip

# Rebuild the MoveIt workspace to ensure all dependencies are properly linked
RUN cd /root/ws_moveit && \
    source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install CUDA
ENV DEBIAN_FRONTEND noninteractive
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb && \
    dpkg -i cuda-keyring_1.0-1_all.deb && \
    apt-get update && \
    apt-get -y install cuda-cudart-${CUDA_MAJOR_VERSION}-${CUDA_MINOR_VERSION}  cuda-compat-${CUDA_MAJOR_VERSION}-${CUDA_MINOR_VERSION} cuda-toolkit-${CUDA_MAJOR_VERSION}-${CUDA_MINOR_VERSION}
ENV PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:$LD_LIBRARY_PATH

# Install pytorch
RUN pip3 install torch torchvision

# Use Fast DDS as middleware and load the required config for NVIDIA Isaac Sim.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN mkdir -p /opt/.ros
COPY ./doc/how_to_guides/isaac_panda/.docker/fastdds.xml /opt/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/.ros/fastdds.xml

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=${CUDA_MAJOR_VERSION}.${CUDA_MINOR_VERSION}"

# Set up the entrypoint for both container start and interactive terminals.
COPY ./doc/how_to_guides/isaac_panda/.docker/ros_entrypoint.sh /opt/.ros/
RUN echo "source /opt/.ros/ros_entrypoint.sh" >> ~/.bashrc
ENTRYPOINT [ "/opt/.ros/ros_entrypoint.sh" ]
