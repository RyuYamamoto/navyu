FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt update \
 && apt install -y --no-install-recommends locales software-properties-common tzdata \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && add-apt-repository universe

ENV LANG en_US.UTF-8
ENV TZ=Asia/Tokyo

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt install -y git tmux wget curl gnupg lsb-release

RUN mkdir -p ~/ros2_ws/src
WORKDIR ~/ros2_ws/src

RUN mkdir -p ${HOME}/.gazebo/models && \
    wget -O /tmp/sun http://models.gazebosim.org/sun/model.tar.gz && \
    wget -O /tmp/ground_plane http://models.gazebosim.org/ground_plane/model.tar.gz && \
    tar xvzf /tmp/sun -C ${HOME}/.gazebo/models && \
    tar xvzf /tmp/ground_plane -C ${HOME}/.gazebo/models

RUN git clone https://github.com/RyuYamamoto/navyu.git
WORKDIR ../
RUN apt update && apt install -y python3-rosdep2 python3-colcon-common-extensions
RUN rosdep update
RUN apt install -y \
    ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
    ros-humble-controller-manager \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-nav2-bringup
RUN source /opt/ros/humble/setup.bash && \
    rosdep install -iry --from-paths src --rosdistro humble && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to navyu

ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ros2_ws/src/navyu/navyu_simulator/world

RUN mkdir -p "$HOME/.gazebo/models" && mkdir /tmp/models && \
    cd /tmp/models && \
    for m in "cube_20k" "dumpster" "jersey_barrier" "ground_plane" "sun" "willowgarage"; do \
    curl -Os "http://models.gazebosim.org/$m/model.tar.gz" && \
    tar -zvxf model.tar.gz && \
    cp -vfR $m "$HOME/.gazebo/models/"; \
    rm model.tar.gz; \
    done && \
    cd .. && rm -r /tmp/models

COPY entrypoint.sh /usr/bin/
RUN chmod +x /usr/bin/entrypoint.sh

ENTRYPOINT ["entrypoint.sh"]
