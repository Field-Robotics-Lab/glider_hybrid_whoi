# This Dockerfile builds a "kitchen-sink" production ready image. This image is
# useful for deployment to desktop computers, but is not ideal for deployment
# to resource constrained robots (not that you'd want to deploy this package to
# a robot anyways).

#------------------------------------------------------------------#
# Dependency versions
ARG version_dave=896ccda30833949defda239e946220a1653d3465
ARG version_uuv_simulator=4936e739d48807527fdcd3ba3d19c147277003ce
ARG version_uuv_manipulators=d324a6a6ac9f3b6c86427d7ed9795c09c14e659c
ARG version_uw_sensors_gazebo=eda6787830bd2c7f31eacfe9f2aab961b879ab3e
ARG ds_sim_commit=169f805fb2fff2e095643e6e1df5de223ef45698
ARG ds_msgs_commit=179631d46871b31fad28a21f81d865e3c73446d3
ARG frl_msgs_commit=a41b1d3d192dacd32a7e0a717a9c1b0a30eaca64
#------------------------------------------------------------------#

#------------------------------------------------------------------#
# Other arguments
ARG ros_distro=noetic
ARG user=ros
ARG group=ros
#------------------------------------------------------------------#

#------------------------------------------------------------------#
# source-deps stage
#------------------------------------------------------------------#

# This stage is responsible for downloading the source code of dependencies
# that we're installing from source and using rosdep to generate an script to
# install /their/ dependencies. The script is located at
# /tmp/install-script-for-deps. The source code is added to a workspace located
# at /glider_hybrid_whoi_deps.
FROM ros:$ros_distro-ros-base as source-deps

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Initialize the dependency workspace
WORKDIR /glider_hybrid_whoi_deps/src

RUN . "/opt/ros/$ROS_DISTRO/setup.bash" \
    && catkin_init_workspace

# Get the devel, etc. folders created
WORKDIR /glider_hybrid_whoi_deps
RUN . "/opt/ros/$ROS_DISTRO/setup.bash" \
    && catkin_make

# Required utilities from https://github.com/Field-Robotics-Lab/dockwater/blob/main/noetic/Dockerfile
RUN apt update \
 && apt install -y --no-install-recommends\
        build-essential \
        cmake \
        cppcheck \
        curl \
        git \
        gnupg \
        libeigen3-dev \
        libgles2-mesa-dev \
        lsb-release \
        pkg-config \
        protobuf-compiler \
        python3-dbg \
        python3-pip \
        python3-venv \
        qtbase5-dev \
        ruby \
        software-properties-common \
        sudo \
        wget \
 && apt clean

# Get ROS and Gazebo from https://github.com/Field-Robotics-Lab/dockwater/blob/main/noetic/Dockerfile
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list' \
 && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743 \
 && apt update \
 && apt install -y --no-install-recommends \
    gazebo11 \
    libgazebo11-dev \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-gdal \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-effort-controllers \
    ros-noetic-geographic-info \
    ros-noetic-hector-gazebo-plugins \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-state-publisher \
    ros-noetic-joy \
    ros-noetic-joy-teleop \
    ros-noetic-kdl-parser-py \
    ros-noetic-key-teleop \
    ros-noetic-move-base \
    ros-noetic-robot-localization \
    ros-noetic-robot-state-publisher \
    ros-noetic-ros-base \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-rqt-robot-plugins \
    ros-noetic-rviz \
    ros-noetic-teleop-tools \
    ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-tools \
    ros-noetic-urdfdom-py \
    ros-noetic-velodyne-gazebo-plugins \
    ros-noetic-velodyne-simulator \
    ros-noetic-xacro \
 && apt clean

WORKDIR /glider_hybrid_whoi_deps/src

ARG version_dave
ENV VERSION_DAVE=$version_dave
ARG version_uw_sensors_gazebo
ENV VERSION_UW_SENSORS_GAZEBO=$version_uw_sensors_gazebo
RUN curl -fsSL https://github.com/Field-Robotics-Lab/dave/archive/$VERSION_DAVE.tar.gz | tar xz \
    && curl -fsSL https://github.com/Field-Robotics-Lab/nps_uw_sensors_gazebo/archive/$VERSION_UW_SENSORS_GAZEBO.tar.gz | tar xz

ARG version_uuv_simulator
ENV VERSION_UUV=$version_uuv_simulator
RUN curl -fsSL https://github.com/Field-Robotics-Lab/uuv_simulator/archive/$VERSION_UUV.tar.gz | tar xz

ARG version_uuv_manipulators
ENV VERSION_UUV_MANIPULATORS=$version_uuv_manipulators
RUN curl -fsSL https://github.com/Field-Robotics-Lab/uuv_manipulators/archive/$VERSION_UUV_MANIPULATORS.tar.gz | tar xz

ARG ds_sim_commit
ENV DS_SIM_COMMIT=$ds_sim_commit
RUN curl -fsSL https://github.com/Field-Robotics-Lab/ds_sim/archive/$DS_SIM_COMMIT.tar.gz | tar xz

ARG ds_msgs_commit
ENV DS_MSGS_COMMIT=$ds_msgs_commit
RUN curl -fsSL https://github.com/Field-Robotics-Lab/ds_msgs/archive/$DS_MSGS_COMMIT.tar.gz | tar xz

ARG frl_msgs_commit
ENV FRL_MSGS_COMMIT=$frl_msgs_commit
RUN curl -fsSL https://github.com/Field-Robotics-Lab/frl_msgs/archive/$FRL_MSGS_COMMIT.tar.gz | tar xz

# Generate the install script needed to install the dependencies of our
# dependencies.
RUN . "/opt/ros/$ROS_DISTRO/setup.bash" \
    && apt-get update \
    && rosdep install --from-paths . --ignore-src -y -s --as-root pip:false --as-root apt:false | tee /tmp/install-script-for-deps

#------------------------------------------------------------------#
# deps stage
#------------------------------------------------------------------#

# The purpose of this stage is to extract all of our dependencies and generate
# a script that can be used in later stages to install them. The script is
# located at /tmp/install-script.
#
# This stage builds off the last one and simply imports our code into the same
# workspace (for simplicity). We make it a separate stage so that later we can
# easily extract only the source code for the deps we install from source.

FROM source-deps as deps

COPY . glider_hybrid_whoi/

# Generate the install script!
RUN . "/opt/ros/$ROS_DISTRO/setup.sh" \
    && apt-get update \
    && rosdep install --from-paths . --ignore-src -y -s --as-root pip:false --as-root apt:false | tee /tmp/install-script

#------------------------------------------------------------------#
# user-creation stage
#------------------------------------------------------------------#

# The purpose of this stage is to create the non-privileged user whose home dir
# everything will be installed into in the final image. Additionally, this
# installs the deps of our source deps. This stage *will* end up in the final
# image, so we need to be careful about apt lists.

FROM ros:$ros_distro-ros-base as user-creation
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG user
ARG group

RUN addgroup --gid 1000 $group \
    && adduser --uid 1000 --ingroup $group --home /home/$user --shell /bin/sh --disabled-password --gecos "" $user

COPY --from=source-deps /tmp/install-script-for-deps /tmp/install-script-for-deps

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get install -y --no-upgrade --no-install-recommends \
       python3-pip curl apt-utils software-properties-common xauth \
    && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && curl -fsSL http://packages.osrfoundation.org/gazebo.key | apt-key add - \
    && apt-get update \
    && sh /tmp/install-script-for-deps \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# Get ROS and Gazebo from https://github.com/Field-Robotics-Lab/dockwater/blob/main/noetic/Dockerfile
RUN apt-get update \
    && apt-get -y --no-upgrade --no-install-recommends install \
    gazebo11 \
    libgazebo11-dev \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-gdal \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-effort-controllers \
    ros-noetic-geographic-info \
    ros-noetic-hector-gazebo-plugins \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-state-publisher \
    ros-noetic-joy \
    ros-noetic-joy-teleop \
    ros-noetic-kdl-parser-py \
    ros-noetic-key-teleop \
    ros-noetic-move-base \
    ros-noetic-robot-localization \
    ros-noetic-robot-state-publisher \
    ros-noetic-ros-base \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-rqt-robot-plugins \
    ros-noetic-rviz \
    ros-noetic-teleop-tools \
    ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-tools \
    ros-noetic-urdfdom-py \
    ros-noetic-velodyne-gazebo-plugins \
    ros-noetic-velodyne-simulator \
    ros-noetic-xacro \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir pynmea2

#------------------------------------------------------------------#
# source-deps-builder stage
#------------------------------------------------------------------#

# The purpose of this stage is to build and install all of the dependencies
# that we're installing from source.

FROM user-creation as source-deps-builder

ARG user
ARG group

USER $user:$group

# It seems that WORKDIR doesn't respect the user??
RUN mkdir /home/$user/glider_hybrid_whoi_deps

WORKDIR /home/$user/glider_hybrid_whoi_deps

COPY --from=source-deps --chown=$user:$group /glider_hybrid_whoi_deps/src src

RUN . "/opt/ros/$ROS_DISTRO/setup.sh" \
    && catkin_make \
    && catkin_make install

#------------------------------------------------------------------#
# builder stage
#------------------------------------------------------------------#

# Now we can build our first party code!

FROM source-deps-builder as builder

USER root:root

COPY --from=deps /tmp/install-script /tmp/install-script

RUN apt-get update \
    && sh /tmp/install-script \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

ARG user
ARG group

USER $user:$group

# It seems that WORKDIR doesn't respect the user??
RUN mkdir -p /home/$user/glider_hybrid_whoi/src

WORKDIR /home/$user/glider_hybrid_whoi/src
COPY --chown=$user:$group . glider_hybrid_whoi

WORKDIR /home/$user/glider_hybrid_whoi

RUN . "/home/$user/glider_hybrid_whoi_deps/install/setup.bash" \
    && catkin_make \
    && catkin_make install

#------------------------------------------------------------------#
# final stage
#------------------------------------------------------------------#

# Now we can finally start assembling the final image!

FROM user-creation

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ARG user
ARG group

# Add fixuid so we can fixup file permissions at runtime. It's very sad we have
# to resort to this and Docker doesn't have a better first-class solution.
ARG fixuid_version=0.5
RUN apt-get update \
    && apt-get install -y --no-install-recommends curl ca-certificates \
    && curl -fSsL https://github.com/boxboat/fixuid/releases/download/v$fixuid_version/fixuid-$fixuid_version-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - \
    && chown root:root /usr/local/bin/fixuid \
    && chmod 4755 /usr/local/bin/fixuid \
    && mkdir -p /etc/fixuid \
    && printf "user: $user\ngroup: $group\n" > /etc/fixuid/config.yml \
    && rm -rf /var/lib/apt/lists/*

COPY --from=deps /tmp/install-script /tmp/install-script

RUN apt-get update \
    && sh /tmp/install-script \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# Carry forward the dependency versions
ARG version_dave
ENV VERSION_DAVE=$version_dave
ARG version_uw_sensors_gazebo
ENV VERSION_UW_SENSORS_GAZEBO=$version_uw_sensors_gazebo
ARG version_uuv_simulator
ENV VERSION_UUV_SIMULATOR=$version_uuv_simulator
ARG version_uuv_manipulators
ENV VERSION_UUV_MANIPULATORS=$version_uuv_manipulators
ARG ds_msgs_commit
ENV DS_MSGS_COMMIT=$ds_msgs_commit
ARG ds_sim_commit
ENV DS_SIM_COMMIT=$ds_sim_commit
ARG frl_msgs_commit
ENV FRL_MSGS_COMMIT=$frl_msgs_commit

COPY --from=source-deps-builder --chown=$user:$group /home/$user/glider_hybrid_whoi_deps/install /home/$user/glider_hybrid_whoi_deps/install
COPY --from=builder --chown=$user:$group /home/$user/glider_hybrid_whoi/install /home/$user/glider_hybrid_whoi/install

ENV GLIDER_HYBRID_WHOI_USER=$user
RUN curl -fsSL https://raw.githubusercontent.com/Field-Robotics-Lab/glider_hybrid_whoi/master/docker/docker-entrypoint.sh > /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
#COPY /docker/docker-entrypoint.sh /ros_entrypoint.sh

USER $user:$group
