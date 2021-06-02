# This Dockerfile builds a "kitchen-sink" production ready image. This image is
# useful for deployment to desktop computers, but is not ideal for deployment
# to resource constrained robots (not that you'd want to deploy this package to
# a robot anyways).

#------------------------------------------------------------------#
# Dependency versions
ARG version_dave=f3a289dd1f2a6f56daccae502299f7d87d9e0dca
ARG version_uuv_simulator=0.6.13
ARG version_uuv_manipulators=0.6.1
ARG version_uw_sensors_gazebo=b9083c054c48fc9553eddeadae5dec447058607a
ARG ds_sim_commit=36835b85c30f5e063384378992f6ea60b5410d66
ARG ds_msgs_commit=4a7d4a5002d17631e67eb01c45b974293fe5f3c2
ARG frl_msgs_commit=a41b1d3d192dacd32a7e0a717a9c1b0a30eaca64
#------------------------------------------------------------------#

#------------------------------------------------------------------#
# Other arguments
ARG ros_distro=melodic
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

# Install ROS dependencies that are not available via a package manager.
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get install -y --no-install-recommends --no-upgrade curl ca-certificates python-pip

WORKDIR /glider_hybrid_whoi_deps/src

ARG version_dave
ENV VERSION_DAVE=$version_dave
ARG version_uw_sensors_gazebo
ENV VERSION_UW_SENSORS_GAZEBO=$version_uw_sensors_gazebo
RUN curl -fsSL https://github.com/Field-Robotics-Lab/dave/archive/$VERSION_DAVE.tar.gz | tar xz \
    && curl -fsSL https://github.com/Field-Robotics-Lab/nps_uw_sensors_gazebo/archive/$VERSION_UW_SENSORS_GAZEBO.tar.gz | tar xz

ARG version_uuv_simulator
ENV VERSION_UUV=$version_uuv_simulator
RUN curl -fsSL https://github.com/uuvsimulator/uuv_simulator/archive/$VERSION_UUV.tar.gz | tar xz

ARG version_uuv_manipulators
ENV VERSION_UUV_MANIPULATORS=$version_uuv_manipulators
RUN curl -fsSL https://github.com/uuvsimulator/uuv_manipulators/archive/$VERSION_UUV_MANIPULATORS.tar.gz | tar xz

ARG ds_sim_commit
ENV DS_SIM_COMMIT=$ds_sim_commit
RUN curl -fsSL https://bitbucket.org/daewok/ds_sim/get/$DS_SIM_COMMIT.tar.gz | tar xz

ARG ds_msgs_commit
ENV DS_MSGS_COMMIT=$ds_msgs_commit
RUN curl -fsSL https://bitbucket.org/whoidsl/ds_msgs/get/$DS_MSGS_COMMIT.tar.gz | tar xz

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
       python-pip curl apt-utils software-properties-common xauth \
    && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && curl -fsSL http://packages.osrfoundation.org/gazebo.key | apt-key add - \
    && apt-get update \
    && sh /tmp/install-script-for-deps \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

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

ARG user
ARG group

COPY --from=source-deps-builder --chown=$user:$group /home/$user/glider_hybrid_whoi_deps/install /home/$user/glider_hybrid_whoi_deps/install
COPY --from=builder --chown=$user:$group /home/$user/glider_hybrid_whoi/install /home/$user/glider_hybrid_whoi/install

ENV GLIDER_HYBRID_WHOI_USER=$user
COPY docker/docker-entrypoint.sh /ros_entrypoint.sh

USER $user:$group
