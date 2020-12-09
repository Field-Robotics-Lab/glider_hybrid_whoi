# Dockerfile for Glider Kinematics Plugin
#------------------------------------------------------------------#
ARG version_dave=2.0.0
ARG version_uuv_simulator=0.6.13
ARG version_uuv_manipulators=0.6.1
ARG ds_sim_commit=d8b8d1383436978b0244796654950518224c7cfd
ARG ds_msgs_commit=4a7d4a5002d17631e67eb01c45b974293fe5f3c2
ARG frl_msgs_commit=ab362d2f7264ce8be4d6b12bf41538ca4215bfba
#------------------------------------------------------------------#

ARG ros_distro=melodic

FROM ros:$ros_distro-ros-base as deps-extractor

ARG ros_distro

# Initialize the workspace
WORKDIR /uuv_ws/src

# hadolint ignore=SC1091
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && catkin_init_workspace

# Get the devel, etc. folders created
WORKDIR /uuv_ws
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && catkin_make

# Install repositories
RUN apt-get update \
  && apt-get install -y wget \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /uuv_ws/src

ARG version_dave
ENV VERSION_DAVE=$version_dave
RUN wget https://github.com/Field-Robotics-Lab/dave/archive/v$VERSION_DAVE.tar.gz \
    && tar xvzf v$VERSION_DAVE.tar.gz  \
    && mv dave-$VERSION_DAVE dave \
    && rm v$VERSION_DAVE.tar.gz
RUN wget https://github.com/Field-Robotics-Lab/nps_uw_sensors_gazebo/archive/v$VERSION_DAVE.tar.gz \
    && tar xvzf v$VERSION_DAVE.tar.gz  \
    && mv nps_uw_sensors_gazebo-$VERSION_DAVE nps_uw_sensors_gazebo \
    && rm v$VERSION_DAVE.tar.gz

ARG version_uuv_simulator
ENV VERSION_UUV=$version_uuv_simulator
RUN wget https://github.com/uuvsimulator/uuv_simulator/archive/$VERSION_UUV.tar.gz \
    && tar xvzf $VERSION_UUV.tar.gz  \
    && mv uuv_simulator-$VERSION_UUV uuv_simulator \
    && rm $VERSION_UUV.tar.gz

ARG version_uuv_manipulators
ENV VERSION_UUV_MANIPULATORS=$version_uuv_manipulators
RUN wget https://github.com/uuvsimulator/uuv_manipulators/archive/$VERSION_UUV_MANIPULATORS.tar.gz \
    && tar xvzf $VERSION_UUV_MANIPULATORS.tar.gz  \
    && mv uuv_manipulators-$VERSION_UUV_MANIPULATORS uuv_manipulators \
    && rm $VERSION_UUV_MANIPULATORS.tar.gz

WORKDIR /uuv_ws

ARG ds_sim_commit
ENV DS_SIM_COMMIT=$ds_sim_commit
RUN git clone https://bitbucket.org/whoidsl/ds_sim.git \
    && git -C ds_sim archive --prefix=src/ds_sim/ "$DS_SIM_COMMIT" | tar x \
    && rm -rf ds_sim

ARG ds_msgs_commit
ENV DS_MSGS_COMMIT=$ds_msgs_commit
RUN git clone https://bitbucket.org/whoidsl/ds_msgs.git \
    && git -C ds_msgs archive --prefix=src/ds_msgs/ "$DS_MSGS_COMMIT" | tar x \
    && rm -rf ds_msgs

ARG frl_msgs_commit
ENV FRL_MSGS_COMMIT=$frl_msgs_commit
RUN git clone https://github.com/Field-Robotics-Lab/frl_msgs.git \
    && git -C frl_msgs archive --prefix=src/frl_msgs/ "$FRL_MSGS_COMMIT" | tar x \
    && rm -rf frl_msgs

RUN apt-get update \
    && apt-get install -y --no-upgrade --no-install-recommends python-pip \
    && rm -rf /var/lib/apt/lists/*

ADD . src/glider_hybrid_whoi/

# Generate the install script!
WORKDIR /uuv_ws
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -y -s --as-root pip:false --as-root apt:false | sort -u | tee /tmp/install-script \
    && rm -rf /var/lib/apt/lists/*

# Assemble the runtime image.
FROM nvidia/opengl:base-ubuntu18.04
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display

FROM ros:$ros_distro-ros-base

ARG ros_distro
COPY --from=deps-extractor /tmp/install-script /tmp/install-script

# Install the dependencies
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && apt-get update \
    && apt-get install -y --no-upgrade --no-install-recommends \
       python-pip curl apt-utils sudo wget \
       libeigen3-dev \
       build-essential \
       pkg-config \
       protobuf-compiler \
       software-properties-common \
    && cat /tmp/install-script \
    && sh /tmp/install-script \
    #&& rm /tmp/install-script
    && pip install pandas \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# Install gazebo 9
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
    && apt-get update \
    && apt-get install -y ros-$ros_distro-gazebo-ros-pkgs ros-$ros_distro-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# Carry forward the commit as an env var in this image
ARG ds_msgs_commit
ENV DS_MSGS_COMMIT=$ds_msgs_commit
ARG ds_sim_commit
ENV DS_SIM_COMMIT=$ds_sim_commit
ARG frl_msgs_commit
ENV FRL_MSGS_COMMIT=$frl_msgs_commit
ARG glider_hybrid_whoi_commit
ENV GLIDER_HYBRID_WHOI_COMMIT=$glider_hybrid_whoi_commit

# Set USER and GROUP
ARG USER=glider
ARG GROUP=glider

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`.
RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

RUN addgroup --gid 1000 $USER && \
    adduser --uid 1000 --ingroup $USER --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER

RUN adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# Commands below run as the user.
USER $USER:$GROUP

RUN  sudo apt-get update \
  && sudo apt-get install -y \
    qtbase5-dev \
    libgles2-mesa-dev \
    ros-$ros_distro-velodyne-gazebo-plugins \
    ros-$ros_distro-effort-controllers \
    ros-$ros_distro-rqt \
    ros-$ros_distro-rqt-robot-plugins \
    ros-$ros_distro-rqt-common-plugins \
    ros-$ros_distro-joy \
    ros-$ros_distro-teleop-twist-joy \
    ros-$ros_distro-teleop-twist-keyboard \
    ros-$ros_distro-teleop-tools \
    ros-$ros_distro-joy-teleop \
    ros-$ros_distro-key-teleop \
    ros-$ros_distro-geographic-info \
    ros-$ros_distro-move-base \
    ros-$ros_distro-robot-localization \
    ros-$ros_distro-robot-state-publisher \
    ros-$ros_distro-xacro \
    libignition-math2 \
  && sudo apt-get clean \
  && sudo rm -rf /var/lib/apt/lists/*

# When running a container start in the home folder.
WORKDIR /home/$USER/uuv_ws
RUN sudo chown -R $USER:$GROUP /home/$USER/uuv_ws
COPY --from=deps-extractor --chown=$USER:$GROUP /uuv_ws/src src

# make and install!
# RUN . "/opt/ros/$ros_distro/setup.sh" \
#     && catkin_make \
#     && catkin_make install

# # Source all the needed environment files.
RUN /bin/sh -c 'echo ". /opt/ros/$ros_distro/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc' \
 && /bin/sh -c 'echo "source ~/uuv_ws/devel/setup.bash" >> ~/.bashrc'

# Replace the standard ROS entrypoint with one that sources our overlay and
# execs fixuid.
ADD docker/docker-entrypoint.sh /ros_entrypoint.sh

CMD ["/bin/bash"]
