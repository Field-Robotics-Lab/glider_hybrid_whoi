#!/usr/bin/env bash

#
# Copyright (C) 2018 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

# Builds a Docker image.
BUILD_BASE=""
BUILD_ROS_GAZ=""
image_name="glider_kinematics"

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -w|--without-nvidia)
    BUILD_BASE="--build-arg BASEIMG=ubuntu:bionic"
    image_name="glider_kinematics"
    shift
    ;;
    *)    # unknown option
    POSITIONAL+=("$1")
    shift
    ;;
esac
done

set -- "${POSITIONAL[@]}"

if [ $# -lt 1 ]
then
    echo "Usage: $0 [-n --nvidia] <root of vrx repo>"
    exit 1
fi

if [ ! -f $1/Dockerfile ]
then
    echo "Err: Directory does not contain a Dockerfile to build."
    exit 1
fi


image_plus_tag=$image_name:$(export LC_ALL=C; date +%Y_%m_%d_%H%M)
echo ".*" > "${1}"/.dockerignore
docker build --rm -t $image_plus_tag -f "${1}"/Dockerfile "${1}" $BUILD_BASE $BUILD_ROS_GAZ && \
docker tag $image_plus_tag $image_name:latest && \
echo "Built $image_plus_tag and tagged as $image_name:latest"
rm "${1}"/.dockerignore
