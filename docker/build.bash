#!/usr/bin/env bash
set -Eeuo pipefail

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

# Change into the parent directory of this script, no matter how it is invoked.
cd "$(dirname "$(readlink -f "$BASH_SOURCE")")/.."

# Builds a Docker image.
BUILD_ARGS=""
image_name="glider_kinematics"

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -w|--without-nvidia)
    BUILD_ARGS="--build-arg BASEIMG=ubuntu:bionic"
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

image_plus_tag=$image_name:$(export LC_ALL=C; date +%Y_%m_%d_%H%M)
docker build --rm -t "$image_plus_tag" $BUILD_ARGS -f docker/Dockerfile "$@" .
docker tag $image_plus_tag $image_name:latest
echo "Built $image_plus_tag and tagged as $image_name:latest"
