#!/usr/bin/env bash
set -Eeuo pipefail

# Copyright (C) 2020 glider_hybrid_whoi authors
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

# Determine the parent directory of this script, no matter how it is invoked.
cd "$(dirname "$(readlink -f "$BASH_SOURCE")")/.."

docker build -t "glider_hybrid_whoi:dev" -f docker/Dockerfile.dev "$@" .
echo "Built glider_hybrid_whoi:dev"
