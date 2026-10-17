#!/usr/bin/env bash
# Build the simulator's pinned RGL plugin without installing system files.
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
simulator="$workspace/src/rm_sim_26"
build_dir="${1:-$workspace/build/rgl_plugin}"
install_dir="${2:-$workspace/install/rgl_plugin}"

if [[ ! -f "$simulator/.gitmodules" ]]; then
  echo 'Import simulation.repos into src before building RGL.' >&2
  exit 1
fi
if [[ "$(git -C "$simulator" config --bool core.sparseCheckout || true)" == true ]]; then
  git -C "$simulator" sparse-checkout add external
fi
git -C "$simulator" submodule update --init external/RGLGazeboPlugin

# ROS setup files may reference unset environment variables.
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u
cmake -S "$simulator/external/RGLGazeboPlugin" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$install_dir"
cmake --build "$build_dir" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cmake --install "$build_dir"
echo "RGL libraries: $install_dir/RGLServerPlugin"
