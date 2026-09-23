#!/usr/bin/env bash
# Build the model bundles that Dockerfile.cumotion bakes into the image.
#
# Run this on the host, where the driver workspace lives. Run it again only when
# the driver's URDF or SRDF changes -- model.json records a source_sha256 of both
# so a stale bundle can be told apart from a current one.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${ISAAC_ROS_WS:-$HOME/isaac_ros_ws}"
DRIVER_WORKSPACE="${DRIVER_WS:-$HOME/ros2_driver_ws}"
# Collision spheres come from the SDK's capsules, so its models must be present.
export RBY1_SDK_MODELS="${RBY1_SDK_MODELS:-$HOME/sdk/rby1-sdk/models}"
MODELS=("$@")
[[ ${#MODELS[@]} -eq 0 ]] && MODELS=(m_1_0 m_1_1 m_1_2 m_1_3 a_1_0 a_1_1 a_1_2)
TOLERANCE="${TOLERANCE:-0.02}"

# ROS setup scripts read unset variables, so keep nounset off while sourcing.
if [[ ! -d "${RBY1_SDK_MODELS}" ]]; then
    echo "RBY1 SDK models not found at ${RBY1_SDK_MODELS}" >&2
    echo "  set RBY1_SDK_MODELS to the rby1-sdk/models directory" >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source "${DRIVER_WORKSPACE}/install/setup.bash"
# Run straight from the source tree. The workspace's install/ is shared with the
# container, and a host build there would fight the container's own.
export PYTHONPATH="${HERE}/../rby1_cumotion:${PYTHONPATH:-}"

for model in "${MODELS[@]}"; do
    echo "== ${model} =="
    rm -rf "${HERE}/bundles/${model}"
    python3 -m rby1_cumotion.model \
        --model "${model}" --tolerance "${TOLERANCE}" \
        --output "${HERE}/bundles/${model}"
done

echo
du -sh "${HERE}/bundles"/*
echo "Bundles ready. Rebuild the image to bake them in."
