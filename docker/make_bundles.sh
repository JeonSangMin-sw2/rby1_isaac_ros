#!/usr/bin/env bash
# Stage everything Dockerfile.cumotion takes from the driver side of the host:
#   bundles/<model>/   model bundles (URDF/SRDF + SDK capsules)
#   driver_msgs/       rby1_msgs interface definitions, so container nodes can
#                      call the driver's services and actions
#
# Required once before the first image build: both directories are gitignored,
# so a fresh clone has neither and the image would bake empty ones.
# Run it again when the driver's URDF, SRDF or rby1_msgs change -- model.json
# records a source_sha256 so a stale bundle can be told apart from a current one.
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

# Interface definitions only -- the driver itself, and the SDK it links, stay on
# the host. The container needs the types to talk to the driver over DDS.
MSGS_SOURCE="${DRIVER_WORKSPACE}/src/rby1_ros2/rby1_msgs"
if [[ ! -f "${MSGS_SOURCE}/package.xml" ]]; then
    echo "rby1_msgs not found at ${MSGS_SOURCE}" >&2
    exit 1
fi
rm -rf "${HERE}/driver_msgs"
mkdir -p "${HERE}/driver_msgs"
cp -r "${MSGS_SOURCE}" "${HERE}/driver_msgs/rby1_msgs"

echo
du -sh "${HERE}/bundles"/* "${HERE}/driver_msgs"
echo "Staged. Rebuild the image to bake them in."
