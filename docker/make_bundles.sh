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
MODELS=("$@")
[[ ${#MODELS[@]} -eq 0 ]] && MODELS=(m_1_0 m_1_1 m_1_2 m_1_3 a_1_0 a_1_1 a_1_2)
TOLERANCE="${TOLERANCE:-0.02}"

# ROS setup scripts read unset variables, so keep nounset off while sourcing.
source /opt/ros/humble/setup.bash

# The driver workspace: DRIVER_WS, else the one this shell has sourced, else the
# only one under $HOME. Several there is an error, not a guess.
if [[ -n "${DRIVER_WS:-}" ]]; then
    DRIVER_WORKSPACE="${DRIVER_WS}"
elif prefix="$(ros2 pkg prefix rby1_msgs 2>/dev/null)"; then
    DRIVER_WORKSPACE="${prefix%/install*}"
else
    mapfile -t found < <(find "${HOME}" -maxdepth 5 -type d -path '*/install/rby1_msgs' 2>/dev/null)
    if [[ ${#found[@]} -ne 1 ]]; then
        echo "Driver workspace: ${#found[@]} found under ${HOME} (a built one has install/rby1_msgs)" >&2
        for one in "${found[@]}"; do echo "  ${one%/install/rby1_msgs}" >&2; done
        echo "  set DRIVER_WS to the one to use" >&2
        exit 1
    fi
    DRIVER_WORKSPACE="${found[0]%/install/rby1_msgs}"
fi
if [[ ! -f "${DRIVER_WORKSPACE}/install/setup.bash" ]]; then
    echo "No built driver workspace at ${DRIVER_WORKSPACE} (install/setup.bash is missing)" >&2
    echo "  build it, or set DRIVER_WS" >&2
    exit 1
fi

# Collision spheres come from the SDK's capsules, so its models must be present:
# RBY1_SDK_MODELS, else the SDK of RBY1_SDK_PATH (what the driver build reads), else
# the SDK the built driver links (its RUNPATH), else ~/sdk/rby1-sdk.
if [[ -z "${RBY1_SDK_MODELS:-}" ]]; then
    linked="$(readelf -d "${DRIVER_WORKSPACE}/install/rby1_driver/lib/rby1_driver/rby1_ros2_driver" \
        2>/dev/null | grep -o '[^:[]*/build/src' | head -1 || true)"
    for sdk in "${RBY1_SDK_PATH:-}" "${linked%/build/src}" "${HOME}/sdk/rby1-sdk"; do
        if [[ -n "${sdk}" && -d "${sdk}/models" ]]; then
            RBY1_SDK_MODELS="${sdk}/models"
            break
        fi
    done
fi
if [[ ! -d "${RBY1_SDK_MODELS:-}" ]]; then
    echo "RBY1 SDK models not found at ${RBY1_SDK_MODELS:-RBY1_SDK_PATH, the SDK the driver links, or ~/sdk/rby1-sdk}" >&2
    echo "  set RBY1_SDK_MODELS to the rby1-sdk/models directory" >&2
    exit 1
fi
export RBY1_SDK_MODELS
echo "driver workspace: ${DRIVER_WORKSPACE}"
echo "SDK models:       ${RBY1_SDK_MODELS}"

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
MSGS_XML="$(find "${DRIVER_WORKSPACE}/src" -path '*/rby1_msgs/package.xml' -print -quit)"
if [[ -z "${MSGS_XML}" ]]; then
    echo "rby1_msgs not found under ${DRIVER_WORKSPACE}/src" >&2
    exit 1
fi
MSGS_SOURCE="$(dirname "${MSGS_XML}")"
rm -rf "${HERE}/driver_msgs"
mkdir -p "${HERE}/driver_msgs"
cp -r "${MSGS_SOURCE}" "${HERE}/driver_msgs/rby1_msgs"

echo
du -sh "${HERE}/bundles"/* "${HERE}/driver_msgs"
echo "Staged. Rebuild the image to bake them in."
