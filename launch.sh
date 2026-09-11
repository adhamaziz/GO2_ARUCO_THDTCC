#!/usr/bin/env bash
# Launch script for the go2-aruco container.
#
# Usage:
#   ./launch.sh                              # DICT_4X4_50, 5cm markers, defaults
#   ./launch.sh --marker-size 0.10           # 10cm markers
#   ./launch.sh --dict DICT_6X6_250
#   ./launch.sh --build                      # rebuild first, then launch
#
set -e

IMAGE_NAME="go2-aruco:latest"
CONTAINER_NAME="aruco_go2"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
INPUT_TOPIC="/camera/color/image_raw"
CAMERA_INFO_TOPIC="/camera/color/camera_info"
IMAGE_TOPIC="/go2/aruco/image_annotated"
POSES_TOPIC="/go2/aruco/poses"
IDS_TOPIC="/go2/aruco/marker_ids"
DICTIONARY="DICT_4X4_50"
MARKER_SIZE="0.05"
DO_BUILD=0

usage() {
  echo "Usage: $0 [--dict DICT_4X4_50] [--marker-size 0.05] [--topic /camera/color/image_raw] [--build]"
  exit 1
}

while [ $# -gt 0 ]; do
  case "$1" in
    --dict)         DICTIONARY="$2"; shift 2 ;;
    --marker-size)  MARKER_SIZE="$2"; shift 2 ;;
    --topic)        INPUT_TOPIC="$2"; shift 2 ;;
    --build)        DO_BUILD=1; shift ;;
    -h|--help)      usage ;;
    *) echo "Unknown option: $1"; usage ;;
  esac
done

if [ "${DO_BUILD}" = "1" ] || ! docker image inspect "${IMAGE_NAME}" > /dev/null 2>&1; then
  echo "==> Building ${IMAGE_NAME}"
  docker build -f Dockerfile.aruco -t "${IMAGE_NAME}" .
fi

echo "==> Checking for ${INPUT_TOPIC} on the host ROS2 graph (5s timeout)..."
if command -v ros2 > /dev/null 2>&1; then
  if timeout 5 ros2 topic list 2>/dev/null | grep -qx "${INPUT_TOPIC}"; then
    echo "    Found it -- realsense node looks like it's running."
  else
    echo "    WARNING: ${INPUT_TOPIC} not seen on the host. Is your realsense node running?"
    echo "    Continuing anyway -- the container will just wait for frames."
  fi
else
  echo "    (ros2 CLI not on PATH in this shell, skipping check)"
fi

echo "==> Launching ${CONTAINER_NAME}"
echo "    ROS_DOMAIN_ID   = ${ROS_DOMAIN_ID}"
echo "    input_topic     = ${INPUT_TOPIC}"
echo "    dictionary      = ${DICTIONARY}"
echo "    marker_size     = ${MARKER_SIZE} m"
echo "    output image    = ${IMAGE_TOPIC}"
echo "    output poses    = ${POSES_TOPIC}"
echo

exec docker run -it --rm \
  --network host \
  --name "${CONTAINER_NAME}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  "${IMAGE_NAME}" \
  --ros-args \
    -p "input_topic:=${INPUT_TOPIC}" \
    -p "camera_info_topic:=${CAMERA_INFO_TOPIC}" \
    -p "image_topic:=${IMAGE_TOPIC}" \
    -p "poses_topic:=${POSES_TOPIC}" \
    -p "ids_topic:=${IDS_TOPIC}" \
    -p "dictionary:=${DICTIONARY}" \
    -p "marker_size:=${MARKER_SIZE}"