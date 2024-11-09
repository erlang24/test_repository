#!/bin/bash
#coding=utf-8

#VERSION="123" # 版本号
#DATETIME=`date -d today +"%Y%m%d%H%M"`               # 时间

SELF_RELATIVE_DIR=`dirname $0`                       # 获取 脚本文件所在的相对路径
#SELF_ABSOLUTE_DIR=$(readlink -f "$SELF_RELATIVE_DIR")
SELF_ABSOLUTE_DIR=`readlink -f "$SELF_RELATIVE_DIR"` # 当前 脚本文件，所在的绝对路径


#echo "================================================="
#echo 123456 | sudo -S busybox devmem 0x0c303018 w 0xc458
#sudo busybox devmem 0x0c303010 w 0xc400
#sudo busybox devmem 0x0c303008 w 0xc458
#sudo busybox devmem 0x0c303000 w 0xc400
#sudo modprobe can
#sudo modprobe can_raw
#sudo modprobe mttcan
#sudo ip link set down can0
#sudo ip link set can0 up type can bitrate 500000
#echo "================================================="

# Default settings
CUDA="on"
IMAGE_NAME="autoware/autoware"
TAG_PREFIX="latest"
ROS_DISTRO="melodic"
BASE_ONLY="false"
PRE_RELEASE="off"
AUTOWARE_HOST_DIR=""
USER_ID="$(id -u)"

function usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "    -b,--base-only <AUTOWARE_HOST_DIR> If provided, run the base image only and mount the provided Autoware folder."
    echo "                                       Default: Use pre-compiled Autoware image"
    echo "    -c,--cuda <on|off>                 Enable Cuda support in the Docker."
    echo "                                       Default: $CUDA"
    echo "    -h,--help                          Display the usage and exit."
    echo "    -i,--image <name>                  Set docker images name."
    echo "                                       Default: $IMAGE_NAME"
    echo "    -p,--pre-release <on|off>          Use pre-release image."
    echo "                                       Default: $PRE_RELEASE"
    echo "    -r,--ros-distro <name>             Set ROS distribution name."
    echo "                                       Default: $ROS_DISTRO"
    echo "    -s,--skip-uid-fix                  Skip uid modification step required when host uid != 1000"
    echo "    -t,--tag-prefix <tag>              Tag prefix use for the docker images."
    echo "                                       Default: $TAG_PREFIX"
}

# Convert a relative directory path to absolute
function abspath() {
    local path=$1
    if [ ! -d $path ]; then
	exit 1
    fi
    pushd $path > /dev/null
    echo $(pwd)
    popd > /dev/null
}


OPTS=`getopt --options b:c:hi:p:r:st: \
         --long base-only:,cuda:,help,image-name:,pre-release:,ros-distro:,skip-uid-fix,tag-prefix: \
         --name "$0" -- "$@"`
eval set -- "$OPTS"

while true; do
  case $1 in
    -b|--base-only)
      BASE_ONLY="true"
      AUTOWARE_HOST_DIR=$(abspath "$2")
      shift 2
      ;;
    -c|--cuda)
      param=$(echo $2 | tr '[:upper:]' '[:lower:]')
      case "${param}" in
        "on"|"off") CUDA="${param}" ;;
        *) echo "Invalid cuda option: $2"; exit 1 ;;
      esac
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -i|--image-name)
      IMAGE_NAME="$2"
      shift 2
      ;;
    -p|--pre-release)
      param=$(echo $2 | tr '[:upper:]' '[:lower:]')
      case "${param}" in
        "on"|"off") PRE_RELEASE="${param}" ;;
        *) echo "Invalid pre-release option: $2"; exit 1 ;;
      esac
      shift 2
      ;;
    -r|--ros-distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    -s|--skip-uid-fix)
      USER_ID=1000
      shift 1
      ;;
    -t|--tag-prefix)
      TAG_PREFIX="$2"
      shift 2
      ;;
    --)
      if [ ! -z $2 ];
      then
        echo "Invalid parameter: $2"
        exit 1
      fi
      break
      ;;
    *)
      echo "Invalid option"
      exit 1
      ;;
  esac
done

echo "Using options:"
echo -e "\tROS distro: $ROS_DISTRO"
echo -e "\tImage name: $IMAGE_NAME"
echo -e "\tTag prefix: $TAG_PREFIX"
echo -e "\tCuda support: $CUDA"
if [ "$BASE_ONLY" == "true" ]; then
  echo -e "\tAutoware Home: $AUTOWARE_HOST_DIR"
fi
echo -e "\tPre-release version: $PRE_RELEASE"
echo -e "\tUID: <$USER_ID>"

SUFFIX=""
RUNTIME=""

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
FONTS=/usr/share/fonts # 字体问题


SHARED_DOCKER_DIR=${SELF_ABSOLUTE_DIR}/../../
SHARED_HOST_DIR=${SELF_ABSOLUTE_DIR}/../../

AUTOWARE_DOCKER_DIR=/home/promote/autoware.docker

  
VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=$XAUTH:$XAUTH:rw
         --volume=/dev:/dev:rw         
         --volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw
         --volume=$HOME/autoware_data:/root/autoware_data
         --volume=$FONTS:$FONTS:rw" # 字体问题

DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)
if [ $CUDA == "on" ]; then
    SUFFIX=$SUFFIX"-cuda"
    if [[ ! $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
        RUNTIME="--gpus all"
    else
        RUNTIME="--runtime=nvidia"
    fi
fi

if [ $PRE_RELEASE == "on" ]; then
    SUFFIX=$SUFFIX"-rc"
fi

# Create the shared directory in advance to ensure it is owned by the host user
mkdir -p $SHARED_HOST_DIR


if [ "$(uname -m)" = "aarch64" ]; then
  echo "platform: aarch64"
  IMAGE_NAME="pm-pilot-stage1-promote-universe-v2x-image"
  TAG_PREFIX="humble-latest-cuda-arm64-autopilot-v2x-tag"
  RUNTIME="--runtime=nvidia"
fi

if [ "$(uname -m)" = "x86_64" ]; then
  echo "platform: amd64"
  IMAGE_NAME="pm-pilot-stage1-promote-universe-v2x-image"
  TAG_PREFIX="humble-latest-cuda-amd64-autopilot-v2x-tag"
fi

IMAGE_NAME="192.168.2.100:8086/pmpilot-4.0/${IMAGE_NAME}"

IMAGE="${IMAGE_NAME}:${TAG_PREFIX}"

echo "Launching $IMAGE"

# xhost + 
# --env="ROS_DOMAIN=55" \
docker run \
    -it  \
    --rm \
    --name="xiaoche-humble" \
    --workdir=${SELF_ABSOLUTE_DIR} \
    $VOLUMES \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="USER_ID=$USER_ID" \
    --privileged \
    --net=host \
    --ipc=host \
    $RUNTIME \
    ${IMAGE} \
    /bin/bash


