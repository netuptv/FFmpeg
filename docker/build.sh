#! /bin/bash

set -ex

TARGET=ffmpeg
SCRIPT_DIR=$(cd $(dirname ${0}); pwd)
SRC_DIR=${SCRIPT_DIR}/..
REVISION=$(cd ${SRC_DIR}; git rev-parse HEAD)
OUT_DIR=$(pwd)/out/${TARGET}
CCACHE_DIR=$(pwd)/ccache/${TARGET}
IMAGE_TAG=$(head -n 1 ${SCRIPT_DIR}/build.tag)
IMAGE_NAME=build.netup:5000/iptv_2.0_build:${IMAGE_TAG}

mkdir -p ${OUT_DIR} ${CCACHE_DIR}

docker run --rm \
    --volume ${SRC_DIR}:/mnt/src \
    --volume ${OUT_DIR}:/mnt/out \
    --volume ${CCACHE_DIR}:/mnt/ccache \
    --user ${UID} \
    -t \
    ${IMAGE_NAME} \
    /mnt/src/docker/scripts/make.sh $@

printf "%s_revision=%s\n" "${TARGET}" "${REVISION}" > ${OUT_DIR}/build.info
