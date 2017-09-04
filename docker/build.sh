#! /bin/bash

set -e

TARGET=ffmpeg
SCRIPT_DIR=$(cd $(dirname ${0}); pwd)
SRC_DIR=${SCRIPT_DIR}/..
REVISION=$(cd ${SRC_DIR}; git rev-parse HEAD)
[ -z "${BRANCH_NAME}" ] && \
    BRANCH_NAME=$(cd ${SRC_DIR}; git symbolic-ref -q --short HEAD || echo 'unknown')
OUT_DIR=$(pwd)/out/${TARGET}
BUILD_DIR=$(pwd)/build/${TARGET}
CCACHE_DIR=$(pwd)/ccache/${TARGET}
IMAGE_TAG=$(head -n 1 ${SCRIPT_DIR}/build.tag)
IMAGE_NAME=build.netup:5000/iptv_2.0_build:${IMAGE_TAG}

mkdir -p ${OUT_DIR} ${CCACHE_DIR} ${BUILD_DIR}

docker run --rm \
    --volume ${SRC_DIR}:/mnt/src:ro \
    --volume ${OUT_DIR}:/opt/ffmpeg \
    --volume ${BUILD_DIR}:/mnt/build \
    --volume ${CCACHE_DIR}:/mnt/ccache \
    --user ${UID} \
    -t \
    ${IMAGE_NAME} \
    /mnt/src/docker/scripts/make.sh $@

printf '%s_revision="%s %s %s"\n' "${TARGET}" "${REVISION}" "${BRANCH_NAME}" "${BUILD_URL}" > ${OUT_DIR}/build.info
