#! /bin/bash

set -e

SCRIPT_DIR="$(realpath "$(dirname "${0}")")"
SRC_DIR="${SCRIPT_DIR}/.."
OUT_DIR="$(pwd)/out/ffmpeg"
BUILD_DIR="$(pwd)/build/ffmpeg"
CCACHE_DIR="$(pwd)/ccache/ffmpeg"
REVISION="$(cd ${SRC_DIR}; git rev-parse HEAD)"
[ -z "${BRANCH_NAME}" ] && \
    BRANCH_NAME=$(cd ${SRC_DIR}; git symbolic-ref -q --short HEAD || echo 'unknown')
if [ -z "${BUILD_NUMBER}" ]; then
    BUILD_NUMBER="$( date --utc '+%Y%m%dT%H%M' )-$( id -un )"
fi
TAG="$( sed 's#/#_#g' <<<"${BRANCH_NAME}" ).${BUILD_NUMBER}"

mkdir -p "${OUT_DIR}" "${CCACHE_DIR}" "${BUILD_DIR}"

. "${SCRIPT_DIR}/image/image.env"
SNAPSHOT_URL="http://snapshot.debian.org/archive/debian/${SNAPSHOT}"
docker build --force-rm --iidfile "${BUILD_DIR}/image.id" - <<EOF
FROM debian:${IMAGE_TAG}

RUN echo 'deb ${SNAPSHOT_URL} stretch main' > /etc/apt/sources.list && \
    apt-get update && \
    apt-get install --no-install-recommends --assume-yes \
        gcc make libssl1.0-dev libc6-dev yasm libmp3lame-dev pkg-config \
        libzvbi-dev && \
    apt-get install --no-install-recommends --assume-yes ccache && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
EOF
IMAGE_ID="$( head -n1 "${BUILD_DIR}/image.id" )"

docker run --rm \
    --volume ${SRC_DIR}:/mnt/src:ro \
    --volume ${OUT_DIR}:/opt/ffmpeg \
    --volume ${BUILD_DIR}:/mnt/build \
    --volume ${CCACHE_DIR}:/mnt/ccache \
    --user ${UID} \
    --tty \
    ${IMAGE_ID} \
    /mnt/src/docker/scripts/make.sh $@

printf 'ffmpeg_revision="%s %s %s"\n' "${REVISION}" "${BRANCH_NAME}" "${BUILD_URL}" > ${OUT_DIR}/build.info
