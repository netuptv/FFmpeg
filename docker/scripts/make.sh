#!/bin/sh

set -e

SCRIPT_DIR=$(cd $(dirname $0); pwd)
SRC_DIR=$(cd ${SCRIPT_DIR}/../..; pwd)
OUT_DIR=/opt/ffmpeg
BUILD_DIR=/mnt/build
CCACHE_DIR=/mnt/ccache

cd ${BUILD_DIR}

export PATH=/usr/lib/ccache/:${PATH}
export CCACHE_DIR

${SRC_DIR}/configure --prefix=${OUT_DIR} --enable-pic --enable-libmp3lame
make -j $(nproc) install-libs install-headers
