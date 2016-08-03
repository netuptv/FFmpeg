#!/bin/sh

set -ex

SCRIPT_DIR=$(cd $(dirname $0); pwd)
SRC_DIR=$(cd ${SCRIPT_DIR}/../..; pwd)
OUT_DIR=/opt/ffmpeg
CCACHE_DIR=/mnt/ccache

cd ${SRC_DIR}

export PATH=/usr/lib/ccache/:${PATH}
export CCACHE_DIR

./configure --prefix=${OUT_DIR}
make -j $(nproc)
make install-libs install-headers
