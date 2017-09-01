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

OPTS=$( getopt -o '' --long "shared" -n "$( basename ${0} )" -- "$@" )
eval set -- "${OPTS}"

SHARED=

while true; do
    case "${1}" in
        --shared)
            SHARED=--enable-shared
            ;;
        --)
            shift
            break
    esac
    shift
done

${SRC_DIR}/configure --prefix=${OUT_DIR} --enable-pic --enable-libmp3lame --disable-vaapi ${SHARED}
make -j $(nproc) install-libs install-headers
