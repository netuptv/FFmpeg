#!/bin/sh

set -e

SCRIPT_DIR=$(cd $(dirname $0); pwd)
SRC_DIR=$(cd ${SCRIPT_DIR}/../..; pwd)
OUT_DIR=/opt/ffmpeg
OUT_X264_DIR=/opt/ffmpeg_x264
BUILD_DIR=/mnt/build/ffmpeg
BUILD_X264_DIR=/mnt/build/ffmpeg_x264
CCACHE_DIR=/mnt/ccache

export PATH=/usr/lib/ccache/:${PATH}
export CCACHE_DIR

mkdir -p "${BUILD_DIR}" "${BUILD_X264_DIR}" "${OUT_DIR}" "${BUILD_X264_DIR}"

OPTS=$( getopt -o '' --long "debug" -n "$( basename ${0} )" -- "$@" )
eval set -- "${OPTS}"

DEBUG=

while true; do
    case "${1}" in
        --debug)
            DEBUG="--enable-debug=3 --disable-optimizations --disable-stripping"
            ;;
        --)
            shift
            break
    esac
    shift
done

cd ${BUILD_DIR}
${SRC_DIR}/configure \
    --prefix=${OUT_DIR} \
    --enable-pic \
    --enable-libmp3lame \
    --disable-vaapi \
    --enable-openssl \
    --disable-programs \
    --enable-ffprobe \
    --disable-doc \
    --enable-libzvbi \
    --enable-shared \
    ${DEBUG}
make -j $(nproc) install-libs install-headers install-progs

cd ${BUILD_X264_DIR}
${SRC_DIR}/configure \
    --prefix=${OUT_X264_DIR} \
    --enable-pic \
    --enable-libmp3lame \
    --disable-vaapi \
    --disable-programs \
    --enable-ffprobe \
    --disable-doc \
    --enable-libzvbi \
    --enable-shared \
    --disable-nvenc \
    --enable-libx264 \
    --enable-gpl \
    ${DEBUG}
make -j $(nproc) install-libs install-headers
