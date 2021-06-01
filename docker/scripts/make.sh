#!/bin/sh

set -e

SCRIPT_DIR=$(cd $(dirname $0); pwd)
SRC_DIR=$(cd ${SCRIPT_DIR}/../..; pwd)
OUT_FFMPEG_DIR=/opt/ffmpeg
OUT_X264_DIR=/opt/ffmpeg_x264
OUT_SRT_DIR=/opt/srt
BUILD_DIR=/mnt/build
BUILD_FFMPEG_DIR="${BUILD_DIR}/ffmpeg"
BUILD_X264_DIR="${BUILD_DIR}/x264"
BUILD_SRT_DIR="${BUILD_DIR}/srt"
CCACHE_DIR=/mnt/ccache

export PATH=/usr/lib/ccache/:${PATH}
export CCACHE_DIR

mkdir -p "${BUILD_FFMPEG_DIR}" \
         "${BUILD_X264_DIR}" \
         "${BUILD_SRT_DIR}" \
         "${OUT_FFMPEG_DIR}" \
         "${OUT_X264_DIR}" \
         "${OUT_SRT_DIR}"

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

cd "${BUILD_SRT_DIR}"
if ! [ -d src ]; then
    git clone --branch=v1.4.1 --depth=1 https://github.com/Haivision/srt.git src
fi

mkdir -p build
cd build
../src/configure \
    --prefix="${OUT_SRT_DIR}" \
    --enable-apps=OFF
make -j $(nproc) install

cd "${BUILD_FFMPEG_DIR}"
PKG_CONFIG_PATH="${OUT_SRT_DIR}/lib/pkgconfig" ${SRC_DIR}/configure \
    --prefix=${OUT_FFMPEG_DIR} \
    --enable-pic \
    --enable-libmp3lame \
    --disable-vaapi \
    --enable-openssl \
    --disable-programs \
    --disable-ffmpeg \
    --enable-ffprobe \
    --disable-ffplay \
    --disable-doc \
    --enable-libzvbi \
    --enable-shared \
    --enable-libxml2 \
    --enable-libsrt \
    --enable-demuxer=dash \
    ${DEBUG}
make -j $(nproc) install-libs install-headers install-progs

cd ${BUILD_X264_DIR}
${SRC_DIR}/configure \
    --prefix=${OUT_X264_DIR} \
    --disable-programs \
    --disable-doc \
    --disable-nvenc \
    --disable-vaapi \
    --disable-everything \
    --enable-decoders \
    --enable-filter=yadif \
    --enable-encoders \
    --enable-swscale \
    --enable-swresample \
    --enable-libx264 \
    --enable-libx265 \
    --enable-gpl \
    ${DEBUG}
make -j $(nproc) install-libs install-headers
