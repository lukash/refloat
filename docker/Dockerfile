FROM ubuntu:latest

# Step 1. Install Prerequisites
RUN : \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get install -y --no-install-recommends \
        git wget python3 python-is-python3 make gcc-arm-none-eabi libnewlib-arm-none-eabi \
        libsdl2-2.0-0 libsm6 libxcb-icccm4 libxcb-image0 \
        libxcb-keysyms1 libxcb-render-util0 libxcb-shape0 libxcb-xinerama0 \
        libxcb-xkb1 libxkbcommon-x11-0 libegl1 libgles2 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && :