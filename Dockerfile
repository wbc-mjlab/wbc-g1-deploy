# syntax=docker/dockerfile:1
#
# Self-contained build of the Unitree G1 WBC deploy controller.
#
#   docker build -t wbc-g1-deploy .
#   docker run --rm wbc-g1-deploy                    # prints --help (default CMD)
#   docker run --rm --network host wbc-g1-deploy \
#       build/wbc_g1_ctrl --network=eth0             # run against a robot (DDS domain 0)
#
# The image installs the system toolchain + libs, builds and installs
# unitree_sdk2 into /opt/unitree_robotics (a prefix the app's CMake checks),
# downloads ONNX Runtime via scripts/bootstrap_thirdparty.sh, then builds the
# controller and smoke-tests it -- all with no robot attached.
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
# Pin the unitree_sdk2 revision here to make image builds reproducible.
ARG UNITREE_SDK_REF=main

# Build toolchain + runtime dependencies (see README "Dependencies").
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      curl \
      ca-certificates \
      libyaml-cpp-dev \
      libboost-program-options-dev \
      libeigen3-dev \
      libspdlog-dev \
      libfmt-dev \
      zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

# Build + install Unitree SDK2 into /opt/unitree_robotics (checked by the app's
# CMakeLists). This also installs the bundled CycloneDDS shared libraries.
RUN git clone --depth 1 --branch "${UNITREE_SDK_REF}" \
      https://github.com/unitreerobotics/unitree_sdk2.git /tmp/unitree_sdk2 \
    && cmake -S /tmp/unitree_sdk2 -B /tmp/unitree_sdk2/build \
         -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics \
    && cmake --build /tmp/unitree_sdk2/build -j"$(nproc)" \
    && cmake --install /tmp/unitree_sdk2/build \
    && rm -rf /tmp/unitree_sdk2 \
    && echo /opt/unitree_robotics/lib > /etc/ld.so.conf.d/unitree.conf \
    && ldconfig

WORKDIR /opt/wbc-g1-deploy
COPY . .

# Fetch ONNX Runtime, then configure and build the controller.
RUN scripts/bootstrap_thirdparty.sh \
    && cmake -S . -B build \
    && cmake --build build -j"$(nproc)"

# ONNX Runtime (linked by absolute path) and the Unitree/DDS libs already
# resolve via RPATH + ldconfig; keep them on the loader path explicitly too.
ENV LD_LIBRARY_PATH=/opt/unitree_robotics/lib:/opt/wbc-g1-deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib

CMD ["build/wbc_g1_ctrl", "--help"]
