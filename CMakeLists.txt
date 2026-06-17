cmake_minimum_required(VERSION 3.12)
project(wbc_g1_deploy)

set(CMAKE_CXX_STANDARD 17)

set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)

list(APPEND CMAKE_PREFIX_PATH
  "$ENV{UNITREE_SDK_PREFIX}/lib/cmake"
  "/opt/unitree_robotics/lib/cmake"
  "$ENV{HOME}/.local/unitree_robotics/lib/cmake"
)
find_package(unitree_sdk2 REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ONNXRUNTIME_DIR "${REPO_ROOT}/thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ONNXRUNTIME_DIR "${REPO_ROOT}/thirdparty/onnxruntime-linux-aarch64-1.22.0")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(NOT EXISTS "${ONNXRUNTIME_DIR}/include/onnxruntime_c_api.h")
  message(FATAL_ERROR
    "ONNX Runtime not found at ${ONNXRUNTIME_DIR}. "
    "Run: scripts/bootstrap_thirdparty.sh")
endif()

include_directories(
  /usr/include/eigen3
  ${REPO_ROOT}/include
  ${ONNXRUNTIME_DIR}/include
  ${REPO_ROOT}/thirdparty/cnpy
)

file(GLOB CNPY_SRC ${REPO_ROOT}/thirdparty/cnpy/cnpy.cpp)
add_library(cnpy_lib STATIC ${CNPY_SRC})

link_libraries(
  unitree_sdk2
  rt
  pthread
  Boost::program_options
  yaml-cpp
  fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
  cnpy_lib
  ZLIB::ZLIB
)

set(WBC_LIB_SRC
  ${REPO_ROOT}/src/MotionClipLibrary.cpp
  ${REPO_ROOT}/src/State_WbcTracking.cpp
  ${REPO_ROOT}/src/WbcMotionLoader.cpp
  ${REPO_ROOT}/src/wbc_mdp_registrations.cpp
  ${REPO_ROOT}/src/motion_npz.cpp
  ${REPO_ROOT}/src/motion_reference_obs.cpp
  ${REPO_ROOT}/src/wbc_tracking_params.cpp
)

add_library(${PROJECT_NAME}_lib ${WBC_LIB_SRC})
link_libraries(${PROJECT_NAME}_lib)

add_subdirectory(apps)
