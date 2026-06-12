cmake_minimum_required(VERSION 3.20)

project(obs-nyan-real-3dof VERSION 0.6.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(OBS_SRC "${CMAKE_SOURCE_DIR}/deps/obs-studio" CACHE PATH "Path to obs-studio source tree")
set(OBS_LIB "${CMAKE_SOURCE_DIR}/deps/obslib/obs.lib" CACHE FILEPATH "Path to obs.lib import library")
set(OBS_FRONTEND_LIB "${CMAKE_SOURCE_DIR}/deps/obslib/obs-frontend-api.lib" CACHE FILEPATH "Path to obs-frontend-api.lib import library")
set(QT_ROOT "${CMAKE_SOURCE_DIR}/deps/qt/6.8.3/msvc2022_64" CACHE PATH "Optional Qt root for dock UI")

if(EXISTS "${QT_ROOT}/lib/cmake/Qt6/Qt6Config.cmake")
  list(APPEND CMAKE_PREFIX_PATH "${QT_ROOT}")
endif()
find_package(Qt6 COMPONENTS Widgets QUIET)

add_library(obs-nyan-real-3dof MODULE)
target_sources(obs-nyan-real-3dof PRIVATE
  src/plugin-main.cpp
  src/cursor_fence.cpp
  src/device_manager.cpp
  src/device_registry.cpp
  src/hid_io.cpp
  src/dock.cpp
  src/virtual_source.cpp
  src/display-wall-source.cpp
  src/audio-wall-source.cpp
  src/ws_audio_server.cpp
  src/spatial_audio_filter.cpp
  src/transports/xreal_tcp.cpp
  src/transports/xreal_air.cpp
  src/transports/rayneo.cpp
  src/transports/moverio.cpp
  src/transports/rokid.cpp
  src/transports/viture.cpp
  src/transports/nreal_light.cpp
  src/marker_tracker.cpp)

# apriltag (BSD-2-Clause), cloned by setup.ps1 into deps/apriltag. Only the
# tag36h11 family is compiled; getopt is excluded (unused, POSIX-flavored).
set(APRILTAG_DIR "${CMAKE_SOURCE_DIR}/deps/apriltag")
file(GLOB APRILTAG_COMMON_SOURCES "${APRILTAG_DIR}/common/*.c")
list(FILTER APRILTAG_COMMON_SOURCES EXCLUDE REGEX "getopt")
add_library(apriltag_static STATIC
  "${APRILTAG_DIR}/apriltag.c"
  "${APRILTAG_DIR}/apriltag_pose.c"
  "${APRILTAG_DIR}/apriltag_quad_thresh.c"
  "${APRILTAG_DIR}/tag36h11.c"
  ${APRILTAG_COMMON_SOURCES})
target_include_directories(apriltag_static PUBLIC "${APRILTAG_DIR}")
if(MSVC)
  # No WIN32_LEAN_AND_MEAN here: apriltag's time_util needs struct timeval
  # from the winsock header that plain windows.h pulls in.
  target_compile_definitions(apriltag_static PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
  target_compile_options(apriltag_static PRIVATE /utf-8)
endif()

target_include_directories(obs-nyan-real-3dof PRIVATE
  "${OBS_SRC}/libobs"
  "${OBS_SRC}/frontend/api"
  "${CMAKE_SOURCE_DIR}/deps/gen")

target_link_libraries(obs-nyan-real-3dof PRIVATE
  "${OBS_LIB}"
  "${OBS_FRONTEND_LIB}"
  Ws2_32
  Iphlpapi
  User32
  SetupAPI
  Hid
  Sensorsapi
  PortableDeviceGuids
  Ole32
  Advapi32
  apriltag_static
  Mf
  Mfplat
  Mfreadwrite
  Mfuuid)

if(Qt6_FOUND)
  target_link_libraries(obs-nyan-real-3dof PRIVATE Qt6::Widgets)
  target_compile_definitions(obs-nyan-real-3dof PRIVATE NYAN_REAL_3DOF_WITH_QT_DOCK=1)
  message(STATUS "nyan Real 3DoF dock UI: enabled (Qt ${Qt6_VERSION})")
else()
  message(WARNING "nyan Real 3DoF dock UI: disabled (Qt6 Widgets not found)")
endif()

set_target_properties(obs-nyan-real-3dof PROPERTIES PREFIX "" OUTPUT_NAME "obs-nyan-real-3dof")

if(NOT DEFINED PLUGIN_VERSION OR PLUGIN_VERSION STREQUAL "")
  set(PLUGIN_VERSION "${PROJECT_VERSION}-dev")
endif()
# Configure-time stamp with an explicit timezone: __DATE__/__TIME__ would use
# the build machine's local time (UTC on CI, JST locally), which reads as a
# 9-hour offset depending on where the binary was built.
string(TIMESTAMP PLUGIN_BUILD_TIME "%Y-%m-%d %H:%M UTC" UTC)
target_compile_definitions(obs-nyan-real-3dof PRIVATE
  NYAN_REAL_3DOF_VERSION="${PLUGIN_VERSION}"
  NYAN_REAL_3DOF_BUILD_TIME="${PLUGIN_BUILD_TIME}")
message(STATUS "obs-nyan-real-3dof version: ${PLUGIN_VERSION}")

if(MSVC)
  target_compile_definitions(obs-nyan-real-3dof PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
  target_compile_options(obs-nyan-real-3dof PRIVATE /utf-8 /EHsc)
endif()
