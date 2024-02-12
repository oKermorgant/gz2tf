cmake_minimum_required(VERSION 3.5)

project(ros_gz_world_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# TODO(CH3): Deprecated. Remove on tock.
if("$ENV{GZ_VERSION}" STREQUAL "" AND NOT "$ENV{IGNITION_VERSION}" STREQUAL "")
  message(DEPRECATION "Environment variable [IGNITION_VERSION] is deprecated. Use [GZ_VERSION] instead.")
  set(ENV{GZ_VERSION} $ENV{IGNITION_VERSION})
endif()

# Harmonic
if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
  find_package(sdformat14 REQUIRED)
  set(SDF_VER ${sdformat14_VERSION_MAJOR})
  set(GZ_TARGET_PREFIX gz)
  set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
  set(GZ_SIM_PATH gz-sim8)
  message(STATUS "Compiling against Gazebo Harmonic")

# Garden
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(gz-transport12 REQUIRED)
  find_package(gz-msgs9 REQUIRED)
  find_package(sdformat13 REQUIRED)
  set(SDF_VER ${sdformat13_VERSION_MAJOR})
  set(GZ_TARGET_PREFIX gz)
  set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})
  set(GZ_SIM_PATH gz-sim7)
  message(STATUS "Compiling against Gazebo Garden")

# Default to Fortress
else()
  find_package(ignition-transport11 REQUIRED)
  find_package(ignition-msgs8 REQUIRED)
  find_package(sdformat12 REQUIRED)
  set(SDF_VER ${sdformat12_VERSION_MAJOR})
  set(GZ_TARGET_PREFIX ignition)
  set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
  set(GZ_SIM_PATH ignition-gazebo6)
  add_definitions(-DGZ_FORTRESS)
  message(STATUS "Compiling against Ignition Fortress")
endif()

add_definitions(-DGZ_SIM_SYS_PATH="/usr/share/${GZ_TARGET_PREFIX}/${GZ_SIM_PATH}/worlds")
add_definitions(-DGZ_SIM_HOME_PATH=".${GZ_TARGET_PREFIX}/fuel")
add_definitions(-DGZ_NS=${GZ_TARGET_PREFIX})

set(GZ_MSGS_VERSION_MAJOR ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_MAJOR})
set(GZ_MSGS_VERSION_MINOR ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_MINOR})
set(GZ_MSGS_VERSION_PATCH ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_PATCH})
set(GZ_MSGS_VERSION_FULL ${GZ_MSGS_VERSION_MAJOR}.${GZ_MSGS_VERSION_MINOR}.${GZ_MSGS_VERSION_PATCH})

ament_auto_add_executable(world_bridge world_bridge.cpp sdformat_urdf/sdformat_urdf/src/sdformat_urdf.cpp)
target_include_directories(world_bridge PRIVATE
    include
    ${GZ_TARGET_PREFIX}/sdformat${SDF_VER}
    ${GZ_TARGET_PREFIX}/msgs${GZ_MSGS_VER}
    ${GZ_TARGET_PREFIX}/transport${GZ_TRANSPORT_VER}
    sdformat_urdf/sdformat_urdf/include
    )

target_link_libraries(world_bridge
  ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
  sdformat${SDF_VER}::sdformat${SDF_VER}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package()
