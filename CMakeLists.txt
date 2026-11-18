cmake_minimum_required(VERSION 2.8.3)
project(fast_lio)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

# Number of cores used for MP. Set to a positive integer to force a value
# (e.g. set(MP_CORES 4)); leave as set(MP_CORES "auto") for upstream behaviour.
set(MP_CORES 3)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(MP_CORES STREQUAL "auto")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
    if(N GREATER 4)
      set(MP_NUM 3)
    elseif(N GREATER 3)
      set(MP_NUM 2)
    else()
      set(MP_NUM 1)
    endif()
  else()
    set(MP_NUM 1)
  endif()
elseif(MP_CORES MATCHES "^[0-9]+$" AND MP_CORES GREATER 0)
  set(MP_NUM ${MP_CORES})
else()
  message(FATAL_ERROR "MP_CORES must be 'auto' or a positive integer")
endif()

if(MP_NUM GREATER 1)
  add_definitions(-DMP_EN)
endif()
add_definitions(-DMP_PROC_NUM=${MP_NUM})
message("core for MP: ${MP_NUM}")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
)

## Optional: Livox CustomMsg support (lidar_type: 1, Avia/Horizon/Mid-70).
## livox_ros_driver is NOT a hard dependency. It is auto-detected here, so it must be
## built in a workspace that is sourced BEFORE building fast_lio (an underlay); it is
## deliberately not declared in package.xml, so catkin will not order it for us.
find_package(livox_ros_driver QUIET)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

if(livox_ros_driver_FOUND)
  message(STATUS "fast_lio: livox_ros_driver found -- Livox CustomMsg support ENABLED (lidar_type: 1)")
  target_compile_definitions(fastlio_mapping PRIVATE HAVE_LIVOX)
  target_include_directories(fastlio_mapping SYSTEM PRIVATE ${livox_ros_driver_INCLUDE_DIRS})
  add_dependencies(fastlio_mapping ${livox_ros_driver_EXPORTED_TARGETS})
else()
  message(STATUS "fast_lio: livox_ros_driver NOT found -- Livox CustomMsg support DISABLED (lidar_type: 1 will exit with an error). "
                 "To enable it, build livox_ros_driver in a workspace, source it, then rebuild fast_lio.")
endif()