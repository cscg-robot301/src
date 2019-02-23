cmake_minimum_required(VERSION 2.8.3)
project(cscg)

#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#set(G2O_ROOT /usr/local/include/g2o)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_extras
  mavros_msgs
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  rosconsole
  stereo_msgs
  dynamic_reconfigure
  tf2_ros
  nodelet
  tf2_geometry_msgs
  message_generation
  diagnostic_updater
)

#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIRS})
find_package(serial REQUIRED)
include_directories(${serial_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

find_package(Pangolin REQUIRED)
include_directories( ${Pangolin_INCLUDE_DIRS} )

find_package(Boost REQUIRED COMPONENTS filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )

find_package(ZED 2.3)
include_directories( ${ZED_INCLUDE_DIRS})

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" ) # Jetson TX
    SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

find_package(CUDA)
include_directories(${CUDA_INCLUDE_DIRS})

find_package(OpenMP)
include_directories(${catkin_INCLUDE_DIRS})

if(NOT DEFINED CUDA_NPP_LIBRARIES_ZED)
    #To deal with cuda 9 nppi libs and previous versions of ZED SDK
    set(CUDA_NPP_LIBRARIES_ZED ${CUDA_npp_LIBRARY} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY})
endif()

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


catkin_package(
        CATKIN_DEPENDS
        roscpp
        rosconsole
        mavros_msgs
        sensor_msgs
        stereo_msgs
        image_transport
        dynamic_reconfigure
        tf2_ros
        tf2_geometry_msgs
        #message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  decision/include
  common
  control
  odemetry/include
  parameters
  drivers/camera/src/tools/include
  drivers/camera/src/nodelet/include
  drivers/camera
  drivers/robot_arm_driver/src
  drivers/sdk/include
  drivers/sdk/src
  drivers/Yesense
  Thirdparty/DBoW2/DBoW2
  Thirdparty/DBoW2/DUtils
  Thirdparty/g2o/g2o
)



set(LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/odemetry/lib/libORB_SLAM2.so
        -lboost_system
        )


add_subdirectory(common)
add_subdirectory(decision)
add_subdirectory(control)
add_subdirectory(drivers/robot_arm_driver)
add_subdirectory(drivers/sdk)
add_subdirectory(odemetry)
add_subdirectory(src/Odemetry_node)
add_subdirectory(drivers/camera)
add_subdirectory(drivers/Yesense)


#add_executable(yesense_node src/yesense_node.cpp ${SERIAL_SRC} ${YESENSE_SRC})
#target_link_libraries(yesense_node ${catkin_LIBRARIES} ${PROJECT_NAME} )