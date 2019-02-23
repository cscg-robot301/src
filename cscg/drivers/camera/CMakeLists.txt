
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

add_library(CAMERA SHARED
        src/tools/src/sl_tools.cpp
        src/nodelet/src/zed_wrapper_nodelet.cpp
        )

target_link_libraries(CAMERA
        ${catkin_LIBRARIES}
        ${ZED_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${CUDA_NPP_LIBRARIES_ZED}
        )

#add_executable(camera_node src/zed_wrapper_node.cpp)
#target_link_libraries(camera_node CAMERA)