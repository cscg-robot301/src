

# Node for monocular camera
add_executable(Mono ros_mono.cc)
target_link_libraries(Mono ${LIBS} ${catkin_LIBRARIES})

add_executable(MonoAR
AR/ros_mono_ar.cc
AR/ViewerAR.h
AR/ViewerAR.cc)
target_link_libraries(MonoAR ${LIBS} ${catkin_LIBRARIES})

add_executable(Stereo ros_stereo.cc)
target_link_libraries(Stereo ${LIBS} ${catkin_LIBRARIES})

# Node for RGB-D camera
add_executable(RGBD ros_rgbd.cc)
target_link_libraries(RGBD ${LIBS} ${catkin_LIBRARIES})

