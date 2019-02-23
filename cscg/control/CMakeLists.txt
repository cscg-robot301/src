add_library(control SHARED
        hex_pos_control.cpp
        ControlMath.cpp)

target_link_libraries(control
        ${catkin_LIBRARIES} common)