add_library(robotArmCom
        src/serial.cpp
        )

add_executable(robot_arm_node src/serial_robot.cpp )
target_link_libraries(robot_arm_node ${catkin_LIBRARIES} robotArmCom)
