add_library(ladar
        src/arch/linux/net_serial.cpp
        src/arch/linux/net_socket.cpp
        src/arch/linux/timer.cpp
        src/hal/thread.cpp
        src/rplidar_driver.cpp
        )

add_executable(rplidar_A2 rplidar_A2.cpp)
target_link_libraries(rplidar_A2 ${catkin_LIBRARIES} ladar)

add_executable(rplidar_client rplidar_client.cpp )
target_link_libraries(rplidar_client ${catkin_LIBRARIES} ladar)