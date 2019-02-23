add_library(yesense
        yesense.cpp)
target_link_libraries(yesense ${serial_LIBRARIES} ${catkin_LIBRARIES})


add_executable(yesense_node yesense_node.cpp)
target_link_libraries(yesense_node ${catkin_LIBRARIES} ${serial_LIBRARIES} yesense)