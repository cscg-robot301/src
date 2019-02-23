add_library(decision
        uavPlanning.cpp)

#include_directories(uavPlanning.h)

target_link_libraries(decision
        ${catkin_LIBRARIES}
        ${Ceres_LIBRARIES}
        ${G2O_LIBRARIES})

add_executable(armtest robotArm.cpp)
target_link_libraries(armtest
        ${catkin_LIBRARIES}
        ${Ceres_LIBRARIES}
        ${G2O_LIBRARIES}
        decision)