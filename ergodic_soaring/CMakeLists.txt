cmake_minimum_required(VERSION 2.8.3)
project(ergodic_soaring)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  grid_map_core
  grid_map_cv
  grid_map_msgs
  grid_map_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs sensor_msgs tf
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/ergodic_soaring.cpp
  src/ergodic_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
add_executable(ergodic_soaring_node
  src/ergodic_soaring_node.cpp
)
target_link_libraries(ergodic_soaring_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_ergodic_map_node
  src/test_ergodic_map_node.cpp
)
target_link_libraries(test_ergodic_map_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_ergodic_trajectory_node
  src/test_ergodic_trajectory_node.cpp
)
target_link_libraries(test_ergodic_trajectory_node ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# TESTING#
##########

if(CATKIN_ENABLE_TESTING)
    # Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
                                          test/test_example.cpp
                                          test/test_ergodiccontrol.cpp
    )
    if(TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
                                                 ${catkin_LIBRARIES}
        )
    endif()

endif()
