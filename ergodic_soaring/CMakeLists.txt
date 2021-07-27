cmake_minimum_required(VERSION 2.8.3)
project(ergodic_soaring)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/ergodic_soaring.cpp
  src/ergodic_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(ergodic_soaring_node
  src/ergodic_soaring_node.cpp
)
target_link_libraries(ergodic_soaring_node ${PROJECT_NAME} ${catkin_LIBRARIES})
##########
# EXPORT #
##########
cs_install()
cs_export()

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
