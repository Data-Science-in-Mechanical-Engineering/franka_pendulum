cmake_minimum_required(VERSION 3.0.2)
project(franka_pendulum_example)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  franka_pendulum
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS franka_pendulum
)

###########
## Build ##
###########

## Declare a C++ library
add_library(franka_pendulum_example_controllers SHARED
src/hybrid_controller.cpp)
target_include_directories(franka_pendulum_example_controllers PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(franka_pendulum_example_controllers PUBLIC ${catkin_LIBRARIES})
add_dependencies(franka_pendulum_example_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})