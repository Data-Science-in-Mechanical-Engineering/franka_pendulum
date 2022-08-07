cmake_minimum_required(VERSION 3.0.2)
project(franka_pole_example)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  franka_pole
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS franka_pole
)

###########
## Build ##
###########

## Declare a C++ library
add_library(franka_pole_example_controllers SHARED
src/hybrid_controller.cpp)
target_include_directories(franka_pole_example_controllers PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(franka_pole_example_controllers PUBLIC ${catkin_LIBRARIES})
add_dependencies(franka_pole_example_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})