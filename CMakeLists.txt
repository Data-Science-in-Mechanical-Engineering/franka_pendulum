cmake_minimum_required(VERSION 3.0.2)
project(franka_pendulum)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  controller_interface
  franka_description
  franka_gazebo
)
find_package(gazebo REQUIRED)
find_package(pinocchio REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Sample.msg
    CommandParameters.msg
    CommandPosition.msg
    CommandAcceleration.msg
    CommandReset.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES std_msgs
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_pendulum_controllers
  CATKIN_DEPENDS roscpp std_msgs message_generation controller_interface franka_description franka_gazebo
  DEPENDS gazebo pinocchio
)

###########
## Build ##
###########

## Create temp directory
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/temp")
  execute_process(COMMAND mkdir "${PROJECT_SOURCE_DIR}/temp")
endif()

## Generate URDF files
add_custom_command(OUTPUT "${PROJECT_SOURCE_DIR}/robots/franka_pendulum.urdf" DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum.xacro" COMMAND xacro ARGS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum.xacro" > "${PROJECT_SOURCE_DIR}/robots/franka_pendulum.urdf")
add_custom_target(franka_pendulum_xacro ALL DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum.urdf")
add_custom_command(OUTPUT "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_1D.urdf" DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_1D.xacro" COMMAND xacro ARGS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_1D.xacro" > "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_1D.urdf")
add_custom_target(franka_pendulum_xacro_1D ALL DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_1D.urdf")
add_custom_command(OUTPUT "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2D.urdf" DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2D.xacro" COMMAND xacro ARGS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2D.xacro" > "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2D.urdf")
add_custom_target(franka_pendulum_xacro_2D ALL DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2D.urdf")
add_custom_command(OUTPUT "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2Db.urdf" DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2Db.xacro" COMMAND xacro ARGS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2Db.xacro" > "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2Db.urdf")
add_custom_target(franka_pendulum_xacro_2Db ALL DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2Db.urdf")
add_custom_command(OUTPUT "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2Dc.urdf" DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2Dc.xacro" COMMAND xacro ARGS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2Dc.xacro" > "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2Dc.urdf")
add_custom_target(franka_pendulum_xacro_2Dc ALL DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pendulum_2Dc.urdf")

## Declare C++ libraries
add_library(franka_pendulum_common SHARED
src/parameters.cpp
src/parameter_reader.cpp
src/franka_model.cpp
src/model.cpp)
target_include_directories(franka_pendulum_common PUBLIC include)
target_compile_options(franka_pendulum_common PUBLIC -O3)
target_include_directories(franka_pendulum_common PUBLIC ${catkin_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS})
target_link_libraries(franka_pendulum_common PUBLIC ${catkin_LIBRARIES} ${pinocchio_LIBRARIES})
#target_compile_definitions(franka_pendulum_common PUBLIC FRANKA_POLE_VELOCITY_INTERFACE)
add_dependencies(franka_pendulum_common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(franka_pendulum_controllers SHARED
src/franka_state.cpp
src/pendulum_state.cpp
src/publisher.cpp
src/controller.cpp
src/position_controller.cpp
src/acceleration_controller.cpp
src/external_position_controller.cpp
src/simple_position_controller.cpp
src/test_position_controller.cpp
src/external_acceleration_controller.cpp
src/simple_acceleration_controller.cpp
src/test_acceleration_controller.cpp)
target_link_libraries(franka_pendulum_controllers PUBLIC franka_pendulum_common)

add_library(franka_pendulum_plugin SHARED
src/plugin.cpp)
target_link_libraries(franka_pendulum_plugin PUBLIC franka_pendulum_common)
target_include_directories(franka_pendulum_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(franka_pendulum_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(franka_pendulum_plugin PRIVATE ${GAZEBO_LIBRARIES})
target_compile_options(franka_pendulum_plugin PRIVATE ${GAZEBO_CXX_FLAGS})