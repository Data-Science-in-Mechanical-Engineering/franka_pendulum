cmake_minimum_required(VERSION 3.0.2)
project(franka_pole)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  franka_description
  franka_gazebo
  message_generation
)
find_package(pinocchio REQUIRED)
find_package(gazebo REQUIRED)

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
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package()

###########
## Build ##
###########

## Create temp directory
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/temp")
  execute_process(COMMAND mkdir "${PROJECT_SOURCE_DIR}/temp")
endif()

## Generate URDF files
add_custom_command(OUTPUT "${PROJECT_SOURCE_DIR}/robots/franka_pole.urdf" DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pole.xacro" COMMAND xacro ARGS "${PROJECT_SOURCE_DIR}/robots/franka_pole.xacro" > "${PROJECT_SOURCE_DIR}/robots/franka_pole.urdf")
add_custom_target(franka_pole_xacro ALL DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pole.urdf")
add_custom_command(OUTPUT "${PROJECT_SOURCE_DIR}/robots/franka_pole_1D.urdf" DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pole_1D.xacro" COMMAND xacro ARGS "${PROJECT_SOURCE_DIR}/robots/franka_pole_1D.xacro" > "${PROJECT_SOURCE_DIR}/robots/franka_pole_1D.urdf")
add_custom_target(franka_pole_xacro_1D ALL DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pole_1D.urdf")
add_custom_command(OUTPUT "${PROJECT_SOURCE_DIR}/robots/franka_pole_2D.urdf" DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pole_2D.xacro" COMMAND xacro ARGS "${PROJECT_SOURCE_DIR}/robots/franka_pole_2D.xacro" > "${PROJECT_SOURCE_DIR}/robots/franka_pole_2D.urdf")
add_custom_target(franka_pole_xacro_2D ALL DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pole_2D.urdf")
add_custom_command(OUTPUT "${PROJECT_SOURCE_DIR}/robots/franka_pole_2Db.urdf" DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pole_2Db.xacro" COMMAND xacro ARGS "${PROJECT_SOURCE_DIR}/robots/franka_pole_2Db.xacro" > "${PROJECT_SOURCE_DIR}/robots/franka_pole_2Db.urdf")
add_custom_target(franka_pole_xacro_2Db ALL DEPENDS "${PROJECT_SOURCE_DIR}/robots/franka_pole_2Db.urdf")

## Declare a C++ libraries
add_library(franka_pole_common SHARED
src/parameters.cpp
src/parameter_reader.cpp
src/franka_model.cpp)
target_include_directories(franka_pole_common PUBLIC include)
target_compile_options(franka_pole_common PUBLIC -O3)
target_include_directories(franka_pole_common PUBLIC ${catkin_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS})
target_link_libraries(franka_pole_common PUBLIC ${pinocchio_LIBRARIES})

add_library(franka_pole_controllers SHARED
src/franka_state.cpp
src/pole_state.cpp
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
target_link_libraries(franka_pole_controllers PUBLIC franka_pole_common)

add_library(franka_pole_plugin SHARED
src/plugin.cpp)
target_link_libraries(franka_pole_plugin PUBLIC franka_pole_common)
target_include_directories(franka_pole_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(franka_pole_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(franka_pole_plugin PRIVATE ${GAZEBO_LIBRARIES})
target_compile_options(franka_pole_plugin PRIVATE ${GAZEBO_CXX_FLAGS})