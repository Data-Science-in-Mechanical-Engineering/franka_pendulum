#!/bin/sh
PIDFILE=$(rospack find franka_pole)/temp/run.pid
if [ "$1" = "start" -o "$1" = "stop" ]; then
    if [ -f "${PIDFILE}" ]; then
        while read -r pid; do kill $pid; done < "${PIDFILE}"
        rm "${PIDFILE}"
    fi
else
    echo Invalid usage
fi

if [ "$1" = "start" ]; then
    COMMAND='roslaunch franka_pole franka_pole.launch model:=2Db'

    # Simulation
    ROS_MASTER_URI=http://localhost:11312 rosrun fkie_master_discovery master_discovery &
    echo $! >> "${PIDFILE}"
    ROS_MASTER_URI=http://localhost:11312 $COMMAND source:=gazebo namespace:=franka_pole_sim &
    echo $! >> "${PIDFILE}"

    # Real robot
    rosrun fkie_master_discovery master_discovery &
    echo $! >> "${PIDFILE}"
    $COMMAND source:=robot namespace:=franka_pole &
    echo $! >> "${PIDFILE}"

    # Syncronization
    rosrun fkie_master_sync master_sync ~interface_url:=http://localhost:11312 ~sync_topics:=/franka_pole_sim/* &
    echo $! >> "${PIDFILE}"
fi