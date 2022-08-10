#!/bin/sh
PIDFILE=$(rospack find franka_pole)/temp/run.pid
if [ "$1" = "kill" ]; then
    if [ -f "${PIDFILE}" ]; then
        while read -r pid; do kill $pid; done < "${PIDFILE}"
        rm "${PIDFILE}"
    fi
else
    COMMAND='roslaunch franka_pole franka_pole.launch model:=2Db'

    # Simulation
    $COMMAND namespace:=franka_pole &
    echo $! >> "${PIDFILE}"
fi