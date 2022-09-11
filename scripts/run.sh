#!/bin/sh
PIDFILE=$(rospack find franka_pendulum)/temp/run.pid
if [ "$1" = "start" -o "$1" = "stop" ]; then
    if [ -f "${PIDFILE}" ]; then
        while read -r pid; do kill $pid; done < "${PIDFILE}"
        rm "${PIDFILE}"
    fi
else
    echo Invalid usage
fi

if [ "$1" = "start" ]; then
    COMMAND='roslaunch franka_pendulum franka_pendulum.launch model:=2Db'

    # Simulation
    $COMMAND namespace:=franka_pendulum &
    echo $! >> "${PIDFILE}"
fi