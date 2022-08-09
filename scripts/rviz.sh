#!/bin/sh
SOURCE=$(rospack find franka_example_controllers)/launch/robot.rviz
DEST=$(rospack find franka_pole)/temp/$1.rviz
if [ ! -f "${DEST}" -o "${SOURCE}" -nt "${DEST}" ]; then cat "${SOURCE}" | sed "s/panda/$1/g" > "${DEST}"; fi
rviz -d "${DEST}"