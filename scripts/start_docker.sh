#!/bin/bash

if [ "$#" -eq  "0" ]
then
    ros_distro=jazzy
    echo "No ROS distro specified; using ${ros_distro}"
else
    ros_distro=$1
fi

docker_image="moveit/moveit2:${ros_distro}-release"

echo "Pulling and using docker image ${docker_image}"

rocker --nvidia --x11 --user --pull "${docker_image}"

# now you can use setup_workspace.sh and run_test.sh