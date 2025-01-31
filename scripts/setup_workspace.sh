#!/bin/bash

# this script will install packages and is supposed to be run within a docker container (e.g. with rocker and/or the start_docker.sh script next to this file)!

rm -rf /tmp/easy_handeye_test
mkdir -p !$
cd !$

mkdir -p easy_handeye2_test_ws/src
cd easy_handeye2_test_ws/src

git clone https://github.com/marcoesposito1988/easy_handeye2.git
git clone https://github.com/marcoesposito1988/easy_handeye2_demo.git

cd ..

sudo apt update
sudo apt install python3-pip
rosdep update
PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install -yir --from-paths src

colcon build

