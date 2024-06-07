#!/bin/bash
cd ~/projects/shadow_robot/base/src
git clone https://gitlab.com/botasys/bota_driver.git
sudo apt update
cd ~/projects/shadow_robot/base
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y libxmlrpcpp-dev librosconsole-dev
catkin_make
