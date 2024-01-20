#!/bin/bash
cd ..
cd ..
catkin build
source ./devel/setup.bash
cd src/Quata/mujoco_sim/
# rosrun controller JumpController
# rosrun controller JumpController
python3 quata_sim.py
# gnome-terminal --tab "XXD_ros" -- bash -c "python3 quata_sim.py;exec bash"

# gnome-terminal --tab "XXD_ros" -- bash -c "rosrun controller JumpController;exec bash"
