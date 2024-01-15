#!/bin/bash
cd ..
cd ..
catkin build
source ./devel/setup.bash
cd src/Quata/mujoco_sim/
python3 quata_sim.py
