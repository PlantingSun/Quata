# Quata
QUAdruped robot with delTA.

quata1.0 is a one-legged robot to test

# How to start Simulation?
**new:** git clone these into a src folder and you can run ./run_quata_sim.sh to start the simulation. 

 1. make the Quata folder into a catkin workspace's src path, like `~/quata_ws/src/Quata`
 2. build the workspace and source
 ```
 # In ~/quata_ws path
 catkin build
 souce devel/setup.bash  #Or setup.zsh if you use zsh
 ```
 3. run simulation
 ```
 # In ~/quata_ws/src/Quata/mujoco_sim
 python3 quata_sim.py
 # rosrun mujoco_sim quata_sim.py doesn't work, you can fix this
 ```
 4. simulation commands.
  `space` button is used to pause and resume simulation
  `backspace` button is used to restart the simulation
  You can add you own command by yourself.
  `Esc` button to stop the simulation.
 5. control by controller.
 ```
 # In ~/quata_ws
 rosrun controller JumpController
 ```

# How to use ROS to communicate with it
1. you can subscribe Imu data through "Imu" message type, "/bodyImu" topic.
2. you can subscribe joint state data through "motor_data" message type, "/bodyMotor" topic.
3. you can publish joint command data through "motor_data" message type, "/jointCmd" topic.
