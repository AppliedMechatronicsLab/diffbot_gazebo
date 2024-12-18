**Most of this repo is based on: https://github.com/joshnewans/articubot_one
**
## Installation
1. Create ros2 workspace: `mkdir ~/ros2_ws`
2. Create src directory inside ros2_ws and navigate into: `mkdir ros2_ws/src && cd ros2_ws/src`
3. Clone this repo: `git clone https://github.com/AppliedMechatronicsLab/diffbot_gazebo.git`
4. Install dependencies: `cd .. && rosdep install -y -i --from-paths src/`
5. Build package: `colcon build --symlink-install`
6. Source the workspace: `source install/setup.bash`
## Usage
Remember to source workspace everytime open a new terminal OR type this: `echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc`
To show robot in gazebo: `ros2 launch diffbot_gazebo robot.launch.py`
Move robot around, open a new terminal and type: `ros2 launch ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel_unstamped`

To run SLAM: `cd ~/ros2_ws/src/diffbot_gazebo`
             `ros2 launch diffbot_gazebo robot.launch.py world:=worlds/turtlebot3_world.world`
In a new terminal: `ros2 launch diffbot_gazebo diffbot_online_async_launch.py`
Open third terminal and open rviz: `rviz2 -d config/slam_config.rviz`
To run navigation with Nav2: `ros2 launch diffbot_gazebo diffbot_navigation_launch.py`

rtabmap, docker, ignition gazebo...
