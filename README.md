**Most of this repo is based on: https://github.com/joshnewans/articubot_one
**
## Installation
1. Create ros2 workspace: `mkdir ~/ros2_ws`
2. Create src directory inside ros2_ws and navigate into: `mkdir ros2_ws/src && cd ros2_ws/src`
3. Install git: `sudo apt install git-all`
4. Clone this repo: `git clone https://github.com/AppliedMechatronicsLab/diffbot_gazebo.git`
5. Install dependencies: `cd .. && rosdep install -y -i --from-paths src/`
6. Build package: `colcon build --symlink-install`
7. Source the workspace: `source install/setup.bash`
## Usage
Remember to source workspace everytime open a new terminal OR type this: `echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc`

To show robot in gazebo: `ros2 launch diffbot_gazebo robot.launch.py`
![image](https://github.com/user-attachments/assets/263be886-4db4-4422-a59e-091483011e67)

Move robot around, open a new terminal and type: `ros2 launch ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel_unstamped`
![image](https://github.com/user-attachments/assets/9fe7bcf1-1d8e-4291-ba02-4c2f568555b6)

To run SLAM: `cd ~/ros2_ws/src/diffbot_gazebo`
             `ros2 launch diffbot_gazebo robot.launch.py world:=worlds/turtlebot3_world.world`

In a new terminal: `ros2 launch diffbot_gazebo diffbot_slam.launch.py`
![Screenshot from 2024-11-23 21-24-03](https://github.com/user-attachments/assets/2572a992-b6f4-4237-872e-2dc250521492)

To run navigation with Nav2 while doing SLAM: `ros2 launch diffbot_gazebo diffbot_navigation_launch.py`
                                              `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz`
Click on 2D Goal Pose to command robot.
![image](https://github.com/user-attachments/assets/aa433f2c-b3ca-42f2-ab30-c8b54996232a)

To run nav2 with a pre-saved map: `ros2 launch nav2_bringup localization_launch.py map:=/PATH/TO/MAP.yaml use_sim_time:=true # replace /PATH/TO/MAP with your path`
Open Rviz and click 2D Pose Estimate `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz`
![image](https://github.com/user-attachments/assets/00eb2652-46fe-4f49-b63a-40d27c5a08f8)
Choose where robot is in the map and drive around a bit for localization node to corrects robot's position
Start navigation `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true`
Click on 2D Goal Pose to command robot.

Rtabmap:
Open robot in gazebo: `ros2 launch diffbot_gazebo robot.launch.py`
Launch rtabmap: `ros2 launch diffbot_gazebo turtlebot3_rgbd.launch.py use_sim_time:=true`
Open rviz and see the result after driving around
![Screenshot from 2024-11-21 21-27-52](https://github.com/user-attachments/assets/7f422d74-9b3c-43c0-971b-cf029f39c191)

Ignition gazebo:
Gazebo Classic goes end-of-life in January of 2025
Switch to ign_gazebo branch: `git checkout ign_gazebo`
If you built the package with `--symlink-install` then you can run the below step right away or else you have to `colcon build` again
To show robot in ignition gazebo: `ros2 launch diffbot_gazebo robot.launch.py`
Every thing else is the same.

docker...
