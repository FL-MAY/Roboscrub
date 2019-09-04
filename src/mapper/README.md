Get start with mapper
===
Install cartographer_ros
---
* Install wstool and rosdep<br>
`sudo apt-get update`<br>
`sudo apt-get install -y python-wstool python-rosdep ninja-build`<br>
* Create a new workspace in 'carto_ws'<br>
`mkdir ~/carto_ws`<br>
`cd carto_ws`<br>
`wstool init src`<br>
* Merge the cartographer_ros.rosinstall file and fetch code for dependencies<br>
`wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall`<br>
* Modify install files to aviod network unreachable<br>
`cd src`<br>
`mv .rosinstall rosinstall`<br>
`gedit rosinstall`<br>
Replace with<br>
"local-name: ceres-solver<br>
uri: https://github.com/ceres-solver/ceres-solver.git"<br>
`mv rosinstall .rosinstall`<br>
`cd ..`<br>
* Continue<br>
`wstool update -t src`<br>
`rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y`<br>
* Build and install<br>
`catkin_make_isolated --install --use-ninja`<br>
`source install_isolated/setup.bash`<br>

Use cartographer mapping
---
* Open a simulation environment, read navigator/README.md<br>
* Open terminal 1. Input `roslaunch mapper mapper.launch`<br>
* Open terminal 2. Input `python RoboScrub_Nav/src/navigator/teleop/navigator_teleop_key`<br>
Control the robot by the program hint.<br>
* Save map: open terminal 3. Input `rosrun map_server map_saver -f ~/map`<br>

Incrementally mapping
---
* Purpose: support mapping at existing map.<br>
* After save map for a new mapping, save cartographer pbstream.<br>
`rosservice call /write_state "{filename: '~/map.pbstream'}"`<br>
* `roslaunch mapper mapper.launch pure_localization:=true`<br>
* Cartographer pure localization is slow to acquire initial pose, so we strongly suggest set an initial pose for the node.<br>
Like `roslaunch mapper mapper.launch pure_localization:=true initial_x:=1.0 initial_y:=2.0`<br>
(initial_a is not considered cause it need to be transformed to quaternion.)<br>
* Wait the robot localization a correct angle.<br>
* Move the robot for incrementally mapping.<br>
* When finish, save the new map and pbstream file.<br>
