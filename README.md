# Source code for MARCEL

<p align="center">
	<img src="cover_picture.png?raw=true" width="600">
</p>


Source code for MARCEL (Mobile Active Rover Chassis for Enhanced Locomotion)

### Build the ROS package on the rover:

Let's say your catkin workspace is *~/catkin_ws* for example:<br />
`CATKIN_WS=~/catkin_ws`

Put the *rover_ctrl* package among the catkin workspace sources:<br />
`ln -s $(readlink -f MARCEL_src/RaspberryPi/rover_ctrl) $CATKIN_WS/src`

Downloads the sources for the force-torque sensor package:<br />
`cd ~/Downloads`<br />
`git clone --depth=1 https://github.com/ros-industrial/robotiq.git`<br />
`ln -s ~/Downloads/robotiq/robotiq_ft_sensor $CATKIN_WS/src`

Download the C++ sources for the model trees:<br />
`cd $CATKIN_WS/src/rover_ctrl/src`<br />
`git clone --depth=1 https://github.com/Bouty92/ModelTree`

To autonomously start the low-level control node (nav_node) at boot:<br />
`sudo ln -s $(readlink -f MARCEL_src/RaspberryPi/nav_node.service) /etc/systemd/system`<br />
`sudo systemctl enable nav_node.service`


### Install nodes for the operator's remote computer:

To operate the rover from a remote computer, you only need the nodes in python:<br />
`mkdir -p $CATKIN_WS/src/rover_ctrl`<br />
`ln -s $(readlink -f MARCEL_src/RaspberryPi/rover_ctrl/scripts) $CATKIN_WS/src/rover_ctrl`


### Control the rover from a remote computer:

For a manual driving, start the ROS node *keyboard_ctrl* which publishes on *nav_ctrl* topic:<br />
`rosrun rover_ctrl keyboard_ctrl.py`
