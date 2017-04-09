#!/bin/bash

# script needs to be run as sudo! This is because of the chmod on /dev/input/jsX

# util I found which lets you emulate a steam controller as an xbox controller so that it
# appears on /dev/input which is what rosjoy expects
# https://github.com/ynsta/steamcontroller
sc-xbox.py start

# this is necessary for the joystick to show up on the list of hid devices on /dev/input
# aka untill you perform an input the virtual file won't be created
read -n1 -r -p "Perform an input action on the controller, then press any key to continue..."

# necessary to change permissions
chmod a+rw /dev/input/js0

source /opt/ros/kinetic/setup.bash
source /home/roboboat/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.0.10:11311/

rosparam set joy_node/dev "/dev/input/js0"

rosrun joy joy_node &

python ./joystick_controller.py
