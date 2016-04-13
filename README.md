#ros_planner

To execute :
roscore
rosrun ros_planner planner_input.py
rosrun ros_planner planner.py

ROS_PLANNER is the node linking most of the modules on the deck with the tools on the platform.
The DECK folder contains files describing the content of the deck. These are only text files.
For more information on the deck files, take a look at the README in /deck

The JSON folder contains json files describing the protocol the user want the platform to do.

The scripts folder contains files describing function for labware, pipette, tac and gripper for example. 
It also contain a console HMI that allow the user to control the platform.
