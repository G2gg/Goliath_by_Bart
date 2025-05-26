# Goliath_by_Bart
This is a simulation package for the Goliath robot working on a mobile base created by Mr. Bart van der Haagen.

I have created a Gazebo Classic simulation working with key_teleop and Moveit2 in ROS2 Humble.

To run the whole package at once, use the command in the terminal (after sourcing the workspace) as below:
```
ros2 launch goliath_bringup simulation.launch.py
```

For running moveit:
```
ros2 launch goliath_moveit moveit.launch.py
```
Note: Change the planning library in moveit (Rviz) to "OMPL" for better trajectory planning.


And to directly run key_teleop (for moving mobile base):
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/amr_controller/cmd_vel
```
