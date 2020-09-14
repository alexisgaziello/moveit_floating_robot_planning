# moveit_floating_robot_planning
Plugin for RVIZ and MoveIt for underwater ROV mission planning.

## Description

This plugin is part of the UNEXMIN project and aims to ease mission planning for underwater automonus submarines.

GUI features:
- Add/modify robot actions
- Automatic waypoints generations with colision checking.
- Export and import of UNEXMIN formated missions 
- Ability to load environments (octomap format)

## Installation 

1. Add required packages: https://drive.google.com/file/d/1tqglZlt11jMm5FY875d0x9670OkPI268/view?usp=sharing
The packages are: the unexmin description for the robot mesh, the moveit unexmin config package, the moveit package, and some packages for publishing octomaps.

2. Add resources folder to catkin_ws (at the same level as src, devel, build, etc): https://drive.google.com/file/d/1Gi2Qsiru5EUdCbwXLIDW3d2NzxvtzX23/view?usp=sharing

3. run:
catkin build
source ~/catkin_ws/devel/setup.bash
roslaunch moveit_unexmin_config demo.launch

4. Add panel moveit_floating_robot_planning

5. Add visualization -> marker -> select topic: action_markers

6. For octomaps: add pointcloud2 or markerarray -> select topic: octomap_XXX
