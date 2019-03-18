## Gauss Manipulator Simulation Environment
### Interactive Environment: Rviz
launch command:  
roslaunch gauss_moveit_config demo.launch joint_states:=/joint_states

### Interactive Environment: Gazebo + Rviz
#### OMPL Planner(default)
launch command:  
roslaunch gauss_moveit_config gauss_planning_execution.launch
#### Pilz Command Planner
launch command:  
roslaunch gauss_moveit_config gauss_planning_execution.launch pipeline:=command_planner

## Interactive with Gauss by scripts
## joint space and pose
launch command:  
rosrun gauss_moveit_config move_group_python_interface_tutorial.py
## catesian plan
launch command:  
rosrun gauss_moveit_config move_group_cartesian_path_tutorial.py

## Use Gauss IK Solver
launch Gauss IK Solver service:  
rosrun gauss_gazebo gauss_IK_server.py

launch Gauss IK Solver test script:  
rosrun gauss_gazebo gauss_IK_pose.py


