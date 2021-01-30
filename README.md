# UR5_Pick_Place
Author: Sameer Todkar

Date: 29 Jan 2021

---


## Description:
The project focuses on simulating a pick up and place behavior using the MoveIt motion planning API. The robot simulation is performed using RVIz/ROS.

Robot Used: Universal Robot (UR5)

Gripper Used: Robotiq85

Tasks Performed:
- Pick up and place of object
- Graphical User Interface to set the home position of robot, start position of the box, end position of the box. 
- Visualising the box orientation during the path planning 

---

## File Strucutre: 


```
ur5/ur5_control/
    ├── basic_gui.py
    ├── CMakeLists.txt
    ├── images
    │   └── robot.png
    ├── package.xml
    ├── trial.py
    ├── trial.pyc
    └── visualize.py
ur5/ur5_moveit_config/
    ├── CMakeLists.txt
    ├── config
    │   ├── chomp_planning.yaml
    │   ├── fake_controllers.yaml
    │   ├── joint_limits.yaml
    │   ├── kinematics.yaml
    │   ├── ompl_planning.yaml
    │   ├── ros_controllers.yaml
    │   ├── sensors_3d.yaml
    │   └── ur5.srdf
    ├── launch
    │   ├── chomp_planning_pipeline.launch.xml
    │   ├── default_warehouse_db.launch
    │   ├── demo_gazebo.launch
    │   ├── demo.launch
    │   ├── fake_moveit_controller_manager.launch.xml
    │   ├── gazebo.launch
    │   ├── joystick_control.launch
    │   ├── move_group.launch
    │   ├── moveit.rviz
    │   ├── moveit_rviz.launch
    │   ├── ompl_planning_pipeline.launch.xml
    │   ├── planning_context.launch
    │   ├── planning_pipeline.launch.xml
    │   ├── ros_controllers.launch
    │   ├── run_benchmark_ompl.launch
    │   ├── sensor_manager.launch.xml
    │   ├── setup_assistant.launch
    │   ├── trajectory_execution.launch.xml
    │   ├── ur5_moveit_controller_manager.launch.xml
    │   ├── ur5_moveit_sensor_manager.launch.xml
    │   ├── warehouse.launch
    │   └── warehouse_settings.launch.xml
    └── package.xml
```
** Oher files from ur5 github mentioned in ur5 readme

---
## Dependencies

- ROS Melodic
- MoveIt Melodic
- Tkinter
- Pygame, OpenGL
- Threading

---