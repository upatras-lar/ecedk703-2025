## ROS2 Lab Descriptions

### Lab 1: Introduction to ROS2 Basics
**ROS2 Intro, Packages, Nodes, Communication**

This foundational lab introduces ROS2 architecture and core concepts. Students learn the ROS2 ecosystem including package creation with colcon build system, node development using Python rclpy library, and communication patterns through topics, services, and actions. The lab covers publisher/subscriber architecture for continuous data flow, service client/server interactions for request-response communication, and action-based communication for long-running tasks. Practical implementation includes the Jerry Robot simulation exercise where students create a complete ROS2 graph with command publisher, robot controller node, and distance calculation service, demonstrating real-world application of ROS2 communication principles.

### Lab 2: Parameters, Launch Files & Raspberry Pi Integration  
**Advanced ROS2 Concepts & Hardware Control**

This lab explores advanced ROS2 features for configurable systems and physical hardware integration. Students learn parameter management for dynamic node behavior, including static vs dynamic parameters, YAML configuration files, and runtime parameter updates via CLI callbacks. Launch file creation teaches multi-node system orchestration with parameter passing, topic remapping, and executable configuration. The hands-on component uses Raspberry Pi GPIO for LED control exercises, implementing publisher/subscriber patterns for blinking LEDs, button input handling with pull-up resistor configurations, and parameter-driven blink rate adjustment. This bridges theoretical ROS2 concepts with physical hardware control, preparing students for embedded robotics applications.

### Lab 3: RViz2 & Visualization Tools
**3D Robot Visualization, RQT Plugins, URDF Modeling**

Students master robot visualization and debugging tools essential for robotics development. The lab covers RViz2 for 3D robot state visualization, the RQT plugin ecosystem including rqt_graph, rqt_plot, rqt_publisher, and rqt_topic for system monitoring, and URDF file creation for robot modeling with links, joints, visual and collision elements. Practical exercises include double pendulum visualization using MD80 motor encoders and joint state publishing, and Elephant MyArm 300 Pi robotic arm control with GUI sliders. Students learn TF (Transform) frame trees, robot_state_publisher integration, and the relationship between /joint_states topics and /tf transforms for accurate robot visualization.

### Lab 4: Sensor Data Visualization with RViz2
**Camera & LIDAR Integration, Pose Estimation**

This lab focuses on sensor integration and computer vision in ROS2. Students study sensor classifications (proprioceptive vs exteroceptive), camera theory including pinhole camera model with intrinsic/extrinsic parameters, and LIDAR principles including time-of-flight distance measurement. Practical work includes Logitech C270 camera integration using v4l2_camera driver, ArUco marker detection and pose estimation using OpenCV for visual servoing applications, and RPLIDAR A1M8 integration for 2D scanning and obstacle detection. Students implement visualization_msgs/Marker publishing for RViz2 display, sensor calibration procedures, and real-time sensor data processing pipelines.

### Lab 5: MuJoCo Simulation & Control Schemes  
**Physics Simulation, Velocity & Torque Control**

Students compare robot simulators and learn advanced control strategies for manipulators. The lab contrasts Gazebo with MuJoCo, highlighting MuJoCo's advantages in speed, stability, and contact dynamics for research applications. Control theory covers joint space vs operational space control, velocity control using Jacobian pseudoinverse for task-space trajectory tracking, and torque control with gravity compensation and null-space regularization. Practical implementation involves Kuka LBR iiwa 14 robot control in MuJoCo, tracking mathematical trajectories including circles, Lissajous curves, and figure-eight patterns. Students experience both kinematic simulation (dynamics disabled) and full physics simulation with external disturbance handling.

### Lab 6: All-In-One Exercise with Real Robot Control
**LIDAR-Based Obstacle Following with Real Robot Control**

The final lab implements a complete perception-to-action pipeline on physical hardware. Students develop a LIDAR-guided obstacle tracking system that integrates sensor processing, coordinate transformations, and real-time robot control. The lab covers RPLIDAR A1M8 integration for 2D environmental scanning, angular sector-based obstacle detection with configurable distance thresholds, and static frame transformations between LIDAR and robot coordinate systems. Practical implementation involves the Elephant MyArm 300 Pi 7-DOF manipulator with custom inverse kinematics solver using screw theory and numerical optimization, real-time obstacle pose estimation and marker visualization in RViz2, and position-based filtering to ensure smooth robot motion. Students deploy the complete system on Raspberry Pi hardware, managing serial communication with the physical robot, handling sensor-to-robot frame transformations with TF2, and implementing safety considerations for real-world operation.
