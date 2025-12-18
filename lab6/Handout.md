# ROS2 Laboratory 6 – All‑In‑One
In the previous labs we learned how to:
- create ROS2 packages and nodes,
- use **topics**, **parameters**, and **launch files**,
- work with **TF** and **URDF** robot descriptions, and
- integrate **real sensors** (camera + LiDAR) and visualize everything in **RViz2**.

In this final lab we combine all of that into one complete **perception‑to‑action** pipeline.

We will use an **RPLIDAR** to detect an obstacle in a configurable “radar sector”, like we did on lab4.
But this time when an obstacle is detected (or when it moves), the **myArm 300 Pi** will compute **inverse kinematics** and move its end‑effector to a pose above the obstacle.

> **Safety first:** keep hands, cables, and fragile objects away from the arm workspace while testing.  
> Always be ready to stop the system (Ctrl‑C).

## What you are given

Inside `lab6/myarm_follow_ws/src/` there are two packages:
- `lidar_follow_pkg`
- `rplidar_ros`

This lab uses (and launches) the following nodes:

- **`rplidar_ros/rplidar_node`**  
  Publishes:
  - `/scan` (`sensor_msgs/msg/LaserScan`)

- **`lidar_follow_pkg/radar_node`**  
  Subscribes:
  - `/scan`  
  Publishes:
  - `/radar_marker` (`visualization_msgs/msg/Marker`)
  - `/obstacle_pose` (`geometry_msgs/msg/PoseStamped`) – obstacle pose in the **LiDAR frame**

- **`lidar_follow_pkg/myarm_node`**  
  Subscribes:
  - `/obstacle_pose`  

  Publishes:
  - `/joint_states` (`sensor_msgs/msg/JointState`) – used for RViz visualization

- **`tf2_ros/static_transform_publisher`**  
  Publishes a fixed TF transform between:
  - `myarm_base_frame` → `lidar_frame`

- **`robot_state_publisher`**  
  Reads the robot's URDF and `/joint_states` and publishes the arm TF tree.

- **RViz2**  
  Loads a preconfigured RViz layout for this lab.

## The pipeline

```text
/scan  (LaserScan)
  │
  ▼
radar_node  ───────────────► /radar_marker (Marker)
  │
  └──────────────► /obstacle_pose (PoseStamped, in lidar_frame)
                       │
                       ▼  (TF: lidar_frame → myarm_base_frame)
                   myarm_node
                       │  (IK)
                       ▼
                  real myArm motion  +  /joint_states
                                         │
                                         ▼
                                robot_state_publisher → TF → RViz
```

The “glue” that makes this work is **TF**:
- `radar_node` publishes the obstacle pose in the LiDAR frame (`lidar_frame`)
- `myarm_node` uses TF to transform that point into the arm base frame (`myarm_base_frame`)
- then it computes IK and moves the arm

## Cloning the rplidar_ros package

In order to get `/scan` measurements from our lidar, we need the `rplidar_ros` package. We will clone that from github directly into our workspace. We can do that from the terminal.

```bash
cd src
git clone -b ros2 --depth 1 https://github.com/Slamtec/rplidar_ros.git
```

This clones the ros2 compatible branch for the `rplidar_ros` repository. Now that we have, it we are going to source and build it according to the maintainers instructions:

```bash
cd myarm_follow_pkg
source /opt/ros/galactic/setup.bash
colcon build --symlink-install --packages-select rplidar_ros
```
Now that we have done that we don't have to worry about this package again for the rest of the exercise. We can directly launch it from our launch file when we need it.

## Making the myarm_node


```python
import math
import rclpy
import tf2_ros
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from lidar_follow_pkg.myarm_utils.solve_inv_kine import solve_ik
from lidar_follow_pkg.myarm_utils.myarm_connect import connect
from lidar_follow_pkg.myarm_utils.math_utils import quat_to_rot


class MyArmNode(Node):
    def __init__(self):
        super().__init__("myarm_node")

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Declare parameter for the connection port
        self.declare_parameter("port", "/dev/ttyAMA0")
        port = self.get_parameter("port").value

        # Declare parameter for the connection baud rate
        self.declare_parameter("baudrate", 115200)
        baudrate = self.get_parameter("baudrate").value

        # Declare parameter for the speed of myarm joints 
        self.declare_parameter("speed", 80)
        self.speed = self.get_parameter("speed").value

        # Print the connection settings
        self.get_logger().info(f"MyArm Connect! -> port: {port}, baudrate: {baudrate}!")

        # Create myarm object
        self.myarm = connect(port, baudrate, 1.0)

        # Create publisher and timer for the joint states
        ############# ENTER CODE HERE ###################
        

        #################################################

        # Create subscription for the detected obstacle
        ############# ENTER CODE HERE ###################
        
        
        #################################################

        self.q0_list = None
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_z = 0.0
    
    def publish_myarm_state(self) -> None:
        """
        This function is periodically called by the timer. It publishes the current state (joint angles)
        of the arm's joints to the /joint_states topic.
        """

        # Create message with time stamp
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set joint names
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

        # Read the current positions of the joints (in degrees)
        ############# ENTER CODE HERE ###################
        
        
        #################################################
       
        # Convert the joint angles to radians
        positions = []
        for angle_deg in joint_angles_list:
            angle_rad = round(math.radians(angle_deg), 3)
            positions.append(angle_rad)

        # Set joint positions
        msg.position = positions

        # Publish the message
        ############# ENTER CODE HERE ###################
        
        
        #################################################

    def follow_obstacle(self, msg: PoseStamped):
        """
        Control myarm to follow the detected obstacle.
        The obstacle pose is transformed from lidar frame to myarm base frame.
        """

        self.speed = self.get_parameter("speed").value

        # Transform from lidar frame to myarm base (world) frame
        try:
            transform = self.tf_buffer.lookup_transform(
                "myarm_base_frame",
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout = rclpy.duration.Duration(seconds = 0.1)
            )
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"TF lookup failed: {e}!")
            return

        # Translation of lidar frame origin expressed in myarm base (world) frame
        t = transform.transform.translation
        t_bl = np.array([t.x, t.y, t.z])

        # Rotation of lidar frame expressed in myarm base (world) frame
        rq = transform.transform.rotation
        R_bl = quat_to_rot(np.array([rq.x, rq.y, rq.z, rq.w]))

        # Obstacle's point in lidar frame
        p_lidar = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # Transform obstacle's point from lidar frame into myarm base (world) frame
        ############# ENTER CODE HERE ###################
        
        
        #################################################
        posx_w, posy_w, posz_w = p_base

        # Build desired end-effector pose in myarm base (world) frame
        ############# ENTER CODE HERE ###################
        
        
        #################################################
 
        # Solve inverse kinematics
        if abs(posx_w - self.last_x) > 0.015 or abs(posy_w - self.last_y) > 0.015 or abs(posz_w - self.last_z) > 0.015:
            self.get_logger().info(f"{Twbd}")
            self.get_logger().warn("Starting inverse kinematics ...")
            q_rad, ik_success = solve_ik(Twbd, "space", self.q0_list, 1e-5)

            if ik_success:
                # Convert radians → degrees and send command to myarm
                q_deg = [math.degrees(q) for q in q_rad]
                self.myarm.send_angles(q_deg, self.speed)
                self.q0_list = [np.copy(q_rad), np.array([-1.557, 1.177, -0.532, -1.209, 0.612, -0.940, -2.180])]
                self.get_logger().info(f"SUCCESSFUL inverse kinematics! Found q = {np.round(q_deg, 2)} (degrees)")
            else:
                self.get_logger().info("FAILED inverse kinematics!")
            
            # Keep the last obstacle's position
            self.last_x = posx_w
            self.last_y = posy_w
            self.last_z = posz_w


def main(args = None):
    try:
        rclpy.init(args = args)
        node = MyArmNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```
> **Student TODO I:** Create the `visualize_publisher` as a class variable to publish `JointState` messages in the `/joint_states` topic every 0.01 seconds.

> **Student TODO II:** Create the `obstacle_subscription` as a class variable to listen for `PoseStamped` messages in the `/obstacle_pose` topic and call the `follow_obstacle` method when it receives a message.

> **Student TODO III:** In the `publish_my_arm_state` method we read the current state of the robot's joints and then use our publisher to publish it to `/joint_states`.
> - Inspect the fields of the `JointState` message with `ros2 interface show sensor_msgs/msg/JointState`
> - Use the `get_angles()` method on the `myarm` object that we created in `__init__` and save in on a variable called `joint_angles_list`.
> - Use the publisher to publish the message that we created.

> **Student TODO IV:** The `follow_obstacle` method is called whenever a new obstacle is detected. The `msg` here is the message that the subscriber received, which is the pose of the detected obstacle in the lidar frame. We need to transform it to the robot base frame and then use inverse kinematics to hover over that position.
> - Use the rotation matrix `R_bl` and the translation `t_bl` of the lidar expressed in world frame to calculate the 3D position of the obstacle with respect to the world frame and save it in `p_base` variable.
> - Create the trasnformation matrix `Twbd` using a rotation of 180 degress around the y axis and the obstacle translation in the world frame that you calculated previously.

## Complete the launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from lidar_follow_pkg.myarm_utils.math_utils import rot_to_quat
import numpy as np


pkg_name = "lidar_follow_pkg"

# Parameters for the static transformation from myarm base frame to lidar frame
lidar_frame_id = "lidar_frame"
myarm_base_frame_id = "myarm_base_frame"

t_bl = np.array([0.0, 0.0, 0.0])

zrot_bl = np.deg2rad(0.0)
R_bl = np.array([[np.cos(zrot_bl), -np.sin(zrot_bl), 0.0],
                [np.sin(zrot_bl), np.cos(zrot_bl), 0.0],
                [0.0, 0.0, 1.0]])

q_bl = rot_to_quat(R_bl)


def generate_launch_description():
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "myarm_300_pi.urdf"])
    rviz_config_file = PathJoinSubstitution([pkg_share, "config", "rviz", "myarm_lidar_obstacle.rviz"])

    channel_type =  LaunchConfiguration('channel_type', default = 'serial')
    serial_port = LaunchConfiguration('serial_port', default = '/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default = '115200')
    frame_id = LaunchConfiguration('frame_id', default = lidar_frame_id)
    inverted = LaunchConfiguration('inverted', default = 'false')
    angle_compensate = LaunchConfiguration('angle_compensate', default = 'true')
    scan_mode = LaunchConfiguration('scan_mode', default = 'Sensitivity')
    
    return LaunchDescription([

        DeclareLaunchArgument('channel_type', default_value = channel_type, description = 'Specifying channel type of lidar'),
        DeclareLaunchArgument('serial_port', default_value = serial_port, description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument('serial_baudrate', default_value = serial_baudrate, description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument('frame_id', default_value = frame_id, description = 'Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted', default_value = inverted, description = 'Specifying whether or not to invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate, description = 'Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode, description='Specifying scan mode of lidar'),

        Node(
            package = 'rplidar_ros',
            executable = 'rplidar_node',
            name = 'rplidar_node',
            output = 'screen',
            parameters = [{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
            }],
        ),

        Node(
            package = pkg_name,
            executable = "radar_node",
            name = "radar_node",
            output = "screen",
            parameters = [{
                "min_radar_angle": -45.0,
                "max_radar_angle": 20.0,
                "trigger_distance": 0.6,
            }],
        ),

        Node(
            package = pkg_name,
            executable = "myarm_node",
            name = "myarm_node",
            output = "screen",
            parameters = [{
                "port": "/dev/ttyAMA0",
                "baudrate": 115200,
                "speed": 80,
            }],
        ),

        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            name = "myarm_base_to_lidar",
            arguments = [
                str(t_bl[0]), str(t_bl[1]), str(t_bl[2]),
                str(q_bl[0]), str(q_bl[1]), str(q_bl[2]), str(q_bl[3]),
                myarm_base_frame_id,
                lidar_frame_id,
            ],
        ),

        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            output = "screen",
            parameters = [{
                "robot_description": ParameterValue(Command(["cat ", urdf_file]), value_type = str),
            }],
        ),

        Node(
            package = "rviz2",
            executable = "rviz2",
            name = "rviz2",
            output = "screen",
            arguments = ["-d", rviz_config_file],
        ),
    ])
```
> **Student TODO:** Measure the distance in the x and y axis of the lidar with respect to the world frame. We need to insert this tranlation information into the launch file, so that our program nows the relative transformation between the robot and the lidar.
> - Modify the `t_bl` variable to matrch the translation that you measured above.
> - If the axes of the lidar are rotated relative to the world frame, you need to insert that rotation around the z axis in the `zrot_bl`

## Launch the system

Now we can build and launch our system.

```bash
source install/setup.bash
colcon build --packages-select lidar_follow_pkg
ros2 launch lidar_follow_pkg follow_obstacle_launch.py
```

## Radar parameters

`radar_node` exposes these parameters:

- `min_radar_angle` (degrees)
- `max_radar_angle` (degrees)
- `trigger_distance` (meters)

You can inspect them:

```bash
ros2 param list /radar_node
ros2 param get /radar_node trigger_distance
```

You can change them at runtime:

```bash
ros2 param set /radar_node trigger_distance 0.45
ros2 param set /radar_node min_radar_angle -30.0
ros2 param set /radar_node max_radar_angle  30.0
```