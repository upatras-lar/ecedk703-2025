import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import math


class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')

        # declare radar parameters
        self.declare_parameter('min_radar_angle', -20.0)
        self.declare_parameter('max_radar_angle', 20.0)
        self.declare_parameter('trigger_distance', 0.3)

        # create subscriber for lidar's scanning data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # create publisher for rviz maker
        self.marker_pub = self.create_publisher(
            Marker,
            '/radar_marker',
            10
        )

        # create publisher for detected obstacle's pose
        self.obstacle_pub = self.create_publisher(
            PoseStamped,
            '/obstacle_pose',
            10
        )

        self.get_logger().info('Radar node started!')

    def scan_callback(self, msg: LaserScan):

        min_angle_deg = self.get_parameter('min_radar_angle').value
        max_angle_deg = self.get_parameter('max_radar_angle').value
        trigger_dist = self.get_parameter('trigger_distance').value

        min_angle_rad = math.radians(min_angle_deg)
        max_angle_rad = math.radians(max_angle_deg)

        closest_range = None
        closest_angle = None

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment

            if angle < min_angle_rad or angle > max_angle_rad:
                continue

            if closest_range is None or r < closest_range:
                closest_range = r
                closest_angle = angle

        # if there is no obstacle inside the defined circular sector region, delete the marker
        if closest_range is None or closest_range >= trigger_dist:
            self.clear_marker(msg.header)
            return
        
        # in any other case, publish the detected obstacle's pose and the marker
        self.publish_obstacle_pose(closest_range, closest_angle, msg.header)
        self.publish_marker(closest_range, closest_angle, msg.header)

        # # print detection info
        # self.get_logger().info(
        #     f'PING! Obstacle {closest_range:.2f} m at {math.degrees(closest_angle):.1f}°'
        # )
    
    # function to delete the marker
    def clear_marker(self, header):
        """
        Clear the obstacle Marker!
        """
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp
        marker.ns = 'radar'
        marker.id = 1
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    # function to publish the detected obstacle's pose
    def publish_obstacle_pose(self, dist, angle, header):
        """
        Publish the pose of the detected obstacle w.r.t. lidar frame (header.frame_id)
        """
        # position in the lidar frame (polar -> cartesian coordinates)
        pos_x = dist * math.cos(angle)
        pos_y = dist * math.sin(angle)
        pos_z = 0.0

        obst_pose = PoseStamped()
        obst_pose.header.frame_id = header.frame_id
        obst_pose.header.stamp = header.stamp
        obst_pose.pose.position.x = pos_x
        obst_pose.pose.position.y = pos_y
        obst_pose.pose.position.z = pos_z

        self.obstacle_pub.publish(obst_pose)

    # function to publish the marker
    def publish_marker(self, dist, angle, header):
        """
        Publish a red sphere where the obstacle is detected!
        """
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp
        marker.ns = 'radar'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # position in the lidar frame (polar -> cartesian coordinates)
        marker.pose.position.x = dist * math.cos(angle)
        marker.pose.position.y = dist * math.sin(angle)
        marker.pose.position.z = 0.0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0

        self.marker_pub.publish(marker)


def main(args = None):
    try:
        rclpy.init(args = args)
        node = RadarNode()
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

if __name__ == "__main__":
    main()
