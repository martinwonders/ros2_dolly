#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class DollyFollow(Node):  # Modify the class name

    def __init__(self):
        super().__init__("dolly_follow")
        self.get_logger().info("started /dolly_follow node")
        self.pub = self.create_publisher(Twist, '/dolly/cmd_vel', 10)
        default_qos = rclpy.qos.qos_profile_system_default
        self.sub = self.create_subscription(
            LaserScan, '/dolly/laser_scan', self.callback_on_sensor_msg, default_qos)
        # Minimum allowed distance from target
        self.min_dist = 1.0

        # Scale linear velocity, chosen by trial and error
        self.linear_scale = 0.02

        # Scale angular velocity, chosen by trial and error
        self.angular_scale = 0.08

    def callback_on_sensor_msg(self, msg):
        min_range = msg.range_max + 1
        idx = -1

        # Find closest hit from the laser ranges
        for i in range(len(msg.ranges)):
            distance = msg.ranges[i]
            if distance > msg.range_min and distance < msg.range_max and distance < min_range:
                min_range = distance
                idx = i

        # Calculate desired yaw(turning angle) change
        turn = float(msg.angle_min) + msg.angle_increment * idx

        # Populate cmd_vel message, all weights have been calculated by trial and error
        cmd_msg = Twist()
        # Bad readings, stop
        if idx < 0:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
        elif min_range <= self.min_dist:
            # Too close, just rotate
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = turn * self.angular_scale
        else:
            cmd_msg.linear.x = float(self.linear_scale) / abs(turn)
            cmd_msg.angular.z = turn * self.angular_scale

        self.pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DollyFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
