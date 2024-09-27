#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # Declare parameters 'v' (velocity) and 'd' (steering angle)
        self.declare_parameter('v', 0.0)
        self.declare_parameter('d', 0.0)

        # Create publisher for the AckermannDriveStamped message
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.timer = self.create_timer(0.01, self.publish_message)  # Publishing as fast as possible

    def publish_message(self):
        # Get parameters 'v' and 'd'
        v = self.get_parameter('v').get_parameter_value().double_value
        d = self.get_parameter('d').get_parameter_value().double_value

        # Create the AckermannDriveStamped message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = v
        drive_msg.drive.steering_angle = d

        # Publish the message
        self.publisher_.publish(drive_msg)
        self.get_logger().info(f'Publishing: speed={v}, steering_angle={d}')

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()