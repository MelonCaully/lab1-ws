#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):
    def __init__(self):
        super().__init__('relay')

        # Create subscriber to 'drive' topic
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.drive_callback,
            10
        )
        
        # Create publisher for 'drive_relay' topic
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def drive_callback(self, msg):
        # Multiply the speed and steering_angle by 3
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = msg.drive.speed * 20
        new_msg.drive.steering_angle = msg.drive.steering_angle * 50

        # Publish the modified message to 'drive_relay' topic
        self.publisher_.publish(new_msg)
        self.get_logger().info(f'Relaying: speed={new_msg.drive.speed}, steering_angle={new_msg.drive.steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = Relay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()