#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        
        # Create a publisher to publish the command on the "drone/command" topic
        self.publisher_ = self.create_publisher(String, 'formation/command', 10)
        
        # Timer to periodically check for input and publish commands (every 5ms)
        self.timer = self.create_timer(0.05, self.publish_command)
    
    def publish_command(self):
        message = String()

        # Get input from the user
        input_command = input("Enter command: ").strip()

        # If no input, publish 'None'
        if not input_command:
            message.data = None
        else:
            # Otherwise, publish the input command
            message.data = input_command

        # Publish the command to the topic
        self.publisher_.publish(message)
        self.get_logger().info(f"Published command: '{message.data}'")

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the command publisher node
    command_publisher = CommandPublisher()

    # Spin the node so that it remains active and continues to publish commands
    rclpy.spin(command_publisher)

    # Shutdown the ROS 2 node when done
    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
