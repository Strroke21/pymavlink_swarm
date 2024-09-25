#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point

class DroneSubscriber(Node):

    def __init__(self):
        super().__init__('drone_subscriber')
        
        # Initialize attributes
        self.leader_vx = None
        self.leader_vy = None
        self.leader_vz = None
        self.leader_lat = None
        self.leader_lon = None
        self.leader_hdg = None
        
        # Subscribing to the /leader/velocity topic
        self.subscription = self.create_subscription(TwistStamped,'/leader/velocity',self.velocity_callback,10)
        
        # Subscribing to the /leader/position topic
        self.subscription1 = self.create_subscription(Point,'/leader/position',self.position_callback,10)

    def velocity_callback(self, msg):
        # Callback function to process velocity data
        self.leader_vx = msg.twist.linear.x
        self.leader_vy = msg.twist.linear.y
        self.leader_vz = msg.twist.linear.z
        
        # Print out the velocity components
        self.get_logger().info(f"Leader velocity: vx={self.leader_vx} m/s, vy={self.leader_vy} m/s, vz={self.leader_vz} m/s")

    def position_callback(self, msg):
        # Callback function to process position data
        self.leader_lat = msg.x
        self.leader_lon = msg.y
        self.leader_hdg = msg.z
        
        self.get_logger().info(f"Leader position: lat={self.leader_lat}, lon={self.leader_lon}, heading={self.leader_hdg}")


def main(args=None):
    rclpy.init(args=args)

    # Create the subscriber node
    drone_subscriber = DroneSubscriber()

    # Keep the node spinning to continuously receive data
    rclpy.spin(drone_subscriber)

    # Destroy the node explicitly (optional)
    drone_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
