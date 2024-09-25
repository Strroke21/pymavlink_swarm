import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from geometry_msgs.msg import Point
from pymavlink import mavutil

class VelocityPositionPublisher(Node):
    def __init__(self, vehicle):
        super().__init__('velocity_position_publisher')
        self.vehicle = vehicle
        
        # Publishers for velocity and position
        self.velocity_publisher = self.create_publisher(TwistStamped, 'leader/velocity', 10)
        self.position_publisher = self.create_publisher(Point, 'leader/position', 10)
        self.status_publisher = self.create_publisher(String, 'leader/status', 10)
        self.alt_publisher = self.create_publisher(Point,'leader/local_pos', 10)
        # Timer to trigger the callback function to publish data every second (10 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def arm_status(self):
        heartbeat = self.vehicle.recv_match(type='HEARTBEAT', blocking=True)
        if heartbeat:
            armed = heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if armed:
                return True
            else:
                return False
            
    def get_local_position(self):
        self.vehicle.wait_heartbeat()
        self.vehicle.mav.request_data_stream_send(
        self.vehicle.target_system, 
        self.vehicle.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        100,1)
        msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        pos_x = msg.x # Degrees
        pos_y = msg.y  # Degrees
        pos_z = msg.z  # Meters
        return [pos_x,pos_y,pos_z]

    def get_global_position(self):
        self.vehicle.wait_heartbeat()
        self.vehicle.mav.request_data_stream_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            100, 1)
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        
        lat = msg.lat / 1e7  # Latitude in degrees
        lon = msg.lon / 1e7  # Longitude in degrees
        alt = msg.alt / 1000  # Altitude in meters
        vx = msg.vx / 100  # Velocity X in m/s (converted from cm/s)
        vy = msg.vy / 100  # Velocity Y in m/s (converted from cm/s)
        vz = msg.vz / 100  # Velocity Z in m/s (converted from cm/s)
        hdg = msg.hdg / 100  # Heading in degrees (converted from hundredths of degrees)
        
        return [lat, lon, alt, vx, vy, vz, hdg]

    def timer_callback(self):
        # Get position and velocity data from the vehicle
        lat, lon, alt, vx, vy, vz, hdg = self.get_global_position()
        pos_x, pos_y, pos_z = self.get_local_position()

        # Create and publish the velocity message
        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = self.get_clock().now().to_msg()  # Timestamp
        velocity_msg.twist.linear.x = vx
        velocity_msg.twist.linear.y = vy
        velocity_msg.twist.linear.z = vz
        self.velocity_publisher.publish(velocity_msg)

        # Create and publish the position message
        # Here we use Point and add a custom header field to include heading information
        position_msg = Point()
        position_msg.x = lat
        position_msg.y = lon
        position_msg.z = hdg
        self.position_publisher.publish(position_msg)

        local_pos = Point()
        local_pos.x = pos_x
        local_pos.y = pos_y
        local_pos.z = pos_z
        self.alt_publisher.publish(local_pos)
        
        # Adding custom heading information in logging or as an extension
        self.get_logger().info(f'Global Position: lat={lat}, lon={lon}, alt={alt}, hdg={hdg}')

        # Log the full data (position and velocity)
        self.get_logger().info(f'Velocity: vx={vx}, vy={vy}, vz={vz}')
        self.get_logger().info(f'Local Position: x: {pos_x}, y: {pos_y}, z: {pos_z}')


        if (abs(self.get_local_position()[2])>=5):
            status_msg = String()
            status_msg.data = 'ready'
            self.status_publisher.publish(status_msg)
            self.get_logger().info('Leader is armed, Status: ready')

        else:
            self.get_logger().info(f'Leader Ready status: {self.arm_status()}')


vehicle = mavutil.mavlink_connection('tcp:127.0.0.1:5772')

def main(args=None):
    rclpy.init(args=args)

    # Connect to the vehicle (adjust the connection string as needed)
    # Create the velocity and position publisher node
    velocity_position_publisher = VelocityPositionPublisher(vehicle)

    # Spin the node to keep it alive and publishing data
    rclpy.spin(velocity_position_publisher)

    # Shutdown the ROS 2 node after spinning is done
    velocity_position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
