#!/usr/bin/env python3

import math
from pymavlink import mavutil
from math import radians, cos, sin, sqrt, atan2
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point
from std_msgs.msg import String

def connect(connection_string):

    vehicle =  mavutil.mavlink_connection(connection_string)

    return vehicle

def arm(vehicle):
    #arm the drone
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
    	                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
def VehicleMode(vehicle,mode):

    modes = ["STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED", "LOITER", "RTL", "CIRCLE","","LAND"]
    if mode in modes:
        mode_id = modes.index(mode)
    else:
        mode_id = 12
    ##### changing to guided mode #####
    #mode_id = 0:STABILIZE, 1:ACRO, 2: ALT_HOLD, 3:AUTO, 4:GUIDED, 5:LOITER, 6:RTL, 7:CIRCLE, 9:LAND 12:None
    vehicle.mav.set_mode_send(
        vehicle.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)
    
def drone_takeoff(vehicle, altitude):
    
    # Send MAVLink command to takeoff
    vehicle.mav.command_long_send(
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
        0,                          # confirmation
        0,                          # param1 (min pitch, not used)
        0,                          # param2 (empty for now, not used)
        0,                          # param3 (empty for now, not used)
        0,                          # param4 (yaw angle in degrees, not used)
        0,                          # param5 (latitude, not used)
        0,                          # param6 (longitude, not used)
        altitude                    # param7 (altitude in meters)
    )

def get_global_position(vehicle):
    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    100,1)
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat/1e7 # lat
    lon = msg.lon/1e7 # lon
    alt = msg.alt/1000  # alt
    return [lat,lon,alt]

def goto_waypoint(vehicle,latitude, longitude, altitude):
    msg = vehicle.mav.set_position_target_global_int_encode(
        time_boot_ms=10,
        target_system=vehicle.target_system,       # Target system (usually 1 for drones)
        target_component=vehicle.target_component,    # Target component (usually 1 for drones)
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame of reference for the coordinate system
        type_mask=0b0000111111111000,        # Bitmask to indicate which dimensions should be ignored (0b0000111111111000 means all ignored except position)
        lat_int=int(latitude * 1e7),       # Latitude in degrees * 1e7 (to convert to integer)
        lon_int=int(longitude * 1e7),      # Longitude in degrees * 1e7 (to convert to integer)
        alt=altitude,           # Altitude in meters (converted to millimeters)
        vx=0,                         # X velocity in m/s (not used)
        vy=0,                         # Y velocity in m/s (not used)
        vz=0,                         # Z velocity in m/s (not used)
        afx=0, afy=0, afz=0,                   # Accel x, y, z (not used)
        yaw=0, yaw_rate=0                       # Yaw and yaw rate (not used)
    )
    vehicle.mav.send(msg)


def get_heading(vehicle):

    vehicle.mav.command_long_send(vehicle.target_system,
    vehicle.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,74,0,0,0,0,0,0)
    ack_msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    #### COMMAND_ACK has a message id of 512.

    msg = vehicle.recv_match(type='VFR_HUD',blocking=True)

    heading = msg.heading
        #print(heading)
    return heading #degrees


def relative_pos(lat, lon, distance, heading, follower_heading):
        # Convert heading to radians and calculate new heading
    EARTH_RADIUS = 6371000
    heading_rad = math.radians(heading)
    new_heading_rad = heading_rad + math.radians(follower_heading)
    
    # Convert latitude and longitude to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    # Calculate the new latitude and longitude
    delta_lat = distance * math.cos(new_heading_rad) / EARTH_RADIUS
    delta_lon = distance * math.sin(new_heading_rad) / (EARTH_RADIUS * math.cos(lat_rad))
    
    new_lat_rad = lat_rad + delta_lat
    new_lon_rad = lon_rad + delta_lon
    
    # Convert the new latitude and longitude back to degrees
    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)
    
    return new_lat, new_lon


def formation(follower_vehicle,lat_lead, lon_lead,lead_yaw, angle, distance,target_alt):
    counter = 0
    while True:
        ######## leader ######
        # Retrieve leader's position from the subscriber
        if lat_lead is not None and lon_lead is not None:
            counter += 1
            print(f"Leader Position: lat={lat_lead}, lon={lon_lead}, Heading: {lead_yaw} deg.")
        
            ######### follower1 ######
            yaw1 = angle  # on the right
            dist1 = distance  # distance for the follower drone
            lat1, lon1 = relative_pos(lat_lead, lon_lead, dist1, lead_yaw, yaw1)
            print(f"Follower waypoint: lat={lat1}, lon={lon1}")
            
            ##### send msg to follower1 drone #############
            if counter == 1:
                goto_waypoint(follower_vehicle, lat1, lon1, target_alt)  # Adjust to the same altitude as leader
                print("Formation command sent.")
                time.sleep(0.15)
    
            dist_to_target = distance_between(follower_vehicle, lat1, lon1)
            print(f"Distance to formation: {round(dist_to_target, 2)} m.")
            
            if dist_to_target <= 2:
                print("Follower has reached the formation position.")
                time.sleep(1)
                break
        else:
            print("No data retrieved from the leader.")
            time.sleep(1)  # Wait before trying again if no data is available



def distance_to_leader(vehicle,leader_lat,leader_lon):
    
    msg2 = get_global_position(vehicle)
    
    current_lat = msg2[0] # lat
    current_lon = msg2[1] # lon
    #current_alt = msg2[2] # alt
    
    R = 6371000  # Earth radius in meters
    dlat = radians(current_lat - leader_lat)
    dlon = radians(current_lon - leader_lon)
    a = sin(dlat / 2)**2 + cos(radians(leader_lat)) * cos(radians(current_lat)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance #in meters

def get_local_position(vehicle):
    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    100,1)
    msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    pos_x = msg.x # Degrees
    pos_y = msg.y  # Degrees
    pos_z = msg.z  # Meters
    return [pos_x,pos_y,pos_z]

def send_velocity_setpoint(vehicle, vx, vy, vz):

    # Send MAVLink command to set velocity
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (not used)
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,        # type_mask (only vx, vy, vz, yaw_rate)
        0, 0, 0,                    # position (not used)
        vx, vy, vz,                 # velocity in m/s
        0, 0, 0,                    # acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )

def angle_difference_with_heading(lat1, lon1, heading, lat2, lon2): # angle difference relative to origin lat, lon, heading and current lat, lon

    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Compute the difference in the longitudes
    delta_lon = lon2 - lon1

    # Calculate the bearing from the reference point to the second location
    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))
    bearing_to_point = math.atan2(x, y)

    # Convert bearing from radians to degrees
    bearing_to_point = math.degrees(bearing_to_point)

    # Normalize the bearing to a value between 0 and 360 degrees
    bearing_to_point = (bearing_to_point + 360) % 360

    # Calculate the angle difference relative to the reference heading
    angle_diff = (bearing_to_point - heading + 360) % 360

    return angle_diff

def send_position_setpoint(vehicle, pos_x, pos_y, pos_z):

    # Send MAVLink command to set velocity
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (not used)
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b110111111000,        # type_mask (only for postion)
        pos_x, pos_y, pos_z,   # position 
        0, 0, 0,                 # velocity in m/s (not used)
        0, 0, 0,                    # acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )


def distance_between(vehicle,leader_lat,leader_lon):
    
    msg2 = get_global_position(vehicle)
    
    current_lat = msg2[0] # lat
    current_lon = msg2[1] # lon
    #current_alt = msg2[2] # alt
    
    R = 6371000  # Earth radius in meters
    dlat = radians(current_lat - leader_lat)
    dlon = radians(current_lon - leader_lon)
    a = sin(dlat / 2)**2 + cos(radians(leader_lat)) * cos(radians(current_lat)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance #in meters

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
        self.counter = 0
        
        # Subscribing to the /leader/velocity topic
        self.subscription = self.create_subscription(TwistStamped,'/leader/velocity',self.velocity_callback,10)
        
        # Subscribing to the /leader/position topic
        self.subscription1 = self.create_subscription(Point,'/leader/position',self.position_callback,10)
        
        #subscribing to /leader/status
        self.subscription2 = self.create_subscription(String,'/leader/status',self.status_callback,10)

        self.subscription3 = self.create_subscription(Point,'/leader/local_pos',self.localpos_callback,10)


    def position_callback(self, msg):
        # Callback function to process position data
        self.leader_lat = msg.x
        self.leader_lon = msg.y
        self.leader_hdg = msg.z
        self.get_logger().info(f"Leader position: lat={self.leader_lat}, lon={self.leader_lon}, heading={self.leader_hdg}")

    def velocity_callback(self, msg):
        # Callback function to process velocity data
        self.leader_vx = msg.twist.linear.x
        self.leader_vy = msg.twist.linear.y
        self.leader_vz = msg.twist.linear.z
        send_velocity_setpoint(follower1, self.leader_vx, self.leader_vy, 0)  # Sending only vx and vy
        # Print out the velocity components
        self.get_logger().info(f"leader velocity: vx={self.leader_vx} m/s, vy={self.leader_vy} m/s, vz={self.leader_vz} m/s")

    def status_callback(self,msg):
        
        self.status = msg.data
        if self.status == 'ready':
            self.counter += 1
            self.get_logger().info('Leader is armed and ready.')
            if self.counter == 1:
                VehicleMode(follower1, "GUIDED")
                print("in guided mode")
                time.sleep(1)
                arm(follower1)
                time.sleep(5)
                drone_takeoff(follower1, target_alt)

                while True:
                    altitude = abs(get_local_position(follower1)[2])
                    print(f"Altitude: {altitude} m.")
                    if altitude>9:
                        print("Target altitude reached.")
                        break
                
                formation(follower1,self.leader_lat,self.leader_lon,self.leader_hdg,135,8.1,target_alt)
                time.sleep(1)
                lat,lon,alt = get_global_position(follower1)
                new_alt = target_alt + 10
                goto_waypoint(follower1,lat,lon,new_alt)
                while True:
                    altitude = abs(get_local_position(follower1)[2])
                    if altitude>19:
                        break
                    print(f"current altitude: {altitude} m.")
                time.sleep(1)

        else:
            self.get_logger().info('Leader not ready')

    def localpos_callback(self,msg):

        self.pos_x = msg.x
        self.pos_y = msg.y
        self.pos_z = msg.z
        self.get_logger().info(f"local position: x={self.pos_x} m, y={self.pos_y} m, z={self.pos_z} m")
        if abs(self.pos_z)<5:
            VehicleMode(follower1,"LAND")
            time.sleep(1)


   
follower1 = connect('tcp:127.0.0.1:5783')
print("connected")
time.sleep(1)
target_alt = 10

def main(args=None):

    rclpy.init(args=args)

    # Create and start the subscriber node
    drone_subscriber = DroneSubscriber()
    rclpy.spin(drone_subscriber)

if __name__ == '__main__':
    main()
