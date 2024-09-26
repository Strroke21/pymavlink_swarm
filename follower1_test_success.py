#!/usr/bin/env python3

import math
from pymavlink import mavutil
from math import radians, cos, sin, sqrt, atan2
import time
import pyudev
#from receiver_rfd900x_multi_v2 import mavlink_receiver_rfd900x, mavlink_data_queue
from receiver_rfd900x import mavlink_receiver_rfd900x, mavlink_data_queue


def connect(connection_string):

    vehicle =  mavutil.mavlink_connection(connection_string)

    return vehicle

def arm(vehicle):
    #arm the drone
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
    	                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
def VehicleMode(vehicle,mode):

    modes = ["STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED", "LOITER", "RTL", "CIRCLE", "LAND"]
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



def find_device_address():
    global rfd900x_address
    context = pyudev.Context()
    devices = context.list_devices(subsystem = 'tty')
    address = list()
    for device in devices:
        # print(device)
        if device.get('ID_VENDOR_ID') == '10c4' and device.get('ID_MODEL_ID') == 'ea60' and device.device_path.split('/')[-4] == '1-1.1:1.0':
            # print(device.device_node, device.get('ID_VEDOR'), device.get('ID_VEDOR'), device.get('ID_MODEL'), device.get('ID_SERIAL_SHORT'), device.device_path)
            rfd900x_address = device.device_node 


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


def formation(angle,distance,form_alt):
    counter = 0
    while True:
        ######## leader ######
        data =  mavlink_data_queue.get()
        current_pos = data[1]
        if current_pos:
            counter+=1
            lat_lead = current_pos['lat']/1e7
            lon_lead = current_pos['lon']/1e7
            lead_yaw = (current_pos['hdg'])/100
            print(f"Leader Heading: {lead_yaw} deg.")
            ######### follower1 ######
            yaw1 = angle #on the right
            dist1 = distance#change this distance to add new follower
            lat1, lon1 = relative_pos(lat_lead, lon_lead, dist1, lead_yaw, yaw1)
            print(lat1, lon1)
            ##### send msg to follower1 drone #############

            if counter==1:
                goto_waypoint(follower1,lat1,lon1,form_alt) #altitude 1
                print("formation command sent.")
                time.sleep(0.15)
    
            dist_to_target = distance_between(follower1,lat1,lon1)
            print(f"distance to formation: {round(dist_to_target,2)} m.")
            if dist_to_target<=2:
                time.sleep(1)
                break

        else:
            print("no data retrieved from the leader.")


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

baud_rate = 57600
mavlink_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud_rate)
drone_data = mavlink_receiver_rfd900x(mavlink_connection)
drone_data.start_receiving()

#-------------------------------------arming checkpoint for follower from leader------------------------------
while True:
    data = mavlink_data_queue.get()
    if data:
        alt = abs(data[1]['relative_alt'])/100
        if alt >=10:
            break

        print(f"Leader Altitude: {round(alt,2)} m.")
    else:
        print("no data received from leader")

#----------- Variables-----------
counter = 0
relativePosition = 'right' #follower1 in the straight right of the leader
distance = 10 #10 m in right
angle = 90 #angular pos relative to leader
form_alt = 10 #initial formation altitude
tar_alt = 20

#---------- follower connection ---------
follower1 = connect('/dev/ttyACM0')#('tcp:127.0.0.1:5763')
print("follower1 connected.")

#-------------- arming and takeoff checkpoint 

while True:
#----------guided mode ----------
    VehicleMode(follower1,"GUIDED")
    print("follower1 in GUIDED mode")
    time.sleep(1)
    #------- arm --------
    arm(follower1)
    print("arming the follower1")
    time.sleep(5)
    #---------- guided takeoff -----------
    drone_takeoff(follower1,form_alt)
    print("taking off follower1")
    time.sleep(3)
    alt = abs(get_local_position(follower1)[2])
    print(f"Initial altitude Check: {alt} m.")
    if alt>=0.5:
        break


#--------------------- checkpoint for target altitude --------------
while True:
    altitude = abs(get_local_position(follower1)[2])
    print(f"Altitude: {altitude} m.")
    if altitude>(form_alt-2):
        time.sleep(2)
        print("Target altitude reached.")
        break

#######------------------- main ------------------ ######
while True:
    #position fetch
    pos = mavlink_data_queue.get()
    print(pos)
    counter+=1
    if counter==1:
        formation(angle,distance,form_alt) #initial formation command
        lat,lon,_ = get_global_position(follower1)
        goto_waypoint(follower1,lat,lon,tar_alt) #final altitude
        #------ checkpoint for final altitude ------------------
        while True:
            altitude = abs(get_local_position(follower1)[2])
            if altitude>(tar_alt-2):
                time.sleep(2)
                break
            print(f"current altitude: {altitude} m.")
        time.sleep(1)

    if pos:
        #fetch leader velocity in all direction in m/s
        lead_vx = pos[1]['vx']/100
        lead_vy = pos[1]['vy']/100
        lead_vz = pos[1]['vz']/100
        rel_alt = pos[1]['relative_alt']/100
        send_velocity_setpoint(follower1,lead_vx,lead_vy,0) #sending only vx and vy
        print(f"leader vx: {lead_vx} m/s leader vy: {lead_vy} m/s leader vz: {lead_vz} m/s")

        if (lead_vx <= abs(0.2)) and (lead_vy <= abs(0.2)):

            formation(angle,distance,tar_alt)   #position correction with final altitude   

        if abs(rel_alt) <= 5: #this altitude should be local not relative just for now we are using this
            VehicleMode(follower1,"LAND")
            time.sleep(1)

        else:
            pass

        
       
