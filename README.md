# Multicopter Swarm

# 1. Gabebo garden/harmonic install on ubuntu 22 jammy
https://gazebosim.org/docs/garden/install_ubuntu/

* Allow system to dowload resources for gazebo:

``` sudo ufw allow from system_ip_address ```

# 2. Ros2 humble installation for gazebo garden/harmonic
[Getting Started with ROS2: Install and Setup ROS2 Humble on Ubuntu 22.04(LTS)](https://medium.com/spinor/getting-started-with-ros2-install-and-setup-ros2-humble-on-ubuntu-22-04-lts-ad718d4a3ac2)

# 3. Gazebo garden/harmonic and ros2 humble plugin
https://gazebosim.org/docs/latest/ros_installation/

# 4. Ardupilot Plugin for gazebo garden/harmonic 
https://ardupilot.org/dev/docs/sitl-with-gazebo.html

* [Note: install harmonic (stable than garden) while cloning add  - -recursive to clone all dependencies][Note: dont forget to replace harmonic with garden and while cloning add  - -recursive to clone all dependencies]

# 5. command to launch multiple ardupilot sitl:

``` sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=udp:127.0.0.1:14551 -I1 ```

``` sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=udp:127.0.0.1:14551 -I2 ```

[note: replace -I1 with I2, I3… for multiple instances ]

# 6. Spawn multiple drones in gazebo garden/harmonic
* make multiple copies of multirotor model with unique names as per your requirement
* models can be found in the follwoing folder

  ```cd /gz_ws/src/ardupilot_gazebo/models```
* edit communication fdm address of ardupilot plugin in multirotr model file

``` cd /gz_ws/src/ardupilot_gazebo/models/follower1 ```
[note: follower1 is one of my multicopters name ]

* edit model.sdf file for each multicopter model to communicate will ardupilot sitl instances:
  
  ```
    <plugin name="ArduPilotPlugin"
      filename="ArduPilotPlugin">
      <!-- Port settings -->
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9012</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
  
  ```

[Note: The fdm port should be unique for every instance replace 9022 by adding 10 to it for new multicopter instance]
* after modification include those different multicopters in world's model.sdf file it can be found in the following directory:

``` cd /gz_ws/src/ardupilot_gazebo/worlds ```

*Include all multicopters like this

```     <!-- LEADER -->
    <include>
      <uri>model://leader</uri>
      <pose degrees="true">0 0 0.195 0 0 90</pose>
    </include>
    
     <!-- follower1 -->
    <include>
      <uri>model://follower1</uri>
      <pose degrees="true">2 0 0.195 0 0 90</pose>
    </include>
    
    <!-- follower2 -->
    <include>
      <uri>model://follower2</uri>
      <pose degrees="true">0 2 0.195 0 0 90</pose>
    </include>
    
    <!-- follower3 -->
    <include>
      <uri>model://follower3</uri>
      <pose degrees="true">2 2 0.195 0 0 90</pose>
    </include>
```

# 7. Launching Ardupilot SITL and Gazebo
* each instance should be in different terminal
* Ardupilot sitl Launch
  
``` sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=udp:127.0.0.1:14551 -I1 ```

``` sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=udp:127.0.0.1:14551 -I2 ```

``` sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=udp:127.0.0.1:14551 -I3 ```
* Gazebo Garden/harmonic launch
  
``` gz sim -v4 -r swarm.sdf ```

# 8. Broadcast Leader Position

``` cd pymavlink_swarm ```

``` python3 sim_leader.py ```

# 9. Run follower script 

``` python3 sim_follower1.py ```

``` python3 sim_follower2.py ```

``` python3 sim_follower3.py ```


  

