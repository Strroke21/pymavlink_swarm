# 1. Gabebo garden/harmonic install on ubuntu 22 jammy
https://gazebosim.org/docs/garden/install_ubuntu/

* Allow system to dowload resources for gazebo:
``` sudo ufw allow from system_ip_address ```

# 2. Ros2 humble installation for gazebo garden/harmonic
[Getting Started with ROS2: Install and Setup ROS2 Humble on Ubuntu 22.04(LTS)](https://medium.com/spinor/getting-started-with-ros2-install-and-setup-ros2-humble-on-ubuntu-22-04-lts-ad718d4a3ac2)

# 3. Gazebo and ros2 humble plugin
https://gazebosim.org/docs/latest/ros_installation/

# 4. Ardupilot Plugin for gazebo garden/harmonic 
https://ardupilot.org/dev/docs/sitl-with-gazebo.html

* [Note: install harmonic (stable than garden) while cloning add  - -recursive to clone all dependencies][Note: dont forget to replace harmonic with garden and while cloning add  - -recursive to clone all dependencies]

# 5.command to launch multiple ardupilot sitl:

``` sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=udp:127.0.0.1:14551 -I1 ```
``` sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=udp:127.0.0.1:14551 -I2 ```

[note: replace -I1 with I2, I3… for multiple instances ]
