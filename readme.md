## MultiAgent Systems repo
The entire build is made in Ubuntu 18.04, and the ros2 version used is eloquent.(Assuming these are present in the system)

## Task 1
turtlebot3 and turtlebot3_simulation repositories are present in this repository for eloquent environment.

Follow these steps to install the dependencies and source them.
```sh
sudo apt install ros-eloquent-gazebo-ros-pkgs
sudo apt install ros-eloquent-cartographer
sudo apt install ros-eloquent-cartographer-ros
sudo apt install ros-eloquent-navigation2
sudo apt install ros-eloquent-nav2-bringup
sudo apt install python3-vcstool

git clone <this-repo>
cd <this-repo>
#installing teleop keyboard first, that repository is present inside main turtlebot repository
cd turtlebot3/turtlebot3_teleop
colcon build --symlink-install
source install/setup.bash
#installing complete repository
cd ..
colcon build --symlink-install
source install/setup.bash 
#installing turtlebot3_simulations
cd turtlebot3_simulations
colcon build --symlink-install
source install/setup.bash

echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
source ~/.bashrc

#open a new terminal tab
#source all the directories that are sourced above
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#navigate to the previous tab run
ros2 run turtlebot3_teleop teleop_keyboard
#to navigate the turtlebot using /cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}}' -1
```

To run a simple publisher subscriber goto py_pubsub directory and run the following commands.
The source code is present inside py_pubsub/py_pubsub.
```sh
cd ../py_pubsub
source install/setup.bash
ros2 run py_pubsub talker
#open a new tab and source
ros2 run py_pubsub listener
```

## Task 2

To install mininet_wifi on your system run the following commands in the present repository 
``` sh
cd mininet_wifi
sudo util/install.sh -Wlnfv

#this command sets up one access point(ap1) and two stations(sta1 sta2) 
sudo mn --wifi 

# to check the connections type the following commands in the mininet console
mininet_wifi>nodes 
#the expected output looks like:
#   available nodes are: 
#   ap1 c0 sta1 sta2
#   Level 25:mininet:available nodes are: 
#   ap1 c0 sta1 sta2
#additionally to verify 
mininet_wifi>links #shows connection between sta1-wlan0; sta2-wlan0
mininet_wifi>exit
$sudo mn -c
$sudo mn --wifi --topo linear,2 #launches a network with 2 access points and 2 stations
```
A python script demoing the same command line functionality has been developed inside the mininet_wifi folder. Run the following commands to see the network being formed.

```sh
cd <path-to-this-repository>/mininet_wifi
sudo python3 Mininet_demo.py

mininet_wifi> links
mininet_wifi> nodes

```
