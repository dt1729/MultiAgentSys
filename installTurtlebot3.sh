mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
vcs import src < turtlebot3.repos

colcon build --symlink-install

#Source the repository
#echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc


#echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
#echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
source ~/.bashrc

