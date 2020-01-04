CHOOSE_ROS_DISTRO=dashing
sudo ./create_cpuset.sh
source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash
cd test_delays
colcon build
source ./install/setup.bash
cd ..
