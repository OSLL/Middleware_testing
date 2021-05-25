# ROS2 testing guid

## ROS2 installing (only for ubuntu 18.04)

```
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-dashing-desktop
```

## Setting up the environment

``` source /opt/ros/dashing/setup.bash ```

## Building project

1. ```cd dds_testing/ROS2/src/```
2. ```colcon build```
3. ```source ./install/setup.sh```

## Running program

To run the program use command: ```ros2 run ros2_node node```
To get info about parameters use: ```ros2 run ros2_node node --help```

