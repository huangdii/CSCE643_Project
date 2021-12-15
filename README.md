# Mobile Robot Project - TAMU CSCE 643 
### Comparison between Dynamic Window Approach based navigation and Reinforcement Learning based navigation

## Download ROS full Packages 
- Ubuntu 18.04 | ROS Melodic full package including Gazebo
- Python 3.6 virtual environment
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-melodic.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full     // this will install gazebo simulators, perception, desktop.
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```

## Setup packages
- [rl_rapid] git clone https://github.com/huangdii/MobileRobotRL
- [OpenAI] git clone https://bitbucket.org/theconstructcore/openai_ros.git
- Download (baselines, geometry, geometry2, turtlebot3) packages

## Copy openai folder's file to real OpenAI package's folder to make rapid Environment
```bash
$ cp MobileRobotRL/openai/robot_envs/rapid_env.py ~/openai_ros/src/openai_ros/robot_envs/rapid_env.py
$ cp -rf MobileRobotRL/openai/task_envs/rapid ~/openai_ros/src/openai_ros/task_envs/
$ cp MobileRobotRL/openai/task_envs_list.py ~/openai_ros/src/openai_ros/task_envs/task_envs.list.py
```

## Build - catkin make
```bash
$ catkin_make -DPYTHON_EXECUTABLE:FILEPATH=/["PATH-TO-YOUR-VIRTUALENV"]/py3venv/bin/python
```
## Q-Learning to make a mobile robot head to the goal position without collision
```bash
$ roslaunch rl_rapid rapid_training.launch
```
## Draw a plot and save the results (It will save in the rapid_results folder)
```bash
1. Plot of Rewards - Episodes
$ python ~/rl_rapid/src/plot.py
2. Plot of Number of Steps of each Episode - Episodes
$ python ~/rl_rapid/src/plot_steps.py 
```

## Dynamic Window Approach based Navigation without a global map
```bash
$ (term1) roslaunch rl_rapid rapid_gazebo.launch    // setup gazebo simulator
$ (term2) roslaunch rl_rapid nav_blankmap.launch    // make the robot move on rviz
$ (term3) python ~/rl_rapid/src/waypoint_nav.py     // set the goal position and make the robot go to there
``` 
