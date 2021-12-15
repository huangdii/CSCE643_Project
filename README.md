# Mobile Robot Project - Comparison between Dynamic Window Approach based navigation and Reinforcement Learning based navigation

## Download ROS full Packages 
- Ubuntu 18.04 | ROS Melodic full package including Gazebo
- Python 3.6 virtual environment


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

## build , catkin make
```bash
$ catkin_make -DPYTHON_EXECUTABLE:FILEPATH=/[PATH-TO-VIRTUALENV]/py3venv/bin/python
```
## Q-Learning to make a mobile robot head to the goal position without collision
```bash
$ roslaunch rl_rapid rapid_training.launch $
```
## Make a plot and save the results (It will save in the rapid_results folder)
```bash
$ python ~/rl_rapid/src/plot.py
```

## Dynamic Window Approach based Navigation without a global map
```bash
$ (term1) roslaunch rl_rapid rapid_gazebo.launch
$ (term2) roslaunch rl_rapid nav_blankmap.launch
``` 
