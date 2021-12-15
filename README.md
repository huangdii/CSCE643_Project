# Mobile Robot Project - 2021 Fall TAMU CSCE 643 
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
- These files should be located in the openai_ros package to utilize openai gym.
```bash
$ cp MobileRobotRL/openai/robot_envs/rapid_env.py ~/openai_ros/src/openai_ros/robot_envs/rapid_env.py
$ cp -rf MobileRobotRL/openai/task_envs/rapid ~/openai_ros/src/openai_ros/task_envs/
$ cp MobileRobotRL/openai/task_envs_list.py ~/openai_ros/src/openai_ros/task_envs/task_envs.list.py
```
#### Or you can add the following lines in  ~/openai_ros/src/openai_ros/task_envs/task_envs.list.py file. You can customize your env
```bash
    elif task_env == 'RapidWorld-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.rapid.rapid_world:RapidWorldEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.rapid import rapid_world
``` 

## Build - catkin make
```bash
$ catkin_make -DPYTHON_EXECUTABLE:FILEPATH=/["PATH-TO-YOUR-VIRTUALENV"]/bin/python
```
## Q-Learning to make a mobile robot head to the goal position without collision
```bash
$ roslaunch rl_rapid rapid_training.launch
```

#### If you want to change qlearn parameters...
```bash
$ vim ~/MobileRobotRL/rl_rapid/config/rapid_params.yaml
```

- Customize the qlearn parameters: alpha, gamma, epsilon, epsilon_discount, number of episodes, number of steps, ....
```bash
rapid: #namespace
    task_and_robot_environment_name: 'RapidWorld-v0'
    ros_ws_abspath: "/home/jiyoon/python3_ws"
    running_step: 0.04 
    pos_step: 0.016     
    
    #qlearn parameters
    alpha: 0.1
    gamma: 0.7
    epsilon: 0.9
    epsilon_discount: 0.999
    nepisodes: 500
    nsteps: 1000
    running_step: 0.06
```

- Customize the other parameters regarding gazebo environment
```bash
rapid: #namespace

    n_actions: 3 # We have 3 actions, Forwards,TurnLeft,TurnRight

    speed_step: 1.0 # Time to wait in the reset phases

    linear_forward_speed: 0.7 # Speed for going fowards / 0.7
    linear_turn_speed: 0.6 # Linear speed when turning / 0.1
    angular_speed: 0.9 # Angular speed when turning Left or Right
    init_linear_forward_speed: 0.3 # Initial linear speed in which we start each episode
    init_linear_turn_speed: 0.2 # Initial angular speed in shich we start each episode
    
    new_ranges: 5 # How many laser readings 
    min_range: 0.2 # Minimum meters below which we consider we have crashed
    max_laser_value: 6 # Value considered Ok, no wall
    min_laser_value: 0 # Value considered there is an obstacle or crashed
    max_linear_aceleration: 18.0 # Linear acceleration value in which we consider RAPID has crashed into something

    max_roll: 1.57 # pi/2
    max_pitch: 1.57 # pi/2

    desired_pose_x: -1.23  # Goal point position
    desired_pose_y: 2.13  # Goal point position
    desired_point_epsilon: 0.1  # Accuracy difference between Goal point and current position
    
    turn_reward: 5 # Points Given to turn as action
    get_to_goal_point: 2000 # Points given when arrived at goal point
    closer_to_goal_point: 100 # Points given when closer to goal point
    end_episode_points: 300 # Points given when ending an episode
```


## Stop simulator 
```bash
Use ctrl+c in the terminal that you give command of "roslaunch rl_rapid rapid_training.launch"
```
- Shutting down the gazebo simulator will make two json files under rapid_results folder.


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

#### You can customize parameters for DWA planner 
```bash
vim ~/rl_rapid/include/robot/params/dwa_local_planner_params.yaml
```


## I recorded every video using OBS Studio