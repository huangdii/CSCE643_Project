from threading import current_thread
import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import rapid_env
from gym.envs.registration import register
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os


class RapidWorldEnv(rapid_env.RapidEnv):
    def __init__(self):
        """
        This Task Env is designed for having the Rapid in the Rapid world
        closed room with columns.
        It will learn how to move around without crashing.
        """
        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = "/home/jiyoon/python3_ws"
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="rl_rapid",
                    launch_file_name="rapid_gazebo.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/rapid/config",
                               yaml_file_name="rapid_world.yaml")


        # Here we will add any init functions prior to starting the MyRobotEnv
        super(RapidWorldEnv, self).__init__(ros_ws_abspath)

        # Only variable needed to be set here
        number_actions = rospy.get_param('/rapid/n_actions')
        self.action_space = spaces.Discrete(number_actions)

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)

        """
        We set the Observation space for the 6 observations
        cube_observations = [
            round(current_disk_roll_vel, 0),
            round(y_distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(y_linear_speed,1),
            round(yaw, 1),
        ]
        """

        # Actions and Observations
        self.linear_forward_speed = rospy.get_param('/rapid/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/rapid/linear_turn_speed')
        self.angular_speed = rospy.get_param('/rapid/angular_speed')
        self.init_linear_forward_speed = rospy.get_param('/rapid/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param('/rapid/init_linear_turn_speed')

        self.new_ranges = rospy.get_param('/rapid/new_ranges')
        self.min_range = rospy.get_param('/rapid/min_range')
        self.max_laser_value = rospy.get_param('/rapid/max_laser_value')
        self.min_laser_value = rospy.get_param('/rapid/min_laser_value')
        self.max_linear_aceleration = rospy.get_param('/rapid/max_linear_aceleration')

	    # Get Desired Point to Get
        self.desired_point = Point()
        self.desired_point.x = rospy.get_param("/rapid/desired_pose_x")
        self.desired_point.y = rospy.get_param("/rapid/desired_pose_y")

        self.desired_point_epsilon = rospy.get_param("/rapid/desired_point_epsilon")



        # We create two arrays based on the binary values that will be assigned
        # In the discretization method.
        laser_scan = self.get_laser_scan()
        num_laser_readings = int(len(laser_scan.ranges)/self.new_ranges)
        high = numpy.full((num_laser_readings), self.max_laser_value)
        low = numpy.full((num_laser_readings), self.min_laser_value)

        # We only use two integers
        self.observation_space = spaces.Box(low, high)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))

        # Rewards
        # self.forwards_reward = rospy.get_param("/rapid/forwards_reward")
        # self.turn_reward = rospy.get_param("/rapid/turn_reward")
        self.closer_to__goal_point = rospy.get_param("/rapid/closer_to_goal_point")
        self.end_episode_points = rospy.get_param("/rapid/end_episode_points")
        self.get_to_goal_point = rospy.get_param("/rapid/get_to_goal_point")
        self.turn_reward = rospy.get_param("/rapid/turn_reward")
        self.cumulated_steps = 0.0


    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base( self.init_linear_forward_speed,
                        self.init_linear_turn_speed,
                        epsilon=0.05,
                        update_rate=10)

        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False
	
	    # Get the initial pose to measure the distance from the GOAL point.
        gt_pose = self.get_base_pose()
        self.previous_distance_from_goal_point = self.get_distance_from_goal_point(gt_pose.position)


    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the turtlebot2
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """

        rospy.logdebug("Start Set Action ==>"+str(action))
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        if action == 0: #FORWARD
            linear_speed = self.linear_forward_speed
            angular_speed = 0.0
            self.last_action = "FORWARDS"
        elif action == 1: #LEFT
            linear_speed = self.linear_turn_speed
            angular_speed = self.angular_speed
            self.last_action = "TURN_LEFT"
        elif action == 2: #RIGHT
            linear_speed = self.linear_turn_speed
            angular_speed = -1*self.angular_speed
            self.last_action = "TURN_RIGHT"

        # We tell TurtleBot2 the linear and angular speed to set to execute
        self.move_base(linear_speed, angular_speed, epsilon=0.05, update_rate=10)

        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to,
        :return:
        """
        rospy.logdebug("Start Get Observation ==>")
        # We get the laser scan data
        laser_scan = self.get_laser_scan()
        # We get the pose data
        gt_pose = self.get_base_pose()
        # We get the orientation of the cube in RPY
        roll, pitch, yaw = self.get_orientation_euler(gt_pose.orientation)
        discretized_observations = self.discretize_scan_observation(    laser_scan,
                                                                        self.new_ranges
                                                                        )
        observations = [round(gt_pose.position.x, 1),
                        round(gt_pose.position.y, 1),
                        round(roll, 1),
                        round(pitch, 1),
                        discretized_observations]
        rospy.logdebug("Observations==>"+str(observations))
        rospy.logdebug("END Get Observation ==>")
        return observations


    def _is_done(self, observations):

        if self._episode_done:
            rospy.logerr("Robot is Too Close to wall==>")
        else:
            rospy.logwarn("Robot is NOT close to a wall ==>")
        # Check the robot reached goal point
        current_position = Point()
        current_position.x = observations[0]
        current_position.y = observations[1]
        
        has_reached_goal_point = self.is_in_goal_position(
            current_position, self.desired_point_epsilon)
        if has_reached_goal_point:
            rospy.logerr("has_reached_goal_point="+str(has_reached_goal_point))
        else:
            rospy.logwarn("has_reached_goal_point="+str(has_reached_goal_point))

        # Now we check if it has crashed based on the imu
        current_orientation = Point()
        current_orientation.x = observations[2]
        current_orientation.y = observations[3]
        robot_flipped = self.robot_has_flipped(current_orientation)
        
        imu_data = self.get_imu()
        linear_acceleration_magnitude = self.get_vector_magnitude(imu_data.linear_acceleration)
        if linear_acceleration_magnitude > self.max_linear_aceleration or robot_flipped:
            rospy.logerr("Robot Crashed!! ==>"+str(linear_acceleration_magnitude)+">"+str(self.max_linear_aceleration))
            self._episode_done = True
        elif has_reached_goal_point:
            rospy.logerr("Robot reached goal point!! ==>"+str(has_reached_goal_point))
            self._episode_done = True
        else:
            rospy.logerr("DIDNT crash Robot ==>"+str(linear_acceleration_magnitude)+">"+str(self.max_linear_aceleration))


        return self._episode_done

    def _compute_reward(self, observations, done):

        current_position = Point()
        current_position.x = observations[0]
        current_position.y = observations[1]

        distance_from_goal_point = self.get_distance_from_goal_point(
            current_position)
        distance_difference = distance_from_goal_point - \
                                self.previous_distance_from_goal_point

        if not done:
            if distance_difference < 0.0:
                rospy.logwarn("Decrease in distance good")
                reward = self.closer_to__goal_point
            elif self.last_action == "TURN_LEFT":
                reward = self.turn_reward
            else:
                reward = 0

            # if self.last_action == "FORWARDS":
            #     reward = self.forwards_reward
            # if self.last_action == "TURN_LEFT":
            #     reward = self.turn_reward * 3
            # else:
            #     reward = self.turn_reward
        else:
            if self.is_in_goal_position(current_position, epsilon=0.5):
                reward = self.get_to_goal_point
            else:
                reward = -1*self.end_episode_points

        self.previous_distance_from_goal_point = distance_from_goal_point


        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward


    # Internal TaskEnv Methods

    def discretize_scan_observation(self,data,new_ranges):
        """
        Discards all the laser readings that are not multiple in index of new_ranges
        value.
        """
        self._episode_done = False

        discretized_ranges = []
        mod = len(data.ranges)/new_ranges

        rospy.logdebug("data=" + str(data))
        rospy.logdebug("new_ranges=" + str(new_ranges))
        rospy.logdebug("mod=" + str(mod))

        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if item == float ('Inf') or numpy.isinf(item):
                    discretized_ranges.append(self.max_laser_value)
                elif numpy.isnan(item):
                    discretized_ranges.append(self.min_laser_value)
                else:
                    discretized_ranges.append(int(item))

                if (self.min_range > item > 0):
                    rospy.logerr("done Validation >>> item=" + str(item)+"< "+str(self.min_range))
                    self._episode_done = True
                else:
                    rospy.logdebug("NOT done Validation >>> item=" + str(item)+"< "+str(self.min_range))


        return discretized_ranges


    def get_vector_magnitude(self, vector):
        """
        It calculated the magnitude of the Vector3 given.
        This is usefull for reading imu accelerations and knowing if there has been
        a crash
        :return:
        """
        contact_force_np = numpy.array((vector.x, vector.y, vector.z))
        force_magnitude = numpy.linalg.norm(contact_force_np)

        return force_magnitude

    def get_base_pose(self):
    
        odom = self.get_odom()
        base_pose = odom.pose.pose

        return base_pose

    def get_distance_from_goal_point(self, current_position):
        """
        Calculates the distance from the current position to the desired point
        :param start_point:
        :return:
        """
        distance = self.get_distance_from_point(current_position,
                                                self.desired_point)

        return distance

    def get_distance_from_point(self, pstart, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((pstart.x, pstart.y, pstart.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = numpy.linalg.norm(a - b)

        return distance

    def is_in_goal_position(self, current_position, epsilon=0.05):
        """
        It return True if the current position is similar to the desired poistion
        """

        is_in_goal_pos = False

        x_pos_plus = self.desired_point.x + epsilon
        x_pos_minus = self.desired_point.x - epsilon
        y_pos_plus = self.desired_point.y + epsilon
        y_pos_minus = self.desired_point.y - epsilon

        x_current = current_position.x
        y_current = current_position.y

        x_pos_are_close = (x_current <= x_pos_plus) and (
                x_current > x_pos_minus)
        y_pos_are_close = (y_current <= y_pos_plus) and (
                y_current > y_pos_minus)

        is_in_goal_pos = x_pos_are_close and y_pos_are_close 

        rospy.logwarn("###### IS GOAL POS ? ######")
        rospy.logwarn("current_position"+str(current_position))
        rospy.logwarn("x_pos_plus"+str(x_pos_plus) +
                      ",x_pos_minus="+str(x_pos_minus))
        rospy.logwarn("y_pos_plus"+str(y_pos_plus) +
                      ",y_pos_minus="+str(y_pos_minus))
        rospy.logwarn("x_pos_are_close"+str(x_pos_are_close))
        rospy.logwarn("y_pos_are_close"+str(y_pos_are_close))
        rospy.logwarn("is_in_goal_pos"+str(is_in_goal_pos))
        rospy.logwarn("############")

        return is_in_goal_pos

    def euler_from_quat(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def get_orientation_euler(self, quaternion_vector):
        # We convert from quaternions to euler
        orientation_list = [quaternion_vector.x,
                            quaternion_vector.y,
                            quaternion_vector.z,
                            quaternion_vector.w]

        roll, pitch, yaw = self.euler_from_quat(quaternion_vector.x, quaternion_vector.y, quaternion_vector.z, quaternion_vector.w)
        return roll, pitch, yaw

    # def get_base_rpy(self):
    
    #     imu = self.get_imu()
    #     base_orientation = imu.orientation

    #     euler_rpy = Vector3()
    #     euler = self.euler_from_quat(base_orientation.x,
    #                                     base_orientation.y,
    #                                     base_orientation.z,
    #                                     base_orientation.w
    #                                   )
    #     euler_rpy.x = euler[0]
    #     euler_rpy.y = euler[1]
    #     euler_rpy.z = euler[2]

    #     return euler_rpy


    def robot_has_flipped(self, current_orientation):
        """
        Based on the orientation RPY given states if the drone has flipped
        """
        has_flipped = True

        self.max_roll = rospy.get_param("/rapid/max_roll")
        self.max_pitch = rospy.get_param("/rapid/max_pitch")

        rospy.logwarn("#### HAS FLIPPED? ########")
        rospy.logwarn("RPY current_orientation"+str(current_orientation))
        rospy.logwarn("max_roll"+str(self.max_roll) +
                      ",min_roll="+str(-1*self.max_roll))
        rospy.logwarn("max_pitch"+str(self.max_pitch) +
                      ",min_pitch="+str(-1*self.max_pitch))
        rospy.logwarn("############")

        if current_orientation.x > -1*self.max_roll and current_orientation.x <= self.max_roll:
            if current_orientation.y > -1*self.max_pitch and current_orientation.y <= self.max_pitch:
                has_flipped = False

        return has_flipped
