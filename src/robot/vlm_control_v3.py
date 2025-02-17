#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, Float32
from ds4_driver.msg import Status, Feedback
import random


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import  Twist, PoseStamped, Pose
import dynamic_reconfigure.client
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionGoal
import copy


 

#!/usr/bin/env python3
# license removed for brevity
# from https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
import os
import rospy
import actionlib
import time
 
import numpy as np
import random
import socket
import sys
import math
import pygame


class RobEnv():
    def __init__(self):
        self.rate_rob = rospy.Rate(10)
        self.odomm = Odometry()
        self.costmapp = OccupancyGrid()
        self.cmd_vell = np.zeros(2)
        self.cmd_vel_dwaa = np.zeros(2)
        self.scann = LaserScan()
        self.buttonn = 0
        self.cross = 0
        self.stop_run = 0
        self.x_axis = 0
        self.y_axis = 0
        self.rate_rob = rospy.Rate(10)

        self.goall = Pose()

        self.base_scann = LaserScan()
        # Global planner
        self.gpp = 0

        # Supervisor
        self.supervisor_counter = 0
        self.max_supervisor = 40

        # Sensors
        print("Suscribing to scan")
        rospy.Subscriber("scan", LaserScan, self._scan_callback)
        print("Suscribed!")

        print("Suscribing to odom")
        rospy.Subscriber("t265/odom/sample", Odometry, self._odom_callback)
        print("Suscribed!")

        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick detected.")
            rospy.signal_shutdown("No joystick detected.")
            return

        # Initialize joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Joystick connected: {self.joystick.get_name()}")

        # Gmapping
        # print("Publishing to base_scan")
        # self.base_scan_pub = rospy.Publisher('base_scan', LaserScan, queue_size=10)
        # print("Published!")

        # Local planner
        print("Suscribing to cmd_vel")
        rospy.Subscriber('/cmd_vel/managed', Twist, self._cmd_vel_callback)
        print("Suscribed!")

        print("Suscribing to cmd_vel_dwa")
        rospy.Subscriber('/cmd_vel_dwa', Twist, self._cmd_vel_dwa_callback)
        print("Suscribed!")

        # Costmap
        print("Suscribing to costmap")
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self._costmap_callback)
        print("Suscribed!")

        print("Waiting for move_base server")
        rospy.wait_for_service('/move_base/clear_costmaps')
        self._clear_costmap_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        print("Connected!")

        print("Suscribing to goal")
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self._goal_callback_action)
        print("Suscribed!")


        # Publishers
        print("here")
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Actions
        # self._tuning_client_local = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS',timeout=4, config_callback=None)
        # self._tuning_client_inflation = dynamic_reconfigure.client.Client('move_base/local_costmap/inflation_layer',timeout=4, config_callback=None)
        print("there")

        super(RobEnv, self).__init__()


    def _button_callback(self):
        # Check for joystick events (buttons and axis)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rospy.signal_shutdown("Quit event")

        # Read joystick axis values (left stick X and Y)
        self.x_axis = self.joystick.get_axis(0)  # Left stick X-axis
        self.y_axis = self.joystick.get_axis(1)  # Left stick Y-axis

        # Read joystick button states (for the circle button logic)
        button_circle = self.joystick.get_button(2)  # Typically the 'Circle' button


        # Toggle stop_run when the Circle button is pressed
        if button_circle == 1:  # Button is pressed
            self.stop_run = 1 - self.stop_run
            print(f"Stop run toggled: {self.stop_run}")
            pygame.time.wait(200)  # Avoid double triggering by adding a small delay

    def _scan_callback(self, msg):

        # Update lidar reading
        current_time = rospy.Time.now()
        self.scann.header.stamp = current_time
        self.scann.header.frame_id = msg.header.frame_id 
        self.scann.angle_min = msg.angle_min
        self.scann.angle_max = msg.angle_max
        self.scann.angle_increment = msg.angle_increment
        self.scann.time_increment = msg.time_increment
        self.scann.range_min = msg.range_min
        self.scann.range_max = msg.range_max
        self.scann.ranges = msg.ranges
        self.scann.ranges = self.downsample(self.scann.ranges)
        self.scann.intensities = msg.intensities
        #print(f"laser length: {len(self.scann.ranges)}")
        #print(self.scann.ranges)

        # Publish downsampled for gmapping
        # self.base_scann.header.stamp = current_time
        # self.base_scann.header.frame_id = msg.header.frame_id 
        # self.base_scann.angle_min = msg.angle_min
        # self.base_scann.angle_max = msg.angle_max
        # self.base_scann.angle_increment = msg.angle_increment
        # self.base_scann.time_increment = msg.time_increment
        # self.base_scann.range_min = msg.range_min
        # self.base_scann.range_max = msg.range_max
        # self.base_scann.ranges = self.downsample(self.scann.ranges)
        # self.base_scann.ranges = self.scann.ranges
        # self.base_scann.intensities = msg.intensities

        # self.base_scan_pub.publish(self.base_scann)
        #print(f"laser length: {len(self.base_scann.ranges)}")

    def _odom_callback(self, msg):

        # Update odometry reading
        current_time = rospy.Time.now()
        self.odomm.header.stamp = current_time
        self.odomm.pose.pose.position.x = msg.pose.pose.position.x
        self.odomm.pose.pose.position.y = msg.pose.pose.position.y
        self.odomm.pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.odomm.pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.odomm.pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.odomm.pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.odomm.twist.twist.linear.x = msg.twist.twist.linear.x
        self.odomm.twist.twist.linear.y = msg.twist.twist.linear.y
        self.odomm.twist.twist.linear.z = msg.twist.twist.linear.z
        self.odomm.twist.twist.angular.x = msg.twist.twist.angular.x
        self.odomm.twist.twist.angular.y = msg.twist.twist.angular.y
        self.odomm.twist.twist.angular.z = msg.twist.twist.angular.z

        #(roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        #print(f"position: {self.odomm.pose.pose.position.x}, {self.odomm.pose.pose.position.y}, {yaw}")

    def _costmap_callback(self, msg):
        current_time = rospy.Time.now()
        self.costmapp.header.stamp = current_time
        self.costmapp.header.frame_id = msg.header.frame_id
        self.costmapp.info.height = msg.info.height
        self.costmapp.info.width = msg.info.width
        self.costmapp.info.origin.position.x = msg.info.origin.position.x
        self.costmapp.info.origin.position.y = msg.info.origin.position.y
        self.costmapp.info.origin.orientation.x = msg.info.origin.orientation.x 
        self.costmapp.info.origin.orientation.y = msg.info.origin.orientation.y 
        self.costmapp.info.origin.orientation.z = msg.info.origin.orientation.z 
        self.costmapp.info.origin.orientation.w = msg.info.origin.orientation.w

        self.costmapp.data = msg.data

    def _cmd_vel_callback(self, msg):
        self.cmd_vell[0] = msg.linear.x
        self.cmd_vell[1] = msg.angular.z

    def _cmd_vel_dwa_callback(self, msg):
        self.cmd_vel_dwaa[0] = msg.linear.x
        self.cmd_vel_dwaa[1] = msg.angular.z

    
    def _goal_callback(self, msg):
        self.goall.position.x = msg.pose.position.x
        self.goall.position.y = msg.pose.position.y

    def _goal_callback_action(self, msg):
        goal_position = msg.goal.target_pose.pose.position
        self.goall.position.x = goal_position.x
        self.goall.position.y = goal_position.y

    def clear_costmap(self):
        # Clear costmaps

        clear_costmap_object = EmptyRequest()

        result = self._clear_costmap_service(clear_costmap_object)
        #print("Clearing costmaps" + str(result))

    def get_odom(self):
        odom = copy.deepcopy(self.odomm)
        return odom

    def get_costmap(self):
        costmap = copy.deepcopy(self.costmapp)

        return costmap

    def get_cmd_vel(self):
        cmd_vel = copy.deepcopy(self.cmd_vell)

        return cmd_vel

    def get_cmd_vel_dwa(self):
        cmd_vel = copy.deepcopy(self.cmd_vel_dwaa)

        return cmd_vel
    
    def get_scan(self):
        scan = copy.deepcopy(self.scann)

        return scan

    def get_gp(self):
        gp = copy.deepcopy(self.gpp)
        return gp

    def get_goal(self):
        goal = copy.deepcopy(self.goall)
        return goal

    def get_reward(self):
        reward = copy.deepcopy(self.cross)

        return reward

    def send_vel(self, vel):
        cmd_vel = Twist()
        if len(vel) == 2:
            # Clamp values between -1.5 and 1.5
            cmd_vel.linear.x = np.clip(vel[0], -1.5, 1.5)
            cmd_vel.angular.z = np.clip(vel[1], -1.5, 1.5)
        else:
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0

        self._vel_pub.publish(cmd_vel)

    def downsample(self, scan_ranges):
        len_scan = len(scan_ranges)
        desired_len = 720
        rate = len_scan/desired_len
        new_scan = np.zeros(desired_len)
        for i in range(desired_len):
            new_scan[i] = scan_ranges[int(math.floor(rate*i))]
        return new_scan



class TaskEnv(RobEnv):
    """ Class for PTDRL Task environment """

    def __init__(self):
        self.robot_ip = '172.18.8.23'
        self.robot_port = 10093
        self.circle_points = self.generate_circle_points(radius=3, num_points=3)
        self.current_point_idx = 0
        self.last_goal_time = -1
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.max_vel = 1
        print("Initialized TaskEnv")

        RobEnv.__init__(self)

    def generate_circle_points(self, radius, num_points):
        """ Generate points equally spaced in a circle """
        points = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.append((x, y))
        return points

    def new_goal(self, x, y):
        """ Send navigation goal to move_base """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # Default orientation
        rospy.loginfo(f"Navigating to ({x}, {y})")
        self.last_goal_time = -1
        self.client.send_goal(goal)

    def set_max_vel(self, environment):
        envs = {
            "office": 0.5,
            "open space": 1.5,
            "corridor": 0.75,
            "cafeteria": 0.5,
            "crowded area": 0.25,
            "park": 1.5
        }

        # Check if the environment exists in the dictionary and set max_vel
        self.max_vel = envs.get(environment, 1.0) 

    def listen(self):
        """ Listen for incoming commands over WiFi """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow reusing the same port
        self.sock.bind((self.robot_ip, self.robot_port))
        self.sock.listen(1)
        self.sock.settimeout(1.0)  # Prevent blocking indefinitely
        rospy.loginfo("Listening for connections...")

        try:
            while not rospy.is_shutdown():
                try:
                    connection, client_address = self.sock.accept()
                except socket.timeout:
                    continue  # Timeout to check for shutdown signals

                rospy.loginfo(f"Connection from {client_address}")

                try:
                    while not rospy.is_shutdown():
                        self._button_callback() # Read joystick

                        data = connection.recv(128).decode('utf-8')
                        if not data:
                            break

                        rospy.loginfo(f"Received: {data}")
                        parts = data.split(">")
                        state = parts[0][1:]  # Remove the leading '<'
                        print(f"state: {state}")
                        message = parts[1][1:]  # Remove the leading '<' in message
                        print(f"message: {message}")

                        if not self.stop_run:
                            if state == "NAVIGATE":
                                if time.time() - self.last_goal_time >= 10:
                                    self.last_goal_time = time.time()
                                    self.new_goal(*self.circle_points[self.current_point_idx])
                                    self.current_point_idx = (self.current_point_idx + 1) % len(self.circle_points)
                                dwa = self.get_cmd_vel_dwa()
                                self.set_max_vel(message)
                                #self.send_vel(dwa*self.max_vel)
                                print("navigate")
                            elif state == "FOLLOW":
                                try:
                                    parameters = list(map(float, message.split()))
                                except ValueError:
                                    rospy.logwarn("Invalid message values.")
                                    continue
                                print(f"parameters: {parameters}")
                                self.send_vel(parameters)
                            elif state == "REMOTE":
                                # Map the received message to corresponding commands
                                command_map = {
                                    "x": [0, 0],
                                    "a": [0, 0.2],
                                    "s": [-0.2, 0],
                                    "d": [0, -0.2],
                                    "w": [0.2, 0]
                                }
                                # Check if the received message is one of the valid commands
                                if message in command_map:
                                    parameters = command_map[message]  # Map the message to the corresponding command
                                    print(f"Mapped remote command: {parameters}")
                                else:
                                    rospy.logwarn(f"Invalid remote command: {message}")
                                    continue
                                print(f"parameters: {parameters}")
                                self.send_vel(parameters)
                        else:
                            self.send_vel([-self.y_axis, self.x_axis])
                        self.rate_rob.sleep()

                except (socket.error, ConnectionResetError) as e:
                    rospy.logwarn(f"Connection error: {e}")

                finally:
                    connection.close()

        except KeyboardInterrupt:
            rospy.loginfo("Shutting down server...")

        finally:
            self.sock.close()
            rospy.loginfo("Socket closed.")

    def run(self):
        """ Start the environment """
        try:
            self.listen()
        except KeyboardInterrupt:
            rospy.loginfo("TaskEnv stopped by user.")
        finally:
            rospy.signal_shutdown("Shutdown requested")  # Ensures clean exit

if __name__ == "__main__":
    rospy.init_node('init_state_action_collection', disable_signals=True)  # Prevent ROS from interfering with Ctrl+C
    env = TaskEnv()
    env.run()
