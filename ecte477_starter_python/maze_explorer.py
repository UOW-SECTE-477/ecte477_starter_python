#!/usr/bin/env python3
# Simple Explorer
# Written by Jeff Moscrop
# March 2024

import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler as tquat
from tf_transformations import euler_from_quaternion as teul

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class TurtlebotExplore(Node):

    def __init__(self):
        super().__init__('turtlebot_explorer_node')
        self.initPose = False
        self.setInitPose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

        qos_policy = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        
        self.laserSub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_policy)
        self.odomSub = self.create_subscription(Odometry, '/odom', self.odom_callback, 5)
        self.move_action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal = None
        self.laserSub
        self.odomSub
        self.ahead = 0
        self.right = 0
        self.behind = 0
        self.left = 0
        self.px = 0.0
        self.py = 0.0
        self.rotAngle = math.pi/4
        self.rob_dir = [0.0, 0.0]
        self.move_dir = [0.0, 0.0]
        self.directions = [[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]]
        self.goRight = {0:1, 1:2, 2:3, 3:0}
        self.goLeft = {0:3, 1:0, 2:1, 3:2}
        self.laser_done = False
        self.odom_done = False
        self.start = True
        self.finished = False
        self.goal_reached = True
        self.goal_stack = []

        while not self.finished:
            if self.goal_reached:
                while not self.laser_done or not self.odom_done:
                    self.get_logger().info('Getting sensor update')
                    rclpy.spin_once(self, timeout_sec=1.0)  # wait for lidar and odom callbacks
                self.laser_done = False
                self.odom_done = False
                if self.right > 1.1:
                    self.set_goal(-1, self.right)
                if self.left > 1.1:
                    self.set_goal(1, self.left)
                if self.ahead > 1.1: 
                    self.set_goal(0, self.ahead)
                if not self.goal_stack:
                    if self.start:
                        self.set_goal(2,0)
                    else:
                        self.get_logger().info('Exploration Completed')
                        self.finished = True
                        break
                self.setPose(self.goal_stack.pop())
                self.goGoal(self.goal)

    def set_goal(self, dir, move):
        self.start = False
        if move > 1.5:
            move = 1.5
        else:
            move = move/2
        self.move_dir = self.rob_dir
        if dir == 1:
            self.rotate()
        elif dir == 2:
            self.rotate()
            self.rotate()
        elif dir == -1:
            self.rotate()
            self.rotate()
            self.rotate()
        if self.move_dir == [1.0, 0.0]:
            nOrient = 0
        elif self.move_dir == [0.0, 1.0]:
            nOrient = 2*self.rotAngle
        elif self.move_dir == [-1.0, 0.0]:
            nOrient = 4*self.rotAngle
        elif self.move_dir == [0.0, -1.0]:
            nOrient = -2*self.rotAngle
        [qx,qy,qz,qw] = tquat(0,0,nOrient)
        move_x = self.move_dir[0] * move
        move_y = self.move_dir[1] * move
        x = self.px + move_x
        y = self.py + move_y
        new_goal = [x, y, qz, qw]
        if self.goal_stack:
            self.compare_goal(new_goal)
        self.goal_stack.append(new_goal)

    def compare_goal(self,new):
        prev = self.goal_stack[-1]
        if (new[2] == prev[2]) and (new[3] == prev[3]):
            if (abs(new[0] - prev[0]) < 1) and (abs(new[1] - prev[1]) < 1):
                self.goal_stack.pop()


    def rotate(self):
        for i in range(0, 4):
            if self.move_dir == self.directions[i]:
                dir_index = i
        if dir_index == 3:
            self.move_dir = self.directions[0]
        else:
            self.move_dir = self.directions[dir_index+1]

    def goGoal(self, goal):
        self.goal_reached = False
        action_request = NavigateToPose.Goal()
        action_request.pose = goal

        self.move_action.wait_for_server()
        self.get_logger().info('Sending goal request .....')
        send_goal_future = self.move_action.send_goal_async(action_request)

        try:
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
        
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
        else:
            self.get_logger().info('Goal accepted')

        get_result_future = self.goal_handle.get_result_async()

        self.get_logger().info("Waiting for 'Navigate to Pose' action to complete...")
        try:
            rclpy.spin_until_future_complete(self, get_result_future)
            self.get_logger().info('goal reached')
            time.sleep(5)
            self.goal_reached = True
            return
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

    def lidar_callback(self, msg):
        self.ahead = msg.ranges[0]
        self.left = msg.ranges[90]
        self.behind = msg.ranges[180]
        self.right = msg.ranges[270]
        self.laser_done = True
        time.sleep(1)

    def odom_callback(self, msg):
        # first set the initial pose for AMCL
        if not self.initPose:
            initialPose = PoseWithCovarianceStamped()
            initialPose.header = msg.header
            initialPose.header.frame_id = 'map'
            initialPose.pose = msg.pose
            self.setInitPose.publish(initialPose)
            self.initPose = True

        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        cOrient = [0,0,qz,qw]
        r,p,y = teul(cOrient)
        robOrient = y
        self.rob_dir = 0
        if (robOrient > -self.rotAngle) and (robOrient < self.rotAngle):
            self.rob_dir = self.directions[0]
        elif (robOrient > self.rotAngle) and (robOrient < 3*self.rotAngle):
            self.rob_dir = self.directions[1]
        elif (robOrient > 3*self.rotAngle) and (robOrient < 5*self.rotAngle):
            self.rob_dir = self.directions[2]
        elif (robOrient > 5*self.rotAngle) and (robOrient < 7*self.rotAngle):
            self.rob_dir = self.directions[3]
        elif (robOrient < -self.rotAngle) and (robOrient > -3*self.rotAngle):
            self.rob_dir = self.directions[3]
        elif (robOrient < -3*self.rotAngle) and (robOrient > -5*self.rotAngle):
            self.rob_dir = self.directions[2]
        elif (robOrient < -5*self.rotAngle) and (robOrient > -7*self.rotAngle):
            self.rob_dir = self.directions[1]

        self.odom_done = True
        time.sleep(1)
    
    def setPose(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.orientation.z = pose[2]
        msg.pose.orientation.w = pose[3]
        self.goal = msg
        
def main(args=None):
    rclpy.init(args=args)

    tb3_explore = TurtlebotExplore()

    rclpy.spin(tb3_explore)

if __name__ == '__main__':
    main()