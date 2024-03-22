#!/usr/bin/env python3
"""
    wall_follower_node.py

    A ROS node that commands a Turtlebot3 to follow a wall to its right
    according to the latest laser scan.

    Subscribed: /scan, /cmd
    Publishes: /cmd_vel

    Created: 2020/02/04
    Author: Brendan Halloran
    Updated for ROS2 by Jeff Moscrop
"""

import rclpy
import numpy as np 
import math
from . import transformations as trans
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from enum import Enum
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from .commands import RobotState

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# Constants
BASE_FRAME =        "base_link"         # Param from the SLAM module
MAX_SIDE_LIMIT =    0.50                # This furthest distance we 'see' the wall to the side
MIN_APPROACH_DIST = 0.30                # The closest we want to get to a wall from the front
MAX_APPROACH_DIST = 0.50                # The distance we want to start slowing to approach a front wall
ROBOT_RADIUS =      0.20                # The bounding circle around the robot
MAX_TRANS_SPEED =   0.25                # Forward movement
MAX_TURN_SPEED =    1.4                 # Rotation

class FollowSide(Enum):
    LEFT = 1
    RIGHT = 2

class wall_follower_node(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.stopped = False    # Assume robot is moving and needs to be stopped first
        self.explore = False
        self.follow_side = FollowSide.LEFT

        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)
        qos_policy = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.subscriber_laser_scan = self.create_subscription(LaserScan, '/scan', self.callback_laser_scan, qos_policy)
        self.subscriber_state = self.create_subscription(String, '/state', self.callback_state, 1)
        self.publisher_twist = self.create_publisher(Twist, '/cmd_vel', 1)

    # Send a final stop command before shutting off the node
    def shutdown(self):
        if not self.stopped:
            self.stopped = True
            self.explore = False
            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
            self.publisher_twist.publish(vel_msg)

    def get_transform(self, data):
        try:
            #tf_ready = self.tf_buffer.wait_for_transform_async(BASE_FRAME, data.header.frame_id, data.header.stamp)
            #rclpy.spin_until_future_complete(self, tf_ready)
            new_transform = self.tf_buffer.lookup_transform(BASE_FRAME, data.header.frame_id, data.header.stamp)
            position = new_transform.transform.translation
            quaternion = new_transform.transform.rotation
            return trans.pq_to_se3(position, quaternion)
        except Exception as e:
            print('Unable to get transformation! %r' % (e))   
            return []

    def callback_laser_scan(self, data): 
        # If we are no longer exloring don't process the scan
        if not self.explore:
            # If we aren't exploring, but we haven't told the robot to stop, do so now.
            # This is located here rather than in the other callback because the stop command
            # might be received halfway through processing a laser scan, then the laser scan
            # callback's move command will come after the stop and the robot will start moving
            # again.
            if not self.stopped:
                self.stopped = True
                vel_msg = Twist()
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = 0.0
                self.publisher_twist.publish(vel_msg)
            return
        # Otherwise, process the scan.

        # Find the transform between the laser scanner and the robots base frame
        # The transformations are all populated at startup
        transformation = []
        while transformation == []:
            transformation = self.get_transform(data)

        # We want to find how far in front of the robot the wall to the left extends
        # and how far away the nearest point in front of the robot is.
        x_side_max = -np.inf
        x_front_min = np.inf
        angle = data.angle_min
        
        # Transform scans into 3D points relative to robots base frame
        for scan in data.ranges: 
            # 3D homogeneous point [x; y; z; 1]
            if scan > 10:
                scan = 1000.0
            point = np.array([[math.cos(angle) * scan], [math.sin(angle) * scan], [0.0], [1.0]])
            point = np.matmul(transformation, point)

            # Update angle
            angle += data.angle_increment

            # Unwrap
            point_x = point[0][0]
            point_y = point[1][0]

            # Find if point is forward-most wall point on correct side 
            if (math.fabs(point_x) <= ROBOT_RADIUS) and (math.fabs(point_y) <= MAX_SIDE_LIMIT):
                if (self.follow_side == FollowSide.LEFT and point_y > 0) or (self.follow_side == FollowSide.RIGHT and point_y < 0):
                    # Point is between the front and back of robot and within range on correct side
                    if point_x > x_side_max:
                        x_side_max = point_x

            # Find if point is nearest wall point in front of the robot
            if point_x > 0 and point_x <= MAX_APPROACH_DIST and math.fabs(point_y) < ROBOT_RADIUS:
                # Point is between left and right sides of robot and within range from the front
                if point_x < x_front_min:
                    x_front_min = point_x

        # print "Detected walls {0} left, {1} front".format(x_side_max, x_front_min)

        turn = 0.0
        drive = 0.0
        if x_side_max == -np.inf:
            # No wall to side, so turn that way
            turn = 1.0
            drive = 0.0
        elif x_front_min <= MIN_APPROACH_DIST:
            # Wall to side and blocked at front, so turn opposite way
            turn = -1.0
            drive = 0.0
        else:
            # Wall to side but front is clear, so drive forward and start turning if 
            # an opening is starting to appear to side

            # If side wall doesn't extend past front of robot start turning and reduce speed
            turn_side = np.clip((ROBOT_RADIUS - x_side_max) / (2 * ROBOT_RADIUS), 0.0, 1.0)
            drive_side = np.clip((ROBOT_RADIUS + x_side_max) / (2 * ROBOT_RADIUS), 0.0, 1.0)

            # If front wall is between MAX_APPROACH_DIST and MIN_APPROACH_DIST then slow down and reduce turn
            turn_front = np.clip((MAX_APPROACH_DIST - x_front_min) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST), 0.0, 1.0)
            drive_front = np.clip((x_front_min - MIN_APPROACH_DIST) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST), 0.0, 1.0)

            # Combine two turn and drive decisions
            turn = turn_side - turn_front
            drive = drive_side * drive_front

        # Flip turn if we are following to the right
        if self.follow_side == FollowSide.RIGHT:
            turn *= -1.0

        # Convert to a Twist message for 'cmd_vel/'
        vel_msg = Twist()
        vel_msg.linear.x = drive * MAX_TRANS_SPEED
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = turn * MAX_TURN_SPEED
        # print "Publishing velocities {0} m/s, {1} r/s".format(vel_msg.linear.x, vel_msg.angular.z)
        self.publisher_twist.publish(vel_msg)
        self.stopped = False

    def callback_state(self, data):
        state = RobotState(data.data)

        if state is RobotState.EXPLORING and not self.explore:
            self.explore = True
            print('Starting Exploring')
        elif (state is RobotState.RETURNING or state is RobotState.PAUSED) and self.explore:
            print('Stopping Exploring')
            self.explore = False
            self.stopped = False
            # setting explore to false and stopped to false will cause the next laser scan
            # callback to send a stop command then set stopped to true.
            
def main(args=None):
    
    rclpy.init(args=args)
    print("Starting ROS Wall Following Module")
    wf = wall_follower_node()
    rclpy.spin(wf)

if __name__ == '__main__':
    main()